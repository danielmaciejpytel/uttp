"""Export local-only native recoil curves for offline authored-pose review.

No game or source animation is modified. Raw GLB curves and explicitly authored
endpoint fades are separate. TSV converts the exported coordinate basis to CP
(X,Y,Z -> GLB X,Z,-Y); it is not native graph/constraint evaluation.

Two modes share one numeric core:
  * default (Nue): ten add_recoil_single_shot_* clips of pwa_w_handgun__tsunami_nue.anims.glb, as before.
  * --profiles: one runtime profile per weapon family (the pair add_recoil_single_shot_shoulder / _ironsight of that
    family's pwa_w_*.anims.glb). A profile is written only when BOTH clips pass the gates (type, first-frame reference,
    runtime bounds of src/mm/AuthoredRecoil.hpp); every other family is reported with its reason and falls back to the
    authored aim pose without recoil (fallback A of the driver).
"""
from __future__ import annotations
import argparse
import hashlib
import json
import math
import sys
from pathlib import Path
import numpy as np

_HERE = Path(__file__).resolve().parent
sys.path[:0] = [str(_HERE), str(_HERE.parent / 'recon')]  # works from the main tree (Source\tools) and from a worktree
from compare_nue_magazine_sources import Animation, rotation, slerp  # noqa: E402


def _project_root() -> Path:
    here = Path(__file__).resolve()
    for parent in here.parents:
        if (parent / 'Extracted').is_dir() and (parent / 'Research').is_dir() and (parent / 'Data').is_dir():
            return parent
    return here.parents[3]


ROOT = _project_root()

# Runtime bounds of src/mm/AuthoredRecoil.hpp (Clip::Load rejects any key beyond them).
MAX_TRANSLATION = .25
MAX_ROTATION = .70
# Reading of the stored additive keys. The Nue export (run()) subtracts the first key of each clip. For the weapon profiles the
# keys are read against the BIND pose of WeaponRight instead (delta = bind^-1 * key, no first-key subtraction). The two readings
# agree where a clip starts at the bind pose (Nue: first key within 0.9 mm / 0.034 rad of it), and the bind reading is the one the
# data supports: in every start/loop/end set the end of `start`, the beginning of `loop` and the beginning of `end` coincide in
# bind-relative space (e.g. darra_pulsar ironsight start->loop: (-0.076,-0.019,0.009) m on both sides), while a first-key
# reading would restart every loop cycle from zero. Single-shot clips of AdditiveWithoutFirstFrame type may therefore begin
# at a kicked pose; the sampler file starts at identity and reaches that pose within its first sample interval.
FIRST_KEY_REPORT_M = .003
FIRST_KEY_REPORT_RAD = .05

# Runtime profile name -> source animset (GLB written by WolvenKit.CLI export, see the command log of the session).
# The name is the lower-case family tag of the weapon record (TweakDB tags), the key the redscript binding resolves.
PROFILE_SOURCES = {
    'liberty': 'pwa_w_handgun__constitutional_liberty.anims.glb',
    'unity': 'pwa_w_handgun__constitutional_unity.anims.glb',
    'yukimura': 'pwa_w_handgun__arasaka_yukimura.anims.glb',
    'chao': 'pwa_w_handgun__kangtao_chao.anims.glb',
    'lexington': 'pwa_w_handgun__militech_lexington.anims.glb',
    'slaughtomatic': 'pwa_w_handgun__budget_slaughtomatic.anims.glb',
    # nova / overture / burya (pwa_w_revolver__darra_nova, malorian_overture, techtronika_burya) are retired: their single_shot clips are a
    # revolver hammer-cock (7-10 cm sideways, roll about the barrel) that the hand IK follows by 3-30% only; see RecoilProfiles.hpp.
    'quasar': 'pwa_w_revolver__darra_quasar.anims.glb',
    'ajax': 'pwa_w_rifle_assault__militech_ajax.anims.glb',
    'copperhead': 'pwa_w_rifle_assault__nokota_copperhead.anims.glb',
    'sidewinder': 'pwa_w_rifle_assault__nokota_sidewinder.anims.glb',
    'masamune': 'pwa_w_rifle_assault__arasaka_masamune.anims.glb',
    'shingen': 'pwa_w_smg__arasaka_shingen.anims.glb',
    'pulsar': 'pwa_w_smg__darra_pulsar.anims.glb',
    'saratoga': 'pwa_w_smg__militech_saratoga.anims.glb',
    'senkoh': 'pwa_w_smg__senkoh_lx.anims.glb',
    # 2.5 additions (recoil-missing-20261109): own single-shot clips; overrides in PROFILE_CLIP_OVERRIDES / PROFILE_BUDGET_FACTOR
    'kenshin': 'pwa_w_handgun__arasaka_kenshin.anims.glb',
    'omaha': 'pwa_w_handgun__militech_omaha.anims.glb',
    'ticon': 'pwa_w_handgun__militech_ticon.anims.glb',
    'defender': 'pwa_w_lmg__constitutional_defender.anims.glb',
    'dian': 'pwa_w_special_kangtao_dian.anims.glb',
    'warden': 'pwa_w_smg__krausser_warden.anims.glb',
    'borg4a': 'pwa_w_smg__midnight_borg.anims.glb',
    'hercules': 'pwa_w_special__militech_hercules.anims.glb',
}
# Nue is not a --profiles family (its curves are the first-key export of run()), but it is the 16th family of the male/female comparison.
COMPARE_SOURCES = dict(PROFILE_SOURCES, nue='pwa_w_handgun__tsunami_nue.anims.glb')
PROFILE_CLIPS = (('shoulder', 'add_recoil_single_shot_shoulder'), ('ironsight', 'add_recoil_single_shot_ironsight'))

# Clip of a family whose animset has no add_recoil_single_shot_ironsight (tech weapons aim with the charged shot) or no ADS
# clip at all; per-family budget factor on the final peak (hand-IK reach, recoil-missing-20261109): budget = factor x peak.
PROFILE_CLIP_OVERRIDES = {
    'kenshin': {'ironsight': 'add_recoil_charged_ironsight'},
    'omaha': {'ironsight': 'add_recoil_charged_ironsight'},
    'ticon': {'ironsight': 'add_recoil_charged_ironsight'},
}
PROFILE_BUDGET_FACTOR = {
    ('kenshin', 'ironsight'): 0.5,
    ('omaha', 'shoulder'): 0.5,
    ('omaha', 'ironsight'): 0.5,
    ('defender', 'shoulder'): 0.85,
    ('defender', 'ironsight'): 0.85,
}
# male pma_* files whose name differs from the pwa_* source by more than the prefix
PMA_NAME_FIXES = {'pwa_w_rifle_precision_midnight_sor22.anims.glb': 'pwa_w_rifle_precision__midnight_sor22.anims.glb'}


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()

def quaternion(matrix):
    r = matrix[:3, :3]
    w = math.sqrt(max(0.0, 1 + np.trace(r))) / 2
    if w < .1:
        raise ValueError('Native recoil exceeded supported small-angle quaternion extraction')
    q = np.array([r[2,1]-r[1,2], r[0,2]-r[2,0], r[1,0]-r[0,1], 4*w*w]) / (4*w)
    return q / np.linalg.norm(q)

def metric(matrix):
    return dict(translationM=float(np.linalg.norm(matrix[:3,3])),
                turnDeg=float(math.degrees(math.acos(np.clip((np.trace(matrix[:3,:3])-1)/2, -1, 1)))))

def run(source, output, report, samples=97, fade=.15):
    for path in (source, output, report):
        if not path.resolve().is_relative_to(ROOT):
            raise ValueError('All artifacts must remain in project')
    if samples < 3 or fade <= 0:
        raise ValueError('Invalid samples/fade')
    animation = Animation(source)
    names = [n for n in animation.clips if 'add_recoil_single_shot_' in n]
    if len(names) != 10:
        raise ValueError('Expected ten native Nue recoil clips')
    output.mkdir(parents=True, exist_ok=True)
    cp_to_glb = np.eye(4)
    cp_to_glb[:3,:3] = [[1,0,0],[0,0,1],[0,-1,0]]
    clips, manifest, tsv = [], [], []
    for name in names:
        duration = animation.clips[name][0]
        if fade >= duration:
            raise ValueError('Fade must be shorter than clip')
        metadata = next(a for a in animation.doc['animations'] if a['name']==name).get('extras',{})
        if metadata.get('animationType') != 'AdditiveWithoutFirstFrame':
            raise ValueError('Unexpected native animation type')
        start, rest = animation.model(name,0), animation.model()
        curves = {bone: [] for bone in ('WeaponRight','arm_right_ik_offset')}
        raw_rows = []
        for time in np.linspace(0,duration,samples):
            model = animation.model(name,float(time))
            phase = float(np.clip((time-(duration-fade))/fade,0,1))
            weight = 1-phase**3*(phase*(phase*6-15)+10)
            row = {'seconds':float(time)}
            for bone in curves:
                delta = np.linalg.inv(start[bone]) @ model[bone]
                q, t = quaternion(delta), delta[:3,3]
                faded_q = np.array(slerp([0,0,0,1],q,weight))
                faded_t = weight*t
                if time == 0 or time == duration:
                    faded_q, faded_t = np.array([0.,0.,0.,1.]),np.zeros(3)
                candidate=np.eye(4);candidate[:3,:3]=rotation(faded_q);candidate[:3,3]=faded_t
                cp = cp_to_glb.T @ candidate @ cp_to_glb
                value = dict(seconds=float(time),rawTranslation=t.tolist(),rawRotationXYZW=q.tolist(),
                             zeroEndedTranslation=faded_t.tolist(),zeroEndedRotationXYZW=faded_q.tolist(),
                             zeroEndedCpTranslation=cp[:3,3].tolist(),zeroEndedCpRotationXYZW=quaternion(cp).tolist())
                curves[bone].append(value)
                if bone=='WeaponRight':
                    tsv.append(name+' '+format(float(time),'.10g')+' '+
                               ' '.join(format(float(x),'.10g') for x in (*quaternion(cp),*cp[:3,3])))
                row[bone]=metric(delta)
            for bone in ('Root','Hips','Spine3'):
                row[bone]=metric(np.linalg.inv(start[bone])@model[bone])
            raw_rows.append(row)
        for rows in curves.values():
            for endpoint in (rows[0], rows[-1]):
                assert endpoint['zeroEndedTranslation']==[0.,0.,0.]
                assert endpoint['zeroEndedRotationXYZW']==[0.,0.,0.,1.]
        initial={bone:{'modelMatrixAtZero':start[bone].tolist(),
                       'bindRelativeAtZero':(np.linalg.inv(rest[bone])@start[bone]).tolist()} for bone in curves}
        document=dict(source=str(source.relative_to(ROOT)),sourceSha256=digest(source),clip=name,duration=duration,
                      nativeAnimationType=metadata['animationType'],initialFrames=initial,curves=curves,
                      endpointTreatment=f't0 relative; last {fade}s faded by C2 smootherstep; raw retained',
                      basis='GLB original and CP candidate coordinate conversion explicitly separated',
                      runtimeVerified=False,constraintVerified=False)
        destination=output/(name+'.json')
        destination.write_text(json.dumps(document,indent=2)+'\n',encoding='utf-8')
        manifest.append(dict(path=str(destination.relative_to(ROOT)),sha256=digest(destination),
                             samplesPerBone=samples,startEndIdentityVerified=True))
        # Fixed-capacity game-thread sampler input. Binding to a verified weapon
        # remains an explicit caller decision; filenames do not imply runtime use.
        simple=output/(name+'.utpprecoil')
        simple_rows=[f'UTPP_RECOIL 1',f'{samples} {duration:.10g}']
        for value in curves['WeaponRight']:
            simple_rows.append(' '.join(format(float(x),'.10g') for x in
                (value['seconds'],*value['zeroEndedCpRotationXYZW'],*value['zeroEndedCpTranslation'])))
        simple.write_text('\n'.join(simple_rows)+'\n',encoding='utf-8')
        manifest[-1]['samplerFile']=dict(path=str(simple.relative_to(ROOT)),sha256=digest(simple))
        clips.append(dict(name=name,duration=duration,animationType=metadata['animationType'],samples=samples,
                          maxima={bone:{key:max(row[bone][key] for row in raw_rows)
                                        for key in ('translationM','turnDeg')} for bone in raw_rows[0] if bone!='seconds'},
                          rows=raw_rows))
    table=output/'weapon-curves-cp.tsv'
    table.write_text('\n'.join(tsv)+'\n',encoding='utf-8')
    semantics=ROOT/'Tools/ext/wolvenkit-import-source/AnimationTools.cs'
    result=dict(source=str(source.relative_to(ROOT)),sha256=digest(source),runtimeVerified=False,
                movingCaptureVerified=False,clips=clips,curveManifest=manifest,
                tsv=dict(path=str(table.relative_to(ROOT)),sha256=digest(table)),
                additiveSemanticsEvidence=dict(source=str(semantics.relative_to(ROOT)),sha256=digest(semantics),
                  lines='199,244-265',confirmed='Non-Normal export T=bindT+deltaT,R=bindR*deltaR. No first-key subtraction; native WithoutFirstFrame evaluator semantics remain unproven.'),
                limitations=['Nue only. No native graph selection, additive weighting or active IK constraint proof.',
                  'WeaponRight FK and active hand controller differ; neither is claimed as final constrained native trajectory.',
                  'Endpoint fade is an authored choice, not a native animation change.',
                  'Sampled extrema, not continuous extrema. Synthetic rigid movement is not actual walking/running capture.'])
    report.parent.mkdir(parents=True,exist_ok=True)
    report.write_text(json.dumps(result,indent=2)+'\n',encoding='utf-8')
    print(f'Exported {len(clips)} clips, {len(tsv)} weapon samples, endpoints PASS: {table}')


# ---------------------------------------------------------------------------------------------------------------------
# Runtime profiles (one pair of sampler files per weapon family)
# ---------------------------------------------------------------------------------------------------------------------
def _turn(matrix):
    return float(math.acos(np.clip((np.trace(matrix[:3, :3]) - 1) / 2, -1, 1)))


def weapon_curve(animation, clip, samples, fade, reference='bind', limit_amplitude=False, start_ramp=0.0, budget=None):
    """WeaponRight delta of one clip, faded to identity over the last `fade` seconds, in CP.

    reference 'bind': delta against the bind pose (profiles). reference 'first': against the clip's first key, the reading of
    run(); with it the formulas are those of run() for the WeaponRight bone, so a Nue clip yields the same rows through both
    entry points. Returns (rows, report) with rows = [(seconds, qx, qy, qz, qw, tx, ty, tz)] and report = measurements and gates.

    Shaping of the profiles (all off by default, so the Nue path above is untouched):
      start_ramp  seconds: the curve is multiplied by a smootherstep over its first `start_ramp` seconds, so a clip whose first key is
                  already a kick (Masamune: 15.6 cm after 7 ms) rises over a few frames instead of in one;
      budget      (metres, radians): peak per shot. The whole curve is scaled by one factor so its peak translation and rotation stay
                  within the budget (the factor is recorded as amplitudeScale); nothing is scaled up.
    """
    cp_to_glb = np.eye(4)
    cp_to_glb[:3, :3] = [[1, 0, 0], [0, 0, 1], [0, -1, 0]]
    duration = animation.clips[clip][0]
    metadata = next(a for a in animation.doc['animations'] if a['name'] == clip).get('extras', {})
    kind = metadata.get('animationType')
    rest = animation.model()['WeaponRight']
    first = np.linalg.inv(rest) @ animation.model(clip, 0)['WeaponRight']
    start = rest if reference == 'bind' else animation.model(clip, 0)['WeaponRight']
    rows, peak_t, peak_r = [], 0.0, 0.0
    fade = min(fade, duration * .5)  # a 0.07-0.1 s automatic-weapon clip cannot hold a 0.15 s fade; Nue (0.8 s) is unaffected
    for time in np.linspace(0, duration, samples):
        model = animation.model(clip, float(time))['WeaponRight']
        phase = float(np.clip((time - (duration - fade)) / fade, 0, 1))
        weight = 1 - phase ** 3 * (phase * (phase * 6 - 15) + 10)
        delta = np.linalg.inv(start) @ model
        q, t = quaternion(delta), delta[:3, 3]
        faded_q = np.array(slerp([0, 0, 0, 1], q, weight))
        faded_t = weight * t
        if time == 0 or time == duration:
            faded_q, faded_t = np.array([0., 0., 0., 1.]), np.zeros(3)
        candidate = np.eye(4)
        candidate[:3, :3] = rotation(faded_q)
        candidate[:3, 3] = faded_t
        cp = cp_to_glb.T @ candidate @ cp_to_glb
        rows.append((float(time), *quaternion(cp), *cp[:3, 3]))
        peak_t = max(peak_t, float(np.linalg.norm(faded_t)))
        peak_r = max(peak_r, _turn(candidate))
    reasons = []
    scale = 1.0
    raw_peak_t, raw_peak_r = peak_t, peak_r
    if start_ramp > 0:
        rows = weigh_rows(rows, [ramp_weight(row[0], start_ramp) for row in rows])
        peak_t, peak_r = row_peaks(rows)
    if budget is not None:
        scale = min(1.0, budget[0] / peak_t if peak_t > 0 else 1.0, budget[1] / peak_r if peak_r > 0 else 1.0)
        if scale < 1.0:
            rows = scale_rows(rows, scale)
            peak_t, peak_r = peak_t * scale, peak_r * scale
    if kind not in ('Additive', 'AdditiveWithoutFirstFrame'):
        reasons.append(f'animation type {kind}')
    if peak_t > MAX_TRANSLATION or peak_r > MAX_ROTATION:
        if limit_amplitude and budget is None:
            # One uniform factor on rotation angle and translation keeps the shape of the kick; it is recorded in the report.
            scale = min(MAX_TRANSLATION / peak_t if peak_t > 0 else 1.0, MAX_ROTATION / peak_r if peak_r > 0 else 1.0) * .995
            rows = scale_rows(rows, scale)
            peak_t, peak_r = peak_t * scale, peak_r * scale
        else:
            if peak_t > MAX_TRANSLATION:
                reasons.append('peak translation %.3f m exceeds the runtime bound %.2f m' % (peak_t, MAX_TRANSLATION))
            if peak_r > MAX_ROTATION:
                reasons.append('peak rotation %.3f rad exceeds the runtime bound %.2f rad' % (peak_r, MAX_ROTATION))
    first_t = float(np.linalg.norm(first[:3, 3]))
    return rows, dict(clip=clip, animationType=kind, duration=duration, fadeSeconds=fade, reference=reference,
                      firstKeyOffsetM=first_t, firstKeyOffsetRad=_turn(first),
                      impulseStart=bool(first_t > FIRST_KEY_REPORT_M or _turn(first) > FIRST_KEY_REPORT_RAD),
                      rawPeakTranslationM=raw_peak_t, rawPeakRotationRad=raw_peak_r, amplitudeScale=scale, startRampSeconds=start_ramp,
                      budgetM=None if budget is None else budget[0], budgetRad=None if budget is None else budget[1],
                      peakTranslationM=peak_t, peakRotationRad=peak_r, reasons=reasons)


def ramp_weight(seconds, ramp):
    x = min(1.0, max(0.0, seconds / ramp))
    return x ** 3 * (x * (x * 6 - 15) + 10)


def row_peaks(rows):
    peak_t = max(math.sqrt(r[5] ** 2 + r[6] ** 2 + r[7] ** 2) for r in rows)
    peak_r = max(2 * math.atan2(math.sqrt(r[1] ** 2 + r[2] ** 2 + r[3] ** 2), abs(r[4])) for r in rows)
    return peak_t, peak_r


def weigh_rows(rows, weights):
    """Scale the rotation angle and the translation of each sample by its own weight (sample times unchanged)."""
    return [scale_rows([row], weight)[0] for row, weight in zip(rows, weights)]


def scale_rows(rows, scale):
    """Uniformly scale the rotation angle and the translation of every sample (sample times unchanged)."""
    out = []
    for row in rows:
        x, y, z, w = row[1:5]
        norm = math.sqrt(x * x + y * y + z * z)
        if norm > 1e-12:
            half = (1.0 if w >= 0 else -1.0) * math.atan2(norm, abs(w))  # half angle, sign carries the hemisphere
            axis = (x / norm, y / norm, z / norm)
            x, y, z, w = (axis[0] * math.sin(half * scale), axis[1] * math.sin(half * scale),
                          axis[2] * math.sin(half * scale), math.cos(half * scale))
        out.append((row[0], x, y, z, w, row[5] * scale, row[6] * scale, row[7] * scale))
    return out


def sampler_text(rows, duration, samples):
    lines = ['UTPP_RECOIL 1', f'{samples} {duration:.10g}']
    for row in rows:
        lines.append(' '.join(format(float(x), '.10g') for x in row))
    return '\n'.join(lines) + '\n'


def find_sources(glb_roots, prefix='pwa'):
    """name (pwa_ form, as in PROFILE_SOURCES) -> GLB path; with prefix 'pma' the male file pma_<rest> stands in for pwa_<rest>."""
    found = {}
    for root in glb_roots:
        for path in root.rglob('*.anims.glb'):
            name = path.name
            if prefix != 'pwa':
                if not name.startswith(prefix + '_'):
                    continue
                name = 'pwa_' + name[len(prefix) + 1:]
                name = PMA_NAME_FIXES.get(name, name)
            found.setdefault(name, path)
    return found


def run_profiles(names, glb_roots, output, report, samples=97, fade=.15, write=True, reference='bind', limit_amplitude=False,
                 start_ramp=0.0, budget=None, prefix='pwa'):
    for path in (output, report):
        if not path.resolve().is_relative_to(ROOT):
            raise ValueError('All artifacts must remain in project')
    if samples < 3 or fade <= 0:
        raise ValueError('Invalid samples/fade')
    unknown = [n for n in names if n not in PROFILE_SOURCES]
    if unknown:
        raise ValueError('Unknown profile ' + ','.join(unknown))
    found = find_sources(glb_roots, prefix)
    result = []
    for name in names:
        glb = found.get(PROFILE_SOURCES[name])
        entry = dict(profile=name, source=None, accepted=False, files=[], clips=[], reasons=[])
        result.append(entry)
        if glb is None:
            entry['reasons'].append('source GLB ' + PROFILE_SOURCES[name] + ' not found')
            continue
        entry.update(source=str(glb.relative_to(ROOT)), sourceSha256=digest(glb))
        animation = Animation(glb)
        curves = {}
        for kind, clip in PROFILE_CLIPS:
            clip = PROFILE_CLIP_OVERRIDES.get(name, {}).get(kind, clip)
            if clip not in animation.clips:
                entry['reasons'].append(f'clip {clip} missing')
                continue
            rows, clip_report = weapon_curve(animation, clip, samples, fade, reference, limit_amplitude, start_ramp, budget)
            factor = PROFILE_BUDGET_FACTOR.get((name, kind))
            if factor:
                peak_t, peak_r = row_peaks(rows)
                rows, clip_report = weapon_curve(animation, clip, samples, fade, reference, limit_amplitude, start_ramp,
                                                 (factor * peak_t, factor * peak_r))
                clip_report['familyBudgetFactor'] = factor
            curves[kind] = (rows, clip_report)
            entry['clips'].append(clip_report)
            entry['reasons'] += [f'{clip}: {why}' for why in clip_report['reasons']]
        if len(curves) != len(PROFILE_CLIPS) or entry['reasons']:
            continue
        entry['accepted'] = True
        for kind, (rows, clip_report) in curves.items():
            destination = output / f'{name}_single_shot_{kind}.utpprecoil'
            if write:
                output.mkdir(parents=True, exist_ok=True)
                destination.write_text(sampler_text(rows, clip_report['duration'], samples), encoding='utf-8')
                entry['files'].append(dict(path=str(destination.relative_to(ROOT)), sha256=digest(destination)))
    document = dict(generator='tools/analysis/export_nue_native_recoil_curves.py --profiles', samples=samples, fadeSeconds=fade,
                    bounds=dict(maxTranslationM=MAX_TRANSLATION, maxRotationRad=MAX_ROTATION, reference=reference,
                                limitAmplitude=limit_amplitude,
                                startRampSeconds=start_ramp, budgetM=None if budget is None else budget[0],
                                budgetRad=None if budget is None else budget[1]),
                    runtimeVerified=False, profiles=result)
    if write:
        report.parent.mkdir(parents=True, exist_ok=True)
        report.write_text(json.dumps(document, indent=2) + '\n', encoding='utf-8')
    for entry in result:
        print(('ACCEPT ' if entry['accepted'] else 'REJECT ') + entry['profile'].ljust(14),
              '; '.join(entry['reasons']) if entry['reasons'] else ' '.join(
                  f"{c['clip'].split('single_shot_')[-1]}:{c['duration']:.3f}s/{c['peakTranslationM']*100:.1f}cm/"
                  f"{c['peakRotationRad']:.3f}rad{'*' if c['impulseStart'] else ''}"
                  f"{' x%.3f' % c['amplitudeScale'] if c['amplitudeScale'] < 1 else ''}" for c in entry['clips']))
    return result


def _curve_xyz(rows):
    return np.array([r[5:8] for r in rows])


def _curve_angle(rows):
    return np.array([2 * math.atan2(math.sqrt(r[1] ** 2 + r[2] ** 2 + r[3] ** 2), abs(r[4])) for r in rows])


def compare_profiles(names, female_roots, male_roots, samples=97, fade=.15, reference='bind', report=None, start_ramp=0.0, budget=None):
    """Read-only comparison of the weapon-local WeaponRight recoil curves of pwa_* (female) and pma_* (male) per family and clip.

    Per clip: duration, peak translation / rotation (CP basis, as the profiles), time of the translation peak, direction of the
    peak translation (angle between the female and the male peak vector), and the worst / RMS difference of the two curves after
    resampling the male one on the female time base. Writes nothing but the optional JSON report; the shipped profiles are untouched.
    """
    female = find_sources(female_roots, 'pwa')
    male = find_sources(male_roots, 'pma')
    rows_out = []
    for name in names:
        fg, mg = female.get(COMPARE_SOURCES[name]), male.get(COMPARE_SOURCES[name])
        if fg is None or mg is None:
            rows_out.append(dict(profile=name, error='missing ' + ('female' if fg is None else 'male') + ' GLB'))
            continue
        fa, ma = Animation(fg), Animation(mg)
        for kind, clip in PROFILE_CLIPS:
            clip = PROFILE_CLIP_OVERRIDES.get(name, {}).get(kind, clip)
            if clip not in fa.clips or clip not in ma.clips:
                rows_out.append(dict(profile=name, clip=kind, error='clip missing'))
                continue
            fr, frep = weapon_curve(fa, clip, samples, fade, reference, False, start_ramp, budget)
            mr, mrep = weapon_curve(ma, clip, samples, fade, reference, False, start_ramp, budget)
            ft, mt = _curve_xyz(fr), _curve_xyz(mr)
            fn, mn = np.linalg.norm(ft, axis=1), np.linalg.norm(mt, axis=1)
            fi, mi = int(np.argmax(fn)), int(np.argmax(mn))
            cosang = float(np.dot(ft[fi], mt[mi]) / (fn[fi] * mn[mi])) if fn[fi] > 1e-9 and mn[mi] > 1e-9 else 1.0
            # male curve on the female time base (times are linspace(0, duration) of each clip)
            ftimes = np.array([r[0] for r in fr])
            mtimes = np.array([r[0] for r in mr])
            m_on_f = np.stack([np.interp(ftimes, mtimes, mt[:, k]) for k in range(3)], axis=1)
            diff = np.linalg.norm(m_on_f - ft, axis=1)
            fang, mang = _curve_angle(fr), _curve_angle(mr)
            m_ang_on_f = np.interp(ftimes, mtimes, mang)
            rows_out.append(dict(
                profile=name, clip=kind, durationF=frep['duration'], durationM=mrep['duration'],
                peakTF=frep['peakTranslationM'], peakTM=mrep['peakTranslationM'],
                peakRF=frep['peakRotationRad'], peakRM=mrep['peakRotationRad'],
                peakTimeF=float(ftimes[fi]), peakTimeM=float(mtimes[mi]),
                peakDirAngleDeg=float(math.degrees(math.acos(max(-1.0, min(1.0, cosang))))),
                maxDiffT=float(diff.max()), rmsDiffT=float(math.sqrt((diff ** 2).mean())),
                maxDiffR=float(np.abs(m_ang_on_f - fang).max()),
                impulseF=frep['impulseStart'], impulseM=mrep['impulseStart'],
                reasonsF=frep['reasons'], reasonsM=mrep['reasons']))
    print('profile        clip      dur F/M (s)   peakT F/M (mm)   dT%   peakR F/M (rad)   dR%   tPeak F/M (s)  dirAng(deg)  maxDiffT(mm)  rmsDiffT(mm)  maxDiffR(rad)')
    for r in rows_out:
        if 'error' in r:
            print(r['profile'].ljust(14), r.get('clip', ''), r['error'])
            continue
        dt = 100 * (r['peakTM'] - r['peakTF']) / r['peakTF'] if r['peakTF'] > 1e-9 else 0.0
        dr = 100 * (r['peakRM'] - r['peakRF']) / r['peakRF'] if r['peakRF'] > 1e-9 else 0.0
        print(f"{r['profile']:<14} {r['clip']:<9} {r['durationF']:.3f}/{r['durationM']:.3f}  {r['peakTF']*1000:7.2f}/{r['peakTM']*1000:7.2f}  {dt:+6.1f}  "
              f"{r['peakRF']:.4f}/{r['peakRM']:.4f}  {dr:+6.1f}  {r['peakTimeF']:.3f}/{r['peakTimeM']:.3f}  {r['peakDirAngleDeg']:8.2f}  "
              f"{r['maxDiffT']*1000:11.3f}  {r['rmsDiffT']*1000:11.3f}  {r['maxDiffR']:.5f}")
    if report is not None:
        if not report.resolve().is_relative_to(ROOT):
            raise ValueError('All artifacts must remain in project')
        report.parent.mkdir(parents=True, exist_ok=True)
        report.write_text(json.dumps(dict(generator='export_nue_native_recoil_curves.py --compare-male', samples=samples, fadeSeconds=fade,
                                          reference=reference, rows=rows_out), indent=2) + '\n', encoding='utf-8')
    return rows_out


def verify_nue(source, existing, samples=97, fade=.15):
    """Control: the first-key reading of weapon_curve() reproduces the shipped Nue sampler files byte for byte,
    and reports how far the bind reading used by the profiles deviates from them (max translation / rotation of the delta)."""
    animation = Animation(source)
    outcome = {}
    for kind, clip in PROFILE_CLIPS:
        rows_first, _ = weapon_curve(animation, clip, samples, fade, 'first')
        rows_bind, report_bind = weapon_curve(animation, clip, samples, fade, 'bind')
        duration = animation.clips[clip][0]
        text = sampler_text(rows_first, duration, samples).replace('\n', '\r\n').encode('utf-8')  # write_text on Windows
        shipped = (existing / f'add_recoil_single_shot_{kind}.utpprecoil').read_bytes()
        worst_t = worst_r = 0.0
        for a, b in zip(rows_first, rows_bind):
            worst_t = max(worst_t, float(np.linalg.norm(np.array(a[5:8]) - np.array(b[5:8]))))
            qa, qb = np.array(a[1:5]), np.array(b[1:5])
            worst_r = max(worst_r, 2 * math.acos(min(1.0, abs(float(qa @ qb)))))
        outcome[kind] = dict(byteIdentical=text == shipped, sha256=hashlib.sha256(text).hexdigest(),
                             maxTranslationDifferenceBindVsFirstKeyM=worst_t, maxRotationDifferenceBindVsFirstKeyRad=worst_r,
                             bindReport=report_bind)
        print(f'Nue {kind}: first-key reading byte-identical to the shipped file: {text == shipped}; '
              f'bind reading differs by at most {worst_t * 1000:.2f} mm / {worst_r:.4f} rad')
    return outcome


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source',type=Path,default=ROOT/'Extracted/game/glb/weapon-grip-vanilla/pwa_w_handgun__tsunami_nue.anims.glb')
    parser.add_argument('--output',type=Path,default=ROOT/'Animations/Blender/Pistol1H/offline-polish-20261017/recoil')
    parser.add_argument('--report',type=Path,default=ROOT/'Research/inventory/nue-native-recoil-assets-20261017.json')
    parser.add_argument('--samples',type=int,default=97)
    parser.add_argument('--fade',type=float,default=.15)
    parser.add_argument('--profiles',help='comma separated profile names, or "all": write per-family sampler files instead of the Nue export')
    parser.add_argument('--glb-root',type=Path,action='append',help='directory searched for the *.anims.glb of --profiles (repeatable)')
    parser.add_argument('--source-dir',type=Path,action='append',help='alias of --glb-root; with --source-prefix pma the male pma_* GLB of this directory stand in for each pwa_* source (read-only; default: unchanged)')
    parser.add_argument('--source-prefix',choices=('pwa','pma'),default='pwa',help='file prefix of the GLB found in --source-dir/--glb-root (pma = male exports); default pwa')
    parser.add_argument('--compare-male',type=Path,metavar='MALE_DIR',help='read-only comparison of the weapon-local pwa_* curves (--glb-root, default weapon-grip-vanilla + weapon-recoil-20261021) with the pma_* GLB of MALE_DIR; --profiles selects families (default all), --profiles-report is the optional JSON path')
    parser.add_argument('--profiles-out',type=Path,help='directory of the profile sampler files (default: Data/recoil)')
    parser.add_argument('--profiles-report',type=Path,help='JSON report of --profiles (default: Data/recoil/profile-export.json)')
    parser.add_argument('--evaluate-only',action='store_true',help='with --profiles: apply the gates, write nothing')
    parser.add_argument('--reference',choices=('bind','first'),default='bind',help='with --profiles: delta against the bind pose (default) or the first key')
    parser.add_argument('--limit-amplitude',action='store_true',help='with --profiles: scale a clip that exceeds the runtime bounds uniformly down to them (recorded as amplitudeScale) instead of rejecting the profile')
    parser.add_argument('--budget-translation',type=float,help='with --profiles: peak translation per shot in metres; a larger curve is scaled down uniformly')
    parser.add_argument('--budget-rotation',type=float,help='with --profiles: peak rotation per shot in radians (given together with --budget-translation)')
    parser.add_argument('--start-ramp',type=float,default=0.0,help='with --profiles: seconds of the smootherstep that opens the curve (0 = none)')
    parser.add_argument('--verify-nue',type=Path,metavar='SHIPPED_DIR',help='compare the first-key reading with the shipped Nue sampler files of SHIPPED_DIR (read-only)')
    options=parser.parse_args()
    if options.verify_nue:
        verify_nue(options.source.resolve(),options.verify_nue.resolve(),options.samples,options.fade)
    elif options.compare_male:
        wanted=list(COMPARE_SOURCES) if (options.profiles or 'all')=='all' else options.profiles.split(',')
        female_roots=[p.resolve() for p in (options.glb_root or [ROOT/'Extracted/game/glb/weapon-grip-vanilla',ROOT/'Extracted/game/glb/weapon-recoil-20261021'])]
        compare_profiles(wanted,female_roots,[options.compare_male.resolve()],options.samples,options.fade,options.reference,
                         options.profiles_report.resolve() if options.profiles_report else None,options.start_ramp,
                         None if options.budget_translation is None else (options.budget_translation,options.budget_rotation))
    elif options.profiles:
        wanted=list(PROFILE_SOURCES) if options.profiles=='all' else options.profiles.split(',')
        roots=[p.resolve() for p in ((options.source_dir or []) + (options.glb_root or []) or [ROOT/'Extracted/game/glb/weapon-grip-vanilla'])]
        run_profiles(wanted,roots,(options.profiles_out or ROOT/'Data/recoil').resolve(),
                     (options.profiles_report or ROOT/'Data/recoil/profile-export.json').resolve(),options.samples,options.fade,
                     write=not options.evaluate_only,reference=options.reference,limit_amplitude=options.limit_amplitude,start_ramp=options.start_ramp,
                     budget=None if options.budget_translation is None else (options.budget_translation,options.budget_rotation),
                     prefix=options.source_prefix)
    else:
        run(options.source.resolve(),options.output.resolve(),options.report.resolve(),options.samples,options.fade)
