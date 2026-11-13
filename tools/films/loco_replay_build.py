"""Build of the replay used by the locomotion comparison films (2026-11-09): the official utpp_motion_replay and a COPY of it with a
per-frame pose dump.  Source\\plugin is only read; the copy is written to Build\\work\\loco-films-20261109\\replay.

  python -I loco_replay_build.py            -> Build\\UTPP\\utpp_motion_replay.exe (ninja, official target, unchanged source)
                                               Build\\work\\loco-films-20261109\\replay\\loco_replay_dump.cpp / .exe (patched copy)
Patches of the copy (string replacements on Source\\plugin\\UTPP\\tests\\FeedReplayMotion.cpp, each asserted to match exactly once):
  1 --dump <file>: inside FlFrame (the DLL leg tail: visual root rootframe::Processor -> FootLocker, needs --footlock) every frame
    writes   F <feed> <i> <t> <capX> <capY> <actorYaw rad> <visX> <visY> <visYaw rad> <clip> <clipTime> <stance> <gait> <mode> <jumped> <db>
             P <73 x (px py pz qx qy qz qw)>   = S.fl.Output(), the FootLocker output pose (game-local, Root / Trajectory identity)
             R <73 x (px py pz qx qy qz qw)>   = the FootLocker input (evaluator pose, Root / Trajectory identity), same placement
    and once per run the skeleton:  B <index> <name> <parent>;  per foot (placement check of the films, loco_common.placement_check):
             T <side> <lock bone> <world x> <world y> <contact>   = the FootLocker's own world position of the lock bone (post).
  2 --parity (film runs only; without it the copy must reproduce the official CSV bit for bit): the MapConfig values the DLL sets from
    the installed settings (utpp_settings.txt 2026-11-09: mm.walkSpeed 1.6, mm.minWalkSpeed 0.82, MotionDriver.cpp 4418-4447) and
    the capsule acceleration / braking of the synthetic feed (--opt capAccel=<m/s2> --opt capBrake=<m/s2>, = aRaw.capsuleAcceleration /
    capsuleBrakingDeceleration of the DLL, which come from the game there).
Everything else of the replay (MotionAdapter, PoseEvaluator with overlay Default + style Feminine, chooser, slide feed with --slide,
FootLocker with --footlock) is the official code, unchanged.
"""
import os
import subprocess
import sys

ROOT = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', '..'))
SRC = os.path.join(ROOT, 'Source', 'plugin', 'UTPP', 'tests', 'FeedReplayMotion.cpp')
OUT = os.path.join(ROOT, 'Build', 'work', 'loco-films-20261109', 'replay')
LOG = os.path.join(ROOT, 'Build', 'work', 'loco-films-20261109', 'logs', 'build_replay.log')
VCVARS = os.path.join(os.environ.get('ProgramFiles', ''), 'Microsoft Visual Studio', '2022', 'Community', 'VC', 'Auxiliary', 'Build', 'vcvars64.bat')
NINJA_DIR = os.path.join(ROOT, 'Tools', 'ninja')

PATCHES = [
    ('FlRunTotals g_flTot;\n',
     'FlRunTotals g_flTot;\n'
     'FILE* g_dump = nullptr;  // loco films: --dump (Source/tools/films/loco_replay_build.py)\n'
     'bool g_dumpBones = false;\n'
     'bool g_parity = false;   // loco films: --parity (DLL MapConfig values of the installed settings)\n'),
    ('    const std::span<const BoneTransform> out = S.fl.Output();\n',
     '    const std::span<const BoneTransform> out = S.fl.Output();\n'
     '    if (g_dump)\n'
     '    {\n'
     '        std::fprintf(g_dump, "F %s %zu %.5f %.6f %.6f %.7f %.6f %.6f %.7f %s %.5f %d %d %d %d %d\\n", f.name.c_str(), i,\n'
     '                     static_cast<double>(r.t), static_cast<double>(f.px[i]), static_cast<double>(f.py[i]), static_cast<double>(actorYaw),\n'
     '                     vpx, vpy, static_cast<double>(vyaw), clipName.c_str(), static_cast<double>(m.clipTime), cs.stance, cs.gait,\n'
     '                     cs.movementMode, m.jumped ? 1 : 0, m.database);\n'
     '        std::fprintf(g_dump, "P");\n'
     '        for (uint32_t b = 0; b < out.size(); ++b)\n'
     '            std::fprintf(g_dump, " %.6f %.6f %.6f %.7f %.7f %.7f %.7f", static_cast<double>(out[b].p.x), static_cast<double>(out[b].p.y),\n'
     '                         static_cast<double>(out[b].p.z), static_cast<double>(out[b].q.x), static_cast<double>(out[b].q.y),\n'
     '                         static_cast<double>(out[b].q.z), static_cast<double>(out[b].q.w));\n'
     '        std::fprintf(g_dump, "\\n");\n'
     '        std::fprintf(g_dump, "R");  // the leg tail INPUT (evaluator pose, Root / Trajectory identity): pre-FootLocker check\n'
     '        for (uint32_t b = 0; b < aPose.size(); ++b)\n'
     '            std::fprintf(g_dump, " %.6f %.6f %.6f %.7f %.7f %.7f %.7f", static_cast<double>(aPose[b].p.x), static_cast<double>(aPose[b].p.y),\n'
     '                         static_cast<double>(aPose[b].p.z), static_cast<double>(aPose[b].q.x), static_cast<double>(aPose[b].q.y),\n'
     '                         static_cast<double>(aPose[b].q.z), static_cast<double>(aPose[b].q.w));\n'
     '        std::fprintf(g_dump, "\\n");\n'
     '    }\n'),
    ('    mc.stanceCleanMaxSpeedMs = adapter::kStanceCleanMaxSpeedMs;\n',
     '    mc.stanceCleanMaxSpeedMs = adapter::kStanceCleanMaxSpeedMs;\n'
     '    if (g_parity)\n'
     '    {\n'
     '        mc.gameWalkTogglePoint = 1.6f;      // mm.walkSpeed (installed utpp_settings.txt)\n'
     '        adapter::SetAnalogFloor(mc, 0.82f);    // mm.minWalkSpeed\n'
     '        mc.capsuleAcceleration = g_fl.Opt("capAccel", 0.0f);\n'
     '        mc.capsuleBrakingDeceleration = g_fl.Opt("capBrake", 0.0f);\n'
     '    }\n'
     '    if (g_dump && !g_dumpBones)\n'
     '    {\n'
     '        for (uint32_t b = 0; b < db.NumBones(); ++b)\n'
     '        {\n'
     '            const std::string bn(db.BoneName(b));\n'
     '            std::fprintf(g_dump, "B %u %s %d\\n", b, bn.c_str(), static_cast<int>(db.BoneParent(b)));\n'
     '        }\n'
     '        g_dumpBones = true;\n'
     '    }\n'),
    ('        else if (a == "--db" && i + 1 < argc)\n',
     '        else if (a == "--dump" && i + 1 < argc)\n'
     '            g_dump = OpenWrite(argv[++i]);\n'
     '        else if (a == "--parity")\n'
     '            g_parity = true;\n'
     '        else if (a == "--db" && i + 1 < argc)\n'),
    # placement check of the films: the FootLocker's own world position of each foot's lock bone (post) and its contact flag
    ('            S.fl.CharacterToWorld(S.fl.ModelSpace(out, lb).p, x, y);\n            post[0] = x;\n            post[1] = y;\n        }\n',
     '            S.fl.CharacterToWorld(S.fl.ModelSpace(out, lb).p, x, y);\n            post[0] = x;\n            post[1] = y;\n        }\n'
     '        if (g_dump)\n'
     '            std::fprintf(g_dump, "T %d %d %.6f %.6f %d\\n", s, lb, post[0], post[1], d.contact ? 1 : 0);\n'),
    ('    std::printf("\\nREPLAY_FAILS=%d\\n", g_fail);\n',
     '    if (g_dump)\n'
     '        std::fclose(g_dump);\n'
     '    std::printf("\\nREPLAY_FAILS=%d\\n", g_fail);\n'),
]


def main():
    os.makedirs(OUT, exist_ok=True)
    os.makedirs(os.path.dirname(LOG), exist_ok=True)
    code = open(SRC, encoding='utf-8').read()
    for old, new in PATCHES:
        assert code.count(old) == 1, ('patch anchor not unique / missing', old[:80], code.count(old))
        code = code.replace(old, new)
    code = '// GENERATED by Source/tools/films/loco_replay_build.py from Source/plugin/UTPP/tests/FeedReplayMotion.cpp (not a project source).\n' + code
    cpp = os.path.join(OUT, 'loco_replay_dump.cpp')
    open(cpp, 'w', encoding='utf-8', newline='\n').write(code)
    inc = os.path.join(ROOT, 'Source', 'plugin', 'UTPP', 'tests')
    cmd = os.path.join(OUT, 'build.cmd')
    with open(cmd, 'w', encoding='ascii', newline='\r\n') as f:
        f.write('@echo off\n')
        f.write('set "PATH=%s;%%PATH%%"\n' % NINJA_DIR)
        f.write('call "%s" >nul\n' % VCVARS)
        f.write('ninja -C "%s" utpp_motion_replay\n' % os.path.join(ROOT, 'Build', 'UTPP'))
        f.write('if errorlevel 1 exit /b 11\n')
        f.write('cd /d "%s"\n' % OUT)
        f.write('cl /nologo /TP /DWIN32 /D_WINDOWS /EHsc /O2 /Ob2 /DNDEBUG -std:c++20 -MD /W4 /permissive- /utf-8 /I "%s" "%s" '
                '/Fo"%s" /Fe"%s" /link /INCREMENTAL:NO /subsystem:console\n'
                % (inc, cpp, os.path.join(OUT, 'loco_replay_dump.obj'), os.path.join(OUT, 'loco_replay_dump.exe')))
        f.write('if errorlevel 1 exit /b 12\n')
        f.write('echo BUILD_OK\n')
    p = subprocess.run(['cmd', '/c', cmd], capture_output=True, text=True)
    open(LOG, 'w', encoding='utf-8').write(p.stdout + '\n' + p.stderr)
    print(p.stdout[-3000:], p.stderr[-2000:])
    print('EXIT', p.returncode)
    return p.returncode


if __name__ == '__main__':
    sys.exit(main())
