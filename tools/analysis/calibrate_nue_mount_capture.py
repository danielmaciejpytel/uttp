"""Compute a mount candidate from a finite native slot capture, not entity origin.
No game writes. Require motion and consistency before marking measured evidence.
"""
import argparse,hashlib,json,math,re
from pathlib import Path
import numpy as np
from compare_nue_magazine_sources import transform
from correct_p1h_reload_rigid_contacts import ROOT

def pose(text):
 v=np.array([float(x) for x in text.split(',')]);assert len(v)==7 and np.isfinite(v).all()
 assert abs(np.linalg.norm(v[3:])-1)<.01
 return transform(v[:3],v[3:],[1,1,1])
def angle(a,b):return math.degrees(math.acos(float(np.clip((np.trace(a[:3,:3].T@b[:3,:3])-1)/2,-1,1))))
def analyze(text,capture):
 rows=[];unavailable=[]
 for line in text.splitlines():
  m=re.search(r'\[utpp_mount_route\] DATA capture=(\d+) sample=(\d+) (.*)',line)
  if not m or int(m[1])!=capture:continue
  fields=m[3].split('|');components={};slots=[];player=None;metadata={}
  for field in fields:
   if field.startswith('playerWorld='):player=pose(field.split('=',1)[1])
   elif field.startswith(('slot=','component=')):
    values=dict(item.split('=',1) for item in field.split(';'));name=values.get('component')
    if name:assert name not in components;components[name]=values
    elif values.get('bone')=='WeaponRight':slots.append(values)
   elif '=' in field:
    key,value=field.split('=',1);metadata[key]=value
  if player is None or 'Barrel' not in components or not slots:unavailable.append(int(m[2]));continue
  barrel=pose(components['Barrel']['world'])
  for slot in slots:
   slotWorld=pose(slot['world']);relative=pose(slot['relative']);boneWorld=slotWorld@np.linalg.inv(relative)
   rows.append({'sample':int(m[2]),'slot':slot['slot'],'mount':np.linalg.inv(boneWorld)@barrel,'boneInPlayer':np.linalg.inv(player)@boneWorld,'metadata':metadata})
 if not rows:return {'runtimeVerified':False,'reason':'No complete WeaponRight-slot/Barrel samples','missingSamples':unavailable}
 reference=rows[0]['mount'];base=rows[0]['boneInPlayer']
 dp=max(float(np.linalg.norm(r['mount'][:3,3]-reference[:3,3])) for r in rows);dr=max(angle(reference,r['mount']) for r in rows)
 motionP=max(float(np.linalg.norm(r['boneInPlayer'][:3,3]-base[:3,3])) for r in rows);motionR=max(angle(base,r['boneInPlayer']) for r in rows)
 samples=len({r['sample'] for r in rows});complete=bool(re.search(r'\[utpp_mount_route\] DONE capture='+str(capture)+r' samples=\d+ reason=limit\b',text))
 stable=dp<.002 and dr<.5;moving=motionP>.03 or motionR>5
 routes={(r['metadata'].get('playerTemplate'),r['metadata'].get('weaponTemplate')) for r in rows}
 consistent=len(routes)==1 and all(all(v for v in route) for route in routes)
 verified=samples>=5 and stable and moving and complete and consistent and not unavailable
 return {'runtimeVerified':verified,'weapon':'Items.Preset_Nue_Default','basis':'CP-player-WeaponRight-to-Barrel','matrix':reference.reshape(-1).tolist(),'capture':capture,'samples':samples,'slotRows':len(rows),'captureCompleted':complete,'routesConsistent':consistent,'missingSamples':unavailable,'maxMountTranslationSpreadM':dp,'maxMountRotationSpreadDeg':dr,'weaponRightMotionInPlayerM':motionP,'weaponRightMotionInPlayerDeg':motionR,'motionSufficient':moving,'mountStable':stable,'metadata':rows[0]['metadata'],'deployable':False,'limitations':['Native GetSlotTransform and component.worldTransform must represent the active animation binding; validate assembled contact visually.','A stable measured mount is not proof of body skin, graph feature reception or complete reload.','Candidate uses first valid sample; do not average inconsistent samples or relax thresholds to pass.']}
def main():
 p=argparse.ArgumentParser();p.add_argument('log',type=Path);p.add_argument('--capture',type=int,required=True);p.add_argument('--out',default='Research/inventory/nue-measured-mount-20261016.json');args=p.parse_args()
 source=args.log.resolve();raw=source.read_bytes();report=analyze(raw.decode('utf-8',errors='replace'),args.capture)
 report.update(sourceEvidence=str(source),sourceSHA256=hashlib.sha256(raw).hexdigest());target=(ROOT/args.out).resolve();assert target.is_relative_to(ROOT)
 if target.exists():raise FileExistsError(target)
 target.write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2))
if __name__=='__main__':main()
