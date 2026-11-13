"""Local asset copies: remove FPP plane animation bindings from female TPP nails only."""
import json,copy,hashlib
from pathlib import Path
ROOT=Path(__file__).resolve().parents[3]
SRC=ROOT/'Extracted/game/vanilla-bodies/base/characters/common/player_base_bodies/appearances'
OUT=ROOT/'Build/experiments/NailsPlaneFix-20261017'
TARGET={'a0_000_pwa_base_nails_l','a0_000_pwa_base_nails_r'}
def value(x):return x.get('$value') if isinstance(x,dict) else x
def patch(x,edits):
 if isinstance(x,dict):
  if x.get('$type')=='entMorphTargetSkinnedMeshComponent' and value(x.get('name')) in TARGET:
   field=x['renderingPlaneAnimationParam'];old=field['$value'];assert old in ('renderPlaneLeftArm','renderPlane')
   field['$value']='None';edits.append({'component':value(x['name']),'before':old,'after':'None'})
  for y in x.values():patch(y,edits)
 elif isinstance(x,list):
  for y in x:patch(y,edits)
def main():
 report={'runtimeVerified':False,'scope':'Two female TPP nail names only; clear animation-driven plane binding. Scene default must still be confirmed visually. No geometry/material/FPP edits.','assets':[]}
 for name in ('a0_000_base__nails.app','a0_000_base__full.app'):
  src=SRC/(name+'.json');d=json.loads(src.read_text());edits=[];patch(d,edits);assert edits and {e['component'] for e in edits}==TARGET
  out=OUT/'json'/ 'base/characters/common/player_base_bodies/appearances'/(name+'.json');out.parent.mkdir(parents=True,exist_ok=True);out.write_bytes(json.dumps(d,indent=2).encode())
  report['assets'].append({'resource':str(out.relative_to(OUT/'json')),'sourceSHA256':hashlib.sha256(src.read_bytes()).hexdigest(),'changes':edits,'count':len(edits)})
 fpp=SRC/'a0_000_fpp__nails.app';report['fppUntouchedSHA256']=hashlib.sha256(fpp.read_bytes()).hexdigest()
 (ROOT/'Research/inventory/female-tpp-nails-plane-20261017.json').write_bytes(json.dumps(report,indent=2).encode());print([(a['resource'],a['count'])for a in report['assets']])
if __name__=='__main__':main()
