"""Synthetic transforms only; never creates runtime calibration evidence."""
import math,numpy as np
from calibrate_nue_mount_capture import analyze
from compare_nue_magazine_sources import transform
from correct_p1h_reload_rigid_contacts import local_row

def tr(p,q=(0,0,0,1)):return transform(p,q,[1,1,1])
def csv(m):return ','.join(f'{v:.12g}' for v in local_row(m)[:7])
mount=tr([.012,-.008,.004],[math.sin(.11),0,0,math.cos(.11)])
relative=tr([.03,.01,-.02],[0,0,math.sin(.07),math.cos(.07)])
def fixture(moving=True,bad=False,complete=True):
 rows=[]
 for i in range(6):
  player=tr([2000+i,1000,.3],[0,0,math.sin(.2),math.cos(.2)])
  a=.12*i if moving else 0
  bone=tr([.3,0,1.2],[0,math.sin(a),0,math.cos(a)])
  prop=player@bone@mount
  if bad and i==5:prop[0,3]+=.05
  rows.append('[utpp_mount_route] DATA capture=1 sample='+str(i+1)+' playerTemplate=test/player|weaponTemplate=test/nue|playerWorld='+csv(player)+'|slot=right;bone=WeaponRight;world='+csv(player@bone@relative)+';relative='+csv(relative)+'|component=Barrel;world='+csv(prop)+';local='+csv(np.eye(4)))
 if complete:rows.append('[utpp_mount_route] DONE capture=1 samples=6 reason=limit')
 return '\n'.join(rows)
r=analyze(fixture(),1);assert r['runtimeVerified'] and np.max(np.abs(np.array(r['matrix']).reshape(4,4)-mount))<1e-8
assert not analyze(fixture(moving=False),1)['runtimeVerified']
assert not analyze(fixture(bad=True),1)['runtimeVerified']
assert not analyze(fixture(complete=False),1)['runtimeVerified']
assert not analyze(fixture().replace('reason=limit','reason=unavailable'),1)['runtimeVerified']
assert not analyze(fixture(),2)['runtimeVerified']
assert not analyze(fixture().replace('slot=right;bone=WeaponRight','slot=right;bone=Other'),1)['runtimeVerified']
print('mount calibration synthetic: 7 cases PASS; no runtime evidence produced')
