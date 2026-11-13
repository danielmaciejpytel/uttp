"""Rig/mesh consistency of the body deformation rigs: per helper joint, bind distance to the rig parent (glb mesh) vs the rig reference local translation length (cm). python -I check_deform_rig_bind.py"""
import sys,numpy as np
sys.path.insert(0,__import__('os').path.dirname(__import__('os').path.abspath(__file__)))
import measure_body_deformation as M
SK=M.SK
groups=['butterfly','latissimus','SHL','scapula','deltoid','chest_front','triceps','elbow','biceps','knee','calf','thigh','neck']
res={}
for body in ['woman','man']:
    rig=M.Rig(body)
    p=rig.parts[0]   # torso mesh carries the helper tree
    rows=[]
    for i,n in enumerate(rig.names):
        if rig.par[i]<0 or n not in p['node_of']: continue
        pn=rig.names[rig.par[i]]
        if pn not in p['node_of']: continue
        a=p['nodew'][p['node_of'][n]][:3,3]; b=p['nodew'][p['node_of'][pn]][:3,3]
        dbind=np.linalg.norm(a-b); drig=np.linalg.norm(rig.ref[i][:3,3])
        rows.append((n,pn,dbind*100,drig*100,(drig-dbind)*100))
    res[body]=rows
    print(body,'joints compared',len(rows))
    for g in groups:
        r=[x for x in rows if g in x[0]]
        if r: print('  %-12s n=%2d mean|d| %.2f cm max|d| %.2f cm (%s)'%(g,len(r),np.mean([abs(x[4]) for x in r]),max(abs(x[4]) for x in r),max(r,key=lambda x:abs(x[4]))[0]))
    allr=[abs(x[4]) for x in rows]; print('  ALL mean %.2f max %.2f'%(np.mean(allr),max(allr)))
# top male
print('male top deltas:');
for x in sorted(res['man'],key=lambda x:-abs(x[4]))[:8]: print('  %s <- %s bind %.2f rig %.2f d %.2f cm'%x)
print('female top deltas:')
for x in sorted(res['woman'],key=lambda x:-abs(x[4]))[:8]: print('  %s <- %s bind %.2f rig %.2f d %.2f cm'%x)
