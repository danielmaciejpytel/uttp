"""Compare candidate magazine skin motions after rest-basis alignment.
No selection claim; identity WeaponRight->Barrel remains an unverified preview assumption.
"""
import hashlib,json,math,sys
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[3]
sys.path.insert(0,str(ROOT/'Source/tools/recon'))
from glb_anim_pose import load,accessor

def rotation(q):
    x,y,z,w=q;length=np.linalg.norm(q)
    if not np.isfinite(length) or length<1e-8:raise ValueError('Invalid quaternion')
    x,y,z,w=np.array(q)/length
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
                     [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
                     [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
def slerp(a,b,w):
    a=np.array(a);b=np.array(b);dot=float(a@b)
    if dot<0:b=-b;dot=-dot
    if dot>.9995:
        q=a+w*(b-a);return q/np.linalg.norm(q)
    angle=math.acos(np.clip(dot,-1,1));return (a*math.sin((1-w)*angle)+b*math.sin(w*angle))/math.sin(angle)
def transform(t,r,s):
    out=np.eye(4);out[:3,:3]=rotation(r)@np.diag(s);out[:3,3]=t;return out
class Animation:
    def __init__(self,path):
        self.path=path;self.doc,self.data=load(path);self.nodes=self.doc['nodes'];self.parents=[-1]*len(self.nodes)
        for p,n in enumerate(self.nodes):
            for child in n.get('children',[]):self.parents[child]=p
        self.names={n.get('name'):i for i,n in enumerate(self.nodes)};self.clips={}
        for a in self.doc['animations']:
            channels=[];duration=0
            for ch in a['channels']:
                sp=a['samplers'][ch['sampler']];times=np.array(accessor(self.doc,self.data,sp['input'])).reshape(-1);vals=accessor(self.doc,self.data,sp['output'])
                assert sp.get('interpolation','LINEAR') in ('LINEAR','STEP')
                duration=max(duration,float(times[-1]));channels.append((ch['target']['node'],ch['target']['path'],times,vals,sp.get('interpolation','LINEAR')))
            self.clips[a['name']]=(duration,channels)
    def model(self,name=None,time=0):
        rows=[[n.get('translation',[0,0,0]),n.get('rotation',[0,0,0,1]),n.get('scale',[1,1,1])] for n in self.nodes]
        if name:
            duration,channels=self.clips[name]
            for index,kind,times,vals,interp in channels:
                if kind not in ('translation','rotation','scale'):continue
                hi=min(len(times)-1,int(np.searchsorted(times,time,'right')));lo=max(0,hi-1)
                if time<=times[0]:lo=hi=0
                if time>=times[-1]:lo=hi=len(times)-1
                w=0 if hi==lo or interp=='STEP' else float((time-times[lo])/(times[hi]-times[lo]))
                value=slerp(vals[lo],vals[hi],w) if kind=='rotation' else (1-w)*np.array(vals[lo])+w*np.array(vals[hi])
                rows[index][{'translation':0,'rotation':1,'scale':2}[kind]]=value
        locals=[transform(*r) for r in rows];cache={}
        def world(i):
            if i not in cache:cache[i]=world(self.parents[i])@locals[i] if self.parents[i]>=0 else locals[i]
            return cache[i]
        return {name:world(i) for name,i in self.names.items()}

def main():
    generic=Animation(ROOT/'Build/test-data/ReloadPhaseProbe/retail-binding-export/w_handgun__mag_std.anims.glb')
    nue=Animation(ROOT/'Build/experiments/BodyDeformationOffline/weapons/nue-animation-export/base/animations/weapon/firearms/handgun/tsunami_nue/w_handgun__tsunami_nue__mag_std.anims.glb')
    restG=generic.model();restN=nue.model();report={'runtimeVerified':False,'basis':'GLB model space; generic skin motion multiplied by Nue rest basis','sources':{str(a.path.relative_to(ROOT)):hashlib.sha256(a.path.read_bytes()).hexdigest() for a in (generic,nue)},'clips':[]}
    for name in ('reload','reload_01','reload_02','empty_reload'):
        if name not in generic.clips or name not in nue.clips:continue
        dg=generic.clips[name][0];dn=nue.clips[name][0]
        for mode in ('same_seconds','normalized_phase'):
            rows=[]
            for k in range(81):
                t=k*min(dg,dn)/80 if mode=='same_seconds' else k*dn/80
                tg=t if mode=='same_seconds' else t/dn*dg
                mg=generic.model(name,tg);mn=nue.model(name,t)
                row={'nueSeconds':t,'genericSeconds':tg}
                for bone in ('mag_std','mag_stdr'):
                    aligned=mg[bone]@np.linalg.inv(restG[bone])@restN[bone]
                    difference=np.linalg.inv(mn[bone])@aligned
                    row[bone]={'positionDifferenceM':float(np.linalg.norm(aligned[:3,3]-mn[bone][:3,3])),
                        'rotationDifferenceDeg':math.degrees(math.acos(np.clip((np.trace(difference[:3,:3])-1)/2,-1,1)))}
                rows.append(row)
            report['clips'].append({'name':name,'comparison':mode,'genericDuration':dg,'nueDuration':dn,
                'maxDifferences':{bone:{key:max(r[bone][key] for r in rows) for key in ('positionDifferenceM','rotationDifferenceDeg')} for bone in ('mag_std','mag_stdr')},'samples':rows})
    report['limitations']=['Different candidate sets, not proof of loaded retail animation.','Rest-basis alignment is a diagnostic comparison, not a verified runtime remapping.','Visibility thresholds and WeaponRight-to-Barrel mount remain unverified.']
    out=ROOT/'Research/inventory/nue-magazine-source-motion-comparison-20261016.json';out.write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps([{k:v for k,v in r.items() if k!='samples'} for r in report['clips']],indent=2))
if __name__=='__main__':main()
