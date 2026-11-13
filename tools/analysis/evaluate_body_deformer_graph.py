"""Offline model-space deformer evaluator. Validate retail residuals before use."""
import json,sys,hashlib,csv
from pathlib import Path
import numpy as np
from replay_body_skin import trs,read_capture
sys.setrecursionlimit(20000)
ROOT=Path(__file__).resolve().parents[3]
# body -> (graph json, rig json, clip whose bone_names are the physical CORE set). woman = historical default (unchanged behaviour).
# man = player_man_fpp_deformations (the only male player deformation rig/graph in the game archives; 423 nodes vs 425 for woman:
# 2 fewer TranslationLimit nodes), serialized from the current game by WolvenKit into Extracted/game/body-v/json.
BODY_FILES={'woman':(ROOT/'Extracted/game/json/vanilla-extra/player_woman_base_deformations.animgraph.json',ROOT/'Extracted/game/json/vanilla-extra/player_woman_base_deformations.rig.json',ROOT/'Animations/retarget/out_sourcepack_woman/Pose_Pistol2H_Stand_Aim_Idle.utppclip.json'),
'man':(ROOT/'Extracted/game/body-v/json/player_man_fpp_deformations.animgraph.json',ROOT/'Extracted/game/body-v/json/player_man_fpp_deformations.rig.json',ROOT/'Animations/retarget/out_sourcepack_man/Pose_Pistol2H_Stand_Aim_Idle.utppclip.json')}
def vec(v):return np.array([v[k]for k in 'XYZ'],float)
def tm(q):return trs(list(vec(q['Translation']))+[1]+[q['Rotation'][k]for k in ['i','j','k','r']]+list(vec(q['Scale']))+[1])
def unit(v):
 n=np.linalg.norm(v)
 if n<1e-10:raise ValueError('Degenerate vector')
 return v/n
def quat(R):
 R=np.asarray(R);a,b,c=R[0];d,e,f=R[1];g,h,i=R[2]
 K=np.array([[a-e-i,b+d,c+g,h-f],[b+d,e-a-i,f+h,c-g],[c+g,f+h,i-a-e,d-b],[h-f,c-g,d-b,a+e+i]])/3
 _,v=np.linalg.eigh(K);return v[:,-1]
def rot(q):return trs([0,0,0,1]+list(q)+[1,1,1,1])[:3,:3]
def slerp(a,b,w):
 if np.dot(a,b)<0:b=-b
 c=np.clip(np.dot(a,b),-1,1)
 if c>.9995:return unit((1-w)*a+w*b)
 t=np.arccos(c);return (np.sin((1-w)*t)*a+np.sin(w*t)*b)/np.sin(t)
def basis(a,b):a=unit(a);b=unit(b-a*np.dot(a,b));return np.column_stack([a,b,np.cross(a,b)])
def swing(a,b):
 a=unit(a);b=unit(b);v=np.cross(a,b);c=np.dot(a,b)
 if c<-.999999:raise ValueError('Antipodal')
 K=np.array([[0,-v[2],v[1]],[v[2],0,-v[0]],[-v[1],v[0],0]]);return np.eye(3)+K+K@K/(1+c)
def twist(A,B,axis):
 side=np.array([0.,0.,1.])if abs(axis[2])<.5 else np.array([1.,0.,0.]);af=A@axis;aside=A@side;bs=swing(B@axis,af)@B@side
 return np.arctan2(np.dot(bs,np.cross(af,aside)),np.dot(aside,bs))
def axis(s):
 sign=-1 if s.startswith('Negative')else 1;return np.eye(3)['XYZ'.index(s[-1])]*sign
class Evaluator:
 def __init__(self,allow_rotation_bounce=False,body='woman'):
  self.allow_rotation_bounce=allow_rotation_bounce
  self.body=body;self.graphPath,self.rigPath,self.coreClipPath=BODY_FILES[body];graph=json.loads(self.graphPath.read_text());self.nodes={};stack=[graph]
  while stack:
   x=stack.pop()
   if isinstance(x,dict):
    if 'HandleId'in x and isinstance(x.get('Data'),dict)and str(x['Data'].get('$type','')).startswith('animAnimNode_'):self.nodes[int(x['HandleId'])]=x['Data']
    stack.extend(x.values())
   elif isinstance(x,list):stack.extend(x)
  self.rig=json.load(open(self.rigPath))['Data']['RootChunk'];self.names=[n['$value']for n in self.rig['boneNames']];self.parents=self.rig['boneParentIndexes'];self.refLocal={n:tm(t)for n,t in zip(self.names,self.rig['boneTransforms'])}
  self.coreNames=set(json.load(open(self.coreClipPath))['bone_names'])
  self.poseChainAssertions=[]
  for h in range(2,max(self.nodes)):
   following=int(self.nodes[h]['inputLink']['node']['HandleId'])
   if following!=h+1:raise ValueError('Graph pose chain differs from execution ordering')
   self.poseChainAssertions.append([h,following])
 def run(self,core,stop=1):
  if set(core)-self.coreNames:raise ValueError('Captured helper supplied as CORE input')
  missing=(self.coreNames&set(self.names))-set(core)
  if missing:raise ValueError('Missing physical core input: '+str(sorted(missing)))
  M={};dependencies={};parentNames={n:(self.names[self.parents[i]]if self.parents[i]>=0 else None)for i,n in enumerate(self.names)};self.writes={};self.snapshots={};self.skipped=[];self.nonstatic=[];self.tracks={}
  for i,n in enumerate(self.names):
   parent=self.names[self.parents[i]]if self.parents[i]>=0 else None
   dependencies[n]=(set(dependencies[parent])if parent else set())|({n}if n in self.coreNames else set())
   M[n]=(M[parent]if parent else np.eye(4))@core.get(n,self.refLocal[n])
  def name(x):return x['name']['$value']
  def check(n):
   absent=dependencies.get(n,set())-set(core)
   if absent:raise ValueError('Uncaptured influencing CORE dependency: '+str(sorted(absent)))
  def get(x):
   n=name(x);check(n);return M[n]
  def save(n,h):self.writes[n]=h
  for h in sorted(self.nodes,reverse=True):
   if h<stop:break
   node=self.nodes[h];t=node['$type'].removeprefix('animAnimNode_')
   if 'SourceChannel'in t:continue
   if t in ['Root','Output','ReferencePoseTerminator','SharedMetaPose','PoseLsToMs','PoseMsToLs','StackTransformsShrinker','StackTracksExtender','StackTracksShrinker','ConditionalSegmentBegin','ConditionalSegmentEnd','SkipPerformanceModeBegin','SkipPerformanceModeEnd']:continue
   if t=='StackTransformsExtender':
    if set(node['snapMethods'])!={'NoSnapping'}or any(node['snapToReferenceValues'])or any(node['offsetToReferenceValues']):raise ValueError('Unsupported extender snapping')
    if any(x['name']['$value']!='None'for x in node['offsetSpaceBones']):raise ValueError('Unsupported extender offset space')
    if any(np.max(np.abs(tm(x)-np.eye(4)))>1e-7 for x in node['offsets']):raise ValueError('Unsupported extender offsets')
    pending=list(node['transformInfos'])
    while pending:
     done=[]
     for e in pending:
      n=e['name']['$value'];p=e['parentName']['$value'];parentNames[n]=p
      if p in M or p=='None':
       if p in M:check(p)
       M[n]=(M[p]if p in M else np.eye(4))@tm(e['referenceTransformLs']);dependencies[n]=set(dependencies.get(p,set()));done.append(e)
     if not done:raise ValueError('Unresolved logical parents')
     pending=[e for e in pending if e not in done]
    continue
   if node.get('weightMode','Static')!='Static':self.nonstatic.append(h)
   if node.get('weight',1)!=1:raise ValueError('Unsupported fractional constraint weight')
   if t=='TwistConstraint':
    A=get(node['transformA'])[:3,:3];B=get(node['transformB'])[:3,:3];ang=twist(A,B,axis(node['frontAxis']))
    for e in node['outputs']:
     n=name(e['twistedTransform']);p=parentNames[n];v=axis(e['twistAxis']);theta=ang*e['negativeScale'if ang<0 else 'positiveScale'];q=np.r_[v*np.sin(theta/2),np.cos(theta/2)];M[n][:3,:3]=M[p][:3,:3]@rot(q);save(n,h)
   elif t=='SetBoneTransform':
    for e in node['entries']:
     n=name(e['transformToChange']);s=name(e['sourceBone']);p=name(e['offsetSpaceBone'])
     if e['snapToReference']or e['offsetToReference']:raise ValueError('Reference snapping unsupported')
     if e['setMethod']=='WholeTransform':M[n]=M[s].copy()
     elif e['setMethod']=='TranslationOnly':M[n][:3,3]=M[s][:3,3]
     elif e['setMethod']=='RotationOnly':M[n][:3,:3]=M[s][:3,:3]
     elif e['setMethod']!='NoSnapping':raise ValueError(e['setMethod'])
     if p!='None':
      P=M[p].copy();L=np.linalg.inv(P)@M[n];off=tm(e['offset']);L[:3,3]+=off[:3,3];L[:3,:3]=off[:3,:3]@L[:3,:3];M[n]=P@L
     save(n,h)
   elif t in ['PointConstraint','OrientConstraint']:
    n=name(node['transformIndex']);entries=node['inputWeightedTransforms'];weights=node['preprocessedWeights']
    if t=='PointConstraint':M[n][:3,3]=sum(w*get(e['transform'])[:3,3]for w,e in zip(weights,entries))
    else:
     q=quat(get(entries[0]['transform'])[:3,:3])
     for w,e in zip(weights,entries[1:]):q=slerp(q,quat(get(e['transform'])[:3,:3]),w)
     M[n][:3,:3]=rot(q)
    save(n,h)
   elif t.startswith('AimConstraint_'):
    n=name(node['transformIndex']);P=M[n][:3,3];d=get(node['targetTransform'])[:3,3]-P
    up=get(node['upTransform'])
    u=up[:3,:3]@vec(node['upTransformVector'])if t.endswith('ObjectRotationUp')else up[:3,3]-P
    try:R=basis(d,u)@basis(vec(node['forwardAxisLS']),vec(node['upAxisLS'])).T
    except ValueError:self.skipped.append({'handle':h,'reason':'degenerate aim'});continue
    M[n][:3,:3]=R;save(n,h)
   elif t=='SimpleSpline':
    n=name(node['constrainedTransform']);u=node['defaultProgress']
    if node['progressMode']=='FloatTrack':
     track=name(node['progressTrack'])
     if track not in self.tracks:raise ValueError('Unproduced spline track '+track)
     u=float(np.clip(self.tracks[track],0,1))
    elif node['progressMode']!='Static':raise ValueError('Unsupported spline progress mode')
    M[n][:3,3]=(1-u)**2*get(node['startTransform'])[:3,3]+2*u*(1-u)*get(node['middleTransform'])[:3,3]+u*u*get(node['endTransform'])[:3,3];save(n,h)
   elif t=='SimpleBounce':
    val=(np.linalg.inv(get(node['startTransform']))@get(node['endTransform']))[0,3];mult=node['negativeMultiplier']if val<0 else node['multiplier'];v=max(0,abs(val)-node['delay']);smooth=node['smoothStep'];out=(v*v/smooth if smooth and v<smooth else v)*mult+node['offset']
    if name(node['outputDriverTrack'])!='None':raise ValueError('Unsupported bounce driver track')
    for e in node['trackOutputs']:
     track=name(e['targetTrack']);value=float(out*e['multiplier'])
     if not np.isfinite(value):raise ValueError('Nonfinite bounce track')
     self.tracks[track]=value
    for e in node['transformOutputs']:
     n=name(e['targetTransform']);P=get(e['parentTransform']).copy();L=np.linalg.inv(P)@M[n]
     rotations={}
     for c in e['channelEntries']:
      channel=c['transformChannel']
      a='XYZ'.index(channel[-1]);value=c['multiplier']*out
      if channel.startswith('Pos'):L[a,3]=value
      elif channel.startswith('Scale'):L[:3,a]=unit(L[:3,a])*value
      elif channel.startswith('Rot') and self.allow_rotation_bounce:rotations[channel]=value
      else:raise ValueError('Rotation bounce unsupported '+channel)
     if rotations:
      from deformer_rotation_channels import replace_rotation_channels
      L=replace_rotation_channels(L,rotations)
     M[n]=P@L;save(n,h)
   elif t=='TranslationLimit':
    n=name(node['constrainedTransform']);pn=name(node['parentTransform']);P=M[parentNames[n]]if pn=='None'else M[pn];L=np.linalg.inv(P)@M[n]
    for a in range(3):
     c=node['limitOn'+'XYZ'[a]+'Axis']
     if c['useMin']:L[a,3]=max(L[a,3],c['min'])
     if c['useMax']:L[a,3]=min(L[a,3],c['max'])
    M[n]=P@L;save(n,h)
   else:raise ValueError('Unsupported '+t)
   if self.body=='woman' and h in [325,247,315,245]:self.snapshots[str(h)]={n:m.copy()for n,m in M.items()if any(x in n for x in ['scapula_A_','deltoid_back_bot','backShoulder_musclePoint','OrbitRig_backMusclePoint','upArm_backMusclePoint'])}
  return M

def load_checkpoint_core(proof,coreNames):
 top,frames=read_capture(ROOT/proof['capture']);key,rows=sorted(frames.items())[proof['ordinal']]
 if key[0]!=proof['ticks']:raise ValueError('Checkpoint ordinal/ticks mismatch')
 local={r['name']:trs([float(r[f's0_{k}'])for k in range(12)])for r in rows.values()}
 core={n:m for n,m in local.items()if n in coreNames};supplement=[]
 trace=ROOT/'Research/logs/game/skin-complete-runtime-20261015/utpp-pose-4370899098832.csv'
 with trace.open()as f:
  for r in csv.DictReader(x for x in f if not x.startswith('#')):
   if int(r['ticks'])!=proof['earlyTicks']:continue
   if (int(r['desc']),int(r['thread']),int(r['rig']))!=key[1:4]:raise ValueError('Paired early identity mismatch')
   b=int(r['bone']);n=coreNames[b]
   if n not in core and int(r['gameIndex'])>=0:
    core[n]=trs([float(r[f's2_{k}'])for k in range(12)]);supplement.append(n)
 return top,key,rows,local,core,supplement

def main():
 base=ROOT/'Research/logs/game/skin-complete-runtime-20261015';proof=json.load(open(base/'shoulder-spike-contributors.json'));ev=Evaluator();coreNames=json.load(open(ROOT/'Animations/retarget/out_sourcepack_woman/Pose_Pistol2H_Stand_Aim_Idle.utppclip.json'))['bone_names'];report={'scope':'Experimental model-space graph evaluator, shoulder branch handles424 down to192 only; elbow/wrist later handles not executed. Captured CORE only as input; helpers start from resource reference, never captured helper values. Historical formulas require retail output residual validation.','states':{}}
 for label,st in proof['states'].items():
  top,key,rows,local,core,supplement=load_checkpoint_core(st,coreNames);result=ev.run(core,stop=192);actual={}
  def fk(i):
   if i not in actual:
    if i not in rows:raise ValueError('Missing captured ancestor')
    p=top[i][1];actual[i]=(fk(p)if p>=0 else np.eye(4))@local[rows[i]['name']]
   return actual[i]
  residual=[]
  for i,r in rows.items():
   n=r['name']
   if n in coreNames or n not in ev.writes:continue
   A=fk(i);B=result[n];residual.append({'name':n,'lastWriter':ev.writes.get(n),'positionErrorMeters':float(np.linalg.norm(A[:3,3]-B[:3,3])),'rotationErrorDeg':float(np.degrees(np.arccos(np.clip((np.trace(rot(quat(A[:3,:3])).T@rot(quat(B[:3,:3])))-1)/2,-1,1))))})
  residual.sort(key=lambda x:x['positionErrorMeters'],reverse=True)
  report['states'][label]={'captureTicks':key[0],'capturedCoreNames':sorted(core),'pairedEarlyS2SupplementCoreNames':supplement,'uncapturedCoreNames':sorted(set(coreNames)-set(core)),'skipped':ev.skipped,'nonstaticConstraints':ev.nonstatic,'helperResiduals':residual,'verifiedModelNames':sorted(set(core)|{r['name']for r in residual if r['positionErrorMeters']<1e-5 and r['rotationErrorDeg']<.01}),'retailResidualCriteria':{'positionMeters':1e-5,'rotationDegrees':.01},'executedHandleRange':[424,192],'poseChainEdgesAsserted':len(ev.poseChainAssertions),'noCapturedHelpersAsInput':True,'missingInfluencingCoreFailsClosed':True}
  np.savez(ROOT/'Build/experiments/BodyDeformationOffline'/('deformer-eval-'+label+'.npz'),**{n:m for n,m in result.items()})
 meta=json.load(open(ROOT/'Build/experiments/BodyDeformationOffline/female/body-skin-data.json'));skin=np.load(ROOT/'Build/experiments/BodyDeformationOffline/female/body-skin-data.npz')
 for label,state in report['states'].items():
  state['primitiveCoverage']={}
  for primitive in ['primitive_0','primitive_1']:
   js=skin[primitive+'_joints'];weights=skin[primitive+'_weights'];weighted=sorted({meta['jointNames'][i]for i in js[weights>0]});missing=sorted(set(weighted)-set(state['verifiedModelNames']))
   state['primitiveCoverage'][primitive]={'requiredNonzeroJointNames':weighted,'missingVerifiedNames':missing,'dependencyComplete':not missing}
 inputs=[ev.graphPath,ROOT/'Extracted/game/json/vanilla-extra/player_woman_base_deformations.rig.json',base/'utpp-pose-4370899098832.csv',base/'utpp-2026-10-15-12-17-54.log']+[ROOT/st['capture']for st in proof['states'].values()]
 report['provenance']={'captureStage':'early-main paired with body helper probe s0; helpers not inputs','requiredLogMarker':'UTPP early pose experiment: owner=Mapping catalogValid=1','inputs':[{'path':str(p.relative_to(ROOT)),'sha256':hashlib.sha256(p.read_bytes()).hexdigest()}for p in inputs]}
 if report['provenance']['requiredLogMarker']not in inputs[3].read_text():raise ValueError('Missing early owner provenance')
 report['checkpointLabelCaveat']='Folder ready is lowered Relax-like; action0/overlay2/aim0 does not establish internal Ready.'
 p=base/'deformer-graph-evaluator-validation.json';p.write_text(json.dumps(report,indent=2)+'\n');print(json.dumps({k:v['helperResiduals'][:8]for k,v in report['states'].items()},indent=2))
if __name__=='__main__':main()

