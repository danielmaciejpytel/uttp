import sys
def load(l,g):
    t={};r={};tot=''
    for ln in open(f'out/{l}_{g}.txt'):
        f=ln.split()
        if ln.startswith('FLTAB') and len(f)>=13 and f[1]!='key':
            t[f[1]]=dict(n=int(f[2]),pre=float(f[3]),post=float(f[4]),cmax=float(f[12]))
        if ln.startswith('FLROB') and f[1]!='key': r[f[1]]=float(f[6]) if len(f)>6 else 0
        if ln.startswith('FLTOT'): tot=ln.strip()
    return t,r,tot
keys=['Walk/Start','Run/Start','Sprint/Start','Crouch/Start','Walk/Reface','Run/Reface','Run/Turn','Sprint/Turn','Walk/Turn','Crouch/Turn','Walk/Loop','Run/Loop','Crouch/Loop','Walk/Stop','Run/Stop','Crouch/Stop','Idle/Loop','ALL']
A,B=sys.argv[1],sys.argv[2]
for g in ('woman','man'):
    ta,ra,tota=load(A,g); tb,rb,totb=load(B,g)
    print(f'## {g}  (cm/s of the planted toe in contact segments; pre = raw animation pose, post = after FootLocker)')
    print('%-13s %5s %7s | %8s %6s | %8s %6s | %7s | %s'%('state','n','raw','before','>25%','after','>25%','delta','corr max cm before/after'))
    for k in keys:
        if k in ta and k in tb:
            a=ta[k];b=tb[k]
            d=(b['post']-a['post'])/a['post']*100 if a['post']>0.5 else 0
            print('%-13s %5d %7.1f | %8.1f %6.1f | %8.1f %6.1f | %+6.0f%% | %.1f / %.1f'%(k,a['n'],a['pre'],a['post'],ra.get(k,0),b['post'],rb.get(k,0),d,a['cmax'],b['cmax']))
    print(tota[:200]); print(totb[:200])
