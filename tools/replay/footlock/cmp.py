import sys,re
labels=sys.argv[1:]
def load(l,g):
    t={};r={}
    tot=None
    for ln in open(f'out/{l}_{g}.txt'):
        f=ln.split()
        if ln.startswith('FLTAB') and len(f)>=13 and f[1] not in ('key',):
            t[f[1]]=dict(n=int(f[2]),pre=float(f[3]),post=float(f[4]),p95=float(f[5]),step=float(f[8]),nostep=float(f[9]),cm=float(f[10]),cp95=float(f[11]),cmax=float(f[12]))
        if ln.startswith('FLROB') and f[1]!='key':
            r[f[1]]=dict(p50=float(f[4]),o25=float(f[6]))
        if ln.startswith('FLTOT'):
            tot=ln.strip()
    return t,r,tot
for g in ('woman','man'):
    print('##',g)
    data={l:load(l,g) for l in labels}
    keys=[k for k in data[labels[0]][0]]
    print('%-14s %5s %7s'%('key','n','pre')+''.join('| %-6s mean  p50  >25%%  cmax '%l[:6] for l in labels))
    for k in keys:
        b=data[labels[0]][0][k]
        row='%-14s %5d %7.1f'%(k,b['n'],b['pre'])
        for l in labels:
            t,r,_=data[l]
            x=t.get(k); y=r.get(k)
            row+='| %6.1f %5.1f %5.1f %5.1f '%(x['post'],y['p50'],y['o25'],x['cmax']) if x else '| -'
        print(row)
    for l in labels:
        print(l, data[l][2][:260] if data[l][2] else '')
