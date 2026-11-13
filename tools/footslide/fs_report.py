"""python -I fs_report.py <m.json> [more.json for 'after'] : family summary + worst clips (cm/frame in contact)."""
import json, re, sys
import numpy as np
fam = [('Turn_',r'_Turn_'),('IdleTurn',r'Idle_Turn|_turn_(left|right)'),('Pivot',r'_Pivot_'),('Reface',r'_Reface_'),('Loop/Start/Stop etc',r'.')]
def famof(n):
    for k,rx in fam[:-1]:
        if re.search(rx,n): return k
    return 'other'
def stat(m):
    return np.mean([ (m[s]['mean']) for s in 'LR' if m[s]['frames']]) if any(m[s]['frames'] for s in 'LR') else 0
a=json.load(open(sys.argv[1]))
by={}
for n,m in a.items():
    by.setdefault(famof(n),[]).append((n,m))
print('%-10s %4s %8s %8s %8s %8s %8s'%('family','n','mean','p95mean','maxmax','srcmean','pathsum'))
for k,l in by.items():
    mean=np.mean([stat(m) for n,m in l]); p95=np.mean([max(m['L']['p95'],m['R']['p95']) for n,m in l]); mx=max(max(m['L']['max'],m['R']['max']) for n,m in l)
    sm=np.mean([ (m['srcL']['mean']+m['srcR']['mean'])/2 for n,m in l]); ps=np.mean([m['L']['path']+m['R']['path'] for n,m in l])
    print('%-10s %4d %8.2f %8.2f %8.2f %8.2f %8.1f'%(k,len(l),mean,p95,mx,sm,ps))
