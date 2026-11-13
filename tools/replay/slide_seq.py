import csv,collections,json,sys
by=collections.defaultdict(list)
for r in csv.DictReader(open(sys.argv[1])): by[r['feed']].append(r)
meta={m['file']:m for m in json.load(open(sys.argv[2]))}
for f,rows in sorted(by.items()):
    seq=[];last=None
    for r in rows:
        t=float(r['t'])
        if meta[f]['slide_start']<=t<=meta[f]['slide_end']+1.0:
            c=r['motionclip'].replace('M_Neutral_Slide_','').replace('M_Neutral_','')
            if c!=last: seq.append((round(t-meta[f]['slide_start'],2),c[:26],r['speed'])); last=c
    print(f,'changes',len(seq)-1,seq[:7])
