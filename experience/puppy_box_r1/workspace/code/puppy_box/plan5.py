import walkplan as w, math, itertools, sys
a=[float(v) for v in sys.argv[1:]]
start=(a[0],a[1],math.radians(a[2])); tgt=(a[3],a[4],math.radians(a[5])); edge=a[6]; mc=a[7] if len(a)>7 else 0.36
acts=[('turn',c) for c in (-100,-90,-80,-70,-60,-50,-45,-40,-30,-20,20,30,40,45,50,60,70,80,90,100)]+[('walk',c) for c in (-0.1,0.15,0.2,0.3)]
best=[]
for n in (1,2,3):
  for seq in itertools.product(acts,repeat=n):
    q,ok=start,True
    for i,s in enumerate(seq):
      q=w.step(q,s)
      if edge-q[0]<mc: ok=False;break
    if not ok: continue
    e=math.hypot(q[0]-tgt[0],q[1]-tgt[1])+0.25*abs(q[2]-tgt[2])+0.01*n
    best.append((e,seq,q))
best.sort(key=lambda t:t[0])
for e,s,q in best[:8]: print(round(e,3),s,[round(q[0],3),round(q[1],3),round(math.degrees(q[2]),1)])
