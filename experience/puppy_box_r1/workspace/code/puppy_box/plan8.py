"""Depth-5 structured planner: back^k, turn L, [walk], turn R, [walk]. python plan8.py x y psi tx ty tpsi edge"""
import walkplan as w, math, itertools, sys
a=[float(v) for v in sys.argv[1:]]
start=(a[0],a[1],math.radians(a[2])); tgt=(a[3],a[4],math.radians(a[5])); edge=a[6]
res=[]
for nb in (0,1,2,3):
  for tl in (40,50,60,70,80,90):
    for w1 in (None,0.15,0.2,0.3):
      for tr in (-40,-50,-60,-70,-80,-90):
        for w2 in (None,0.15,0.2):
          seq=[('walk',-0.1)]*nb+[('turn',tl)]+([('walk',w1)] if w1 else [])+[('turn',tr)]+([('walk',w2)] if w2 else [])
          q=start; ok=True; mind=9
          for s in seq:
            q=w.step(q,s); mind=min(mind,edge-q[0])
            if edge-q[0]<0.34: ok=False;break
          if not ok: continue
          e=math.hypot(q[0]-tgt[0],q[1]-tgt[1])+0.25*abs(q[2]-tgt[2])+0.01*len(seq)
          res.append((e,seq,q,mind))
res.sort(key=lambda t:t[0])
for e,s,q,m in res[:10]: print(round(e,3),s,[round(q[0],3),round(q[1],3),round(math.degrees(q[2]),1)],'edge',round(edge-q[0],3),'min',round(m,3))
