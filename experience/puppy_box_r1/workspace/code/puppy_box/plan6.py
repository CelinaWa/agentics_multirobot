"""Plan walks scoring the PAW position in the final robot frame. python plan6.py x y psi paw_x paw_y edge [want_x want_y psi_lo psi_hi]"""
import walkplan as w, math, itertools, sys
a=[float(v) for v in sys.argv[1:]]
start=(a[0],a[1],math.radians(a[2])); paw=(a[3],a[4]); edge=a[5]
wx,wy=(a[6],a[7]) if len(a)>7 else (0.31,-0.20)
plo,phi=(a[8],a[9]) if len(a)>9 else (0,12)
acts=[('turn',c) for c in (-100,-90,-80,-70,-60,-50,-45,-40,-35,-30,-25,-20,20,25,30,35,40,45,50,60,70,80,90,100)]+[('walk',c) for c in (-0.1,0.15,0.2,0.3)]
def pawr(q):
    dx,dy=paw[0]-q[0],paw[1]-q[1]; c,s=math.cos(q[2]),math.sin(q[2])
    return (c*dx+s*dy, -s*dx+c*dy)
best=[]
for n in (1,2,3):
  for seq in itertools.product(acts,repeat=n):
    q,ok=start,True
    for i,st in enumerate(seq):
      q=w.step(q,st)
      if edge-q[0]<0.30: ok=False;break
    if not ok: continue
    px,py=pawr(q); d=math.degrees(q[2])
    e=math.hypot(px-wx,py-wy)+0.01*max(0,plo-d,d-phi)+0.01*n
    best.append((e,seq,q,(px,py)))
best.sort(key=lambda t:t[0])
for e,s,q,p in best[:8]: print(round(e,3),s,[round(q[0],3),round(q[1],3),round(math.degrees(q[2]),1)],'paw',[round(p[0],3),round(p[1],3)],'edge',round(edge-q[0],3))
