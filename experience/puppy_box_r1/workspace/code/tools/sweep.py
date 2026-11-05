import sys, json, numpy as np
sys.path.insert(0,'kin')
from r1_kin import Kinematics
K = Kinematics('kin/r1_piper_kin.xml')
OFF=0.107
def run(g, a, f, waist=False):
    a=np.asarray(a,float); a/=np.linalg.norm(a)
    gl=np.asarray(g)+OFF*a
    q,err,ang=K.ik('right', gl.tolist(), a.tolist(), None, waist, 0.15, 300, 0.05, f, 8, 0)
    K.set(q); p,R=K.site('right_grasp')
    fa=abs(R[:,1]@np.asarray(f)/np.linalg.norm(f)) if f is not None else None
    return err, ang, fa, q
appr = {'side+y':(0,1,0),'diag45L':(0.7,0.7,0),'diag60L':(0.5,0.87,0),'diag37L':(0.8,0.6,0),'diag45R':(0.7,-0.7,0),'diag60R':(0.5,-0.87,0),
        'below60':(0.5,0,0.87),'below75':(0.26,0,0.97),'up':(0,0,1),'fwd':(1,0,0.1), 'below45':(0.7,0,0.7), 'belowdiag':(0.5,0.5,0.7)}
for name,a in appr.items():
    for y in (-0.3,-0.2,-0.1,0.0,0.1):
        row=[]
        for x in (0.3,0.4,0.5):
            f=(0,0,1) if abs(a[2])<0.5 else (0,1,0)
            err,ang,fa,q=run([x,y,0.0],a,f, waist=False)
            row.append(f"{err:.3f}/{ang:3.0f}/{fa:.2f}")
        print(f"{name:9s} y={y:+.1f} ", "  ".join(row))
