"""3-D pelvis-frame points for pixels of a saved frame. python p3d.py frame u,v [u,v ...]"""
import sys, json
import numpy as np
sys.path.insert(0, '/home/junjieye/rax-runtime-r1/sessions/20261106T043952Z_puppy_box_r1_cf6e88/agent-workspace/code/puppy_box')
from percep import load_frame, cloud
meta, rgb, dep = load_frame(sys.argv[1])
P, ok = cloud(meta, dep, stride=1)
for a in sys.argv[2:]:
    u, v = [int(t) for t in a.split(',')]
    w = P[max(v-1,0):v+2, max(u-1,0):u+2].reshape(-1, 3)
    print(a, np.round(np.median(w, 0), 4).tolist())
