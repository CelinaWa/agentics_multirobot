"""Pelvis-frame points for pixels of a saved frame: python px.py <frame_id> u,v u,v ..."""
import sys, numpy as np
sys.path.insert(0, '/home/junjieye/rax-runtime-r1/sessions/20261106T010905Z_puppy_box_r1_f6b331/agent-workspace/code/puppy_box')
from percep import load_frame, cloud
meta, rgb, dep = load_frame(sys.argv[1])
P, ok = cloud(meta, dep, stride=1)
for s in sys.argv[2:]:
    u, v = map(int, s.split(','))
    w = P[max(v-1,0):v+2, max(u-1,0):u+2].reshape(-1, 3)
    print(s, np.round(np.median(w, 0), 3).tolist(), 'bgr', rgb[v, u].tolist())
