"""Localize the overhanging hind leg from the head-camera DEPTH (the real pipeline), compare to ground truth.
Pitch the head down so the grasp workspace is in view, back-project depth -> pelvis-frame point cloud, find the
table top (z-histogram peak), take the forward near-table cluster past the edge as the hanging paw, report its
position and the error vs the GT frontmost paw. Perception <= GT: this validates the depth path, it does not beat GT."""
import sys, json, zmq, time, numpy as np
from scipy.spatial.transform import Rotation
PORT=sys.argv[1]; PITCH=float(sys.argv[2]) if len(sys.argv)>2 else 0.55
ctx=zmq.Context.instance()
def req(op, want_img=False, timeout=30000, **f):
    s=ctx.socket(zmq.REQ); s.setsockopt(zmq.LINGER,0); s.setsockopt(zmq.RCVTIMEO,timeout); s.setsockopt(zmq.SNDTIMEO,8000)
    s.connect(f"tcp://127.0.0.1:{PORT}"); s.send(json.dumps({'op':op,**f}).encode()); parts=s.recv_multipart(); s.close()
    meta=json.loads(parts[0])
    if want_img:
        h,w=meta['height'],meta['width']
        rgb=np.frombuffer(parts[1],np.uint8).reshape(h,w,3); depth=np.frombuffer(parts[2],np.float32).reshape(h,w)
        return meta,rgb,depth
    return meta
for _ in range(40):
    try:
        if req('state',timeout=3000).get('ok'): break
    except Exception: pass
    time.sleep(1)
# operator overhang: push the dog so a frontmost paw hangs ~6 cm past the front edge
g0=req('gt'); paws0=g0['dog_paws']; frontx=min(p[0] for p in paws0)
tp=g0['table']['pos'] if 'table' in g0 else None; edgex=g0.get('table_edge_x')
if edgex is None and tp is not None: edgex=tp[0]-0.30
if edgex is not None:
    dx=max(-0.25,min(0.0,(edgex-0.06)-frontx)); dp=g0['dog']['pos']
    req('place', body='dog', pos=[dp[0]+dx,dp[1],dp[2]], quat=g0['dog']['quat']); time.sleep(0.6)
req('head', pitch=PITCH); time.sleep(2.0)                      # point the camera at the workspace
meta,rgb,depth=req('camera',want_img=True)
K=np.array(meta['intrinsics']); fx,fy,cx,cy=K[0,0],K[1,1],K[0,2],K[1,2]; H,W=meta['height'],meta['width']
cp=np.array(meta['pose_pelvis']['position_m']); qw,qx,qy,qz=meta['pose_pelvis']['quaternion_wxyz']
Rc=Rotation.from_quat([qx,qy,qz,qw]).as_matrix()
us,vs=np.meshgrid(np.arange(W),np.arange(H)); z=depth; valid=z>0.05
Xc=z; Yc=-(us-cx)/fx*z; Zc=-(vs-cy)/fy*z
pts=(Rc@np.stack([Xc[valid],Yc[valid],Zc[valid]],1).T).T+cp
# table top = densest horizontal z-band among the table region (x in [0.35,0.9])
reg=pts[(pts[:,0]>0.35)&(pts[:,0]<0.9)]
hist,edges=np.histogram(reg[:,2],bins=60); tz=float(edges[hist.argmax()]+ (edges[1]-edges[0])/2)
# hanging paw = forward of the edge (small x), near table height, the min-x cluster
cand=pts[(pts[:,0]>0.12)&(pts[:,0]<0.40)&(pts[:,2]>tz-0.12)&(pts[:,2]<tz+0.06)]
gt=req('gt'); pelp=np.array(gt['pelvis']['pos']); pq=gt['pelvis']['quat']
Rp=Rotation.from_quat([pq[1],pq[2],pq[3],pq[0]]).as_matrix()
near=min(gt['dog_paws'],key=lambda p:p[0]); gt_pel=Rp.T@(np.array(near)-pelp)
if len(cand)>=15:
    order=cand[cand[:,0].argsort()]; tip=order[:max(15,len(order)//8)].mean(0)
    err=float(np.linalg.norm(tip-gt_pel))
    print("PERC "+json.dumps({"n_pts":int(len(cand)),"table_z":round(tz,3),
          "perceived_paw_pel":tip.round(3).tolist(),"gt_paw_pel":gt_pel.round(3).tolist(),
          "localization_err_cm":round(err*100,1)}))
else:
    print("PERC "+json.dumps({"n_pts":int(len(cand)),"table_z":round(tz,3),"result":"TOO_FEW","gt_paw_pel":gt_pel.round(3).tolist()}))
