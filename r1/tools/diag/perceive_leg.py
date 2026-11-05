"""Localize the overhanging hind leg from the head-camera DEPTH (the real pipeline), and compare to ground truth.
Back-project depth -> point cloud in the pelvis frame; find the table top; the overhanging leg = dog points past the
front edge and below the table top; report its tip position and the error vs GT frontmost paw."""
import sys, json, zmq, time, numpy as np
PORT = sys.argv[1]
ctx = zmq.Context.instance()
def req(op, want_img=False, timeout=30000, **f):
    s = ctx.socket(zmq.REQ); s.setsockopt(zmq.LINGER,0); s.setsockopt(zmq.RCVTIMEO,timeout); s.setsockopt(zmq.SNDTIMEO,8000)
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
# settle a bit so the scene is at rest (operator has set the overhang in a real run; here we read the layout as-is)
meta,rgb,depth = req('camera', want_img=True)
K=np.array(meta['intrinsics']); fx,fy,cx,cy=K[0,0],K[1,1],K[0,2],K[1,2]
H,W=meta['height'],meta['width']
# camera pose in pelvis frame
cp=np.array(meta['pose_pelvis']['position_m']); qw,qx,qy,qz=meta['pose_pelvis']['quaternion_wxyz']
from scipy.spatial.transform import Rotation
Rc=Rotation.from_quat([qx,qy,qz,qw]).as_matrix()   # camera->pelvis (axes: X fwd, Y left, Z up per the server)
us,vs=np.meshgrid(np.arange(W),np.arange(H))
z=depth  # distance along optical axis (camera X forward)
valid=z>0.05
# camera-frame coords: optical axis = X fwd; pixel (u,v): x_cam=z (fwd), y_cam=-(u-cx)/fx*z (left), z_cam=-(v-cy)/fy*z (up)
Xc=z; Yc=-(us-cx)/fx*z; Zc=-(vs-cy)/fy*z
pts_cam=np.stack([Xc[valid],Yc[valid],Zc[valid]],1)
pts_pel=(Rc@pts_cam.T).T+cp
# table top z = dominant high-density z among table-height points
tz=np.median(pts_pel[(pts_pel[:,2]>0.5)&(pts_pel[:,2]<0.95),2]) if np.any((pts_pel[:,2]>0.5)&(pts_pel[:,2]<0.95)) else 0.75
# overhanging leg: dog points FORWARD (x in reach) and BELOW the table top (hanging past the edge)
fwd=(pts_pel[:,0]>0.15)&(pts_pel[:,0]<0.55)
hang=(pts_pel[:,2]<tz-0.005)&(pts_pel[:,2]>tz-0.14)   # below the top, above the floor region
leg=pts_pel[fwd&hang]
gt=req('gt'); paws=gt['dog_paws']; near=min(paws,key=lambda p:p[0])  # GT frontmost paw (world)
# GT in pelvis frame
pelp=np.array(gt['pelvis']['pos']); 
# approximate: compare in a frame-agnostic way via the x (fwd) + z; report both
if len(leg)>20:
    # leg tip = the most-forward (lowest x) cluster centroid
    order=leg[leg[:,0].argsort()]; tip=order[:max(20,len(order)//10)].mean(0)
    print(json.dumps({"n_leg_pts":int(len(leg)), "perceived_leg_pel":tip.round(3).tolist(), "table_z":round(float(tz),3),
                      "gt_near_paw_world":[round(float(x),3) for x in near]}))
else:
    print(json.dumps({"n_leg_pts":int(len(leg)), "perceived":"TOO_FEW_POINTS", "table_z":round(float(tz),3)}))
req('shutdown')
