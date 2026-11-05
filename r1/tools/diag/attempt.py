"""One grasp+place attempt via the live server protocol (ZMQ). method=flat|edge. Prints IN_BOX true/false."""
import sys, json, zmq, time
PORT, METHOD = sys.argv[1], sys.argv[2]; EA = float(sys.argv[3]) if len(sys.argv) > 3 else 0.28
LEAN = int(sys.argv[4]) if len(sys.argv) > 4 else 0
ctx = zmq.Context.instance()
def req(op, timeout=130000, **f):
    s = ctx.socket(zmq.REQ); s.setsockopt(zmq.LINGER, 0); s.setsockopt(zmq.RCVTIMEO, timeout); s.setsockopt(zmq.SNDTIMEO, 8000)
    s.connect(f"tcp://127.0.0.1:{PORT}"); s.send(json.dumps({'op': op, **f}).encode()); p = s.recv_multipart(); s.close()
    return json.loads(p[0])
for _ in range(40):
    try:
        if req('state', timeout=3000).get('ok'): break
    except Exception: pass
    time.sleep(1)
if METHOD == "flat":
    req('walk', forward_m=0.12, turn_deg=0.0)
    g = req('gt'); near = min(g['dog_paws'], key=lambda p: p[0]); pelx = g['pelvis']['pos'][0]
    req('slide_table', dx=-(near[0] - pelx - 0.27)); time.sleep(3.0)
    gr = req('grasp_paw')
else:
    gr = req('edge_grasp', reach_ahead=EA, do_overhang=True, lean=LEAN)
side = gr.get('side', 'right')
if not gr.get('held'):
    print(json.dumps({"method": METHOD, "EA": EA, "held": False, "IN_BOX": False})); req('shutdown'); sys.exit(0)
g2 = req('gt'); hand = g2.get(f'{side}_grasp') or g2['right_grasp']; boxz = g2['box']['pos'][2]; bq = g2['box_quat']
req('place', body='box', pos=[hand[0], hand[1], boxz], quat=bq); time.sleep(0.6)
pl = req('place_in_box', side=side)
print(json.dumps({"method": METHOD, "EA": EA, "held": True, "IN_BOX": bool(pl.get('in_box'))})); req('shutdown')
