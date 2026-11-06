import sys, time, json
sys.path.insert(0, '/home/junjieye/workspace/multirobot/real_g1_release/rax/runtime/sessions/20261105T232821Z_puppy_box_r1_c4f6c3/agent-workspace')
from api.runtime import Context
t0=time.time()
with Context() as ctx:
    while time.time()-t0 < float(sys.argv[1]):
        s=ctx.value('r1_status')
        if s['motion']['armed']:
            print('ARMED after', round(time.time()-t0,1)); break
        time.sleep(3)
    else:
        print('still not armed:', s['motion'])
