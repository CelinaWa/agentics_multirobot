import sys, time, json
sys.path.insert(0, '/home/junjieye/workspace/multirobot/real_g1_release/rax/runtime/sessions/20261105T222431Z_puppy_box_r1_e0ac0c/agent-workspace')
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
