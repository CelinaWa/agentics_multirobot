"""Unattended chain of pb3 steps: prep, perceive, hover (on no reach: lean 0.1, perceive, hover), grasp, lift, carry,
release, home. Stops at the first failed check and prints every step's report. python code/puppy_box/run_pb3.py [head]"""
import io, json, os, sys, contextlib
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import pb3


def run(ctx, name, st, *a):
    buf = io.StringIO()
    ok = True
    try:
        with contextlib.redirect_stdout(buf):
            pb3.STEPS[name](ctx, st, *a)
    except pb3.Fail:
        ok = False
    line = buf.getvalue().strip().splitlines()[-1]
    print(name, line[:400], flush=True)
    return ok, json.loads(line)


if __name__ == '__main__':
    head = sys.argv[1] if len(sys.argv) > 1 else 'left'
    from api.runtime import Context
    with Context() as ctx:
        st = {}
        for name, a in (('prep', ()), ('perceive', (head,)), ('hover', ())):
            ok, r = run(ctx, name, st, *a)
            if not ok and name == 'hover' and 'reach' in r['why']:
                for n2, a2 in (('lean', (0.1,)), ('perceive', (head,)), ('hover', ())):
                    ok, r = run(ctx, n2, st, *a2)
                    if not ok:
                        break
            if not ok:
                sys.exit(1)
        for name in ('grasp', 'lift', 'carry', 'release', 'home'):
            ok, r = run(ctx, name, st)
            if not ok:
                sys.exit(1)
        print('DONE', flush=True)
