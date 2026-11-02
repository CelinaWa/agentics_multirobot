"""Session 996d97 controller (box RIGHT, puppy centre-left, head LEFT, right hand only), on top of the library
plush-pick-place-box-same-side v4 controller3 primitives. Changes that made run 1 work:
  shift  grasp y moved 3 cm toward the tail after hover (head left -> tail is -y), re-hover at top + 0.06
  grasp2 press like controller3 (stops at the first palm stall; one deeper level only while tilt < 6), close 4 s
  lift2  3 cm / 4 s steps (controller3: 4 cm / 3 s slipped at z 0.115 in attempt 1)
  carry  STALL 1.45 instead of 1.3: a held plush read idx 1.37 / mid 1.31 and still went into the box; empty fist ~1.53
Usage: PYTHONPATH=. python code/puppy_box/run_996.py prep,perceive,hover,shift     -> LOOK at the shift frame
       PYTHONPATH=. python code/puppy_box/run_996.py grasp2,lift2                   -> LOOK at the lifted frame
       PYTHONPATH=. python code/puppy_box/run_996.py carry,release,home             -> confirm placement
Optional env: DY (shift, default -0.03), HEAD (left|right, default left)."""
import os, sys, json, subprocess
ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
LIB = os.path.join(ROOT, 'skill-library/skills/plush-pick-place-box-same-side/code')
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, LIB)


def sh(args):
    env = dict(os.environ, PYTHONPATH=ROOT + os.pathsep + os.environ.get('PYTHONPATH', ''))
    out = subprocess.run([sys.executable] + args, cwd=ROOT, env=env, capture_output=True, text=True, timeout=900)
    last = (out.stdout.strip().splitlines() or ['{}'])[-1]
    try:
        return json.loads(last)
    except Exception:
        return {'ok': False, 'why': f'unparsable output: {last[:200]} {out.stderr[-300:]}'}


def run_step(name):
    head = os.environ.get('HEAD', 'left')
    if name in ('prep', 'perceive', 'hover', 'release', 'home', 'stance'):
        r = sh([os.path.join(LIB, 'run.py'), head, name])
        return r if 'measured' not in r else dict(r['measured'].get(name, {}), ok=r['ok'], why=r.get('why', ''))
    if name == 'shift':
        r = sh([os.path.join(HERE, 'shift.py'), '0', os.environ.get('DY', '-0.03')])
        return r
    if name == 'grasp2':
        return sh([os.path.join(HERE, 'grasp2.py'), '1'])
    if name == 'lift2':
        return sh([os.path.join(HERE, 'lift2.py')])
    if name == 'carry':
        return carry()
    raise ValueError(name)


def carry():
    """controller3 step_carry with STALL 1.45 (a held plush read 1.31-1.37 in run 1; an empty fist ~1.53)."""
    import io, contextlib
    import controller3 as c3
    from api.runtime import Context
    c3.STALL = 1.45
    st = c3.load(); buf = io.StringIO()
    with Context() as ctx, contextlib.redirect_stdout(buf):
        try:
            c3.step_carry(ctx, st)
        except c3.Fail:
            pass
    r = json.loads(buf.getvalue().strip().splitlines()[-1])
    return dict(r.get('measured', {}), ok=r['ok'], why=r['why'])


if __name__ == '__main__':
    steps = sys.argv[1].split(',')
    for s in steps:
        r = run_step(s)
        print(json.dumps({'step': s, **{k: v for k, v in r.items() if k != 'history'}}, default=float), flush=True)
        if not r.get('ok', False):
            print(json.dumps({'stopped_at': s, 'why': r.get('why')}))
            sys.exit(1)
