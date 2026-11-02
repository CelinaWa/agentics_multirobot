"""Chain the cross-body steps (box LEFT, plush RIGHT, right hand). Each step = one function in lr.py / pb3.py taking
(ctx, state). main(ctx, box='x0,x1,y0,y1', steps=...) -> {ok, measured: {step: report}, why}. The box footprint must
come from colour corners checked by the agent (boxdet is ~0.2 m off for a box on the left). Verified only with the agent
looking at the carry/swing frames; chain without looks only after more runs."""
import contextlib, io, json, os, sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import pb3, lr

STEPS = ['prep', ('perceive', 'right'), 'box', 'waist', 'over', 'hover', 'grasp', 'lift', 'carry', 'swing', 'release', 'home']


def main(ctx, box, waist=(0.9, 0.15), grasp_head_shift=0.03, steps=STEPS):
    st = pb3.load(); measured = {}
    for s in steps:
        name, args = (s, ()) if isinstance(s, str) else (s[0], s[1:])
        if name == 'box':
            args = (box,)
        if name == 'waist':
            st = pb3.load(); st['waist_cmd'], st['waist_roll'] = waist
            st['grasp'] = [st['grasp'][0] + 0.015, st['grasp'][1] - grasp_head_shift]   # head-side middle, +1.5 cm for the short landing
            st['release'] = [st['box']['x'][0] + 0.10, st['box']['y'][0] + 0.175, 0.27]; st['press_depth'] = -0.06
            pb3.save(st)
        fn = lr.STEPS.get(name) or pb3.STEPS[name]
        buf = io.StringIO()
        try:
            with contextlib.redirect_stdout(buf):
                fn(ctx, pb3.load(), *args)
            ok = True
        except pb3.Fail:
            ok = False
        rep = json.loads(buf.getvalue().strip().splitlines()[-1])
        measured[name] = rep
        if not ok:
            return {'ok': False, 'measured': measured, 'why': f"{name}: {rep['why']}"}
    return {'ok': True, 'measured': measured, 'why': ''}
