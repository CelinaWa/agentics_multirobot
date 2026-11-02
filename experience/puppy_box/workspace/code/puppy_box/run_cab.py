"""Session cab719 controller: stance step + library plush-pick-place-box-same-side chain.
Usage: PYTHONPATH=. python code/puppy_box/run_cab.py [stance|skill|all] [head]
stance: tuck both arms, measure the table edge (layout2, arms tucked), and while it is > EDGE_OK walk forward (carry=true)
by edge - EDGE_GOAL (+ a few cm: short walks under-run), then g1_crouch(0, forward FWD, arms_home False) to undo the
straightened legs; at most MAX_WALKS walks. skill: the library run.py chain (prep..home) with its lean fallback."""
import sys, os, json, time
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..',
                                'skill-library', 'skills', 'plush-pick-place-box-same-side', 'code'))
import controller as c, layout2  # noqa: E402

EDGE_OK, EDGE_GOAL, MAX_WALKS, FWD = 0.235, 0.20, 3, 0.08


def measure(ctx, tag):
    ob, path = c.observe(ctx, tag)
    lay = layout2.layout(path)
    return lay, ob


def both(d):
    return {**c.mirror('left', d), **c.mirror('right', d)}


def stance(ctx):
    # hands back first, then forearms up, then forward to the tuck: at ~0.22 m from the edge the elbow alone would
    # swing the hands up through the paws lying at the table edge
    c.pps.ramp(ctx, both({'shoulder_pitch': 0.9}), 2.5, hands=(0, 0))
    c.pps.ramp(ctx, both({'elbow': -0.5}), 3.0, hands=(0, 0))
    c.pps.ramp(ctx, both({'shoulder_pitch': 0.45}), 2.5, hands=(0, 0))
    time.sleep(1.0)
    lay, ob = measure(ctx, 'stance0')
    hist = [lay['edge']]
    for _ in range(MAX_WALKS):
        if lay['edge'] <= EDGE_OK:
            break
        if ob['tilt_deg'] > 6:
            return dict(ok=False, why=f"tilt {ob['tilt_deg']}", edges=hist)
        w = ctx.value('g1_walk', forward_m=round(min(0.15, lay['edge'] - EDGE_GOAL + 0.03), 3), carry=True, speed_mps=0.2)
        ctx.value('g1_crouch', pelvis_drop_m=0, forward_rad=FWD, arms_home=False, secs=3)
        time.sleep(1.0)
        lay, ob = measure(ctx, 'stance')
        hist.append(lay['edge'])
        print(json.dumps({'walk_odo': w['transition']['feedback']['odometry_m']['forward'], 'edge': lay['edge']}))
    ok = lay['edge'] <= EDGE_OK + 0.03
    return dict(ok=ok, why='' if ok else f"edge {lay['edge']} after {MAX_WALKS} walks: ask the operator",
                edges=hist, layout=lay, tilt=ob['tilt_deg'])


if __name__ == '__main__':
    from api.runtime import Context
    import importlib.util
    spec = importlib.util.spec_from_file_location('same_side_run', os.path.join(
        os.path.dirname(os.path.abspath(__file__)), '..', '..', 'skill-library', 'skills', 'plush-pick-place-box-same-side', 'code', 'run.py'))
    skill = importlib.util.module_from_spec(spec); spec.loader.exec_module(skill)
    what = sys.argv[1] if len(sys.argv) > 1 else 'all'
    head = sys.argv[2] if len(sys.argv) > 2 else 'left'
    out = {}
    with Context() as ctx:
        if what in ('stance', 'all'):
            out['stance'] = stance(ctx)
            print(json.dumps(out['stance'], default=float))
        if what in ('skill', 'all') and out.get('stance', {'ok': True})['ok']:
            out['skill'] = skill.main(ctx, head=head)
    print(json.dumps(out, default=float))
