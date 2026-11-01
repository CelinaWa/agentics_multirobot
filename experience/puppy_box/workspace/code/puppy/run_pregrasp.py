"""Run the library skill hand-claw-pregrasp on the puppy torso (perception -> target -> pre-grasp)."""
import importlib.util, json, sys, os
ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
os.chdir(ROOT)
sys.path.insert(0, ROOT)
from api.runtime import Context
SK = os.path.join(ROOT, 'skill-library/skills/hand-claw-pregrasp/code')
sys.path.insert(0, SK)
spec = importlib.util.spec_from_file_location("skill", os.path.join(SK, 'run.py'))
skill = importlib.util.module_from_spec(spec); spec.loader.exec_module(skill)
import perception
with Context() as ctx:
    o = ctx.value('g1_observe')
    t, sl = perception.slices(o, ymin=-0.15, ymax=0.35)
    # torso = slices with the widest high extent near the body middle: take the y-range of the 5 highest-count slices
    target, blob = perception.largest_blob(sl)
    print('table', round(t, 3), 'torso target', target, 'frame', o['camera']['frame_id'])
    res = skill.main(ctx, side='left', target=target, clearance=0.10, tilt=0.52, yaw=0.0, lean=float(sys.argv[1]) if len(sys.argv) > 1 else 0.15)
    print(json.dumps(res, indent=1, default=str))
    json.dump(res, open(os.path.join(ROOT, 'output', 'pregrasp_result.json'), 'w'), default=str)
