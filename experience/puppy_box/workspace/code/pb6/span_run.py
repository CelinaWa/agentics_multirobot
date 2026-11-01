import importlib.util, json, sys
sys.path.insert(0, '.')
from api.runtime import Context
spec = importlib.util.spec_from_file_location("skill", "skill-library/skills/place-reach-check/code/run.py")
skill = importlib.util.module_from_spec(spec); spec.loader.exec_module(skill)
ctx = Context()
res = {}
for side, g in (('left', (0.47, 0.30, 0.08)), ('right', (0.47, 0.30, 0.08))):
    r = skill.main(ctx, side=side, target=(0.41, -0.36, 0.25), grasp_target=g, log=lambda *a: None)
    res[side] = {'ok': r['ok'], 'why': r['why'], 'grasp_best': r['measured']['span']['grasp']['best'],
                 'place_best': r['measured']['span']['place']['best'], 'frame': r['measured']['frame_id']}
print(json.dumps(res))
