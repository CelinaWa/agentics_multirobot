"""Run the library skill r1-leg-lowtilt-lift-advance-carry end to end (look -> grasp -> low-tilt lift -> advance ->
carry/release -> retreat) and save its result.  python code/puppy_box/run_skill.py <tag> [key=value ...]"""
import sys, json, importlib.util, time
sys.path.insert(0, '.')
from api.runtime import Context
spec = importlib.util.spec_from_file_location('skill', 'skill-library/skills/r1-leg-lowtilt-lift-advance-carry/code/run.py')
skill = importlib.util.module_from_spec(spec); spec.loader.exec_module(skill)
if __name__ == '__main__':
    tag = sys.argv[1] if len(sys.argv) > 1 else 'run'
    kw = {}
    for a in sys.argv[2:]:
        k, v = a.split('='); kw[k] = (v == 'True') if v in ('True', 'False') else float(v)
    ctx = Context()
    t0 = time.time()
    res = skill.main(ctx, **kw)
    res['tag'] = tag; res['secs'] = round(time.time() - t0, 1)
    with open('output/skill_runs.jsonl', 'a') as fh:
        fh.write(json.dumps(res, default=str) + '\n')
    print(json.dumps({k: res[k] for k in ('ok', 'why', 'secs')}))
    print(json.dumps(res['measured'].get('lift', {}), default=str)[:400])
    print(json.dumps(res['measured'].get('carry', {}), default=str)[:600])
