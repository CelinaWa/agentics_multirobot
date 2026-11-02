"""Run selected run_bb steps by name: bb_steps.py prep,perceive"""
import sys, json
sys.path.insert(0, 'skill-library/skills/plush-pick-place-box-behind/code')
import run_bb as R
for s in sys.argv[1].split(','):
    if s == 'perceive':
        r = R.perceive()
    elif s == 'prep':
        r = R.sh([R.PBB, 'prep'])
    else:
        raise SystemExit('only prep/perceive here; use run_bb.py --from for the rest')
    print(json.dumps({s: r}, default=float), flush=True)
    if not r.get('ok'):
        break
