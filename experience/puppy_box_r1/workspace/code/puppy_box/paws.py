"""Perceive only (head .6): print edge + every paw (centre, n, overhang) and the selected clean overhanging paw.
    python code/puppy_box/paws.py [tag]"""
import sys, json
sys.path.insert(0, '.'); sys.path.insert(0, 'skill-library/skills/r1-leg-axis-grasp-short-carry/code')
from api.runtime import Context
import chain
chain.ctx = Context()
chain.good = lambda p: p['n'] >= 300 and p['rms'] <= 0.001
o = chain.perceive(sys.argv[1] if len(sys.argv) > 1 else 'paws')
T, R = chain.select(o)
print(json.dumps({'frame': o['frame'], 'edge': o['edge'], 'table_z': o['table_z'], 'tilt': o['tilt'],
                  'paws': [{'c': [round(v, 3) for v in p['c']], 'n': p['n'], 'ovh': p.get('overhang'), 'pix': p['pix']} for p in o['paws']],
                  'T': T and [round(v, 3) for v in T['c']], 'Tovh': T and T.get('overhang')}))
