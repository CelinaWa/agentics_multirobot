"""Perceive and print table edge + paws (no motion). python code/puppy_box/scan.py <tag>"""
import sys; sys.path.insert(0, '.'); sys.path.insert(0, 'code/puppy_box')
import chain
o = chain.perceive(sys.argv[1] if len(sys.argv) > 1 else 'scan')
print('frame', o['frame'], 'edge', o['edge'], 'table_z', round(o['table_z'], 3), 'tilt', o.get('tilt'))
for p in o['paws']:
    print('paw', [round(x, 3) for x in p['c']], 'n', p['n'], 'rms', p['rms'], 'ovh', p.get('overhang'))
T, R = chain.select(o); print('T', T and [round(x, 3) for x in T['c']], 'R', R and [round(x, 3) for x in R['c']])
