"""Waypoints for the left-arm claw grasp, checked against the table with FK."""
import json, sys
import numpy as np
sys.path.insert(0, '.')
import ik

L = ['left_%s_joint' % a for a in ik.ARM]
PTS = ('palm', 'thumb_tip', 'index_tip', 'middle_tip', 'hand_center')


def clearance(j, side, table_z, edge_x, frac=0.0):
    """Lowest margin of hand points above the table top where they are over the table."""
    t = ik.tips(j, side, frac)
    worst = 9
    for k, p in t.items():
        if p[0] > edge_x - 0.03:
            worst = min(worst, p[2] - table_z)
    return worst


def check_path(j0, j1, side, table_z, edge_x, n=20):
    worst = 9
    for s in np.linspace(0, 1, n):
        j = {k: j0[k] + (j1[k] - j0[k]) * s for k in j0}
        worst = min(worst, clearance(j, side, table_z, edge_x))
    return worst
