"""Tilted leg-bundle grasp geometry: fingers point right-down at angle a (yawed forward by yaw), palm normal
left-down; thumb on the bundle's left side, fingers over its right side."""
import numpy as np
from robot import quat_from_axes

def pose(c, a_deg=40, yaw_deg=20, off=(0.065, 0.05)):
    a = np.radians(a_deg); ps = np.radians(yaw_deg)
    h = np.array([np.sin(ps), -np.cos(ps), 0])
    f = np.cos(a) * h + np.array([0, 0, -np.sin(a)])
    n = np.sin(a) * (-h) + np.array([0, 0, -np.cos(a)])
    p = np.asarray(c, float) - off[0] * f - off[1] * n
    return p, quat_from_axes(f, n)
