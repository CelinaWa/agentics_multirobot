"""Waist back to 0, then the pb3 prep pose (left tucked, right hand high palm down)."""
import sys, os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__))); import cr
from api.runtime import Context
with Context() as ctx:
    cr.pb3.c.pps.ramp(ctx, {'waist_yaw_joint': 0.0, 'waist_roll_joint': 0.0}, 3.0)
    cr.pb3.step_prep(ctx, cr.pb3.load())
