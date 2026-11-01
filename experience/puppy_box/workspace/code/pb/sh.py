"""Shared helpers: G1 instance, look(), print joints."""
import os, sys
ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
sys.path.insert(0, os.path.join(ROOT, 'skill-library/skills/hand-claw-pregrasp/code'))
sys.path.insert(0, os.path.dirname(__file__))
import g1 as g1mod, perception as P, ik, servo  # noqa
from look import look  # noqa
