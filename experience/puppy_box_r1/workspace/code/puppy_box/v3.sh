#!/bin/bash
# v3 carry test from an operator stance: grasp+lift (lift_only), waist +W, walk .07, relative raise +DZ, head-up look.
W=${1:-0.15}; DZ=${2:-0.04}
cd "$(dirname "$0")/../.."
python code/puppy_box/home.py || exit 1
python code/puppy_box/chain.py look 2>&1 | grep -E '"selected"|STOP' | cut -c1-160 | grep -q STOP && { echo LOOK_STOP; exit 2; }
python code/puppy_box/chain.py pre 2>&1 | grep -E 'REFUSED|STOP' && { echo PRE_FAIL; exit 2; }
python code/puppy_box/chain.py go 0 0.06 -0.004 1 2>&1 | grep -E '"closed"|SLIP|MISS|STOP|lift_only' | cut -c1-200
[ ${PIPESTATUS[0]} -ne 0 ] && { echo GO_FAIL; exit 3; }
python code/puppy_box/carry.py waist $W 3 2>&1 | tail -1 | cut -c1-260 | tee /dev/stderr | grep -q SLIP && exit 3
python code/puppy_box/carry.py walk 0.07 2>&1 | tail -1 | cut -c1-300 | tee /dev/stderr | grep -q SLIP && exit 3
python code/puppy_box/carry.py move 0 0 $DZ 2>&1 | tail -1 | cut -c1-200 | tee /dev/stderr | grep -q SLIP && exit 3
python code/puppy_box/carry.py look 0.3 2>&1 | tail -1 | cut -c1-400
