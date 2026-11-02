# RUNS (session 20261102T022251Z_puppy_box_810c60, sim)
| # | time | how | layout | outcome | notes |
|---|---|---|---|---|---|
| 1 | 02:30 | by hand | puppy centre-left head left, box right (x.24-.53 y-.125..-.515) | FAIL lift | yaw30 pinch at front body, cmd z .06, grip lost at z .22-.29, puppy back on table |
| 2 | 02:33 | by hand | same, puppy +4 cm y | FAIL carry | deep press (cmd .02), held lift to .27; waist swing .7->.45 during carry dropped it |
| 3 | 02:38 | by hand | reset 1, same layout | SUCCESS | waist cmd .45 fixed, deep press mid-body, arm-only 5 s carry, release over centre; operator confirmed |
| 4 | 02:42 | pb3.py step by step (reset 2) | same layout | FAIL grasp | BUG: home had restored the default flat hand shape, pb3 never set the pinch shape -> fist; press rolled the puppy; retry (re-perceive, override box near wall from earlier view) closed but lost at z .13; puppy turned 90 deg on its back. Fixed: hover sets pinch shape |
| 5 | 02:47 | pb3.py prep+perceive (reset 3) | robot/box ~18 cm farther (box near wall .42, puppy far edge .50) | STOPPED correctly | perceive range check: out of reach; operator asked, did not move anything; no walking; home |
| 6 | 02:50 | pb3.py (reset 4; operator moved robot back: it had drifted 22 cm) | box 8 cm further right (inner wall -.203) | FAIL lift | grasp ok mid-body (idx .99 mid .98); BUG lift first step jumped cmd .033->.133: grip lost at z .17. Also my offline test overwrote pb3_state.json (hover used stale grasp; fixed by hand). Fixed: lift starts from press cmd |
| 7 | 02:52 | pb3.py step by step, same puppy (no reset), lift fix | box inner wall -.203 | SUCCESS | perceive -> box override from a clean waist-right view (near wall .236); hover jump>1 fixed (max_jump 1.6 free space); grasp idx .87 mid .95; lift to .29 held (mid .93); carry 5 steps palm (.351,-.381,.33); release; home. Operator: dog INSIDE the box. WARNING: home last ramp tilt 11.6 deg, track err .96; robot drifted back from table again |

## Session 20261102T025843Z_puppy_box_152e7f (sim) - box RIGHT, puppy centre-left head left, table z -0.052 pelvis
| run | what | result |
|---|---|---|
| 1 | pb3 steps; hover patched to back off grasp (waist 0.55, press only to top-.03) | press shallow (palm top+.017); slid during lift, hung by rear; set down |
| 2 | waist 0.7 (deep press reachable); HEAD_SHIFT .06 | hand ended on rear third after press/close; head hung; set down; reset |
| 3 | HEAD_SHIFT .03; looked at hover/grasp/lift/carry frames | SUCCESS, operator confirmed in box |
| 4 | reset 3, stance further: lean 0.1 + re-perceive; prep/perceive/hover chained, looked at hover + lift | SUCCESS, operator confirmed |
| 5 | reset 4, run_pb3.py chained unattended | opened over the box OK; release raise refused (IK jump 1.05) -> raised by hand, home; SUCCESS, operator confirmed. Fix: max_jump 1.6 |

## Session 20261102T033140Z_puppy_box_221440 (sim) - puppy RIGHT head right, box LEFT, right hand only
| # | time | how | layout | outcome | notes |
|---|---|---|---|---|---|
| 1 | 03:44 | by hand (carry.py/mv.py steps) | puppy mid y ~-0.36, far .41 | FAIL grasp, puppy off edge | waist .55 blind (camera looks left), yaw -30, palm stalled .09 (on top): fist. Reset 1 |
| 2 | 03:48 | by hand | same | FAIL grasp, puppy off edge | + waist roll .15, palm .06, arm deflected to y -.34: fist. Reset 2 |
| 3 | 03:52 | by hand | mid y ~-0.31, far .37 | FAIL grasp | waist .7 roll .2 yaw 0, palm .052 tilt 6.8: fist; puppy pushed right to .58 m. Asked operator: "continue". Reset 3 |
| 4 | 03:56 | by hand | mid y ~-0.30, far .36 | FAIL grasp | rear body (0.28,-0.25), palm .062 tilt 6.6: idx 1.38 -> fist on 4 cm lift. Stopped; home |
| 5 | 04:01 | pb3 steps (prep/hover/grasp/lift/carry/release/home), state by hand | puppy pushed to mid y -0.20, box x .34-.64 y -.03..0.39 | FAIL release | grasp+lift+carry held (mid .8-1.05); palm reached only y .04 (7 cm inside the right wall); plush hung right of the palm and fell beside the box |

## Session 20261102T041502Z_puppy_box_6a0bfc (sim) - puppy ahead head RIGHT, box BEHIND the puppy
| # | time | how | layout | outcome | notes |
|---|---|---|---|---|---|
| 0 | 04:17 | no motion, layout check | edge 0.26, puppy x .38-.51, box x .56-.84 | STOPPED before motion | IK: grasp+release out of reach; asked operator; operator disarmed: robot had slid ("stance slide"), session restart |
| 1 | 04:33-04:43 | by hand + pbb.py steps, lean_release.py | after 3 walks: edge .196, puppy far .38 mid (.30,-.21), box near wall .437 (after lean slide), centre (.57,-.19) | SUCCESS (operator: INSIDE) | grasp idx1.02/mid.91, lift .22, carry palm .453 (short) -> higher .461 -> lean .2 gated release at palm .488; puppy fully inside; home ok tilt .8; left hand tucked (palm y+.23 z+.15), operator did not answer the left-hand question |
| 2 | 04:46-04:50 | reset 1, walk 0.06 (operator), pbb steps | edge .208, near wall .434 after lean, puppy far .386 | FAIL lift, puppy on FLOOR | hover ok; press stalled palm z .017 (2.4 cm ABOVE the top), close: index_0/middle_0 only 0.13/0.15 (fingers flat on the body, only tips hooked) -> passed the <1.3 check; 1 cm test lift ok, grip lost at palm z .19 (fist), puppy fell off the front edge. Fix: reject a close with base joints < 0.4 (weak hook): open, raise, re-grasp |
| 3 | 04:51-04:55 | reset 2, pbb steps | near wall .421, puppy far .381 (lying slightly diagonal), grasp far-0.11 | FAIL lift, puppy on FLOOR | close 1: weak hook (base .42/.27) -> opened, raised, shifted y -2.5 cm, close 2: good wrap (mid .90 idx 1.02, = run 1) but grip lost at palm z .074 on the 2nd-3rd lift step; lowered to z .035, opened, home; puppy ended on the floor anyway. Robot yaw drifted to ~7.7 deg left |

## Session 20261102T045636Z_puppy_box_37fe31 (sim) - box BEHIND the puppy, robot too far from the table; NOT attempted
| # | time | how | layout | outcome | notes |
|---|---|---|---|---|---|
| 0 | 04:57 | start (only g1_hold) | edge 0.32 | puppy on FLOOR before any command | start easing moved the robot back; reset 1 |
| 1 | 04:58-05:08 | arms_down.py, g1_walk 0.12 + 0.15 (carry, operator-approved, max 2), pb3 prep, reach_probe.py | edge 0.28 after both walks (+4 cm, then +0), puppy far .42 head left, box near wall .49-.53, centre ~.62-.66 | STOPPED before grasp: release out of reach | release needs palm ~.575-.59; IK max ~.53 (lean .15, yaw cmd .7), probe refused (IK err 5 cm at .58); rear-grasp/head-down idea also out of reach (grasp .41 misses 1-3 cm). pb3 prep's arm swing bumped the puppy (rotated ~40 deg, nose over the front edge). Ended home: arms down, hands open, puppy on table, box untouched, tilt .8 |

## Session 20261102T062330Z_puppy_box_89afad (sim) - puppy RIGHT (head right), box LEFT, right hand only
| # | time | how | layout | outcome | notes |
|---|---|---|---|---|---|
| 1 | 06:24-06:52 | by hand + pb3 steps + lr.py (waist 1.1/roll .15, via-point, palm-yaw swing 15->75 before release) | edge ~0.2 after 2 walks; box x .28-.585 y .025-.43; puppy far .42 y -.37..-.09 | SUCCESS (operator: dog INSIDE the box) | looked at: carry frame (body on right wall by depth) -> rotated palm; after-release frame (in box); home frame. Overhead-IK-branch incident mid-run, recovered. Left hand never near the puppy (tucked, measured further back) - operator did not answer the left-hand question explicitly |
| 2 | 06:54-06:59 | operator reset; pb3 prep/perceive, lr box (my corners), skill run.py chained over..swing (waist lowered 1.1 -> 0.9 after tilt 6.25 stop), looked at the swing frame, lr release, pb3 home | same layout (puppy far .406) | SUCCESS (operator: dog INSIDE) | swing depth gate said 0.25 (arm occludes the body); my look: body over the interior, head toward the right wall -> released; landed on its back inside |

## Session 20261102T070019Z_puppy_box_170401 (sim) - box RIGHT (x .27-.54, y -.165..-.555), puppy centre-left head left
| run | time | what | result |
|---|---|---|---|
| 1 | 07:00-07:19 | tuck.py; layout2 edge .277; g1_walk .09 carry (odo .069) -> edge .20 but legs straight (table z -.0975, hover: grasp 4-9 cm out of reach at every waist); g1_crouch(0, fwd .05, arms_home False) -> pelvis .80->.78, slid 2.7 cm back; pb3 perceive left / hover (waist .7, press -.045, back .03) / LOOKED: hand near the neck -> shift.py 0 -0.02 / grasp (idx .80 mid .89) / lift (mid 1.13) / carry palm (.367,-.375,.352) / release / home | SUCCESS: operator "dog INSIDE the box"; own frame shows a cream leg inside the box, table empty; tilt 4.1 still, hands open |
| 2 | 07:19-07:28 | reset 1; `python code/puppy_box/run_pb3.py left` chained, no looks (waist .7, grip idx .72 mid .84, carry palm (.356,-.349,.332)) | SUCCESS unattended; operator "INSIDE"; own home frame shows it in the box |
| 3 | 07:29-07:34 | reset 2; library `PYTHONPATH=. python skill-library/skills/plush-pick-place-box-same-side/code/run.py left` (commit f6b13f4), no looks | SUCCESS unattended; operator "INSIDE"; own home frame shows it in the box; carry max tilt 5.8 |

## Session 20261102T085157Z_puppy_box_ce59d5 (sim) - box LEFT-ahead (near wall ~.19-.23), puppy RIGHT beside the hip, head LEFT
| # | time | how | outcome | notes |
|---|---|---|---|---|
| 1 | 09:06-09:11 | code/puppy_box/rl2.py steps (lib pb3/lr), waist yaw 0 roll .2 pitch .12, grasp (.20,-.38) blind (body out of view) | UNKNOWN placement, then FALL in home | grasp held, lift .32, carry to (.333,.064,.315), body over the box interior in frame 0908, opened; then camera view swung; home's last ramp (sp .5 el .6) -> tilt 63, forward fall |

## Session 20261102T091205Z_puppy_box_8cef10 (sim) - box BEHIND puppy (right), puppy head right; NOT solved (0/2)
| Run | Steps | Looked at | Outcome |
|---|---|---|---|
| 1 | tuck, 4 walks (edge .295 -> .24), pbb prep, handout, lean .1, via waypoints, hover yaw .7 (shift y -2 cm after looking), pb3 grasp, lift | hover frame 202611020920274041b9 (hand over rear-mid body) -> shifted toward head | close base mid .69/idx .38 (palm stalled at top+0.04), grip lost at z .24, puppy back on table |
| 2 | re-perceive (waist .3), hover, bb.py grasp_deep (top-0.06 forced), lift | hover 202611020923174454af ok | palm stuck at top+0.017, tilt 6, base .55/.45, lost at z .23; home_safe tilt .8 |
| 3 | reset 1; prep, look (hand right), lean .1, staged waist 0/.35/.7, hover yaw -27, grasp_deep gate top+.005 | hover 20261102093000497214 ok | palm at top, finger base 0/0 (tips on the table): opened, raised; asked operator -> continue |
| 4 | waist -.25 view, yaw -18, palm far-.096, light press (DEPTHS 0,-.015,-.03 GATE .012), lift .27, carry | lift 202611020933553539f5 (held), carry 20261102093434db6f29 | GOOD WRAP 1.00/.92, lift held; lost during the forward carry at palm (.39,-.31,.295) = near wall; puppy on the table in front of the box. home_safe |
| 5 | reset 2; prep, hand aside, PCA yaw -4, lean .1, staged approach, hover, light press, lift .26, carry (.37->.465 at y -.22), lean_release .2, home_safe | carry 20261102094025bf8d71 (body over the box), home 202611020941374c1550 | **SUCCESS** (operator: INSIDE); base 1.03/.89; palm .483 at open; tilt <= 2 |
| 6 | reset 3; run_bb.py prep..hover, looked at hover 2026110209441603b804, then --from grasp | open 20261102094535e69748 (puppy hanging on the robot side) | FAIL: grip weakened in the lift (.65/.60), puppy fell to the floor at the release; robot home |

## Session 20261102T095006Z_puppy_box_cab719 (sim) - box RIGHT (x .23-.54, y -.17..-.56), puppy centre head left
| run | time | what | result |
|---|---|---|---|
| 1 | 09:50-09:55 | tuck, 3 walks + 2 crouch restores (edge .278 -> .221), library same-side run.py steps (prep/perceive/hover; LOOKED hover; grasp/lift; LOOKED lift; carry/release/home) | SUCCESS: operator "INSIDE"; home frame table empty, paws in box; tilt 1.1 |
| 2 | 09:56-09:59 | reset 1; run_cab.py stance (edge .202, no walk) + library chain prep..lift without looks | FAIL: hover over neck (frame 2026110209573592f965), grip lost at z .22, puppy back on table; skill home -> robot self-stepped 0.17 m, disarm/re-arm |
| 3 | 10:00-10:12 | reset 2; in-place turns -14/-7/-12/-15 + crouch restores, tucked-view grasp, inferred box, release (.37,-.40), hover LOOKED 20261102100902f53e4e, lift LOOKED 2026110210095754cb46, carry flag -> finished with mv.move, home_low | SUCCESS: operator "INSIDE"; frame 20261102101146393519 paws in box |
| 4 | 10:13-10:22 | reset 3; library v4 run.py left stance,prep,perceive,hover | STOPPED safely: box out of view (bogus box), hover refused, lean fallback refused, home |
| 5 | 10:23-10:34 | tuck, turn -15 in place (yaw 2.7), library run.py left stance,prep,perceive,hover (stance walk .246 -> .201) | STOPPED safely: perceive took the drawer (left) as the box, hover refused twice (~4 min each); home tilt 6-7 steady |

## Session 0878bd (sim) - box LEFT, puppy RIGHT head LEFT
| run | attempt | result | notes |
|---|---|---|---|
| 1 | 1 | grip lost on carry step 1 | grasp y -0.385 yaw 15 roll 0.25: tilt-limited descent (palm 0.06), one finger held; plush fell on the table |
| 1 | 2 | SUCCESS (operator: INSIDE) | grasp behind neck yaw -36 (perp. to body), roll 0.15, soft descent to stall, carry yaw -36->15; looked at lift/carry/release/home frames |
| 2 | 1-3 | FAIL (stopped safely) | after reset 1; run_cr chain; 3 closes at yaw -12/-30/-10 on a body along y = empty fists, plush pushed toward the edge; home_cr |
| 3 | 1-2 | FAIL (stopped safely) | after reset 2; yaw -40 at the success point (.25,-.35) and on the torso (.25,-.39): empty fists; recovered from the low command (sync to measured, straight up, home_cr); operator informed |

## Session 20261102T111240Z_puppy_box_2a8660 (sim) - puppy RIGHT head right, box LEFT (x .33-.61, y -.03..+.38), right hand
- Run 1 (11:13-11:32): SUCCESS (operator: INSIDE), 4 grasp closes. A1/A2 cross-right-hand v2 pinch on the REAR (perceive cut
  off the head) -> empty fists. A3 mid-body yaw -11.7 waist .7/.15 soft press: held (mid .96) but lost at palm z .18 on a
  4 cm-step lift; puppy rotated diagonal. A4 look+plan PCA centre (.30,-.312) yaw -57.6, palm 3 cm back against the finger dir,
  grasp_soft (stall at palm z .078, middle 1.02 / index fist), 3 cm lift steps to palm z .36 (middle 1.11-1.19), carry z .35
  -> release (.43,.10,.30) (one IK refusal at err .010 -> finished with max_err .02), palm (.417,.076,.30), swing yaw 50/75:
  depth 100 % inside, LOOKED (body over the interior), opened, home_cr. Max tilt 6.3 (pressing), box untouched.
- Run 2 (reset 1, 11:33-11:37): FAIL safe. run_x2a stance,look,plan (box moved to x .26-.56; layout2 box ok-ish), PCA yaw 4.8,
  hover within 0.7 cm, grasp_soft held (1.01/0.97), lift held 0.67-0.84 to palm z .21 then slipped at .24; puppy on its back at
  the front edge -> reset 2. Looked at plan overlay + slip frames.
- Run 3 (reset 2, 11:37-11:39): FAIL safe. GYAW=-45 + BOX colour corners; hover palm 4 cm high (wrist posture), stall at palm z .101
  (top .067) -> empty fist. Stopped for session time; home, puppy on the table, box in place, no falls (max tilt 6.3 all session).
- Run 4 (11:40-11:44): FAIL safe. GYAW=-30, lift held (mid .72-1.07) to palm z .307, grip lost on the 1st carry step (both fist);
  puppy flipped on the table, head over the front edge -> reset 3.
- Run 5 (reset 3, 11:44-11:49): SUCCESS (operator: INSIDE). GYAW=-30, chained look..lift and carry,swing (one-finger hold idx .88-.98).
  After the swing the body stuck out toward the robot (depth x .11-.25, in front of the near wall) -> I did NOT open; palm to
  (.44,.10,.30) yaw 30 swung it over the interior (looked + depth), opened, home. Now step `adjust`.
Session tally: 2 successes / 5 runs, 3 resets, 0 falls, max tilt 6.3, box never touched, left arm always tucked.

# Session 20261102T115009Z_puppy_box_eed312 (sim) - box BEHIND the puppy (head right), right hand only, edge 0.26
- Run 1 (11:53-12:01): run_bb prep/lean/perceive/approach/hover by hand (looked at hover). 2 failed presses (far-0.105), then crest
  grasp (palm .324, base .87/.86) LIFTED to z .27; carry palm .435 (= wall .426); gated lean release opened at .467 -> puppy fell back
  in FRONT of the box (legs up). FAIL (release too close to the wall). Reset 1.
- Run 2 (12:02-12:09, run_eed.py): crest pinch -> fingertips on rim; yaw -40 -> fingers over rim (FK); drag moved puppy right. FAIL, no lift.
- Run 3 (12:10-12:18): 3 closes, finger bases on the rim (FK) -> opened; my joint-space raise dragged the box ~20 cm, home swept it,
  tipped at the front edge. FAIL + INCIDENT (box). Reset 3. Tally 0/3, 1 lift / 9 closes, 0 falls, left arm tucked throughout.
- Run 4 (reset 3, 12:20-12:24): roll close (wall-0.125) then crest grasp (wall-0.10, meas .351): weak close, palm z .079 -> opened,
  raised straight up, home. FAIL, no lift. End: robot home, box in place, puppy on table. Session 0/4.

## Session 20261102T122445Z_puppy_box_ad5d0f (sim) - box BEHIND, puppy head right; 0/1 run (2 closes), no resets
- Run 1 (by hand, run_eed steps + code/pb_ad5/*.py): stance stuck at 0.265 (walks advance ~0), crouch fwd 0.1/0.2 -> 0.228;
  close 1 (palm z .076, light press, yaw cmd -65) empty; close 2 (palm z .053, deeper press, -0.05 IK-refused) empty.
  Looked at: hover frames (yaw 0 fingers over the wall -> changed yaw), grasp frame (fingers hidden by the arm -> FK + 3 cm raise test),
  raise frames (puppy not lifted). Decided to stop for the time budget; home_safe; box untouched, puppy on the table.
- Attempt 3 (puppy shoved closer, pinch yaw -65, deep press): empty. Attempt 4 (yaw -10, stale far side): fingers at the wall, open+raise.
- Reset 1. Attempt 5 = run 1 by hand: across-body light press -> close base 0/0 -> CLOSED raise rolled the plush in -> lift .25 -> carry .459
  -> lean_release.py opened at .481 -> puppy on the FLOOR (operator). Looked at hover, raise, lift, carry frames; open frame showed nothing.
- Reset 2. Run 2 (code/pb_ad5/run_ad5.py prep..lift): grip lost in the lift; plush back on the table; box nudged ~3 cm.
  Run 2b without reset: plush now far .313 -> hover refused / palm-down not trackable near the body -> home.
- Reset 3. Run 3 (run_ad5.py all steps, looked at hover / lift / open frames): INSIDE (operator). Release with release_up.py (flat hand,
  wait 2 s, straight up). End: home_safe, tilt 2.8, hands open.

## Session 996d97 (2026-11-02, sim) - puppy centre-left head LEFT, box RIGHT (x .25-.52, y -.18..-.57); edge .19 after stance
- Run 1 attempt 1 (13:06): library same-side v4 stance,prep,perceive,hover (waist .7) -> LOOKED, shift y -0.025 -> grasp: stall palm z .045
  vs depth top .025 (press cmd -0.005, tilt 6.2), base idx .80 mid .92 -> lift: index slipped at z .115, grip lost z .157; puppy back on table.
- Run 1 attempt 2 (13:08-13:11): opened/raised, perceive+hover again (waist .7), shift y -0.03 (LOOKED), code/puppy_box/grasp2.py (palm stalled .045
  again, tilt 6.7, base .79/.83), code/puppy_box/lift2.py (3 cm / 4 s) HELD to z .28 (mid base .74, idx 1.38), carry: grip flag fired at
  idx 1.37/mid 1.31 with palm (.35,-.32,.32) -> finished with one move to release (.335,-.399,.35 measured), release, home (tilt 4.7).
  Operator: INSIDE. SUCCESS. Frames: hover 20261102130753a1af71, grasp 20261102130828701339, lifted 20261102130935605259, home 202611021311027bc84c.
- Run 2 (reset 1, 13:12-13:16): code/puppy_box/run_996.py prep,perceive,hover,shift (LOOKED: mid-body) -> grasp2,lift2 (LOOKED: lifted) ->
  carry,release,home chained. far edge read .397 (robot ~4 cm further back than run 1), hover backed grasp to x .297; press stall palm .038
  tilt 6.85, base .88/.90; lift held by the middle finger only (1.13; index 1.54); carry palm (.33,-.41,.34) inside; home tilt 5.1.
  Operator: INSIDE. SUCCESS. Frames: shift 20261102131334a77edf, lifted 20261102131451211006, home 2026110213160186122e.
- Run 3 (reset 2, 13:16-13:24): full chain stopped at grasp2 tilt 7.83 (far .408; robot drifting back) -> raised open hand, home, stance
  (edge .222, no walk), g1_walk 0.05 carry (odometry -0.006!), crouch(0, fwd .1, arms_home False). Chain again: stopped at grasp2 (extra
  level refused by IK) -> patched grasp2 (close at the stalled level), `grasp2.py close -0.009` (base .82/.84), then lift2,carry,release,home
  chained, carry palm (.319,-.412,.325). Operator: INSIDE. SUCCESS. Tally 3/3 runs placed (4 grasp closes, 1 slip).
- Run 4 (reset 3, 13:26-13:31): library run_v5.py full chain, unattended -> perceive garbage (puppy y .125-.168 only: raised right arm hid it;
  box x .87: out of view, heading ~10 deg left) -> hover refused safely (no contact). home: tilt 6.75 flagged (steady 6.5 standing).
  Stopped here (30 min left; re-squaring needs a turn in place = 0.1-0.2 m step toward the table, not allowed). Puppy on the table, untouched.
