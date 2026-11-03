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

## Session 20261102T133348Z_puppy_box_f0a010 (sim) - box directly BEHIND the puppy (head right), gap 3.5 cm, near wall 0.288 (slanted .262@y-.40 -> .300@y-.025)
- Stance (library same-side run_v5 stance): edges .296/.282 -> .193 after 2 walks (0.126, 0.112 m). Yaw 5.7 deg.
- Run 1 attempt 1 (13:35-13:41): run_ad5 prep, perceive (lean .1): far .253, mid y -.145 (I overrode to -.185: depth rows show body
  -.27..-.07), wall .288. run_ad5 hover refused (waypoint x .127 z .28 IK err .037) -> own via.py at waist 0 to palm (.161,-.163,.137)
  (LOOKED). Pinch shape, grasp_deep light press: palm stalled z .063 (top .038), palm drifted to x .18, base .45/.44; closed raise base
  .65/.65; lift: grip lost at z .21 (held ~.09-.17). INCIDENT: box and puppy dragged ~9 cm toward the robot (fingertips ~x .29 = wall;
  I skipped the finger-FK-vs-wall check). Home_safe, reset 1.
- After reset 1 (13:43): the puppy was on the FLOOR (operator) before I touched it: the reset put it back at its start pose right
  in front of a robot that stood only ~0.09-0.10 m from the edge (box label x .499 at start -> .293 now = 0.206 m advance; the
  stance step's 0.193 edge was the bottom of the camera view, not the edge), and prep_side's arm motion presumably swept it. Reset 2.
- Run 2 (reset 2, 13:45-13:55, code/f0a010/ctl.py steps + 2 by-hand moves): perceive (far .257, top .047, wall .288 given, gy -.15
  set by hand: head hidden behind the tucked right hand) -> hover (FK-path-checked; high side point, over at z+.22, z+.13; LOOKED)
  -> grasp (cmd x .135 = 1 cm back so finger links stay <= wall-.007; palm stalled z .063, links .270-.274; close: base 0/.36;
  closed raise -> .56/.53) LOOKED -> lift 3 cm/4 s to cmd .24 (palm .208, mid base .81) LOOKED -> straight-up past .24 IK-refused
  at x .136 -> diagonal (.17,.27),(.20,.30),(.22,.33) held, LOOKED: paws lowest at z .215, 11 cm over the rim -> carry 4 cm/5 s
  at z .33 to palm (.418,-.145) -> depth: paws x .36 (7 cm past the wall) -> open flat (raise after open IK-refused, harmless) ->
  LOOKED: puppy lying inside -> home_safe + shoulders .6. Operator: INSIDE. SUCCESS. Tilt <= 6.2 (press), left palm y +.17 z +.12..+.14
  throughout. Frames: hover 20261102134903e37552, grasp 20261102135006f07491, lifted 20261102135039c6f39a, high 202611021352202de3a9,
  carry 20261102135309aa64c8, open 20261102135340f7c5a0, home 202611021354071069ad.
- Run 3 (reset 3, 13:55-14:02, ctl.py steps): perceive (far .255) -> hover via IK waypoints left the arm 0.6-0.9 rad off its command
  (palm 6-9 cm off; no contact, LOOKED) -> synced arm to measured -> g1_free_space_move to (.15,-.15,.20) (palm .151,-.151,.197) ->
  grasp (cmd x .139; links <= .276; close base .77/.75; raise .72/.76) -> lift with diagonal rise to (.20, .343) (LOOKED) -> carry to .414
  (paws x .39, LOOKED) -> open (rear tilted up near the left wall, settled) -> operator INSIDE. SUCCESS. Home + shoulders .6, tilt 2.6.
  Tally: 2/3 runs placed (runs 2-3), 3 grasp closes, 0 falls, 1 box-drag incident (run 1), resets used 3 of 5.

## Session 20261102T140446Z_puppy_box_e578ed (sim) - box BEHIND, puppy head right, right hand only
| # | approach | looked at | result |
|---|---|---|---|
| 1 | run_f0 grasp (fingers fwd) at waist cmd 0.7 | hover, grasp, lift frames | lifted to z .40 (IK branch flip), lost on carry step 1; table |
| 2 | ctl.py jump-limited, diagonal lift | hover, grasp, lift | slipped z .13; FLOOR -> reset 1/2 |
| 3 | yaw +30, gate top+0, diagonal lift | hover, grasp | slipped z .14; table |
| 4 | yaw +30, straight 2 cm lift | hover | slipped z .10; table |
| 5 | yaw 30 + palm 2 cm fwd + measured-palm raise | hover | slipped first step; asked operator (continue) -> reset 3 |
| 6 | fingers fwd + gate top+0 + measured-palm raise + straight lift + carry z .24 + nudges | hover, lift, carry (paws by depth vs wall), release, home | SUCCESS, operator INSIDE; end pose tucked, tilt 1.05 |
| 7 | repeat test of #6 recipe after reset 4 (perceive occluded -> reset-1/3 numbers) | hover, lift | slipped z .12 (press stalled at .047 above top .033); puppy on table; home_safe, tilt 0.8 |

## Session 20261102T144058Z_puppy_box_4ddb74 (sim) run 1: SUCCESS (operator: INSIDE)
Box directly behind, puppy head left at the table edge, gap 2.4 cm. walkpose -> walks (.29 -> .178) -> perceive (GY -0.11 override) -> prep.py
(FK-checked) -> hover2 -> LOOK (hand over mid/rear body) -> grasp (press cmd .225; stopped on finger-wall gate, closed anyway: base .79/.91)
-> lift z .33 (held, middle 1.10) -> carry stopped at IK (palm .375) -> release_up lean .2 (palm .465, opened) -> asked operator: INSIDE ->
home_safe (tilt 2.2, hands open). No reset used, box untouched, no falls, left hand tucked throughout (left palm x .08 y .21-.23 z .13-.14).

## Session 20261102T145458Z_puppy_box_258bd8 (sim) - box LEFT ahead (rotated 12 deg), puppy RIGHT head LEFT; NOT solved (0/5 closes lifted past z .12)
Controller: code/puppy_box/run_x2a.py (library cross-right-hand v4 + env WAIST/GC/LIFT_STEP/LIFT_SECS, step lean) + code/p258/*.py.
Stance: 4 walks did not advance (edge .27 at y 0, .22 at y -.2). Box from colour corners (code/p258/corners.py). TABLE_Z=-0.0375 needed.
- A1 waist .7/.15, PCA centre y -.338 (front legs), yaw -30: palm stalled .088 (cmd .045), empty fist; puppy pushed 6 cm to the edge. Reset 1.
- A2 waist .5/.25, mid-body (.29,-.39), yaw -30, sink 2 cm: closed .80/.90 -> lift 3 cm/3 s held to palm .12, fist at .15-.18; puppy flipped on its back.
- A3 flipped belly, yaw -55 (-69: no IK at z .30): middle .95 / index fist -> lost at first lift step. -> request_operator (3 approaches): "continue".
- A4 reset 2, as A2 with sink 3 cm, lift 2 cm/4 s: middle .46 / index fist -> lost at first lift step; puppy pushed 8 cm away. Reset 3.
- A5 as A2 + lean .1 (own ramp, tilt 1.4): hover 1 cm high (was 2); stall fired at palm .107 -> middle .39 / index fist; re-press open to cmd -.01
  (palm .066) + close -> empty fist (puppy moved). Stopped: home (arms tucked, hands open, tilt .6), puppy on the table (pushed back), box untouched.
Frames: A2 lift-fail 20261102151115b8b669, flipped look 20261102151203b8a070, final home 20261102152257667090.

## Session 20261102T152531Z_puppy_box_da90c5 (sim) - box RIGHT, puppy centre-left head LEFT, far stance .24-.25, low table (z -.07)
| # | approach | looked at | result |
|---|---|---|---|
| 1 | code/da90/ctl.py restore,prep / waist / hover (DY -.03 after look) / grasp / raise / lift + run_v5 carry,release,home | perceive overlay, hover x2, grasp, lift, home | SUCCESS, operator INSIDE; tilt <= 5.9, box untouched, no reset |
| 2 | same, after reset 1; robot had drifted 2 cm back (far .453), waist .9/.15 tilt 7.2-7.6 | hover, grasp x2 | 2 empty fists (index at fist 1.53 then press stopped on tilt 7.16 at top+.045); presses rotated the puppy -> reset 2 |
| 2b | regain.py: hand up, waist 0, crouch fwd .2 (box near wall .302 -> .279) | perceive | puppy rotated (far .525): no grasp, reset |
| 3 | ctl.py prep,waist(WAIST_YAW=0.7 -> tilt 5.4),hover DY -.03 | hover, lift, home | SUCCESS, operator INSIDE; bases .84/.80, lift .292 (middle 1.11) |
Tally: 2 placements / 3 runs, 5 closes (3 held), 2 resets, 0 falls, box untouched, left hand tucked throughout.

## Session 20261102T155840Z_puppy_box_3a46e1 (sim) - NEW: operator slides the table to ~0.20 m on request; unified code/puppy_box/run.py
| # | layout | steps / where I looked | result |
|---|---|---|---|
| 1 | box RIGHT (rim x .228-.517, y -.218..-.608 clipped->extended), puppy centre-left head left (paw-pair mid y .045, far .398, top .044), edge .203 | run.py scene (overlay) -> approach (LOOK hover: over chest) -> shift 0 -0.02 -> grasp (LOOK: bases .79/.93) -> lift (LOOK: hangs mid-body, z .27) -> carry (palm .342,-.414,.323; camera turned, no body view) -> release,home,verify (LOOK: in box) | SUCCESS, operator INSIDE; waist .45/.15, tilt <= 4, box untouched, left tucked throughout |
| 2 | reset 1: box RIGHT (x .232-.522, y -.168..-.558), puppy head left (detected via nose blob), mid y .034 | scene (overlay), approach (LOOK: mid-body, no shift), grasp,lift chained (LOOK lift: head ~1-2 cm from the tucked LEFT fingertips at waist .3!), carry (LOOK: 61 % over interior), release,home,verify | operator INSIDE; left-hand contact UNKNOWN (operator did not answer) -> fix: LEFT_GUARD sp .9/el -.9 after the waist turn + lift drift toward the box |
| 3 | reset 2: box LEFT (x .229-.536, y -.072..+.324), puppy RIGHT head left (nose at the box's right wall), body y -.42..-.10; tucked hands hid it -> new LOOK pose (arms down) + side view (waist cmd -0.7, meas -0.26) merged | scene x3 (overlay + side view), approach (hover blind: camera turned left; palm err 3 mm), grasp (bases .83/.81), lift (LOOK: paws hanging; middle 1.12 holds, index fist), carry (LOOK: body over interior 75 %), release,home,verify (LOOK: sitting in box) | SUCCESS, operator INSIDE; waist .3/0, carry 44 cm cross-body arm-only, palm reached (.384,.115,.303); tilt <= 4 |
| 4 | reset 3: box LEFT (x .232-.541, y -.023..+.363), puppy RIGHT diagonal (axis 13.6 deg), head left (eye blob), paws over the front edge (centre .088 inside: allowed after rule change) | scene x2 (operator did not move it), approach (blind hover, err 1.7 cm), grasp,lift (LOOK: held, middle .65), carry STOPPED at step 3: IK jump 1.62 refused; cmd arm on a weird branch (sp .88 roll -2.07) vs measured (sp -1.36 roll -1.58); manual sync to measured (BUG: synced the right LEG too, restored after), carry finished, palm (.374,.097,.275) | FAIL: at the open the plush stayed draped over the hand, release raise + home dragged it, box TIPPED over; puppy ended near the robot; reset 4 |
| 5 | reset 4: box LEFT (x .245-.557, y -.023..+.365), puppy RIGHT diagonal head left (same family as 4) | approach (arm dev <= .45, no sync), grasp (.83/.83), lift SLIPPED at z .10 (drift toward the box started after 5 cm) -> abort (clean) -> scene again (puppy rotated a bit) -> approach,grasp,lift chained (held, middle .67) -> carry (LOOK 63 % over interior) -> release (hang check 211 px = dropped) -> home,verify (LOOK lying in box) | SUCCESS (2nd attempt), operator INSIDE; lift drift now starts after 12 cm |
| 6 | reset 5: box BEHIND (near wall x .348, y -.38..+.03), puppy in front (far .33 = gap ~2 cm), head left | scene (LOOK ok), approach (LOOK: over the rear half), grasp -> EMPTY (finger-wall check broke the press at the 1st level: links .344 > wall-.007) -> abort; fix: step the palm back 1 cm per check, palm wall-.135; scene, approach (LOOK rear half again -> shift +.02 toward the head), grasp (.76/.76) -> lift SLIPPED at z .13 -> abort; puppy now at the front edge (mid-body -.017 inside) -> scene refuses | FAIL (box-behind grasp: 1 empty, 1 slip); added EXTRA_PRESS=1 (one level past the stall); reset 6 |
| 7 | reset 6: box RIGHT (near wall x .197 = closer than the puppy, y -.238..-.628), puppy centre head RIGHT (toward the box), LEGS AWAY from the robot | scene (LOOK ok), approach (LOOK: rear half -> shift -.02 toward the head), grasp (.77/.92, press to top-.045, palm .049) -> lift held to z .18 then SLIPPED at .216 (first drift step) -> abort, scene: puppy on its back, mid-body .059 inside the edge -> stop | FAIL; drift removed; raise+lift now from the MEASURED palm xy after the close (press pushed the palm 3 cm past its command) |
| 8 | reset 7: box RIGHT (near wall .187, y -.183..-.573), puppy centre head left | scene, approach (LOOK: mid/rear, ok), pick (grasp .77/.93, lift from measured palm held to .288, middle .82), carry (LOOK: head near the left rim), place (hang check 0 px) | SUCCESS, operator INSIDE (verify count false-negative: sitting head outside footprint -> loosened) |
| 9 (post-eval, v9) | box RIGHT (x .26-.549, y -.196..-.586), puppy head LEFT far back (grasp x .367), edge .233, waist .7/.15 | hover LOOK ok, pick ok (.70/.83), carry refused (IK jump 3.44, shoulder roll cmd -2.19 vs meas -1.57) -> sync_arm -> carry ok, palm (.406,-.341) = 14.5 cm inside the left wall (cmd -.391, lag 5 cm), place: hang 0 px | FAIL, operator "on the table": tipped out over the head-side wall again -> v10: nudge until the MEASURED palm is >= .18 from the head-side wall (untested) |
| 10 (post-eval, v10) | box BEHIND (near wall .411), puppy head left LEGS TOWARD the robot, edge .236 -> .254 | scene: rank search chose waist .9 (tilt risk) -> fix: behind = smallest waist (.45/.15, release wall+.09); approach stopped (hover 3.6 cm off) -> LOOK: over the chest -> shift 0 -.03; pick: close .77/1.0, lift held to z .177 then lost at .195; retry: scene refused (mid-body .014 inside the edge, puppy rotated) | FAIL (slip; first legs-toward slip since run 5: legs-toward lifts now 10/12). Robot home, puppy on the table |

## Session 20261102T175700Z_puppy_box_d65273 (sim) - run.py v10.1 -> v11
| # | layout | steps / where I looked | result |
|---|---|---|---|
| 1 | box BEHIND (near wall .384, y -.307..+.091), puppy in front head LEFT legs toward the robot (far .339 = gap 4.5 cm, top .059), edge .180 after the table slide | scene (overlay: grasp over the rear -> TAIL_SHIFT set 0, grasp y -.137), approach waist .45/0 (LOOK hover: just behind the front legs, palm 1 cm toward the head) -> shift 0 -.015, grasp (.77/.89, tips .365 < wall), lift to .295 (LOOK; left clearance 5 cm by sat>60 mask), carry to palm .465 (wall+.081, 79 % over the interior, LOOK), release (raise IK-refused -> check counted nothing: fixed v11), home, verify | SUCCESS, operator INSIDE (on its back, head over the left rim); tilt <= 4.5 |

## Session 20261102T175700Z_puppy_box_d65273 (sim) - run.py v10.1 -> v11
| # | layout | steps / where I looked | result |
|---|---|---|---|
| 1 | box BEHIND (near wall .384, y -.307..+.091), puppy in front head LEFT legs toward the robot (far .339 = gap 4.5 cm, top .059), edge .180 after the table slide | scene (overlay: grasp over the rear -> TAIL_SHIFT set 0, grasp y -.137), approach waist .45/0 (LOOK hover: just behind the front legs, palm 1 cm toward the head) -> shift 0 -.015, grasp (.77/.89, tips .365 < wall), lift to .295 (LOOK; left clearance 5 cm by sat>60 mask), carry to palm .465 (wall+.081, 79 % over the interior, LOOK), release (raise IK-refused -> check counted nothing: fixed v11), home, verify | SUCCESS, operator INSIDE (on its back, head over the left rim); tilt <= 4.5 |
| 2 | reset 1: box ON ITS SIDE at the front-left edge (depth: vertical panel x .21, 28 cm tall), puppy RIGHT at the front edge (mid-body .023 inside), head left | scene: puppy blob = the box, box = far table band -> refused (box size, edge check); the side look (waist .7 -> .28 measured) with the arms in LOOK (sp .9 el 0) swept the right fingertips to x .20 under the edge -> puppy on the FLOOR (operator) | FAIL before contact (perception refused correctly; the look motion knocked the puppy off). Fix v11.1: LOOK el .6 (hands x <= .05 at any waist) |
| 3 | reset 2: box LEFT (x .222-.515, y .011-.401), puppy RIGHT head LEFT legs toward the robot (centre .321,-.276, far .385, top .075), edge .181 | scene ok (waist .9/.15 chosen for the centre release), approach: blind hover (camera on the box), palm 4.6 cm left/2.4 short -> shift .024 -.046 (palm then .267,-.280), grasp: press tracked the shifted command to y -.326 (rear), EMPTY fist (index 1.53) -> abort/home; re-scene: BOX GONE from the table, puppy moved 20 cm right/away | FAIL + INCIDENT: waist cmd .9 oscillated (meas .27<->.46, palm swinging 15 cm); box left its place between 18:10:11 and 18:10:29 (frames), mechanism not seen (FK: no hand link in the footprint). Fix v11.2: waist <= .7; shift compensation does not hold at press height (use measured palm per level) |
| 4 | reset 3: box LEFT (x .237-.532, y -.006..+.376), puppy RIGHT head LEFT legs toward (centre .344,-.284, far .402, top .074), edge .199 | scene ok (waist .7/.15, centre release), approach v12 auto-correction (.016,-.017) -> palm 1.1 cm off (blind hover), grasp: palm stalled at .09 (top+.016) for 3 levels, tilt 3-4; FK: thumb 3.5 cm into the plush, index/middle tips 6 cm ABOVE the body past the far side -> EMPTY fist | FAIL (empty). Run 1 for comparison: thumb pressed to 2.3 cm above the table, palm top-.007. Fix v12.1: press on while palm > top and tilt < 5.5 |
| 5 | same layout after run 4's press rotated the puppy (axis -45..-51, centre .386,-.335, head left), edge .204 | run 5a: waist .3/0, palm far-.10: press IK-refused below cmd .044, palm .095, EMPTY fist (abort, scene). run 5b: GRASP_BACK .075 (param), waist .45/.15, blind hover err 1 cm, grasp v12.1 pressed to palm .063 (top .075), held .66/.62, lift .322 (middle 1.07, index fist), carry to palm (.387,.097) (LOOK: held near the shoulders, body hanging toward the right wall, frac .45) -> manual nudge.py (.40,.125,.30): grip let go, fell INSIDE; release/home/verify | SUCCESS (operator INSIDE), with one manual nudge -> v13 automates the centre nudge |
| 6 | reset 4: box RIGHT (x .204-.497, y -.541..-.151); the reset laid the puppy at the front edge BETWEEN the tucked hands (frame 182823) | scene: tuck -> look move dragged the puppy off the table (frames 182823-182830), then no puppy in view (new: right/left side looks) -> operator: FLOOR | FAIL before grasp. Fix v14: home ends in the look pose; scene stops if plush < 6 cm from a hand (offline: 1.7 cm on 182823) |
| 7 | reset 5: box RIGHT (x .273-.569, y -.54..-.15), puppy centre-left head LEFT legs toward (centre .322,.047, top .068), edge .200 | v14: scene (no arm move needed), approach waist .45/0 (auto corr -.011,-.015; LOOK: over the chest) -> shift 0 -.02, grasp (pressed to palm .069, top .068; .87/.96 -> .61/.76), lift .317 (middle 1.11; LOOK hanging), carry to palm (.403,-.329) = box centre (camera turned: body out of view, frac meaningless -> v14.1 None), release (0 px), home (look pose), verify 18801 in / 132 out | SUCCESS, operator INSIDE; no manual moves except the LOOK shift |
| 8 | reset 6: box RIGHT (x .267-.563, y -.537..-.147), puppy centre-left head LEFT (centre .315,.067), edge .197 | scene, approach (auto corr; LOOK: chest) -> shift 0 -.015, grasp,lift,carry chained (.62/.63; lift .315 middle 1.09; carry to (.408,-.329), frac None = out of view), place | SUCCESS, operator INSIDE |

### Post-eval (v15/v16)
| # | layout | steps | result |
|---|---|---|---|
| 9 | box LEFT, puppy RIGHT head left, mid y -.288 (then -.31 diagonal) | v15: 2 empty fists (palm stalled at top+.003 / top+.016 at waist .6/.15), aborts; GYAW=-45 experiment scene -> puppy rolled to the edge, not tried | FAIL: lateral cross-body grasp is the blocker (this session 1 hold in 6 closes at mid y <= -.27) |
| 10 | box RIGHT (x .275-.571), puppy centre-left head LEFT (mid y .08), waist .6/.15 | v15: shift -.015, grasp .72/.84, lift .309, carry to the box centre, release 0 px (plush inside) -> HOME swept the hand down (palm z .46 -> .17, index (.42,-.09,.07) beside the box's left wall) and dragged the box with the plush off the table | FAIL + INCIDENT (box and puppy on the floor). Fix v16: Cartesian retreat (up to z .40, back to x .24) at the current waist before waist 0; offline FK on runs 10 / eval 8: >= 13 cm above the rim |
| 11 | box LEFT (x .245-.537, y .00-.38), puppy RIGHT head LEFT legs toward, mid y -.32, waist .45/0 | v16 + GYAW=-45 (env): grasp .77/.72 (press stalled at top+.014 but held), lift .30 (index .76, left clearance 4.9 cm), carry to (.376,.118) = near wall + .131, frac .70 (LOOK: body across the box, head over the left wall), release 32 px, home with the new retreat (box stayed) | SUCCESS, operator INSIDE -> v17 makes yaw -45 the default for box-left grasps beyond y -.25 |
| 12 | box LEFT (x .255-.547, y -.01..+.38), puppy RIGHT head LEFT, mid y -.31, waist .7/.15 | v17 (yaw -45 auto): hover corr (-.014,-.02), press to palm .05, EMPTY fist (palm drifted to y -.33); abort: the open yaw -45 hand lifted the plush with it, home's retreat dragged it back over the front edge (frames 195951-195959), box shifted ~5 cm | FAIL + plush on the FLOOR. Lateral yaw -45 now 1/2 (held at waist .45/0, empty at .7/.15). Fix v17.2: abort checks the hand is empty before home (set it back down, stop) |

## Session 20261102T202144Z_puppy_box_4f16bd (sim) - EVALUATION ONLY of run.py 41edb62 on the elliptic friction cone
8 trials, one row each in memory/EVAL.md block 3 (where I looked and what I decided per trial): 5/8 placed (1, 3, 4 retry, 6, 8).
Looked at every scene overlay, hover (shift 0 -.015 when over the chest: trials 1, 4, 5; centred: 3, 6, 8; blind: 2, 7), lift and carry frame
and every verify frame. One deviation: trial 7 went home with tmp/safe_home.py instead of `abort` (skips the left TUCK that tipped the box in trial 2).

## Session 20261102T212757Z_puppy_box_f20de8 (sim) - run.py v18 development (box LEFT lateral grasp, home without TUCK)
| # | layout | steps / where I looked | result |
|---|---|---|---|
| 1 | start layout: box LEFT (x .267-.561, y -.011..+.381; near wall .267), puppy RIGHT head LEFT legs toward (centre .317,-.277), edge .208, TABLE LOW (z -.072) | v17.2 + home fix: scene (waist .45/.15 = IK no margin), approach (blind), grasp .98/.59 (press to .034), lift: index fist at the 1st step, middle lost at z .135; measured arm off its command (wrist roll -.80 vs -1.20, sh yaw +.14 vs -.30) | FAIL (lift). abort + new home: box (near wall .267) untouched |
| 2 | same, puppy rotated (axis -33, centre .311,-.304) | v18a: waist .3/.25 (grasp IK margin 3 cm) + release waist .45/.15 (swing): press to palm .018 (top .028), close .75/.50, lift to .266 steady (mid .64, idx .68->.87), left clear 5 cm; carry: idx fist at the 1st 'up' (1 cm), swing .3/.25 -> .45/.15 held (middle .63), lost on the 2nd lateral carry step (palm y -.21, yaw rotating -45 -> +30) | FAIL (carry slip, plush on the table). Grasp + lift now work at the lateral reach; carry loses it |
| 3 | reset 1: box LEFT (x .279-.572, y -.028..+.366), puppy RIGHT head LEFT legs toward (centre .321,-.31), edge .209 | v18b: grasp waist .3/.25, release waist .7/.15 ryaw -15: approach,grasp,lift chained (blind hover err .5 cm; press to .034; close .89/.68; lift .27 steady .72/.68, left clear 22 cm); carry: swing held (.79/.67), then EVERY carry step synced the arm (dev .5-.94 with the palm within 1-3 cm = SONIC's waist compensation, not a branch problem); palm lagged 7 cm in y; the hanging plush dragged against the box's RIGHT wall and PUSHED THE BOX 13 cm left (rim y -.028 -> +.103); grip lost after the carry | FAIL + INCIDENT (box pushed 13 cm, upright, on the table; plush on the table). Fixes v18c: sync only when the palm is off (> 4 cm), carry height from the measured hang below the palm + 5 cm |
| 4 | reset 2: box LEFT (x .258-.551, y .039-.419), puppy RIGHT head LEFT legs toward (centre .346,-.265), edge .207 | v18c: waist .45/.25, gyaw -45: press to .038, close .93/.65, the closed 3 cm raise -> FIST (empty) | FAIL (empty after raise), puppy pushed 3.5 cm away/right |
| 5 | same layout, puppy centre .381,-.31 | v18c: waist .45/.35 (tilt <= 5.5), gyaw -45: press to .025, close .92/.55 -> raise .70/.17; lift .266 with middle ~0 -> .69, index .68 -> .93 (creeping), hang not in view; carry: 'up' to rim+.27 (palm z .343, sync 1.16) -> .92/1.02, swing step 2 -> middle fist | FAIL (lost in the swing after a high 'up' + sync); plush fell to x .52 (out of reach) -> reset |
| 6 | reset 3: box LEFT (x .236-.531, y .034-.41), puppy RIGHT head LEFT legs toward (centre .324,-.262), edge .198 | v18d (across-body yaw 13 at lateral, chained approach,grasp,lift): waist .3/.25, press to .027, close .98/.94, lift .267 steady (middle .66, index .89), left clear 18.6 cm; carry: up to rim+.27 (z .338), swing .3/.25 -> .6/.15 (meas .24/.20) grip steady .73-.77, 9 lateral steps at yaw 13.5 -> 15 with NO syncs (dev .5-.75 but palm within 2 cm), palm (.399,.122,.295), frac .72; LOOK carry: paws over the interior -> place: 32 px on the hand, home, verify 22348 in / 343 out | SUCCESS, operator INSIDE. First box-LEFT placement by the program on the elliptic cone |
| 7 | reset 4: box LEFT (x .249-.545, y .038-.416), puppy RIGHT head LEFT legs toward (centre .333,-.236), edge .197 | v18e (plan_left: grasp waist = first with margin, release waist searched separately): grasp .3/.25 yaw 12.7, release .7/.15 yaw 12.7 (no rotation); approach..carry chained: press to .022, close 1.05/1.03 -> .93/.93, lift .278 steady (.66/.78), swing held, 7 carry steps no syncs, palm (.405,.12,.277) frac .64; LOOK carry: paws over the interior -> place: plush stayed on the open hand (2584 px) -> set-down recovery -> 19 px, home, verify 25312 in / 189 out | SUCCESS, operator INSIDE |
| 8 | reset 5: box LEFT (x .219-.518 = near wall 2 cm inside the edge, y .045-.435), puppy RIGHT head LEFT legs toward (centre .352,-.237), edge .199 | v18.1 chained approach..carry: grasp .3/.25 yaw 12.8, close 1.02/1.01, lift .268 steady (.79/.86), hang .124 (measured), swing held, 1st lateral carry step index -> fist, middle later -> fist: the plush rolled onto the BACK of the fist and rode along to palm (.373,.102,.347) = over the box; program stopped (grip lost). By hand: tmp/mvto.py to (.369,.165,.30) -> it slid off INTO the box; run.py place (27 px, home, verify 29677 in) | operator INSIDE, but a manual recovery (not a program success). Home with near wall .219: box untouched. Fix v18.2: carry continues while the plush is on the hand |

### Post-eval (v18.3-v18.6)
| # | layout | steps / where I looked | result |
|---|---|---|---|
| 9 | box BEHIND (x .413-.723, y -.235..+.165), puppy head LEFT legs toward (far .381, mid y -.041), edge .179 (table closer than usual) | v18.5 scene REFUSED (box 0.32 x 0.72: a thin strip at rim height joined the rim blob) -> v18.6 dense-run extents (box .31 x .40); approach (LOOK: chest) -> shift 0 -.015; grasp .73/.86, lift .271, hang .168; carry at cz .338 (reachable at the release per the new check) 4 steps, index fist + sync at the last step, middle .70 held; down to (.488,-.085,.287) frac .98; place 0 px, home, verify 7417 in / 9725 out (plush on its back, legs up) | SUCCESS, operator INSIDE. Unexplained transient after home: left wrist pitch measured -1.13 (cmd 0), max tracking error 1.05 rad, left hip pitch -.16; back to normal within seconds, tilt <= 1.4 |
| 10 | box LEFT (x .246-.534, y .063-.453), puppy diagonal (-37 deg) head LEFT legs toward, at the front edge (centre .273,-.221), edge .181 | v18.6 chained approach..carry: grasp .3/.25 yaw -27, close .80/.87, lift .265 (left clear 4.4 cm), swing held, carry with yaw -27 -> +30 (index fist at step 2, middle .70-.80 held), palm (.401,.129,.286) = 6.6 cm inside the right wall (check wants 7) -> LOOK: paws inside, tail over the right wall -> place 2 px | SUCCESS, operator INSIDE. HOME: both arms blocked in LOOK (fingers_max .55, left wrist pitch -1.09 vs 0, elbows .41/.10 vs .6); right recovered alone; left = resting on the DRAWER UNIT beside the robot (fingertips ~(.05,.30,0)); shoulder pitch .9 -> .7 did not free it, back to guard (.9/-.9) did. Asked the operator first (approved). Fix v18.7: home returns a blocked arm to its guard pose |
| 11 | box BEHIND (x .422-.692, y -.256..+.141), puppy diagonal (-18 deg) head LEFT legs toward (far .359), edge .180 | v18.8: the 1st reset laid the puppy next to the raised left guard hand -> scene refused (hands_clearance 1.9 cm = FALSE: the raised hand's own shading passed the cream mask; fixed: plush points below table + .16 -> 18.8 cm); 2nd reset: scene (to_look found the left arm blocked again -> guard, automatic), approach (LOOK: chest) -> shift 0 -.015, grasp .73/.89, lift .248 (left clear 25.7 cm), carry: v18.5 lowered cz to .318 before moving, both fingers held (.80/.62) to palm (.494,-.117,.293), frac .96; place 0 px; home: BOTH arms blocked in LOOK (dev .84/.81) -> both to guard (automatic) | SUCCESS, operator INSIDE. End pose: both arms guard (up/back near the chest), hands open |

## Session 20261102T235110Z_puppy_box_3b3f99 (sim) - evaluation only (code 80b8524), 8 trials = memory/EVAL.md block 5
Trials 1-5, 7, 8 SUCCESS (operator INSIDE); trial 6 FAIL (diagonal head-toward puppy, short behind carry, plush rode the open hand, fell
in front of the box on home). My decisions: no shift at any hover (centred between the legs where visible); trial 6 chose release at the
program's 'release or abort' stop (abort would open at the same spot). Overlays output/run_*.jpg; frames under runtime/frames/.
