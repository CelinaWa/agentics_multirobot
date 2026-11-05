"""Scripted operator for R1 puppy_box sim sessions: r1_operator.py <session root> <minutes> [--port 5591] [--rax DIR]

Arms the session once the robot service is up, keeps it armed while there is no fault, answers every agent request
after a few seconds and watches the sim's ground truth (operator side only; the agent never sees it):
  - a request to bring the table closer/further: slides the table (with the puppy and the box) so its front edge is
    the distance the message names ahead of the pelvis (default 0.25 m), unless a hand is still below the table top
    near the edge (then it says so);
  - kind `reset`: the puppy and the box go back where they started on the table (a fallen robot is stood up again);
  - kind `help` after a fault: a fallen robot is reset standing, then motion is armed again;
  - anything else: approved, with where the puppy is in words.
Every success ("puppy INSIDE the box", released, robot standing) and every fall goes to the log with its time."""
import argparse, json, re, sys, time
from pathlib import Path
import numpy as np
import zmq

ap = argparse.ArgumentParser()
ap.add_argument("root", type=Path)
ap.add_argument("minutes", type=float)
ap.add_argument("--port", type=int, default=5591)
ap.add_argument("--rax", type=Path, default=Path("/home/junjieye/workspace/multirobot/2/homebody/.cache/rax-g1real"))
ap.add_argument("--resets", type=int, default=5)
a = ap.parse_args()
sys.path.insert(0, str(a.rax))
from services import arming  # noqa: E402
from services.controller import operator  # noqa: E402

ctx = zmq.Context.instance()
sock = None


def stack(op, **kw):
    global sock
    for _ in range(3):
        if sock is None:
            sock = ctx.socket(zmq.REQ); sock.setsockopt(zmq.LINGER, 0); sock.setsockopt(zmq.RCVTIMEO, 5000)
            sock.connect(f"tcp://127.0.0.1:{a.port}")
        try:
            sock.send(json.dumps({"op": op, **kw}).encode())
            r = json.loads(sock.recv_multipart()[0])
            return r
        except zmq.ZMQError:
            sock.close(); sock = None
    raise RuntimeError("sim server not answering")


def log(*x):
    print(time.strftime("%H:%M:%S"), *x, flush=True)


def event(kind, **kw):
    with (a.root / "operator_events.jsonl").open("a") as f:
        f.write(json.dumps({"t": time.time(), "kind": kind, **kw}) + "\n")


BOX_IN = (0.14 - 0.01, 0.195 - 0.01)      # inner half-extents (x across 28 cm, y along 39 cm), wall 3 mm


def where(g):
    dog, box = np.array(g["dog"]["pos"]), np.array(g["box"]["pos"])
    grasp = np.array(g["right_grasp"])
    if dog[2] < 0.3:
        return "floor"
    q = g["box_quat"]; yaw = np.arctan2(2 * (q[0] * q[3] + q[1] * q[2]), 1 - 2 * (q[2] ** 2 + q[3] ** 2))
    rel = np.array([[np.cos(yaw), np.sin(yaw)], [-np.sin(yaw), np.cos(yaw)]]) @ (dog[:2] - box[:2])
    inside = abs(rel[0]) < BOX_IN[0] and abs(rel[1]) < BOX_IN[1] and dog[2] < box[2] + 0.14
    # held = the gripped leg's paw is at the jaws (robust when the puppy hangs: the body is 15-25 cm from the grasp,
    # so a body-centre test wrongly read "not held" and a table slide dragged the held puppy along, s10 D4)
    paws = [np.array(p) for p in g.get("dog_paws", [])]
    near_paw = min((np.linalg.norm(p - grasp) for p in paws), default=9.0)
    held = near_paw < 0.11 or (np.linalg.norm(dog - grasp) < 0.16 and dog[2] > g["table_top_z"] + 0.08)
    if inside and not held:
        return "box"
    if held:
        return "hand"
    if dog[2] > g["table_top_z"] + 0.12:
        return "air"
    return "table"


WORDS = {"floor": "The puppy is on the FLOOR, out of reach: ask for a reset.",
         "box": "The puppy is INSIDE the box.", "hand": "The puppy hangs in your right gripper.",
         "air": "The puppy is above the table, not resting on it.", "table": "The puppy is on the table, not in the box."}
DIST = re.compile(r"(\d?\.\d+)\s*m")
TABLE_HALF = (0.3, 0.9)   # the table top: 0.6 deep (x), 1.8 long (y), centred on the table body


def layout_pose(g, key):
    """xyz + wxyz of a layout 'place=...' string (where the layout put the object on the table at the start)."""
    f = [float(v) for v in g["layout"][key].split("=")[1].split(",")[1:]]
    return np.array(f[:3]), f[3:7]


def on_table(p, g, margin):
    tp = np.array(g["table"]["pos"]); top = g["table_top_z"]
    # (the top is only 0.6 m deep and the layout puts the puppy 0.17 m from its centre: a small margin across the depth)
    return abs(p[0] - tp[0]) < TABLE_HALF[0] - min(margin, 0.05) and abs(p[1] - tp[1]) < TABLE_HALF[1] - margin and top - 0.03 < p[2] < top + 0.15


def settle_scene(g, keep_dog):
    """The table is a free body: a slide into the robot's legs shoved it 0.4 m back while the server's commanded shift
    stayed put, so reset_scene dropped the box past the far edge (session 4876b1, resets 2-3). After a reset or a
    slide, anything not on the table top is put back where the layout had it RELATIVE TO THE TABLE AS IT IS.
    A puppy in the gripper (keep_dog) is left alone. Returns the names re-placed."""
    tp = np.array(g["table"]["pos"]); base = np.array([tp[0] - (g["layout"]["edge"] + TABLE_HALF[0]), tp[1]])
    fixed = []
    for body, key, margin in (("box", "place_box", 0.2), ("dog", "place", 0.2)):
        if body == "dog" and keep_dog:
            continue
        if on_table(np.array(g[body]["pos"]), g, margin):
            continue
        xyz, q = layout_pose(g, key)
        stack("place", body=body, pos=[float(xyz[0] + base[0]), float(xyz[1] + base[1]), float(g["table_top_z"] + (xyz[2] - g["layout"]["table_h"]))], quat=q)
        fixed.append(body)
    if fixed:
        time.sleep(1.0); log(f"settle_scene: put back on the table: {fixed}"); event("settle", fixed=fixed)
    return fixed

t_end = time.time() + 60 * a.minutes
armed_once, seen, resets, success_logged = False, set(), 0, False
last_state, fell_logged = None, False
start = time.time()
log("operator up", a.root)
while time.time() < t_end:
    time.sleep(1.0)
    try:
        g = stack("gt")
    except Exception as e:
        log("no sim:", e); continue
    st = where(g)
    if last_state is None and st != "hand":   # a scene broken before this operator came up (an old one's bad reset)
        settle_scene(g, keep_dog=False); g = stack("gt"); st = where(g)
    if st != last_state:
        log(f"puppy: {st}"); event("puppy", where=st); last_state = st
    if st == "box" and not success_logged:
        log("SUCCESS: the puppy is INSIDE the box"); event("success", t_s=round(time.time() - start, 1)); success_logged = True
    if st != "box":
        success_logged = False
    if g["fallen"] and not fell_logged:
        log("FALL: the robot is down"); event("fall", t_s=round(time.time() - start, 1)); fell_logged = True
    status = a.root / "operator" / "stack_status.json"
    if not armed_once and time.time() - start > 8 and (a.root / "agent-workspace").exists():
        arming.write(a.root, True, by="r1_operator"); armed_once = True; log("armed")
    for r in sorted(operator.pending(a.root), key=lambda r: r["created"]):
        if r["id"] in seen or time.time() - r["created"] < 3:
            continue
        seen.add(r["id"]); msg = r["message"]; low = msg.lower()
        log(f"request {r['kind']}: {msg[:300]}"); event("request", request_kind=r["kind"], message=msg[:500])
        # operator/notice.txt: something the operator must tell the agent at its next request (one-shot), e.g. that
        # the scene was repaired after a harness fault
        notice = a.root / "operator" / "notice.txt"
        if notice.is_file():
            text = notice.read_text().strip(); notice.unlink()
            _reply = operator.reply
            operator.reply = lambda root, rid, reply, _r=_reply, _t=text: (_r(root, rid, _t + " " + reply), setattr(operator, "reply", _r))[0]
        if g["fallen"] or (r["kind"] == "help" and not arming.read(a.root)["armed"]):
            if g["fallen"]:
                stack("reset"); fell_logged = False
                time.sleep(3)
                reply = "The robot had fallen: I stood it up again at its start, the table went back too. " \
                        "Motion is armed again; observe before moving."
            else:
                reply = "I checked the robot and the space around it: all clear. Motion is armed again; observe first."
            arming.write(a.root, True, by="r1_operator")
            operator.reply(a.root, r["id"], reply); log("  ->", reply); continue
        if r["kind"] == "reset":
            if resets >= a.resets:
                operator.reply(a.root, r["id"], f"No more resets: the {a.resets} allowed are used up. " + WORDS[st]); continue
            resets += 1
            stack("reset_scene"); time.sleep(1.5)
            g = stack("gt"); fixed = settle_scene(g, keep_dog=False)
            g = stack("gt"); st = where(g)
            reply = f"Scene reset done ({resets} of {a.resets}): the puppy and the box are back where they started. " + WORDS[st]
            if fixed:
                reply += " (The table had been pushed; I placed the " + " and ".join(fixed) + " on it where the layout puts them.)"
            if not arming.read(a.root)["armed"]:
                arming.write(a.root, True, by="r1_operator")
            operator.reply(a.root, r["id"], reply); log("  ->", reply); event("reset", n=resets); continue
        asks_slide = "table" in low and any(w in low for w in ("closer", "slide", "bring", "move the table", "further", "farther", "push the table", "pull the table", "shift the table")) \
            and not any(w in low for w in ("not asking", "no slide", "don't slide", "do not slide", "without sliding", "not a table"))
        if asks_slide:
            # Session 2 (4876b1): "the box is 0.6-1.0 m to my LEFT ... could you slide the table ..." was read as "edge
            # 0.6 m ahead": the table went AWAY and the puppy flew off. Parse only the request's own clause (from the
            # slide verb on), take its last left/right as the direction, and move sideways along the pelvis' left axis;
            # with no side word it is a front-edge distance as before. Check the hands against the table's current and
            # future footprint for every slide, slide in 1 cm sub-steps, and keep a held puppy in the gripper (the
            # server moves the puppy root with the table).
            edge, top = g["table_edge_ahead_m"], g["table_top_z"]
            pq = g["pelvis"]["quat"]
            yaw = np.arctan2(2 * (pq[0] * pq[3] + pq[1] * pq[2]), 1 - 2 * (pq[2] ** 2 + pq[3] ** 2))
            fwd, left = np.array([np.cos(yaw), np.sin(yaw)]), np.array([-np.sin(yaw), np.cos(yaw)])
            k0 = min([low.find(w) for w in ("slide", "move the table", "bring", "push", "pull", "shift") if w in low] or [0])
            tail = low[k0:]
            md = re.search(r"(\d?\.\d+)\s*m\b", tail)
            # the SLIDE direction is the one phrased "to my/the <dir>" or "<dir>-hand side"; a stray "left/right"
            # describing where a hand hangs ("the left hangs at my side") must NOT count (session 0a9c49 slid the
            # wrong way twice). Prefer the directional phrase; else the direction nearest the distance; else first.
            dirm = re.search(r"to (?:my|the|your)\s+(left|right)|(left|right)[- ]hand side", tail)
            side_word = (dirm.group(1) or dirm.group(2)) if dirm else None
            sides = re.findall(r"\b(left|right)\b", tail)
            if side_word is None and sides and md:
                i = tail.find(md.group(0)); side_word = min(sides, key=lambda w: abs(tail.find(w) - i))
            if side_word is None and sides:
                side_word = sides[0]
            lateral = bool(side_word) and not any(w in tail for w in ("ahead of", "closer", "further", "farther", "edge is"))
            if lateral:
                want = float(np.clip(float(md.group(1)) if md else 0.35, 0.05, 0.8))
                sgn = 1.0 if side_word == "left" else -1.0
                shift = sgn * want * left
                what = f"{want:.2f} m to your {side_word}"
            else:
                x = float(md.group(1)) if md else None
                # "0.05 m closer" / "0.16 m further (away)" are relative to the edge as it is; "edge ... 0.28 m ahead" is
                # absolute (the task prompt suggests both phrasings); no number: 0.25 m ahead
                if x is not None and re.search(r"\d?\.\d+\s*m\b[^.]{0,25}\b(closer|nearer)\b", tail):
                    want = edge - x
                elif x is not None and re.search(r"\d?\.\d+\s*m\b[^.]{0,25}\b(further|farther|away|back)\b", tail):
                    want = edge + x
                else:
                    want = x if x is not None else 0.25
                want = float(np.clip(want, 0.18, 0.6))   # 0.12 m put the table into the robot's legs (shoved 0.4 m)
                shift = -(edge - want) * fwd
                what = f"front edge {edge:.2f} -> {want:.2f} m ahead"
            tp = np.array(g["table"]["pos"][:2]); hx, hy = 0.3 + 0.05, 0.9 + 0.05   # the table: 0.6 deep, 1.8 long
            def over_table(p):
                return any(abs(p[0] - c[0]) < hx and abs(p[1] - c[1]) < hy for c in (tp, tp + shift))
            blocking = [k_ for k_ in ("right_gripper_base", "left_gripper_base")
                        if np.array(g[k_]["pos"])[2] < top + 0.15 and over_table(np.array(g[k_]["pos"])[:2])]
            if blocking:
                names = ", ".join(b.replace("_gripper_base", " gripper") for b in blocking)
                reply = (f"Not yet: your {names} is less than 15 cm above the table top over where the table is or will be. "
                         "Raise both hands above that (or pull them back behind the edge), then ask again.")
            else:
                held = st == "hand"
                dog_pose = (g["dog"]["pos"], g["dog"]["quat"]) if held else None
                n = max(3, int(np.ceil(np.linalg.norm(shift) / 0.01)))
                for _ in range(n):
                    stack("slide_table", dx=float(shift[0] / n), dy=float(shift[1] / n))
                    if dog_pose is not None:
                        stack("place", body="dog", pos=list(dog_pose[0]), quat=list(dog_pose[1]))
                    time.sleep(0.12)
                g2 = stack("gt"); fixed = settle_scene(g2, keep_dog=held); g2 = stack("gt") if fixed else g2
                reply = (f"Done: I slid the table {what}, with the box and the puppy on it"
                         + (f" (the {' and '.join(fixed)} slipped off and I put it back on the table)" if fixed else "")
                         + (", keeping the puppy in your gripper" if held else "")
                         + f". Its front edge is now {g2['table_edge_ahead_m']:.2f} m ahead of your pelvis; measure before moving.")
            operator.reply(a.root, r["id"], reply); log("  ->", reply, f"(shift {np.round(shift, 3).tolist()})")
            event("table", shift=np.round(shift, 3).tolist(), was_edge=round(edge, 3), blocked=bool(blocking)); continue
        # position the puppy so a hind leg overhangs the table's front edge (the only way a flat-lying puppy can be
        # gripped by the parallel gripper: the hanging leg has empty space under it). A person could nudge it there.
        if "overhang" in low or (("puppy" in low or "leg" in low) and any(w in low for w in ("over the edge", "past the edge", "off the edge", "over the front edge"))):
            tp = np.array(g["table"]["pos"]); edge_x = tp[0] - TABLE_HALF[0]
            dogp = np.array(g["dog"]["pos"]); dq = g["dog"]["quat"]
            paws = [np.array(p) for p in g.get("dog_paws", [])]
            front_paw_x = min(p[0] for p in paws) if paws else dogp[0] - 0.11   # frontmost paw (toward the robot)
            dx = float(np.clip((edge_x - 0.06) - front_paw_x, -0.25, 0.0))      # frontmost paw ~6 cm past the edge
            stack("place", body="dog", pos=[float(dogp[0] + dx), float(dogp[1]), float(dogp[2])], quat=list(dq))
            time.sleep(0.5); g2 = stack("gt")
            reply = ("Done: I nudged the puppy forward so a hind leg now overhangs the table's front edge (its hanging "
                     "leg has open space beneath it). Observe, then grip the hanging leg from the front with the jaws "
                     "below the table top.")
            operator.reply(a.root, r["id"], reply); log("  ->", reply, f"(dx {dx:.3f})"); event("overhang", dx=round(dx, 3)); continue
        if not arming.read(a.root)["armed"]:
            arming.write(a.root, True, by="r1_operator")
        reply = "Yes: approved, go ahead. Motion is armed. " + WORDS[st]
        if "reach" in low and ("box" in low or "left" in low or "right" in low):
            reply += (" I can also slide the table sideways, with the box and the puppy on it: say for example "
                      "'slide the table 0.4 m to my right' (or 'to my left'), with both hands at least 15 cm above the table top.")
        operator.reply(a.root, r["id"], reply); log("  ->", reply)
log("operator done")
