---
name: spend-the-budget
description: Plan and spend the session's wall-clock limit, model calls and the operator's time deliberately - action length by risk, scripts for the fast loop, budget checks before long sequences. Read when planning an attempt or when a budget runs low.
---

# Spend the budgets deliberately

A session has three budgets: its wall-clock limit (`status`, your own thinking included), your model
calls (`status`), and the operator's time and the robot's battery, which no counter shows. When the
wall or call limit ends the session, the robot keeps standing on its last frame and the operator
takes over; nothing you have not written down survives.

- **Counts come from `status`, not from notes.** The budgets left are in `status`; counts remembered
  from notes or earlier context are a common source of error. When another source disagrees on a fact
  (your notes, a code comment, an old plan), TASK.md, `status` and the live tool schemas hold; noting
  the conflict in `memory/PROGRESS.md` keeps the stale information from coming back.
- **A budget plan** in `memory/PROGRESS.md` helps: what each next attempt is for and what it should
  measure, and which of them your code runs on its own while you watch. Revising it after each attempt
  keeps it useful; attempts without a stated purpose were usually wasted.
- **Action length can follow risk:** long calls (up to 100 frames, 2 s) for open-space motion and
  settled holds, short ones near contact, obstacles and people.
- **A script for the fast loop** runs observe–decide–act in Python far more efficiently than deciding
  each step by hand. Thinking between steps and letting the script act within them works well.
- **When an approach proves infeasible, a different strategy** usually beats small variations of
  the same one; identical repeated actions rarely change the outcome, and on a real object each
  repeat can push it further away.
- **A budget check before a long sequence** (enough wall time and calls to finish it and to leave
  the robot in a safe pose, and a plan for when they do not suffice) avoids ending mid-way.
- **Write before you run out.** Notes, the skill library merge and the handoff take minutes; start
  them while enough budget is left, not when `status` says it is gone.
