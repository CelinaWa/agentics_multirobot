---
name: strategy-review
description: At the start of a session, have a fresh subagent read all the experience (funnel, ground truth, notes) and write a one-page strategy; at the end of a session, curate the notes to a size the next session can read and file tool requests. Read at the start and at the end of every session.
---

# Look at the whole history before following the last plan

Every session inherits the previous session's notes, and their "next step" pulls you into continuing the last plan.
After many sessions that is local search: each session improves the last idea a little and nobody asks whether the
idea is the bottleneck. A short review with fresh eyes, from data, before acting, is cheap.

## At the start: the strategy page (`memory/STRATEGY.md`)

At the start, start a subagent (the Agent tool, a strong model, in the background) that has not seen your plans, with this
job (while it works, run what every plan begins with - bring-up checks, the stages all strategies share - rather than
leave the robots idle; S30 waited 8 of its first 14 minutes): read `TASK.md`, `memory/CURRENT.md` (or, if it does not exist yet, `handoff/README.md` and `memory/`), the funnel
`python -m api.attempts`, the ground truth under `memory/ground_truth/`, and `memory/TOOL_REQUESTS.md`; then write
`memory/STRATEGY.md`, one page:

1. Where the task stands, in numbers: stage by stage tries and successes over all sessions, how far the task has
   really got (ground truth), how many sessions that took.
2. The bottleneck: the stage that loses the most, and why (cause counts, not impressions). Whether it is a skill
   problem (practice can fix it), a method problem (a different way is needed) or a tool problem (a tool cannot do
   what the stage needs; say which number).
3. Two or three strategies for this session with expected gain and cost, including ones the notes have not tried,
   and which one it recommends. A strategy may be: practise one stage (see
   [practice-by-stage](../practice-by-stage/SKILL.md)), change the method of a stage, change the split of work between
   the robots, or whole attempts when the stages are each reliable.
4. What it would stop doing.
5. **The exclusions, audited.** List every rule in the notes that rules a direction, a method or a split of work out
   ("never", "only", "impossible", "does not work"), each with its evidence: how many tries, in which session, and
   whether something else could explain them (a weak grip already, a reset or operator action at the same time, a
   different tool version). A rule from one or two tries, or from confounded tries, is an open question, not a fact:
   name the cheapest test that would settle it. Notes written by agents carry their mistakes forward; the rules that
   shape every later session are the ones most worth re-checking.
6. **Does the plan lead to the goal?** For each strategy, say how its success moves the task toward `TASK.md`'s
   success condition. A stage practised to reliability in a direction or a way the route cannot use (carrying away
   from the goal, a drop the task does not allow) is not progress; at least one strategy must move the task toward
   the goal itself, even if it is less tried.
7. **How to practise the bottleneck.** For the bottleneck stage: its start state, the cheapest way to reach it
   (operator placement, a one-robot stand-in, a proxy object), and how many repetitions fit in the session. Whole
   attempts from a reset that reach the bottleneck once each are not practice of it.
8. **The stages never tried.** Once ground truth shows a stage succeeding at least twice, the stages after it get
   tries this session - a fixed share of the robot time (a third is a good start), even while the earlier stage is
   still unreliable: a stage never tried is the largest unknown on the route, and polishing the stage before it
   only pays if the later ones work. Say how to start them (from a successful earlier stage, or a start state the
   operator can set up).

The harness keeps the task's progress from ground truth (each reset reply's STATE: task_progress). When it says
TASK PROGRESS STALLED, have the page written again by a fresh subagent that gets the ground truth and the funnel
before any notes, and decide again.

Then decide yourself, write your decision and why under the page, and plan the session's time by it. Disagreeing
with the page is fine; ignoring it is not. When you change the plan mid-session (more steps than the plan allowed, a
different method), write the change and its reason under the page before you run it, and check the funnel
(`python -m api.attempts`) to choose the next practice, not only to report it.

## At the end: curate and hand over

The harness ends robot work some minutes before the session ends and asks you for this; do it then, and earlier if
you stop robot work early:

- **`memory/CURRENT.md`, at most 30 KB**: the current state of knowledge, rewritten, not appended: the stage graph and
  funnel numbers, what works (with the controller and its run command), what does not and why, parameters that
  matter, the open questions and the recommended next step with its alternative. A session that reads only this file
  and the skill library must be able to work.
- **Archive, do not delete**: move long logs (old RUNS, TIPS sections superseded, session stories) under
  `memory/archive/` and keep a one-line pointer in CURRENT.md. `handoff/README.md` keeps the transfer notes for a
  different scene, short, and points to CURRENT.md.
- **`memory/TOOL_REQUESTS.md`**: what the tools cannot do that the task needs, each with the evidence (numbers,
  how many tries it cost) and what would fix it. The developers read this between sessions.
- **Skill library**: update each skill you used with its measured success rate from the funnel; mark a skill whose
  rate stayed low as such rather than leaving it to be reused blindly. Merge.
- **Look from outside**: where the simulation's operator saved third-person frames (`runtime/stream/replay/`, one at
  each reset and fall, named in `runtime/stream/ground_truth.jsonl`) [sim-only], look at the failures' frames: a
  knee against the box or an arm sweeping it shows there and not in the robots' own cameras.
