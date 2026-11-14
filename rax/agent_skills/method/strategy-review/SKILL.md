---
name: strategy-review
description: Before the first motion of a session, have a fresh subagent read all the experience (funnel, ground truth, notes) and write a one-page strategy; at the end of a session, curate the notes to a size the next session can read and file tool requests. Read at the start and at the end of every session.
---

# Look at the whole history before following the last plan

Every session inherits the previous session's notes, and their "next step" pulls you into continuing the last plan.
After many sessions that is local search: each session improves the last idea a little and nobody asks whether the
idea is the bottleneck. A short review with fresh eyes, from data, before acting, is cheap.

## At the start: the strategy page (`memory/STRATEGY.md`)

Before the first motion, start a subagent (the Agent tool, a strong model) that has not seen your plans, with this
job: read `TASK.md`, `memory/CURRENT.md` (or, if it does not exist yet, `handoff/README.md` and `memory/`), the funnel
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

Then decide yourself, write your decision and why under the page, and plan the session's time by it. Disagreeing
with the page is fine; ignoring it is not.

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
