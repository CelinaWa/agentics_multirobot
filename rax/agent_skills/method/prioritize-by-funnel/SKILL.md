---
name: prioritize-by-funnel
description: Account for every attempt by stage in a cumulative funnel and fix the stage that loses the most first, with a backlog of known issues. Read after each attempt and when choosing what to work on next.
---

# Account for every attempt by stage and fix the biggest loss first

- **A funnel by stage.** Naming the stages of your task in order (for example find, approach,
  contact, transport, finish; your task's own) and recording after each attempt whether each stage
  was reached and succeeded, with the cause of the first failure, gives a cumulative table in
  `memory/` that shows where attempts are lost. Retries within an attempt count as attempts. On a
  real robot you get few attempts in a session; the funnel is what makes a dozen of them readable.
- **Best gain per cost first.** Gain: the attempts lost at a stage times the chance that the fix
  works. Cost: effort, risk to stages that work, risk to the robot, and the robot time needed to
  validate it. Late stages are best judged per attempt that reached them; their losses are easy to
  undercount.
- **A backlog** of known issues, each open, fixed in version k, or rejected for a stated reason, keeps
  repeated findings from slipping.
- **Practise where the uncertainty is.** A late stage reached only twice is worth practising on
  its own (ask the operator to set the scene up at that stage) before an earlier one is polished.
- **A summary of the final funnel and the open backlog in `memory/`** before the session ends tells
  the next session, and the operator, where the task stands and what to work on first.
