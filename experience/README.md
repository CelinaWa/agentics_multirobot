# Experience carried between machines

What the agent learned in one place (the simulation on the DGX, or an earlier day on the robot),
packed so a session elsewhere starts from it instead of from zero. One folder per task:

```
experience/<task>/
    SOURCE.txt                     which session, which robot, when
    workspace/memory/              TIPS.md, RUNS.md, INDEX.md, skills/ (the task memory; PROGRESS.md becomes history)
    workspace/handoff/README.md    how to plan each subtask in a differently laid-out scene
    workspace/code/                the controller the session wrote, if any
    skill-library-<robot>.bundle   git bundle of the robot's skill library (verified, executable skills)
```

Produce it where the session ran: `bash rax/scripts/export_experience.sh <session root> experience/<task>`.
Use it on the robot host:

```bash
bash rax/scripts/import_experience.sh experience/<task>                       # library bundle -> this host's library
bash rax/scripts/g1_up.sh agent --task-file tasks/<task>.md --experience experience/<task>/workspace
```

The agent then finds `handoff/README.md`, `memory/TIPS.md` and `memory/history/<session>/PROGRESS.md`
in its workspace and the library skills in `skill-library/`, and is told (AGENTS.md) to verify every
inherited number on the real scene before relying on it. After the real session, export again from
the robot host and commit, so the next sim batch starts from the real experience too.

## History

- 2026-11-01: `experience/puppy_box` from the first eleven sim sessions was withdrawn. That simulation modelled the
  plush puppy as one rigid body and placed it mostly out of reach, so its grasp conclusions ("tail pinch fails",
  "only a torso straddle can work") were artefacts and would mislead a session on the real robot. A new export
  follows from the sessions on the corrected simulation (soft, floppy puppy; reachable layouts). Until then, start
  the robot without `--experience`.
- 2026-11-03: `experience/puppy_box` is session 3b3f99 (sim): one program, `code/puppy_box/run.py` v18.8, for every
  layout (puppy left of, right of or in front of the box, right hand only). Its last no-code-change evaluation placed
  7 of 8 on a new scene with no falls; every row is tagged; `handoff/README.md` starts with what to measure first.

## Sim to real (2026-11-02)

An export records where it ran (`SOURCE.txt`: `environment: sim|real`) and how its memory is tagged
(`TRANSFER_REPORT.txt`). Tips are tagged `[strategy]` (follow), `[param]` (re-measure; values in
`workspace/memory/PARAMS.json` with a `sim` and a `real` entry) or `[sim-only]` (dropped on the robot). When a real
session starts from a sim export, the launcher moves the `[sim-only]` rows into `memory/history/` and writes
`memory/TRANSFER.md`, the list of sim numbers to measure first (rax skill `sim-to-real`, `docs/RAX_INSIDE.md` 9.1).

`puppy_box/`: sim session 20261102T130220Z_puppy_box_996d97 and its 16 predecessors (memory inherited along the chain),
kind tags and PARAMS.json curated by hand for the export. Sim result on the randomized layouts, right hand only:
puppy left of the box 10 confirmed successes, right of it 5, in front of it 3; one fall (seed 33, before the runaway
watch), none in the 6 sessions since. The front layout (box behind the puppy) is the weakest: the release sits at the
right hand's loaded reach (`reach.loaded_palm_max_x_m`).
