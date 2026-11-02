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
