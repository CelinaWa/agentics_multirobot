# Multi-robot task decomposition (stage 1 of 3), branch `nick`

Browser view: open `nick/plan_nick.html` (same content, with the reused / new / edit diff).

## Status

| # | Checkpoint | Status | How it was tested |
|---|---|---|---|
| 1.0 | Branch `nick`, `nick/` folder with plan and page | done 2026-11-09, awaiting review | `nick/plan_nick.html` generated; not yet opened by a reviewer |
| 1.1 | Plan checks, skill catalog, capability cards, sample skill folders | not started | |
| 1.2 | Coordinator role inside RAX | not started | |
| 1.3 | Coordinator instructions, example goals, first live run | not started | |
| 1.4 | Expectations, evaluator, how-to document (stage 1 acceptance) | not started | |
| 1.5 | Hand-off to the collaborator | not started | |

Each step stops for review before the next one starts.

## Context

Today RAX drives one robot per session on one hand-written task file. Nothing decomposes a
high-level goal, assigns work across robots, or checks which skills are already trained.
The goal is a coordinator in three stages:

1. **Decomposition (this plan):** goal -> essential subtasks -> robot assignment (single or
   multi-robot) -> reuse of trained skills -> RAX-ready task files for skills still to build.
2. **Orchestration (later, once the collaborator's skills exist):** closed-loop dispatch,
   task switching, replanning.
3. **Self-improvement (last):** learn from failures caused by bad decomposition/orchestration.

Nothing exists yet. Approach: **the decomposer is a RAX session**, not a new program. RAX
already launches a sandboxed Claude Code agent with markdown instructions, tools, memory, a
skill library and an operator channel. We add a "coordinator" role to it. The decomposition
logic lives in markdown; the only Python is a small tool backend.

**Terms.** "Skill" means a robot behaviour for one subtask: a RAX-built controller in a
robot's skill library, or a trained policy (single-robot = one robot name, multi-robot =
several). The Markdown files under `agent_skills/` that guide the Claude agent are called
"coordinator instructions" here, although RAX stores them as `SKILL.md`.

## Working folder `nick/` (repo root, created first)

- `nick/plan_nick.md`: the current plan. Updated in the same commit as every step, with a
  status line per checkpoint (not started / done + date + how it was tested).
- `nick/plan_nick.html`: the browser view of the plan and the reused/new/edit diff. A
  self-contained page (inline CSS, no build step, system-font fallback) that anyone with the
  repo opens by double-clicking. Regenerated whenever `plan_nick.md` changes; the same
  content is published to the Claude artifact link.

## What is reused as-is (no changes)

| Need | Existing piece |
|---|---|
| Start a session, sandbox, login, budgets, resume | `scripts/configure_session.py`, `scripts/start_agent.sh`, `harness/claude_cli/launcher.py` |
| Goal input | `--task-file` -> `TASK.md`; same Markdown shape as `tasks/*.md` incl. prose `## Scene` |
| Fleet input | `--backend-options fleet.json` (already an arbitrary JSON passed to the backend) |
| Trained RAX skills | per-robot libraries at `<artifact root>/skill-library/<robot>/`, filled by `scripts/import_experience.sh` from `experience/*/skill-library-*.bundle`; `INDEX.md`, `SKILL.md` frontmatter (`status` draft/verified), `evidence/runs.jsonl` |
| Reading skills | `skill_library._frontmatter`, `_list_field`, `STATUSES`; `embodiment._library_files` |
| Robot tool lists and limits | each backend's `tool_definitions()` (`services/g1sonic/tools.py`, `r1sonic/tools.py`, `rby1/backend.py`) |
| Ask a human (plan approval, open questions) | `request_operator` tool, `services/controller/operator.py` |
| Critic pass | `--subagents` (Claude Code subagents inside the session) |
| Memory of what worked | workspace `memory/` (`TIPS.md`, `PROGRESS.md`, incidents table) and method instructions in `agent_skills/method/` |
| Stage 3 for free | a `coordinator` skill library is created automatically by `skill_library.checkout`; `skill_library_merge`, `export_experience.sh` / `--experience` carry decomposition lessons between sessions |
| Stage 2 building blocks | `operator.pending/reply`, `services/arming.py`, `agent-lifecycle.json`, `MockBackend` |
| Tests | `local_tests/unit/conftest.py` fixtures (`configuration`, `frontend`), `--prepare-only` |

## What is added (all new files, under `rax/` unless noted)

```
services/coordinator/
  backend.py        CoordinatorBackend: read-only tools, moves nothing
  catalog.py        list robots (cards) and skills (RAX libraries + policy manifest); also a CLI
  plan.py           plan dataclasses + deterministic checks + task-file writer
  evaluate.py       checks a submitted plan against a goal's expectations file
../robots/{g1sonic,r1sonic,rby1}.json   capability cards, one per robot type; add a file to add a type
coordinator_agent/  workspace template: AGENTS.md, START_PROMPT.md  (the role's instructions)
agent_skills/coordinator/   four coordinator instruction files:
  task-decomposition/  robot-assignment/  skill-reuse/  plan-format/ (+ references/example.json)
local_tests/unit/test_coordinator.py
../tasks/coordinator/{clean_house,tidy_table,move_bench}.md   example goals
../tasks/coordinator/*.expect.json                            what a good plan must contain
../tasks/coordinator/fleet.example.json                       fleet file
../tasks/coordinator/example_skills/                          two sample skill folders (one single,
                                                              one multi) showing how a trained policy is registered
../docs/COORDINATOR.md          how to run it; the contracts for the collaborator
../nick/plan_nick.md, plan_nick.html
```

### Tools the coordinator agent gets (`backend.py`)
- `fleet`: robots from `fleet.json` (`id`, `robot` type, optional location note) joined
  with their capability card.
- `skill_catalog`: every trained skill, each tied to robot names. One source and one
  format, the existing skill library (no new manifest):
  - `skill-library/<robot>/`: single-robot skills, robot = the library's robot. Both
    RAX-built controllers and the collaborator's trained single-robot policies live here
    (a policy is a skill folder whose `entry` .py starts the policy).
  - `skill-library/multi/`: multi-robot skills, same format plus one extra frontmatter
    line `robots: [g1sonic, r1sonic]` (extra fields are already tolerated by
    `skill_library.validate_skill`; its required `robot` field is set to `multi`).
  - For each skill: status, version, and runs / successes / unattended successes computed
    from `evidence/runs.jsonl`. Validation, bundling, `import_experience.sh` and
    `export_experience.sh` work on all of these unchanged.
  - A robot name matches a fleet `id` first, then a robot type.
- `plan_check(plan)`: deterministic checks, returns the list of problems: schema; no
  dependency cycles; assigned robots exist and hold each required capability; a bound skill
  exists and its robots match the assignment; multi-robot leaves have >= 2 robots and a
  reason; every leaf has an observable success condition; parallel leaves do not share a
  robot or an object/space.
- `plan_submit(plan)`: runs the checks, then writes to `<session root>/plan/` (host side):
  `plan.json`, `plan.md`, one `tasks/<subtask>.md` per `to_build` leaf in the existing
  task-file structure, and `commands.sh` with the matching `configure_session.py` /
  `start_agent.sh` lines. Nothing is launched. This is the machine-readable result.

### Closed loop inside one session (written in `AGENTS.md` + instruction files)
read goal -> `fleet` + `skill_catalog` -> propose task tree -> split composite nodes again
(depth limit) -> `plan_check` -> repair until clean -> critic subagent looks for missing or
unnecessary tasks and poor assignments -> `request_operator` for open questions (optional)
-> `plan_submit` -> write `memory/PROGRESS.md` and tips.

### plan.json (contract for orchestration and the collaborator)
Per subtask: `id`, `title`, `description`, `parent`, `depends_on`, `kind` (`single`|`multi`),
`robots`, `assignment_reason`, `alternatives`, `required_capabilities`, `skill`
(`{source, name, status, evidence}` or `{"status":"to_build"}`), `preconditions`,
`success_condition`, `resources`, `on_failure`. Plan level: `assumptions`, `open_questions`.

### Any number of robots, any capabilities
- Nothing is fixed to two or three robots or to the three robot types in the repo. The fleet
  file lists any number of robots; a multi-robot subtask or skill names any number >= 2
  (`robots: [...]`, optional role per robot).
- RAX has no file documenting what each robot can do today: it is spread over tool schemas
  in code, prose in `ROBOT_FACTS` (stale for R1) and the robot-interface guides. So the one
  thing added per robot type is a **capability card**, `robots/<type>.json` at the repo root
  (not under `services/`, so anyone can add one without touching code).
- Capability tags are open vocabulary (free strings such as `walk`, `crouch`,
  `two_finger_grip`, `wheeled_base`, `reach_floor`, `carry_5kg`), not a fixed list. The
  checker only tests that the assigned robots hold the tags a subtask requires.
- A robot type with no RAX backend (e.g. a new robot the collaborator brings) works for
  decomposition with just its card; a fleet entry may also add or override tags for one
  individual robot (a different gripper, a broken arm).
- Example goals and tests include a fleet of 5 robots with a 3-robot subtask and a made-up
  robot type that exists only as a card.

### Robot descriptions (reused, plus a small card)
The `fleet` tool returns, per robot: its tool list read live from the backend's
`tool_definitions()` (not from `ROBOT_FACTS`, which is stale for R1), and the existing
`agent_skills/control/<robot>/robot-interface` guide is copied into the coordinator's
workspace as reference reading (when the type has a backend). The card (`robots/<type>.json`) holds only the few
structured facts `plan_check` needs: capability tags (e.g. `walk`, `crouch`,
`two_finger_grip`, `wheeled_base`), hard limits (e.g. G1 cannot reach the floor), and
`payload: unknown` where the repo gives none.

## Edits to existing files (all additive; no robot path changes)

1. `services/backends.py`: one line in `BACKENDS`: `'coordinator': 'services.coordinator.backend:CoordinatorBackend'`.
2. `services/workspace/setup.py`: one `ROBOT_FACTS['coordinator']` entry (like `mock`), so `Setup` accepts the role.
3. `services/workspace/recipes.py` + the one `recipe_for()` call in `launcher.populate_workspace`:
   add a `COORDINATOR` recipe (template `coordinator_agent/`, layers `coordinator` +
   `method`) and select it when the robot is `coordinator`; every other robot still gets `AGENT`.
4. `agent_skills/README.md` (one table row) and a pointer line in the root `README.md`.

## Stage 1 checkpoints: what is built and what you can test after each

| # | Built | Testable when done (no robot needed for any of these) |
|---|---|---|
| 1.0 | Branch `nick`; `nick/plan_nick.md`, `nick/plan_nick.html` | Open the HTML from the repo; it shows this plan |
| 1.1 | `plan.py` checks, `catalog.py`, small capability cards, sample skill folders | `pytest local_tests/unit/test_coordinator.py`: each check rejects a deliberately broken plan; the sample skill folders pass RAX's own `validate_skill`. `python -m services.coordinator.catalog --fleet ...` prints every trained skill found in the two shipped bundles plus the samples, with robot names, status and success counts |
| 1.2 | Coordinator role: backend with the 4 tools, registry line, recipe | Full existing suite still green (robot sessions unaffected). `configure_session.py --robot coordinator ...` then `launcher.py --prepare-only` builds a workspace you can inspect; tool calls answered in a unit test |
| 1.3 | Coordinator instructions (`AGENTS.md` + 4 files), example goals, expectations | One live run on `clean_house.md` ends with `<session>/plan/plan.json`, `plan.md`, task files and `commands.sh`; `plan_check` reports no problems |
| 1.4 | `evaluate.py`, expectations for 3 goals, `docs/COORDINATOR.md` | **Stage 1 acceptance:** each of the 3 goals run 3 times; every run passes the checks and its expectations: essential tasks present; a two-robot-heavy object gets a multi-robot leaf; "puppy into box" binds the existing G1 skill instead of `to_build`; no floor pick goes to the G1; no robot in two parallel leaves; nearer capable robot chosen when the scene says so |
| 1.5 | Hand-off to the collaborator | (a) Add a skill folder for a new policy to a library (same steps the collaborator will use), rerun: the plan now binds to it instead of `to_build`. (b) One emitted `tasks/<subtask>.md` is accepted by `configure_session.py` (`--robot mock`, and `g1sonic` in sim if the sim host is available) |

Live runs need the RAX Claude login on this machine (`scripts/setup_claude_experiment_login.sh`).

## Later stages: what will be testable (not built now)

- **Stage 2, orchestration.** The same backend gains `dispatch`, `session_status`,
  `answer_operator`, built on `configure_session` + `start_agent.sh --non-interactive`,
  `operator.pending/reply`, `agent-lifecycle.json`.
  - 2.1 (no robots): a plan executes end to end against fake skill executors; order follows
    dependencies; independent tasks run at the same time on different robots; an injected
    failure leads to a retry, a switch to an alternative robot, or a replan; a final report
    lists each subtask's result.
  - 2.2 (sim): one plan drives two real RAX sessions (G1 sim and R1 sim) to completion.
  - 2.3 (real): the collaborator's skills on real robots.
  - RAX gaps to close then: a robot session has no machine-readable success result;
    `g1_up.sh` and the dashboard assume one session; one agent's sandbox can probably reach
    another robot's control socket (untested).
- **Stage 3, self-improvement.** Lessons accumulate through the existing memory, incident
  and skill-library mechanisms under the `coordinator` library.
  - 3.1: replay a logged failure caused by a bad split or assignment; the coordinator
    records a lesson; rerunning the same goal does not repeat the mistake.
  - 3.2: the expectations set from 1.4 is scored before and after learning; the score does
    not drop on any goal.
