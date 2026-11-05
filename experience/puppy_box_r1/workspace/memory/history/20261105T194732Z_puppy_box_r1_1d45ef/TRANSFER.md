# Transfer: sim -> real

Written by the launcher on 2026-11-05 when this session inherited the experience of `puppy_box`, which ran in **sim**. This session runs on **real**.

What came over and how to use it ([sim-to-real](../.agents/skills/sim-to-real/SKILL.md)):

- `[strategy]` rows: your plan. Follow them until a measurement here contradicts one.
- `[param]` rows and every number in PARAMS.json without a `real` entry: priors. Measure each before a committal action depends on it, cheapest check first, and record it with `api.params.record()` so the next real session starts from it.
- Untagged rows: treat any number in them as a `[param]`.
- 0 `[sim-only]` rows were moved to `memory/history/puppy_box/SIM_ONLY.md`; do not act on them.
