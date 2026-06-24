# ode-skills

[![verify](https://github.com/jerryzhao173985/ode-skills/actions/workflows/verify.yml/badge.svg)](https://github.com/jerryzhao173985/ode-skills/actions/workflows/verify.yml)

An installable **[Claude Code](https://claude.com/claude-code) skill** that teaches an agent to design,
build, and *verify* rigid-body physics simulations on the **[Open Dynamics Engine](https://www.ode.org/)
(ODE)** in C/C++ — correctly, on the first try.

It is not a copy of the ODE manual. It is an **orchestrator**: `SKILL.md` frames the mental model and the
build workflow, and routes to ~70 reference files where every rule cites a real `include/ode/*.h:line`.

## What it gives an agent

- **The right mental model first.** ODE is *two independent subsystems you compose* — **dynamics**
  (`dWorld`⊃`dBody`+`dMass`, joints) and **collision** (`dSpace`⊃`dGeom`), bridged by `dGeomSetBody`. The
  collision pipeline is **conditional** — a joints-only mechanism (arm, pendulum, gears) needs none of it.
- **The toolchain reality that actually breaks builds.** `libode` is a C++ library → link with the `c++`
  driver; take flags from `ode-config`; define **no** precision macro; assert `dCheckConfiguration` at
  runtime; Homebrew ships no renderer → **default to headless self-checking**.
- **A verification discipline.** Don't trust "it compiled" — build headless, assert *emergent* invariants
  (energy can't grow, joints stay satisfied, no NaN), and **prove the check can FAIL** (falsification).
- **Per-element component cards** (each joint & geom type), deep references (math, threading, trimesh,
  build/backends), worked examples, and a cross-cutting pitfalls checklist.

## Install

Copy the `ode/` directory into your project's skill folder:

```sh
cp -R ode /path/to/your/project/.claude/skills/
```

Then the agent auto-loads it on ODE tasks (triggers: "ODE", `dWorldStep`, `dBodyCreate`, "rigid body
simulation", …). Requires ODE installed (`brew install ode`; `ode-config` on `PATH`).

## What's inside `ode/`

```
SKILL.md                         orchestrator: mental model · build/run · the canonical loop · routing · hard rules
AGENTS.md                        operating guidance for agents writing/extending ODE code
references/
  foundations/                   mental-model (read-first) · scope-and-limits (when ODE fits / what's validated) ·
                                 stepping-and-stability · erp-cfm-friction · contacts-and-friction ·
                                 units-and-scaling · coordinate-frames · verifying-simulations ·
                                 research-and-diagnosis · recipes · known-issues
  building-and-running.md        ode-config, the c++-driver link, precision, the header-verify recipe
  rendering-and-headless.md      headless self-check harness · drawstuff · the macOS GLUT-backend recipe
  cpp-patterns.md                C-API-from-C++: fn-ptr callbacks, move-only RAII (+ teardown-order guard), controllers
  world-and-stepping · bodies-and-mass · joints · collision-and-contacts · geoms-and-spaces · …  (core API)
  components/                    one card per joint type and geom type (the design-system layer)
  examples/                      cited walkthroughs of tested programs
  (deep)                         math-and-rotation · threading · trimesh-heightfield · build-and-backends · …
assets/                          paste-ready Makefile + complete, tested, headless C/C++ programs
                                 (arm6dof, jenga, buggy_ramp, headless_example, probe_sign, glut_backend, …)
scripts/check-citations.py       validate the skill's cites against YOUR installed ODE version
scripts/verify-build.py          build + run every shipped example against YOUR installed ODE (the reproducible gate)
```

## The workflow it induces

`SKILL.md` encodes the workflow that produced every correct field build:
**probe the toolchain → decide collision in/out → read the few relevant cards + grep the *installed* headers
→ design the headless self-check *and its falsification* first → write → build (`c++` + `ode-config`) → run
→ falsify → harden** (move-only RAII, a `g_ode_alive` teardown guard, a `-Wall -Wextra` clean build, ASan/UBSan).

## How it was built & verified

Extracted from the ODE source headers (the `extract-ds-skill` meta-skill discipline: auto-discover →
cite by `file:line` → progressive-disclosure references → reflexive audit), then **hardened across many real
field sessions** — building everything from a one-file probe to a 6-DOF Stewart platform, a chaotic double
pendulum with a 3-oracle verification, and a complete **dependency-free ODE robotics engine** extracted from
[lpzrobots](https://github.com/georgmartius/lpzrobots). Every shipped example program compiles, links, and
runs headless against the installed library.

### Version note

Citations are pinned to the **ODE 0.16** source; validated against **Homebrew 0.16.6**. Line numbers drift
across versions (and the dev source diverges from the release in *both* directions). Trust the **symbol**;
treat the line as approximate. Run **`python3 ode/scripts/check-citations.py`** to see exactly which cited
symbols differ in *your* installed ODE.

## Credits

Built on the **Open Dynamics Engine** by Russell L. Smith and contributors (BSD / LGPL, <https://www.ode.org/>).
This skill is documentation about ODE's public API; consult the ODE license for the library itself.
