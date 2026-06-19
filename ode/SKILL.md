---
name: ode
description: >-
  Build rigid-body physics simulations with the Open Dynamics Engine (ODE) in C or C++ — a C/C++
  dynamics + collision library (worlds, bodies, geoms, spaces, joints, contacts, mass, rotation, trimesh,
  threading, and the optional drawstuff debug renderer). Use when the user asks to simulate rigid bodies,
  set up an ODE world/body/joint/collision pipeline, build a vehicle/ragdoll/stack, port or debug an ODE
  program, or wire collision detection. Triggers: 'ODE', 'Open Dynamics Engine', 'dWorldStep',
  'dBodyCreate', 'dGeomSetBody', 'dJointCreateHinge', 'rigid body simulation', 'physics engine in C/C++'.
  Scope: the public C API in include/ode/*.h (used from C or C++), the C++ wrapper in odecpp*.h, the
  drawstuff renderer, and the real toolchain (ode-config, the c++-driver link, headless verification).
  Out of scope: ODE internals (ode/src) and the GIMPACT/OPCODE/libccd backend internals (surveyed, not
  documented). IMPORTANT: this file is an orchestrator — load the references/ files in the routing table;
  SKILL.md alone does not carry the full API detail.
---

# ODE — Open Dynamics Engine (C/C++ usage skill)

ODE is *"a free, industrial quality library for simulating articulated rigid body dynamics … with advanced
joints, contact with friction, and built-in collision detection"* (`README.md:9-13`). It is a C library
(usable directly from C++) with an optional thin C++ wrapper.

**The public C API headers in `include/ode/*.h` are canonical truth.** Every rule cites a real `file:line`.
The wiki and web tutorials lose to the headers on conflict (they reference removed APIs). **Do not invent
functions, params, enums, or surface fields — if `include/ode/*.h` does not declare it, it does not exist.**
> **Cites are 0.16-repo line numbers — trust the SYMBOL, treat the line as approximate.** Your installed ODE
> (e.g. Homebrew 0.16.6) drifts ~⅓ of the line numbers, and the two versions differ **bidirectionally**: a few
> APIs are repo-only (the QuickStep dynamic-iteration setters), a few are 0.16.6-only (the
> `dWorldSetSteppingThreadingParameters` threading struct). Re-grep your installed header for an exact line, or
> run `python3 scripts/check-citations.py` to see exactly what differs in YOUR install.

> **"C++ ODE" in practice = the C API written in a `.cpp`, built with the `c++` driver.** That is what every
> real ODE program here does; the `odecpp.h` wrapper is optional sugar (`references/cpp-api.md`). The C++
> work that matters is the *build* (`references/building-and-running.md`), not a different API.

## The mental model (read first — full version in `references/foundations/mental-model.md`)

ODE is **two independent subsystems you compose**, not one monolith:
- **Dynamics** — `dWorldID` ⊃ `dBodyID` (mass + motion, **no shape**) + `dMass`, connected by `dJointID`.
  Simulates a mechanism on its own.
- **Collision** — `dSpaceID` ⊃ `dGeomID` (a shape, **no mass**). `dSpaceCollide`→`dCollide` finds overlaps;
  you turn each into a per-step throwaway **contact joint**. `dGeomSetBody(geom, body)` is the bridge — the
  most important call in the API.
- **Collision is CONDITIONAL — decide it before you write the loop.** A mechanism held by joints
  (pendulum, arm, gears, Stewart platform) uses the **dynamics half only**: no space, no geoms, no
  near-callback — bolting them on is cargo-cult. Things that touch (stacks, drops, vehicles) need both
  halves. (`references/foundations/mental-model.md` §2 is the decision tree.)
- Tunable scalars (gravity, ERP, CFM, friction, damping) are the simulation's "tokens" (`references/tokens.md`).

## Setup, build & run — do this first (it's where builds actually fail)

- **Find ODE, don't rebuild it:** `ode-config --cflags --libs` (or `pkg-config ode`; `brew install ode`).
- **libode is a C++ library → link with the `c++` driver** (`-x c` for `.c` sources), or you get C++-runtime
  link errors. Build a C++ program:
  ```sh
  c++ -O2 -std=c++17 $(ode-config --cflags) sim.cpp $(ode-config --libs) -o sim
  ```
- **Precision: define neither `dSINGLE` nor `dDOUBLE`** — `<ode/ode.h>` pulls `<ode/precision.h>` which
  already matches the lib; assert `dCheckConfiguration("ODE_double_precision")` at runtime (`common.h:573`).
- **Rendering is optional and drawstuff is NOT shipped by Homebrew → default to headless** with a
  self-checking harness. Full detail: **`references/building-and-running.md`** and
  **`references/rendering-and-headless.md`** (+ paste-ready `assets/Makefile.example`, `assets/headless_example.c`).

## How to drive this skill (the workflow that produced every correct build)

0. **Probe the toolchain first, code second** — `ode-config --version/--cflags/--libs`, then compile a
   ~10-line program calling `dCheckConfiguration("ODE_double_precision")` to confirm precision before writing
   the sim.
1. **Decide collision IN or OUT** (`references/foundations/mental-model.md`) — it sets the program's shape.
2. **Read the few relevant cards + grep the *installed* headers** for exact signatures; never invent an API.
3. **Design the headless self-check AND its falsification FIRST** — write down what would make it FAIL
   (`references/foundations/verifying-simulations.md`).
4. **Write → build (`c++` + `ode-config`) → run → FALSIFY** — inject a bug and confirm the check flips to FAIL.
5. **Harden** (library / long-lived objects): move-only RAII, a `g_ode_alive` destructor guard, a final
   `-Wall -Wextra` clean build, and an ASan/UBSan run.
6. **Review in a SEPARATE context — don't self-approve.** Hand a read-only reviewer the *claim + the headers*
   (never your own reasoning); it catches the harness bug you can't see — a tautological check, a falsification
   that passes on a confound. Load-bearing, not ceremony (`references/foundations/research-and-diagnosis.md`).

Calibrate any unknown sign/axis/coordinate convention with a tiny isolated **probe**, not memory; and never
ship a causal/mechanism claim you have not grepped in the installed header.

## The simulation loop — the WITH-COLLISION shape (the one thing to get right)

> This is the loop for programs where things **collide**. A **joints-only mechanism** uses the shorter
> **pure-dynamics loop** — no `space`/`dSpaceCollide`/contacts/`dJointGroupEmpty`, just
> `applyControls(); if(!dWorldStep(world, DT)) break;` — see `references/foundations/mental-model.md` §3.

```cpp
#include <ode/ode.h>
static dWorldID world; static dSpaceID space; static dJointGroupID contactgroup;

// nearCallback must be a plain/static function (NOT a capturing lambda — dSpaceCollide takes a C fn ptr).
static void nearCallback(void *data, dGeomID o1, dGeomID o2) {
    dBodyID b1 = dGeomGetBody(o1), b2 = dGeomGetBody(o2);   // 0 == static world
    dContact contact[8];                                    // an ARRAY — dCollide returns many points
    for (auto &c : contact) { c.surface.mode = dContactApprox1; c.surface.mu = 1.0; }
    int n = dCollide(o1, o2, 8, &contact[0].geom, sizeof(dContact));  // stride is sizeof(dContact)!
    for (int i = 0; i < n; i++) {
        dJointID c = dJointCreateContact(world, contactgroup, &contact[i]);
        dJointAttach(c, b1, b2);                            // a contact joint does nothing until attached
    }
}

int main() {
    dInitODE2(0);                                           // 1. init (before ANY other call)
    world = dWorldCreate(); space = dHashSpaceCreate(0);    // 2-3. dynamics + broadphase
    contactgroup = dJointGroupCreate(0);                    // 4. per-step contact joints
    dWorldSetGravity(world, 0, 0, -9.81);                   //    default gravity is (0,0,0)!
    dWorldSetAutoDisableFlag(world, 1);                     //    set world defaults BEFORE making bodies
    dCreatePlane(space, 0, 0, 1, 0);                        //    static ground: a geom with NO body
    dBodyID b = dBodyCreate(world); dBodySetPosition(b, 0, 0, 5);
    dMass m; dMassSetSphereTotal(&m, 1.0, 0.5); dBodySetMass(b, &m);   // a dynamic body needs a mass
    dGeomID g = dCreateSphere(space, 0.5); dGeomSetBody(g, b);         // LINK shape to body
    for (int i = 0; i < 500; i++) {                         // per step, IN THIS ORDER:
        dSpaceCollide(space, 0, &nearCallback);             //   a. broadphase -> contacts
        if (!dWorldQuickStep(world, 0.01)) break;           //   b. integrate (CHECK return; 0=alloc fail)
        dJointGroupEmpty(contactgroup);                     //   c. discard this step's contacts
    }
    dJointGroupDestroy(contactgroup); dSpaceDestroy(space); // teardown (space frees its geoms)
    dWorldDestroy(world); dCloseODE();
}
```
Cited: `dInitODE2` `odeinit.h:119`; `dWorldCreate` `objects.h:52`; `dHashSpaceCreate` `collision_space.h:53`;
`dJointGroupCreate` `objects.h:1835`; `dWorldSetGravity` `objects.h:93`; `dCreatePlane` `collision.h:1045`;
`dBodyCreate` `objects.h:1011`; `dGeomSetBody` `collision.h:105`; `dCreateSphere` `collision.h:921`;
`dSpaceCollide` `collision.h:822`; `dCollide` `collision.h:792`; `dJointCreateContact` `objects.h:1717`;
`dWorldQuickStep` `objects.h:427`; `dJointGroupEmpty` `objects.h:1852`; `dCloseODE` `odeinit.h:227`.

## When to load references

### Common tasks — goal → which references (the fast lane)
| I want to build… | Load these |
|---|---|
| A **vehicle** (car / buggy) | `foundations/recipes.md` · `components/joint-hinge2.md` · `examples/buggy-headless.md` |
| A **stack / pile** of objects | `foundations/stepping-and-stability.md` · `tokens.md` · `examples/boxstack.md` |
| A **ragdoll** / articulated character | `components/joint-ball.md` · `components/joint-hinge.md` · `components/geom-capsule.md` · `examples/ragdoll.md` |
| A **robot arm / manipulator** (motors) | `components/joint-amotor.md` · `cpp-patterns.md` (controllers) · `examples/arm6dof.md` |
| **Terrain** — heightfield or trimesh world | `components/geom-heightfield.md` · `components/geom-trimesh.md` · `trimesh-heightfield.md` |
| A **pendulum / linkage** (joints, no collision) | `foundations/mental-model.md` (pure-dynamics loop) · `components/joint-hinge.md` · `examples/chain1.md` |
| **Ray sensors** / distance queries | `components/geom-ray.md` · `examples/ray-cast.md` |
| A **gearbox** / coupled rotation | `components/joint-transmission.md` |
| **Many bodies / multi-core** | `threading.md` · `foundations/stepping-and-stability.md` |
| **Anything — prove it works** | `foundations/verifying-simulations.md` · `rendering-and-headless.md` |
| **Debug a misbehaving sim / optimize / research** | `foundations/research-and-diagnosis.md` · `foundations/stepping-and-stability.md` |

### Start here (build/run — the most-needed, previously-missing layer)
| Working on… | Load |
|---|---|
| **The mental model — two subsystems, the build decision tree, the build order** | **`references/foundations/mental-model.md`** |
| Compiling/linking, ode-config, the c++-driver requirement, precision | **`references/building-and-running.md`** |
| Verifying with no window; the headless self-check harness; drawstuff; macOS rendering | **`references/rendering-and-headless.md`** |
| The tunable physics parameters (gravity/ERP/CFM/friction/damping/auto-disable) | **`references/tokens.md`** |
| A ready recipe for a common scene (vehicle, stack, ragdoll, …) | `references/foundations/recipes.md` |
| Writing idiomatic C++ — the nearCallback fn-ptr rule, RAII, std::vector, controllers | `references/cpp-patterns.md` |
| Building a multi-body articulated **robot** — parts+joints+motors, composite bodies, the real near-callback | `references/building-robots.md` |
| Verifying a sim actually works (not just "it ran") — energy / two-phase / falsification | `references/foundations/verifying-simulations.md` |
| Pre-flight checklist — the classic ODE bugs | `references/pitfalls.md` |

### Foundations (load by symptom)
| Symptom / topic | Load |
|---|---|
| The mental model / object concepts | `references/foundations/mental-model.md` |
| Jitter, explosion, energy growth — solver stability | `references/foundations/stepping-and-stability.md` |
| Sinking / interpenetration — ERP/CFM tuning | `references/foundations/erp-cfm-friction.md` |
| Sliding / sticking / rolling — the contact friction model | `references/foundations/contacts-and-friction.md` |
| Wrong scale, things float/sink wrong — units & mass | `references/foundations/units-and-scaling.md` |
| Wrong axis / orientation / quaternion | `references/foundations/coordinate-frames.md` |
| Proving a sim is correct (not just that it ran) | `references/foundations/verifying-simulations.md` |
| Measuring / diagnosing / researching — bottleneck, ablation, the review pass | `references/foundations/research-and-diagnosis.md` |
| Known ODE quirks / limitations | `references/foundations/known-issues.md` |

### Core API
| Layer | Load |
|---|---|
| Init/world/stepping/ERP-CFM/auto-disable/damping | `references/world-and-stepping.md` |
| Bodies: pose, velocity, mass, forces/torques | `references/bodies-and-mass.md` |
| Joints overview + dParam* + feedback | `references/joints.md` |
| Collision detection, dCollide, contacts, near-callback | `references/collision-and-contacts.md` |
| Geom shapes + broadphase spaces | `references/geoms-and-spaces.md` |
| Idiomatic C++ over the C API (callbacks, RAII, controllers) | `references/cpp-patterns.md` |

### Components — one file per element (`references/components/`, see `README.md` for the composition map)
`joint-<type>` (ball, hinge, slider, universal, fixed, hinge2, piston, pr, pu, plane2d, amotor, lmotor,
dball, dhinge, transmission) · `geom-<type>` (sphere, box, capsule, cylinder, plane, ray, convex,
heightfield, trimesh).

### Worked examples (`references/examples/`)
Three complete, tested, headless self-checking **C++** programs shipped in `assets/` and walked through here:
**`arm6dof`** (6-DOF arm — AMotor/DLS control), **`jenga`** (stack + wrecking ball + energy check),
**`buggy-headless`** (hinge2 vehicle + trajectory check). Plus: `chain1`, `boxstack`, `buggy`, `ragdoll`,
`trimesh`, `friction`, `feedback`, `heightfield`, `ray-cast`, `constrain-2d`, `bouncing-sphere`.

### Rarely needed (load only on explicit demand — none were loaded across field tests)
`data-types` (precision/handles — usually covered by the build page + hard rules) · `cpp-api` (the
*optional* `odecpp.h` wrapper classes — the C API from C++ is the dominant path; prefer `cpp-patterns.md`) ·
`math-and-rotation` · `threading` (single-thread is the default) · `trimesh-heightfield` (routed from the
geom cards) · `build-and-backends` (trimesh/libccd/threading build options) · `timing-random-dif` ·
`coding-conventions` · `internals-map` (ode/src survey) · `demos-index` · `version-and-changelog` · `ecosystem`.

## Hard rules (each grounded in source / field-verified)

1. **A body is not a geom** — link with `dGeomSetBody(geom, body)` (`collision.h:105`) or shape and dynamics
   never move together.
2. **Init first**: `dInitODE2(0)` (`odeinit.h:119`); for collision each thread calls
   `dAllocateODEDataForThread(dAllocateMaskAll)` (`odeinit.h:177`); end with `dCloseODE()` (`odeinit.h:227`).
3. **Empty the contact-joint group every step** — `dJointGroupEmpty(contactgroup)` after the step
   (`objects.h:1852`), or contacts accumulate and corrupt the sim.
4. **`dCollide` writes an ARRAY** — pass `dContact contact[N]` and `&contact[0].geom` with stride
   `sizeof(dContact)` (`collision.h:792`). A single `dContact` with N requested is a crash.
5. **Fixed timestep**; check the stepper's int return (`0` = alloc fail) (`objects.h:384,427`).
6. **`dWorldStep`** = accurate LCP (small jointed chains, tighter through impact); **`dWorldQuickStep`** =
   fast iterative (large stacks + auto-disable) (`objects.h:384,427`).
7. **Build with the `c++` driver + `ode-config`; define no precision macro**; assert
   `dCheckConfiguration("ODE_double_precision")` (`common.h:573`). See `references/building-and-running.md`.
8. **Default to headless self-checking**; drawstuff is optional and not shipped by Homebrew. See
   `references/rendering-and-headless.md`.
9. **Set world auto-disable/damping BEFORE creating bodies** — they are defaults copied at creation
   (`objects.h:662`). **Rolling primitives (spheres) defeat auto-disable → enable `dContactRolling` + set
   `surface.rho`** (`contact.h`).
10. **Angles are radians**; drawstuff `hpr` is degrees. **Quaternions are w-first** `[w,x,y,z]`.
11. **Box lengths are FULL side lengths** (`collision.h:1001`); a capsule's length **excludes** its caps
    (`collision.h:1050`); `dMassSetCapsule` direction arg `3` = local-Z long axis.
12. **`get*` returns an internal pointer** — don't free/cache it; use `dBodyCopy*` (`objects.h:1083`).
    Hinge/slider **limit stops are set AFTER the axis** (setting the axis re-zeros the angle).

## Things to never invent
Removed/renamed: `dWorldStepFast1`→`dWorldQuickStep`, `dCreateCCylinder`→`dCreateCapsule`,
`dRayCreate`→`dCreateRay`, `dInitODE`→`dInitODE2`. Grep `include/ode/` before using any unfamiliar symbol.
Details: `references/version-and-changelog.md`.

## Final checks (before claiming an ODE program is correct)
- Lifecycle ordered: `dInitODE2` → world/space/contactgroup → bodies+geoms linked by `dGeomSetBody` →
  per-step `dSpaceCollide`→step→`dJointGroupEmpty` → teardown → `dCloseODE`.
- Every dynamic body has a real mass; every collidable body has a geom linked by `dGeomSetBody`.
- `dCollide` given a `dContact` array; contact group emptied each step; `surface.mode`+`mu` set.
- Built with `c++` + `ode-config`, no precision macro, runtime precision assert present.
- Verified **headless** (energy dissipates, joints stay satisfied, no NaN) — not just "it compiled."
- Self-check is **falsifiable** — you injected a bug and watched it flip to FAIL (not just a green run); any
  script parsing the program's output preserves its **exit code** (`./prog | grep` returns grep's, not yours).
- Compiles against `#include <ode/ode.h>` only; final build is **`-Wall -Wextra` clean** (zero warnings).
