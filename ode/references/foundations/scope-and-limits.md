# Scope, limits, and what this skill is (and isn't) validated against

> Read this **before committing a project to ODE**, and to calibrate how much to trust this skill versus
> your own build. This is the skill's honesty layer: when ODE is the wrong tool, where ODE itself is weak,
> and exactly what here has and hasn't been field-verified. A tool used outside its envelope wastes your time
> no matter how good the rest of this skill is.

## 1. When NOT to use ODE — reach for another engine

ODE is a mature, small, free **rigid-body** dynamics + collision library. It is the right tool for articulated
**rigid** mechanisms — vehicles, robots, ragdolls, machines, stacking — when you want a dependency-light,
hackable, CPU engine and you control the timestep (lpzrobots and many research sims chose it for exactly that).
It is the **wrong** tool for:

| You need… | ODE has… | Reach for |
|---|---|---|
| Soft bodies / cloth / rope / deformables | none — ODE's *soft* means soft **constraints** (ERP/CFM), not soft **bodies**; no deformable/cloth/FEM solver | Bullet (soft bodies/cloth), FleX |
| Fluids / smoke / particles | nothing (not a fluid solver) | SPH / FLIP fluid libraries |
| High-fidelity contact, manipulation, RL training | a dated friction-pyramid contact + a less accurate fast solver | **MuJoCo** (the robotics-RL standard), Drake, Bullet |
| Continuous collision (fast / thin objects) | **no CCD** — fast objects tunnel | Bullet / PhysX (built-in CCD), or substep (§2) |
| Moving trimesh-vs-trimesh contact | weak / unstable | primitives or convex hulls for the dynamic bodies; keep trimeshes static |
| GPU or massive scale (100k+ bodies) | single-CPU, island-threaded only (`references/threading.md`) | PhysX / FleX (GPU), Jolt (modern multicore) |

## 2. ODE's hard edges — and the mitigations that exist

- **Tunneling (no CCD).** A body moving faster than its own thickness per step passes through thin geometry.
  Mitigation: **substep** (run the stepper N× per frame at `dt/N`) or use a smaller fixed `dt`; thicken thin
  walls; cap velocities. There is no continuous-collision fallback. *(Standard ODE practice, not a header feature;
  see `references/foundations/stepping-and-stability.md`.)*
- **High mass ratios** (a light part jointed to a heavy one, ≳10:1) are stiff and jitter/explode; soften with
  CFM, raise QuickStep iterations, or rescale masses closer together (`references/foundations/units-and-scaling.md`).
- **Closed kinematic loops** (4-bar linkages, parallel mechanisms beyond a well-conditioned Stewart platform)
  are over-constrained and drift; soften with CFM, add iterations, or model the loop as one body where possible.
  (The Stewart-platform field test is the one closed-chain case validated here.)
- **Trimesh collision:** trimesh-vs-*primitive* is solid (OPCODE/GIMPACT backends); trimesh-vs-trimesh is weak —
  keep trimeshes static and give dynamic bodies primitive or convex-hull geoms.
- Technical quirks (deprecated symbols, precision, axis/quaternion conventions):
  `references/foundations/known-issues.md`, `references/version-and-changelog.md`.

## 3. What this skill HAS and HAS NOT been validated against (calibrate trust precisely)

- **Field-verified (built + ran):** macOS (Apple Silicon) + Homebrew **ODE 0.16.6** + Apple clang, **headless**.
  Every shipped example builds and runs there — reproduce with `python3 scripts/verify-build.py`.
- **Documented but NOT run here:** **rendering** (drawstuff + the GLUT/X11 backends — Homebrew ships no
  drawstuff, so the visual path is *reasoned, not field-proven*); **Linux** (apt `libode`, stock X11 drawstuff)
  and **Windows** builds; ODE versions other than 0.16.6. Cites target the **0.16 dev source**, which drifts
  ~⅓ of the line numbers from 0.16.6 and differs **bidirectionally** — confirm yours with
  `python3 scripts/check-citations.py`.
- **The deepest caveat — provenance.** This skill was authored and hardened by AI agents mining their *own* ODE
  field runs, so its blind spots may be **shared** with the agent using it. **Canonical truth is the ODE header
  doc-comments and the official wiki — not this skill's prose.** When it matters, grep your installed header,
  check `https://ode.org/wiki/index.php?title=Manual`, and re-verify against *your* install. Treat a confident
  skill claim you cannot ground in a header or the wiki as a hypothesis, not a fact.

## 4. The thin frontier (shallow on purpose — field-test before you rely on it here)

- **Control depth.** The skill teaches the ODE *interface* (read state via `dJointGet*`/`dBodyGet*`, write
  `dParamVel`/`dParamFMax`/`dBodyAddForce` **before** the step) and ships two worked controllers
  (`references/examples/arm6dof.md` — damped-least-squares Jacobian; the Stewart-platform servo) plus the
  velocity-servo stability rule (`references/cpp-patterns.md` §4). It does **not** teach control theory
  (PID/LQR/MPC/learned controllers), sensor-noise modeling, or soft-real-time pacing — everything here is
  fixed-step headless.
- **Scale.** 10k+ bodies / heavy contact: the threading recipe (`references/threading.md`) is documented but
  not field-run at that size.
- **Multi-platform rendering, networking / cross-machine determinism, serialization** — out of validated scope.

> A skill that knows its envelope is more useful than one that pretends to none. Until external ground truth
> exists (a human who has shipped ODE, the project's issue tracker, a reference it didn't author), trust the
> **headers and your own build over this skill's prose**, and use the frontier list above to decide where to
> field-test first.
