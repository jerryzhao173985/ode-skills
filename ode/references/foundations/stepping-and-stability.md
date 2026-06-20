# Stepping and stability

How you advance time, which stepper you pick, and the global knobs (ERP, CFM, iteration
count, auto-disable) that decide whether a simulation rests or jitters and explodes.
Defaults below are quoted from the in-repo headers and the official wiki; do not
substitute other numbers.

## Fixed timestep

ODE was designed with **fixed step-sizes** in mind. Use one constant `stepsize`
throughout your application. The wiki's "Variable Step Size: Don't!" section calls a
variable stepsize "a one-way trip to a headache": a resting body stabilizes at a small
penetration depth, and varying the step shifts that equilibrium each step, causing
jitter and possible spurious energy gain. (wiki
<https://ode.org/wiki/index.php?title=Manual> section 5.2.1 Variable Step Size: Don't!)

Two consequences:

- **Smaller step = more accurate AND more stable.** Step composition is
  non-associative: 10 steps of 0.1 is *not* the same as 5 steps of 0.2. So tune every
  parameter (ERP, CFM, contacts) at the *final* frame rate you will ship with. (wiki
  <https://ode.org/wiki/index.php/HOWTO_make_the_simulation_better> Behavior depends on
  step size)
- The wiki gives a *rule* ("fixed, constant, smaller is better"), not a magic number.
  `1.0/60.0` is a common example, **not** a documented default. Pick a fixed `h` and
  keep it constant; if your render rate varies, accumulate real time and run zero or
  more fixed sub-steps per frame.

## dWorldStep vs dWorldQuickStep

ODE ships two built-in steppers. They have the same signature and you call exactly one
per step.

- **`dWorldStep(world, h)`** — the "big-matrix" method. Time on the order of `m^3` and
  memory on the order of `m^2`, where `m` is the total number of constraint rows. "For
  large systems this will use a lot of memory and can be very slow, but this is
  currently the most accurate method." Returns 1 on success, 0 on allocation failure
  (state unchanged, retryable). (`include/ode/objects.h:384`)
- **`dWorldQuickStep(world, h)`** — an iterative SOR method. Time on the order of `m*N`
  and memory on the order of `m`, where `N` is the iteration count. "For large systems
  this is a lot faster than dWorldStep(), but it is less accurate." It is "great for
  stacks of objects especially when the auto-disable feature is used as well." However,
  "it has poor accuracy for near-singular systems" — high-friction contacts, motors, or
  certain articulated structures (e.g. a multi-legged robot sitting on the ground).
  (`include/ode/objects.h:427`)

The trade-off in one line: **`dWorldStep` for accuracy and small systems;
`dWorldQuickStep` for speed and large/stacked systems**, accepting lower accuracy.
(`include/ode/objects.h:367-427`)

When QuickStep misbehaves on a near-singular system, the header lists the fixes
directly: increase CFM; reduce the number of contacts (use the minimum number of
contacts for feet); don't use excessive friction; use contact slip if appropriate;
avoid kinematic loops; don't use excessive motor strength; use force-based motors
instead of velocity-based motors. "Increasing the number of QuickStep iterations may
help a little bit, but it is not going to help much if your system is really near
singular." (`include/ode/objects.h:397-415`)

## Iteration count (QuickStep)

`dWorldSetQuickStepNumIterations(world, num)` sets how many SOR iterations QuickStep
performs per step. The default is `dWORLDQUICKSTEP_ITERATION_COUNT_DEFAULT`, which is
**20**. More iterations give a more accurate solution but take longer to compute — and
won't rescue a truly near-singular system. (`include/ode/objects.h:451`, `462`)

**Stack instability scales with HEIGHT, not mass.** The SOR sweep propagates the supporting
contact force roughly *one contact-row per iteration*, so a *tall* stack starves the iteration
budget where a merely *heavy* one does not: field-measured, an 8-box stack jitters identically at
per-box mass 5 vs 50 kg, while at the default 20 iterations a 20-box stack collapses outright. The
lever is **iteration count** (or switch to `dWorldStep`), not lighter bodies — and the visible
"sink" is usually inter-box *compression* (the top descends), not the bottom box dropping through
the floor. (Field-verified against ODE 0.16.6.)

## Tuning ERP and CFM for stability

For the *definitions*, ranges/defaults, and the spring-damper derivation
(`ERP = h·kp/(h·kp+kd)`, `CFM = 1/(h·kp+kd)`), see the owner:
`references/foundations/erp-cfm-friction.md`. The tuning rules:

- **When a system misbehaves, raise the global CFM first** (`dWorldSetCFM`). Positive
  CFM softens the constraint, moves the system away from singularity, and improves the
  factorizer's accuracy and stability. Never use negative CFM. (`include/ode/objects.h:127`)
- **Keep ERP in the stable band** (`dWorldSetERP`): leave it near the 0.2 default and
  inside 0.1–0.8. ERP = 0 lets joints drift apart; ERP = 1 is unstable.
  (`include/ode/objects.h:110`)
- **Model stiff springs as soft constraints, not explicit forces.** Derive
  `soft_erp` / `soft_cfm` from your desired `kp`/`kd` at your fixed `h` and apply them
  per-contact (`dContactSoftERP` / `dContactSoftCFM`) or per-joint stop
  (`dParamStopERP` / `dParamStopCFM`). Per-contact/per-joint values override the global
  ones. (`include/ode/contact.h:38-39`)

## Contact parameters: anti-jitter and anti-popping

Two global contact knobs are the *intended* fixes for the two classic contact
pathologies — reach for these before abusing friction or ERP/CFM:

- **`dWorldSetContactSurfaceLayer(world, depth)`** — depth contacts may sink into the
  surface before coming to rest. Default **0**. Increasing it to some small value (e.g.
  **0.001**) helps prevent jittering from contacts being repeatedly made and broken.
  This is the canonical anti-jitter setting. (`include/ode/objects.h:623`)
- **`dWorldSetContactMaxCorrectingVel(world, vel)`** — caps the velocity contacts may
  generate to push penetrating bodies apart. Default **infinity** (no limit). Reducing
  it prevents "popping" of deeply embedded objects. (`include/ode/objects.h:603`)

## Damping

Damping bleeds off energy so bodies settle (and so auto-disable can fire). The
world-default linear damping scale is set with `dWorldSetLinearDamping(world, scale)`
(paired with `dWorldSetAngularDamping`). The default scale is **0** (no damping), and
should be in the interval `[0, 1]`. After each step a body's velocity above the damping
threshold is multiplied by `(1 - scale)`. The default damping threshold is **0.01**.
Damping is applied *after* the stepper so it does not disturb the joint constraints.
(`include/ode/objects.h:826`; wiki <https://ode.org/wiki/index.php?title=Manual> World:
Damping)

## Auto-disable

`dWorldSetAutoDisableFlag(world, do_auto_disable)` sets the default auto-disable flag
inherited by newly created bodies (default is **false**, `include/ode/objects.h:748`);
`dBodySetAutoDisableFlag` is the per-body toggle (`include/ode/objects.h:986`).
Auto-disable lets bodies that have settled below their linear/angular velocity
thresholds stop being updated, saving CPU — especially powerful with `dWorldQuickStep`
on stacks. The island rule governs re-enabling: "If there are any enabled bodies in an
island then every body in the island will be enabled at the next simulation step" — so
an island stays disabled only if every member is disabled, and a contact with an enabled
body re-enables the whole island. (wiki
<https://ode.org/wiki/index.php?title=Manual> Islands and Disabled Bodies)

## Units and scaling

Out-of-band magnitudes lose precision in the LCP factorizer and surface as instability,
so keep lengths/masses in the magnitude window and scale gravity and the time step to
match. Owner: `references/foundations/units-and-scaling.md`.

## Rules

- **Use one fixed, constant `stepsize` everywhere.** A variable stepsize shifts the
  rest-penetration equilibrium and causes jitter / energy gain. (wiki
  <https://ode.org/wiki/index.php?title=Manual> section 5.2.1 Variable Step Size:
  Don't!)
- **Smaller step = more accurate and more stable; tune all parameters at your final
  frame rate** (step composition is non-associative). (wiki
  <https://ode.org/wiki/index.php/HOWTO_make_the_simulation_better> Behavior depends on
  step size)
- **`dWorldStep` for accuracy / small systems; `dWorldQuickStep` for speed / large /
  stacked systems** (accepting lower accuracy). (`include/ode/objects.h:367-427`)
- **For a near-singular QuickStep system, fix the singularity** (increase CFM, fewer
  contacts, less friction, contact slip, no kinematic loops, force-based motors) — don't
  just crank iterations. (`include/ode/objects.h:397-415`)
- **Raise the QuickStep iteration count for accuracy** (default 20), but it won't fix a
  truly near-singular system. (`include/ode/objects.h:451`)
- **If misbehaving, raise the global CFM first; keep ERP near 0.2 (0.1–0.8) and never
  use negative CFM.** Definitions/ranges live in
  `references/foundations/erp-cfm-friction.md`. (`include/ode/objects.h:110`, `127`)
- **Model springs as soft constraints, not stiff explicit forces** — derive
  `soft_erp`/`soft_cfm` from `kp`/`kd` at your fixed `h` (owner:
  `references/foundations/erp-cfm-friction.md`). (`include/ode/contact.h:38-39`)
- **Kill jitter with a small contact surface layer (~0.001) and a finite max correcting
  velocity**, not by abusing friction. (`include/ode/objects.h:623`, `603`)
- **Use damping + auto-disable** to let settled bodies rest and stop consuming CPU; an
  island stays disabled only if every member is disabled. (`include/ode/objects.h:826`,
  `748`)
- **Keep lengths/masses in the magnitude window and scale gravity and the time step to
  match** (owner: `references/foundations/units-and-scaling.md`). (wiki
  <https://www.ode.org/wiki/index.php?title=FAQ> Should I scale my units to be around
  1.0?)

## Common mistakes

| Bad | Good | Why |
| --- | --- | --- |
| Stepping with a per-frame wall-clock `dt` that varies every frame (`dWorldStep(w, realDeltaTime)`). | Step with a fixed constant `h` (e.g. `1.0/60.0`); accumulate real time and run zero or more fixed sub-steps per frame. | Variable `dt` shifts the stable resting-penetration depth each step, causing jitter and possible energy gain; ODE was designed for fixed steps. (wiki <https://ode.org/wiki/index.php?title=Manual> section 5.2.1) |
| Using `dWorldQuickStep` on a stiff/near-singular rig (strong velocity motors, many high-friction contacts, kinematic loops) and just cranking iteration count. | Use `dWorldStep` for accuracy, or reduce contacts/friction, increase CFM, use force-based motors and contact slip. | QuickStep is iterative and explicitly inaccurate near singularities; raising iterations alone won't fix it. (`include/ode/objects.h:397-415`) |
| Driving a body by applying your own large stiff spring force every step. | Use a powered joint / motor, joint limits, or ODE's built-in soft constraint (`soft_erp`/`soft_cfm` from `kp`/`kd`; derivation in `references/foundations/erp-cfm-friction.md`). | Stiff explicit forces are the classic instability source with a first-order integrator; constraints are integrated implicitly and stay stable. (`include/ode/contact.h:38-39`) |
| Using negative CFM, or pushing ERP/CFM to extremes to stop jitter. | Keep ERP 0.1–0.8 and CFM 1e-9 .. 1; kill jitter with a small contact surface layer (~0.001) and a finite max correcting velocity. | Negative CFM causes instability; surface layer and capped correcting velocity are the intended anti-jitter / anti-popping knobs. (`include/ode/objects.h:127`, `623`, `603`) |
| Simulating raw real-world units (a 0.005 m / 0.002 kg part, or a 2000 m / 50000 kg vehicle) and then fighting explosions. | Scale the whole system so lengths/masses sit around 0.1 .. 10 (ideally 0.1 .. 1.0), and scale gravity and the time step to match. | Tiny/huge magnitudes lose precision in the LCP factorizer (worse in single precision), surfacing as instability; rescaling restores numerical headroom. (wiki <https://www.ode.org/wiki/index.php?title=FAQ> Should I scale my units to be around 1.0?) |
| Leaving auto-disable off and never damping, so everything is integrated forever. | Enable auto-disable and add a little damping so settled bodies come to rest and drop out of the solver. | Auto-disable + damping let islands stop consuming CPU once settled (any enabled body re-enables the whole island). (`include/ode/objects.h:748`, `826`) |
