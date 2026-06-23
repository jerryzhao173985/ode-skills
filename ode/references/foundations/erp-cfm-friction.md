# Foundations: ERP, CFM, and the friction model (ODE)

> The two solver "master dials" (ERP, CFM) and how ODE approximates friction. Definitions are from the wiki manual; the flag/field names and defaults are grounded in `include/ode/contact.h` and `objects.h`. **Precedence: headers win on names/values; this file explains the meaning.** For *how to tune* see `references/foundations/stepping-and-stability.md`; for the *parameter table* see `references/tokens.md`.

## ERP — Error Reduction Parameter

- **Definition.** ERP is the proportion of joint/constraint *error* the solver corrects in the next step. Each step the solver applies a corrective force scaled by ERP to push bodies back into alignment. _(Manual: Joint error and the ERP)_
- **Range & default.** 0..1; **recommended 0.1–0.8, default 0.2**. ERP=0 → no correction → bodies drift apart; ERP=1 → tries to fix all error in one step but internal approximations prevent it and it tends to be unstable, so **avoid 1.0**. _(Manual; `dWorldSetERP` `objects.h:110`)_

## CFM — Constraint Force Mixing

- **Hard vs soft.** **CFM=0 → hard** constraint (cannot be violated). **CFM>0 → soft**: the constraint can be "pushed through" proportionally, and a small positive CFM also reduces numerical error and improves stability near singular configurations. _(Manual: CFM)_
- **Never negative.** Negative CFM causes instability — *"Don't do it."* _(Manual: CFM)_
- **Defaults depend on precision.** Global CFM defaults to **1e-5 (single)** / **1e-10 (double)**; the single default is non-zero because the single-precision solver needs a touch of softness to stay stable. _(Manual; `dWorldSetCFM` `objects.h:127`)_
- **Spring-damper equivalence.** A constraint with given ERP/CFM behaves like a spring (stiffness `kp`) + damper (`kd`) under implicit integration with step `h`:
  ```
  ERP = h·kp / (h·kp + kd)
  CFM = 1   / (h·kp + kd)
  ```
  Pick `kp`, `kd`, then derive `soft_erp`/`soft_cfm` for a contact with the feel you want. _(Manual: How to use ERP and CFM to simulate springs)_

## Combining two materials (the production recipe)

A material model exposes intuitive per-surface scalars (roughness, hardness `h`, elasticity `e`, slip) and
combines the **two** contacting materials into `dSurfaceParameters` inside the near-callback. lpzrobots'
`Substance` recipe (`lpzrobots/ode_robots/osg/substance.cpp:54-83`):
```
mu       = roughness1 · roughness2                    // PRODUCT — NOT min(mu1,mu2) (that's a Bullet/PhysX convention)
kp       = 100 · h1·h2 / (h1+h2)                      // hardnesses combine as springs in SERIES
kd       =  50 · ((1-e1)·h2 + (1-e2)·h1) / (h1+h2)    // damping from elasticity (1-e = fraction of energy lost)
soft_erp = h·kp/(h·kp+kd);   soft_cfm = 1/(h·kp+kd)   // = the spring-damper law above, evaluated per contact
slip1 = slip2 = slip1 + slip2                         // SUM
mode  = dContactSoftERP | dContactSoftCFM | dContactApprox1   (+ dContactSlip1|dContactSlip2 only if slip > 1e-4)
```
The `100`/`50` are empirical scale factors. A per-geom callback may also veto/override a contact via a
3-value return: `0` = drop it (no `dJointCreateContact` — a non-colliding geom), `1` = fall through to the
default combine, `2` = "I already wrote `surface`, use as-is" (`substance`/`simulation.cpp:1384-1398`).

**Anisotropic friction (snake belly / tracks): build `fdir1` per contact, every step.** `fdir1` is not a
fixed world vector — recompute `fdir1 = bodyLongAxis × contact.normal` (normalize; fall back to any tangent
when the axis is ∥ the normal), set `mode |= dContactFDir1 | dContactMu2`. The **HIGH** coefficient `mu` rides
on `fdir1` (the **cross-body** direction); the **LOW** `mu2 = mu·ratio` (ratio<1) is on direction 2
(`= normal × fdir1`, **along** the body's travel axis) — so friction is **low along the body's motion, high
across it** (a snake slides forward and grips sideways). (`substance.cpp:204,221`; the doc states "friction
along the axis is ratio-fold of the other directions", `substance.h:107-114`.)

## The friction model

- **Coulomb cone.** Tangential friction is bounded by the normal force: |F_T| ≤ μ·|F_N|; admissible forces form a cone about the contact normal. Solving the exact cone is expensive, so ODE approximates it. _(Manual: Friction Approximation)_
- **Box vs pyramid approximation:**
  - **Default — box friction (`dContactApprox0`, 0x0000):** `mu` is the *maximum friction force* directly. Non-physical (independent of normal force) but cheapest, and can lock rolling bodies.
  - **Pyramid (`dContactApprox1`, 0x7000):** ODE computes normal forces first, then caps tangential force at `mu·|F_N|`; here `mu` is the ordinary unit-less Coulomb ratio. `dContactApprox1_1`/`_2` enable the ratio for friction direction 1/2 independently. _(Manual; `contact.h:47-51`)_ — **The FAQ recommends `dContactApprox1`** so friction scales with normal force (otherwise light rolling bodies can lock).
  - **The pyramid bound is a *rectangle*, not a circle.** Because `mu`/`mu2` cap friction directions 1 and 2 *independently*, the admissible tangential force is a box in the friction plane — so friction is up to ~√2 stronger along a diagonal than along an axis, and a sliding body tends to **drift until it aligns with a principal friction axis** (`fdir1`). Keep `mu == mu2` for isotropic friction, and don't rely on diagonal sliding staying straight. *(The rectangular per-direction bound is header-consistent — `contact.h:48-49`; the √2-diagonal / axis-drift artifact is wiki/ecosystem-observed, not header-stated.)*
- **`mu` / `mu2`.** `surface.mu` (always defined) is the coefficient for friction direction 1; `dContactMu2` enables `surface.mu2` for direction 2 (anisotropic friction). Range 0 (frictionless) … `dInfinity` (no sliding). _(Manual; `contact.h:34,58,61`)_
- **`fdir1`.** With `dContactFDir1`, `dContact.fdir1` sets the first friction direction (a unit vector in the contact tangent plane); direction 2 = normal × fdir1. Needed for anisotropic friction or conveyor-belt surface motion; otherwise ODE picks a direction automatically. _(Manual; `contact.h:36,102`)_
- **Contact slip (`slip1`/`slip2`).** Force-dependent slip lets a contact creep proportionally to tangential force — effectively a per-direction CFM on the friction constraint. A recommended QuickStep-stability tip. _(Manual; `contact.h:43-44,70`)_

## Surface-mode flag table (`dContact.surface.mode`)

`mode` and `mu` are **always** read; every other surface field is read only when its bit is OR'd into `mode` (`contact.h:60`). For the full per-bit table (`dContactMu2` … `dContactApprox1`, `contact.h:34-51`) see the owner: `references/collision-and-contacts.md` (surface.mode bitflags).

## A consequence worth knowing

The contact-joint normal row uses LO=0, HI=∞, so a contact can only **push outward** — bodies can never perfectly stick, and a contact always restores a little energy even at `bounce=0` ([odedevs/ode#88]; see `references/foundations/known-issues.md`).

## See also

This file is the OWNER of the friction-MODEL theory (Coulomb cone, box-vs-pyramid, `dContactApprox1` semantics) and the ERP/CFM spring-damper math (`erp = h·kp/(h·kp+kd)`, `cfm = 1/(h·kp+kd)`); link here for the *why*. For the *how*: `references/collision-and-contacts.md` (filling `dContact`/`dSurfaceParameters` + the full `surface.mode` bitflag table), `references/foundations/stepping-and-stability.md` (tuning recipes), `references/tokens.md` (defaults/ranges as a token table).
