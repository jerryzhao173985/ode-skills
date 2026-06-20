# Contacts and Friction

The conceptual "how contact and friction work in ODE" narrative — sliding,
sticking, and rolling. For the API mechanics (filling `dContact`, the full
`surface.mode` flag table, the canonical `nearCallback`, array sizing) see
`references/collision-and-contacts.md`; for the friction-MODEL theory and the
ERP/CFM spring-damper math see `references/foundations/erp-cfm-friction.md`.
This file carries only the prose neither owner has.

The contact pipeline is always: `dSpaceCollide` -> `nearCallback` -> `dCollide`
-> fill a `dContact` -> `dJointCreateContact` -> `dJointAttach` -> step -> empty
the contact group. Contact joints exist for exactly one step.

---

## The contact descriptor

`dCollide` writes geometry into the embedded `dContactGeom`; you fill the
`dSurfaceParameters surface`; the two travel together inside a `dContact` that
you hand to `dJointCreateContact`. The struct layouts (`dContactGeom`,
`dContact`, `dSurfaceParameters`) are documented in
`references/collision-and-contacts.md` (Key data structures).

Two conventions worth internalizing:

- The `normal` points INTO body 1: moving body 1 along the normal by `depth`
  reduces the penetration to zero. Cite: `include/ode/contact.h:88-94`.
- `surface.mode` + `surface.mu` are read EVERY time; every other
  `dSurfaceParameters` field is read only when its `mode` flag is set. Set the
  bit AND the field. Cite: `include/ode/contact.h:55-71`.

---

## The friction model in one breath

ODE uses the Coulomb model: tangential friction is bounded by
`|F_T| <= mu * |F_N|` (a friction cone), approximated as a friction PYRAMID. The
default `dContactApprox0` box reinterprets `mu` as an absolute max force
(non-physical, can lock rolling bodies); the `dContactApprox1` (`0x7000`) pyramid
keeps `mu` a true unitless Coulomb ratio. For the full box-vs-pyramid theory and
`dContactApprox1` semantics see `references/foundations/erp-cfm-friction.md` (the
friction-model owner).

Practical conceptual notes:

- Keep `mu` a unitless ratio (around ~1.0) and set `dContactApprox1` so friction
  scales with normal force. Cite: `include/ode/contact.h:47-51`.
- `dInfinity` for `mu` requests effectively infinite friction (no sliding within
  the contact's force budget) — common for "sticky" resting contacts in the demos.

### Field-confirmed caveats neither owner states

- HEADER MISLABEL — `dContactApprox1_N` (`0x4000`) carries the comment
  `/**< For rolling friction */`, but EMPIRICALLY setting that bit alone does NOT
  give a rolling sphere a true Coulomb friction pyramid and does NOT by itself
  make it settle / auto-disable. `_N` is one of the three pyramid sub-bits; for a
  true friction pyramid set the FULL composite `dContactApprox1` (`0x7000` =
  `dContactApprox1_1 | dContactApprox1_2 | dContactApprox1_N`). Cite (header text):
  `include/ode/contact.h:47-51`; settling is observed/empirical (header-silent).
- ROLLING/SPINNING FIELDS GATED BY `dContactRolling` — `surface.rho` (rolling),
  `surface.rho2`, and `surface.rhoN` (spinning) are read ONLY when
  `dContactRolling` (`0x400`) is set in `surface.mode`; NO `dContactApprox1*` bit
  enables them. Without the flag the `rho`/`rho2`/`rhoN` values are silently
  ignored. Cite: `include/ode/contact.h:45` (flag), `include/ode/contact.h:62-64`
  (fields).
- SURFACE-MOTION SIGN IS o1/o2-ORDER DEPENDENT — `surface.motion1` /
  `surface.motion2` / `surface.motionN` (conveyor/belt target surface velocities,
  read when `dContactMotion1` `0x020` / `dContactMotion2` `0x040` / `dContactMotionN`
  `0x080` is set) are signed relative to geom2-vs-geom1, so they FLIP with the
  broadphase `o1`/`o2` order that `nearCallback` receives (which geom is `o1` is not
  guaranteed). The precise rule (NOT a blanket sign swap): when the belt is
  `contact.geom.g1`, negate `motionN` and `motion2` but **NOT** `motion1` — `fdir1` is
  an author-chosen vector, so its component keeps the sign you set. Recipe for motion in
  any direction: `dPlaneSpace(contact.geom.normal, d1, d2)`; `fdir1 = d1`;
  `motion1 = dot(vel,d1)`; `motion2 = inv*dot(vel,d2)`; `motionN = inv*dot(vel,normal)`,
  with `inv = -1` only when `g1` is the belt — otherwise a first run can drive objects
  UPSTREAM. Cite (flags + fields): `include/ode/contact.h:40-41, 69`; the sign rule and
  the projection recipe are documented in `ode/demo/demo_motion.cpp:142-143, 154-165`
  (it is NOT header-silent). `dPlaneSpace`: `references/math-and-rotation.md`.

---

## Soft contacts, bounce, and slip — the concept

A contact joint defaults to the world's global ERP/CFM. `dContactSoftERP`
(`0x008`) / `dContactSoftCFM` (`0x010`) override that per contact, reading
`surface.soft_erp` / `surface.soft_cfm` instead. Cite:
`include/ode/contact.h:38-39`. ERP+CFM together model an implicit first-order
spring-damper; pick a spring `kp` and damping `kd` and derive `soft_erp` /
`soft_cfm` — the derivation lives in
`references/foundations/erp-cfm-friction.md`. `CFM=0` is a hard constraint,
positive CFM softens it, negative CFM is unstable — never use it.

`dContactBounce` (`0x004`) makes `surface.bounce` the restitution (0..1) and
`surface.bounce_vel` the minimum incoming normal velocity required to bounce.
Cite: `include/ode/contact.h:37, 65-66`. Slip (`dContactSlip1` `0x100` /
`dContactSlip2` `0x200` with `surface.slip1` / `surface.slip2`) adds
force-dependent slip in the friction directions and is a recommended mitigation
for near-singular QuickStep systems (e.g. high-friction articulated rigs).
Cite: `include/ode/contact.h:43-44, 70`.

The full per-bit `surface.mode` table and the `nearCallback` that wires this
together live in `references/collision-and-contacts.md`.
