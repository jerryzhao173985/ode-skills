# The ODE mental model for building (read this first)

> The single most important thing to get right is not an API call — it is the **shape of the program**.
> Field-tested: across many real builds, the agents who got this model right wrote correct programs on the
> first try; the ones who pattern-matched a single "canonical loop" bolted on machinery they didn't need.

> **The deeper lens: ODE is a constraint solver over rigid bodies.** Joints, motors, contacts, loop closures,
> and friction are all *constraints* with different lifetimes — joints persistent · motors impose a target each
> step · contact joints last one step · a joint can be created or destroyed mid-sim (lifetime is a tool, not a
> fixed property). Most ODE failures are constraint stiffness/softness (ERP/CFM), timestep, solver convergence,
> or frame/sign — not wrong API calls.

## 1. ODE is TWO independent subsystems you compose

The public headers split cleanly, and so should your mental model:

| Subsystem | Owns | Header | What it does |
|---|---|---|---|
| **Dynamics** | `dWorldID` ⊃ `dBodyID` (+ `dMass`) connected by `dJointID` | `objects.h`, `mass.h` | integrates motion; enforces joint constraints & motors |
| **Collision** | `dSpaceID` ⊃ `dGeomID` (a shape) | `collision*.h`, `contact.h` | finds overlapping pairs; you turn each into a contact |

- A **`dBody`** has mass + motion, **no shape**. A **`dGeom`** has a shape, **no mass**.
- The **bridge** is `dGeomSetBody(geom, body)` — it welds a collision shape to a dynamics body so the shape
  follows the physics. The *other* direction (collision pushing back on dynamics) happens through **contact
  joints**: per step you run `dCollide`, and for each contact you create a throwaway `dJointCreateContact`
  in the dynamics world.

- **A one-question test for every new ODE fact: is it *dynamics*, *collision*, or *the bridge* (`dGeomSetBody`)?**
  Every rule sits on one side or on the bridge; if you can't place it, you don't understand it yet.

That's the whole architecture. Everything else is detail.

**The six objects, one line each:** **world** (`dWorldID`) — the dynamics container (gravity/ERP/CFM), owns
the bodies, you step it; **body** (`dBodyID`) — mass + inertia + pose + velocity, **no shape**; **geom**
(`dGeomID`) — a collision shape, attached to a body or static (body `0`); **space** (`dSpaceID`) — broadphase
container for geoms (is itself a geom; nestable); **joint** (`dJointID`) — a constraint between two bodies
(or a body and the static world), optionally motored/limited; **contact** — a *transient* contact joint
created per touching pair each step, then discarded.

## 2. The build decision: do you even need collision?

**This is the question that decides your program's shape.** Ask: *does anything need to physically touch?*

- **NO — a mechanism held together by joints** (pendulum, robot arm, Stewart platform, gear train, linkage):
  use **dynamics only**. Create a world, bodies, masses, and joints; step. There is **no `dSpace`, no
  `dGeom`, no near-callback, no contact-joint group.** Omitting them is correct — bolting them on is
  cargo-cult code that does nothing.
- **YES — things collide** (a falling stack, a dropped object, a vehicle on terrain, a marble in a funnel):
  add the **collision subsystem**: a space, geoms linked to bodies with `dGeomSetBody`, a near-callback that
  builds contact joints, and `dJointGroupEmpty` each step.

> A robot arm reaching a target needs **zero** collision code. A box dropped on the floor needs **both**
> subsystems. Decide this before you write a line.

## 3. The two loop shapes (pick one)

```c
/* PURE-DYNAMICS loop — a jointed mechanism, nothing collides */
for (int i = 0; i < STEPS; i++) {
    applyControls();                         // set joint motor params / add forces
    if (!dWorldStep(world, DT)) break;       // accurate LCP; check the return
}

/* WITH-COLLISION loop — things touch */
for (int i = 0; i < STEPS; i++) {
    applyControls();
    dSpaceCollide(space, &state, &nearCallback);   // -> dCollide -> dJointCreateContact -> dJointAttach
    if (!dWorldQuickStep(world, DT)) break;        // fast iterative; pair with auto-disable
    dJointGroupEmpty(contactgroup);                // contacts are valid for ONE step
}
```

The famous "collide → step → empty" rhythm is the **second** shape only. If your links never touch, the
first shape is the whole loop.

## 4. The canonical build order (what every correct build did)

1. **Toolchain first.** `ode-config --cflags --libs` + the **`c++`** driver; define **no** precision macro;
   gate at runtime with `dCheckConfiguration("ODE_double_precision")`. (`references/building-and-running.md`)
2. `dInitODE2(0)` → `dAllocateODEDataForThread(dAllocateMaskAll)` (if colliding) → `dWorldCreate`
   → (`dHashSpaceCreate` + `dJointGroupCreate` **only if colliding**).
3. **World tokens BEFORE bodies** — gravity, ERP, CFM, auto-disable, damping are defaults copied at body
   creation (`references/tokens.md`).
4. Bodies: `dBodyCreate` → `dMassSet*` + `dBodySetMass` (every dynamic body needs a real mass)
   → (`dCreate*` + `dGeomSetBody` **only if colliding**).
5. Joints: `dJointCreate*` → `dJointAttach` → set anchor/axis → set stops (in that order; setting the axis
   re-zeros the angle).
6. The loop (§3) — apply controls **before** the step; check the step's int return.
7. Teardown in reverse: `dJointGroupDestroy` → `dSpaceDestroy` (frees its geoms) → mesh/heightfield data
   destroy **after** the geoms → `dWorldDestroy` → `dCloseODE`.

## 5. Two choices that matter

- **Stepper.** Small jointed chain → `dWorldStep` (accurate Dantzig LCP; energy stays flat, so energy drift
  becomes a *meaningful* correctness signal). Large contact-heavy scene → `dWorldQuickStep` + auto-disable
  (fast SOR; accuracy bounded by iteration count). Most "the physics looks wrong" is **solver convergence,
  not a wrong API call**.
- **Timestep.** Always **fixed** `DT`; never a wall-clock step into an ERP/CFM-tuned solver.

## 6. Verify, don't trust — it is part of building

Build **headless** and make the program **grade itself** with checks that can actually FAIL (energy that
can't grow, two-phase transient/rest invariants, a falsifiable target, determinism). "It compiled and ran"
is the weakest possible evidence. Full method: `references/foundations/verifying-simulations.md`. Calibrate
any uncertain sign/axis convention **empirically** (a tiny probe), never by assumption.

## 7. Sources: headers are truth, grep YOUR install

This skill's `file:line` cites are against the **0.16** repo; your installed lib is **0.16.x** and line
numbers drift. So: **trust the symbol and its semantics; treat the line number as approximate** and re-grep
the installed header when an exact line matters (recipe in `references/building-and-running.md` §6). The
component cards earn their keep not by the signature (a 1-line grep gives that, accurately) but by the
**ordering, units, conventions, read-back meaning, and gotchas** grep can't give.

## The workflow in one line

**Decide collision? → probe toolchain → read the few relevant cards + grep installed headers → design WITH
the self-check first → write → build → run → falsify (prove it can FAIL) → clean rebuild.** That order is
what separated the correct builds from the plausible ones.

## Glossary (the jargon, one line each)

- **ERP** — Error Reduction Parameter: fraction of joint/contact error corrected per step (0.1–0.8, default 0.2).
- **CFM** — Constraint Force Mixing: constraint *softness*; a small positive value stabilizes (default ~1e-5); 0 = hard.
- **broadphase / narrowphase** — broadphase (`dSpaceCollide`) finds candidate geom *pairs* cheaply; narrowphase (`dCollide`) computes the exact contacts for a pair.
- **contact joint** — a *transient* joint created per contact point each step (in a group emptied every step) — how collision pushes back on dynamics.
- **auto-disable** — ODE sleeps a body that has been near-stationary for a while (skips it) until something disturbs it.
- **LCP** — Linear Complementarity Problem: the constraint system `dWorldStep` solves *exactly* (accurate, ~O(m²) memory).
- **SOR / iterative** — `dWorldQuickStep`'s successive-over-relaxation solver: fast, accuracy bounded by the iteration count.
- **island** — a connected group of bodies (sharing joints/contacts) ODE solves independently; islands parallelize.
- **placeable geom** — a geom welded to a body via `dGeomSetBody`; a static geom has body `0`.
- **kinematic body** — a body you move directly (`dBodySetKinematic`): it affects others but is *not* moved by forces.
