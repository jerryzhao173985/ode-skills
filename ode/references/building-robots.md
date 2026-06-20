# Building articulated robots on ODE

> A robot is **a bag of rigid PARTS (each a body+geom) + JOINTS connecting them + a sensor/motor I/O the
> controller drives.** This page is the next altitude above the single-mechanism examples — multi-body,
> articulated, actuated. Every pattern here is distilled from a **verified extraction of lpzrobots** (a
> production ODE robotics simulator) into a clean dependency-free engine: it compiled and ran, a
> differential-drive robot drove straight + turned, 11/11 physics invariants passed.

## 1. A part = body + geom + an owner back-pointer

Each rigid part welds a `dBody` (mass+motion) to a `dGeom` (shape), then stores itself ON the geom:

```c
dBodyID b = dBodyCreate(world);
dGeomID g = dCreateBox(space, lx, ly, lz);
dGeomSetBody(g, b);                          // weld shape to body
dMass m; dMassSetBoxTotal(&m, kg, lx, ly, lz); dBodySetMass(b, &m);
dGeomSetData(g, ownerPtr);                   // owner back-pointer (collision.h:75)
```

**`dGeomSetData(geom, owner)` is the production linchpin** (`include/ode/collision.h:75`): in the
near-callback you recover each geom's owner with `dGeomGetData` (`collision.h:84`) to read its **material**,
its category, or who it belongs to. Every real material/filtering system is built on this — the demo
near-callback that only calls `dGeomGetBody` can't do per-geom materials.

## 2. Place parts with pose math — POINT vs DIRECTION

Build everything relative to the robot's `pose`. The one rule that makes a robot place correctly at **any
orientation** (not just at the origin): distinguish points from directions.

- A part's world pose = `localPose * robotPose`.
- A joint **anchor** is a local **POINT** (homogeneous w=1 → rotates *and* translates).
- A joint **axis** is a local **DIRECTION** (w=0 → rotates only, never translates).

Mixing these up gives a robot that's correct at the origin and wrong the moment it's placed rotated — a
silent bug. (Verify by placing the robot under yaw and checking it still assembles.)

## 3. Joints connect parts

Each joint is a thin wrapper over the C API: `dJointCreate<Type>` → `dJointAttach(j, partA, partB)` → set
anchor (a point) and axis (a direction) → set stops. Attach to body `0` to pin a part to the static world.
**Set the anchor/axis AFTER attaching** (setting the axis re-zeros the joint angle). Per-type detail:
`references/components/joint-*.md`; the dParam family + feedback: `references/joints.md`.

## 4. Composite (multi-shape) bodies

ODE gives one body **one** inertia. For a part made of several shapes:

- Put each child geom in the space, `dGeomSetBody` them all to the **same** body, and give each a local
  offset with `dGeomSetOffsetPosition`/`dGeomSetOffsetRotation` (`collision.h:542,558`). This is the modern
  replacement for the **deprecated** `dCreateGeomTransform` — simpler and faster.
- Offset geoms are **collision-only (massless)**. Build the body's `dMass` yourself: each shape's `dMass`,
  `dMassTranslate`/`dMassRotate` into place, `dMassAdd` them, **then `dMassTranslate(&m, -m.c[0], -m.c[1],
  -m.c[2])` so the combined centre of mass lands at the body origin** (ODE *requires* the CoM at the origin —
  `references/foundations/coordinate-frames.md`), then `dBodySetMass` once. **Skipping the re-centre makes the
  body jump/double-move when the mass is set** — a second offset part double-moves the body (field-verified).
  Skipping the mass build entirely leaves one shape's inertia and it tumbles wrong. *If incremental
  composition gets fragile, the robust alternative is one shape per body joined by `dJointCreateFixed` welds —
  what the field robot-engine settled on.*

## 5. Actuation — drive a velocity servo

Robots are position/velocity-controlled through **motorized joints**: set `dParamVel` (target velocity) and
`dParamFMax` (force/torque ceiling) each step. A velocity servo is stable at any gain; a hand-rolled
PD-on-position servo is subject to the discrete-stability rule (**keep KD < 1** — `references/cpp-patterns.md`
§4, the velocity-mode trap). Read joint state back with `dJointGetHingeAngle`/`...Rate` etc.

## 6. The robot per-step loop (sense → control → actuate → step)

```
for each step:
    read sensors   (joint angles/rates, body poses)        // sense
    controller computes commands                           // control
    set motor targets (dParamVel/dParamFMax) BEFORE step   // actuate
    dSpaceCollide(space, data, &nearCallback)              // collide
    dWorldStep / dWorldQuickStep (check return)            // integrate (fixed dt)
    dJointGroupEmpty(contactgroup)                         // clear contacts
```

## 7. The near-callback, scaled up for a multi-body robot

The demo near-callback is the simple case. A robot world needs three more things (see lpzrobots'
`Simulation::nearCallback`):

```c
static void nearCallback(void *data, dGeomID o1, dGeomID o2) {
    if (dGeomIsSpace(o1) || dGeomIsSpace(o2)) {            // a robot in its own sub-space
        dSpaceCollide2(o1, o2, data, &nearCallback);       // recurse (collision_space.h)
        return;
    }
    dBodyID b1 = dGeomGetBody(o1), b2 = dGeomGetBody(o2);
    if (b1 && b2 && dAreConnectedExcluding(b1, b2, dJointTypeContact)) return;  // skip jointed parts (objects.h:3499)
    Material *m1 = (Material*)dGeomGetData(o1), *m2 = (Material*)dGeomGetData(o2);  // per-geom materials
    dContact c[64];                                        // bigger array for busy robots
    int n = dCollide(o1, o2, 64, &c[0].geom, sizeof(dContact));
    for (int i = 0; i < n; i++) {
        deriveSurface(&c[i].surface, m1, m2);              // combine BOTH materials -> mu/ERP/CFM
        dJointAttach(dJointCreateContact(world, cg, &c[i]), b1, b2);
    }
}
```

**Material model.** Rather than asking for ODE's unintuitive ERP/CFM/mu directly, real frameworks expose
intuitive scalars (roughness, slip, hardness, elasticity) per material and **combine the two contacting
geoms' materials** into `dSurfaceParameters` in the callback. Store the material on the geom via
`dGeomSetData` (§1).

## 8. Robot gotchas — each one cost a real debug

- **Auto-disable FREEZES a motorized body.** A robot that settles while auto-disable is on gets put to
  sleep, and a **sleeping body ignores its joint motors** — the robot then won't respond to commands at all.
  Call `dBodySetAutoDisableFlag(body, 0)` (`objects.h:986`) on every **actuated** robot body. (Auto-disable
  is still great for passive debris/props.)
- **Lifetime: the world must outlive every body/geom/joint.** If you wrap ODE handles in RAII
  (`references/cpp-patterns.md` §2), they must be destroyed **before** `dWorldDestroy`/`dCloseODE`, or a
  destructor calls `dBodyDestroy` on an already-freed world → crash. Own/declare the world **first** so it
  tears down **last**.
- **Universal/Hinge2 axes must be linearly independent** — parallel axes make ODE abort.
- **Self-collision** between joint-connected parts is skipped by the `dAreConnectedExcluding` guard (§7); for
  broader control, build the robot in its **own sub-space**.
- **Capsule long axis is local Z**; `dMassSetCapsule` direction = 3; rotate the part to orient a limb
  (`references/components/geom-capsule.md`).

## 9. Verify the orientation bridge (where silent bugs live)

If you bridge ODE poses to a renderer or math library: `dBodyGetRotation` returns a **row-major 3×4**
`dMatrix3`; a graphics 4×4 usually needs the **transpose** with translation in the **last row** (the
row-vector `v*M` convention). Orientation bugs are invisible at runtime (the sim still steps), so **validate
the bridge by round-tripping a known point through ODE's own functions** (`dBodyGetRelPointPos`,
`dBodyGetRotation`) — see `references/foundations/verifying-simulations.md`. Don't check your math against
your own assumptions; close the loop through ODE.

## Things to never invent
- A multi-shape body with several massful bodies welded — use ONE body + offset geoms (§4).
- `dCreateGeomTransform` for new composites — it's deprecated; use `dGeomSetOffset*` (§4).
- A motorized robot left under auto-disable expecting it to keep responding — it freezes (§8).
