# Joints (constraints): catalog, lifecycle, dParam family, feedback

> Source of truth: include/ode/objects.h, include/ode/common.h, include/ode/contact.h. Every rule cites real ODE source by file:line; headers win over the wiki on conflict. C symbols start with 'd'; do not invent symbols.

This is the joint **overview**. Per-joint setters/getters (anchors, axes, angle/position readouts, full `dParam` tables) live in the component files linked in the catalog below — link to them, do not duplicate every setter here.

## Mental model

- A joint is a constraint between two bodies (or one body + the static world). Create it, **attach** it, then configure anchors/axes/params. A joint with no `dJointAttach` is in "limbo" and does nothing (include/ode/objects.h:1688).
- `dJointAttach(j, b1, b2)`: a zero body means the static environment; both zero = inert (include/ode/objects.h:1866).
- **Contact joints are special**: they are created per simulation step inside the collision near-callback from a filled `dContact`, then thrown away via `dJointGroupEmpty` (include/ode/objects.h:1717). See references/collision-and-contacts.md.
- Limits and motors are tuned through the `dParam*` integer constant family passed to `dJointSet<Type>Param`. A motor only acts when `dParamFMax > 0` (include/ode/common.h:446).
- Second/third axis params use suffixed names (`dParamVel2`, `dParamLoStop3`) or base + `dParamGroup`*n; an unimplemented param for a joint is silently ignored (include/ode/objects.h:1676).
- Force/torque on a joint is read by registering a user-owned `dJointFeedback` with `dJointSetFeedback`; ODE fills it each step (include/ode/objects.h:1944).

## When to use

- Connecting bodies into mechanisms: hinges (doors, wheels), sliders (pistons), ball-and-socket (ragdolls), hinge-2 (steering+suspension), universals, motors.
- Pinning a body to the world: attach with one body = 0.
- Resolving collisions: contact joints generated each step from `dCollide` results (see references/collision-and-contacts.md).
- Measuring constraint forces (e.g. is a bridge segment overloaded): `dJointFeedback`.

## Joint catalog

Every joint type is created by a `dJointCreate*` factory returning a `dJointID`; `group=0` allocates normally, nonzero allocates in that joint group (include/ode/objects.h:1693). After create you **must** `dJointAttach`. Per-type configuration lives in the linked component file.

| Joint type | Create function | dJointType enum | Component file | Source |
|---|---|---|---|---|
| Ball-and-socket | `dJointCreateBall` | `dJointTypeBall` | references/components/joint-ball.md | include/ode/objects.h:1693 |
| Hinge | `dJointCreateHinge` | `dJointTypeHinge` | references/components/joint-hinge.md | include/ode/objects.h:1701 |
| Slider (prismatic) | `dJointCreateSlider` | `dJointTypeSlider` | references/components/joint-slider.md | include/ode/objects.h:1709 |
| Contact | `dJointCreateContact` | `dJointTypeContact` | references/collision-and-contacts.md | include/ode/objects.h:1717 |
| Hinge-2 (steering+suspension) | `dJointCreateHinge2` | `dJointTypeHinge2` | references/components/joint-hinge2.md | include/ode/objects.h:1725 |
| Universal (two-axis) | `dJointCreateUniversal` | `dJointTypeUniversal` | references/components/joint-universal.md | include/ode/objects.h:1733 |
| PR (Prismatic+Rotoide) | `dJointCreatePR` | `dJointTypePR` | references/components/joint-pr.md | include/ode/objects.h:1741 |
| PU (Prismatic+Universal) | `dJointCreatePU` | `dJointTypePU` | references/components/joint-pu.md | include/ode/objects.h:1749 |
| Piston | `dJointCreatePiston` | `dJointTypePiston` | references/components/joint-piston.md | include/ode/objects.h:1758 |
| Fixed | `dJointCreateFixed` | `dJointTypeFixed` | references/components/joint-fixed.md | include/ode/objects.h:1766 |
| Null (no constraint) | `dJointCreateNull` | `dJointTypeNull` | references/components/joint-null.md | include/ode/objects.h:1768 |
| Angular motor (AMotor) | `dJointCreateAMotor` | `dJointTypeAMotor` | references/components/joint-amotor.md | include/ode/objects.h:1776 |
| Linear motor (LMotor) | `dJointCreateLMotor` | `dJointTypeLMotor` | references/components/joint-lmotor.md | include/ode/objects.h:1784 |
| Plane-2D | `dJointCreatePlane2D` | `dJointTypePlane2D` | references/components/joint-plane2d.md | include/ode/objects.h:1792 |
| Double-ball (DBall) | `dJointCreateDBall` | `dJointTypeDBall` | references/components/joint-dball.md | include/ode/objects.h:1800 |
| Double-hinge (DHinge) | `dJointCreateDHinge` | `dJointTypeDHinge` | references/components/joint-dhinge.md | include/ode/objects.h:1808 |
| Transmission (gear-like) | `dJointCreateTransmission` | `dJointTypeTransmission` | references/components/joint-transmission.md | include/ode/objects.h:1816 |

The full `dJointType` enum (`dJointTypeNone`..`dJointTypeTransmission`) is at include/ode/common.h:384.

## Public API (C)

### Lifecycle and groups

| Symbol | Signature | Source | Purpose |
|---|---|---|---|
| `dJointDestroy` | `void dJointDestroy (dJointID)` | include/ode/objects.h:1827 | Disconnect and remove a joint; no effect if it is a group member (empty/destroy the group instead). |
| `dJointGroupCreate` | `dJointGroupID dJointGroupCreate (int max_size)` | include/ode/objects.h:1835 | Create a joint group; `max_size` is deprecated, pass `0`. |
| `dJointGroupDestroy` | `void dJointGroupDestroy (dJointGroupID)` | include/ode/objects.h:1843 | Destroy a joint group and all joints in it. |
| `dJointGroupEmpty` | `void dJointGroupEmpty (dJointGroupID)` | include/ode/objects.h:1852 | Destroy all joints in the group but keep the group; call once per step to clear contact joints. |
| `dJointGetNumBodies` | `int dJointGetNumBodies(dJointID)` | include/ode/objects.h:1858 | Number of bodies attached to the joint. |
| `dJointAttach` | `void dJointAttach (dJointID, dBodyID body1, dBodyID body2)` | include/ode/objects.h:1873 | Required after create; either body 0 anchors to the static environment, both 0 puts joint in limbo. |

### Enable / disable / data / introspection

| Symbol | Signature | Source | Purpose |
|---|---|---|---|
| `dJointEnable` | `void dJointEnable (dJointID)` | include/ode/objects.h:1880 | Manually enable a joint. |
| `dJointDisable` | `void dJointDisable (dJointID)` | include/ode/objects.h:1889 | Manually disable a joint, retaining anchors/axes for later re-enable. |
| `dJointIsEnabled` | `int dJointIsEnabled (dJointID)` | include/ode/objects.h:1896 | Return 1 if enabled, 0 if disabled. |
| `dJointSetData` | `void dJointSetData (dJointID, void *data)` | include/ode/objects.h:1902 | Set the joint's user-data pointer. |
| `dJointGetData` | `void *dJointGetData (dJointID)` | include/ode/objects.h:1908 | Get the joint's user-data pointer. |
| `dJointGetType` | `dJointType dJointGetType (dJointID)` | include/ode/objects.h:1929 | Return the joint type enum. |
| `dJointGetBody` | `dBodyID dJointGetBody (dJointID, int index)` | include/ode/objects.h:1941 | Return attached body 0 or 1; a returned 0 = that side is the static environment. |

### Feedback

| Symbol | Signature | Source | Purpose |
|---|---|---|---|
| `dJointSetFeedback` | `void dJointSetFeedback (dJointID, dJointFeedback *)` | include/ode/objects.h:1950 | Register a user-owned `dJointFeedback` to receive per-step force/torque feedback. |
| `dJointGetFeedback` | `dJointFeedback *dJointGetFeedback (dJointID)` | include/ode/objects.h:1956 | Get the currently registered feedback struct pointer. |

### Contact joints

| Symbol | Signature | Source | Purpose |
|---|---|---|---|
| `dJointCreateContact` | `dJointID dJointCreateContact (dWorldID, dJointGroupID, const dContact *)` | include/ode/objects.h:1717 | Create a contact joint from a filled `dContact`; created per-step inside a collision near-callback. See references/collision-and-contacts.md. |

### Per-type setters/getters (catalog only — full API in component files)

Each joint type has its own `dJointSet<Type>Anchor`/`Axis`/`Param` and `dJointGet<Type>...` family. They are **not** repeated here; open the component card for the exact signatures and source lines. The per-type setter block starts at include/ode/objects.h:1965 (`dJointSetBallAnchor`).

| Joint type | Setter/getter family | Component card |
|---|---|---|
| Ball-and-socket | `dJointSetBallAnchor`, `dJointGetBallAnchor` | references/components/joint-ball.md |
| Hinge | `dJointSetHingeAnchor`/`Axis`/`Param`, `dJointAddHingeTorque`, `dJointGetHinge*` | references/components/joint-hinge.md |
| Slider | `dJointSetSliderAxis`/`Param`, `dJointGetSliderPosition` | references/components/joint-slider.md |
| Hinge-2 | `dJointSetHinge2Anchor`/`Axes`/`Param`, `dJointGetHinge2*` | references/components/joint-hinge2.md |
| Universal | `dJointSetUniversalAnchor`/`Axis*`/`Param`, `dJointGetUniversal*` | references/components/joint-universal.md |
| PR / PU / Piston | `dJointSetPR*` / `dJointSetPU*` / `dJointSetPiston*` | joint-pr.md, joint-pu.md, joint-piston.md |
| Fixed | `dJointSetFixed` (call after attach) | references/components/joint-fixed.md |
| AMotor | `dJointSetAMotorNumAxes`/`Axis`/`Mode`/`Param`, `dJointGetAMotor*` | references/components/joint-amotor.md |
| LMotor | `dJointSetLMotorNumAxes`/`Axis`/`Param` | references/components/joint-lmotor.md |
| Plane-2D / DBall / DHinge / Transmission | `dJointSetPlane2D*` / `dJointSetDBall*` / `dJointSetDHinge*` / `dJointSetTransmission*` | joint-plane2d.md, joint-dball.md, joint-dhinge.md, joint-transmission.md |

### Types

| Symbol | Definition | Source | Purpose |
|---|---|---|---|
| `dJointID` | `typedef struct dxJoint *dJointID` | include/ode/common.h:368 | Opaque handle to a joint. |
| `dJointGroupID` | `typedef struct dxJointGroup *dJointGroupID` | include/ode/common.h:369 | Opaque handle to a joint group. |
| `dJointType` | `typedef enum { dJointTypeNone=0 .. dJointTypeTransmission }` | include/ode/common.h:384 | Joint type enum returned by `dJointGetType`. |
| `dJointFeedback` | `struct { dVector3 f1,t1,f2,t2; }` | include/ode/common.h:516 | Force/torque applied to body1 and body2; user-owned, filled each step. |
| `dContact` | `struct { dSurfaceParameters surface; dContactGeom geom; dVector3 fdir1; }` | include/ode/contact.h:99 | Passed to `dJointCreateContact`. See references/collision-and-contacts.md. |
| `dSurfaceParameters` | contact surface settings | include/ode/contact.h:55 | Required `mode` and `mu`; optional `mu2`/`bounce`/`soft_erp`/`soft_cfm`/`slip` gated by mode flags. |
| `dContactGeom` | `struct { dVector3 pos,normal; dReal depth; dGeomID g1,g2; int side1,side2; }` | include/ode/contact.h:88 | Filled by `dCollide`; lives at `dContact.geom`. |

## Key rules

- A freshly created joint is in "limbo" and has no effect until `dJointAttach` connects bodies (include/ode/objects.h:1688).
- `dJointAttach(joint, b1, b2)`: passing 0 for one body anchors to the static environment; passing 0 for both leaves it inert (include/ode/objects.h:1866).
- Set anchors/axes **after** attaching — the anchor/axis is interpreted relative to the attached bodies' current world poses (include/ode/objects.h:1873).
- Contact joints are created per-step from a `dContact` inside the collision near-callback via `dJointCreateContact(world, contactgroup, &contact)` (include/ode/objects.h:1717).
- Per-step loop order is `dSpaceCollide` (whose near-callback creates+attaches contact joints) → `dWorldStep` → `dJointGroupEmpty(contactgroup)` (ode/demo/demo_buggy.cpp:185).
- `dJointGroupEmpty` destroys all joints in the group without destroying the group; call it every step so contact joints do not accumulate (include/ode/objects.h:1852).
- `dJointDestroy` has no effect on a joint that belongs to a group; destroy or empty the group instead (include/ode/objects.h:1824).
- The contactgroup is created once with `dJointGroupCreate(0)` before the loop and destroyed once with `dJointGroupDestroy` after (ode/demo/demo_buggy.cpp:226).
- `dContact.surface.mode` and `surface.mu` must always be set; other surface fields (`mu2`, `bounce`, `soft_cfm`, `slip1`...) only take effect when their flag is OR-ed into `mode` (include/ode/contact.h:56).
- `dCollide` writes into `&contact[0].geom` with stride `sizeof(dContact)` so each `dContactGeom` lands inside its parent `dContact` ready for `dJointCreateContact` (ode/demo/demo_buggy.cpp:95).
- `dJointSetFixed` must be called (after attaching) to lock a fixed joint to the bodies' current relative pose (include/ode/objects.h:2451).
- `dJointSetAMotorAngle` should only be called in `dAMotorUser` mode, where the motor cannot otherwise know joint angles (include/ode/objects.h:2477).
- Second/third axis parameters use the suffixed names (e.g. `dParamVel2`, `dParamLoStop3`) or base + `dParamGroup`*n; an unimplemented parameter for a joint is silently ignored (include/ode/objects.h:1678).
- `dJointSetHingeAnchor` or `dJointSetHingeAxis` resets the joint's angle 'zero' reference (include/ode/objects.h:2018).
- The feedback struct passed to `dJointSetFeedback` is user-owned and must outlive the joint; ODE fills it each step (include/ode/objects.h:1944).

## Worked examples

Canonical near-callback — create + attach contact joints per collision (ode/demo/demo_buggy.cpp:84-111):

```c
static void nearCallback (void *, dGeomID o1, dGeomID o2)
{
  const int N = 10;
  dContact contact[N];
  int n = dCollide (o1,o2,N,&contact[0].geom,sizeof(dContact));
  if (n > 0) {
    for (int i=0; i<n; i++) {
      contact[i].surface.mode = dContactSlip1 | dContactSlip2 |
        dContactSoftERP | dContactSoftCFM | dContactApprox1;
      contact[i].surface.mu = dInfinity;
      contact[i].surface.slip1 = 0.1;
      contact[i].surface.slip2 = 0.1;
      contact[i].surface.soft_erp = 0.5;
      contact[i].surface.soft_cfm = 0.3;
      dJointID c = dJointCreateContact (world,contactgroup,&contact[i]);
      dJointAttach (c,
                    dGeomGetBody(contact[i].geom.g1),
                    dGeomGetBody(contact[i].geom.g2));
    }
  }
}
```

Per-step loop — collide → step → empty contact group (ode/demo/demo_buggy.cpp:185-189):

```c
dSpaceCollide (space,0,&nearCallback);
dWorldStep (world,0.05);
// remove all contact joints
dJointGroupEmpty (contactgroup);
```

Contact group lifecycle — create once, destroy once (ode/demo/demo_buggy.cpp:226, ode/demo/demo_buggy.cpp:303):

```c
contactgroup = dJointGroupCreate (0);
/* ... simulation loop ... */
dJointGroupDestroy (contactgroup);
```

Powered hinge with feedback registration (ode/demo/demo_feedback.cpp:253-258):

```c
dJointSetHingeAnchor (hinges[i], i + 0.5 - SEGMCNT/2.0, 0, 5);
dJointSetHingeAxis (hinges[i], 0,1,0);
dJointSetHingeParam (hinges[i],dParamFMax,  8000.0);
dJointSetFeedback (hinges[i], jfeedbacks+i);
```

## Common mistakes

| Bad | Good | Why |
|---|---|---|
| `dJointID j = dJointCreateHinge(world, 0); dJointSetHingeAnchor(j, ...); // never attached` | `dJointID j = dJointCreateHinge(world, 0); dJointAttach(j, b1, b2); dJointSetHingeAnchor(j, ...);` | Forgetting `dJointAttach` leaves the joint in limbo with zero effect (include/ode/objects.h:1688). |
| `dSpaceCollide(...); dWorldStep(world, dt); // no dJointGroupEmpty` | `dSpaceCollide(...); dWorldStep(world, dt); dJointGroupEmpty(contactgroup);` | Not emptying the contact group each step leaks contact joints and corrupts the sim; `dJointDestroy` won't free group members either (include/ode/objects.h:1852). |
| `dJointSetHingeParam(j, dParamVel, 2.0); // FMax left at 0` | `dJointSetHingeParam(j, dParamVel, 2.0); dJointSetHingeParam(j, dParamFMax, 8000.0);` | A motor does nothing unless `dParamFMax` is positive (include/ode/common.h:446). |
| `contact.surface.bounce = 0.5; // dContactBounce not in mode` | `contact.surface.mode = dContactBounce; contact.surface.bounce = 0.5; contact.surface.mu = dInfinity;` | Optional surface fields are ignored unless their flag is in `mode` (include/ode/contact.h:56). |
| `dJointSetHinge2Param(j, dParamVel, v); // only axis 1` | `dJointSetHinge2Param(j, dParamVel2, v); // axis 2 (steering vs wheel-spin)` | Second-axis params need suffixed names; the base constant addresses axis 1 only (include/ode/objects.h:1678). |

## Named constants

### dParam family (limits and motors) — passed to `dJointSet<Type>Param`

| Name | Value | Source | Purpose |
|---|---|---|---|
| `dParamLoStop` | base value 0 of the dParam group | include/ode/common.h:441 | Low stop angle/position for a limit. |
| `dParamHiStop` | next | include/ode/common.h:442 | High stop angle/position for a limit. |
| `dParamVel` | next | include/ode/common.h:443 | Desired motor velocity (angular or linear). |
| `dParamFMax` | next | include/ode/common.h:446 | Max force/torque the motor applies to reach `dParamVel`; must be >0 for the motor to act. |
| `dParamFudgeFactor` | next | include/ode/common.h:447 | Fudge factor (0..1) tuning motor-at-stop behavior. |
| `dParamBounce` | next | include/ode/common.h:448 | Restitution/bounciness of the stops (0..1). |
| `dParamCFM` | next | include/ode/common.h:449 | Constraint force mixing for the joint's normal (non-stop) constraints. |
| `dParamStopERP` | next | include/ode/common.h:450 | Error reduction parameter applied at the stops. |
| `dParamStopCFM` | next | include/ode/common.h:451 | Constraint force mixing applied at the stops. |
| `dParamSuspensionERP` / `dParamSuspensionCFM` | next | include/ode/common.h:453 | Suspension ERP/CFM, currently only implemented on the hinge-2 joint. |

### Axis-group addressing

| Name | Value | Source | Purpose |
|---|---|---|---|
| `dParamGroup` | `0x100` | include/ode/common.h:494 | Stride; add multiples (or use 1/2/3-suffixed names) to address 2nd/3rd-axis params, e.g. `dParamVel2`, and `(dParamGroup2 \| dParamFMax) == dParamFMax2`. |

### AMotor modes — passed to `dJointSetAMotorMode`

| Name | Value | Source | Purpose |
|---|---|---|---|
| `dAMotorUser` | `0` | include/ode/common.h:501 | User-driven AMotor mode (you call `dJointSetAMotorAngle`). |
| `dAMotorEuler` | `1` | include/ode/common.h:502 | Euler-angle AMotor mode. |

### Contact surface mode flags — OR-ed into `dContact.surface.mode`

Owned by references/collision-and-contacts.md → "surface.mode bitflags" (every flag in contact.h, plus the full `dSurfaceParameters` field list). The enum opens at include/ode/contact.h:33.

## Things to never invent

- `dJointCreateGearbox` — no such function; use `dJointCreateTransmission` (include/ode/objects.h:1816).
- `dJointGroupAdd` / `dJointGroupRemove` — joints are added implicitly via the group arg to `dJointCreate*` (include/ode/objects.h:1693).
- `dJointSetContactParam` — contacts are configured via the `dContact` struct, not a setter (include/ode/objects.h:1717).
- `dJointSetMotorVelocity` / `dJointSetMaxForce` — use `dJointSet<Type>Param` with `dParamVel` / `dParamFMax` (include/ode/common.h:443).
- `dJointDetach` — re-call `dJointAttach` with 0 bodies instead (include/ode/objects.h:1866).
- `dParamMaxForce` / `dParamMotorVel` / `dParamLowStop` / `dParamHighStop` — correct names are `dParamFMax`, `dParamVel`, `dParamLoStop`, `dParamHiStop` (include/ode/common.h:441).
- `dJointGetForce` / `dJointGetTorque` — read forces via `dJointFeedback` set with `dJointSetFeedback` (include/ode/objects.h:1950).
- `dJointTypeSpring` / `dJointTypeWheel` — not in `dJointType` (include/ode/common.h:384).

## [VERIFY] markers

The following facts could not be fully grounded in the joint headers and are flagged for verification:

- `dParamLoVel` and `dParamHiVel` exist in the enum (include/ode/common.h:444-445) but are not documented in the objects.h `dParam` doc block; their semantics are not described in the public headers. [VERIFY]
- The hinge-2 "axis2 uses the +1 suffixed dParam forms" convention is inferred from include/ode/objects.h:1678-1680 plus the `dParamGroup` stride at include/ode/common.h:494; the headers give no explicit hinge-2 worked example. [VERIFY]
- `dInfinity` (used for `surface.mu` in the demos) is referenced but defined outside the joint headers; not grounded here. [VERIFY]
