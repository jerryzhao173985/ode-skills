# Coordinate Frames

ODE mixes two frames: the global WORLD frame (where positions, joint anchors, and
gravity live) and each body's BODY-LOCAL frame (its current pose). Getting force,
torque, and anchor calls into the right frame is the difference between a body that
behaves and one that snaps or drifts.

---

## World frame vs body-local frame

- Body positions, velocities, and gravity are in WORLD coordinates.
- Joint anchors and axes are in WORLD (global) coordinates — see below.
- "Rel" variants of the force/point functions take BODY-LOCAL coordinates and
  convert internally using the body's current orientation.

Each body's reference point MUST coincide with its center of mass. Joint
parameters (anchors/axes) are given in GLOBAL coordinates, so position bodies
correctly BEFORE attaching joints. Body shape is not a dynamical property — only
collision cares about shape; dynamics uses mass + inertia matrix. Cite:
`https://ode.org/wiki/index.php?title=Manual (Concepts: Rigid bodies / Joints and constraints)`.

---

## +Z-up convention in the demos

The drawstuff debug renderer (and therefore every bundled demo) uses +Z as up:

> In the virtual world, the z axis is "up" and z=0 is the floor.

Cite: `include/drawstuff/drawstuff.h:32`. ODE's dynamics core has no built-in "up"
axis — gravity is whatever vector you pass — but to match the demos and drawstuff
you put gravity along -Z:

```c
dWorldSetGravity(world, 0, 0, -9.81);   /* +Z up, Earth gravity */
```
Cite: `include/ode/objects.h:93` (`dWorldSetGravity`); the demos use this -Z form,
e.g. `dWorldSetGravity(world, 0, 0, -GRAVITY)` at `ode/demo/demo_boxstack.cpp:582`.

The drawstuff camera takes heading/pitch/roll in DEGREES (`dsSetViewpoint`, `hpr`
in degrees — cite `include/drawstuff/drawstuff.h:165`), but that is a rendering
concern, distinct from ODE's dynamics angles (radians; see below).

---

## AddForce vs AddRelForce

```c
void dBodyAddForce            (dBodyID, dReal fx, dReal fy, dReal fz);
void dBodyAddTorque           (dBodyID, dReal fx, dReal fy, dReal fz);
void dBodyAddRelForce         (dBodyID, dReal fx, dReal fy, dReal fz);
void dBodyAddRelTorque        (dBodyID, dReal fx, dReal fy, dReal fz);
```
Cite: `include/ode/objects.h:1165, 1171, 1177, 1183`.

- `dBodyAddForce` / `dBodyAddTorque`: the vector is in WORLD coordinates, applied at
  the center of mass. Cite: `include/ode/objects.h:1165, 1171`.
- `dBodyAddRelForce`: "Add force at centre of mass of body in coordinates relative
  to body." Cite: `include/ode/objects.h:1173-1177`.
- `dBodyAddRelTorque`: "Add torque at centre of mass of body in coordinates relative
  to body." Cite: `include/ode/objects.h:1179-1183`.

So a thruster that always pushes "forward" along the body's own nose uses
`dBodyAddRelForce` (the +X body axis), and it keeps pushing the right way no matter
how the body has rotated. A wind force that always blows north uses `dBodyAddForce`
with a fixed world vector.

For forces applied at a point (not the CoM), the `AtPos` / `AtRelPos` variants pick
the frame of the force and of the application point independently:

```c
void dBodyAddForceAtPos       (dBodyID, dReal fx, dReal fy, dReal fz, ...);  /* world force, world point */
void dBodyAddForceAtRelPos    (dBodyID, dReal fx, dReal fy, dReal fz, ...);  /* world force, body point */
void dBodyAddRelForceAtPos    (dBodyID, dReal fx, dReal fy, dReal fz, ...);  /* body force,  world point */
void dBodyAddRelForceAtRelPos (dBodyID, dReal fx, dReal fy, dReal fz, ...);  /* body force,  body point */
```
Cite: `include/ode/objects.h:1189, 1195, 1201, 1207`.

To convert between frames yourself: `dBodyVectorToWorld` (local -> world, cite
`include/ode/objects.h:1305`), `dBodyGetRelPointPos` (world position of a
body-relative point, cite `include/ode/objects.h:1257`), and `dBodyGetPosRelPoint`
(its inverse: world point -> body-relative, cite `include/ode/objects.h:1290`).

Force/torque accumulators are zeroed after every `dWorldStep`/`dWorldQuickStep`, so
continuous forces — world OR relative — must be reapplied every step. Cite:
`https://ode.org/wiki/index.php/Manual (3.4 Force accumulators)`.

---

## Anchors and axes are in global coordinates

Joint anchor points are specified in the WORLD frame:

> @param x The X position of the anchor point in world frame

Cite: `include/ode/objects.h:2302-2304`. Anchor GETTERS likewise return world
coordinates: "Get the joint anchor point, in world coordinates." Cite:
`include/ode/objects.h:2550`.

Because anchors/axes are global and many joints capture their zero reference (zero
angle/position) from the current body pose at the moment the anchor/axis is set,
you must position and orient both bodies FIRST, then `dJointAttach`, then set the
anchor/axis. Cite: `https://ode.org/wiki/index.php/Manual (3.5 Joints and
constraints / 7.1)` and `ode/demo/demo_boxstack.cpp:159` (`dJointAttach`).

`dJointAttach(c, b1, b2)`: pass `0` for a body to connect to the static
environment; `0` for both returns the joint to limbo. Cite:
`ode/demo/demo_boxstack.cpp:159`.

---

## Angles are in radians

ODE's dynamics angles (joint angles, axis-and-angle rotations) are in RADIANS.
Cite: `https://ode.org/wiki/index.php?title=Manual`. Rotation matrices/quaternions
are built from an axis and an angle:

```c
void dRFromAxisAndAngle (dMatrix3 R, dReal ax, dReal ay, dReal az, dReal angle);
void dQFromAxisAndAngle (dQuaternion q, dReal ax, dReal ay, dReal az, dReal angle);
```
Cite: `include/ode/rotation.h:36-37, 48-49`. Pass `angle` in radians. The
drawstuff CAMERA `hpr` is in degrees (cite `include/drawstuff/drawstuff.h:165`) —
do not confuse the two; convert with `angle * M_PI / 180.0` if you mix them.

[VERIFY] The exact wording "angles are in radians" is the standard ODE convention
and is consistent with `dRFromAxisAndAngle`/`dQFromAxisAndAngle` taking an `angle`,
but the header lines I read label these parameters generically as "angle" without a
verbatim "radians" string; the radians claim is wiki/convention-grounded, not a
verbatim header quote.

---

## Rules

- A body's reference point coincides with its center of mass; dynamics uses
  mass+inertia, not shape (shape is collision-only). Cite: `https://ode.org/wiki/index.php?title=Manual (Concepts: Rigid bodies / Joints and constraints)`.
- Demos and drawstuff use +Z up, z=0 floor; put gravity along -Z to match. Cite:
  `include/drawstuff/drawstuff.h:32`, `include/ode/objects.h:93`.
- `dBodyAddForce`/`dBodyAddTorque` take WORLD vectors; `dBodyAddRelForce`/
  `dBodyAddRelTorque` take BODY-LOCAL vectors. Cite: `include/ode/objects.h:1165, 1171, 1177, 1183`.
- Use `AtPos`/`AtRelPos` variants to choose the force frame and the point frame
  independently. Cite: `include/ode/objects.h:1189-1207`.
- Joint anchors and axes are in GLOBAL/world coordinates; position and orient both
  bodies BEFORE setting them. Cite: `include/ode/objects.h:2302-2304`,
  `https://ode.org/wiki/index.php/Manual (3.5 / 7.1)`.
- Pass `0` to `dJointAttach` to bind a body to the static world; both `0` = limbo.
  Cite: `ode/demo/demo_boxstack.cpp:159`.
- Force/torque accumulators (world and relative) are zeroed each step; reapply
  continuous forces every step. Cite: `https://ode.org/wiki/index.php/Manual (3.4 Force accumulators)`.
- Dynamics angles are in radians; the drawstuff camera `hpr` is in degrees. Cite:
  `include/ode/rotation.h:36-37`, `include/drawstuff/drawstuff.h:165`.

---

## Common mistakes

| Bad | Good | Why |
|-----|------|-----|
| Using `dBodyAddForce` with a fixed vector for a thruster that should push along the body's nose | Use `dBodyAddRelForce` so the force rotates with the body | `dBodyAddForce` is world-frame; the thrust direction must track the body's orientation, which is what the Rel variant does. Cite: `include/ode/objects.h:1165, 1177` |
| Setting joint anchors/axes before positioning the bodies, or assuming anchors are body-local | Position and orient both bodies first, then `dJointAttach`, then set anchors/axes in WORLD coordinates | Anchors/axes are global and joints capture their zero reference from the current pose; setting them first yields wrong zero angles and a snap at the first step. Cite: `include/ode/objects.h:2302-2304`, `https://ode.org/wiki/index.php/Manual (3.5 / 7.1)` |
| Applying a continuous (world or relative) force once and expecting it to persist | Reapply `dBodyAddForce`/`dBodyAddRelForce` every step before stepping | Force accumulators are zeroed after every step. Cite: `https://ode.org/wiki/index.php/Manual (3.4 Force accumulators)` |
| Passing degrees to `dRFromAxisAndAngle`/`dJointSet...Param` angle limits | Pass radians for dynamics; convert camera `hpr` (degrees) separately | ODE dynamics angles are radians; the drawstuff camera uses degrees — mixing them rotates by ~57x too much. Cite: `include/ode/rotation.h:36-37`, `include/drawstuff/drawstuff.h:165` |
| Building the world Y-up while drawing with drawstuff | Use +Z up (z=0 floor) to match drawstuff, with gravity along -Z | drawstuff fixes z as up; a mismatched up-axis makes the rendered scene look sideways. Cite: `include/drawstuff/drawstuff.h:32` |
