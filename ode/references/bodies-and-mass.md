# ODE Bodies and Mass

> Source of truth: include/ode/objects.h (BODY section) and include/ode/mass.h. Every rule cites real ODE source by file:line; headers win over the wiki on conflict. C symbols start with 'd'; do not invent symbols.

## Mental model

- A `dBodyID` is the dynamic state of a rigid body: position, orientation, linear/angular velocity, accumulated force/torque, and a mass. It lives inside a `dWorldID` and is the thing the integrator moves (include/ode/objects.h:1011).
- Mass and inertia are NOT part of the body struct you build directly. You fill a separate `dMass` value, then hand it to the body with `dBodySetMass` (include/ode/objects.h:1153, include/ode/mass.h:88).
- `dBodyGet*` for pose/velocity/force return `const dReal *` pointers straight into ODE internals: read-only, never free, valid only until the system changes. To own a snapshot, use the matching `dBodyCopy*` (include/ode/objects.h:1083).
- Forces and torques ACCUMULATE via the `dBodyAdd*` family and are cleared automatically after each world step; `Rel*` variants interpret vectors/points in the body's own local frame, plain variants use world frame (include/ode/objects.h:1177).
- A dynamic body must have a real (positive-definite) mass before you step the world; the demos always do Create -> SetMass -> SetPosition (ode/demo/demo_hinge.cpp:144).

## When to use

- Use this module when you create the moving rigid bodies of a simulation, position/orient them, read their state back each frame, or push them with forces, torques, and impulse-like effects.
- Use `dMass*` when you need a physically correct mass/inertia tensor for a primitive (box, sphere, capsule, cylinder, trimesh), to combine parts into a compound body, or to rescale/recenter mass.
- Use enable/disable and kinematic/dynamic controls for sleeping bodies and for scripted (animation-driven) objects that should not be pushed by forces.

## Public API (C)

### Lifecycle, world, and user-data

| Symbol | Signature | Source | Purpose |
| --- | --- | --- | --- |
| dBodyCreate | `dBodyID dBodyCreate (dWorldID)` | include/ode/objects.h:1011 | Create a rigid body in a world; default mass at (0,0,0). |
| dBodyDestroy | `void dBodyDestroy (dBodyID)` | include/ode/objects.h:1021 | Destroy a body; attached joints become unattached (limbo), not deleted. |
| dBodyGetWorld | `dWorldID dBodyGetWorld (dBodyID)` | include/ode/objects.h:1003 | Return the world a body belongs to. |
| dBodySetData | `void dBodySetData (dBodyID, void *data)` | include/ode/objects.h:1028 | Set the body's arbitrary user-data pointer. |
| dBodyGetData | `void *dBodyGetData (dBodyID)` | include/ode/objects.h:1035 | Return the body's user-data pointer. |

### Pose and orientation (set)

| Symbol | Signature | Source | Purpose |
| --- | --- | --- | --- |
| dBodySetPosition | `void dBodySetPosition (dBodyID, dReal x, dReal y, dReal z)` | include/ode/objects.h:1045 | Set world position of the body center of mass. |
| dBodySetRotation | `void dBodySetRotation (dBodyID, const dMatrix3 R)` | include/ode/objects.h:1055 | Set orientation from a 3x3 (stored 4x3) rotation matrix. |
| dBodySetQuaternion | `void dBodySetQuaternion (dBodyID, const dQuaternion q)` | include/ode/objects.h:1065 | Set orientation from a quaternion. |
| dBodySetLinearVel | `void dBodySetLinearVel (dBodyID, dReal x, dReal y, dReal z)` | include/ode/objects.h:1071 | Set the linear velocity of the body. |
| dBodySetAngularVel | `void dBodySetAngularVel (dBodyID, dReal x, dReal y, dReal z)` | include/ode/objects.h:1077 | Set the angular velocity of the body. |

### Pose, velocity, and force (get pointer / copy)

| Symbol | Signature | Source | Purpose |
| --- | --- | --- | --- |
| dBodyGetPosition | `const dReal * dBodyGetPosition (dBodyID)` | include/ode/objects.h:1088 | Pointer to internal position 3-vector; do not free. |
| dBodyCopyPosition | `void dBodyCopyPosition (dBodyID body, dVector3 pos)` | include/ode/objects.h:1098 | Copy position into a caller-owned dVector3. |
| dBodyGetRotation | `const dReal * dBodyGetRotation (dBodyID)` | include/ode/objects.h:1106 | Pointer to internal 4x3 rotation matrix; do not free. |
| dBodyCopyRotation | `void dBodyCopyRotation (dBodyID, dMatrix3 R)` | include/ode/objects.h:1116 | Copy rotation matrix into a caller-owned dMatrix3. |
| dBodyGetQuaternion | `const dReal * dBodyGetQuaternion (dBodyID)` | include/ode/objects.h:1124 | Pointer to internal 4-scalar quaternion; do not free. |
| dBodyCopyQuaternion | `void dBodyCopyQuaternion (dBodyID body, dQuaternion quat)` | include/ode/objects.h:1134 | Copy orientation quaternion into caller-owned dQuaternion. |
| dBodyGetLinearVel | `const dReal * dBodyGetLinearVel (dBodyID)` | include/ode/objects.h:1141 | Pointer to internal linear-velocity 3-vector; do not free. |
| dBodyGetAngularVel | `const dReal * dBodyGetAngularVel (dBodyID)` | include/ode/objects.h:1147 | Pointer to internal angular-velocity 3-vector; do not free. |
| dBodyGetForce | `const dReal * dBodyGetForce (dBodyID)` | include/ode/objects.h:1219 | Pointer to accumulated force 3-vector; valid until system changes. |
| dBodyGetTorque | `const dReal * dBodyGetTorque (dBodyID)` | include/ode/objects.h:1230 | Pointer to accumulated torque 3-vector; valid until system changes. |

### Mass assignment on the body

| Symbol | Signature | Source | Purpose |
| --- | --- | --- | --- |
| dBodySetMass | `void dBodySetMass (dBodyID, const dMass *mass)` | include/ode/objects.h:1153 | Assign a mass/inertia struct to the body (copied in). |
| dBodyGetMass | `void dBodyGetMass (dBodyID, dMass *mass)` | include/ode/objects.h:1159 | Copy the body's current mass struct out into caller storage. |

### Force / torque family

| Symbol | Signature | Source | Purpose |
| --- | --- | --- | --- |
| dBodyAddForce | `void dBodyAddForce (dBodyID, dReal fx, dReal fy, dReal fz)` | include/ode/objects.h:1165 | Add force at COM in world coordinates; cleared each step. |
| dBodyAddTorque | `void dBodyAddTorque (dBodyID, dReal fx, dReal fy, dReal fz)` | include/ode/objects.h:1171 | Add torque at COM in world coordinates; cleared each step. |
| dBodyAddRelForce | `void dBodyAddRelForce (dBodyID, dReal fx, dReal fy, dReal fz)` | include/ode/objects.h:1177 | Add force at COM in body-relative coordinates; cleared each step. |
| dBodyAddRelTorque | `void dBodyAddRelTorque (dBodyID, dReal fx, dReal fy, dReal fz)` | include/ode/objects.h:1183 | Add torque at COM in body-relative coordinates. |
| dBodyAddForceAtPos | `void dBodyAddForceAtPos (dBodyID, dReal fx, dReal fy, dReal fz, dReal px, dReal py, dReal pz)` | include/ode/objects.h:1189 | World-frame force at a world-frame point (induces torque). |
| dBodyAddForceAtRelPos | `void dBodyAddForceAtRelPos (dBodyID, dReal fx, dReal fy, dReal fz, dReal px, dReal py, dReal pz)` | include/ode/objects.h:1195 | World-frame force at a body-local point. |
| dBodyAddRelForceAtPos | `void dBodyAddRelForceAtPos (dBodyID, dReal fx, dReal fy, dReal fz, dReal px, dReal py, dReal pz)` | include/ode/objects.h:1201 | Body-frame force at a world-frame point. |
| dBodyAddRelForceAtRelPos | `void dBodyAddRelForceAtRelPos (dBodyID, dReal fx, dReal fy, dReal fz, dReal px, dReal py, dReal pz)` | include/ode/objects.h:1207 | Body-frame force at a body-local point. |
| dBodySetForce | `void dBodySetForce (dBodyID b, dReal x, dReal y, dReal z)` | include/ode/objects.h:1240 | Overwrite the accumulated force (mainly to zero before reactivating). |
| dBodySetTorque | `void dBodySetTorque (dBodyID b, dReal x, dReal y, dReal z)` | include/ode/objects.h:1250 | Overwrite the accumulated torque (mainly to zero before reactivating). |

### Point and vector frame transforms

| Symbol | Signature | Source | Purpose |
| --- | --- | --- | --- |
| dBodyGetRelPointPos | `void dBodyGetRelPointPos (dBodyID, dReal px, dReal py, dReal pz, dVector3 result)` | include/ode/objects.h:1257 | World position of a body-relative point -> result. |
| dBodyGetRelPointVel | `void dBodyGetRelPointVel (dBodyID, dReal px, dReal py, dReal pz, dVector3 result)` | include/ode/objects.h:1268 | World-coords velocity of a body-relative point -> result. |
| dBodyGetPointVel | `void dBodyGetPointVel (dBodyID, dReal px, dReal py, dReal pz, dVector3 result)` | include/ode/objects.h:1280 | World-coords velocity of a globally-specified point on the body. |
| dBodyGetPosRelPoint | `void dBodyGetPosRelPoint (dBodyID, dReal px, dReal py, dReal pz, dVector3 result)` | include/ode/objects.h:1294 | Inverse of GetRelPointPos: global point -> body-relative coords. |
| dBodyVectorToWorld | `void dBodyVectorToWorld (dBodyID, dReal px, dReal py, dReal pz, dVector3 result)` | include/ode/objects.h:1305 | Rotate a local-frame vector into world coordinates. |
| dBodyVectorFromWorld | `void dBodyVectorFromWorld (dBodyID, dReal px, dReal py, dReal pz, dVector3 result)` | include/ode/objects.h:1316 | Rotate a world-frame vector into body-local coordinates. |

### Finite rotation and gyroscopic term

| Symbol | Signature | Source | Purpose |
| --- | --- | --- | --- |
| dBodySetFiniteRotationMode | `void dBodySetFiniteRotationMode (dBodyID, int mode)` | include/ode/objects.h:1339 | 0 = infinitesimal (default), 1 = finite for high spin. |
| dBodySetFiniteRotationAxis | `void dBodySetFiniteRotationAxis (dBodyID, dReal x, dReal y, dReal z)` | include/ode/objects.h:1357 | Set finite-rotation axis (only in finite mode); zero = full finite. |
| dBodyGetFiniteRotationMode | `int dBodyGetFiniteRotationMode (dBodyID)` | include/ode/objects.h:1364 | Return 0 (infinitesimal) or 1 (finite) rotation mode. |
| dBodyGetFiniteRotationAxis | `void dBodyGetFiniteRotationAxis (dBodyID, dVector3 result)` | include/ode/objects.h:1371 | Write the finite-rotation axis to result. |
| dBodyGetGyroscopicMode | `int dBodyGetGyroscopicMode (dBodyID b)` | include/ode/objects.h:1599 | Nonzero if gyroscopic-term computation is enabled (default). |
| dBodySetGyroscopicMode | `void dBodySetGyroscopicMode (dBodyID b, int enabled)` | include/ode/objects.h:1613 | Enable (nonzero, default) or disable (0) the gyroscopic term; disabling aids stability. |

### Joints attached to a body, geom list, moved callback

| Symbol | Signature | Source | Purpose |
| --- | --- | --- | --- |
| dBodyGetNumJoints | `int dBodyGetNumJoints (dBodyID b)` | include/ode/objects.h:1378 | Number of joints attached to the body. |
| dBodyGetJoint | `dJointID dBodyGetJoint (dBodyID, int index)` | include/ode/objects.h:1386 | Attached joint at index in [0, n-1]. |
| dBodyGetFirstGeom | `dGeomID dBodyGetFirstGeom (dBodyID b)` | include/ode/objects.h:1483 | First geom attached to the body (iterate with GetNextGeom). |
| dBodyGetNextGeom | `dGeomID dBodyGetNextGeom (dGeomID g)` | include/ode/objects.h:1493 | Next geom in the body's geom list. |
| dBodySetMovedCallback | `void dBodySetMovedCallback (dBodyID b, void (*callback)(dBodyID))` | include/ode/objects.h:1471 | Callback fired when the body's pose changes during a step. |

### Enable/disable, kinematic/dynamic, gravity

| Symbol | Signature | Source | Purpose |
| --- | --- | --- | --- |
| dBodySetDynamic | `void dBodySetDynamic (dBodyID)` | include/ode/objects.h:1396 | Set the body to dynamic state (the default). |
| dBodySetKinematic | `void dBodySetKinematic (dBodyID)` | include/ode/objects.h:1410 | Kinematic: unstoppable, ignores forces, controlled by pos/vel, infinite mass. |
| dBodyIsKinematic | `int dBodyIsKinematic (dBodyID)` | include/ode/objects.h:1417 | 1 if kinematic, 0 if dynamic. |
| dBodyEnable | `void dBodyEnable (dBodyID)` | include/ode/objects.h:1424 | Manually enable (activate) a body. |
| dBodyDisable | `void dBodyDisable (dBodyID)` | include/ode/objects.h:1433 | Manually disable; auto re-enabled if jointed to an enabled body next step. |
| dBodyIsEnabled | `int dBodyIsEnabled (dBodyID)` | include/ode/objects.h:1440 | 1 if enabled, 0 if disabled. |
| dBodySetGravityMode | `void dBodySetGravityMode (dBodyID b, int mode)` | include/ode/objects.h:1449 | Nonzero = world gravity affects this body (default); zero = ignore. |
| dBodyGetGravityMode | `int dBodyGetGravityMode (dBodyID b)` | include/ode/objects.h:1456 | Nonzero if the body is affected by world gravity. |

### Mass construction (dMass*, in include/ode/mass.h)

| Symbol | Signature | Source | Purpose |
| --- | --- | --- | --- |
| dMassCheck | `int dMassCheck (const dMass *m)` | include/ode/mass.h:43 | 1 if mass and inertia matrix are positive-definite (valid). |
| dMassSetZero | `void dMassSetZero (dMass *)` | include/ode/mass.h:45 | Zero the mass struct (mass, center, inertia all 0). |
| dMassSetParameters | `void dMassSetParameters (dMass *, dReal themass, dReal cgx, dReal cgy, dReal cgz, dReal I11, dReal I22, dReal I33, dReal I12, dReal I13, dReal I23)` | include/ode/mass.h:47 | Set mass, center of gravity, and the six unique inertia-tensor entries directly. |
| dMassSetSphere | `void dMassSetSphere (dMass *, dReal density, dReal radius)` | include/ode/mass.h:52 | Sphere mass/inertia from density and radius. |
| dMassSetSphereTotal | `void dMassSetSphereTotal (dMass *, dReal total_mass, dReal radius)` | include/ode/mass.h:53 | Sphere mass/inertia from target total mass and radius. |
| dMassSetCapsule | `void dMassSetCapsule (dMass *, dReal density, int direction, dReal radius, dReal length)` | include/ode/mass.h:55 | Capsule mass from density; direction 1/2/3 = x/y/z long axis. |
| dMassSetCapsuleTotal | `void dMassSetCapsuleTotal (dMass *, dReal total_mass, int direction, dReal radius, dReal length)` | include/ode/mass.h:57 | Capsule mass from total mass; direction 1/2/3 = x/y/z long axis. |
| dMassSetCylinder | `void dMassSetCylinder (dMass *, dReal density, int direction, dReal radius, dReal length)` | include/ode/mass.h:60 | Flat-ended cylinder mass from density; direction 1/2/3 = axis. |
| dMassSetCylinderTotal | `void dMassSetCylinderTotal (dMass *, dReal total_mass, int direction, dReal radius, dReal length)` | include/ode/mass.h:62 | Cylinder mass from total mass; direction 1/2/3 = axis. |
| dMassSetBox | `void dMassSetBox (dMass *, dReal density, dReal lx, dReal ly, dReal lz)` | include/ode/mass.h:65 | Box mass/inertia from density and side lengths. |
| dMassSetBoxTotal | `void dMassSetBoxTotal (dMass *, dReal total_mass, dReal lx, dReal ly, dReal lz)` | include/ode/mass.h:67 | Box mass/inertia from target total mass and side lengths. |
| dMassSetTrimesh | `void dMassSetTrimesh (dMass *, dReal density, dGeomID g)` | include/ode/mass.h:70 | Mass/inertia from a trimesh geom and density. |
| dMassSetTrimeshTotal | `void dMassSetTrimeshTotal (dMass *m, dReal total_mass, dGeomID g)` | include/ode/mass.h:72 | Mass/inertia from a trimesh geom and total mass. |
| dMassAdjust | `void dMassAdjust (dMass *, dReal newmass)` | include/ode/mass.h:74 | Rescale the mass struct to a new total mass, keeping shape/center. |
| dMassTranslate | `void dMassTranslate (dMass *, dReal x, dReal y, dReal z)` | include/ode/mass.h:76 | Translate the mass so its center of gravity shifts by (x,y,z). |
| dMassRotate | `void dMassRotate (dMass *, const dMatrix3 R)` | include/ode/mass.h:78 | Rotate the mass distribution by matrix R. |
| dMassAdd | `void dMassAdd (dMass *a, const dMass *b)` | include/ode/mass.h:80 | Accumulate mass b into a (combine compound bodies). |

### Types used throughout

| Symbol | Definition | Source | Purpose |
| --- | --- | --- | --- |
| dMass | `struct dMass { dReal mass; dVector3 c; dMatrix3 I; ... }` | include/ode/mass.h:88 | Mass + center of gravity `c` + inertia tensor `I` about origin; C++ ctor calls dMassSetZero. |
| dBodyID | `typedef struct dxBody *dBodyID` | include/ode/common.h:366 | Opaque handle to a rigid body. |
| dWorldID | `typedef struct dxWorld *dWorldID` | include/ode/common.h:364 | Opaque handle to a world (container for bodies). |
| dReal | `float` or `double` | include/ode/common.h:57 | Scalar float type; float in single-precision, double in double-precision builds. |
| dVector3 | fixed `dReal` array (padded to 4) | include/ode/common.h:270 | 3D positions/velocities. |
| dMatrix3 | fixed `dReal` array (4x3 / 12 entries) | include/ode/common.h:272 | 3x3 rotation, stored as 4x3. |
| dQuaternion | fixed `dReal` array | include/ode/common.h:275 | Quaternion in (w,x,y,z) order: the `dQuatElement` enum lists `dQUE_R` (real/w) first, then `dQUE_I/J/K` (include/ode/common.h:256). |

## Key rules

- A body needs a valid mass before stepping: fill a `dMass` (e.g. `dMassSetBox`) then call `dBodySetMass` on the freshly created body; demos always do Create -> SetMass -> SetPosition (ode/demo/demo_hinge.cpp:144).
- Build mass on a local `dMass`: `dMassSetBox(&m,...)` then `dMassAdjust(&m,MASS)` to hit a target total mass, then `dBodySetMass(body,&m)`. There is no `dMassSetMass`; use `dMassAdjust` or the `*Total` setters (ode/demo/demo_hinge.cpp:137).
- `dBodyGetPosition`/`GetRotation`/`GetQuaternion`/`GetLinearVel`/`GetAngularVel`/`GetForce`/`GetTorque` return `const dReal*` pointers to internal data: read-only, do NOT free, valid only until the rigid-body system changes; use `dBodyCopy*` to snapshot (include/ode/objects.h:1083).
- `dBodyAddRelForce`/`dBodyAddRelTorque` and the `*RelPos` variants interpret force/point in the body's own frame; plain `dBodyAddForce`/`dBodyAddTorque` use the world frame. Forces accumulate and are cleared automatically after each world step (include/ode/objects.h:1177).
- Setting a kinematic body's mass to something other than infinite mass reverts it to a normal dynamic body; `dBodySetKinematic` gives infinite mass and immunity to forces (include/ode/objects.h:1404).
- Setting position/rotation/quaternion after joints are attached yields undefined simulation if the new pose is inconsistent with the joints/constraints (include/ode/objects.h:1040).
- `dBodyDestroy` does not delete joints attached to the body; they become unattached limbo joints that must be destroyed separately (include/ode/objects.h:1016).
- Use `dBodySetForce`/`dBodySetTorque` to zero accumulated force/torque on a deactivated body before reactivating it (include/ode/objects.h:1235).
- A finite-rotation axis only has meaning when finite rotation mode (mode=1) is enabled; a zero axis means full finite rotation (include/ode/objects.h:1345).
- The mass capsule/cylinder `direction` parameter is an int axis selector (1=x, 2=y, 3=z); pass the long axis of the shape (include/ode/mass.h:55).

## Common mistakes

| Bad | Good | Why |
| --- | --- | --- |
| `b = dBodyCreate(world); dBodySetPosition(b,0,0,1); /* step with no mass */` | `dMass m; dMassSetBox(&m,1,SIDE,SIDE,SIDE); dMassAdjust(&m,MASS); b=dBodyCreate(world); dBodySetMass(b,&m);` | A body with no/zero mass produces garbage/unstable simulation; ODE expects a valid positive-definite mass before stepping (ode/demo/demo_hinge.cpp:144). |
| `const dReal* p = dBodyGetPosition(b); free((void*)p);` | `dVector3 p; dBodyCopyPosition(b, p); /* p is caller-owned */` | `dBodyGetPosition` returns a pointer into ODE's internals; freeing or caching it across changes corrupts memory or reads stale data — use `dBodyCopyPosition` (include/ode/objects.h:1083). |
| `dBodyAddForce(b, 0,0,thrust); // expecting 'up along body axis'` | `dBodyAddRelForce(b, 0,0,thrust); // thrust along body's own +z` | `dBodyAddForce` is world-frame; `dBodyAddRelForce` is the body's local frame — the wrong one applies thrust in the wrong direction as the body rotates (include/ode/objects.h:1177). |
| `dBodySetMass(b, 5.0); // second arg is const dMass*, not a scalar` | `dMass m; dMassSetSphereTotal(&m, 5.0, r); dBodySetMass(b, &m);` | There is no per-call mass setter and no `dMassSetMass`: total mass is set via the `*Total` setters or `dMassAdjust`, then handed to `dBodySetMass` (include/ode/objects.h:1153). |

## Named constants

This module defines no public named constants. (The `direction` argument to the capsule/cylinder mass setters takes the plain integers 1/2/3 for the x/y/z axis; include/ode/mass.h:55.)

## Things to never invent

These symbols do NOT exist in the ODE public API. Do not call or document them:

- `dMassSetMass` — no such function; use `dMassAdjust` or the `*Total` setters.
- `dBodyGetMassMatrix` and `dBodyGetMassMatrixInv` — read mass via `dBodyGetMass` into a `dMass`.
- `dBodySetLinearDampingMode` — not in this module.
- `dBodyGetVelocity` — use `dBodyGetLinearVel` / `dBodyGetAngularVel`.
- `dBodySetMassParameters` — it is `dMassSetParameters` on the `dMass`, not on the body.
- `dBodyAddImpulse` / `dBodyApplyImpulse` — no impulse API; accumulate force/torque instead.
- `dBodySetInertia` — set inertia through a `dMass`.
- `dMassScale` — it is `dMassAdjust`.
- `dBodyFreePosition` / freeing the pointer returned by `dBodyGetPosition` — the pointer is owned by ODE.
- `dBodySetStatic` — use `dBodySetKinematic`, or attach a geom with no body for static geometry.

## Appendix: canonical snippets

Canonical body + mass creation (ode/demo/demo_hinge.cpp:137):

```c
dMass m;
dMassSetBox (&m,1,SIDE,SIDE,SIDE);
dMassAdjust (&m,MASS);

dQuaternion q;
dQFromAxisAndAngle (q,1,1,0,0.25*M_PI);

body[0] = dBodyCreate (world);
dBodySetMass (body[0],&m);
dBodySetPosition (body[0],0.5*SIDE,0.5*SIDE,1);
dBodySetQuaternion (body[0],q);
```

Applying force and re-enabling jointed bodies (ode/demo/demo_crash.cpp:511):

```c
dBodyAddForce(body[bodies-1],lspeed,0,0);
...
dBodyEnable(dJointGetBody(joint[j],0));
dBodyEnable(dJointGetBody(joint[j],1));
```

> Notes: (1) `dQuaternion` element order is (w,x,y,z): the `dQuatElement` enum places `dQUE_R` (real/w) first, then the axis parts `dQUE_I/J/K` (include/ode/common.h:256). (2) [VERIFY] The claim that accumulated forces are cleared automatically after each `dWorldStep` is standard ODE behavior documented in the world/step section, not in the dBody force comments shown here; `dBodySetForce`'s comment about reactivating deactivated bodies (include/ode/objects.h:1235) implies but does not explicitly state the per-step clear.
