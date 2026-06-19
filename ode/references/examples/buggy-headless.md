# Example: headless hinge2 buggy with a trajectory self-check

A standalone, headless port of ODE's canonical `demo_buggy`: a chassis box on three
spheres, one **hinge2** joint per wheel (steering/suspension on axis1, rolling on axis2),
soft springy suspension, front-wheel drive with a proportional steering servo, and a
tilted ground-box ramp. The drawstuff renderer is gone; in its place is an **autopilot**
that cruises the buggy at the ramp and a **headless self-check** that traces the chassis
trajectory and grades the run with six objective PASS/FAIL checks. It builds with
`ode-config`, runs with no window/GL/textures, prints a trace, and returns its verdict as
the process exit code. The shipped asset is `assets/demo_buggy.cpp`.

## What it demonstrates
- **The hinge2 vehicle idiom**: one `dJointCreateHinge2` per wheel, attached chassis→wheel, with axis1 = steering/suspension and axis2 = rolling — the standard ODE car suspension joint (`assets/demo_buggy.cpp:174-180`; `include/ode/objects.h:1725`).
- **Soft suspension as a deliberately mushy spring** along axis1 via `dParamSuspensionERP`/`dParamSuspensionCFM`, and locking the rear wheels straight with `dParamLoStop = dParamHiStop = 0` (`assets/demo_buggy.cpp:182-192`).
- **Actuation through joint motors, not forces**: a drive motor on axis2 (`dParamVel2`/`dParamFMax2`) and a P-servo on axis1 that reads back the live steering angle with `dJointGetHinge2Angle1` (`assets/demo_buggy.cpp:106-122`).
- **A tire contact surface** (`mu = dInfinity`, force-dependent slip, soft ERP/CFM, `dContactApprox1`) generated in a near-callback that only collides the car against the ground (`assets/demo_buggy.cpp:84-98`).
- **A self-collision-free sub-space**: the car's geoms go in a no-cleanup `dSimpleSpace` so the chassis and wheels never collide with each other (`assets/demo_buggy.cpp:194-200`).
- **A headless verification harness** with a vehicle-scale joint-integrity metric (Anchor vs Anchor2 read-back separation) and exit-code-as-verdict, proven live by adversarial falsification (`assets/demo_buggy.cpp:237-364`).

## Build & run
The program `#define`s **neither** `dSINGLE` nor `dDOUBLE`. It includes `<ode/ode.h>` only,
which pulls the installed library's own precision, then asserts the match at runtime with
`dCheckConfiguration` (`assets/demo_buggy.cpp:254`; `include/ode/common.h:573`). Let
`ode-config` supply the include path and libs so they always match the library:

```sh
c++ -O2 -std=c++14 -Wall assets/demo_buggy.cpp \
    $(ode-config --cflags) $(ode-config --libs) -o demo_buggy
./demo_buggy ; echo "exit=$?"
```

No window, no GL, no textures. The run prints a trajectory trace and a `checks:` block,
then `RESULT: PASS` or `RESULT: FAIL`. The verdict is the **exit code**, so it drops
straight into CI / a Makefile `verify` target:

```
RESULT: PASS        -> exit 0   (all six checks PASS)
RESULT: FAIL        -> exit 1   (any check FAIL)
precision mismatch  -> exit 2   (lib is not double precision)
```

Verified against Homebrew libode 0.16.6 (double precision): baseline exits `0` with
net forward travel ~12.55 m and worst wheel-joint Δ ~0.086 m (threshold `RADIUS` = 0.18).

## Walkthrough

### World, space, gravity, ground (`assets/demo_buggy.cpp:141-146`)
A world, a hash broadphase, a contact joint group, gentle gravity, and a static ground
plane (`a,b,c,d` = `0,0,1,0`, i.e. the `z = 0` plane, no body):

```cpp
world = dWorldCreate();
space = dHashSpaceCreate(0);
contactgroup = dJointGroupCreate(0);
dWorldSetGravity(world, 0, 0, -0.5);
ground = dCreatePlane(space, 0, 0, 1, 0);
```

`dCreatePlane` (`include/ode/collision.h:1045`) makes a *non-placeable* geom with no body —
the immovable floor.

### Chassis + three wheels (bodies, mass, geoms) (`assets/demo_buggy.cpp:148-171`)
The chassis is a box body; each wheel is a sphere body pre-rotated 90° about X so it rolls
upright. Mass is set by shape then rescaled with `dMassAdjust`, and each collider is linked
to its body with `dGeomSetBody`:

```cpp
dMassSetBox(&m, 1, LENGTH, WIDTH, HEIGHT);
dMassAdjust(&m, CMASS);
dBodySetMass(body[0], &m);
box[0] = dCreateBox(0, LENGTH, WIDTH, HEIGHT);  // space 0 -> added later
dGeomSetBody(box[0], body[0]);
```

Note `dCreateBox(0, ...)` / `dCreateSphere(0, ...)` create the geoms with **no space** (the
`0`); they are added to the car sub-space explicitly below, never to the top space directly.

### One hinge2 per wheel, axis1 = steer, axis2 = roll (`assets/demo_buggy.cpp:174-186`)
The vehicle joint. Each hinge2 is anchored at the wheel's center and given two axes —
`zunit` (steering/suspension, axis1) and `yunit` (rolling, axis2):

```cpp
joint[i] = dJointCreateHinge2(world, 0);
dJointAttach(joint[i], body[0], body[i + 1]);   // chassis = body1, wheel = body2
const dReal *a = dBodyGetPosition(body[i + 1]);
dJointSetHinge2Anchor(joint[i], a[0], a[1], a[2]);
dJointSetHinge2Axes(joint[i], zunit, yunit);
...
dJointSetHinge2Param(joint[i], dParamSuspensionERP, 0.4);  // soft spring along axis1
dJointSetHinge2Param(joint[i], dParamSuspensionCFM, 0.8);  // deliberately mushy
```

The **order of attach matters**: chassis is body1, wheel is body2, which is exactly what
makes `dJointGetHinge2Anchor` (body1's view) and `dJointGetHinge2Anchor2` (body2's view)
the right pair to diff later (`include/ode/objects.h:2070`, `:2084`, `:2110`, `:2673`).
Rear wheels are then frozen straight with zero stops (`assets/demo_buggy.cpp:188-192`):

```cpp
dJointSetHinge2Param(joint[i], dParamLoStop, 0);
dJointSetHinge2Param(joint[i], dParamHiStop, 0);
```

### Car sub-space so the buggy never self-collides (`assets/demo_buggy.cpp:194-200`)
```cpp
car_space = dSimpleSpaceCreate(space);   // nested inside the top space
dSpaceSetCleanup(car_space, 0);          // we own the geoms; don't auto-destroy them
dSpaceAdd(car_space, box[0]);
dSpaceAdd(car_space, sphere[0]); /* ... sphere[1], sphere[2] */
```

`dSpaceCollide` recurses into nested spaces but never pairs two geoms *within* the same
space against each other unless asked, so putting all four car geoms in one sub-space means
the wheels and chassis never generate contacts with one another (`include/ode/collision_space.h:52`,
`:75`, `:149`).

### The ramp (`assets/demo_buggy.cpp:202-207`)
A static, body-less box tilted about Y — the obstacle the buggy climbs. It is created
directly in the top `space` (unlike the car geoms), positioned, and rotated:

```cpp
ground_box = dCreateBox(space, 2, 1.5, 1);
dMatrix3 R;
dRFromAxisAndAngle(R, 0, 1, 0, -0.15);
dGeomSetPosition(ground_box, 2, 0, -0.34);
dGeomSetRotation(ground_box, R);
```

### Near-callback: car-vs-ground only, tire surface (`assets/demo_buggy.cpp:72-99`)
The XOR guard keeps the callback cheap and correct — it only builds contacts when *exactly
one* of the pair is ground, so car-vs-car is skipped and ground-vs-ground never happens:

```cpp
int g1 = (o1 == ground || o1 == ground_box);
int g2 = (o2 == ground || o2 == ground_box);
if (!(g1 ^ g2)) return;
const int N = 10;
dContact contact[N];
n = dCollide(o1, o2, N, &contact[0].geom, sizeof(dContact));
```

Each contact gets the tire surface (`assets/demo_buggy.cpp:86-92`) — infinite Coulomb
friction with force-dependent slip and soft contact, the same tuning as the original demo:

```cpp
contact[i].surface.mode = dContactSlip1 | dContactSlip2 |
  dContactSoftERP | dContactSoftCFM | dContactApprox1;
contact[i].surface.mu       = dInfinity;
contact[i].surface.slip1    = 0.1;   contact[i].surface.slip2 = 0.1;
contact[i].surface.soft_erp = 0.5;   contact[i].surface.soft_cfm = 0.3;
```

The mode flags name which `surface` fields are live (`include/ode/contact.h:38-51`); only
fields whose flag is set are read. The contact joint is created in `contactgroup` and
attached to the two bodies behind the colliding geoms.

### Controller: drive motor + P-servo (`assets/demo_buggy.cpp:106-122`)
The drive is a velocity motor on the front wheel's **rolling** axis (axis2); steering is a
proportional servo on the **steering** axis (axis1) that reads back the current angle:

```cpp
dJointSetHinge2Param(joint[0], dParamVel2,  -speed);   // drive on axis2
dJointSetHinge2Param(joint[0], dParamFMax2,  0.1);
dReal v = steer - dJointGetHinge2Angle1(joint[0]);     // error = target - actual
if (v >  0.1) v =  0.1;  if (v < -0.1) v = -0.1;        // clamp before gain
v *= 10.0;                                              // P gain
dJointSetHinge2Param(joint[0], dParamVel,  v);          // steer on axis1
dJointSetHinge2Param(joint[0], dParamFMax, 0.2);
```

Actuation is done entirely through joint motor params (`dParamVel`/`dParamFMax`, group-2
variants `dParamVel2`/`dParamFMax2`) — no `dBodyAddForce` anywhere. The `dParam*` enum is
built by macro in `include/ode/common.h:441,468-481`.

### Per-step loop (`assets/demo_buggy.cpp:124-130`)
The canonical ODE step order: controls → broadphase collide → integrate → flush contacts.

```cpp
applyControls();
dSpaceCollide(space, 0, &nearCallback);  // broadphase -> nearCallback -> contacts
dWorldStep(world, 0.05);                 // one fixed timestep
dJointGroupEmpty(contactgroup);          // discard THIS step's contacts
```

`dJointGroupEmpty` (not `Destroy`) is called every step: contact joints are transient and
must be cleared before the next collide pass, or they accumulate forever
(`include/ode/objects.h:1852`).

### Self-check harness: the joint-integrity metric (`assets/demo_buggy.cpp:237-249`)
The non-obvious check. It diffs each hinge2's two anchor read-backs and keeps the worst:

```cpp
dJointGetHinge2Anchor(joint[i], a1);    // anchor as seen from body1 (chassis)
dJointGetHinge2Anchor2(joint[i], a2);   // anchor as seen from body2 (wheel)
dReal d = std::sqrt(dx*dx + dy*dy + dz*dz);
if (d > worst) worst = d;
```

The two anchors are **meant to separate** by the suspension travel — the soft spring along
axis1 (`SuspensionERP=0.4`, `SuspensionCFM=0.8`) compresses under load, so a few cm of gap
is *healthy*, not a broken constraint (commentary at `assets/demo_buggy.cpp:227-236`). What
the metric really tests is that the gap stays **bounded at vehicle scale**: a genuine
blow-up (a wheel flying off, the constraint diverging) runs to *metres*, not cm.

### Self-check harness: the six checks + verdict (`assets/demo_buggy.cpp:333-364`)
The autopilot sets `speed = 10.0`, `steer = 0.0` and the loop runs 240 steps × 0.05 s = 12 s
(`assets/demo_buggy.cpp:265-270`), aggregating the trajectory each step. The verdict is six
physically motivated thresholds, not reverse-engineered numbers:

```cpp
int forward_ok  = (last_x - start_x > 0.5);          // drive really propels it
int reached_ok  = reached_ramp;                      // chassis x crossed 1.0
int straight_ok = (max_abs_y < 0.5);                 // steer=0 -> stays straight
int joint_ok    = (max_jsep < RADIUS);               // wheels stay mounted (0.18 m)
int grounded_ok = (!fell_through && min_z > -0.2);   // never tunneled the floor
int stable_ok   = (!exploded && max_speed < 50.0);   // no NaN, no blow-up
int ok = forward_ok && reached_ok && straight_ok &&
         joint_ok && grounded_ok && stable_ok;
return ok ? 0 : 1;                                   // verdict == exit code
```

The `joint_ok` threshold is the load-bearing one: bounding at **one wheel radius**
(`RADIUS` = 0.18 m) is the gap that *admits* the soft suspension travel (observed worst
~0.086 m, comfortably under) yet still *trips on a real blow-up*, where the separation runs
to metres (`assets/demo_buggy.cpp:338-340`). Exploded/NaN and below-floor are also tracked
every step (`assets/demo_buggy.cpp:299-302`) and break the loop early on `NaN`.

### Falsification: the harness actually flips
The checks are proven live by adversarial falsification — inject a regression, confirm the
program flips to `RESULT: FAIL` and exit code `1`:

- **Kill the drive** (`speed = 10.0` → `speed = 0.0`): the buggy travels ~0.01 m, so
  `forward_ok` and `reached_ok` go `FAIL`, `RESULT: FAIL`, **exit 1**.
- **Tighten the joint bound** (`max_jsep < RADIUS` → `max_jsep < 0.001`): the *healthy*
  suspension travel (~0.086 m) now exceeds the bound, `joint_ok` goes `FAIL`, **exit 1** —
  proving the metric is wired to a live, moving quantity and not a constant.

A check that cannot be made to fail proves nothing; both injections were run and confirmed
to flip the verdict while the unmodified program exits `0`.

## What to copy into your own program
The reusable skeleton for any wheeled / suspended ODE rig with a CI-grade self-check:

1. **Precision guard first**: `if (!dCheckConfiguration("ODE_double_precision")) return 2;`
   before any `dInit`. Catches the #1 silent integration bug — a single-precision lib under
   double-precision code (`include/ode/common.h:573`).
2. **hinge2 = car suspension joint**: create one per wheel, `dJointAttach(j, chassis, wheel)`
   in **that order**, anchor at the wheel center, axis1 = steering+suspension, axis2 = roll.
   Drive on axis2 motor params (`dParamVel2`/`dParamFMax2`); steer with a P-servo on axis1
   that reads `dJointGetHinge2Angle1`; lock unsteered wheels with `LoStop = HiStop = 0`.
3. **Soft suspension is a feature, not a bug**: tune `dParamSuspensionERP`/`...CFM` low and
   expect anchor read-backs to separate by the travel. Bound integrity at a *physical scale*
   (one wheel radius), never at zero.
4. **One sub-space per articulated body** (`dSimpleSpaceCreate(space)` + `dSpaceSetCleanup(.,0)`)
   so its parts never self-collide; free those hand-owned geoms yourself at teardown.
5. **The fixed step order** every frame: `applyControls()` → `dSpaceCollide` → `dWorldStep(dt)`
   → `dJointGroupEmpty(contactgroup)`. Same `dt` for the whole run.
6. **Grade with bounded trajectory invariants** (net forward, lateral drift, min_z, max
   speed, NaN, joint Δ) and `return ok ? 0 : 1;`. Then **falsify**: inject each regression
   and confirm the harness flips. Verdict-as-exit-code makes it a one-line CI gate.

## Gotchas this program handles
- **Precision is asserted, not assumed.** The file defines neither `dSINGLE` nor `dDOUBLE`
  and instead runtime-checks `dCheckConfiguration("ODE_double_precision")`, exiting `2` on
  mismatch (`assets/demo_buggy.cpp:254-259`). ODE silently miscomputes if the macro and the
  linked lib disagree; `ode-config --cflags` plus this guard removes the guesswork.
- **The joint-integrity bound is vehicle-scale, not zero.** A naive check would assert the
  two hinge2 anchors coincide and would *fail on a healthy buggy*, because the soft
  suspension is supposed to let them drift. The threshold is `RADIUS` = 0.18 m — large
  enough to admit suspension travel, small enough to catch a real divergence
  (`assets/demo_buggy.cpp:227-236, 338-340`).
- **Attach order is load-bearing for the metric.** `dJointGetHinge2Anchor` returns the
  anchor from body1's frame and `Anchor2` from body2's; the diff is only meaningful because
  the joint was attached chassis-first, wheel-second (`assets/demo_buggy.cpp:176, 242-243`;
  `include/ode/objects.h:2673`).
- **Contacts are emptied, not destroyed, each step.** Using `dJointGroupEmpty` per frame
  (vs `dJointGroupDestroy` once) is the correct lifecycle for the transient contact joints
  rebuilt every collide pass (`assets/demo_buggy.cpp:129`; `include/ode/objects.h:1852`).
- **The near-callback XOR-guards to car-vs-ground only.** `if (!(g1 ^ g2)) return;` skips
  both car-vs-car and ground-vs-ground pairs in one test, so the wheels never fight the
  chassis through contacts (the sub-space already prevents that) and the static ramp never
  collides with the static plane (`assets/demo_buggy.cpp:77-79`).
- **Car geoms live in `space 0`, then a no-cleanup sub-space — so they're freed by hand.**
  Because `dSpaceSetCleanup(car_space, 0)` is set, destroying the space won't destroy the
  geoms; the teardown calls `dGeomDestroy` on `box[0]` and the three spheres explicitly to
  avoid a leak (`assets/demo_buggy.cpp:194-196, 210-217`).
- **`dContactApprox1` plus `mu = dInfinity` is a tire, not glue.** Infinite Coulomb friction
  with the box-friction approximation and force-dependent slip (`slip1/slip2`) models a tire
  that grips but still creeps; without `dContactApprox1` the default pyramid friction with
  `dInfinity` mu behaves very differently (`assets/demo_buggy.cpp:86-92`;
  `include/ode/contact.h:48-51`).
