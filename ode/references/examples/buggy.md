# Walkthrough: the hinge2 buggy (`demo_buggy.cpp`)

This is the canonical "how do I build a wheeled vehicle in ODE" example. It is a
chassis box plus three wheels, each wheel mounted on a **hinge2 joint** (steering
axis + rolling axis), with suspension, a drive motor and steering servo on the
front wheel, a per-car sub-space that prevents self-collision, and a friction
`nearCallback` tuned for tires on ground.

Everything below is excerpted verbatim from
`ode/demo/demo_buggy.cpp` and cited to the exact line range. The "What to copy"
section at the end distills the reusable recipe.

---

## 1. Globals: the world, the bodies, and the unit axes

The demo keeps everything in file-scope globals. Note `body[4]` (chassis +
3 wheels), `joint[3]` (one hinge2 per wheel, `joint[0]` is the front wheel),
and the two unit axis vectors `yunit`/`zunit` that the hinge2 axes are built
from.

> `ode/demo/demo_buggy.cpp:49-78`

```c
// some constants
#define LENGTH 0.7	// chassis length
#define WIDTH 0.5	// chassis width
#define HEIGHT 0.2	// chassis height
#define RADIUS 0.18	// wheel radius
#define STARTZ 0.5	// starting height of chassis
#define CMASS 1		// chassis mass
#define WMASS 0.2	// wheel mass

static const dVector3 yunit = { 0, 1, 0 }, zunit = { 0, 0, 1 };

// dynamics and collision objects (chassis, 3 wheels, environment)
static dWorldID world;
static dSpaceID space;
static dBodyID body[4];
static dJointID joint[3];	// joint[0] is the front wheel
static dJointGroupID contactgroup;
static dGeomID ground;
static dSpaceID car_space;
static dGeomID box[1];
static dGeomID sphere[3];
static dGeomID ground_box;

// things that the user controls
static dReal speed=0,steer=0;	// user commands
```

The whole buggy is steered by mutating just two scalars, `speed` and `steer`.

---

## 2. World / space / ground

> `ode/demo/demo_buggy.cpp:222-228`

```c
// create world
dInitODE2(0);
world = dWorldCreate();
space = dHashSpaceCreate (0);
contactgroup = dJointGroupCreate (0);
dWorldSetGravity (world,0,0,-0.5);
ground = dCreatePlane (space,0,0,1,0);
```

`dInitODE2(0)` boots the library; the top-level space is a hash space; the
ground is an infinite plane `(0,0,1,0)` (normal +Z, through the origin).

---

## 3. Chassis body: build mass, then attach a body-less geom

> `ode/demo/demo_buggy.cpp:230-237`

```c
// chassis body
body[0] = dBodyCreate (world);
dBodySetPosition (body[0],0,0,STARTZ);
dMassSetBox (&m,1,LENGTH,WIDTH,HEIGHT);
dMassAdjust (&m,CMASS);
dBodySetMass (body[0],&m);
box[0] = dCreateBox (0,LENGTH,WIDTH,HEIGHT);
dGeomSetBody (box[0],body[0]);
```

The pattern here is load-bearing and repeats for every part:

- `dMassSetBox(&m,1,...)` builds a **unit-density** mass, then
  `dMassAdjust(&m,CMASS)` rescales it to the desired total. (Rule:
  `ode/demo/demo_buggy.cpp:233-247`.)
- `dCreateBox(0,...)` creates the geom with space arg **`0`** — i.e. no space.
  `dGeomSetBody(box[0],body[0])` then binds it to the body, after which the
  geom follows the body automatically. (Rule: `ode/demo/demo_buggy.cpp:236-249`.)

---

## 4. Wheel bodies: pre-rotate, sphere mass, sphere geom, position front/back

> `ode/demo/demo_buggy.cpp:239-253`

```c
// wheel bodies
for (i=1; i<=3; i++) {
  body[i] = dBodyCreate (world);
  dQuaternion q;
  dQFromAxisAndAngle (q,1,0,0,M_PI*0.5);
  dBodySetQuaternion (body[i],q);
  dMassSetSphere (&m,1,RADIUS);
  dMassAdjust (&m,WMASS);
  dBodySetMass (body[i],&m);
  sphere[i-1] = dCreateSphere (0,RADIUS);
  dGeomSetBody (sphere[i-1],body[i]);
}
dBodySetPosition (body[1],0.5*LENGTH,0,STARTZ-HEIGHT*0.5);
dBodySetPosition (body[2],-0.5*LENGTH, WIDTH*0.5,STARTZ-HEIGHT*0.5);
dBodySetPosition (body[3],-0.5*LENGTH,-WIDTH*0.5,STARTZ-HEIGHT*0.5);
```

Two subtleties:

- **The collision shape is a sphere, even though the wheel is drawn as a
  cylinder.** `dMassSetSphere` + `dCreateSphere` give each wheel a sphere
  collider; the cylinder is purely a render choice (see §10). A sphere rolls
  cleanly and is cheap to collide. (Rule: `ode/demo/demo_buggy.cpp:242-249`.)
- Each wheel body is pre-rotated **+90° about X** with
  `dQFromAxisAndAngle(q,1,0,0,M_PI*0.5)` + `dBodySetQuaternion` so the
  render-cylinder lies flat across the car. (Rule:
  `ode/demo/demo_buggy.cpp:242-249`.)
- `body[1]` is the front wheel (placed at `+0.5*LENGTH`); `body[2]`/`body[3]`
  are the rear-left/rear-right wheels.

---

## 5. The hinge2 joints: one per wheel

This is the heart of the vehicle. A hinge2 has **two axes**: axis-1 is the
vertical steering/suspension axis, axis-2 is the horizontal rolling axis.

> `ode/demo/demo_buggy.cpp:255-262`

```c
// front and back wheel hinges
for (i=0; i<3; i++) {
  joint[i] = dJointCreateHinge2 (world,0);
  dJointAttach (joint[i],body[0],body[i+1]);
  const dReal *a = dBodyGetPosition (body[i+1]);
  dJointSetHinge2Anchor (joint[i],a[0],a[1],a[2]);
  dJointSetHinge2Axes (joint[i], zunit, yunit);
}
```

The conventions are exactly what you copy:

- **Attach order is chassis-then-wheel:** `dJointAttach(joint,body[0],body[i+1])`.
- **The anchor is the wheel's own center**, read straight from
  `dBodyGetPosition(body[i+1])` and passed to `dJointSetHinge2Anchor`.
- **`dJointSetHinge2Axes(joint, zunit, yunit)`** sets axis1 = Z (vertical,
  steering/suspension) and axis2 = Y (horizontal, rolling).

(Rule: `ode/demo/demo_buggy.cpp:255-262`.)

---

## 6. Suspension: the spring lives on axis-1

> `ode/demo/demo_buggy.cpp:264-268`

```c
// set joint suspension
for (i=0; i<3; i++) {
  dJointSetHinge2Param (joint[i],dParamSuspensionERP,0.4);
  dJointSetHinge2Param (joint[i],dParamSuspensionCFM,0.8);
}
```

Suspension is the spring along hinge2 axis-1, configured with
`dParamSuspensionERP` / `dParamSuspensionCFM` (here `0.4` / `0.8`). This is
what makes the wheel mounts springy instead of rigid. (Rule:
`ode/demo/demo_buggy.cpp:264-268`.)

---

## 7. Lock the rear wheels straight

> `ode/demo/demo_buggy.cpp:270-279`

```c
// lock back wheels along the steering axis
for (i=1; i<3; i++) {
  // set stops to make sure wheels always stay in alignment
  dJointSetHinge2Param (joint[i],dParamLoStop,0);
  dJointSetHinge2Param (joint[i],dParamHiStop,0);
  // the following alternative method is no good as the wheels may get out
  // of alignment:
  //   dJointSetHinge2Param (joint[i],dParamVel,0);
  //   dJointSetHinge2Param (joint[i],dParamFMax,dInfinity);
}
```

To make a wheel non-steering, clamp its hinge2 axis-1 with
`dParamLoStop = dParamHiStop = 0`. The in-code comment warns that the
velocity-motor alternative (`dParamVel=0` + `dParamFMax=dInfinity`) is worse
because the wheels can drift out of alignment. (Rule:
`ode/demo/demo_buggy.cpp:270-279`.)

---

## 8. The car sub-space (no self-collision) and the ramp obstacle

> `ode/demo/demo_buggy.cpp:281-294`

```c
// create car space and add it to the top level space
car_space = dSimpleSpaceCreate (space);
dSpaceSetCleanup (car_space,0);
dSpaceAdd (car_space,box[0]);
dSpaceAdd (car_space,sphere[0]);
dSpaceAdd (car_space,sphere[1]);
dSpaceAdd (car_space,sphere[2]);

// environment
ground_box = dCreateBox (space,2,1.5,1);
dMatrix3 R;
dRFromAxisAndAngle (R,0,1,0,-0.15);
dGeomSetPosition (ground_box,2,0,-0.34);
dGeomSetRotation (ground_box,R);
```

Two things happen here:

- **Self-collision is avoided** by putting the whole car (chassis box + 3 wheel
  spheres) in its own `dSimpleSpace` sub-space added to the top-level hash
  space. `dSpaceSetCleanup(car_space,0)` keeps the sub-space from destroying
  its geoms on teardown (so the demo can destroy them by hand later). (Rule:
  `ode/demo/demo_buggy.cpp:281-287`.)
- The inclined **ramp obstacle** is a static (body-less) box created directly in
  the world space (`dCreateBox(space,...)`), tilted about Y by `-0.15` rad with
  `dRFromAxisAndAngle`, and placed with `dGeomSetPosition` /
  `dGeomSetRotation`. (Rule: `ode/demo/demo_buggy.cpp:289-294`.)

---

## 9. Motor + steering, every frame, on the front wheel

> `ode/demo/demo_buggy.cpp:166-191`

```c
static void simLoop (int pause)
{
  int i;
  if (!pause) {
    // motor
    dJointSetHinge2Param (joint[0],dParamVel2,-speed);
    dJointSetHinge2Param (joint[0],dParamFMax2,0.1);

    // steering
    dReal v = steer - dJointGetHinge2Angle1 (joint[0]);
    if (v > 0.1) v = 0.1;
    if (v < -0.1) v = -0.1;
    v *= 10.0;
    dJointSetHinge2Param (joint[0],dParamVel,v);
    dJointSetHinge2Param (joint[0],dParamFMax,0.2);
    dJointSetHinge2Param (joint[0],dParamLoStop,-0.75);
    dJointSetHinge2Param (joint[0],dParamHiStop,0.75);
    dJointSetHinge2Param (joint[0],dParamFudgeFactor,0.1);

    dSpaceCollide (space,0,&nearCallback);
    dWorldStep (world,0.05);

    // remove all contact joints
    dJointGroupEmpty (contactgroup);
  }
```

This block carries three reusable ideas:

- **Drive motor is on axis-2 (rolling):** `dParamVel2` sets the target wheel
  spin (negated `speed`) and `dParamFMax2` caps the motor force at `0.1`. Only
  `joint[0]` (the front wheel) is driven. (Rule:
  `ode/demo/demo_buggy.cpp:171-172`.)
- **Steering is a proportional servo on axis-1:** the error
  `v = steer - dJointGetHinge2Angle1(joint[0])` is clamped to ±0.1, multiplied
  by gain 10, then driven via `dParamVel`/`dParamFMax`; runtime stops
  `dParamLoStop -0.75` / `dParamHiStop 0.75` bound the steering angle and
  `dParamFudgeFactor 0.1` stabilizes the motor. (Rule:
  `ode/demo/demo_buggy.cpp:175-183`.)
- **The per-step pipeline** is the standard ODE loop:
  `dSpaceCollide(space,0,&nearCallback)` → (inside the callback)
  `dCollide` + `dJointCreateContact` → `dWorldStep(world,0.05)` →
  `dJointGroupEmpty(contactgroup)` to discard the contact joints before the
  next frame. (Rule: `ode/demo/demo_buggy.cpp:185-189`.)

---

## 10. The `nearCallback`: car-vs-ground only, tire friction surface

> `ode/demo/demo_buggy.cpp:84-111`

```c
static void nearCallback (void *, dGeomID o1, dGeomID o2)
{
  int i,n;
  // only collide things with the ground
  int g1 = (o1 == ground || o1 == ground_box);
  int g2 = (o2 == ground || o2 == ground_box);
  if (!(g1 ^ g2)) return;

  const int N = 10;
  dContact contact[N];
  n = dCollide (o1,o2,N,&contact[0].geom,sizeof(dContact));
  if (n > 0) {
    for (i=0; i<n; i++) {
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

- **Contacts are restricted to car-vs-ground only.** Each geom is tagged as
  ground (the plane or `ground_box`), and the callback bails unless exactly one
  of the pair is ground — the XOR test `if (!(g1 ^ g2)) return;`. Because the
  car's own geoms also live in `car_space`, this means the car never generates
  contacts with itself. (Rule: `ode/demo/demo_buggy.cpp:88-91`.)
- **The tire friction surface** combines `mu = dInfinity` (no Coulomb slip
  limit) with explicit force-dependent slip `dContactSlip1|dContactSlip2`
  (`slip1=slip2=0.1`) for controlled lateral/longitudinal slip;
  `dContactSoftERP|dContactSoftCFM` (`soft_erp=0.5`, `soft_cfm=0.3`) soften the
  contact, and `dContactApprox1` uses the pyramid friction approximation. These
  flags are OR'd into `contact[i].surface.mode`. (Rule:
  `ode/demo/demo_buggy.cpp:98-104`.)

---

## 11. Drawing and user input

The render half of `simLoop` reads transforms straight from the simulation —
chassis as a box, wheels as cylinders, ramp from the geom:

> `ode/demo/demo_buggy.cpp:193-205`

```c
  dsSetColor (0,1,1);
  dsSetTexture (DS_WOOD);
  dReal sides[3] = {LENGTH,WIDTH,HEIGHT};
  dsDrawBox (dBodyGetPosition(body[0]),dBodyGetRotation(body[0]),sides);
  dsSetColor (1,1,1);
  for (i=1; i<=3; i++) dsDrawCylinder (dBodyGetPosition(body[i]),
                                       dBodyGetRotation(body[i]),0.02f,RADIUS);

  dVector3 ss;
  dGeomBoxGetLengths (ground_box,ss);
  dsDrawBox (dGeomGetPosition(ground_box),dGeomGetRotation(ground_box),ss);
```

The user controls just two globals via the drawstuff `command` callback:

> `ode/demo/demo_buggy.cpp:134-161`

```c
static void command (int cmd)
{
  switch (cmd) {
  case 'a': case 'A':
    speed += 0.3;
    break;
  case 'z': case 'Z':
    speed -= 0.3;
    break;
  case ',':
    steer -= 0.5;
    break;
  case '.':
    steer += 0.5;
    break;
  case ' ':
    speed = 0;
    steer = 0;
    break;
  case '1': {
      FILE *f = fopen ("state.dif","wt");
      if (f) {
        dWorldExportDIF (world,f,"");
        fclose (f);
      }
    }
  }
}
```

`a`/`z` adjust `speed` by ±0.3, `,`/`.` adjust `steer` by ±0.5, space resets
both, and `1` dumps the whole world to `state.dif` via `dWorldExportDIF`.
(Rule: `ode/demo/demo_buggy.cpp:134-161`.)

---

## 12. drawstuff wiring and teardown

> `ode/demo/demo_buggy.cpp:213-307`

```c
// setup pointers to drawstuff callback functions
dsFunctions fn;
fn.version = DS_VERSION;
fn.start = &start;
fn.step = &simLoop;
fn.command = &command;
fn.stop = 0;
fn.path_to_textures = DRAWSTUFF_TEXTURE_PATH;
// ...
// run simulation
dsSimulationLoop (argc, argv, DS_SIMULATION_DEFAULT_WIDTH,  DS_SIMULATION_DEFAULT_HEIGHT, &fn);

dGeomDestroy (box[0]);
dGeomDestroy (sphere[0]);
dGeomDestroy (sphere[1]);
dGeomDestroy (sphere[2]);
dJointGroupDestroy (contactgroup);
dSpaceDestroy (space);
dWorldDestroy (world);
dCloseODE();
```

Note the **explicit `dGeomDestroy` calls** for the car geoms: because
`dSpaceSetCleanup(car_space,0)` disabled the sub-space cleanup (§8), the demo
must free those four geoms by hand before `dSpaceDestroy`/`dWorldDestroy`/
`dCloseODE`. (Rule: `ode/demo/demo_buggy.cpp:223-306`.) The `start()` callback
(not shown) must call `dAllocateODEDataForThread(dAllocateMaskAll)` before
touching ODE on the render thread. (Rule:
`ode/demo/demo_buggy.cpp:116-122`.)

---

## What to copy — recipe for any wheeled vehicle

When you build your own car/rover/robot on wheels, lift this exact pattern:

1. **One hinge2 joint per wheel.** Create with `dJointCreateHinge2(world,0)`,
   attach **chassis first, wheel second** (`dJointAttach(joint,chassis,wheel)`),
   anchor at the wheel center (`dJointSetHinge2Anchor` with the wheel's own
   position), and set axes with `dJointSetHinge2Axes(joint, zunit, yunit)` so
   axis-1 = Z (steering) and axis-2 = Y (rolling).
   (`ode/demo/demo_buggy.cpp:255-262`.)

2. **Suspension = spring on axis-1.** `dParamSuspensionERP` /
   `dParamSuspensionCFM`. (`ode/demo/demo_buggy.cpp:264-268`.)

3. **Non-steering wheels: clamp axis-1 stops to 0.** `dParamLoStop = dParamHiStop
   = 0`. Don't fight them with a velocity motor.
   (`ode/demo/demo_buggy.cpp:270-279`.)

4. **Drive = motor on axis-2.** `dParamVel2` (target spin) + `dParamFMax2` (max
   force). **Steer = proportional servo on axis-1** reading
   `dJointGetHinge2Angle1`, driving `dParamVel`/`dParamFMax`, bounded by
   `dParamLoStop`/`dParamHiStop`, stabilized by `dParamFudgeFactor`.
   (`ode/demo/demo_buggy.cpp:166-191`.)

5. **Wheel mass + geom:** build a unit-density primitive then `dMassAdjust` to
   the target; create the geom with space arg `0` and bind with `dGeomSetBody`.
   A **sphere collider** rolls cheaply even if you render a cylinder; pre-rotate
   the body if the render shape needs it.
   (`ode/demo/demo_buggy.cpp:236-249`.)

6. **Put the vehicle in its own sub-space** (`dSimpleSpaceCreate(space)` +
   `dSpaceAdd`) so it never self-collides, and gate the `nearCallback` so only
   vehicle-vs-environment pairs make contacts. If you disable sub-space cleanup,
   destroy those geoms by hand on shutdown.
   (`ode/demo/demo_buggy.cpp:281-287`, `:88-91`, `:223-306`.)

7. **Tire surface:** `mu = dInfinity` plus bounded slip (`dContactSlip1/2`) for
   grip-with-controlled-slide; soften with `dContactSoftERP/CFM`; use
   `dContactApprox1` for the friction pyramid.
   (`ode/demo/demo_buggy.cpp:98-104`.)

8. **Per-step loop:** `dSpaceCollide` → `dCollide` + `dJointCreateContact` →
   `dWorldStep` → `dJointGroupEmpty`. (`ode/demo/demo_buggy.cpp:185-189`.)

---

## Never invent

- Do **not** add a hinge2 parameter, surface flag, or function that does not
  appear in the cited lines above. The exact set used here is:
  `dJointCreateHinge2`, `dJointAttach`, `dJointSetHinge2Anchor`,
  `dJointSetHinge2Axes`, `dJointSetHinge2Param`, `dJointGetHinge2Angle1`, and
  the params `dParamVel`/`dParamFMax`/`dParamVel2`/`dParamFMax2`/`dParamLoStop`/
  `dParamHiStop`/`dParamSuspensionERP`/`dParamSuspensionCFM`/`dParamFudgeFactor`.
- The header-level **values, units, and exact numeric meaning** of the
  `dParam*` constants are defined in `include/ode/common.h` /
  `include/ode/objects.h`, **not** in this demo — confirm them there, not from
  this file. [VERIFY]
- The exact semantics of `dContactApprox1` versus `dContactApprox1_1` /
  `dContactApprox1_2` (friction pyramid) live in `include/ode/contact.h`, not in
  this demo. [VERIFY]
- If you need a symbol not shown here, grep `include/ode/*.h` for it first; never
  paste an API name from memory.
