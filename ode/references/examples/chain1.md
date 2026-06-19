# Walkthrough: `demo_chain1.c` — the simplest complete ODE program

Source: `ode/demo/demo_chain1.c` (172 lines). This is the smallest end-to-end ODE
program: it builds a 10-link chain of ball-jointed bodies (each with a sphere geom),
drives it with a wobbling force, and steps it inside the drawstuff render loop. Read
this first — every other demo is a superset of this skeleton.

Every excerpt below is verbatim from the file, cited as `ode/demo/demo_chain1.c:NN`.
The canonical ordering it demonstrates is summarized at `ode/demo/demo_chain1.c:139`.

---

## 1. Globals / setup (includes + handles)

The program pulls in the single umbrella header `ode/ode.h` plus the optional
drawstuff visualization harness, then declares every long-lived handle as a file-scope
`static`. This is the standard ODE ownership pattern: one world, one space, one contact
joint group, plus parallel arrays of bodies, permanent joints, and geoms.

```c
#include <stdio.h>
#include "ode/ode.h"
#include "drawstuff/drawstuff.h"
#include "texturepath.h"
```
— `ode/demo/demo_chain1.c:25`

```c
/* dynamics and collision objects */

static dWorldID world;
static dSpaceID space;
static dBodyID body[NUM];
static dJointID joint[NUM-1];
static dJointGroupID contactgroup;
static dGeomID sphere[NUM];
```
— `ode/demo/demo_chain1.c:52`

The compile-time constants drive the whole scene:

```c
#define NUM 10			/* number of boxes */
#define SIDE (0.2)		/* side length of a box */
#define MASS (1.0)		/* mass of a box */
#define RADIUS (0.1732f)	/* sphere radius */
```
— `ode/demo/demo_chain1.c:46`

Note `NUM` bodies but `NUM-1` joints — a chain of N links needs N-1 connections.
`world`, `space`, and `contactgroup` are the three objects every ODE program needs.

---

## 2. `nearCallback` — the collision handler

`dSpaceCollide` calls this once per *potentially* colliding geom pair. The job of the
callback is to (a) skip pairs that should not collide, (b) call `dCollide` to generate
real contact points, and (c) turn each contact into a transient **contact joint** that
the next world step will resolve.

```c
static void nearCallback (void *data, dGeomID o1, dGeomID o2)
{
  /* exit without doing anything if the two bodies are connected by a joint */
  dBodyID b1,b2;
  dContact contact;
  (void)data;

  b1 = dGeomGetBody(o1);
  b2 = dGeomGetBody(o2);
  if (b1 && b2 && dAreConnected (b1,b2)) return;

  contact.surface.mode = 0;
  contact.surface.mu = 0.1;
  contact.surface.mu2 = 0;
  if (dCollide (o1,o2,1,&contact.geom,sizeof(dContactGeom))) {
    dJointID c = dJointCreateContact (world,contactgroup,&contact);
    dJointAttach (c,b1,b2);
  }
}
```
— `ode/demo/demo_chain1.c:66`

Four load-bearing details:

- **`dGeomGetBody(o1)` / `dGeomGetBody(o2)`** fetch the bodies behind the geoms
  (`ode/demo/demo_chain1.c:73`). Returns `0` for a static geom like the plane.
- **`dAreConnected(b1,b2)`** lets adjacent chain links pass through each other so the
  ball joints — not contacts — define the chain (`ode/demo/demo_chain1.c:75`).
- **`dCollide(o1,o2,1,&contact.geom,sizeof(dContactGeom))`** writes up to 1 contact
  point into `contact.geom`. The last argument is the *byte stride* and **must** be
  `sizeof(dContactGeom)` (`ode/demo/demo_chain1.c:80`).
- **`dJointCreateContact` + `dJointAttach`** create the contact joint in `contactgroup`
  and bind it to the two bodies for this one step (`ode/demo/demo_chain1.c:81`).

---

## 3. `simLoop` — collide → step → empty

drawstuff calls `simLoop` once per frame. The non-pause branch shows the canonical
three-call per-step sequence; the rest of the function just draws the spheres.

```c
static void simLoop (int pause)
{
  int i;
  if (!pause) {
    static double angle = 0;
    angle += 0.05;
    dBodyAddForce (body[NUM-1],0,0,1.5*(sin(angle)+1.0));

    dSpaceCollide (space,0,&nearCallback);
    dWorldStep (world,0.05);

    /* remove all contact joints */
    dJointGroupEmpty (contactgroup);
  }
```
— `ode/demo/demo_chain1.c:101`

The order is mandatory:

1. **`dSpaceCollide(space,0,&nearCallback)`** — find pairs and (via the callback) build
   this step's contact joints (`ode/demo/demo_chain1.c:109`).
2. **`dWorldStep(world,0.05)`** — integrate one 0.05 s step with the accurate
   big-matrix solver (`ode/demo/demo_chain1.c:110`). `dWorldQuickStep` is a drop-in
   faster alternative with the same signature.
3. **`dJointGroupEmpty(contactgroup)`** — delete every contact joint made this step
   (`ode/demo/demo_chain1.c:113`). Skipping this leaks contact joints every frame until
   the sim explodes or stalls.

The drive force is applied each frame *before* collision:

```c
    dBodyAddForce (body[NUM-1],0,0,1.5*(sin(angle)+1.0));
```
— `ode/demo/demo_chain1.c:107`

Forces added with `dBodyAddForce` are accumulated for the next `dWorldStep` and cleared
automatically after it, so they must be re-applied every frame. The draw loop then reads
each body's pose with `dBodyGetPosition` / `dBodyGetRotation`
(`ode/demo/demo_chain1.c:118`).

---

## 4. `main` — init → build → run → teardown

`main` first fills the drawstuff callback struct, then runs the canonical ODE startup,
builds the scene, hands control to the render loop, and finally tears everything down in
reverse order.

### Init (drawstuff struct + ODE startup)

```c
  /* setup pointers to drawstuff callback functions */
  dsFunctions fn;
  fn.version = DS_VERSION;
  fn.start = &start;
  fn.step = &simLoop;
  fn.command = 0;
  fn.stop = 0;
  fn.path_to_textures = DRAWSTUFF_TEXTURE_PATH;

  /* create world */
  dInitODE2(0);
  world = dWorldCreate();
  space = dHashSpaceCreate (0);
  contactgroup = dJointGroupCreate (1000000);
  dWorldSetGravity (world,0,0,-0.5);
  dCreatePlane (space,0,0,1,0);
```
— `ode/demo/demo_chain1.c:129`

- **`dInitODE2(0)`** must be the very first ODE call (`ode/demo/demo_chain1.c:139`).
- **`dWorldCreate`** makes the dynamics world (`ode/demo/demo_chain1.c:140`).
- **`dHashSpaceCreate(0)`** makes a top-level hash collision space; `0` = no parent
  space (`ode/demo/demo_chain1.c:141`).
- **`dJointGroupCreate(1000000)`** makes the contact joint group
  (`ode/demo/demo_chain1.c:142`). The integer argument is a legacy size hint.
- **`dWorldSetGravity(world,0,0,-0.5)`** — z-down gravity (`ode/demo/demo_chain1.c:143`).
- **`dCreatePlane(space,0,0,1,0)`** — a static ground plane geom with no attached body
  (`ode/demo/demo_chain1.c:144`).

### Build the scene (bodies + geoms, then permanent joints)

```c
  for (i=0; i<NUM; i++) {
    body[i] = dBodyCreate (world);
    k = i*SIDE;
    dBodySetPosition (body[i],k,k,k+0.4);
    dMassSetBox (&m,1,SIDE,SIDE,SIDE);
    dMassAdjust (&m,MASS);
    dBodySetMass (body[i],&m);
    sphere[i] = dCreateSphere (space,RADIUS);
    dGeomSetBody (sphere[i],body[i]);
  }
  for (i=0; i<(NUM-1); i++) {
    joint[i] = dJointCreateBall (world,0);
    dJointAttach (joint[i],body[i],body[i+1]);
    k = (i+0.5)*SIDE;
    dJointSetBallAnchor (joint[i],k,k,k+0.4);
  }
```
— `ode/demo/demo_chain1.c:146`

Per body the pattern is: create body → set position → build a `dMass`
(`dMassSetBox` then `dMassAdjust` to the desired total) → assign it with `dBodySetMass`
→ create a geom → bind geom to body with **`dGeomSetBody`** so the geom tracks the
body's pose (`ode/demo/demo_chain1.c:154`).

The second loop wires the *permanent* ball joints: `dJointCreateBall(world,0)` — note
the `0` group, meaning a permanent joint, not a contact joint
(`ode/demo/demo_chain1.c:157`) — then `dJointAttach` between consecutive bodies and
`dJointSetBallAnchor` to place the pivot.

### Run + teardown

```c
  /* run simulation */
  dsSimulationLoop (argc, argv, DS_SIMULATION_DEFAULT_WIDTH, DS_SIMULATION_DEFAULT_HEIGHT, &fn);

  dJointGroupDestroy (contactgroup);
  dSpaceDestroy (space);
  dWorldDestroy (world);
  dCloseODE();
  return 0;
}
```
— `ode/demo/demo_chain1.c:163`

`dsSimulationLoop` blocks and repeatedly calls `simLoop` until the window closes. The
teardown is the reverse of setup (`ode/demo/demo_chain1.c:166`):

- **`dJointGroupDestroy(contactgroup)`** — drop the contact group.
- **`dSpaceDestroy(space)`** — also frees every geom in the space (no per-geom destroy).
- **`dWorldDestroy(world)`** — also frees every body and permanent joint in the world.
- **`dCloseODE()`** — paired with `dInitODE2`; must be the **last** ODE call.

---

## What to copy

- **Startup order, exactly:** `dInitODE2(0)` first → `dWorldCreate` → space
  (`dHashSpaceCreate(0)`) → `dJointGroupCreate(...)` → `dWorldSetGravity` → build bodies
  and geoms. (`ode/demo/demo_chain1.c:139`)
- **The three per-step calls, in order:** `dSpaceCollide(space,0,&nearCallback)` →
  `dWorldStep(world, dt)` (or `dWorldQuickStep`) → `dJointGroupEmpty(contactgroup)`.
  (`ode/demo/demo_chain1.c:109`)
- **The `nearCallback` template:** fetch `b1`/`b2` with `dGeomGetBody`; early-return on
  `dAreConnected`; set `contact.surface` fields; call
  `dCollide(o1,o2,N,&contact.geom,sizeof(dContactGeom))`; for each contact
  `dJointCreateContact(world,contactgroup,&contact)` then `dJointAttach(c,b1,b2)`.
  (`ode/demo/demo_chain1.c:66`)
- **`sizeof(dContactGeom)` as the `dCollide` stride** — never `0`, never
  `sizeof(dContact)` for a single contact. (`ode/demo/demo_chain1.c:80`)
- **Bind geom to body with `dGeomSetBody`** right after creating each geom so it follows
  the body. (`ode/demo/demo_chain1.c:154`)
- **Mass workflow:** `dMassSetBox` (or another `dMassSet*`) → `dMassAdjust` to the target
  total → `dBodySetMass`. (`ode/demo/demo_chain1.c:150`)
- **Permanent joints use group `0`:** `dJointCreateBall(world,0)` →`dJointAttach` →
  `dJointSet...Anchor`. (`ode/demo/demo_chain1.c:157`)
- **Re-apply forces every frame** — `dBodyAddForce` is cleared by each `dWorldStep`.
  (`ode/demo/demo_chain1.c:107`)
- **Teardown in reverse:** `dJointGroupDestroy` → `dSpaceDestroy` → `dWorldDestroy` →
  `dCloseODE()` last. Destroying the space frees its geoms; destroying the world frees
  its bodies/joints. (`ode/demo/demo_chain1.c:166`)
- **For a headless (non-graphical) app**, delete everything `ds*`/`drawstuff` and drive
  your own loop calling the same three per-step functions; none of the drawstuff API is
  part of ODE physics. (`ode/demo/demo_chain1.c:27`)

---

## Never invent

Only use symbols you can grep in the ODE headers. Names that do **not** exist in this
codebase (do not write them): `dWorldInit`, `dCreateWorld`, `dWorldStep2`,
`dContactJointCreate` (the real name is `dJointCreateContact`), `dInitODE3`,
`dShutdownODE` / `dDestroyODE` (the real teardown is `dCloseODE`), `dWorldSetTimeStep`,
`dBodyStep`, `dCollideGeoms`, and a generic `dSpaceCreate` (use `dHashSpaceCreate` or
`dSimpleSpaceCreate`). If you cannot confirm a symbol by grepping `include/ode/*.h`, it
does not exist — do not use it.
