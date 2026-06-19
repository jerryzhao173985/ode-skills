# Walkthrough: `demo_boxstack.cpp` — primitive spawning, auto-disable, many-contact stacking

Source: `ode/demo/demo_boxstack.cpp` (620 lines). Where `demo_chain1.c` is the minimal
skeleton, boxstack is the canonical *interactive sandbox*: you press a key to drop a new
primitive (box / sphere / capsule / cylinder / convex / composite), bodies pile up on a
ground plane, and ODE's **auto-disable** puts settled bodies to sleep so a deep stack
stays cheap. It also shows the **many-contact** form of the near-callback (an array of
`dContact`, not a single one).

Every excerpt below is verbatim from the file, cited as `ode/demo/demo_boxstack.cpp:NN`.

---

## 1. World setup with damping + auto-disable + threading (`main`)

boxstack turns on far more world tuning than chain1. The key additions are the
**auto-disable** flags (so resting bodies stop being simulated) and the per-step
threading pool.

```c
    // create world
    dInitODE2(0);
    world = dWorldCreate();
    space = dHashSpaceCreate(0);
    contactgroup = dJointGroupCreate(0);
    dWorldSetGravity(world,0,0,-GRAVITY);
    dWorldSetCFM(world,1e-5);
    dWorldSetAutoDisableFlag(world,1);
```
— `ode/demo/demo_boxstack.cpp:577`

```c
    dWorldSetAutoDisableAverageSamplesCount( world, 10 );
```
— `ode/demo/demo_boxstack.cpp:588`

```c
    dWorldSetLinearDamping(world, 0.00001);
    dWorldSetAngularDamping(world, 0.005);
    dWorldSetMaxAngularSpeed(world, 200);

    dWorldSetContactMaxCorrectingVel(world,0.1);
    dWorldSetContactSurfaceLayer(world,0.001);
    dCreatePlane(space,0,0,1,0);
    memset(obj,0,sizeof(obj));
```
— `ode/demo/demo_boxstack.cpp:592`

Notes:

- **`dHashSpaceCreate(0)`** — hash space scales better than a simple space when many
  geoms pile up (`ode/demo/demo_boxstack.cpp:580`).
- **`dJointGroupCreate(0)`** — the contact group; the `0` size hint is the modern idiom
  (chain1 passed `1000000`; both work) (`ode/demo/demo_boxstack.cpp:581`).
- **`dWorldSetAutoDisableFlag(world,1)`** plus
  **`dWorldSetAutoDisableAverageSamplesCount(world,10)`** are the heart of the stacking
  demo: a body whose averaged motion stays below threshold is *disabled* (sleeps) and
  costs almost nothing until something hits it.
- **`dCreatePlane(space,0,0,1,0)`** — the static ground the stack rests on
  (`ode/demo/demo_boxstack.cpp:598`).
- **`memset(obj,0,sizeof(obj))`** zeroes the `MyObject obj[NUM]` table so empty slots
  have `body==0` and `geom[k]==0` (`ode/demo/demo_boxstack.cpp:599`).

The threading block builds a 4-thread pool and binds it to the world's step:

```c
    dThreadingImplementationID threading = dThreadingAllocateMultiThreadedImplementation();
    dThreadingThreadPoolID pool = dThreadingAllocateThreadPool(4, 0, dAllocateFlagBasicData, NULL);
    dThreadingThreadPoolServeMultiThreadedImplementation(pool, threading);
    // dWorldSetStepIslandsProcessingMaxThreadCount(world, 1);
    dWorldSetStepThreadingImplementation(world, dThreadingImplementationGetFunctions(threading), threading);
```
— `ode/demo/demo_boxstack.cpp:601`

---

## 2. The per-object table (`MyObject`)

boxstack does not use parallel `body[]`/`geom[]` arrays. It pairs one body with up to
`GPB` geoms in a struct, and tracks how many slots are live and which to recycle next.

```c
struct MyObject {
  dBodyID body;			// the body
  dGeomID geom[GPB];		// geometries representing this body
};
```
— `ode/demo/demo_boxstack.cpp:102`

```c
static int num=0;		// number of objects in simulation
static int nextobj=0;		// next object to recycle if num==NUM
```
— `ode/demo/demo_boxstack.cpp:107`

`GPB` is 3 (`ode/demo/demo_boxstack.cpp:95`) and `NUM` is 100, the max objects
(`ode/demo/demo_boxstack.cpp:93`). `num`/`nextobj` implement a ring buffer: once `NUM`
objects exist, the oldest slot is destroyed and reused.

---

## 3. Spawning a primitive on keypress (`command`)

### Allocate or recycle a slot, then a fresh body at a random pose

```c
    if (cmd == 'b' || cmd == 's' || cmd == 'c' || cmd == 'x' || cmd == 'y' || cmd == 'v') {
        if (num < NUM) {
            // new object to be created
            i = num;
            num++;
        } else {
            // recycle existing object
            i = nextobj++;
            nextobj %= num; // wrap-around if needed

            // destroy the body and geoms for slot i
            dBodyDestroy (obj[i].body);
            obj[i].body = 0;

            for (k=0; k < GPB; k++)
                if (obj[i].geom[k]) {
                    dGeomDestroy(obj[i].geom[k]);
                    obj[i].geom[k] = 0;
                }
        }

        obj[i].body = dBodyCreate(world);
```
— `ode/demo/demo_boxstack.cpp:222`

The recycle path is the reusable pattern: **`dBodyDestroy`** the old body, then
**`dGeomDestroy`** each non-null geom and null the slot, so the table never holds dangling
handles (`ode/demo/demo_boxstack.cpp:233`).

The new body then gets random side lengths and either a random drop position +
random orientation, or (with `random_pos` off) is placed above the current stack:

```c
        for (k=0; k<3; k++)
            sides[k] = dRandReal()*0.5+0.1;

        dMatrix3 R;
        if (random_pos)  {
            dBodySetPosition(obj[i].body,
                             dRandReal()*2-1,dRandReal()*2-1,dRandReal()+2);
            dRFromAxisAndAngle(R,dRandReal()*2.0-1.0,dRandReal()*2.0-1.0,
                               dRandReal()*2.0-1.0,dRandReal()*10.0-5.0);
        } else {
```
— `ode/demo/demo_boxstack.cpp:245`

`dRandReal()` is ODE's `[0,1)` RNG; `dRFromAxisAndAngle` builds a rotation matrix. The
result is fed to `dBodySetRotation(obj[i].body,R)` at `ode/demo/demo_boxstack.cpp:267`.

### One geom per simple primitive

Each shape key sets a matching `dMass` *and* creates the matching geom in `space`:

```c
        if (cmd == 'b') {

            dMassSetBox(&m,DENSITY,sides[0],sides[1],sides[2]);
            obj[i].geom[0] = dCreateBox(space,sides[0],sides[1],sides[2]);

        } else if (cmd == 'c') {

            sides[0] *= 0.5;
            dMassSetCapsule(&m,DENSITY,3,sides[0],sides[1]);
            obj[i].geom[0] = dCreateCapsule (space,sides[0],sides[1]);
```
— `ode/demo/demo_boxstack.cpp:269`

```c
        } else if (cmd == 'y') {

            dMassSetCylinder(&m,DENSITY,3,sides[0],sides[1]);
            obj[i].geom[0] = dCreateCylinder(space,sides[0],sides[1]);

        } else if (cmd == 's') {

            sides[0] *= 0.5;
            dMassSetSphere (&m,DENSITY,sides[0]);
            obj[i].geom[0] = dCreateSphere (space,sides[0]);
```
— `ode/demo/demo_boxstack.cpp:299`

The invariant: **for every primitive you compute a `dMass` with the matching
`dMassSet*` and create the geom with the matching `dCreate*`** — box↔box, sphere↔sphere,
capsule↔capsule, cylinder↔cylinder. The `3` argument to `dMassSetCapsule`/
`dMassSetCylinder` is the long axis (z).

### Bind geoms to the body and assign mass (non-composite path)

```c
        if (!setBody) { // avoid calling for composite geometries
            for (k=0; k < GPB; k++)
                if (obj[i].geom[k])
                    dGeomSetBody(obj[i].geom[k],obj[i].body);

            dBodySetMass(obj[i].body,&m);
        }
```
— `ode/demo/demo_boxstack.cpp:363`

`dGeomSetBody` binds each geom to the body so it tracks the body's pose, then
`dBodySetMass` installs the accumulated mass. The `setBody` flag skips this for the `'x'`
composite path (next section), which already bound its geoms.

---

## 4. Composite body (`'x'`) — accumulate masses, offset geoms, recenter on COM

The `'x'` key builds one body from `GPB` geoms (sphere + box + capsule), each at a random
offset. This is the reference pattern for multi-geom bodies.

```c
        } else if (cmd == 'x') {

            setBody = true;
            // start accumulating masses for the composite geometries
            dMass m2;
            dMassSetZero (&m);

            dReal dpos[GPB][3];	// delta-positions for composite geometries
            dMatrix3 drot[GPB];
```
— `ode/demo/demo_boxstack.cpp:310`

For each sub-geom it builds a `dMass m2`, rotates and translates it to the geom's local
offset, then **accumulates** it into the running total `m` with `dMassAdd`:

```c
                dRFromAxisAndAngle(drot[k],dRandReal()*2.0-1.0,dRandReal()*2.0-1.0,
                                   dRandReal()*2.0-1.0,dRandReal()*10.0-5.0);
                dMassRotate(&m2,drot[k]);

                dMassTranslate(&m2,dpos[k][0],dpos[k][1],dpos[k][2]);

                // add to the total mass
                dMassAdd(&m,&m2);
```
— `ode/demo/demo_boxstack.cpp:340`

After the total mass (and its center of mass `m.c`) is known, each geom is bound to the
body and given an offset *relative to the COM*, and finally the whole mass is translated
so the COM sits at the body origin:

```c
            for (k=0; k<GPB; k++) {
                dGeomSetBody(obj[i].geom[k],obj[i].body);
                dGeomSetOffsetPosition(obj[i].geom[k],
                                       dpos[k][0]-m.c[0],
                                       dpos[k][1]-m.c[1],
                                       dpos[k][2]-m.c[2]);
                dGeomSetOffsetRotation(obj[i].geom[k], drot[k]);
            }
            dMassTranslate(&m,-m.c[0],-m.c[1],-m.c[2]);
            dBodySetMass(obj[i].body,&m);
```
— `ode/demo/demo_boxstack.cpp:350`

The rule: **build the composite mass with `dMassAdd`, position geoms with
`dGeomSetOffsetPosition`/`dGeomSetOffsetRotation`, subtract `m.c` so the COM lands at the
body origin, then `dBodySetMass`.** ODE requires the COM to coincide with the body
origin, which is why the `-m.c[...]` correction appears in both the offsets and the final
`dMassTranslate`.

---

## 5. `nearCallback` — many contacts per pair

Unlike chain1's single `dContact`, boxstack declares an **array** of `MAX_CONTACTS`
contacts (box-box stacking needs several contact points to be stable) and asks `dCollide`
to fill up to that many.

```c
static void nearCallback (void *, dGeomID o1, dGeomID o2)
{
    int i;
    // if (o1->body && o2->body) return;

    // exit without doing anything if the two bodies are connected by a joint
    dBodyID b1 = dGeomGetBody(o1);
    dBodyID b2 = dGeomGetBody(o2);

    if (b1 && b2 && dAreConnectedExcluding(b1,b2,dJointTypeContact))
        return;

    dContact contact[MAX_CONTACTS];   // up to MAX_CONTACTS contacts per box-box
    for (i=0; i<MAX_CONTACTS; i++) {
        contact[i].surface.mode = dContactBounce | dContactSoftCFM;
        contact[i].surface.mu = dInfinity;
        contact[i].surface.mu2 = 0;
        contact[i].surface.bounce = 0.1;
        contact[i].surface.bounce_vel = 0.1;
        contact[i].surface.soft_cfm = 0.01;
    }
```
— `ode/demo/demo_boxstack.cpp:131`

Then `dCollide` is called once for the pair with `MAX_CONTACTS` as the flags/limit and
**`sizeof(dContact)`** as the stride (because the contacts are laid out as a `dContact[]`,
not a `dContactGeom[]`), and each returned contact becomes a contact joint:

```c
    if (int numc = dCollide(o1,o2,MAX_CONTACTS,&contact[0].geom,
                            sizeof(dContact))) {
        dMatrix3 RI;
        dRSetIdentity (RI);
        const dReal ss[3] = {0.02,0.02,0.02};
        for (i=0; i<numc; i++) {
            dJointID c = dJointCreateContact (world,contactgroup,contact+i);
            dJointAttach (c,b1,b2);
```
— `ode/demo/demo_boxstack.cpp:152`

Key differences from chain1's callback:

- **`dAreConnectedExcluding(b1,b2,dJointTypeContact)`** instead of `dAreConnected`: it
  excludes *contact* joints from the connectivity test, so two stacked bodies keep
  generating fresh contacts every step instead of being suppressed
  (`ode/demo/demo_boxstack.cpp:140`).
- **Surface mode `dContactBounce | dContactSoftCFM`** with `mu = dInfinity` (max
  friction), a small `bounce`, and `soft_cfm` — the surface params are set on *every*
  array entry before collision (`ode/demo/demo_boxstack.cpp:145`).
- **`dCollide(...,MAX_CONTACTS,&contact[0].geom,sizeof(dContact))`**: the limit is now an
  array size and the stride is `sizeof(dContact)`, not `sizeof(dContactGeom)`, because the
  output points stride across full `dContact` structs (`ode/demo/demo_boxstack.cpp:152`).
- **Loop `i<numc`** creating a contact joint per actual contact with
  `dJointCreateContact(world,contactgroup,contact+i)` and `dJointAttach`
  (`ode/demo/demo_boxstack.cpp:157`).

---

## 6. `simLoop` — collide → QuickStep → empty → draw, with disabled-body coloring

```c
static void simLoop(int pause)
{
    dSpaceCollide(space, 0, &nearCallback);

    if (!pause)
        dWorldQuickStep(world, 0.02);
```
— `ode/demo/demo_boxstack.cpp:508`

```c
    // remove all contact joints
    dJointGroupEmpty(contactgroup);

    dsSetTexture(DS_WOOD);
    for (int i=0; i<num; i++) {
        for (int j=0; j < GPB; j++) {
            if (i==selected) {
                dsSetColor(0,0.7,1);
            } else if (!dBodyIsEnabled(obj[i].body)) {
                dsSetColor(1,0.8,0);
            } else {
                dsSetColor(1,1,0);
            }
            drawGeom(obj[i].geom[j],0,0,show_aabb);
        }
    }
}
```
— `ode/demo/demo_boxstack.cpp:547`

Same three-call rhythm as chain1, with two boxstack-specific points:

- **`dWorldQuickStep(world,0.02)`** — the iterative solver, chosen because a tall stack of
  many bodies is exactly the large-system case QuickStep is for
  (`ode/demo/demo_boxstack.cpp:513`).
- **`dBodyIsEnabled(obj[i].body)`** — the draw loop colors auto-disabled (sleeping)
  bodies differently, the visible payoff of the auto-disable flags from `main`
  (`ode/demo/demo_boxstack.cpp:555`).

The teardown mirrors chain1 (plus the threading shutdown):

```c
    dJointGroupDestroy(contactgroup);
    dSpaceDestroy(space);
    dWorldDestroy(world);
    dCloseODE();
```
— `ode/demo/demo_boxstack.cpp:615`

---

## What to copy

- **Track objects in a struct**, not parallel arrays: `MyObject { dBodyID body;
  dGeomID geom[GPB]; }`, plus `num`/`nextobj` for a ring-buffer recycle.
  (`ode/demo/demo_boxstack.cpp:102`)
- **Recycle pattern:** when full, `dBodyDestroy` the old body and `dGeomDestroy` each
  non-null geom, then null the slots before reusing. (`ode/demo/demo_boxstack.cpp:233`)
- **Per primitive, pair the mass and the geom:** `dMassSet*` + the matching `dCreate*`
  (box/sphere/capsule/cylinder), then `dGeomSetBody` + `dBodySetMass`.
  (`ode/demo/demo_boxstack.cpp:269`)
- **Composite body recipe:** `dMassSetZero` → for each sub-geom `dMassSet*` /
  `dMassRotate` / `dMassTranslate` / `dMassAdd` → bind with `dGeomSetBody` +
  `dGeomSetOffsetPosition(... - m.c[...])` + `dGeomSetOffsetRotation` →
  `dMassTranslate(&m,-m.c...)` → `dBodySetMass`. (`ode/demo/demo_boxstack.cpp:310`)
- **Enable auto-disable for stacks:** `dWorldSetAutoDisableFlag(world,1)` and
  `dWorldSetAutoDisableAverageSamplesCount(world,10)`; check `dBodyIsEnabled` when you
  need to know what's asleep. (`ode/demo/demo_boxstack.cpp:584`)
- **Many-contact callback:** declare `dContact contact[MAX_CONTACTS]`, set
  `surface` on every entry, call
  `dCollide(o1,o2,MAX_CONTACTS,&contact[0].geom,sizeof(dContact))`, then create one
  contact joint per `numc`. Note the stride is **`sizeof(dContact)`** for an array.
  (`ode/demo/demo_boxstack.cpp:152`)
- **Use `dAreConnectedExcluding(b1,b2,dJointTypeContact)`** (not `dAreConnected`) so
  stacked bodies keep colliding step after step. (`ode/demo/demo_boxstack.cpp:140`)
- **Use `dWorldQuickStep`** for large many-body systems; `dWorldStep` for small accurate
  ones. (`ode/demo/demo_boxstack.cpp:513`)
- **Same per-step rhythm and teardown as chain1:** collide → step → `dJointGroupEmpty`;
  then `dJointGroupDestroy` → `dSpaceDestroy` → `dWorldDestroy` → `dCloseODE()`.
  (`ode/demo/demo_boxstack.cpp:508`, `ode/demo/demo_boxstack.cpp:615`)

---

## Never invent

Use only symbols you can grep in the ODE headers. In particular, contact joints are
created with `dJointCreateContact` (never `dContactJointCreate`); teardown is `dCloseODE`
(never `dShutdownODE`/`dDestroyODE`); spaces are `dHashSpaceCreate`/`dSimpleSpaceCreate`
(never a generic `dSpaceCreate`); and the world steppers are `dWorldStep`/`dWorldQuickStep`
(never `dWorldStep2`/`dBodyStep`). The composite path here uses `dMassAdd` and
`dGeomSetOffsetPosition`/`dGeomSetOffsetRotation` exactly as named — do not abbreviate or
guess variants. If you cannot confirm a symbol by grepping `include/ode/*.h`, it does not
exist — do not use it.
