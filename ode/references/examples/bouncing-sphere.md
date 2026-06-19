# Example: bouncing sphere — the smallest complete ODE program

> Source: wiki **Simple Bouncing Sphere** (`ode.org/wiki/index.php/Simple_Bouncing_Sphere`), grounded against `include/ode/*.h`. The minimal end-to-end composition of *every* component class — the best first program to read.

## What it builds
A sphere dropped from height 3 onto a static plane; it bounces (restitution 0.9) under weak gravity, rendered with drawstuff.

## Components composed
**World** + **Space** + contact **JointGroup** + a static ground **Geom** (`dCreatePlane`, no body) + one **Body**+**Mass**+**Geom** sphere linked by `dGeomSetBody`. The whole `references/components/README.md` graph in one tiny program.

## The program (verbatim from the wiki, lightly annotated)
```c
#include <ode/ode.h>
#include <drawstuff/drawstuff.h>
static dWorldID world;  static dSpaceID space;  static dBodyID body;
static dGeomID geom;    static dMass m;          static dJointGroupID contactgroup;

static void nearCallback(void *data, dGeomID o1, dGeomID o2) {
    dBodyID b1 = dGeomGetBody(o1), b2 = dGeomGetBody(o2);
    dContact contact;
    contact.surface.mode = dContactBounce | dContactSoftCFM;  // which fields are live
    contact.surface.mu = dInfinity;
    contact.surface.bounce = 0.9;        // restitution 0..1
    contact.surface.bounce_vel = 0.1;    // min impact speed to bounce
    contact.surface.soft_cfm = 0.001;
    if (dCollide(o1, o2, 1, &contact.geom, sizeof(dContact))) {
        dJointID c = dJointCreateContact(world, contactgroup, &contact);
        dJointAttach(c, b1, b2);
    }
}
static void simLoop(int pause) {
    dSpaceCollide(space, 0, &nearCallback);     // 1. contacts
    dWorldQuickStep(world, 0.01);               // 2. step (fixed dt)
    dJointGroupEmpty(contactgroup);             // 3. discard contacts
    const dReal *pos = dGeomGetPosition(geom);  // 4. render
    const dReal *R   = dGeomGetRotation(geom);
    dsDrawSphere(pos, R, dGeomSphereGetRadius(geom));
}
int main(int argc, char **argv) {
    dsFunctions fn; fn.version = DS_VERSION; fn.start = &start; fn.step = &simLoop;
    fn.stop = 0; fn.command = 0; fn.path_to_textures = "../../drawstuff/textures";
    dInitODE();
    world = dWorldCreate();
    space = dHashSpaceCreate(0);
    dWorldSetGravity(world, 0, 0, -0.2);
    dWorldSetCFM(world, 1e-5);
    dCreatePlane(space, 0, 0, 1, 0);            // static floor (a,b,c,d), NO body
    contactgroup = dJointGroupCreate(0);
    body = dBodyCreate(world);
    geom = dCreateSphere(space, 0.5);
    dMassSetSphere(&m, 1, 0.5); dBodySetMass(body, &m);
    dGeomSetBody(geom, body);                   // LINK geom<->body
    dBodySetPosition(body, 0, 0, 3);
    dsSimulationLoop(argc, argv, 352, 288, &fn);
    dJointGroupDestroy(contactgroup); dSpaceDestroy(space); dWorldDestroy(world); dCloseODE();
    return 0;
}
```
Cited: `dGeomGetPosition` `collision.h:181`, `dGeomGetRotation` `collision.h:210`, `dGeomSphereGetRadius` `collision.h:944`, `dGeomSetBody` `collision.h:105`, `dMassSetSphere` `mass.h:52`, plus the loop functions (`references/world-and-stepping.md`).

## Tokens set (see `references/tokens.md`)
`gravity=(0,0,-0.2)`, world `CFM=1e-5`, contact `bounce=0.9`/`bounce_vel=0.1`/`soft_cfm=0.001`, `dt=0.01`.

## Gotchas
- `dCollide(...,1,...)` requests **one** contact — fine for a sphere, but a flat box on a plane needs several (it would rest on a point and tip). Bump the count for resting stability.
- Uses the older `dInitODE()` (≡ `dInitODE2(0)` + allocate-all); new code prefers `dInitODE2`/`dAllocateODEDataForThread` (`references/world-and-stepping.md`).
- Relies on the **default ERP** (never calls `dWorldSetERP`).

## See also
`references/world-and-stepping.md` (the loop contract), `references/components/{world,body,geom,contact}.md`, `references/foundations/erp-cfm-friction.md` (what `bounce`/`soft_cfm` mean).
