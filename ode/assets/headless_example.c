/*
 * falling_stack.c
 * ------------------------------------------------------------------------
 * A headless Open Dynamics Engine (ODE) demo: a tower of mixed boxes and
 * spheres is dropped onto a ground plane, collides with Coulomb friction,
 * tumbles, settles, and is then put to sleep by ODE's auto-disable system.
 *
 * It demonstrates, end to end:
 *   - the canonical ODE lifecycle (dInitODE2 -> world/space/contactgroup ->
 *     bodies+geoms -> per-step collide/step/empty -> teardown -> dCloseODE),
 *   - mixed primitives, each with a matching dMass and dCreate* geom,
 *   - a many-contact nearCallback with real Coulomb friction (dContactApprox1),
 *   - world-level auto-disable so a resting stack stops costing CPU.
 *
 * There is NO drawstuff dependency: Homebrew's libode ships the physics
 * library only, so this program drives its own loop and prints the state.
 *
 * Build (see Makefile):
 *   c++ -O2 -Wall $(ode-config --cflags) -o falling_stack falling_stack.c \
 *       $(ode-config --libs)
 *
 * Precision: do NOT hand-define dSINGLE/dDOUBLE. Including <ode/ode.h> pulls
 * in the build-generated precision.h which matches the linked library
 * (this Homebrew build is dDOUBLE). We print dGetConfiguration() to confirm.
 *
 * Every API call below was verified against the ODE 0.16.6 headers (include/ode).
 */

#include <stdio.h>
#include <stdlib.h>
#include <ode/ode.h>

/* ----------------------------- tunables -------------------------------- */

#define NUM_OBJECTS   14        /* objects in the initial tower            */
#define MAX_CONTACTS  8         /* max contact points per geom pair        */
#define MAX_STEPS     2000      /* hard cap on simulation steps            */
#define REPORT_EVERY  25        /* print a status line every N steps       */

static const dReal DT          = (dReal)0.01;   /* FIXED timestep (rule 5) */
static const dReal BOX_SIDE    = (dReal)0.40;   /* full side length        */
static const dReal SPH_RADIUS  = (dReal)0.20;
static const dReal OBJ_MASS    = (dReal)1.0;    /* uniform mass per object */
static const dReal LAYER_GAP   = (dReal)0.45;   /* vertical spacing (small drop) */
static const dReal BASE_Z      = (dReal)0.30;   /* z of the lowest object  */
static const dReal MU          = (dReal)1.0;    /* Coulomb friction ratio  */

/* --------------------------- global state ------------------------------ */

static dWorldID      world;
static dSpaceID      space;
static dJointGroupID contactgroup;

/* per-object record: one body paired with one geom (boxstack idiom) */
struct Obj {
    dBodyID body;
    dGeomID geom;
    int     is_box;   /* 1 = box, 0 = sphere */
};
static struct Obj obj[NUM_OBJECTS];

static int contacts_this_step = 0;   /* diagnostic counter, reset each step */

/* ----------------------- collision near-callback ----------------------- */
/*
 * dSpaceCollide calls this for every pair of geoms whose AABBs overlap.
 * We narrowphase-test the pair with dCollide, then turn each contact point
 * into a transient contact joint that the next dWorldQuickStep will solve.
 */
static void nearCallback(void *data, dGeomID o1, dGeomID o2)
{
    (void)data;

    dBodyID b1 = dGeomGetBody(o1);   /* 0 for the static ground plane */
    dBodyID b2 = dGeomGetBody(o2);

    /* Skip pairs already joined by a non-contact joint (none here, but this
     * is the canonical guard; dJointTypeContact is excluded so stacked
     * bodies keep generating fresh contacts every step). */
    if (b1 && b2 && dAreConnectedExcluding(b1, b2, dJointTypeContact))
        return;

    /* dCollide writes into an ARRAY: capacity == 3rd arg, start == &c[0].geom,
     * stride == sizeof(dContact) because the dContactGeom is embedded in the
     * larger dContact (rule 4 / the #1 reported ODE crash). */
    dContact contact[MAX_CONTACTS];
    int n = dCollide(o1, o2, MAX_CONTACTS, &contact[0].geom, sizeof(dContact));

    for (int i = 0; i < n; i++) {
        /* mode/mu are always read; every other field needs its flag set.
         * dContactApprox1 (0x7000) keeps mu a true mu*|F_N| ratio;
         * dContactSoftCFM softens the contact a touch to kill jitter;
         * dContactBounce gives dropped objects a little liveliness;
         * dContactRolling adds rolling/spinning resistance so spheres
         * actually stop (and therefore auto-disable) instead of rolling
         * across the plane forever -- sliding friction (mu) alone won't. */
        contact[i].surface.mode       = dContactBounce
                                      | dContactSoftCFM
                                      | dContactApprox1
                                      | dContactRolling;
        contact[i].surface.mu         = MU;        /* sliding friction (Coulomb) */
        contact[i].surface.rho        = (dReal)0.1;    /* rolling friction      */
        contact[i].surface.rho2       = (dReal)0.1;    /* rolling friction (2)  */
        contact[i].surface.rhoN       = (dReal)0.1;    /* spinning friction     */
        contact[i].surface.bounce     = (dReal)0.10;   /* restitution 0..1      */
        contact[i].surface.bounce_vel = (dReal)0.10;   /* min vel to bounce     */
        contact[i].surface.soft_cfm   = (dReal)1e-5;   /* per-contact softness  */

        dJointID c = dJointCreateContact(world, contactgroup, &contact[i]);
        dJointAttach(c, b1, b2);
        contacts_this_step++;
    }
}

/* --------------------------- world building ---------------------------- */

/* deterministic jitter in [-mag, +mag] so the tower isn't a degenerate
 * perfectly-aligned column (which stacks unrealistically). */
static dReal jitter(dReal mag)
{
    return (dReal)(((double)rand() / (double)RAND_MAX) * 2.0 - 1.0) * mag;
}

static void build_stack(void)
{
    for (int i = 0; i < NUM_OBJECTS; i++) {
        obj[i].is_box = (i % 2 == 0);   /* alternate box / sphere */
        obj[i].body   = dBodyCreate(world);

        dReal x = jitter((dReal)0.02);
        dReal y = jitter((dReal)0.02);
        dReal z = BASE_Z + i * LAYER_GAP;
        dBodySetPosition(obj[i].body, x, y, z);

        /* Each primitive pairs a matching dMass with the matching dCreate*.
         * dMassAdjust then rescales to a uniform total mass so contacts
         * between any two objects behave consistently. */
        dMass m;
        if (obj[i].is_box) {
            dMassSetBox(&m, (dReal)1.0, BOX_SIDE, BOX_SIDE, BOX_SIDE);
            dMassAdjust(&m, OBJ_MASS);
            obj[i].geom = dCreateBox(space, BOX_SIDE, BOX_SIDE, BOX_SIDE);
        } else {
            dMassSetSphere(&m, (dReal)1.0, SPH_RADIUS);
            dMassAdjust(&m, OBJ_MASS);
            obj[i].geom = dCreateSphere(space, SPH_RADIUS);
        }

        /* The crux: bind the (massless) geom to the (shapeless) body so the
         * shape tracks the dynamics, then install the mass. */
        dGeomSetBody(obj[i].geom, obj[i].body);
        dBodySetMass(obj[i].body, &m);
    }
}

/* --------------------------- status reporting -------------------------- */

static int count_enabled(void)
{
    int n = 0;
    for (int i = 0; i < NUM_OBJECTS; i++)
        if (dBodyIsEnabled(obj[i].body)) n++;
    return n;
}

static dReal max_height(void)
{
    dReal hi = (dReal)-1e30;
    for (int i = 0; i < NUM_OBJECTS; i++) {
        const dReal *p = dBodyGetPosition(obj[i].body);  /* internal ptr */
        if (p[2] > hi) hi = p[2];
    }
    return hi;
}

/* ------------------------------- main ---------------------------------- */

int main(void)
{
    /* 1. boot the library (must precede every other ODE call) and allocate
     *    this thread's collision data (required before any dCollide). */
    dInitODE2(0);
    dAllocateODEDataForThread(dAllocateMaskAll);

    printf("ODE configuration: %s\n", dGetConfiguration());
    printf("sizeof(dReal) = %zu bytes (%s precision)\n\n",
           sizeof(dReal), sizeof(dReal) == 8 ? "double" : "single");

    /* 2. dynamics world, broadphase space, and the per-step contact group. */
    world        = dWorldCreate();
    space        = dHashSpaceCreate(0);
    contactgroup = dJointGroupCreate(0);

    /* 3. global solver tuning (affects every step). */
    dWorldSetGravity(world, 0, 0, (dReal)-9.81);  /* default is (0,0,0)! */
    dWorldSetCFM(world, (dReal)1e-5);
    dWorldSetERP(world, (dReal)0.2);
    dWorldSetContactMaxCorrectingVel(world, (dReal)5.0); /* stop popping */
    dWorldSetContactSurfaceLayer(world, (dReal)0.001);   /* anti-jitter  */
    dWorldSetQuickStepNumIterations(world, 40);          /* deep stack    */

    /* 4. AUTO-DISABLE + damping are DEFAULTS copied to bodies at creation
     *    time, so set them BEFORE building the stack. A body whose averaged
     *    linear+angular speed stays under threshold for `Steps` steps goes
     *    to sleep and is skipped until something touches it. */
    dWorldSetAutoDisableFlag(world, 1);
    dWorldSetAutoDisableLinearThreshold(world, (dReal)0.01);
    dWorldSetAutoDisableAngularThreshold(world, (dReal)0.01);
    dWorldSetAutoDisableAverageSamplesCount(world, 10);
    dWorldSetAutoDisableSteps(world, 10);
    dWorldSetLinearDamping(world, (dReal)1e-4);
    dWorldSetAngularDamping(world, (dReal)5e-3);

    /* 5. static ground: a plane geom with NO body (z-up, through origin). */
    dCreatePlane(space, 0, 0, 1, 0);

    /* 6. the mixed tower. */
    srand(12345);   /* reproducible jitter */
    build_stack();

    printf("Dropping %d mixed objects (boxes + spheres), mu=%.2f, dt=%.3f\n",
           NUM_OBJECTS, (double)MU, (double)DT);
    printf("%-6s %-8s %-12s %-10s %-9s\n",
           "step", "time", "awake/total", "contacts", "max_z");
    printf("------------------------------------------------------------\n");

    /* 7. the simulation loop: collide -> step (FIXED dt) -> empty. */
    int settled_step = -1;
    int step;
    for (step = 0; step < MAX_STEPS; step++) {
        contacts_this_step = 0;

        dSpaceCollide(space, 0, &nearCallback);          /* a. broadphase */
        if (!dWorldQuickStep(world, DT)) {               /* b. integrate  */
            fprintf(stderr, "dWorldQuickStep: allocation failed at step %d\n",
                    step);
            break;
        }
        dJointGroupEmpty(contactgroup);                  /* c. drop contacts */

        int awake = count_enabled();
        if (step % REPORT_EVERY == 0 || awake == 0) {
            printf("%-6d %-8.2f %2d / %-7d %-10d %-9.3f\n",
                   step, (double)(step * DT), awake, NUM_OBJECTS,
                   contacts_this_step, (double)max_height());
        }

        /* everything asleep == the stack has come to rest */
        if (awake == 0 && step > 20) { settled_step = step; break; }
    }

    /* 8. results. */
    printf("------------------------------------------------------------\n");
    if (settled_step >= 0)
        printf("All bodies auto-disabled (at rest) at step %d (t=%.2fs).\n",
               settled_step, (double)(settled_step * DT));
    else
        printf("Reached step cap; %d/%d bodies still awake.\n",
               count_enabled(), NUM_OBJECTS);

    printf("\nFinal resting state:\n");
    printf("%-4s %-7s %-9s %-9s %-9s %-8s\n",
           "idx", "type", "x", "y", "z", "asleep");
    for (int i = 0; i < NUM_OBJECTS; i++) {
        const dReal *p = dBodyGetPosition(obj[i].body);
        printf("%-4d %-7s %-9.3f %-9.3f %-9.3f %-8s\n",
               i, obj[i].is_box ? "box" : "sphere",
               (double)p[0], (double)p[1], (double)p[2],
               dBodyIsEnabled(obj[i].body) ? "no" : "yes");
    }

    /* 9. teardown: destroying the space frees its geoms and destroying the
     *    world frees its bodies, so we only destroy group/space/world. */
    dJointGroupDestroy(contactgroup);
    dSpaceDestroy(space);
    dWorldDestroy(world);
    dCloseODE();
    return 0;
}
