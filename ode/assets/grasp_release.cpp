// grasp_release.cpp — runtime topology change: GRASP an object by creating a joint mid-simulation,
// carry it, then RELEASE by destroying that joint. The textbook test of "constraints have lifetimes."
//
//   build:  c++ -O2 -std=c++17 $(ode-config --cflags) grasp_release.cpp $(ode-config --libs) -o grasp_release
//   run:    ./grasp_release            # 0 = PASS, 1 = FAIL, 2 = precision mismatch
//   falsify ./grasp_release --bug=no-grasp     # never create the joint  -> "lifted" check FAILs
//           ./grasp_release --bug=no-release    # never destroy the joint -> "released" check FAILs
//
// A KINEMATIC gripper (scripted path, unaffected by forces) descends to a box resting on the ground,
// grasps it with a FIXED joint, lifts+carries it 1 m, and releases it. The grasp joint — not contact —
// couples them, so the gripper<->object contact pair is suppressed in the near-callback.
#include <ode/ode.h>
#include <cstdio>
#include <cstring>
#include <cmath>

static dWorldID world; static dSpaceID space; static dJointGroupID cg;
static dGeomID gripGeom, objGeom;                 // to suppress the grip<->object contact pair

// Near-callback: standard contact pipeline, but the grasp joint (not contact) couples gripper+object,
// so skip that specific pair — a redundant contact would fight the fixed joint (a mini closed loop).
static void nearCallback(void *, dGeomID o1, dGeomID o2) {
    if ((o1 == gripGeom && o2 == objGeom) || (o1 == objGeom && o2 == gripGeom)) return;  // ignored pair
    dBodyID b1 = dGeomGetBody(o1), b2 = dGeomGetBody(o2);
    dContact c[8];
    for (auto &x : c) { x.surface.mode = dContactApprox1; x.surface.mu = 1.0; }
    int n = dCollide(o1, o2, 8, &c[0].geom, sizeof(dContact));
    for (int i = 0; i < n; i++) dJointAttach(dJointCreateContact(world, cg, &c[i]), b1, b2);
}

int main(int argc, char **argv) {
    int bug = 0;                                  // 1 = no-grasp, 2 = no-release
    for (int i = 1; i < argc; i++) {
        if (!strcmp(argv[i], "--bug=no-grasp"))   bug = 1;
        if (!strcmp(argv[i], "--bug=no-release")) bug = 2;
    }
    if (!dInitODE2(0)) return 2;
    if (!dCheckConfiguration("ODE_double_precision")) { printf("precision mismatch\n"); return 2; }
    dAllocateODEDataForThread(dAllocateMaskAll);
    world = dWorldCreate(); space = dHashSpaceCreate(0); cg = dJointGroupCreate(0);
    dWorldSetGravity(world, 0, 0, -9.81);
    dWorldSetERP(world, 0.2); dWorldSetCFM(world, 1e-5);
    dCreatePlane(space, 0, 0, 1, 0);              // static ground (no body)

    // Object: a 0.4^3 box resting on the ground at the origin.
    const double H = 0.2;                         // half-height
    dBodyID obj = dBodyCreate(world); dBodySetPosition(obj, 0, 0, H);
    dMass m; dMassSetBoxTotal(&m, 1.0, 0.4, 0.4, 0.4); dBodySetMass(obj, &m);
    objGeom = dCreateBox(space, 0.4, 0.4, 0.4); dGeomSetBody(objGeom, obj);

    // Gripper: a KINEMATIC box driven along a scripted path (forces don't move it).
    dBodyID grip = dBodyCreate(world); dBodySetKinematic(grip);
    dBodySetPosition(grip, 0, 0, 2.0);
    gripGeom = dCreateBox(space, 0.3, 0.3, 0.2); dGeomSetBody(gripGeom, grip);

    const double DT = 0.005;
    const int DESCEND = 200, LIFT = 200, CARRY = 300, SETTLE = 400;   // phase step counts
    const int GRASP_AT = DESCEND, RELEASE_AT = DESCEND + LIFT + CARRY;
    const int STEPS = DESCEND + LIFT + CARRY + SETTLE;
    dJointID grasp = 0;

    double objMaxZ = -1e9, gripObjGapAtHold = 0;  bool tracked = true, anyNaN = false;
    for (int i = 0; i < STEPS; i++) {
        // ----- scripted kinematic gripper target (z-up) -----
        double tz, tx = 0;
        if (i < DESCEND)                         tz = 2.0 - (2.0 - 0.45) * (double)i / DESCEND;        // down to grip height
        else if (i < DESCEND + LIFT)             tz = 0.45 + (1.0 - 0.45) * (double)(i - DESCEND) / LIFT;   // up to z=1
        else if (i < RELEASE_AT) { tz = 1.0;     tx = 1.0 * (double)(i - DESCEND - LIFT) / CARRY; }   // sideways to x=1
        else                     { tz = 1.0;     tx = 1.0; }                                          // hold above drop point
        const dReal *gp = dBodyGetPosition(grip);
        dBodySetLinearVel(grip, (dReal)((tx - gp[0]) / DT), 0, (dReal)((tz - gp[2]) / DT));  // velocity-drive the kinematic body
        dBodySetAngularVel(grip, 0, 0, 0);

        // ----- runtime topology change: GRASP -----
        if (i == GRASP_AT && bug != 1) {
            dBodyEnable(obj);                                    // a resting body may be auto-disabled — wake it
            grasp = dJointCreateFixed(world, 0);
            dJointAttach(grasp, grip, obj);
            dJointSetFixed(grasp);                              // lock the CURRENT relative pose
        }
        // ----- RELEASE -----
        if (i == RELEASE_AT && grasp && bug != 2) { dJointDestroy(grasp); grasp = 0; }

        dSpaceCollide(space, 0, &nearCallback);
        if (!dWorldStep(world, DT)) { printf("step alloc failed\n"); return 1; }
        dJointGroupEmpty(cg);

        const dReal *op = dBodyGetPosition(obj);
        for (int k = 0; k < 3; k++) if (!std::isfinite(op[k])) anyNaN = true;
        if (i > GRASP_AT && i < RELEASE_AT) {                   // HELD phase
            objMaxZ = fmax(objMaxZ, op[2]);
            double gap = fabs(op[2] - dBodyGetPosition(grip)[2]);   // object should track gripper rigidly
            if (i == RELEASE_AT - 1) gripObjGapAtHold = gap;
            if (gap > 0.6) tracked = false;                    // object fell off the gripper
        }
    }
    const dReal *op = dBodyGetPosition(obj);
    // ----- 3-phase verdict (emergent, falsifiable) -----
    bool lifted   = objMaxZ > 0.6;                             // it left the ground while held
    bool carried  = fabs(op[0] - 1.0) < 0.35;                 // ended near the drop point
    bool released = op[2] < 0.5 && fabs(op[2] - dBodyGetPosition(grip)[2]) > 0.4;  // back near ground, NOT at gripper
    bool finite   = !anyNaN;
    bool ok = lifted && carried && released && finite && tracked;
    printf("grasp/release  objMaxZ=%.3f finalPos=(%.3f,%.3f,%.3f) gripZ=%.3f heldGap=%.4f\n",
           objMaxZ, op[0], op[1], op[2], (double)dBodyGetPosition(grip)[2], gripObjGapAtHold);
    printf("  [%s] lifted   [%s] carried  [%s] released  [%s] tracked  [%s] finite\n",
           lifted?"PASS":"FAIL", carried?"PASS":"FAIL", released?"PASS":"FAIL", tracked?"PASS":"FAIL", finite?"PASS":"FAIL");
    printf("RESULT: %s%s\n", ok?"PASS":"FAIL", bug?(bug==1?" (expected FAIL: no-grasp)":" (expected FAIL: no-release)"):"");
    dJointGroupDestroy(cg); dSpaceDestroy(space); dWorldDestroy(world); dCloseODE();
    return ok ? 0 : 1;
}
