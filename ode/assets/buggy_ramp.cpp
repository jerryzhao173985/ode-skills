/*************************************************************************
 * buggy_ramp.cpp                                                         *
 *                                                                        *
 * An ODE simulation of a 4-wheeled (well, 3-wheeled) buggy that drives   *
 * itself up and over a ramp, rendered with DrawStuff.                    *
 *                                                                        *
 * Physics recipe (all verified against the ODE headers in include/ode    *
 * and the canonical ode/demo/demo_buggy.cpp):                            *
 *   - chassis box body + 3 wheel bodies                                  *
 *   - one hinge2 joint per wheel: axis1 = steering/suspension (z),       *
 *     axis2 = rolling (y)                                                *
 *   - suspension = spring on hinge2 axis1 (dParamSuspensionERP/CFM)      *
 *   - rear wheels locked straight via axis1 LoStop = HiStop = 0          *
 *   - drive = motors on axis2 (dParamVel2 / dParamFMax2) on ALL wheels   *
 *     (AWD) so it can climb; steering = P-servo on the front wheel axis1 *
 *   - tire contact: mu = dInfinity + bounded slip + soft contact         *
 *   - the whole car lives in its own sub-space so it never self-collides *
 *                                                                        *
 * Build modes (see Makefile):                                            *
 *   - default        : DrawStuff/GLUT interactive window                 *
 *   - -DHEADLESS      : no renderer; steps the physics and prints a       *
 *                       trajectory trace + PASS/FAIL self-checks. Used to *
 *                       verify the dynamics without a display.           *
 *                                                                        *
 * Precision: this links Homebrew's ODE 0.16.x, which is built dDOUBLE, so *
 * dReal == double. We never #define dSINGLE/dDOUBLE ourselves; <ode/ode.h>*
 * pulls in the library's own precision.h (the rule from the ODE skill).  *
 *************************************************************************/

#include <ode/ode.h>
#include <cstdio>
#include <cstdlib>
#include <cmath>

#ifndef HEADLESS
#include <drawstuff/drawstuff.h>
/* DrawStuff has float- and double-typed draw entry points; under dDOUBLE the
 * dBody/dGeom getters return double[], so map the names to the D variants. */
#ifdef dDOUBLE
#define dsDrawBox       dsDrawBoxD
#define dsDrawSphere    dsDrawSphereD
#define dsDrawCylinder  dsDrawCylinderD
#define dsDrawCapsule   dsDrawCapsuleD
#endif
#endif /* !HEADLESS */

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* ======================================================================= */
/*  Tunable constants                                                      */
/* ======================================================================= */

/* --- buggy geometry / mass (metres, kilograms) --- */
static const dReal LENGTH = 0.7;    /* chassis length (x) */
static const dReal WIDTH  = 0.5;    /* chassis width  (y) */
static const dReal HEIGHT = 0.2;    /* chassis height (z) */
static const dReal RADIUS = 0.18;   /* wheel radius       */
static const dReal STARTZ = 0.42;   /* chassis start height (small settle) */
static const dReal CMASS  = 2.0;    /* chassis mass       */
static const dReal WMASS  = 0.4;    /* per-wheel mass     */
static const dReal STARTX = -1.2;   /* chassis start x (on the flat, before ramp) */

/* --- world / solver --- */
static const dReal GRAVITY   = -9.81;
static const dReal STEP_SIZE = 0.005;  /* fixed dt (200 Hz) */
static const int   SUBSTEPS  = 4;      /* physics steps per rendered frame  */

/* --- suspension (hinge2 axis1 spring) --- */
static const dReal SUSP_ERP = 0.6;
static const dReal SUSP_CFM = 0.0018;

/* --- drive / steering --- */
static const dReal CRUISE_SPEED = 11.0; /* target wheel-spin command (rad/s-ish) */
static const dReal DRIVE_FMAX   = 8.0;  /* per-wheel drive torque cap (N*m)       */
static const dReal STEER_FMAX   = 2.0;  /* steering servo torque cap              */
static const dReal STEER_LIMIT  = 0.75; /* max steer angle (rad)                  */

/* --- tire contact surface --- */
static const dReal TIRE_SLIP     = 0.10;
static const dReal TIRE_SOFT_ERP = 0.8;
static const dReal TIRE_SOFT_CFM = 0.004;

/* --- ramp: a symmetric HILL made of two inclined slabs whose top faces meet
 * exactly at a ridge, so the buggy drives UP one slope, over the ridge, and
 * DOWN the far slope (landing on its wheels) rather than off a cliff edge. --- */
static const dReal RAMP_ANGLE = 0.26;  /* incline magnitude (rad ~ 15 deg)    */
static const dReal RAMP_LEN   = 2.6;   /* each slab full length (x, pre-rot)  */
static const dReal RAMP_WID   = 3.0;   /* slab full width  (y)                */
static const dReal RAMP_THK   = 0.5;   /* slab full thickness (z)             */
static const dReal RIDGE_X    = 4.0;   /* world x of the hilltop / ridge      */

/* filled in by setupSim(): world x where the up-slope leaves the ground, and
 * the ridge height -- used by the headless harness for its checks. */
static dReal g_ramp_start_x = 0;
static dReal g_ridge_z      = 0;

static const dVector3 yunit = { 0, 1, 0 };  /* hinge2 axis2 (rolling)  */
static const dVector3 zunit = { 0, 0, 1 };  /* hinge2 axis1 (steering) */

/* ======================================================================= */
/*  Dynamics + collision objects                                           */
/* ======================================================================= */

static dWorldID      world;
static dSpaceID      space;        /* top-level collision space             */
static dSpaceID      car_space;    /* sub-space for the car (no self-collide)*/
static dBodyID       body[4];      /* 0 = chassis, 1 = front, 2/3 = rear     */
static dJointID      joint[3];     /* one hinge2 per wheel; joint[0] = front */
static dJointGroupID contactgroup;
static dGeomID       ground;       /* infinite ground plane                  */
static dGeomID       ramp_up;      /* up-slope slab (rises toward +x)        */
static dGeomID       ramp_down;    /* down-slope slab (descends past ridge)  */
static dGeomID       chassis_geom;
static dGeomID       wheel_geom[3];

/* user / autopilot commands */
static dReal speed = 0;            /* >0 drives the buggy toward +x          */
static dReal steer = 0;            /* target steering angle (rad)            */

/* ======================================================================= */
/*  Collision near-callback                                                */
/* ======================================================================= */
/* We only want car-vs-environment contacts. The car geoms all have bodies;
 * the environment geoms (plane, ramp) are static (body == 0). So "exactly one
 * of the pair is static" selects precisely car-vs-environment and rejects both
 * car-vs-car (handled anyway by the sub-space) and env-vs-env. The dGeomIsSpace
 * guard makes this correct even if dSpaceCollide hands us the sub-space. */

static inline int isEnvironment(dGeomID g) { return dGeomGetBody(g) == 0; }

static void nearCallback(void *data, dGeomID o1, dGeomID o2)
{
  if (dGeomIsSpace(o1) || dGeomIsSpace(o2)) {
    dSpaceCollide2(o1, o2, data, &nearCallback);
    return;                 /* (we never collide car_space with itself) */
  }

  int e1 = isEnvironment(o1);
  int e2 = isEnvironment(o2);
  if (!(e1 ^ e2)) return;   /* skip unless exactly one side is environment */

  const int N = 16;
  dContact contact[N];
  int n = dCollide(o1, o2, N, &contact[0].geom, sizeof(dContact));
  for (int i = 0; i < n; i++) {
    contact[i].surface.mode = dContactSlip1 | dContactSlip2 |
                              dContactSoftERP | dContactSoftCFM | dContactApprox1;
    contact[i].surface.mu       = dInfinity;
    contact[i].surface.slip1    = TIRE_SLIP;
    contact[i].surface.slip2    = TIRE_SLIP;
    contact[i].surface.soft_erp = TIRE_SOFT_ERP;
    contact[i].surface.soft_cfm = TIRE_SOFT_CFM;
    dJointID c = dJointCreateContact(world, contactgroup, &contact[i]);
    dJointAttach(c,
                 dGeomGetBody(contact[i].geom.g1),
                 dGeomGetBody(contact[i].geom.g2));
  }
}

/* ======================================================================= */
/*  Controls + one physics step (shared by both build modes)               */
/* ======================================================================= */

static void applyControls(void)
{
  /* AWD: drive every wheel on its rolling axis (axis2). A hinge2 axis2 motor
   * only produces force when dParamFMax2 > 0. */
  for (int i = 0; i < 3; i++) {
    dJointSetHinge2Param(joint[i], dParamVel2,  -speed);
    dJointSetHinge2Param(joint[i], dParamFMax2,  DRIVE_FMAX);
  }

  /* Steering: proportional servo on the FRONT wheel's axis1 only. */
  dReal v = steer - dJointGetHinge2Angle1(joint[0]);
  if (v >  0.1) v =  0.1;
  if (v < -0.1) v = -0.1;
  v *= 10.0;
  dJointSetHinge2Param(joint[0], dParamVel,  v);
  dJointSetHinge2Param(joint[0], dParamFMax, STEER_FMAX);
  dJointSetHinge2Param(joint[0], dParamLoStop, -STEER_LIMIT);
  dJointSetHinge2Param(joint[0], dParamHiStop,  STEER_LIMIT);
  dJointSetHinge2Param(joint[0], dParamFudgeFactor, 0.1);
}

static void stepPhysics(void)
{
  applyControls();
  dSpaceCollide(space, 0, &nearCallback);   /* find pairs -> build contacts */
  dWorldStep(world, STEP_SIZE);             /* integrate one fixed timestep */
  dJointGroupEmpty(contactgroup);           /* drop this step's contacts    */
}

/* ======================================================================= */
/*  World construction                                                     */
/* ======================================================================= */

/* Position+orient an inclined slab (rotation about +y by `angle`) so that the
 * HIGH edge of its top face sits exactly at world (ridgeX, 0, ridgeZ). Under a
 * +y rotation a top-face point (x_local, 0, +rh) maps to world z =
 * -x_local*sin(angle) + rh*cos(angle); the high edge is the +rl end when the
 * slope rises toward +x (angle<0) and the -rl end when it rises toward -x. */
static void placeRampSlab(dGeomID g, dReal angle, dReal ridgeX, dReal ridgeZ)
{
  const dReal rl = RAMP_LEN * 0.5;
  const dReal rh = RAMP_THK * 0.5;
  const dReal ca = std::cos(angle);
  const dReal sa = std::sin(angle);
  const dReal xe = (angle > 0) ? -rl : rl;          /* high-edge local x */
  const dReal offx =  xe * ca + rh * sa;
  const dReal offz = -xe * sa + rh * ca;
  dMatrix3 R;
  dRFromAxisAndAngle(R, 0, 1, 0, angle);
  dGeomSetPosition(g, ridgeX - offx, 0, ridgeZ - offz);
  dGeomSetRotation(g, R);
}

static void setupSim(void)
{
  world = dWorldCreate();
  space = dHashSpaceCreate(0);
  contactgroup = dJointGroupCreate(0);
  dWorldSetGravity(world, 0, 0, GRAVITY);

  /* improve resting stability a little */
  dWorldSetERP(world, 0.4);
  dWorldSetCFM(world, 1e-5);

  ground = dCreatePlane(space, 0, 0, 1, 0);

  /* --- chassis --- */
  dMass m;
  body[0] = dBodyCreate(world);
  dBodySetPosition(body[0], STARTX, 0, STARTZ);
  dMassSetBox(&m, 1, LENGTH, WIDTH, HEIGHT);
  dMassAdjust(&m, CMASS);
  dBodySetMass(body[0], &m);
  chassis_geom = dCreateBox(0, LENGTH, WIDTH, HEIGHT);   /* space 0: link later */
  dGeomSetBody(chassis_geom, body[0]);

  /* --- wheels (sphere colliders, drawn as cylinders) --- */
  for (int i = 1; i <= 3; i++) {
    body[i] = dBodyCreate(world);
    dQuaternion q;
    dQFromAxisAndAngle(q, 1, 0, 0, M_PI * 0.5);  /* lay the render cylinder flat */
    dBodySetQuaternion(body[i], q);
    dMassSetSphere(&m, 1, RADIUS);
    dMassAdjust(&m, WMASS);
    dBodySetMass(body[i], &m);
    wheel_geom[i - 1] = dCreateSphere(0, RADIUS);
    dGeomSetBody(wheel_geom[i - 1], body[i]);
  }
  dBodySetPosition(body[1], STARTX + 0.5 * LENGTH, 0,            STARTZ - HEIGHT * 0.5);
  dBodySetPosition(body[2], STARTX - 0.5 * LENGTH,  WIDTH * 0.5, STARTZ - HEIGHT * 0.5);
  dBodySetPosition(body[3], STARTX - 0.5 * LENGTH, -WIDTH * 0.5, STARTZ - HEIGHT * 0.5);

  /* --- hinge2 joints: chassis first, wheel second --- */
  for (int i = 0; i < 3; i++) {
    joint[i] = dJointCreateHinge2(world, 0);
    dJointAttach(joint[i], body[0], body[i + 1]);
    const dReal *a = dBodyGetPosition(body[i + 1]);
    dJointSetHinge2Anchor(joint[i], a[0], a[1], a[2]);
    dJointSetHinge2Axes(joint[i], zunit, yunit);   /* axis1=steer, axis2=roll */
    dJointSetHinge2Param(joint[i], dParamSuspensionERP, SUSP_ERP);
    dJointSetHinge2Param(joint[i], dParamSuspensionCFM, SUSP_CFM);
  }

  /* lock the two rear wheels straight (axis1 stops pinned to 0) */
  for (int i = 1; i < 3; i++) {
    dJointSetHinge2Param(joint[i], dParamLoStop, 0);
    dJointSetHinge2Param(joint[i], dParamHiStop, 0);
  }

  /* --- car sub-space (prevents self-collision) --- */
  car_space = dSimpleSpaceCreate(space);
  dSpaceSetCleanup(car_space, 0);
  dSpaceAdd(car_space, chassis_geom);
  dSpaceAdd(car_space, wheel_geom[0]);
  dSpaceAdd(car_space, wheel_geom[1]);
  dSpaceAdd(car_space, wheel_geom[2]);

  /* --- the ramp: a symmetric hill of two inclined slabs ---
   * Each slab's TOP face is an inclined plane; we place both so the HIGH edge
   * of each top face lands exactly on the ridge point (RIDGE_X, 0, z_ridge).
   * With z_ridge = 2*rl*sin(A), each slope runs cleanly from ground level up to
   * the ridge and everything below the top face is buried -- so the wheels meet
   * a continuous surface up one side, over the ridge, and down the other. */
  const dReal A  = RAMP_ANGLE;
  const dReal rl = RAMP_LEN * 0.5;
  const dReal z_ridge = 2.0 * rl * std::sin(A);
  g_ridge_z      = z_ridge;
  g_ramp_start_x = RIDGE_X - 2.0 * rl * std::cos(A);   /* foot of the up-slope */

  ramp_up   = dCreateBox(space, RAMP_LEN, RAMP_WID, RAMP_THK);
  ramp_down = dCreateBox(space, RAMP_LEN, RAMP_WID, RAMP_THK);
  placeRampSlab(ramp_up,   -A, RIDGE_X, z_ridge);      /* rises toward +x      */
  placeRampSlab(ramp_down, +A, RIDGE_X, z_ridge);      /* descends toward +x   */

  /* autopilot: start cruising forward toward the ramp */
  speed = CRUISE_SPEED;
  steer = 0;
}

static void teardownSim(void)
{
  /* these four geoms were created with space 0 + added to a no-cleanup
   * sub-space, so we destroy them by hand (the rest is freed with the space). */
  dGeomDestroy(chassis_geom);
  dGeomDestroy(wheel_geom[0]);
  dGeomDestroy(wheel_geom[1]);
  dGeomDestroy(wheel_geom[2]);
  dJointGroupDestroy(contactgroup);
  dSpaceDestroy(space);
  dWorldDestroy(world);
}

/* ======================================================================= */
/*  HEADLESS verification harness                                          */
/* ======================================================================= */
#ifdef HEADLESS

int main(void)
{
  dInitODE2(0);
  dAllocateODEDataForThread(dAllocateMaskAll);
  setupSim();

  const int   FRAMES        = 320;          /* * SUBSTEPS steps             */
  const dReal SIM_DT        = STEP_SIZE * SUBSTEPS;
  dReal start_x = STARTX, rest_z = STARTZ;
  dReal max_x = -1e9, max_z = -1e9, min_z = 1e9, max_abs = 0;
  int   exploded = 0, crossed_ramp = 0;

  printf("# t      x         y         z      | buggy-over-a-ramp headless trace\n");
  for (int f = 0; f < FRAMES; f++) {
    for (int s = 0; s < SUBSTEPS; s++) stepPhysics();

    const dReal *p = dBodyGetPosition(body[0]);
    dReal x = p[0], y = p[1], z = p[2];

    if (std::isnan(x) || std::isnan(y) || std::isnan(z) ||
        std::fabs(x) > 100 || std::fabs(y) > 100 || std::fabs(z) > 100)
      exploded = 1;

    if (x > max_x) max_x = x;
    if (z > max_z) max_z = z;
    if (z < min_z) min_z = z;
    dReal a = std::fabs(x); if (a > max_abs) max_abs = a;
    a = std::fabs(z); if (a > max_abs) max_abs = a;
    if (x > g_ramp_start_x) crossed_ramp = 1;

    if (f % 20 == 0 || f == FRAMES - 1)
      printf("%5.2f  %8.3f  %8.3f  %8.3f\n", f * SIM_DT, x, y, z);
    if (exploded) break;
  }

  /* let it settle for ~0.4 s up front: the resting wheel-z is ~RADIUS, the
   * chassis rest z is around STARTZ; "climbed" means the chassis rose clearly
   * above its starting height as it went up the ramp. */
  dReal climbed = max_z - rest_z;

  printf("\n--- summary ---\n");
  printf("start x          : %8.3f\n", start_x);
  printf("max x reached    : %8.3f   (ramp foot x=%.2f, ridge x=%.2f z=%.2f)\n",
         max_x, g_ramp_start_x, RIDGE_X, g_ridge_z);
  printf("chassis z range  : [%.3f .. %.3f]  (start %.3f)\n", min_z, max_z, rest_z);
  printf("climb above start: %8.3f m\n", climbed);
  printf("exploded/NaN     : %s\n", exploded ? "YES" : "no");

  int forward_ok = (max_x > start_x + 1.0);    /* clearly drove forward      */
  int reached_ok = crossed_ramp;               /* got onto/over the ramp      */
  int climb_ok   = (climbed > 0.15);           /* rose up the incline         */
  int stable_ok  = (!exploded && max_abs < 50); /* bounded, no blow-up        */

  printf("\nchecks:\n");
  printf("  [%s] drove forward   (max_x %.2f > %.2f)\n", forward_ok?"PASS":"FAIL", max_x, start_x + 1.0);
  printf("  [%s] reached ramp    (x crossed %.2f)\n",    reached_ok?"PASS":"FAIL", g_ramp_start_x);
  printf("  [%s] climbed ramp    (rose %.2f m > 0.15)\n",climb_ok?"PASS":"FAIL", climbed);
  printf("  [%s] stayed stable   (no NaN, bounded)\n",   stable_ok?"PASS":"FAIL");

  int ok = forward_ok && reached_ok && climb_ok && stable_ok;
  printf("\nRESULT: %s\n", ok ? "PASS" : "FAIL");

  teardownSim();
  dCloseODE();
  return ok ? 0 : 1;
}

/* ======================================================================= */
/*  GRAPHICAL build (DrawStuff / GLUT)                                     */
/* ======================================================================= */
#else  /* !HEADLESS */

static int follow_cam = 1;   /* camera trails the buggy */

static void start(void)
{
  /* initial viewpoint: behind/left of the start, looking toward +x & the ramp */
  static float xyz[3] = { -3.6f, -3.2f, 2.2f };
  static float hpr[3] = {  38.0f, -24.0f, 0.0f };
  dsSetViewpoint(xyz, hpr);
  printf("Buggy autopilot is ON - it drives toward the ramp by itself.\n"
         "Keys:\n"
         "  a / z : speed up / slow down\n"
         "  , / . : steer left / right\n"
         "  space : reset speed & steering (stop)\n"
         "  f     : toggle follow-camera\n"
         "  r     : reset the buggy to the start\n");
}

static void command(int cmd)
{
  switch (cmd) {
    case 'a': case 'A': speed += 1.0; break;
    case 'z': case 'Z': speed -= 1.0; break;
    case ',':           steer -= 0.25; break;
    case '.':           steer += 0.25; break;
    case ' ':           speed = 0; steer = 0; break;
    case 'f': case 'F': follow_cam ^= 1; break;
    case 'r': case 'R': {
      /* re-place the buggy at the start, at rest */
      dBodySetPosition(body[0], STARTX, 0, STARTZ);
      dMatrix3 I; dRSetIdentity(I);
      dBodySetRotation(body[0], I);
      dBodySetLinearVel(body[0], 0, 0, 0);
      dBodySetAngularVel(body[0], 0, 0, 0);
      dBodySetPosition(body[1], STARTX + 0.5 * LENGTH, 0,            STARTZ - HEIGHT * 0.5);
      dBodySetPosition(body[2], STARTX - 0.5 * LENGTH,  WIDTH * 0.5, STARTZ - HEIGHT * 0.5);
      dBodySetPosition(body[3], STARTX - 0.5 * LENGTH, -WIDTH * 0.5, STARTZ - HEIGHT * 0.5);
      for (int i = 1; i <= 3; i++) {
        dQuaternion q; dQFromAxisAndAngle(q, 1, 0, 0, M_PI * 0.5);
        dBodySetQuaternion(body[i], q);
        dBodySetLinearVel(body[i], 0, 0, 0);
        dBodySetAngularVel(body[i], 0, 0, 0);
      }
      speed = CRUISE_SPEED; steer = 0;
      break;
    }
  }
}

static void drawWheel(int i)
{
  dsDrawCylinder(dBodyGetPosition(body[i]), dBodyGetRotation(body[i]),
                 0.06f, RADIUS);
}

static void simLoop(int pause)
{
  if (!pause) {
    for (int s = 0; s < SUBSTEPS; s++) stepPhysics();
  }

  if (follow_cam) {
    const dReal *cp = dBodyGetPosition(body[0]);
    float xyz[3] = { (float)(cp[0] - 3.4), (float)(cp[1] - 3.0), (float)(cp[2] + 2.0) };
    float hpr[3] = { 40.0f, -26.0f, 0.0f };
    dsSetViewpoint(xyz, hpr);
  }

  /* chassis */
  dsSetColor(0.4f, 0.7f, 1.0f);
  dsSetTexture(DS_WOOD);
  dReal sides[3] = { LENGTH, WIDTH, HEIGHT };
  dsDrawBox(dBodyGetPosition(body[0]), dBodyGetRotation(body[0]), sides);

  /* wheels */
  dsSetColor(0.2f, 0.2f, 0.2f);
  for (int i = 1; i <= 3; i++) drawWheel(i);

  /* ramp (the two hill slabs) */
  dsSetColor(0.85f, 0.75f, 0.45f);
  dsSetTexture(DS_CHECKERED);
  dVector3 ss;
  dGeomBoxGetLengths(ramp_up, ss);
  dsDrawBox(dGeomGetPosition(ramp_up),   dGeomGetRotation(ramp_up),   ss);
  dGeomBoxGetLengths(ramp_down, ss);
  dsDrawBox(dGeomGetPosition(ramp_down), dGeomGetRotation(ramp_down), ss);
}

/* the ODE teardown, registered with atexit so it runs even though the GLUT
 * backend exit()s rather than returning from dsSimulationLoop. */
static void atExitCleanup(void)
{
  teardownSim();
  dCloseODE();
}

int main(int argc, char **argv)
{
  dInitODE2(0);
  dAllocateODEDataForThread(dAllocateMaskAll);
  setupSim();
  atexit(atExitCleanup);

  dsFunctions fn;
  fn.version = DS_VERSION;
  fn.start   = &start;
  fn.step    = &simLoop;
  fn.command = &command;
  fn.stop    = 0;
  fn.path_to_textures = DRAWSTUFF_TEXTURE_PATH;

  dsSimulationLoop(argc, argv, 800, 600, &fn);

  /* Only reached on backends whose loop returns; atExitCleanup covers the
   * GLUT-on-macOS exit() path. */
  return 0;
}

#endif /* HEADLESS */
