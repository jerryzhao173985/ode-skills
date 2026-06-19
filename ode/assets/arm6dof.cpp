// arm6dof.cpp — A 6-DOF serial robot arm (hinge joints + AMotors) reaching a
// Cartesian target, simulated with the Open Dynamics Engine and verified HEADLESS.
//
// Design (grounded in include/ode/*.h — the canonical API):
//   * 6 capsule LINKS form a serial chain. Consecutive links are pinned by a
//     dHinge (1-DOF kinematic constraint). The base link is pinned to the static
//     world (body 0). This yields a 6-DOF arm.
//   * Each hinge's own motor is left OFF (dParamFMax == 0). Actuation is supplied
//     by a parallel dAMotor in USER mode (1 axis), attached to the SAME two bodies
//     as the hinge. Every step we (a) realign the AMotor axis to the current world
//     hinge axis and (b) command an angular-velocity servo with a torque cap.
//   * "Reaching" is closed-loop Cartesian control: a 3x6 geometric Jacobian maps
//     joint rates to end-effector linear velocity; damped least squares (DLS)
//     converts a desired EE velocity (proportional to the position error) into
//     joint rates, which become the per-joint AMotor velocity commands.
//
// Verification is headless and self-grading: no window, prints periodic state and
// a final RESULT: PASS/FAIL with a process exit code (0 = PASS).
//
// Build (Homebrew ODE is double precision; define NO precision macro):
//   c++ -O2 -std=c++17 $(ode-config --cflags) src/arm6dof.cpp $(ode-config --libs) -o arm6dof
//
// Run:    ./arm6dof [tx ty tz] [trajectory.csv]

#include <ode/ode.h>
#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <cstring>

// ----------------------------- tunables -------------------------------------
static const int    N_LINKS    = 6;
static const double SEG_LEN    = 0.25;     // joint-to-joint spacing per link (m)
static const double LINK_R     = 0.035;    // capsule radius (m)
static const double LINK_CYL   = SEG_LEN - 2.0 * LINK_R; // capsule cylinder length (caps excluded)
static const double LINK_MASS  = 0.5;      // kg per link (masses kept within 1 order of magnitude)
static const double BASE_Z     = 0.50;     // height of the base hinge anchor (m)
static const double EE_LOCAL_Z = LINK_CYL * 0.5 + LINK_R; // tip offset along last link's local Z

static const double DT         = 0.005;    // FIXED timestep (s)  -> 200 Hz
static const int    STEPS      = 4000;     // 20 s simulated
static const int    PRINT_EVERY= 200;

static const double GRAVITY    = -9.81;
static const double ERP        = 0.2;      // default; range 0.1-0.8
static const double CFM        = 1e-5;     // positive (soft, moves off singularity)
static const double LIN_DAMP   = 0.01;     // world linear damping scale  [0,1]
static const double ANG_DAMP   = 0.05;     // world angular damping scale [0,1]

// controller gains
static const double FMAX       = 150.0;    // AMotor torque budget per joint (N*m)
static const double KP         = 3.0;      // Cartesian proportional gain (1/s)
static const double VMAX_CART  = 0.7;      // cap on commanded EE speed (m/s)
static const double QDOT_MAX   = 4.0;      // cap on commanded joint rate (rad/s)
// Adaptive damped-least-squares damping (Maciejewski-Klein): damping is engaged
// only near singularities (small manipulability w = sqrt(det(J J^T))) and fades
// to a tiny floor when the arm is well-conditioned, so accuracy is high in the
// dexterous workspace while motion stays bounded near singular poses.
static const double DLS_W0     = 0.05;     // manipulability threshold to start damping
static const double DLS_LAM2MAX= 0.02;     // max squared damping at full singularity
static const double DLS_LAM2MIN= 2e-4;     // numerical floor (always present)

// PASS thresholds
static const double REACH_TOL  = 0.02;     // final EE-to-target distance (m)
static const double SETTLE_SPD = 0.05;     // mean body speed over the tail (m/s)
static const double SEP_TOL    = 2e-3;     // hinge constraint separation when settled (m)
static const double GROUND_TOL = -0.05;    // lowest allowed body z (m)

// ----------------------------- globals --------------------------------------
static dWorldID      world;
static dSpaceID      space;
static dJointGroupID contactgroup;

static dBodyID  link_body[N_LINKS];
static dGeomID  link_geom[N_LINKS];
static dJointID hinge[N_LINKS];
static dJointID amotor[N_LINKS];

// Per-joint command-sign multiplier. ODE's AMotor velocity sign depends on the
// body ordering: attaching the proximal side to the static world (body 0) is the
// OPPOSITE effective sense from attaching two dynamic bodies. Verified by the two
// controlled experiments in src/probe_sign.cpp (world-attached: +1) and
// src/probe_sign2.cpp (both dynamic: -1). Maps our Jacobian's right-hand-rule
// joint rate (axis x r convention) onto the correct dParamVel sign.
static double g_amotor_sign[N_LINKS];

static double target[3] = {0.7, 0.3, 1.0};

// initial world-frame hinge axes (chain is straight along +X, so these are valid
// at setup time; the hinge captures its zero angle from the pose at axis-set time)
static const double AXIS0[N_LINKS][3] = {
    {0, 0, 1},   // J0 base yaw   (about world Z)
    {0, 1, 0},   // J1 pitch      (about world Y)
    {0, 1, 0},   // J2 pitch
    {1, 0, 0},   // J3 roll       (about link axis)
    {0, 1, 0},   // J4 pitch
    {1, 0, 0},   // J5 roll
};

// ----------------------------- helpers --------------------------------------
static inline double vnorm3(const double v[3]) {
    return std::sqrt(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);
}
static inline double norm3(double x, double y, double z) {
    return std::sqrt(x*x + y*y + z*z);
}

// collision callback: ground-vs-arm only. All arm links are dynamic (non-zero
// body); we skip every link-vs-link pair so the free-space reach is unaffected
// by self-contact, while still exercising the real contact pipeline vs the floor.
static void nearCallback(void * /*data*/, dGeomID o1, dGeomID o2) {
    dBodyID b1 = dGeomGetBody(o1);
    dBodyID b2 = dGeomGetBody(o2);
    if (b1 && b2) return;                       // both dynamic => arm-arm, skip
    const int MAXC = 8;
    dContact contact[MAXC];
    for (int i = 0; i < MAXC; i++) {
        contact[i].surface.mode = dContactApprox1 | dContactSoftCFM;
        contact[i].surface.mu   = 1.0;
        contact[i].surface.soft_cfm = 1e-4;
    }
    int n = dCollide(o1, o2, MAXC, &contact[0].geom, sizeof(dContact));
    for (int i = 0; i < n; i++) {
        dJointID c = dJointCreateContact(world, contactgroup, &contact[i]);
        dJointAttach(c, b1, b2);
    }
}

// world position of the end-effector tip (top of the last link)
static void endEffector(double ee[3]) {
    dVector3 p;
    dBodyGetRelPointPos(link_body[N_LINKS - 1], 0, 0, EE_LOCAL_Z, p);
    ee[0] = p[0]; ee[1] = p[1]; ee[2] = p[2];
}

// total mechanical energy (diagnostic only): KE (trans + rot) + PE.
static void energy(double *ke, double *pe) {
    double K = 0.0, P = 0.0;
    for (int i = 0; i < N_LINKS; i++) {
        dMass m; dBodyGetMass(link_body[i], &m);
        const dReal *v = dBodyGetLinearVel(link_body[i]);
        const dReal *w = dBodyGetAngularVel(link_body[i]);   // world frame
        const dReal *R = dBodyGetRotation(link_body[i]);     // body->world, R[r*4+c]
        const dReal *p = dBodyGetPosition(link_body[i]);
        K += 0.5 * m.mass * (v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);
        // rotate world omega into the body frame: wb = R^T w
        double wb[3] = {
            R[0]*w[0] + R[4]*w[1] + R[8]*w[2],
            R[1]*w[0] + R[5]*w[1] + R[9]*w[2],
            R[2]*w[0] + R[6]*w[1] + R[10]*w[2],
        };
        // rotational KE = 0.5 * wb^T I_body wb  (I in body frame, I[r*4+c])
        double Iw[3] = {
            m.I[0]*wb[0] + m.I[1]*wb[1] + m.I[2]*wb[2],
            m.I[4]*wb[0] + m.I[5]*wb[1] + m.I[6]*wb[2],
            m.I[8]*wb[0] + m.I[9]*wb[1] + m.I[10]*wb[2],
        };
        K += 0.5 * (wb[0]*Iw[0] + wb[1]*Iw[1] + wb[2]*Iw[2]);
        P += m.mass * (-GRAVITY) * p[2];
    }
    *ke = K; *pe = P;
}

// max distance between a hinge's two anchor read-backs (constraint stretch)
static double hingeSeparation(dJointID j) {
    dVector3 a1, a2;
    dJointGetHingeAnchor(j, a1);
    dJointGetHingeAnchor2(j, a2);
    double d[3] = {a1[0]-a2[0], a1[1]-a2[1], a1[2]-a2[2]};
    return vnorm3(d);
}

// ----------------------------- controller -----------------------------------
// Damped least squares Cartesian servo. Returns current EE-target distance and
// fills the per-joint AMotor velocity commands; also realigns each AMotor axis.
static bool g_debug = false;
static double controlStep(int s) {
    double ee[3];
    endEffector(ee);
    double e[3] = { target[0]-ee[0], target[1]-ee[1], target[2]-ee[2] };
    double dist = vnorm3(e);

    // desired EE velocity = KP * e, capped at VMAX_CART
    double vdes[3] = { KP*e[0], KP*e[1], KP*e[2] };
    double vmag = vnorm3(vdes);
    if (vmag > VMAX_CART && vmag > 1e-12) {
        double scale = VMAX_CART / vmag;
        vdes[0]*=scale; vdes[1]*=scale; vdes[2]*=scale;
    }

    // geometric Jacobian J (3x6): column i = axis_i x (ee - anchor_i)
    double J[3][N_LINKS];
    double axis[N_LINKS][3];
    for (int i = 0; i < N_LINKS; i++) {
        dVector3 ax, an;
        dJointGetHingeAxis(hinge[i], ax);
        dJointGetHingeAnchor(hinge[i], an);
        axis[i][0]=ax[0]; axis[i][1]=ax[1]; axis[i][2]=ax[2];
        double r[3] = { ee[0]-an[0], ee[1]-an[1], ee[2]-an[2] };
        J[0][i] = ax[1]*r[2] - ax[2]*r[1];
        J[1][i] = ax[2]*r[0] - ax[0]*r[2];
        J[2][i] = ax[0]*r[1] - ax[1]*r[0];
    }

    // B = J J^T   (3x3, symmetric positive semidefinite)
    double B[3][3];
    for (int a = 0; a < 3; a++)
        for (int b = 0; b < 3; b++) {
            double sum = 0.0;
            for (int i = 0; i < N_LINKS; i++) sum += J[a][i]*J[b][i];
            B[a][b] = sum;
        }
    // manipulability w = sqrt(det(J J^T)); engage damping only when w < W0
    double detB = B[0][0]*(B[1][1]*B[2][2] - B[1][2]*B[2][1])
                - B[0][1]*(B[1][0]*B[2][2] - B[1][2]*B[2][0])
                + B[0][2]*(B[1][0]*B[2][1] - B[1][1]*B[2][0]);
    double w = std::sqrt(detB > 0 ? detB : 0.0);
    double lam2 = DLS_LAM2MIN;
    if (w < DLS_W0) {
        double t = 1.0 - (w / DLS_W0);          // 0 at threshold -> 1 at singularity
        lam2 = DLS_LAM2MIN + DLS_LAM2MAX * t * t;
    }
    // A = B + lambda^2 I   (symmetric positive definite)
    double A[3][3];
    for (int a = 0; a < 3; a++)
        for (int b = 0; b < 3; b++)
            A[a][b] = B[a][b] + (a==b ? lam2 : 0.0);

    // invert 3x3 via adjugate / determinant
    double c00 =  (A[1][1]*A[2][2] - A[1][2]*A[2][1]);
    double c01 = -(A[1][0]*A[2][2] - A[1][2]*A[2][0]);
    double c02 =  (A[1][0]*A[2][1] - A[1][1]*A[2][0]);
    double det =  A[0][0]*c00 + A[0][1]*c01 + A[0][2]*c02;
    if (std::fabs(det) < 1e-18) det = (det < 0 ? -1e-18 : 1e-18);
    double inv = 1.0 / det;
    double Ai[3][3];
    Ai[0][0] = c00*inv;
    Ai[0][1] = (A[0][2]*A[2][1] - A[0][1]*A[2][2])*inv;
    Ai[0][2] = (A[0][1]*A[1][2] - A[0][2]*A[1][1])*inv;
    Ai[1][0] = c01*inv;
    Ai[1][1] = (A[0][0]*A[2][2] - A[0][2]*A[2][0])*inv;
    Ai[1][2] = (A[0][2]*A[1][0] - A[0][0]*A[1][2])*inv;
    Ai[2][0] = c02*inv;
    Ai[2][1] = (A[0][1]*A[2][0] - A[0][0]*A[2][1])*inv;
    Ai[2][2] = (A[0][0]*A[1][1] - A[0][1]*A[1][0])*inv;

    // y = A^-1 vdes ; dq = J^T y
    double y[3];
    for (int a = 0; a < 3; a++)
        y[a] = Ai[a][0]*vdes[0] + Ai[a][1]*vdes[1] + Ai[a][2]*vdes[2];

    double dqv[N_LINKS];
    for (int i = 0; i < N_LINKS; i++) {
        double dq = J[0][i]*y[0] + J[1][i]*y[1] + J[2][i]*y[2];
        if (dq >  QDOT_MAX) dq =  QDOT_MAX;
        if (dq < -QDOT_MAX) dq = -QDOT_MAX;
        dqv[i] = dq;
        // realign the AMotor's single user axis to the live world hinge axis and
        // command the velocity servo with its torque budget.
        dJointSetAMotorAxis(amotor[i], 0, 0, axis[i][0], axis[i][1], axis[i][2]);
        dJointSetAMotorParam(amotor[i], dParamVel,  g_amotor_sign[i] * dq);
        dJointSetAMotorParam(amotor[i], dParamFMax, FMAX);
    }
    if (g_debug && (s < 5 || s % 200 == 0)) {
        printf("[dbg s=%4d] |e|=%.4f e=(%+.3f,%+.3f,%+.3f) vdes=(%+.3f,%+.3f,%+.3f) "
               "dq=[%+.2f %+.2f %+.2f %+.2f %+.2f %+.2f]\n",
               s, dist, e[0],e[1],e[2], vdes[0],vdes[1],vdes[2],
               dqv[0],dqv[1],dqv[2],dqv[3],dqv[4],dqv[5]);
        for (int i = 0; i < N_LINKS; i++) {
            double ang = dJointGetHingeAngle(hinge[i]);
            double rate = dJointGetHingeAngleRate(hinge[i]);
            printf("           J%d ang=%+.3f rate=%+.3f axis=(%+.2f,%+.2f,%+.2f)\n",
                   i, ang, rate, axis[i][0], axis[i][1], axis[i][2]);
        }
    }
    return dist;
}

// ----------------------------- setup ----------------------------------------
static void setup() {
    world = dWorldCreate();
    space = dHashSpaceCreate(0);
    contactgroup = dJointGroupCreate(0);

    // world defaults BEFORE creating bodies (they seed per-body defaults)
    dWorldSetGravity(world, 0, 0, GRAVITY);
    dWorldSetERP(world, ERP);
    dWorldSetCFM(world, CFM);
    dWorldSetLinearDamping(world, LIN_DAMP);
    dWorldSetAngularDamping(world, ANG_DAMP);
    dWorldSetAutoDisableFlag(world, 0);          // actuated arm must never sleep
    dWorldSetContactSurfaceLayer(world, 0.001);  // canonical anti-jitter for resting contacts
    dWorldSetContactMaxCorrectingVel(world, 5.0);// cap contact push-out to avoid popping

    dCreatePlane(space, 0, 0, 1, 0);             // static ground (geom, no body)

    // links laid straight along +X at height BASE_Z. Capsule long axis is local
    // Z, so rotate each link by +90 deg about Y to point local Z -> world +X.
    dMatrix3 R;
    dRFromAxisAndAngle(R, 0, 1, 0, M_PI * 0.5);

    for (int i = 0; i < N_LINKS; i++) {
        link_body[i] = dBodyCreate(world);
        dMass m;
        dMassSetZero(&m);
        dMassSetCapsuleTotal(&m, LINK_MASS, 3, LINK_R, LINK_CYL); // dir 3 = local Z
        dBodySetMass(link_body[i], &m);

        double cx = i * SEG_LEN + SEG_LEN * 0.5;  // link center along +X
        dBodySetPosition(link_body[i], cx, 0.0, BASE_Z);
        dBodySetRotation(link_body[i], R);

        link_geom[i] = dCreateCapsule(space, LINK_R, LINK_CYL);
        dGeomSetBody(link_geom[i], link_body[i]);
    }

    // hinges: attach AFTER positioning. base link pinned to world (body 0).
    for (int i = 0; i < N_LINKS; i++) {
        hinge[i] = dJointCreateHinge(world, 0);
        dBodyID b1 = (i == 0) ? 0 : link_body[i-1];
        // command-sign multiplier depends on whether the proximal side is world 0
        g_amotor_sign[i] = (b1 == 0) ? 1.0 : -1.0;
        dJointAttach(hinge[i], b1, link_body[i]);
        dJointSetHingeAnchor(hinge[i], i * SEG_LEN, 0.0, BASE_Z);   // world coords
        dJointSetHingeAxis(hinge[i], AXIS0[i][0], AXIS0[i][1], AXIS0[i][2]);
        // wide limit stops (set AFTER axis). Order hi,lo,hi for reliability.
        dJointSetHingeParam(hinge[i], dParamHiStop,  2.7);
        dJointSetHingeParam(hinge[i], dParamLoStop, -2.7);
        dJointSetHingeParam(hinge[i], dParamHiStop,  2.7);
        // hinge's own motor stays OFF; the parallel AMotor is the sole actuator.
        dJointSetHingeParam(hinge[i], dParamFMax, 0.0);

        // parallel 1-axis USER AMotor on the SAME two bodies.
        amotor[i] = dJointCreateAMotor(world, 0);
        dJointAttach(amotor[i], b1, link_body[i]);
        dJointSetAMotorMode(amotor[i], dAMotorUser);
        dJointSetAMotorNumAxes(amotor[i], 1);
        dJointSetAMotorAxis(amotor[i], 0, 0, AXIS0[i][0], AXIS0[i][1], AXIS0[i][2]);
        dJointSetAMotorParam(amotor[i], dParamFMax, FMAX);
        dJointSetAMotorParam(amotor[i], dParamVel, 0.0);
    }
}

static void teardown() {
    dJointGroupDestroy(contactgroup);
    dSpaceDestroy(space);
    dWorldDestroy(world);
}

// ----------------------------- main / headless self-check -------------------
int main(int argc, char **argv) {
    if (argc >= 4) {
        target[0] = atof(argv[1]);
        target[1] = atof(argv[2]);
        target[2] = atof(argv[3]);
    }
    const char *csvpath = (argc >= 5) ? argv[4] : nullptr;
    if (getenv("ARM_DEBUG")) g_debug = true;

    if (!std::isfinite(target[0]) || !std::isfinite(target[1]) || !std::isfinite(target[2])) {
        fprintf(stderr, "invalid (non-finite) target\n");
        printf("RESULT: FAIL (non-finite target)\n");
        return 1;
    }

    dInitODE2(0);
    dAllocateODEDataForThread(dAllocateMaskAll);

    if (!dCheckConfiguration("ODE_double_precision")) {
        fprintf(stderr, "FATAL: ODE is not double precision (got: %s)\n",
                dGetConfiguration());
        dCloseODE();
        return 2;
    }

    setup();

    FILE *csv = nullptr;
    if (csvpath) {
        csv = fopen(csvpath, "w");
        if (csv) fprintf(csv, "step,t,ee_x,ee_y,ee_z,dist,ke,pe\n");
        else fprintf(stderr, "warning: could not open CSV '%s' for writing\n", csvpath);
    }

    double ee0[3]; endEffector(ee0);
    double d0[3] = { target[0]-ee0[0], target[1]-ee0[1], target[2]-ee0[2] };
    printf("ODE config     : %s\n", dGetConfiguration());
    printf("links=%d  dt=%.4f  steps=%d  reach=%.2fm  actuator=AMotor(user,1-axis)\n",
           N_LINKS, DT, STEPS, N_LINKS * SEG_LEN);
    printf("target         : (%.3f, %.3f, %.3f)\n", target[0], target[1], target[2]);
    printf("ee start       : (%.3f, %.3f, %.3f)  start dist = %.4f m\n",
           ee0[0], ee0[1], ee0[2], vnorm3(d0));
    printf("%-6s %-7s %-9s %-9s %-9s %-9s %-9s\n",
           "step", "t", "dist", "KE", "PE", "vmax", "wmax");

    bool   nan_seen   = false;
    double min_z      = 1e9;
    double max_v      = 0.0, max_w = 0.0;
    double max_sep    = 0.0;
    double min_dist   = 1e9;
    double last_dist  = vnorm3(d0);

    const int TAIL = 200;          // last TAIL steps used for settle checks
    double tail_speed_sum = 0.0;   int tail_n = 0;
    double tail_sep_max   = 0.0;

    for (int s = 0; s < STEPS; s++) {
        last_dist = controlStep(s);             // sets AMotor commands
        if (last_dist < min_dist) min_dist = last_dist;

        dSpaceCollide(space, 0, &nearCallback);
        if (!dWorldStep(world, DT)) {
            printf("RESULT: FAIL (dWorldStep allocation failure at step %d)\n", s);
            if (csv) fclose(csv);
            teardown(); dCloseODE();
            return 1;
        }
        dJointGroupEmpty(contactgroup);

        // per-step instrumentation
        double step_v = 0.0;
        for (int i = 0; i < N_LINKS; i++) {
            const dReal *p = dBodyGetPosition(link_body[i]);
            if (!std::isfinite(p[0]) || !std::isfinite(p[1]) || !std::isfinite(p[2]))
                nan_seen = true;
            if (p[2] < min_z) min_z = p[2];
            const dReal *v = dBodyGetLinearVel(link_body[i]);
            const dReal *w = dBodyGetAngularVel(link_body[i]);
            double vm = norm3(v[0],v[1],v[2]);
            double wm = norm3(w[0],w[1],w[2]);
            if (!std::isfinite(vm) || !std::isfinite(wm)) nan_seen = true;
            if (vm > max_v) max_v = vm;
            if (wm > max_w) max_w = wm;
            step_v += vm;
        }
        step_v /= N_LINKS;

        for (int i = 0; i < N_LINKS; i++) {
            double sep = hingeSeparation(hinge[i]);
            if (sep > max_sep) max_sep = sep;
        }

        if (s >= STEPS - TAIL) {
            tail_speed_sum += step_v; tail_n++;
            for (int i = 0; i < N_LINKS; i++) {
                double sep = hingeSeparation(hinge[i]);
                if (sep > tail_sep_max) tail_sep_max = sep;
            }
        }

        if (csv) {
            double ke, pe; energy(&ke, &pe);
            double ee[3]; endEffector(ee);
            fprintf(csv, "%d,%.4f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f\n",
                    s, s*DT, ee[0], ee[1], ee[2], last_dist, ke, pe);
        }

        if (s % PRINT_EVERY == 0 || s == STEPS - 1) {
            double ke, pe; energy(&ke, &pe);
            double sv = 0.0, sw = 0.0;
            for (int i = 0; i < N_LINKS; i++) {
                const dReal *v = dBodyGetLinearVel(link_body[i]);
                const dReal *w = dBodyGetAngularVel(link_body[i]);
                double vm = norm3(v[0],v[1],v[2]);
                double wm = norm3(w[0],w[1],w[2]);
                if (vm > sv) sv = vm;
                if (wm > sw) sw = wm;
            }
            printf("%-6d %-7.3f %-9.5f %-9.4f %-9.4f %-9.4f %-9.4f\n",
                   s, s*DT, last_dist, ke, pe, sv, sw);
        }
        if (nan_seen) {
            printf("RESULT: FAIL (NaN in body position at step %d)\n", s);
            if (csv) fclose(csv);
            teardown(); dCloseODE();
            return 1;
        }
    }
    if (csv) fclose(csv);

    double tail_speed = (tail_n > 0) ? tail_speed_sum / tail_n : 1e9;

    // grade the TRUE post-integration end-effector position (controlStep's
    // last_dist was measured before this step's dWorldStep — one step stale).
    double ee[3]; endEffector(ee);
    double efin[3] = { target[0]-ee[0], target[1]-ee[1], target[2]-ee[2] };
    double final_dist = vnorm3(efin);
    printf("\n--- self-check ---------------------------------------------\n");
    printf("final ee       : (%.4f, %.4f, %.4f)\n", ee[0], ee[1], ee[2]);
    printf("final dist     : %.5f m   (tol %.3f)\n", final_dist, REACH_TOL);
    printf("min dist seen  : %.5f m\n", min_dist);
    printf("tail mean speed: %.5f m/s (tol %.3f)\n", tail_speed, SETTLE_SPD);
    printf("tail joint sep : %.2e m   (tol %.1e)\n", tail_sep_max, SEP_TOL);
    printf("max joint sep  : %.2e m   (whole run)\n", max_sep);
    printf("min body z     : %.4f m   (tol %.3f)\n", min_z, GROUND_TOL);
    printf("max |v|,|w|    : %.3f m/s, %.3f rad/s\n", max_v, max_w);

    bool reached  = final_dist < REACH_TOL;
    bool settled  = tail_speed < SETTLE_SPD;
    bool joints   = tail_sep_max < SEP_TOL;
    bool grounded = min_z > GROUND_TOL;
    bool finite   = !nan_seen && std::isfinite(final_dist);
    bool ok = reached && settled && joints && grounded && finite;

    printf("checks         : reached=%d settled=%d joints=%d no_tunnel=%d finite=%d\n",
           reached, settled, joints, grounded, finite);
    printf("RESULT: %s\n", ok ? "PASS" : "FAIL");

    teardown();
    dCloseODE();
    return ok ? 0 : 1;
}
