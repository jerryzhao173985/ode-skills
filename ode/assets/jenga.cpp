// jenga.cpp — A Jenga tower of boxes, knocked over by a thrown sphere, in ODE.
//
// This is a *headless* rigid-body simulation built on the Open Dynamics Engine
// (ODE). It demonstrates the full ODE pipeline end to end:
//
//   1. Build a classic Jenga tower: 18 layers x 3 blocks, alternating 90 deg.
//   2. Let the tower settle under gravity and verify it actually stands (and
//      record each block's settled pose as the baseline for "did it move?").
//   3. Throw a heavy "wrecking ball" sphere into it and watch the tower topple.
//   4. Run until everything comes to rest again.
//
// Throughout, it prints per-step diagnostics (contact count, kinetic energy,
// potential energy, total mechanical energy, peak speed, awake-body count,
// tower height) and at the end runs a battery of self-checks that confirm the
// simulation behaved physically:
//   - no value went NaN (during settle AND during the run),
//   - every world step actually advanced (dWorldQuickStep did not fail),
//   - the system reached *sustained* rest (not a one-frame lull),
//   - the tower stood (height within a band of as-built, and at rest),
//   - the tower toppled after impact (top height collapsed),
//   - a majority of blocks were *displaced from their settled pose*,
//   - no spurious per-step energy injection, and net energy was dissipated.
//
// Build (double precision is ODE's Homebrew default; define no precision macro):
//   c++ -O2 -std=c++17 $(ode-config --cflags) jenga.cpp $(ode-config --libs) -o jenga
// Run:
//   ./jenga              # exit 0 == all self-checks passed; 1 == a check failed;
//                        # 2 == precision mismatch; 3 == NaN/step failure mid-run
//
// Every ODE symbol used here is declared in <ode/ode.h>; nothing is invented.

#include <ode/ode.h>

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <vector>

// Compile-time precision guard: catch a header/TU built with the wrong precision
// macro. The runtime dCheckConfiguration below catches a wrong *library*; this
// catches a wrong *translation unit* (dReal would be float => broken ABI).
static_assert(sizeof(dReal) == sizeof(double),
              "dReal must be double — this TU was compiled with the wrong precision macro");

// ----------------------------------------------------------------------------
// Tunable "tokens" — the scalars that decide how the world feels and whether it
// is numerically stable. Kept together so they read like a config block.
// ----------------------------------------------------------------------------
static const double GRAVITY    = 9.81;    // m/s^2, applied as (0,0,-GRAVITY)
static const double STEP       = 0.005;   // fixed timestep (s); small == stable
static const int    QS_ITERS   = 80;      // QuickStep SOR iterations (default 20)

// Jenga geometry. Classic block ratio is ~1:3 (width:length) so 3 blocks make a
// square layer. We scale to ODE's happy range (lengths ~0.1..1.0).
static const int    LAYERS     = 18;      // classic Jenga is 18 layers
static const int    PER_LAYER  = 3;
static const double BLK_LEN    = 0.75;    // long axis
static const double BLK_WID    = 0.25;    // BLK_LEN / 3 -> a square footprint
static const double BLK_HGT    = 0.15;    // block thickness
static const double BLK_DENS   = 5.0;     // density -> per-block mass ~0.14
static const double GAP        = 0.0008;  // tiny vertical gap; blocks settle onto each other

// The projectile — a "wrecking ball": big and heavy enough to engage several
// layers at once and shove the lower-middle of the tower past its tipping point,
// rather than a small ball that just punches one block out Jenga-style.
static const double BALL_R     = 0.50;
static const double BALL_MASS  = 5.0;     // ~35x a block (~0.14) -> a real wrecking ball
static const double BALL_SPEED = 12.0;    // m/s toward the tower (+x)
static const double BALL_X0    = -2.0;    // launch standoff on the -x side
static const double BALL_Z_FRAC= 0.22;    // impact height as a fraction of tower height

// Contact surface tuning.
static const double MU         = 0.9;     // wood-ish friction, high enough to hold a stack
static const double SOFT_CFM   = 0.003;   // compliant resting contacts (stable stacking)
static const double SOFT_ERP   = 0.7;
static const double RHO        = 0.05;    // rolling/spinning friction so the ball stops

// Settling / self-check thresholds.
static const double KE_SETTLE  = 0.02;    // total KE considered "at rest"
static const double V_SETTLE   = 0.08;    // peak speed considered "at rest"
static const int    QUIET_STREAK = 30;    // consecutive quiet steps required to call it settled
static const double T_SETTLE_MAX = 4.0;   // cap for the pre-throw settle phase (s)
static const double T_RUN_MAX    = 30.0;  // cap for the post-throw phase (s)
static const int    MAX_CONTACTS = 16;    // contact points requested per geom pair
static const double DISPLACE_THRESH = 0.30;   // a block that moved this far from its settled pose "collapsed"
static const double ENERGY_RISE_TOL = 2.0;    // max tolerated single-step rise in total mechanical energy (J)

// ----------------------------------------------------------------------------
// Globals the near-callback / step helper need (dSpaceCollide takes a C function
// pointer, so the callback cannot capture state — it reaches these at file scope).
// ----------------------------------------------------------------------------
static dWorldID      world;
static dSpaceID      space;
static dJointGroupID contactgroup;
static long          g_contactsThisStep = 0;   // reset each step, summed in callback
static bool          g_stepFailed       = false; // set if dWorldQuickStep ever returns 0

// nearCallback: broadphase hands us a possibly-touching geom pair; we run the
// narrow-phase dCollide and turn each contact point into a contact joint.
static void nearCallback(void * /*data*/, dGeomID o1, dGeomID o2)
{
    dBodyID b1 = dGeomGetBody(o1);
    dBodyID b2 = dGeomGetBody(o2);

    // Skip pairs already joined by a non-contact joint (we have none here, but
    // dAreConnectedExcluding is the correct, future-proof guard; it lets stacked
    // bodies keep generating fresh contacts every step).
    if (b1 && b2 && dAreConnectedExcluding(b1, b2, dJointTypeContact))
        return;

    // dCollide only writes the .geom sub-struct; .surface is ours to fill. Run
    // the narrow phase first, then set surface params for the actual contacts
    // (n <= MAX_CONTACTS) rather than uselessly filling all 16 slots.
    dContact contact[MAX_CONTACTS];
    int n = dCollide(o1, o2, MAX_CONTACTS, &contact[0].geom, sizeof(dContact));
    for (int i = 0; i < n; i++) {
        // Coulomb-pyramid friction (Approx1) + soft contacts for stable stacking
        // + rolling/spinning friction so the ball doesn't roll forever (which
        // would prevent the system from ever settling).
        contact[i].surface.mode = dContactApprox1 | dContactSoftCFM |
                                  dContactSoftERP | dContactRolling;
        contact[i].surface.mu       = MU;
        contact[i].surface.soft_cfm = SOFT_CFM;
        contact[i].surface.soft_erp = SOFT_ERP;
        contact[i].surface.rho      = RHO;     // rolling friction
        contact[i].surface.rho2     = RHO;
        contact[i].surface.rhoN     = RHO;     // spinning friction

        dJointID c = dJointCreateContact(world, contactgroup, &contact[i]);
        dJointAttach(c, b1, b2);
    }
    g_contactsThisStep += n;
}

// ----------------------------------------------------------------------------
// Per-body energy. ODE has no energy function; we compute it from first
// principles. Rotational KE needs omega in the *body* frame because the inertia
// tensor dMass.I is stored in body coordinates: omega_body = R^T * omega_world.
// (dBodyGetRotation returns a body->world matrix, dMatrix3 row-major 3x4.)
// ----------------------------------------------------------------------------
static void bodyEnergy(dBodyID b, double &keLin, double &keRot, double &pe)
{
    dMass m;
    dBodyGetMass(b, &m);

    const dReal *v = dBodyGetLinearVel(b);
    keLin = 0.5 * m.mass * (v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);

    const dReal *w = dBodyGetAngularVel(b);  // world frame
    const dReal *R = dBodyGetRotation(b);    // body->world, row-major 3x4 (R[r*4+c])

    // omega in body frame: wb[i] = sum_r R(r,i) * w(r)  == (R^T w)
    double wb[3];
    for (int i = 0; i < 3; i++)
        wb[i] = R[0*4+i]*w[0] + R[1*4+i]*w[1] + R[2*4+i]*w[2];

    // KE_rot = 0.5 * wb^T I_body wb,  I(i,j) = m.I[i*4+j]
    double ker = 0.0;
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            ker += wb[i] * m.I[i*4+j] * wb[j];
    keRot = 0.5 * ker;

    const dReal *p = dBodyGetPosition(b);
    pe = m.mass * GRAVITY * p[2];            // reference z = 0 at the ground
}

// Whole-system snapshot.
struct Snapshot {
    double keLin = 0, keRot = 0, pe = 0;
    double maxSpeed = 0;     // peak linear speed of any body
    double maxBlockZ = 0;    // tallest block COM height (tower-height proxy)
    double meanBlockZ = 0;
    int    awake = 0;        // bodies not auto-disabled
    bool   nan = false;      // any non-finite position/velocity
    double ke() const { return keLin + keRot; }
    double e()  const { return keLin + keRot + pe; }
};

static bool finite3(const dReal *a)
{
    return std::isfinite(a[0]) && std::isfinite(a[1]) && std::isfinite(a[2]);
}

static Snapshot snapshot(const std::vector<dBodyID> &blocks, dBodyID ball)
{
    Snapshot s;
    std::vector<dBodyID> all = blocks;
    if (ball) all.push_back(ball);

    double zsum = 0;
    for (dBodyID b : all) {
        double kl, kr, pe;
        bodyEnergy(b, kl, kr, pe);
        s.keLin += kl; s.keRot += kr; s.pe += pe;

        const dReal *v = dBodyGetLinearVel(b);
        const dReal *p = dBodyGetPosition(b);
        if (!finite3(v) || !finite3(p)) s.nan = true;
        double sp = std::sqrt(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);
        if (sp > s.maxSpeed) s.maxSpeed = sp;
        if (dBodyIsEnabled(b)) s.awake++;
    }
    for (dBodyID b : blocks) {
        const dReal *p = dBodyGetPosition(b);
        if (p[2] > s.maxBlockZ) s.maxBlockZ = p[2];
        zsum += p[2];
    }
    s.meanBlockZ = blocks.empty() ? 0 : zsum / (double)blocks.size();
    return s;
}

static bool atRest(const Snapshot &s)
{
    return (s.awake == 0) || (s.ke() < KE_SETTLE && s.maxSpeed < V_SETTLE);
}

// ----------------------------------------------------------------------------
// Tower construction. Odd layers run along x, even layers run along y (rotated
// 90 deg), exactly like real Jenga.
// ----------------------------------------------------------------------------
static void buildTower(std::vector<dBodyID> &blocks)
{
    for (int layer = 0; layer < LAYERS; layer++) {
        bool alongX = (layer % 2 == 0);
        double z = BLK_HGT * 0.5 + layer * (BLK_HGT + GAP);

        // Box full-side lengths depend on orientation.
        double lx = alongX ? BLK_LEN : BLK_WID;
        double ly = alongX ? BLK_WID : BLK_LEN;
        double lz = BLK_HGT;

        for (int j = 0; j < PER_LAYER; j++) {
            double off = (j - 1) * BLK_WID;   // -BLK_WID, 0, +BLK_WID
            double x = alongX ? 0.0 : off;
            double y = alongX ? off : 0.0;

            dBodyID body = dBodyCreate(world);
            dBodySetPosition(body, x, y, z);

            dMass m;
            dMassSetBox(&m, BLK_DENS, lx, ly, lz);
            dBodySetMass(body, &m);

            dGeomID g = dCreateBox(space, lx, ly, lz);
            dGeomSetBody(g, body);

            blocks.push_back(body);
        }
    }
}

// Launch the projectile: returns the new ball body.
static dBodyID throwBall(double towerHeight)
{
    double z = BALL_R + BALL_Z_FRAC * towerHeight;   // aim low-ish for leverage

    dBodyID body = dBodyCreate(world);
    dBodySetPosition(body, BALL_X0, 0.0, z);

    dMass m;
    dMassSetSphereTotal(&m, BALL_MASS, BALL_R);
    dBodySetMass(body, &m);

    dGeomID g = dCreateSphere(space, BALL_R);
    dGeomSetBody(g, body);

    dBodySetLinearVel(body, BALL_SPEED, 0.0, 0.0);   // straight at the tower
    return body;
}

// One integration step: collide -> step -> discard contacts. Returns the
// contact count for the step. Sets g_stepFailed if the stepper could not run
// (allocation failure: the world did NOT advance, so the caller must stop).
static long stepWorld()
{
    g_contactsThisStep = 0;
    dSpaceCollide(space, 0, &nearCallback);
    if (!dWorldQuickStep(world, STEP)) {
        g_stepFailed = true;
        std::fprintf(stderr, "FATAL: dWorldQuickStep allocation failed — "
                             "world did not advance\n");
    }
    dJointGroupEmpty(contactgroup);
    return g_contactsThisStep;
}

static void printRow(const char *phase, double t, long contacts, const Snapshot &s)
{
    std::printf("%-7s t=%6.3f  contacts=%4ld  KE=%9.4f  PE=%9.4f  E=%9.4f  "
                "vmax=%7.4f  awake=%3d  topZ=%6.3f\n",
                phase, t, contacts, s.ke(), s.pe, s.e(), s.maxSpeed,
                s.awake, s.maxBlockZ);
}

// Self-check record. Detail is pre-formatted with a literal format string at the
// call site (no runtime format string, no variadic type pitfalls).
struct Check { std::string name; bool ok; std::string detail; };

int main()
{
    // --- Lifecycle: init must precede any other ODE call. --------------------
    dInitODE2(0);

    // Sanity: the linked library must be double precision (matches our headers).
    if (!dCheckConfiguration("ODE_double_precision")) {
        std::fprintf(stderr, "FATAL: linked ODE library is not double precision.\n");
        dCloseODE();
        return 2;
    }

    world        = dWorldCreate();
    space        = dHashSpaceCreate(0);     // hash space scales for many geoms
    contactgroup = dJointGroupCreate(0);

    // World tokens — set BEFORE creating bodies (auto-disable/damping defaults
    // are copied into each body at creation time).
    dWorldSetGravity(world, 0, 0, -GRAVITY);
    dWorldSetERP(world, 0.2);
    dWorldSetCFM(world, 1e-5);
    dWorldSetQuickStepNumIterations(world, QS_ITERS);
    dWorldSetContactSurfaceLayer(world, 0.001);     // anti-jitter at rest
    dWorldSetContactMaxCorrectingVel(world, 1.0);   // anti-popping
    dWorldSetLinearDamping(world, 0.01);            // bleed residual motion so it settles
    dWorldSetAngularDamping(world, 0.05);
    dWorldSetMaxAngularSpeed(world, 200);

    dWorldSetAutoDisableFlag(world, 1);
    dWorldSetAutoDisableLinearThreshold(world, 0.02);
    dWorldSetAutoDisableAngularThreshold(world, 0.02);
    dWorldSetAutoDisableAverageSamplesCount(world, 10);
    dWorldSetAutoDisableSteps(world, 10);

    dCreatePlane(space, 0, 0, 1, 0);        // static ground (a geom with no body)

    std::vector<dBodyID> blocks;
    buildTower(blocks);

    const double initialTopZ = BLK_HGT * 0.5 + (LAYERS - 1) * (BLK_HGT + GAP);
    std::printf("=== ODE Jenga knock-over ===\n");
    std::printf("blocks=%d (%d layers x %d)  block=%.2fx%.2fx%.2f  "
                "tower height=%.3f\n",
                (int)blocks.size(), LAYERS, PER_LAYER,
                BLK_LEN, BLK_WID, BLK_HGT, initialTopZ + BLK_HGT * 0.5);
    std::printf("stepper=QuickStep  h=%.4f  iters=%d  gravity=%.2f\n\n",
                STEP, QS_ITERS, GRAVITY);

    // --- Phase 1: let the tower settle and confirm it stands. ----------------
    // Require a *sustained* quiet streak, not a single lull (a settling stack
    // passes through transient velocity-reversal dead-spots that look quiet).
    std::printf("--- phase 1: settle (verify the tower stands) ---\n");
    double t = 0.0;
    int settleSteps = 0, quiet = 0;
    long lastC = 0;
    for (int i = 0; i * STEP < T_SETTLE_MAX; i++) {
        lastC = stepWorld();
        if (g_stepFailed) break;
        t += STEP;
        settleSteps = i + 1;

        Snapshot s = snapshot(blocks, 0);
        if (s.nan) {
            std::printf("!! NaN during settle at t=%.3f — aborting\n", t);
            dJointGroupDestroy(contactgroup); dSpaceDestroy(space);
            dWorldDestroy(world); dCloseODE();
            return 3;
        }
        if (i % 40 == 0) printRow("settle", t, lastC, s);

        quiet = atRest(s) ? quiet + 1 : 0;
        if (i > 40 && quiet >= QUIET_STREAK) break;
    }
    Snapshot afterSettle = snapshot(blocks, 0);
    printRow("settle", t, lastC, afterSettle);
    const double standingTopZ  = afterSettle.maxBlockZ;
    const bool   settleRested  = atRest(afterSettle);

    // Record each block's settled pose — the baseline for "did this block move?"
    std::vector<double> sx(blocks.size()), sy(blocks.size()), sz(blocks.size());
    for (size_t k = 0; k < blocks.size(); k++) {
        const dReal *p = dBodyGetPosition(blocks[k]);
        sx[k] = p[0]; sy[k] = p[1]; sz[k] = p[2];
    }
    std::printf("tower settled after %d steps (rested=%s): standing height "
                "(top block COM) = %.3f  (initial %.3f)\n\n",
                settleSteps, settleRested ? "yes" : "no", standingTopZ, initialTopZ);

    // --- Phase 2: throw the sphere. ------------------------------------------
    std::printf("--- phase 2: throw the sphere ---\n");
    dBodyID ball = throwBall(standingTopZ + BLK_HGT * 0.5);
    {
        const dReal *bp = dBodyGetPosition(ball);
        std::printf("ball: r=%.2f mass=%.2f launched from (%.2f,%.2f,%.2f) "
                    "vel=(%.1f,0,0)  KE_in=%.3f\n\n",
                    BALL_R, BALL_MASS, bp[0], bp[1], bp[2], BALL_SPEED,
                    0.5 * BALL_MASS * BALL_SPEED * BALL_SPEED);
    }

    // --- Phase 3: run through impact until the system re-settles. -------------
    // Energy accounting: after launch no external work is added, so total
    // mechanical energy E=KE+PE (which already includes gravity via PE) must be
    // non-increasing. We track the worst single-step *rise* in E (spurious
    // solver energy) and the launch baseline (for net dissipation).
    std::printf("--- phase 3: impact + re-settle ---\n");
    const double launchE = snapshot(blocks, ball).e();
    double prevE = launchE, peakE = launchE, worstRise = 0.0;
    long   peakContacts = 0;
    bool   settled = false;
    int    runSteps = 0;
    quiet = 0;
    for (int i = 0; i * STEP < T_RUN_MAX; i++) {
        lastC = stepWorld();
        if (g_stepFailed) break;
        t += STEP;
        runSteps = i + 1;
        if (lastC > peakContacts) peakContacts = lastC;

        Snapshot s = snapshot(blocks, ball);
        if (s.nan) { std::printf("!! NaN detected at t=%.3f — aborting\n", t); break; }

        double E = s.e();
        if (E - prevE > worstRise) worstRise = E - prevE;
        prevE = E;
        if (E > peakE) peakE = E;

        if (i % 40 == 0) printRow("run", t, lastC, s);

        quiet = atRest(s) ? quiet + 1 : 0;
        if (i > 60 && quiet >= QUIET_STREAK) {
            settled = true;
            printRow("run", t, lastC, s);
            break;
        }
    }

    Snapshot fin = snapshot(blocks, ball);
    std::printf("\n--- final state ---\n");
    printRow("final", t, lastC, fin);

    // --- Collapse metrics: displacement of each block from its settled pose. --
    const double finalTopZ = fin.maxBlockZ;
    int displaced = 0, floored = 0;
    for (size_t k = 0; k < blocks.size(); k++) {
        const dReal *p = dBodyGetPosition(blocks[k]);
        double dx = p[0]-sx[k], dy = p[1]-sy[k], dz = p[2]-sz[k];
        if (std::sqrt(dx*dx + dy*dy + dz*dz) > DISPLACE_THRESH) displaced++;
        if (p[2] < 3 * BLK_HGT) floored++;
    }
    const double toppleFrac = (double)displaced / (double)blocks.size();
    const double dissipated = launchE > 0 ? (launchE - fin.e()) / launchE : 0.0;

    // --- Self-checks ---------------------------------------------------------
    std::vector<Check> checks;
    char d[200];
    auto add = [&](const char *name, bool ok, const std::string &detail) {
        checks.push_back({name, ok, detail});
    };

    std::snprintf(d, sizeof(d), "all positions & velocities finite");
    add("no NaN / non-finite state", !fin.nan, d);

    std::snprintf(d, sizeof(d), "no dWorldQuickStep allocation failure");
    add("every step advanced", !g_stepFailed, d);

    std::snprintf(d, sizeof(d), "sustained >=%d quiet steps; final KE=%.4f (<%.2f), vmax=%.4f",
                  QUIET_STREAK, fin.ke(), KE_SETTLE, fin.maxSpeed);
    add("system settled (sustained)", settled, d);

    bool stood = settleRested &&
                 standingTopZ >= 0.95 * initialTopZ &&
                 standingTopZ <= 1.05 * initialTopZ;
    std::snprintf(d, sizeof(d), "standing topZ=%.3f in [%.3f, %.3f] and at rest=%s",
                  standingTopZ, 0.95*initialTopZ, 1.05*initialTopZ,
                  settleRested ? "yes" : "no");
    add("tower stood during settle", stood, d);

    std::snprintf(d, sizeof(d), "final topZ=%.3f < 0.6*standing=%.3f",
                  finalTopZ, 0.6*standingTopZ);
    add("tower toppled after impact", finalTopZ < 0.6 * standingTopZ, d);

    std::snprintf(d, sizeof(d), "%d/%d blocks (%.0f%%) moved > %.2f from settled pose (>50%% req)",
                  displaced, (int)blocks.size(), toppleFrac*100.0, DISPLACE_THRESH);
    add("majority of blocks displaced", toppleFrac > 0.5, d);

    std::snprintf(d, sizeof(d), "worst single-step E rise=%.4f <= %.2f",
                  worstRise, ENERGY_RISE_TOL);
    add("no spurious energy injection", worstRise <= ENERGY_RISE_TOL, d);

    std::snprintf(d, sizeof(d), "final E=%.3f < launch E=%.3f (%.0f%% dissipated)",
                  fin.e(), launchE, dissipated*100.0);
    add("net energy dissipated", fin.e() < launchE, d);

    std::printf("\n=== self-check ===\n");
    bool allOk = true;
    for (const Check &c : checks) {
        std::printf("  [%s] %-32s  %s\n", c.ok ? "PASS" : "FAIL",
                    c.name.c_str(), c.detail.c_str());
        allOk = allOk && c.ok;
    }

    std::printf("\nsummary: settle steps=%d, run steps=%d, peak contacts/step=%ld, "
                "launch E=%.3f, peak E=%.3f, worst step rise=%.4f, "
                "blocks displaced=%d/%d (%.0f%%), floored=%d (%.0f%%)\n",
                settleSteps, runSteps, peakContacts, launchE, peakE, worstRise,
                displaced, (int)blocks.size(), toppleFrac*100.0,
                floored, 100.0*floored/blocks.size());
    std::printf("RESULT: %s\n", allOk ? "ALL CHECKS PASSED" : "CHECKS FAILED");

    // --- Teardown (space frees the geoms it owns). ---------------------------
    dJointGroupDestroy(contactgroup);
    dSpaceDestroy(space);
    dWorldDestroy(world);
    dCloseODE();

    return allOk ? 0 : 1;
}
