// harness_selftest.cpp — exercises every verify_harness.hpp primitive on a real sim, and doubles as the
// canonical usage example. A 2-link pendulum (hinge joints, NO collision — pure dynamics) dropped and
// graded headless. Build (the c++ driver + ode-config, no precision macro):
//   c++ -O2 -std=c++17 $(ode-config --cflags) harness_selftest.cpp $(ode-config --libs) -o harness_selftest
//   ./harness_selftest        # 0 = all checks pass, 1 = a check failed, 2 = precision mismatch
#include "verify_harness.hpp"
#include <vector>

static const dReal G[3] = {0, 0, -9.81};

int main() {
    if (!dCheckConfiguration("ODE_double_precision")) { std::printf("precision mismatch\n"); return 2; }
    dInitODE2(0);
    dWorldID world = dWorldCreate();
    dWorldSetGravity(world, G[0], G[1], G[2]);
    dWorldSetCFM(world, 1e-5);

    // two box links hanging from a fixed pivot; hinge axes +Y so the swing is planar
    std::vector<dBodyID> bodies;
    std::vector<dJointID> joints;
    dBodyID prev = 0;                                   // 0 = the static world
    dReal x = 0;
    for (int k = 0; k < 2; ++k) {
        dBodyID b = dBodyCreate(world);
        dBodySetPosition(b, x + 0.25, 0, 1.0);
        dMass m; dMassSetBoxTotal(&m, 1.0, 0.5, 0.1, 0.1); dBodySetMass(b, &m);
        dJointID j = dJointCreateHinge(world, 0);
        dJointAttach(j, prev, b);                       // first link attaches to the world (prev==0)
        dJointSetHingeAnchor(j, x, 0, 1.0);
        dJointSetHingeAxis(j, 0, 1, 0);
        bodies.push_back(b); joints.push_back(j);
        prev = b; x += 0.5;
    }

    const dReal DT = 0.001;
    double E0 = ode_verify::total_energy(bodies, G);    // baseline at release
    bool exploded = false;
    double maxSep = 0, lowZ = 1e300, maxE = E0;
    for (int i = 0; i < 4000; ++i) {                    // pure-dynamics loop: no dSpaceCollide / contacts
        if (!dWorldStep(world, DT)) { exploded = true; break; }
        if (!ode_verify::finite(bodies)) { exploded = true; break; }
        double e = ode_verify::total_energy(bodies, G); if (e > maxE) maxE = e;
        for (dJointID j : joints) { double s = ode_verify::joint_separation(j); if (s > maxSep) maxSep = s; }
        double z = ode_verify::min_z(bodies); if (z < lowZ) lowZ = z;
    }

    uint64_t digest = ode_verify::pose_digest(bodies);
    double Escale = std::fabs(E0) + 1.0;

    ode_verify::Checks V;
    V.check("no NaN / explosion", !exploded);
    V.check("energy bounded (no spurious gain)", maxE <= E0 + 1e-2 * Escale);
    V.check("joints intact (sep < 1mm)",         maxSep < 1e-3);
    V.check("no tunneling below pivot",          lowZ > -1.0);
    V.check("bounded speed",                     ode_verify::max_speed(bodies) < 50.0);
    // A/B differential demo: the pendulum must actually SWING (moved metric) vs a frozen baseline.
    V.check("pendulum actually moved",           ode_verify::differs(lowZ, 1.0, 0.1));
    std::printf("  digest = %016llx (run twice; equal ⇒ deterministic)\n", (unsigned long long)digest);

    int rc = V.report("harness_selftest");
    dWorldDestroy(world);
    dCloseODE();
    return rc;
}
