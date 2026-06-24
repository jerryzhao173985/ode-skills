// verify_harness.hpp — reusable headless self-check primitives for ODE C++ sims.
// Header-only; depends only on <ode/ode.h>. This is the verification structure every field build
// re-derived: a NaN/explosion trip-wire, a fixed-baseline energy check (ODE never computes energy),
// a determinism fingerprint, a no-tunnel floor check, joint constraint-satisfaction, an A/B
// "differential" so a check can't pass on a confound, and a pass/fail verdict with an exit code.
// Method & rationale: references/foundations/verifying-simulations.md.  Worked use: assets/harness_selftest.cpp.
//
//   #include "verify_harness.hpp"
//   std::vector<dBodyID> bodies = {...};
//   double E0 = ode_verify::total_energy(bodies, g);          // baseline after settling / at launch
//   for (int i=0;i<STEPS;i++){ step(); if(!ode_verify::finite(bodies)){break;} }
//   ode_verify::Checks V;
//   V.check("finite",   ode_verify::finite(bodies));
//   V.check("no tunnel", ode_verify::min_z(bodies) > FLOOR - 0.05);
//   V.check("energy bounded", ode_verify::total_energy(bodies,g) <= E0 + 1e-3*Escale); // PASSIVE systems only
//   // Non-conservative (thrust/motor-driven) sims ADD energy by design — skip the energy check; instead
//   // assert reached-target + bounded state (e.g. V.check("reached", dist<tol); V.check("upright", tilt<lim);).
//   return V.report("my_sim");                                // prints PASS/FAIL lines + RESULT, returns 0/1
#ifndef ODE_VERIFY_HARNESS_HPP
#define ODE_VERIFY_HARNESS_HPP
#include <ode/ode.h>
#include <cstdio>
#include <cmath>
#include <cstdint>
#include <vector>

namespace ode_verify {

// --- NaN / explosion trip-wire (check every step; fail fast) ----------------
inline bool vec_finite(const dReal* v, int n = 3) {
    for (int i = 0; i < n; ++i) if (!std::isfinite((double)v[i])) return false;
    return true;
}
inline bool finite(const std::vector<dBodyID>& bodies) {
    for (dBodyID b : bodies)
        if (!vec_finite(dBodyGetPosition(b)) || !vec_finite(dBodyGetLinearVel(b)) ||
            !vec_finite(dBodyGetAngularVel(b)) || !vec_finite(dBodyGetQuaternion(b), 4)) return false;
    return true;
}
inline double min_z(const std::vector<dBodyID>& bodies) {      // catch tunneling through a Z-up floor
    double m = 1e300; for (dBodyID b : bodies) { double z = dBodyGetPosition(b)[2]; if (z < m) m = z; } return m;
}
inline double max_speed(const std::vector<dBodyID>& bodies) {  // catch blow-ups
    double m = 0; for (dBodyID b : bodies) { const dReal* v = dBodyGetLinearVel(b);
        double s = std::sqrt((double)(v[0]*v[0] + v[1]*v[1] + v[2]*v[2])); if (s > m) m = s; } return m;
}

// --- total mechanical energy (the recipe ODE doesn't give you) --------------
// dMass.I is a dMatrix3 stored 3x4 row-padded (index I[row*4+col]); angular velocity is world-frame,
// so transform to the body frame with R^T before applying the inertia. g is your gravity vector.
inline double total_energy(const std::vector<dBodyID>& bodies, const dReal g[3]) {
    double E = 0;
    for (dBodyID b : bodies) {
        dMass m; dBodyGetMass(b, &m);
        const dReal *v = dBodyGetLinearVel(b), *w = dBodyGetAngularVel(b),
                    *p = dBodyGetPosition(b),  *R = dBodyGetRotation(b);
        double ke = 0.5 * m.mass * (v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);
        double wb[3]; for (int i = 0; i < 3; ++i) wb[i] = R[0*4+i]*w[0] + R[1*4+i]*w[1] + R[2*4+i]*w[2];
        double Iw[3]; for (int i = 0; i < 3; ++i) Iw[i] = m.I[i*4+0]*wb[0] + m.I[i*4+1]*wb[1] + m.I[i*4+2]*wb[2];
        ke += 0.5 * (wb[0]*Iw[0] + wb[1]*Iw[1] + wb[2]*Iw[2]);
        E += ke - m.mass * (g[0]*p[0] + g[1]*p[1] + g[2]*p[2]);   // + PE (for g_z<0 this is +m|g|z)
    }
    return E;
}

// --- determinism fingerprint (FNV-1a over final poses) ----------------------
// Compare digests across TWO PROCESS LAUNCHES, not in-process reruns: under dWorldQuickStep+dHashSpace an
// in-process rerun reorders contacts (broadphase keyed on geom pointer address) and drifts at the ULP level
// even single-threaded. Equal digests across processes => bit-reproducible (fixed DT, no RNG, single-thread).
inline uint64_t fnv1a(const void* data, size_t n, uint64_t h = 1469598103934665603ULL) {
    const unsigned char* p = (const unsigned char*)data;
    for (size_t i = 0; i < n; ++i) { h ^= p[i]; h *= 1099511628211ULL; }
    return h;
}
inline uint64_t pose_digest(const std::vector<dBodyID>& bodies) {
    uint64_t h = 1469598103934665603ULL;
    for (dBodyID b : bodies) {
        const dReal *p = dBodyGetPosition(b), *q = dBodyGetQuaternion(b);
        double buf[7] = { (double)p[0],(double)p[1],(double)p[2],(double)q[0],(double)q[1],(double)q[2],(double)q[3] };
        h = fnv1a(buf, sizeof(buf), h);
    }
    return h;
}

// --- constraint satisfaction: |anchor − anchor2| for a joint ----------------
// The two anchor read-backs coincide only while the constraint holds; tiny at rest ⇒ joint intact.
// (Slider/fixed/AMotor have no positional anchor pair → returns 0.)
inline double joint_separation(dJointID j) {
    dVector3 a = {0,0,0}, b = {0,0,0};
    switch (dJointGetType(j)) {
        case dJointTypeBall:      dJointGetBallAnchor(j,a);      dJointGetBallAnchor2(j,b);      break;
        case dJointTypeHinge:     dJointGetHingeAnchor(j,a);     dJointGetHingeAnchor2(j,b);     break;
        case dJointTypeHinge2:    dJointGetHinge2Anchor(j,a);    dJointGetHinge2Anchor2(j,b);    break;
        case dJointTypeUniversal: dJointGetUniversalAnchor(j,a); dJointGetUniversalAnchor2(j,b); break;
        default: return 0.0;
    }
    double dx = a[0]-b[0], dy = a[1]-b[1], dz = a[2]-b[2];
    return std::sqrt(dx*dx + dy*dy + dz*dz);
}

// --- A/B differential: a check is only meaningful if removing the mechanism moves the metric.
// Run with the mechanism vs a negative control, and require the DIFFERENCE (not an absolute threshold)
// — so the check can't pass on a confound (e.g. gravity sag instead of the load).
inline bool differs(double with_mechanism, double without_mechanism, double min_gap) {
    return std::fabs(with_mechanism - without_mechanism) >= min_gap;
}

// --- controlled-plant checks (non-conservative, actively-controlled sims: drones, vehicles, arms) ----
// total_energy() does NOT apply when a controller injects energy. Assert reach + attitude + settle instead.
// WARNING: "reached"/"upright" pass TRIVIALLY if the body never moved — pair them with a `differs()`
// negative control (the plant must FALL / drift to its target's miss-distance without the controller).

inline double goal_distance(dBodyID b, const double target[3]) {   // reached-target / goal-distance
    const dReal* p = dBodyGetPosition(b);
    double dx = p[0]-target[0], dy = p[1]-target[1], dz = p[2]-target[2];
    return std::sqrt(dx*dx + dy*dy + dz*dz);
}
// tilt of a body's local +Z away from world +Z, radians (0 = perfectly upright). dBodyGetRotation is 3x4
// row-padded; body +Z in world = column 2 = (R[2],R[6],R[10]), and its dot with world-up (0,0,1) is R[10].
inline double tilt(dBodyID b) {
    double c = (double)dBodyGetRotation(b)[10];
    if (c > 1) c = 1; if (c < -1) c = -1;
    return std::acos(c);
}
// steady-state detector: feed max_speed(bodies) each step; settled() is true once the last `window` samples
// are ALL below `thresh` — proves the system actually came to rest, not that one lucky frame was slow.
// --- passive-oscillator checks. A designed COMPLIANT / sprung system must be verified by its FREQUENCY
// --- and damping, NOT energy: energy-non-growth is BLIND to a wrong-stiffness spring (a 2.25x stiffness
// --- error conserves energy but shifts the measured frequency 1.5x). Feed a recorded scalar signal, e.g.
// --- the mass's z(t) sampled every step; compare against your design target f0 = (1/2pi)*sqrt(kp/m).
inline double measured_frequency(const std::vector<double>& signal, double dt) {  // Hz, upward zero-crossings
    if (signal.size() < 4 || dt <= 0) return 0.0;
    size_t tail = signal.size() / 2; double mean = 0;                  // late mean ~ the settled equilibrium
    for (size_t i = tail; i < signal.size(); ++i) mean += signal[i];
    mean /= double(signal.size() - tail);
    int crossings = 0; double t0 = -1, t1 = -1;
    for (size_t i = 1; i < signal.size(); ++i)
        if (signal[i-1] < mean && signal[i] >= mean) { double t = double(i)*dt; if (t0 < 0) t0 = t; t1 = t; ++crossings; }
    return (crossings < 2 || t1 <= t0) ? 0.0 : double(crossings - 1) / (t1 - t0);
}
inline double damping_ratio_logdec(const std::vector<double>& signal) {            // zeta from log-decrement
    if (signal.size() < 4) return 0.0;
    size_t tail = signal.size() / 2; double mean = 0;
    for (size_t i = tail; i < signal.size(); ++i) mean += signal[i];
    mean /= double(signal.size() - tail);
    std::vector<double> pk;                                            // local maxima above the mean
    for (size_t i = 1; i + 1 < signal.size(); ++i)
        if (signal[i] > mean && signal[i] > signal[i-1] && signal[i] >= signal[i+1]) pk.push_back(signal[i] - mean);
    if (pk.size() < 2 || pk.front() <= 0 || pk.back() <= 0) return 0.0;
    double delta = std::log(pk.front() / pk.back()) / double(pk.size() - 1);        // mean per-cycle decrement
    static const double PI = 3.14159265358979323846;
    return delta / std::sqrt(4.0 * PI * PI + delta * delta);
}

struct Settle {
    std::vector<double> hist;
    void update(double speed) { hist.push_back(speed); }
    bool settled(size_t window, double thresh) const {
        if (hist.size() < window) return false;
        for (size_t i = hist.size() - window; i < hist.size(); ++i) if (hist[i] >= thresh) return false;
        return true;
    }
};
// control-effort integral: accumulate ∑|u|·dt; a huge value flags chatter/saturation — a controller that
// "holds" a pose via violent actuation passes a pose check but is not actually stable.
struct Effort { double total = 0; void add(double u, double dt) { total += std::fabs(u) * dt; } };

// reaction-force probe (the dJointFeedback API) — stateful peak accumulator over a run.
// For OVER-CONSTRAINED / closed loops the constraint violation lives in the REACTION FORCE, not the anchor
// position: ODE's ERP heals |anchor−anchor2| every step, so a loop whose joints fight each other still looks
// "intact" by separation. Attach to the closing joint, update() after each dWorldStep, then assert the peak
// reaction force/torque stays bounded (a fighting loop diverges in force, not position).
// LIFETIME: ODE stores a POINTER to .fb, so this probe must outlive the world and must NOT be copied/moved
// after attach() (a copy would leave ODE pointing at the original's fb).
struct JointForceProbe {
    dJointFeedback fb{};                                        // ODE fills f1/t1 (on body1) + f2/t2 (on body2) each step
    double peakF = 0, peakT = 0, lastF = 0, lastT = 0;          // running peak |force| / |torque| over both bodies
    JointForceProbe() = default;
    JointForceProbe(const JointForceProbe&) = delete;
    JointForceProbe& operator=(const JointForceProbe&) = delete;
    void attach(dJointID j) { dJointSetFeedback(j, &fb); }      // ONCE, after dJointAttach, before stepping
    static double mag3(const dReal* v) { return std::sqrt((double)v[0]*v[0] + (double)v[1]*v[1] + (double)v[2]*v[2]); }
    void update() {                                            // AFTER each dWorldStep
        double f1 = mag3(fb.f1), f2 = mag3(fb.f2); lastF = f1 > f2 ? f1 : f2;
        double t1 = mag3(fb.t1), t2 = mag3(fb.t2); lastT = t1 > t2 ? t1 : t2;
        if (lastF > peakF) peakF = lastF;
        if (lastT > peakT) peakT = lastT;
    }
};

// --- pass/fail accumulator → exit-code verdict ------------------------------
struct Checks {
    int passed = 0, failed = 0;
    void check(const char* name, bool ok, const char* detail = "") {
        std::printf("  [%s] %s%s%s\n", ok ? "PASS" : "FAIL", name, *detail ? "  " : "", detail);
        ok ? ++passed : ++failed;
    }
    void fail(const char* name, const char* detail = "") { check(name, false, detail); }
    int report(const char* label = "sim") {                    // 0 = all pass, 1 = a check failed
        std::printf("RESULT: %s — %d/%d checks passed (%s)\n", label, passed, passed + failed,
                    failed ? "FAIL" : "PASS");
        return failed ? 1 : 0;
    }
};

} // namespace ode_verify
#endif // ODE_VERIFY_HARNESS_HPP
