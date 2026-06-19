// probe_sign2.cpp — interior-joint AMotor sign convention (BOTH bodies dynamic),
// mirroring arm6dof's attach(link[i-1], link[i]). body0 is rigidly fixed to the
// world by a dFixed joint (so it is a dynamic body but immobile); body1 hangs off
// it via hinge+AMotor exactly like an interior arm joint.
//
// anchor=(0,0,1), axis=(0,1,0), body1 capsule along +X centered (0.5,0,1) so
// r=+X and Jacobian column axis x r = (0,1,0)x(1,0,0) = -Z. A POSITIVE command in
// my Jacobian convention should move the tip in -Z. We command +0.5 and look.

#include <ode/ode.h>
#include <cstdio>

int main() {
    dInitODE2(0);
    dAllocateODEDataForThread(dAllocateMaskAll);
    dWorldID world = dWorldCreate();
    dWorldSetGravity(world, 0, 0, 0);
    dWorldSetERP(world, 0.2);
    dWorldSetCFM(world, 1e-5);

    // body0 (proximal) — pinned rigidly to the world but is itself a dynamic body
    dBodyID b0 = dBodyCreate(world);
    dMass m0; dMassSetZero(&m0); dMassSetCapsuleTotal(&m0, 0.5, 3, 0.035, 0.18);
    dBodySetMass(b0, &m0);
    dBodySetPosition(b0, -0.2, 0.0, 1.0);
    dJointID fix = dJointCreateFixed(world, 0);
    dJointAttach(fix, 0, b0);
    dJointSetFixed(fix);

    // body1 (distal) — interior joint attach(b0, b1)
    dBodyID b1 = dBodyCreate(world);
    dMass m1; dMassSetZero(&m1); dMassSetCapsuleTotal(&m1, 0.5, 3, 0.035, 0.18);
    dBodySetMass(b1, &m1);
    dBodySetPosition(b1, 0.5, 0.0, 1.0);
    dMatrix3 R; dRFromAxisAndAngle(R, 0,1,0, M_PI*0.5);
    dBodySetRotation(b1, R);

    dJointID hinge = dJointCreateHinge(world, 0);
    dJointAttach(hinge, b0, b1);                 // interior: body1=proximal, body2=distal
    dJointSetHingeAnchor(hinge, 0, 0, 1);
    dJointSetHingeAxis(hinge, 0, 1, 0);
    dJointSetHingeParam(hinge, dParamFMax, 0.0);

    dJointID am = dJointCreateAMotor(world, 0);
    dJointAttach(am, b0, b1);                     // same order as hinge
    dJointSetAMotorMode(am, dAMotorUser);
    dJointSetAMotorNumAxes(am, 1);
    dJointSetAMotorAxis(am, 0, 0, 0, 1, 0);
    dJointSetAMotorParam(am, dParamFMax, 50.0);
    dJointSetAMotorParam(am, dParamVel, +0.5);

    dVector3 tip0; dBodyGetRelPointPos(b1, 0,0, 0.125, tip0);
    for (int s = 0; s < 60; s++) {
        dJointSetAMotorAxis(am, 0, 0, 0, 1, 0);
        dJointSetAMotorParam(am, dParamVel, +0.5);
        dJointSetAMotorParam(am, dParamFMax, 50.0);
        dWorldStep(world, 0.005);
    }
    dVector3 tipf; dBodyGetRelPointPos(b1, 0,0, 0.125, tipf);
    double dz = tipf[2]-tip0[2];
    printf("interior joint: +dParamVel moved tip_z by %+.4f\n", dz);
    printf("Jacobian predicts -Z for +cmd. Interior sign is %s.\n",
           dz < 0 ? "CONSISTENT (use dq as-is)" : "FLIPPED (negate dq)");
    dWorldDestroy(world);
    dCloseODE();
    return 0;
}
