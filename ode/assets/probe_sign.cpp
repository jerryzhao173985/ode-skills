// probe_sign.cpp — determine the sign mapping between an AMotor dParamVel command
// and the Jacobian convention (axis x r). Gravity OFF; one capsule pinned to the
// world by hinge+AMotor exactly like arm6dof joint i (attach(0, body)).
//
// Setup: anchor=(0,0,1), axis=(0,1,0). Body capsule along +X, center (0.5,0,1) so
// r = ee-anchor = +X. Jacobian column = axis x r = (0,1,0)x(1,0,0) = (0,0,-1):
// so a POSITIVE joint rate (my convention) should move the tip in -Z (downward).
// We command dParamVel=+0.5 and observe the tip's actual z motion + the hinge
// angle rate to learn ODE's true sign.

#include <ode/ode.h>
#include <cstdio>

int main() {
    dInitODE2(0);
    dAllocateODEDataForThread(dAllocateMaskAll);
    dWorldID world = dWorldCreate();
    dWorldSetGravity(world, 0, 0, 0);            // isolate the motor
    dWorldSetERP(world, 0.2);
    dWorldSetCFM(world, 1e-5);

    dBodyID b = dBodyCreate(world);
    dMass m; dMassSetZero(&m);
    dMassSetCapsuleTotal(&m, 0.5, 3, 0.035, 0.18);
    dBodySetMass(b, &m);
    dBodySetPosition(b, 0.5, 0.0, 1.0);
    dMatrix3 R; dRFromAxisAndAngle(R, 0,1,0, M_PI*0.5); // local Z -> world +X
    dBodySetRotation(b, R);

    dJointID hinge = dJointCreateHinge(world, 0);
    dJointAttach(hinge, 0, b);                   // body1 = static world, body2 = b
    dJointSetHingeAnchor(hinge, 0, 0, 1);
    dJointSetHingeAxis(hinge, 0, 1, 0);
    dJointSetHingeParam(hinge, dParamFMax, 0.0); // hinge motor off

    dJointID am = dJointCreateAMotor(world, 0);
    dJointAttach(am, 0, b);
    dJointSetAMotorMode(am, dAMotorUser);
    dJointSetAMotorNumAxes(am, 1);
    dJointSetAMotorAxis(am, 0, 0, 0, 1, 0);
    dJointSetAMotorParam(am, dParamFMax, 50.0);
    dJointSetAMotorParam(am, dParamVel, +0.5);   // command +0.5 rad/s

    dVector3 tip0; dBodyGetRelPointPos(b, 0,0, 0.125, tip0);
    printf("tip0 z=%.4f  (Jacobian predicts +cmd -> tip moves in -Z)\n", tip0[2]);

    for (int s = 0; s < 60; s++) {
        dJointSetAMotorAxis(am, 0, 0, 0, 1, 0);  // refresh like arm6dof
        dJointSetAMotorParam(am, dParamVel, +0.5);
        dJointSetAMotorParam(am, dParamFMax, 50.0);
        dWorldStep(world, 0.005);
        if (s % 10 == 0 || s == 59) {
            dVector3 tip; dBodyGetRelPointPos(b, 0,0, 0.125, tip);
            printf("s=%2d  hingeAngle=%+.4f  hingeRate=%+.4f  tip_z=%+.4f  dz=%+.4f\n",
                   s, dJointGetHingeAngle(hinge), dJointGetHingeAngleRate(hinge),
                   tip[2], tip[2]-tip0[2]);
        }
    }
    dVector3 tipf; dBodyGetRelPointPos(b, 0,0, 0.125, tipf);
    double dz = tipf[2]-tip0[2];
    printf("\nVERDICT: +dParamVel moved tip_z by %+.4f\n", dz);
    printf("Jacobian (axis x r) predicts -Z for +cmd. Sign is %s.\n",
           dz < 0 ? "CONSISTENT (use dq as-is)" : "FLIPPED (negate dq)");
    dWorldDestroy(world);
    dCloseODE();
    return 0;
}
