# Example: 6-DOF Arm — DLS-Jacobian Cartesian reach with AMotor user-mode actuation

A self-contained, tested, **headless, self-checking** C++ program that builds a 6-link serial
robot arm (capsule links pinned by hinges, driven by parallel angular motors) and closes a
**Cartesian reach controller** around it: a 3×6 geometric Jacobian + damped least squares (DLS)
turns an end-effector position error into per-joint velocity commands. There is no window — the
program runs a fixed number of steps and prints `RESULT: PASS`/`FAIL` with the process exit code
as the verdict (`0` = PASS). Source: `assets/arm6dof.cpp` (sign calibration in
`assets/probe_sign.cpp` + `assets/probe_sign2.cpp`).

## What it demonstrates

- **AMotor user-mode actuation wired in parallel to a hinge.** Each link is constrained by a
  `dHinge` (the hinge's *own* motor is left off, `dParamFMax == 0`) and actuated by a separate
  1-axis `dAMotor` in `dAMotorUser` mode attached to the *same two bodies* — the reusable
  "kinematic joint + parallel servo" pattern (`assets/arm6dof.cpp:333-340`).
- **A damped-least-squares Cartesian Jacobian controller** built by hand: geometric Jacobian
  column `J_i = axis_i × (ee − anchor_i)`, manipulability-adaptive damping `(B + λ²I)⁻¹`, joint
  rates `dq = Jᵀ(JJᵀ + λ²I)⁻¹ v_des`, all in plain C arrays with a 3×3 adjugate inverse
  (`assets/arm6dof.cpp:191-263`).
- **Sign calibration as a falsifiable experiment.** ODE's AMotor velocity sign depends on body
  attach order (world-attached vs. two dynamic bodies); the two `probe_sign*.cpp` programs
  measure it instead of guessing, yielding the per-joint `g_amotor_sign[]` multiplier
  (`assets/arm6dof.cpp:79-85`, `assets/arm6dof.cpp:320-322`).
- **Live re-reading of joint geometry every step.** Axes and anchors are pulled from ODE each
  step (`dJointGetHingeAxis` / `dJointGetHingeAnchor`) so the Jacobian is exact for the current
  pose and the AMotor axis tracks the moving hinge (`assets/arm6dof.cpp:194-203`,
  `assets/arm6dof.cpp:260`).
- **A multi-criteria headless self-check:** reach tolerance, settle speed, hinge constraint
  separation, no ground tunneling, and finiteness — folded into one boolean and one exit code
  (`assets/arm6dof.cpp:498-511`).
- **C++ technique:** double-precision-only world (verified at runtime), fixed timestep, NaN
  trip-wire, optional CSV trajectory dump, command-line target — all without drawstuff.

## Build & run

Homebrew/standard ODE is double precision; **define no precision macro** — let `ode-config`
supply the flags:

```sh
c++ -O2 -std=c++17 $(ode-config --cflags) assets/arm6dof.cpp $(ode-config --libs) -o arm6dof
./arm6dof                 # default target (0.7, 0.3, 1.0)
./arm6dof 0.6 -0.2 0.9    # custom Cartesian target
./arm6dof 0.7 0.3 1.0 traj.csv   # also dump a per-step CSV
echo $?                   # 0 = PASS, non-zero = FAIL
```

The program is its own test. It prints a periodic state table and a final self-check block, then
exits `0` only if every criterion passes (`assets/arm6dof.cpp:507-511`). Use the exit code in CI:

```sh
./arm6dof >/dev/null && echo PASS || echo FAIL
```

`ARM_DEBUG=1 ./arm6dof` turns on per-joint controller tracing (`assets/arm6dof.cpp:264-275`,
`assets/arm6dof.cpp:358`).

## Walkthrough

### 1. World defaults set *before* bodies exist
`setup()` creates the world/space/contact group, then configures global parameters **before** any
body is created, because bodies seed their per-body defaults from the world at creation time
(`assets/arm6dof.cpp:280-295`):

```cpp
dWorldSetGravity(world, 0, 0, GRAVITY);
dWorldSetERP(world, ERP);                    // 0.2
dWorldSetCFM(world, CFM);                     // 1e-5, positive -> soft, off the singularity
dWorldSetLinearDamping(world, LIN_DAMP);
dWorldSetAngularDamping(world, ANG_DAMP);
dWorldSetAutoDisableFlag(world, 0);          // an actuated arm must never sleep
dWorldSetContactSurfaceLayer(world, 0.001);  // anti-jitter for resting contacts
dWorldSetContactMaxCorrectingVel(world, 5.0);// cap contact push-out
dCreatePlane(space, 0, 0, 1, 0);             // static ground geom (no body)
```

`dWorldSetAutoDisableFlag`, `dWorldSetContactSurfaceLayer`, and `dWorldSetContactMaxCorrectingVel`
are all in `include/ode/objects.h` (lines 748, 623, 603); `dWorldSetLinearDamping` /
`dWorldSetAngularDamping` at `objects.h:826,840`.

### 2. Links: capsule bodies + mass + geom, laid straight along +X
Capsules' long axis is local **Z**, so each link is rotated +90° about Y to point local-Z → world-+X.
Mass uses `dMassSetCapsuleTotal(&m, total, 3, R, L)` (direction `3` = local Z, `mass.h:57`)
(`assets/arm6dof.cpp:299-315`):

```cpp
dMatrix3 R;
dRFromAxisAndAngle(R, 0, 1, 0, M_PI * 0.5);   // local Z -> world +X
for (int i = 0; i < N_LINKS; i++) {
    link_body[i] = dBodyCreate(world);
    dMass m; dMassSetZero(&m);
    dMassSetCapsuleTotal(&m, LINK_MASS, 3, LINK_R, LINK_CYL);  // dir 3 = local Z
    dBodySetMass(link_body[i], &m);
    dBodySetPosition(link_body[i], i*SEG_LEN + SEG_LEN*0.5, 0.0, BASE_Z);
    dBodySetRotation(link_body[i], R);
    link_geom[i] = dCreateCapsule(space, LINK_R, LINK_CYL);
    dGeomSetBody(link_geom[i], link_body[i]);
}
```

### 3. Joints: hinge constraint + parallel AMotor, attached *after* positioning
The base link is pinned to the static world (body `0`); interior joints connect adjacent links.
Bodies are positioned **first**, because the hinge captures its zero angle from the current pose at
anchor/axis-set time. The per-joint sign multiplier is decided here from the attach order
(`assets/arm6dof.cpp:318-341`):

```cpp
hinge[i] = dJointCreateHinge(world, 0);
dBodyID b1 = (i == 0) ? 0 : link_body[i-1];           // base pinned to world
g_amotor_sign[i] = (b1 == 0) ? 1.0 : -1.0;            // calibrated (see §6)
dJointAttach(hinge[i], b1, link_body[i]);
dJointSetHingeAnchor(hinge[i], i*SEG_LEN, 0.0, BASE_Z);  // WORLD coords
dJointSetHingeAxis(hinge[i], AXIS0[i][0], AXIS0[i][1], AXIS0[i][2]);
dJointSetHingeParam(hinge[i], dParamHiStop,  2.7);
dJointSetHingeParam(hinge[i], dParamLoStop, -2.7);
dJointSetHingeParam(hinge[i], dParamHiStop,  2.7);    // hi,lo,hi ordering (ODE FAQ)
dJointSetHingeParam(hinge[i], dParamFMax, 0.0);       // hinge's OWN motor stays OFF

amotor[i] = dJointCreateAMotor(world, 0);
dJointAttach(amotor[i], b1, link_body[i]);            // SAME two bodies as the hinge
dJointSetAMotorMode(amotor[i], dAMotorUser);          // objects.h:2495
dJointSetAMotorNumAxes(amotor[i], 1);                 // 1-DOF servo
dJointSetAMotorAxis(amotor[i], 0, 0, AXIS0[i][0], AXIS0[i][1], AXIS0[i][2]); // rel=0 -> global
dJointSetAMotorParam(amotor[i], dParamFMax, FMAX);
dJointSetAMotorParam(amotor[i], dParamVel, 0.0);
```

`dJointSetAMotorAxis(j, anum, rel, x,y,z)` takes a frame selector `rel`; `rel=0` means the axis is
given in the **global** frame (`objects.h:2471`), which is why the controller can feed a live
world-frame axis each step.

### 4. The DLS-Jacobian controller (the load-bearing pattern)
Every step, `controlStep()` reads the live end-effector position, forms the Cartesian error, caps a
proportional desired velocity, then builds the geometric Jacobian column-by-column from the *current*
hinge axes and anchors (`assets/arm6dof.cpp:177-203`):

```cpp
double e[3] = { target[0]-ee[0], target[1]-ee[1], target[2]-ee[2] };
double vdes[3] = { KP*e[0], KP*e[1], KP*e[2] };      // proportional EE velocity
// ... cap |vdes| at VMAX_CART ...
for (int i = 0; i < N_LINKS; i++) {
    dVector3 ax, an;
    dJointGetHingeAxis(hinge[i], ax);                // live world axis  (objects.h:2598)
    dJointGetHingeAnchor(hinge[i], an);              // live world anchor (objects.h:2582)
    double r[3] = { ee[0]-an[0], ee[1]-an[1], ee[2]-an[2] };
    J[0][i] = ax[1]*r[2] - ax[2]*r[1];               // column i = axis_i x (ee - anchor_i)
    J[1][i] = ax[2]*r[0] - ax[0]*r[2];
    J[2][i] = ax[0]*r[1] - ax[1]*r[0];
}
```

Damping is **adaptive** (Maciejewski–Klein): manipulability `w = sqrt(det(JJᵀ))` engages damping
only near singularities and fades to a tiny floor when well-conditioned, so accuracy stays high in
the dexterous workspace while motion stays bounded near singular poses
(`assets/arm6dof.cpp:213-227`):

```cpp
double w = std::sqrt(detB > 0 ? detB : 0.0);
double lam2 = DLS_LAM2MIN;
if (w < DLS_W0) {
    double t = 1.0 - (w / DLS_W0);                   // 0 at threshold -> 1 at singularity
    lam2 = DLS_LAM2MIN + DLS_LAM2MAX * t * t;
}
double A[3][3];                                       // A = JJ^T + lambda^2 I  (SPD)
```

`A` (3×3) is inverted in closed form via adjugate/determinant (`assets/arm6dof.cpp:230-245`), then
the joint rates are `dq = Jᵀ A⁻¹ v_des`, each clamped to `±QDOT_MAX` (`assets/arm6dof.cpp:247-257`).

### 5. Actuation wiring: realign axis, command velocity, cap torque
For each joint the controller (a) **realigns** the AMotor's single user axis to the live world hinge
axis, then (b) commands the velocity servo through the calibrated sign and a torque budget
(`assets/arm6dof.cpp:258-263`):

```cpp
dJointSetAMotorAxis(amotor[i], 0, 0, axis[i][0], axis[i][1], axis[i][2]); // track moving hinge
dJointSetAMotorParam(amotor[i], dParamVel,  g_amotor_sign[i] * dq);        // signed rate command
dJointSetAMotorParam(amotor[i], dParamFMax, FMAX);                         // torque cap (N*m)
```

A velocity servo in ODE is exactly this pair: `dParamVel` is the target rate and `dParamFMax` is the
maximum force/torque the joint may apply to reach it (`objects.h:2489`). `dParamFMax == 0` means "no
actuation" — which is why the hinge's own motor (§3) is inert and the AMotor is the sole driver.

### 6. Sign calibration via two controlled probes
The AMotor velocity sign relative to the Jacobian's right-hand-rule convention (`axis × r`) is *not*
assumed — it is measured. Both probes use identical geometry (anchor `(0,0,1)`, axis `(0,1,0)`, tip
along +X so `r = +X`, and `axis × r = (0,0,−1)`), so a positive `dParamVel` *should* move the tip in
−Z. Gravity is off to isolate the motor.

- **`probe_sign.cpp`** attaches the body to the **static world**, `dJointAttach(am, 0, b)` —
  exactly arm6dof's base joint (`assets/probe_sign.cpp:31`, `assets/probe_sign.cpp:37`). It commands
  `dParamVel=+0.5`, steps, and reads back `tip_z` and `dJointGetHingeAngleRate`. The verdict line
  reports whether `+dParamVel` actually moved the tip in −Z (`assets/probe_sign.cpp:59-63`).
- **`probe_sign2.cpp`** mirrors an **interior** joint: a proximal body is rigidly fixed to the world
  with a `dFixed` joint (so it is dynamic but immobile), and the distal body hangs off it via
  `dJointAttach(am, b0, b1)` — two dynamic bodies (`assets/probe_sign2.cpp:26-28`,
  `assets/probe_sign2.cpp:45`). It runs the same command and prints the interior sign verdict
  (`assets/probe_sign2.cpp:61-63`).

The empirical result is encoded once, at joint-build time: world-attached joints take `+1`, interior
(two-dynamic-body) joints take `−1` (`assets/arm6dof.cpp:79-85`, `assets/arm6dof.cpp:320-322`). Both
probes also re-set the axis/vel/fmax every step, matching arm6dof, so the calibration is valid for
the exact command pattern the controller uses.

### 7. The per-step loop and headless self-check
The loop is the canonical ODE order with the controller spliced in *before* the step
(`assets/arm6dof.cpp:407-418`):

```cpp
for (int s = 0; s < STEPS; s++) {
    last_dist = controlStep(s);                  // sets AMotor commands for this step
    dSpaceCollide(space, 0, &nearCallback);      // ground-vs-arm contacts only
    if (!dWorldStep(world, DT)) { /* RESULT: FAIL (alloc failure) */ }
    dJointGroupEmpty(contactgroup);              // clear per-step contact joints
    // ... instrumentation: min_z, max |v|/|w|, hinge separation, NaN trip-wire ...
}
```

The collision callback **skips link-vs-link pairs** (both bodies dynamic) and only generates contacts
against the ground, so free-space reaching is unaffected by self-contact while the real contact
pipeline is still exercised vs. the floor (`assets/arm6dof.cpp:111-127`).

The self-check grades the **true post-integration** end-effector position (note: `controlStep`'s
`last_dist` is measured *before* the step, so it is one step stale — the harness re-reads the tip
after the loop) and combines five criteria into one verdict and exit code
(`assets/arm6dof.cpp:483-511`):

```cpp
double ee[3]; endEffector(ee);
double final_dist = vnorm3(efin);
bool reached  = final_dist < REACH_TOL;          // 0.02 m
bool settled  = tail_speed < SETTLE_SPD;         // mean body speed over last 200 steps
bool joints   = tail_sep_max < SEP_TOL;          // hinge anchor1 vs anchor2 stretch < 2e-3 m
bool grounded = min_z > GROUND_TOL;              // nothing tunneled through the floor
bool finite   = !nan_seen && std::isfinite(final_dist);
bool ok = reached && settled && joints && grounded && finite;
printf("RESULT: %s\n", ok ? "PASS" : "FAIL");
return ok ? 0 : 1;
```

The **joint-integrity** check is clever: `hingeSeparation()` reads both hinge anchor read-backs
(`dJointGetHingeAnchor` / `dJointGetHingeAnchor2`) and measures their distance — these coincide only
while the constraint is satisfied, so a growing gap signals a stretched/blown joint
(`assets/arm6dof.cpp:165-171`; `dJointGetHingeAnchor2` doc at `objects.h:2587`).

## What to copy into your own program

**The "kinematic joint + parallel user-mode AMotor servo" actuator.** Generalize as:

1. Build the structural constraint (`dHinge`/`dUniversal`/…) with its own motor **off**
   (`dParamFMax = 0`).
2. Create a `dAMotor` in `dAMotorUser` mode on the **same two bodies**, `NumAxes = 1`, axis set in
   the **global** frame (`rel = 0`).
3. Each step, before `dWorldStep`: re-read the joint's live world axis, set the AMotor axis to it,
   then command `dParamVel = sign * rate` and `dParamFMax = torque_budget`.

**The DLS Cartesian controller** as a drop-in: read tip + error → `v_des = clamp(KP·e)` → build
`J` columns `axis_i × (ee − anchor_i)` from live joint geometry → `dq = clamp(Jᵀ(JJᵀ + λ²I)⁻¹ v_des)`
with adaptive `λ²` from manipulability `sqrt(det(JJᵀ))`. For a 3-D positional task the normal matrix
is 3×3 and invertible in closed form — no linear-algebra dependency needed.

**Calibrate, don't assume, ODE sign/order conventions.** When a sign depends on attach order or
frame, write a tiny gravity-off probe (like `probe_sign*.cpp`) that isolates one DOF, commands a
known input, and reads back the observable — then bake the measured constant into the real program.

**Headless self-grading:** fixed `DT`, run N steps, accumulate min/max instrumentation + a NaN
trip-wire, grade the post-step state against explicit tolerances, print `RESULT: PASS/FAIL`, and
`return ok ? 0 : 1`.

## Gotchas this program handles

- **AMotor sign depends on attach order, not just the axis.** A joint attached to the static world
  (`dJointAttach(am, 0, b)`) drives the *opposite* effective sense from two dynamic bodies
  (`dJointAttach(am, b_i-1, b_i)`). The program measures both cases and applies `+1`/`−1` per joint
  (`assets/arm6dof.cpp:320-322`). ODE rule: body ordering in `dJointAttach` defines the joint's
  positive sense.
- **The hinge's own motor and the AMotor must not fight.** Only one actuator drives each DOF — the
  hinge motor is disabled with `dParamFMax = 0` and the parallel AMotor is the sole driver
  (`assets/arm6dof.cpp:331`, `assets/arm6dof.cpp:339`). ODE rule: `dParamFMax` is the actuation
  budget; `0` = inert.
- **A user-mode AMotor on a moving link needs its axis refreshed every step.** The hinge axis rotates
  in the world as the arm moves; the controller re-sets the AMotor axis from `dJointGetHingeAxis`
  each step (`assets/arm6dof.cpp:260`). ODE rule: in `dAMotorUser` mode ODE does not infer axes —
  you own them (`objects.h:3110`), and `rel=0` axes are in the global frame (`objects.h:2471`).
- **Anchors/axes are global and reset the zero angle — so position bodies first.** All bodies are
  placed before any joint is attached, and anchors are passed in world coordinates
  (`assets/arm6dof.cpp:302-324`). ODE rule: setting a hinge anchor/axis captures the current pose as
  the joint's reference.
- **Set hinge stops hi → lo → hi.** Limits near ±π behave incorrectly if set in the naive order;
  the program sets `HiStop`, then `LoStop`, then `HiStop` again (`assets/arm6dof.cpp:327-329`). ODE
  rule: the FAQ workaround for the wide-limit bug.
- **CFM is positive, not zero.** `CFM = 1e-5` keeps the constraint system off its singularity and
  stable; a hard `0` invites blow-ups (`assets/arm6dof.cpp:46`, `assets/arm6dof.cpp:288`).
- **Grade the post-integration state.** The controller's distance is measured before `dWorldStep`,
  so it is one step stale; the self-check re-reads the tip after the loop to grade the *true* final
  pose (`assets/arm6dof.cpp:483-487`). ODE rule: `dWorldStep` advances state; reads before it reflect
  the previous frame.
- **Double-precision-only world, verified at runtime.** The program checks
  `dCheckConfiguration("ODE_double_precision")` and aborts (exit `2`) if the lib is single precision,
  so the no-macro `ode-config` build is enforced rather than assumed (`assets/arm6dof.cpp:369-374`).
- **An actuated arm must never auto-disable.** `dWorldSetAutoDisableFlag(world, 0)` prevents ODE from
  sleeping a body that the servo still intends to drive (`assets/arm6dof.cpp:291`).

## See also
`references/joints.md` (hinge/AMotor + `dParam*`), `references/examples/ragdoll.md` (ball+AMotor for
limits), `references/components/{joint,body,mass}.md`, `references/foundations/known-issues.md`
(hinge-limit ±π), `assets/probe_sign.cpp` / `assets/probe_sign2.cpp` (sign calibration).
