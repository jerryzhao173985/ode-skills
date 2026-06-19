# ODE Demo Index — "find a worked example for feature X"

> Source of truth: ode/demo/demo_*.c and demo_*.cpp. Every rule cites real ODE source by file:line; headers win over the wiki on conflict. C symbols start with 'd'; do not invent symbols.

## Mental model

- Every demo is a self-contained drawstuff program: `#include <ode/ode.h>` + `#include <drawstuff/drawstuff.h>`, fill a `dsFunctions` struct, and call `dsSimulationLoop` from `main`; per-frame physics lives in the `step` callback (`simLoop`). Copy a demo to get the harness shape. (ode/demo/demo_chain1.c:25-27)
- The two canonical starting points: `demo_chain1.c` is the minimal **C-API** example ("exercise the C interface"); `demo_chain2.cpp` is its **C++-API** twin ("exercise the C++ interface"). (ode/demo/demo_chain1.c:23)
- Demos guard draw-call names behind an `#ifdef dDOUBLE` block (`#define dsDrawBox dsDrawBoxD ...`) so one source compiles in single and double precision — copy that block when porting. (ode/demo/demo_chain1.c:35-41)
- Several "demos" are actually headless/regression tests, not visual toys: `demo_ode`, `demo_I`, `demo_step`, `demo_collision`, `demo_space`, `demo_joints` run numeric checks. (ode/demo/demo_joints.cpp:24-25)
- Map each demo to the ONE feature it best teaches; pick the demo whose source actually calls the matching API, not the one with the catchiest name.

## When to use

- You need a worked, compilable example of a specific ODE feature (a joint type, a collider, a space type, a solver, a body flag) and want to copy a known-good pattern instead of guessing from headers.
- You are porting/learning the drawstuff harness shape (`dsFunctions` + `dsSimulationLoop` + `simLoop`).
- You need to know which demo to read to see an API used in context, or which demo doubles as a regression/correctness test.

## Public API (C)

This index documents the demo *files*, not new C symbols (the demos only call symbols defined in the headers indexed elsewhere). The table below maps each demo to the single feature it best demonstrates, grouped by theme, cited to its source file.

### Basics / harness

| Demo | Teaches | Source |
| --- | --- | --- |
| demo_chain1.c | Minimal **pure-C** entry example: a falling chain of capsule bodies linked by ball joints with a per-frame collide+step loop. Best first demo for the C API + drawstuff harness. | ode/demo/demo_chain1.c:23 |
| demo_chain2.cpp | C++-interface twin of chain1: `NUM=10` boxes chained using the C++ wrapper classes. Best first demo for the C++ object API. | ode/demo/demo_chain2.cpp:23 |
| demo_boxstack.cpp | Interactive spawning/stacking of every primitive geom on keypress (b=box, s=sphere, c=capsule, y=cylinder, v=convex, x=composite) plus toggles for AABBs (a), contact points (t), save state.dif (1). The go-to sampler of geom types + contact handling. | ode/demo/demo_boxstack.cpp:185-200 |
| demo_ode.cpp | Headless math/library self-test (no drawstuff): RNG, infinity, cross product, `dNormalize3`, `dPlaneSpace`, matrix/rotation helpers. Unit-test harness for ODE's math primitives. | ode/demo/demo_ode.cpp:129-248 |
| demo_I.cpp | Validates inertia tensors: random particle bodies on ball joints yield an effective inertia matrix assigned to a test body; equal torque should give equal motion. Teaches `dMass`/inertia correctness. | ode/demo/demo_I.cpp:23-34 |

### Joints

| Demo | Teaches | Source |
| --- | --- | --- |
| demo_hinge.cpp | Simplest single hinge: two boxes joined by `dJointCreateHinge` with `dJointSetHingeAnchor`/`Axis`, with an occasional perturbation to test constraint robustness. | ode/demo/demo_hinge.cpp:154-157 |
| demo_slider.cpp | Simplest slider (prismatic): two boxes joined by `dJointCreateSlider` with `dJointSetSliderAxis`, plus an occasional-error perturbation. | ode/demo/demo_slider.cpp:161-163 |
| demo_dball.cpp | Double-ball (distance) joint: `dJointCreateDBall` with `dJointSetDBallAnchor1`/`Anchor2` holds two anchors at fixed distance; also shows `dWorldSetDamping`. | ode/demo/demo_dball.cpp:70-78 |
| demo_dhinge.cpp | Double-hinge joint: `dJointCreateDHinge` with `dJointSetDHingeAxis`/`Anchor1`/`Anchor2` (combined with a DBall); `applyForce` flag perturbs the bodies. | ode/demo/demo_dhinge.cpp:72-86 |
| demo_jointPR.cpp | The PR (prismatic + rotoide) joint: prismatic `axisP` drawn red, rotoide `axisR` green. Reference for `dJointCreatePR`. | ode/demo/demo_jointPR.cpp:23-27 |
| demo_jointPU.cpp | The PU joint (Universal + Slider): a universal joint with a slider between anchor and body1, axes/anchor color-coded. Reference for `dJointCreatePU`. | ode/demo/demo_jointPU.cpp:22-36 |
| demo_piston.cpp | The Piston joint: one body slides along an axis relative to another and is free to rotate about it; prismatic axis red, rotoide orange, anchor a light-blue ball; `-h` lists options. Reference for `dJointCreatePiston`. | ode/demo/demo_piston.cpp:25-39 |
| demo_plane2d.cpp | Constrains N bodies to a 2D plane with `dJointCreatePlane2D`, driven via `dJointSetPlane2DXParam`/`YParam` (`dParamVel`/`dParamFMax`). Reference for the Plane2D constraint. | ode/demo/demo_plane2d.cpp:22-23 |
| demo_motor.cpp | Linear and angular motor joints: `dJointCreateLMotor`/`dJointCreateAMotor` driven via `dParamVel`/`dParamVel2`/`dParamVel3` to translate and spin bodies. Reference for motor joints. | ode/demo/demo_motor.cpp:71-83 |
| demo_transmission.cpp | The Transmission joint (gear coupling) in parallel-axes vs intersecting-axes modes via `dJointSetTransmissionMode`/`Ratio`/`Anchor`/`Axis`, with backlash and ratio controls. Reference for `dJointCreateTransmission`. | ode/demo/demo_transmission.cpp:64-93 |
| demo_feedback.cpp | Breaking joints via force feedback: `dJointSetFeedback` gives each hinge a `dJointFeedback`; when measured force exceeds a threshold the joint is destroyed ("SNAP!"). Reference for reading joint reaction forces. | ode/demo/demo_feedback.cpp:23 |
| demo_joints.cpp | Automated regression test over all joint types, numbered "xxyy" (xx=type, yy=sub-test); prints scaled error (<1 good). **Must run with the double-precision build.** The comprehensive joint-correctness harness. | ode/demo/demo_joints.cpp:23-44 |

### Collision shapes / colliders

| Demo | Teaches | Source |
| --- | --- | --- |
| demo_collision.cpp | Test harness for the low-level `dCollide` colliders: no args runs all collision tests on random data; a test number runs that one interactively (spacebar re-randomizes). Direct geom-pair collision testing. | ode/demo/demo_collision.cpp:23-28 |
| demo_convex.cpp | Convex geometry: a pile of convex hulls (Halton point set, `dCreateConvex`) settling under gravity. Reference for building `dGeomConvex` from planes+points arrays. | ode/demo/demo_convex.cpp:23 |
| demo_cyl.cpp | The non-capped flat-ended cylinder geom (`dCreateCylinder`) rolling on a trimesh world. Cylinder collider vs trimesh. | ode/demo/demo_cyl.cpp:23 |
| demo_cylvssphere.cpp | Focused test of the cylinder-vs-sphere collider: one cylinder body and one sphere body interacting. | ode/demo/demo_cylvssphere.cpp:23 |
| demo_basket.cpp | Sphere-vs-trimesh collision: a basketball (`dCreateSphere`) bouncing inside a static triangle-mesh world; spacebar resets. Sphere/trimesh collider + loading a mesh from a `*_geom.h` header. | ode/demo/demo_basket.cpp:23-26 |
| demo_trimesh.cpp | General triangle-mesh collision test (by Erwin de Vries): dynamic bodies (incl. a convex defined by planes) dropped onto/around a triangle mesh. | ode/demo/demo_trimesh.cpp:22 |
| demo_moving_trimesh.cpp | Moving (dynamic) triangle-mesh bodies: `dGeomTriMeshDataBuildSingle` + `dCreateTriMesh` for dynamic mesh objects colliding with each other. Reference for non-static trimesh geoms. | ode/demo/demo_moving_trimesh.cpp:308-313 |
| demo_moving_convex.cpp | Many moving convex hulls (`dCreateConvex` from `convex_bunny_geom`) colliding via a `dSpaceCollide` nearCallback. Space/near-callback pipeline for convex-vs-convex at scale. | ode/demo/demo_moving_convex.cpp:75-98 |
| demo_heightfield.cpp | Heightfield terrain: builds a `dHeightfieldDataID` via `dGeomHeightfieldDataBuildCallback` + `dGeomHeightfieldDataSetBounds`, creates the geom with `dCreateHeightfield`, drops bodies (incl. the bunny mesh). Reference for terrain collision. | ode/demo/demo_heightfield.cpp:665-676 |
| demo_space.cpp | Verifies the collision-space broadphase: random boxes, intersections computed into an n² array, then checks the space reports every correct collision exactly once and no incorrect ones. The space-correctness test. | ode/demo/demo_space.cpp:23-31 |
| demo_space_stress.cpp | Stress-tests broadphase space types with up to `NUM=10000` objects, selectable between `dQuadTreeSpaceCreate`, `dHashSpaceCreate`, `dSweepAndPruneSpaceCreate`, `dSimpleSpaceCreate`. Comparing space implementations at scale. | ode/demo/demo_space_stress.cpp:412-428 |

### Vehicles

| Demo | Teaches | Source |
| --- | --- | --- |
| demo_buggy.cpp | A car with suspension: chassis body + 3 wheels joined by hinge2 joints; also demonstrates geom groups. The vehicle / hinge2-suspension reference. | ode/demo/demo_buggy.cpp:23-25 |
| demo_tracks.cpp | Builds an inclined parametric track (box geoms + a `dCreateTriMesh` ground from `dGeomTriMeshDataBuildSimple`) and rolls two spheres down it; `TEST_CASE` selects geometry. Custom-geometry construction + sphere-on-trimesh rolling. [VERIFY: no top comment; feature inferred from construction code] | ode/demo/demo_tracks.cpp:240-244 |

### Stability / friction

| Demo | Teaches | Source |
| --- | --- | --- |
| demo_cards.cpp | Builds a house of cards (`3*levels²+levels` boxes) to stress stacking stability; +/- change the number of levels. Stable stacking of many thin boxes. | ode/demo/demo_cards.cpp:73-80 |
| demo_friction.cpp | Validates the Coulomb friction approximation: a 10×10 array of boxes with mass `(i+1)*MASS` and push force `(j+1)*FORCE`, tuned so a triangular half slides. The contact `mu` / friction model. | ode/demo/demo_friction.cpp:23-43 |
| demo_gyro2.cpp | Angular (rolling) friction: spheres with rolling friction released on ramps of different pitch. Contact rolling-friction setup. [VERIFY: shares an identical top comment with demo_rfriction.cpp] | ode/demo/demo_gyro2.cpp:22-26 |
| demo_rfriction.cpp | Rolling-friction (variant of gyro2): spheres released down ramps of varying pitch under a configurable `GRAVITY`. Rolling/angular friction tuning. [VERIFY: shares an identical top comment with demo_gyro2.cpp] | ode/demo/demo_rfriction.cpp:22-26 |

### Advanced / solver / body modes

| Demo | Teaches | Source |
| --- | --- | --- |
| demo_crash.cpp | Compares QuickStep and (old) StepFast solvers on a large stack/wall scene: `dWorldSetQuickStepNumIterations` + `dWorldQuickStep` vs `dWorldStep`. Choosing the iterative solver + tuning iterations for large scenes. | ode/demo/demo_crash.cpp:23-24 |
| demo_step.cpp | Compares fast (QuickStep) vs slow (WorldStep) integrators on various NUM-body/NUMJ-joint systems, reports max error (needs `COMPARE_METHODS` defined in step.cpp). Solver accuracy comparison. | ode/demo/demo_step.cpp:22-26 |
| demo_gyroscopic.cpp | The explicit gyroscopic torque term: two spinning tops, one with `setGyroscopicMode(false)`, to see `dBodySetGyroscopicMode`'s effect on precession. | ode/demo/demo_gyroscopic.cpp:240 |
| demo_kinematic.cpp | Kinematic bodies: a body driven by `setKinematic()` (`dBodySetKinematic`) pushes dynamic bodies (a "matraca") — kinematic objects affect dynamics without being affected. Kinematic vs dynamic. | ode/demo/demo_kinematic.cpp:214 |
| demo_motion.cpp | Surface motion / conveyor effect: uses `dContactMotionN` in the contact surface to drive a lifting platform. Reference for contact surface motion velocities. | ode/demo/demo_motion.cpp:22-24 |

## Key rules

- To learn the world+body+joint+collision loop, start from `demo_chain1.c` (C) or `demo_chain2.cpp` (C++); they are the explicitly labelled "exercise the C/C++ interface" entry examples. (ode/demo/demo_chain1.c:23)
- The drawstuff harness is mandatory for visual demos: include both `ode/ode.h` and `drawstuff/drawstuff.h`, then drive `dsSimulationLoop` with a `dsFunctions` whose `step` callback runs collide+step. (ode/demo/demo_chain1.c:25-27)
- Copy the `#ifdef dDOUBLE` draw-name remapping block verbatim when porting a demo so it builds in both precisions. (ode/demo/demo_chain1.c:35-41)
- For the broadest single geom-type sampler (box/sphere/capsule/cylinder/convex/composite) plus AABB and contact-point visualization, read `demo_boxstack.cpp`; its keys are enumerated in its own help text. (ode/demo/demo_boxstack.cpp:185-200)
- `demo_joints.cpp` is a numeric regression test, not a visual demo, and only passes under the double-precision build. (ode/demo/demo_joints.cpp:24-25)
- To pick or stress-test a broadphase space, `demo_space_stress.cpp` switches between `dQuadTreeSpaceCreate`, `dHashSpaceCreate`, `dSweepAndPruneSpaceCreate`, and `dSimpleSpaceCreate` from the command line (defaulting to SAP). (ode/demo/demo_space_stress.cpp:412-428)
- For a vehicle with suspension, `demo_buggy.cpp` is the canonical chassis-plus-hinge2-wheels reference. (ode/demo/demo_buggy.cpp:23-25)
- For reading joint reaction forces and breaking joints over a threshold, attach a `dJointFeedback` as in `demo_feedback.cpp`. (ode/demo/demo_feedback.cpp:23)
- The transmission (gear) joint is demonstrated only in `demo_transmission.cpp`, which calls `dJointSetTransmissionMode` directly. (ode/demo/demo_transmission.cpp:64-93)

## Common mistakes

| Bad | Good | Why |
| --- | --- | --- |
| Treating `demo_ode.cpp` as a graphical demo and looking for a window/keys | Read it as a headless math self-test (RNG, `dNormalize3`, `dPlaneSpace`, matrix/rotation helpers) | It has no drawstuff include and runs numeric checks, not a render loop (ode/demo/demo_ode.cpp:22-24) |
| Running `demo_joints` in single precision and trusting the result | Build double precision before trusting its joint-correctness output | The harness "perform tests on all the joint types" requires the double build to pass (ode/demo/demo_joints.cpp:24-25) |
| Inventing extra boxstack keypresses (e.g. for trimesh) | Use only the keys the demo prints (b/s/c/y/v/x, space, d/e/p/a/t/r/1/f) | The full key list is enumerated in the demo's own help text (ode/demo/demo_boxstack.cpp:185-200) |
| Copying one demo's draw calls without the precision guard | Copy the `#ifdef dDOUBLE` `dsDrawBoxD` remapping block too | Without it the source won't compile in both single and double precision (ode/demo/demo_chain1.c:35-41) |
| Attributing the Transmission joint to a demo that just mentions gears | Cite `demo_transmission.cpp` only because it actually calls `dJointSetTransmissionMode` | A demo teaches a feature only if its source calls the matching API (ode/demo/demo_transmission.cpp:64) |

## Named constants

This index introduces no ODE named constants; the per-demo tuning macros (e.g. `NUM`, `MASS`, `FORCE`, `GRAVITY`, `TEST_CASE`) are demo-local `#define`s, not public API, and are described inline in the tables above with their source cites.

## Things to never invent

- Do not claim a demo teaches a joint/feature unless its source actually calls the matching API (e.g. only attribute Transmission to `demo_transmission` because `dJointSetTransmissionMode` is at demo_transmission.cpp:64).
- Do not invent demo command-line flags or keypresses beyond those printed in the demo's own `start()`/`printf` help (e.g. boxstack keys are enumerated at demo_boxstack.cpp:185-200).
- Do not assume every demo opens a graphics window — `demo_ode.cpp` has no drawstuff include and is a headless math test (demo_ode.cpp:22-24).

## How demos are built / run

- Each demo is one translation unit (`demo_<name>.c` / `.cpp`) under `ode/demo/`; they are compiled and linked against the ODE library and the bundled drawstuff renderer by the project's build (the demo targets are wired up by ODE's build system, e.g. its premake/CMake config — [VERIFY] exact build-file target list not in this inventory).
- A visual demo links drawstuff and runs by calling `dsSimulationLoop(argc, argv, w, h, &fn)` from `main`, where `fn` is a `dsFunctions` with `start`/`step`/`stop`/`command` callbacks; the window opens and the `step` callback advances the world each frame (ode/demo/demo_chain1.c:25-27).
- Headless/test demos (`demo_ode`, `demo_I`, `demo_step`, `demo_collision`, `demo_space`, `demo_joints`) run from the console and print numeric pass/fail or error magnitudes instead of opening a window; `demo_joints` additionally requires the double-precision build (ode/demo/demo_joints.cpp:24-25).
- Many demos accept command-line args or runtime keypresses (e.g. `demo_space_stress` takes `quad`/`hash`/`sap`/`simple`; `demo_piston -h` lists options; `demo_boxstack` spawns geoms on keypress) — always read the demo's own help/`printf` for the authoritative list (ode/demo/demo_space_stress.cpp:412-428).
