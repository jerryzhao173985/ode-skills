# ODE pre-flight checklist — the cross-cutting pitfalls

Read this **before** writing any ODE program and again before you claim a sim "works." These are the
bugs that produce *silent garbage*, *invisible pass-through*, *explode-on-step*, or *crash-on-next-loop* —
not compiler errors. The compiler will not catch a single one.

This is a **checklist, not a re-teach**: rows 1–15 are the terse pre-flight form of `SKILL.md` "Hard rules"
1–12 (the rule number is named per row); the full statement lives there. Rows 16–21 are the **unique**
field gotchas that the Hard rules don't cover. Every **Why** cites a real `include/**` header line (or the
single owner page) — grep it to verify.

If you only memorize three: **(1)** a body and a geom are different objects — `dGeomSetBody` welds them;
**(2)** empty the contact joint group every single step; **(3)** match the library's precision (define
neither `dSINGLE` nor `dDOUBLE`).

## The table

| # | Bad | Good | Why (cited) |
|---|-----|------|-------------|
| 1 | `dBodyCreate(world); dCreateBox(space,…);` then step — body falls, geom never moves / never collides | `dGeomID g = dCreateBox(space,…); dGeomSetBody(g, body);` to weld collision shape to dynamics body | Hard rule 1: body ≠ geom; weld with `dGeomSetBody` (`collision.h:105`). Static floor = body 0 or `dCreatePlane`. |
| 2 | `dSpaceCollide(...); dWorldQuickStep(...);` *(no empty)* | append `dJointGroupEmpty(contactgroup);` once per step; group created **once outside** the loop | Hard rule 3: contact joints are valid one step only; un-emptied they accumulate and blow up (`objects.h:1852`). |
| 3 | Building contact joints **after** the world step, or stepping before collision | Per-step order: update geoms → `dSpaceCollide` (callback: `dCollide` + `dJointCreateContact` + `dJointAttach`) → step → `dJointGroupEmpty` | Hard rule 3 pipeline; the callback runs DURING `dSpaceCollide`, before the step. See `collision-and-contacts.md`. |
| 4 | `dWorldQuickStep(w, dt);` — return value ignored | `if (!dWorldQuickStep(w, dt)) { /* alloc failed; retry */ }` | Hard rule 5: `0` = allocation failed, step did NOT advance (`objects.h:384,427`). |
| 5 | Compile app `dSINGLE` but link a `dDOUBLE` lib (or pass `float*` where `dReal` is double) | Define **no** precision macro; `<ode/ode.h>` matches the lib; assert `dCheckConfiguration("ODE_double_precision")` | Hard rule 7: precision mismatch silently corrupts every `dReal`/`dVector3` — the #1 integration bug (`common.h:573`). |
| 6 | Expecting bodies to fall with no `dWorldSetGravity` | `dWorldSetGravity(world, 0, 0, -9.81);` after `dWorldCreate` (+z up) | Default gravity is `(0,0,0)`; bodies float until you set it (`objects.h:93`). |
| 7 | Applying a force once and expecting it to persist | Re-apply forces **every step** before the world step | `dBodyAddForce`/`dBodyAddTorque` accumulate into a per-body buffer (`objects.h:1165`) that ODE clears after each step; `dBodyGetForce` reads it (`objects.h:1219`). |
| 8 | `dCreateBox(space, 1, 1, 1)` thinking args are half-extents | Pass **full** side lengths; reference point is the center | Hard rule 11: box args are full side lengths, not half (`collision.h:1001`; comment `collision.h:986`). |
| 9 | `dCreateCapsule(space, r, total_len)` expecting end-to-end length | Pass the **cylinder** length only; total height ≈ `length + 2*radius` | Hard rule 11: the `length` arg is the central cylinder; hemispherical caps are added on top (`collision.h:1050`). |
| 10 | `dRFromAxisAndAngle(R, 0,0,1, 90)` expecting 90° | `dRFromAxisAndAngle(R, 0,0,1, M_PI*0.5)` — **all** ODE angle args are radians | Hard rule 10: physics angles are radians (`rotation.h:36`). Asymmetry: drawstuff `hpr` is **degrees** (`drawstuff.h:165`, `dsSetViewpoint` `drawstuff.h:169`). |
| 11 | `dQuaternion q = {x, y, z, w};` (GLM/Bullet/Eigen order) | `q[dQUE_R]=w; q[dQUE_I]=x; q[dQUE_J]=y; q[dQUE_K]=z;` — real part **first**; identity `(1,0,0,0)` | Hard rule 10: ODE quaternion order is `[w,x,y,z]`; the `[x,y,z,w]` graphics convention is wrong (enum `common.h:256-262`). |
| 12 | `const dReal* p = dBodyGetPosition(b); free((void*)p);` or caching `p` across steps | `dVector3 p; dBodyCopyPosition(b, p);` — `p` is caller-owned | Hard rule 12: `dBodyGetPosition` returns an internal pointer (`objects.h:1088`); free/cache corrupts or reads stale — use `dBodyCopyPosition` (`objects.h:1098`). |
| 13 | `dCollide(o1,o2,N,contact,sizeof(dContactGeom));` where `contact` is `dContact[]` | `dCollide(o1,o2,N,&contact[0].geom,sizeof(dContact));` — address of the first `.geom` with the `dContact` stride | Hard rule 4: `dContactGeom` is the `.geom` member of `dContact`, not a separate allocation (`collision.h:792`; `contact.h:99-103`). |
| 14 | `dContact c; dCollide(o1,o2,8,&c.geom,sizeof(dContact));` — 8 contacts into **one** struct | `dContact c[8]; dCollide(o1,o2,8,&c[0].geom,sizeof(dContact));` | Hard rule 4: the 3rd arg is array capacity; one struct + N requested writes past it → crash on the next loop (`collision.h:792`). |
| 15 | Stepping with a per-frame wall-clock `dt` | Fixed `dt`; accumulate real time: `cache += elapsed; while (cache >= h) { step(h); cache -= h; }` | Hard rule 5: ODE assumes a fixed step; varying `dt` shifts the resting depth each step so resting bodies jitter and gain energy (`objects.h:384,427`). |
| 16 | Firing a fast/thin body at thin geometry with a coarse timestep and assuming it collides | Ensure `max_velocity * timestep < smallest shape thickness`; shrink the step, thicken static geometry, use `dCreatePlane` (`collision.h:1045`), cap velocity, or hand-roll ray-cast CCD with `dCreateRay` (`collision.h:1066`) | **ODE has no built-in CCD**: a body moving farther than the shape is thick in one step tunnels straight through without generating a contact (ODE wiki). |
| 17 | `b = dBodyCreate(world); dBodySetPosition(b,0,0,1);` then step with no mass; or `dBodySetMass(b, 5.0)` | `dMass m; dMassSetBoxTotal(&m, MASS, sx,sy,sz); dBodySetMass(b, &m);` — real positive mass via a `dMass` (2nd arg is `const dMass*`) | No-mass/zero-mass body is garbage/unstable; there is no scalar setter — use a `*Total` setter (`mass.h:67`) or `dMassAdjust`, then `dBodySetMass` (`objects.h:1153`). Density ~1 kg/m³ is lighter than air (ODE wiki). |
| 18 | Emitting removed APIs: `dWorldStepFast1`, `dCreateCCylinder`, `dRayCreate`, `dInitODE()` | `dWorldQuickStep`/`dWorldStep`; `dCreateCapsule`; `dCreateRay(space, length)`; `dInitODE2(0)` (+`dCloseODE()`) | `dWorldStepFast1` removed; `dCreateCCylinder` survives only as a back-compat macro (`collision.h:1056`); `dRayCreate` isn't a symbol — use `dCreateRay` (`collision.h:1066`); `dInitODE` is obsolete (`odeinit.h:83,95`), use `dInitODE2` (`odeinit.h:119`). |
| 19 | PD servo via velocity motor with derivative gain ≥ 1: `dJointSetHingeParam(j, dParamVel, KP*err + KD*derr)` with `KD≥1` — chatters/diverges | Keep `KD < 1` (re-issue `dParamVel` each step with `dParamFMax > 0`); for stiff tracking use a torque loop via `dBodyAddTorque` | `dParamVel` is the *desired motor velocity* assigned each step (`objects.h:1642`), `dParamFMax ≥ 0` or the motor is off (`objects.h:1644-1647`). The `KD<1` stability bound is derived/observed — see **cpp-patterns.md §4** (velocity-servo trap) for the full derivation. |
| 20 | Reading `contact.surface.motion1` (or its sign) assuming the callback's `o1`/`o2` match your geom-creation order (e.g. always push "o1 forward" on a conveyor) | Read `o.geom.g1/g2`, derive the sign from the actual geom identities, then set `motion1`/`fdir1` | Broadphase doesn't promise `o1`/`o2` in creation order — it can swap per pair/frame. Callback is `dNearCallback(data, o1, o2)` (`collision_space.h:49`); `dCollide` fills `g1,g2` (`contact.h:92`); `motion1` is conveyor velocity along friction dir 1, honored only with `dContactMotion1` (`contact.h:40,69`). See `collision-and-contacts.md`. |
| 21 | `contact.surface.mode \|= dContactApprox1_N;` (`0x4000`) expecting full rolling/pyramid friction — roller creeps, never settles | Use `dContactApprox1` (`0x7000`) for the full friction-pyramid across all directions (or OR only the per-direction bits you need) | `_N` (`0x4000`) approximates only one friction component, so a rolling body keeps a tangential bias; `dContactApprox1` = `0x7000` = `_1\|_2\|_N` is the full mask almost all samples set (`contact.h:48-51`). See `collision-and-contacts.md`. |
| 22 | A motorized robot settles, then ignores all motor commands (moves exactly 0.000 m) | `dBodySetAutoDisableFlag(body, 0)` on every **actuated** body before stepping | World auto-disable put the resting body to **sleep**, and a sleeping body ignores its joint motors — the robot freezes (`objects.h:986`). Auto-disable is for passive props, not actuated parts. See `building-robots.md` §8. |

## Two more gotchas that bite at setup time (init / static geom)

Same class of silent failure, kept out of the table to keep it skimmable:

- **Collision data never allocated.** `dInitODE2(0)` alone is not enough to use collision on a thread —
  call `dAllocateODEDataForThread(dAllocateMaskAll)` (or at least `dAllocateFlagCollisionData`) on *every*
  thread that touches ODE, including the drawstuff `start()` callback (`odeinit.h:177`; flags
  `odeinit.h:149,151`).
- **Surface field set without its mode bit.** `mode` and `mu` are always read, but
  `bounce`/`soft_cfm`/`slip`/`mu2` are honored only when their bit is in `surface.mode`. Set
  `contact.surface.mode = dContactBounce;` *before* `contact.surface.bounce = 0.5;` (`contact.h:37,57`).
  Full bitmask in `collision-and-contacts.md`.
- **Non-placeable geoms.** Calling `dGeomSetPosition` (`collision.h:131`) / `dGeomSetRotation`
  (`collision.h:146`) on a plane is a debug-build runtime error (`collision.h:98`). Planes are positioned
  by their a,b,c,d equation: `dCreatePlane(space, 0,0,1, 5)` (`collision.h:1045`).

## Stability knobs (when it jitters or explodes rather than crashes)

- Keep `ERP ≈ 0.2` (range `0.1–0.8`, `dWorldSetERP` `objects.h:110`) and `CFM` a small **positive** value
  (e.g. `1e-5`, `dWorldSetCFM` `objects.h:127`). `ERP=1` can't fully correct error and can oscillate;
  negative CFM is unstable (ODE wiki).
- Pick the solver deliberately (Hard rule 6): `dWorldStep` is the accurate big-matrix LCP solver but
  O(m²) memory and slow; `dWorldQuickStep` is the fast iterative SOR solver, less accurate near singular
  configs. Stacks / many-contact scenes want **QuickStep + auto-disable** (`objects.h:384,427`).
- Keep lengths and masses around `0.1–10` (ideally `0.1–1.0` for single-precision factorizer precision)
  and scale gravity and the timestep to match (ODE wiki).
- World auto-disable/damping setters only **seed defaults at body-creation time** (Hard rule 9): setting
  them after bodies exist doesn't change existing bodies — set world defaults first, or set per-body via
  the `dBody*` equivalents (`objects.h:662`).

## Joint gotchas (silent no-ops)

- **Unattached joint = no effect.** `dJointAttach(j, b1, b2)` right after create (`objects.h:1873`).
- **Motor with no force.** A motor does nothing unless `dParamFMax > 0`; setting only `dParamVel`
  produces no force (`objects.h:1644-1647`).
- **Second-axis params need the suffix.** `dJointSetHinge2Param(j, dParamVel, v)` (`objects.h:2110`) only
  touches axis 1; use `dParamVel2` for axis 2 (steering vs wheel-spin).
- **`dInfinity`, not a big number.** For infinite limits/forces use the `dInfinity` macro
  (`odeconfig.h:153`); a hardcoded big number or `HUGE_VAL` can overflow a float build.
