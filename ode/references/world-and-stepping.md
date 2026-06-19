# World & Stepping (library lifecycle + the world container + integrators)

> Source of truth: include/ode/odeinit.h, include/ode/error.h, include/ode/memory.h, include/ode/objects.h, include/ode/common.h, include/ode/odeconfig.h. Every rule cites real ODE source by file:line; headers win over the wiki on conflict. C symbols start with 'd'; do not invent symbols.

## Mental model

- **Init once, allocate per-thread, close once.** `dInitODE2()` boots the library globally; every thread that touches ODE must then call `dAllocateODEDataForThread(...)`; `dCloseODE()` tears it all down. Nothing else (no world, no body, no collision) may run before init.
- **The world is a pure dynamics container** — it holds bodies and joints and integrates them. Geometry/collision lives in *spaces* (separate subsystem); the world only knows the contact joints you feed it.
- **Default gravity is (0,0,0).** Bodies float until you call `dWorldSetGravity`. Earth (+z up) is `(0,0,-9.81)`.
- **Two integrators, one per step.** `dWorldStep` = accurate big-matrix solver (O(m²) memory). `dWorldQuickStep` = fast iterative SOR solver (O(m·N)). Pick one per tick; both return an `int` status you must check.
- **World "set" knobs split into two kinds.** Solver-global (ERP, CFM, QuickStep iterations/W/contact caps) affect the *whole solve every step*. Auto-disable / damping / max-angular-speed are **defaults copied onto each body at creation time** — they are not live retroactive controls.
- **Precision is an ABI contract.** `dReal` is `float` (`dSINGLE`) or `double` (`dDOUBLE`); your app must define the same one the library was built with.

## When to use

Use this reference when you: boot or shut down ODE; create/destroy the world or stash user data on it; set gravity; choose and call an integrator; tune the global solver (ERP, CFM, QuickStep iterations/over-relaxation, contact correcting-velocity/surface-layer); seed world-level auto-disable or damping defaults; or install custom error/message/memory hooks. For bodies, joints, geoms, spaces, and mass, see the sibling references.

## Public API (C)

### Library lifecycle (include/ode/odeinit.h)

| Symbol | Signature | Source | Purpose |
| --- | --- | --- | --- |
| dInitODE | `void dInitODE(void)` | include/ode/odeinit.h:95 | Obsolete init; equals `dInitODE2(0)` then `dAllocateODEDataForThread(dAllocateMaskAll)`. Prefer `dInitODE2`. |
| dInitODE2 | `int dInitODE2(unsigned int uiInitFlags)` | include/ode/odeinit.h:119 | Initialize ODE before first use; pass `0` or a `dInitODEFlags` bitmask; returns nonzero on success, zero on failure. |
| dAllocateODEDataForThread | `int dAllocateODEDataForThread(unsigned int uiAllocateFlags)` | include/ode/odeinit.h:177 | Allocate thread-local data so the calling thread may use ODE; pass a `dAllocateODEDataFlags` bitmask; nonzero on success. |
| dCleanupODEAllDataForThread | `void dCleanupODEAllDataForThread()` | include/ode/odeinit.h:204 | Free the current thread's ODE data; only legal when initialized with `dInitFlagManualThreadCleanup`. |
| dCloseODE | `void dCloseODE(void)` | include/ode/odeinit.h:227 | Shut ODE down, releasing all resources including every thread's local data; pairs with `dInitODE2`. |

### Build configuration query (include/ode/common.h)

| Symbol | Signature | Source | Purpose |
| --- | --- | --- | --- |
| dGetConfiguration | `const char* dGetConfiguration(void)` | include/ode/common.h:563 | Return the build configuration as a space-separated token string (precision + feature extensions). |
| dCheckConfiguration | `int dCheckConfiguration(const char* token)` | include/ode/common.h:573 | Return 1 if the exact (case-sensitive) token is present in the configuration string, else 0. |

### Error / message hooks (include/ode/error.h)

| Symbol | Signature | Source | Purpose |
| --- | --- | --- | --- |
| dSetErrorHandler | `void dSetErrorHandler(dMessageFunction *fn)` | include/ode/error.h:42 | Install a custom fatal-error handler; pass `0` to restore the default. |
| dSetDebugHandler | `void dSetDebugHandler(dMessageFunction *fn)` | include/ode/error.h:43 | Install a custom debug-trap handler; pass `0` to restore the default. |
| dSetMessageHandler | `void dSetMessageHandler(dMessageFunction *fn)` | include/ode/error.h:44 | Install a custom warning/message handler; pass `0` to restore the default. |
| dGetErrorHandler | `dMessageFunction *dGetErrorHandler(void)` | include/ode/error.h:49 | Return current error handler, or `0` if the default is in place. |
| dGetDebugHandler | `dMessageFunction *dGetDebugHandler(void)` | include/ode/error.h:50 | Return current debug handler, or `0` if the default is in place. |
| dGetMessageHandler | `dMessageFunction *dGetMessageHandler(void)` | include/ode/error.h:51 | Return current message handler, or `0` if the default is in place. |
| dError | `void ODE_NORETURN dError(int num, const char *msg, ...)` | include/ode/error.h:54 | Raise a fatal error; does not return (noreturn). |
| dDebug | `void ODE_NORETURN dDebug(int num, const char *msg, ...)` | include/ode/error.h:55 | Raise a debug trap; does not return (noreturn). |
| dMessage | `void dMessage(int num, const char *msg, ...)` | include/ode/error.h:56 | Emit a non-fatal message/warning and return. |

### Memory hooks (include/ode/memory.h)

| Symbol | Signature | Source | Purpose |
| --- | --- | --- | --- |
| dSetAllocHandler | `void dSetAllocHandler(dAllocFunction *fn)` | include/ode/memory.h:41 | Install a custom allocation function; pass `0` to restore the default. |
| dSetReallocHandler | `void dSetReallocHandler(dReallocFunction *fn)` | include/ode/memory.h:42 | Install a custom reallocation function; pass `0` to restore the default. |
| dSetFreeHandler | `void dSetFreeHandler(dFreeFunction *fn)` | include/ode/memory.h:43 | Install a custom free function; pass `0` to restore the default. |
| dGetAllocHandler | `dAllocFunction *dGetAllocHandler(void)` | include/ode/memory.h:46 | Return the current allocation function. |
| dGetReallocHandler | `dReallocFunction *dGetReallocHandler(void)` | include/ode/memory.h:47 | Return the current reallocation function. |
| dGetFreeHandler | `dFreeFunction *dGetFreeHandler(void)` | include/ode/memory.h:48 | Return the current free function. |
| dAlloc | `void * dAlloc(dsizeint size)` | include/ode/memory.h:51 | Allocate memory via the current alloc handler. |
| dRealloc | `void * dRealloc(void *ptr, dsizeint oldsize, dsizeint newsize)` | include/ode/memory.h:52 | Reallocate via the current realloc handler; requires old and new sizes. |
| dFree | `void dFree(void *ptr, dsizeint size)` | include/ode/memory.h:53 | Free via the current free handler; requires the original size. |

### World container (include/ode/objects.h)

| Symbol | Signature | Source | Purpose |
| --- | --- | --- | --- |
| dWorldCreate | `dWorldID dWorldCreate(void)` | include/ode/objects.h:52 | Create a new empty world (container for bodies and joints) and return its ID. |
| dWorldDestroy | `void dWorldDestroy(dWorldID world)` | include/ode/objects.h:64 | Destroy a world and everything in it (all bodies, all non-grouped joints). |
| dWorldSetData | `void dWorldSetData(dWorldID world, void* data)` | include/ode/objects.h:73 | Set the world's user-data pointer. |
| dWorldGetData | `void* dWorldGetData(dWorldID world)` | include/ode/objects.h:82 | Get the world's user-data pointer. |
| dWorldSetGravity | `void dWorldSetGravity(dWorldID, dReal x, dReal y, dReal z)` | include/ode/objects.h:93 | Set the global gravity vector in m/s²; default is (0,0,0). |
| dWorldGetGravity | `void dWorldGetGravity(dWorldID, dVector3 gravity)` | include/ode/objects.h:100 | Write the world's gravity vector into the caller's `dVector3`. |

### Global solver tuning (include/ode/objects.h)

| Symbol | Signature | Source | Purpose |
| --- | --- | --- | --- |
| dWorldSetERP | `void dWorldSetERP(dWorldID, dReal erp)` | include/ode/objects.h:110 | Set global error-reduction parameter; typical 0.1–0.8, default 0.2. |
| dWorldGetERP | `dReal dWorldGetERP(dWorldID)` | include/ode/objects.h:117 | Get the global error-reduction parameter. |
| dWorldSetCFM | `void dWorldSetCFM(dWorldID, dReal cfm)` | include/ode/objects.h:127 | Set global constraint-force-mixing (softness); typical 1e-9..1, default 1e-5 single / 1e-10 double. |
| dWorldGetCFM | `dReal dWorldGetCFM(dWorldID)` | include/ode/objects.h:134 | Get the global constraint-force-mixing value. |
| dWorldSetContactMaxCorrectingVel | `void dWorldSetContactMaxCorrectingVel(dWorldID, dReal vel)` | include/ode/objects.h:603 | Cap the correcting velocity contacts may generate; default infinity. Lower to stop deep-object popping. |
| dWorldGetContactMaxCorrectingVel | `dReal dWorldGetContactMaxCorrectingVel(dWorldID)` | include/ode/objects.h:610 | Get the contact max correcting velocity. |
| dWorldSetContactSurfaceLayer | `void dWorldSetContactSurfaceLayer(dWorldID, dReal depth)` | include/ode/objects.h:623 | Set surface-layer depth contacts may sink before resting; default 0, use ~0.001 to reduce jitter. |
| dWorldGetContactSurfaceLayer | `dReal dWorldGetContactSurfaceLayer(dWorldID)` | include/ode/objects.h:630 | Get the contact surface-layer depth. |

### Integrators (include/ode/objects.h)

| Symbol | Signature | Source | Purpose |
| --- | --- | --- | --- |
| dWorldStep | `int dWorldStep(dWorldID w, dReal stepsize)` | include/ode/objects.h:384 | Accurate big-matrix integrator: O(m²) memory/time; returns 1 success, 0 on alloc failure. |
| dWorldQuickStep | `int dWorldQuickStep(dWorldID w, dReal stepsize)` | include/ode/objects.h:427 | Fast iterative integrator O(m·N), less accurate near-singular; returns 1 success, 0 on alloc failure. |
| dWorldImpulseToForce | `void dWorldImpulseToForce(dWorldID, dReal stepsize, dReal ix, dReal iy, dReal iz, dVector3 force)` | include/ode/objects.h:444 | Convert an impulse to a force vector (scales by 1/stepsize) for use before a `BodyAdd...` call. |

### QuickStep iteration control (include/ode/objects.h)

| Symbol | Signature | Source | Purpose |
| --- | --- | --- | --- |
| dWorldSetQuickStepNumIterations | `void dWorldSetQuickStepNumIterations(dWorldID w, int num)` | include/ode/objects.h:462 | Set QuickStep iterations per step; default `dWORLDQUICKSTEP_ITERATION_COUNT_DEFAULT` (20). |
| dWorldGetQuickStepNumIterations | `int dWorldGetQuickStepNumIterations(dWorldID)` | include/ode/objects.h:470 | Get the QuickStep iterations-per-step count. |
| dWorldSetQuickStepW | `void dWorldSetQuickStepW(dWorldID, dReal over_relaxation)` | include/ode/objects.h:584 | Set the SOR over-relaxation parameter used by QuickStep. |
| dWorldGetQuickStepW | `dReal dWorldGetQuickStepW(dWorldID)` | include/ode/objects.h:591 | Get the SOR over-relaxation parameter. |
| dWorldSetQuickStepDynamicIterationParameters | `void dWorldSetQuickStepDynamicIterationParameters(dWorldID w, const dReal *ptr_iteration_premature_exit_delta, const dReal *ptr_max_num_extra_factor, const dReal *ptr_extra_iteration_requirement_delta)` | include/ode/objects.h:509 | Configure QuickStep premature-exit and extra-iteration deltas; NULL args keep prior values. |
| dWorldGetQuickStepDynamicIterationParameters | `void dWorldGetQuickStepDynamicIterationParameters(dWorldID w, dReal *out_iteration_premature_exit_delta, dReal *out_max_num_extra_factor, dReal *out_extra_iteration_requirement_delta)` | include/ode/objects.h:527 | Retrieve QuickStep dynamic-iteration parameters; NULL args skip that output. |

> **⚠ Version caveat (confirmed against the headers):** the `dWorldSet/GetQuickStepDynamicIterationParameters`
> pair and the `dWORLDQUICKSTEP_ITERATION_*` defaults exist in this **0.16 dev source** but are **NOT in the
> released ODE 0.16.6** (e.g. Homebrew) — a `grep` of your installed `objects.h` returns nothing, so using
> them there is a compile error. For portable iteration control use **`dWorldSetQuickStepNumIterations`**
> (present in both). Run `python3 scripts/check-citations.py` to see which cited symbols are absent in *your*
> installed ODE.

### Auto-disable defaults (include/ode/objects.h)

| Symbol | Signature | Source | Purpose |
| --- | --- | --- | --- |
| dWorldSetAutoDisableFlag | `void dWorldSetAutoDisableFlag(dWorldID, int do_auto_disable)` | include/ode/objects.h:748 | Set the default auto-disable flag for newly created bodies; default false. |
| dWorldGetAutoDisableFlag | `int dWorldGetAutoDisableFlag(dWorldID)` | include/ode/objects.h:741 | Get the default auto-disable flag (0 or 1). |
| dWorldSetAutoDisableLinearThreshold | `void dWorldSetAutoDisableLinearThreshold(dWorldID, dReal linear_average_threshold)` | include/ode/objects.h:677 | Set default linear-velocity idle threshold for new bodies; default 0.01. |
| dWorldGetAutoDisableLinearThreshold | `dReal dWorldGetAutoDisableLinearThreshold(dWorldID)` | include/ode/objects.h:670 | Get the default auto-disable linear threshold. |
| dWorldSetAutoDisableAngularThreshold | `void dWorldSetAutoDisableAngularThreshold(dWorldID, dReal angular_average_threshold)` | include/ode/objects.h:691 | Set default angular-velocity idle threshold for new bodies; default 0.01. |
| dWorldGetAutoDisableAngularThreshold | `dReal dWorldGetAutoDisableAngularThreshold(dWorldID)` | include/ode/objects.h:684 | Get the default auto-disable angular threshold. |
| dWorldSetAutoDisableAverageSamplesCount | `void dWorldSetAutoDisableAverageSamplesCount(dWorldID, unsigned int average_samples_count)` | include/ode/objects.h:706 | Set velocity-averaging sample count; default 1, 0 disables auto-disabling entirely. |
| dWorldGetAutoDisableAverageSamplesCount | `int dWorldGetAutoDisableAverageSamplesCount(dWorldID)` | include/ode/objects.h:698 | Get the velocity-averaging sample count. |
| dWorldSetAutoDisableSteps | `void dWorldSetAutoDisableSteps(dWorldID, int steps)` | include/ode/objects.h:720 | Set idle-step count before a body auto-disables; default 10. |
| dWorldGetAutoDisableSteps | `int dWorldGetAutoDisableSteps(dWorldID)` | include/ode/objects.h:713 | Get the auto-disable idle-step count. |
| dWorldSetAutoDisableTime | `void dWorldSetAutoDisableTime(dWorldID, dReal time)` | include/ode/objects.h:734 | Set idle time (seconds) before a body auto-disables; default 0. |
| dWorldGetAutoDisableTime | `dReal dWorldGetAutoDisableTime(dWorldID)` | include/ode/objects.h:727 | Get the auto-disable idle time in seconds. |

### Damping & max-angular-speed defaults (include/ode/objects.h)

| Symbol | Signature | Source | Purpose |
| --- | --- | --- | --- |
| dWorldSetLinearDamping | `void dWorldSetLinearDamping(dWorldID w, dReal scale)` | include/ode/objects.h:826 | Set world linear-damping scale applied to new bodies; default 0, range [0,1]. |
| dWorldGetLinearDamping | `dReal dWorldGetLinearDamping(dWorldID w)` | include/ode/objects.h:818 | Get the world linear-damping scale. |
| dWorldSetAngularDamping | `void dWorldSetAngularDamping(dWorldID w, dReal scale)` | include/ode/objects.h:840 | Set world angular-damping scale applied to new bodies; default 0, range [0,1]. |
| dWorldGetAngularDamping | `dReal dWorldGetAngularDamping(dWorldID w)` | include/ode/objects.h:832 | Get the world angular-damping scale. |
| dWorldSetLinearDampingThreshold | `void dWorldSetLinearDampingThreshold(dWorldID w, dReal threshold)` | include/ode/objects.h:798 | Set speed below which linear damping is skipped; default 0.01. |
| dWorldGetLinearDampingThreshold | `dReal dWorldGetLinearDampingThreshold(dWorldID w)` | include/ode/objects.h:790 | Get the linear-damping threshold. |
| dWorldSetAngularDampingThreshold | `void dWorldSetAngularDampingThreshold(dWorldID w, dReal threshold)` | include/ode/objects.h:812 | Set speed below which angular damping is skipped; default 0.01. |
| dWorldGetAngularDampingThreshold | `dReal dWorldGetAngularDampingThreshold(dWorldID w)` | include/ode/objects.h:804 | Get the angular-damping threshold. |
| dWorldSetDamping | `void dWorldSetDamping(dWorldID w, dReal linear_scale, dReal angular_scale)` | include/ode/objects.h:848 | Convenience setter for both linear and angular damping scales at once. |
| dWorldSetMaxAngularSpeed | `void dWorldSetMaxAngularSpeed(dWorldID w, dReal max_speed)` | include/ode/objects.h:865 | Set default max angular speed for new bodies (applied before move); default `dInfinity`. |
| dWorldGetMaxAngularSpeed | `dReal dWorldGetMaxAngularSpeed(dWorldID w)` | include/ode/objects.h:857 | Get the default max angular speed. |

### Handle & callback types

| Symbol | Definition | Source | Purpose |
| --- | --- | --- | --- |
| dWorldID | `typedef struct dxWorld *dWorldID` | include/ode/common.h:364 | Opaque handle to a world. |
| dReal | `float` (dSINGLE) / `double` (dDOUBLE) | include/ode/common.h:56 | Core scalar; exactly one precision macro must be defined. |
| dsizeint | size_t/duint64/duint32 (platform) | include/ode/odeconfig.h:102 | Size type used by the memory hooks. |
| dMessageFunction | `typedef void dMessageFunction(int errnum, const char *msg, va_list ap)` | include/ode/error.h:37 | Handler type for error/debug/message (error and debug handlers should not return). |
| dAllocFunction | `typedef void * dAllocFunction(dsizeint size)` | include/ode/memory.h:35 | Custom allocator type. |
| dReallocFunction | `typedef void * dReallocFunction(void *ptr, dsizeint oldsize, dsizeint newsize)` | include/ode/memory.h:36 | Custom reallocator type. |
| dFreeFunction | `typedef void dFreeFunction(void *ptr, dsizeint size)` | include/ode/memory.h:37 | Custom deallocator type. |
| dInitODEFlags | enum; sole member `dInitFlagManualThreadCleanup` | include/ode/odeinit.h:76 | Library init flag set. |
| dAllocateODEDataFlags | enum; `dAllocateFlagBasicData`, `dAllocateFlagCollisionData`, `dAllocateMaskAll` | include/ode/odeinit.h:146 | Per-thread allocation flag set. |
| dWorldQuickStepIterationCount_DynamicAdjustmentStatistics | struct (struct_size, iteration_count, premature_exits, prolonged_execs, full_extra_execs) | include/ode/objects.h:548 | Stats struct attached via the sink function. |

## Key rules

- `dInitODE2` must be called to initialize ODE before the first use of any other ODE call; it may not be called again until `dCloseODE` is called (include/ode/odeinit.h:103).
- `dCloseODE` is the paired shutdown for `dInitODE2`, must only be called after successful initialization, and releases all resources including every thread's thread-local data (include/ode/odeinit.h:215).
- Every thread that will call ODE must first call `dAllocateODEDataForThread`; collision-detection functions may not be called unless `dAllocateFlagCollisionData` (or `dAllocateMaskAll`) was allocated (include/ode/odeinit.h:159).
- `dCleanupODEAllDataForThread` is required on each thread's exit only if initialized with `dInitFlagManualThreadCleanup`; if that flag was NOT set, it must not be called (include/ode/odeinit.h:196).
- Ensure all threads using ODE have terminated before calling `dCloseODE`, and do not call `dCleanupODEAllDataForThread` after `dCloseODE` (include/ode/odeinit.h:219).
- `dInitODE` (no "2") is obsolete; it equals `dInitODE2(0)` followed by `dAllocateODEDataForThread(dAllocateMaskAll)` — use `dInitODE2` instead (include/ode/odeinit.h:83).
- Exactly one of `dSINGLE` or `dDOUBLE` must be `#defined` for the whole program and must match the library build; both or neither is wrong and `dReal` switches float vs double accordingly (include/ode/common.h:56).
- Error and debug handler callbacks should not return; the noreturn `dError`/`dDebug` abort the program after invoking them (include/ode/error.h:34).
- Passing `fn = 0` to any `dSet*Handler` restores the corresponding default handler (include/ode/error.h:39).
- Custom memory hooks must be installed before ODE allocates, and `dRealloc`/`dFree` require the caller to supply the original allocation size(s) (include/ode/memory.h:39).
- `dCheckConfiguration` is case-sensitive and returns 1 only on an exact token match against the `dGetConfiguration` string (include/ode/common.h:567).
- `dWorldCreate` must be called to obtain a `dWorldID` before any body/joint creation, and ODE must be initialized first (ode/demo/demo_boxstack.cpp:578-579).
- `dWorldDestroy` tears down all contained bodies and all joints not in a joint group; grouped joints are deactivated and freed via `dJointGroupEmpty`/`Destroy` instead (include/ode/objects.h:56-64).
- Each simulation tick: run collision (`dSpaceCollide` → fill contact joints), then exactly one integrator call (`dWorldStep` OR `dWorldQuickStep`), then empty the contact joint group (ode/demo/demo_boxstack.cpp:510-548).
- `dWorldStep` and `dWorldQuickStep` return an `int`: 1 = success, 0 = memory-allocation failure, in which case objects are unchanged and the step can be retried (include/ode/objects.h:380-384).
- Default gravity is (0,0,0) (no gravity); call `dWorldSetGravity` explicitly (e.g. `(0,0,-9.81)` for +z-up Earth) before stepping (include/ode/objects.h:88-93).
- Auto-disable, damping, and max-angular-speed are world-level DEFAULTS copied to bodies at creation time, not retroactive global controls; set them on the world before creating bodies (include/ode/objects.h:662).
- `dWorldSetQuickStepNumIterations` only affects `dWorldQuickStep`; it has no effect on `dWorldStep` (include/ode/objects.h:454-462).
- Setting `dWorldSetAutoDisableAverageSamplesCount` to 0 disables velocity sampling and prevents any body from auto-disabling (include/ode/objects.h:701-706).
- Damping scale should stay in [0,1]: a negative scale increases speed and >1 causes per-step oscillation, both destabilizing (include/ode/objects.h:763-765).

## Canonical patterns

```c
/* Boot, build the world, and tear down (single-threaded demo). */
dInitODE2(0);
dAllocateODEDataForThread(dAllocateMaskAll);   /* needed before any collision call */
world = dWorldCreate();
space = dHashSpaceCreate(0);
contactgroup = dJointGroupCreate(0);
dWorldSetGravity(world, 0, 0, -GRAVITY);       /* default is (0,0,0) — set it! */
dWorldSetCFM(world, 1e-5);
dWorldSetAutoDisableFlag(world, 1);
dWorldSetAutoDisableAverageSamplesCount(world, 10);
dWorldSetLinearDamping(world, 0.00001);
dWorldSetAngularDamping(world, 0.005);
dWorldSetMaxAngularSpeed(world, 200);
/* ... simulation ... */
dCloseODE();
/* ode/demo/demo_boxstack.cpp:578-594 ; lifecycle ode/demo/demo_buggy.cpp:223 */
```

```c
/* Per tick: collide, step exactly once (check the return!), empty contacts. */
dSpaceCollide(space, 0, &nearCallback);
if (!pause)
    if (!dWorldQuickStep(world, 0.02))   /* 0 = alloc failed, step did not advance */
        { /* retry when memory is available */ }
/* ... draw ... */
dJointGroupEmpty(contactgroup);
/* ode/demo/demo_boxstack.cpp:510-548 */
```

## Common mistakes

| Bad | Good | Why |
| --- | --- | --- |
| `dInitODE2(0); dCreateBox(space, ...);` (collision data never allocated) | `dInitODE2(0); dAllocateODEDataForThread(dAllocateMaskAll);` then collide | Collision functions require per-thread collision data; `dInitODE2` alone is not enough — must allocate `dAllocateFlagCollisionData`/`dAllocateMaskAll` first (include/ode/odeinit.h:159). |
| `dInitODE2(0); ... dCleanupODEAllDataForThread();` | `dInitODE2(dInitFlagManualThreadCleanup); ... dCleanupODEAllDataForThread();` | Cleanup is only legal when `dInitFlagManualThreadCleanup` was set; otherwise calling it is explicitly not allowed (include/ode/odeinit.h:196). |
| Compile the app with no `-DdDOUBLE` against a double-precision libode | Define the same `dSINGLE`/`dDOUBLE` the library was built with; assert via `dCheckConfiguration("ODE_double_precision")` | `dReal` is float vs double; a mismatch corrupts every vector/matrix across the ABI (include/ode/common.h:56). |
| `dInitODE(); /* exits with no dCloseODE */` | `dInitODE2(0); /* ... */ dCloseODE();` | `dInitODE` is obsolete and `dCloseODE` must pair with `dInitODE2` after all ODE threads end (include/ode/odeinit.h:83, include/ode/odeinit.h:215). |
| Always calling `dWorldStep` for large scenes | `dWorldQuickStep` for big/contact-heavy systems; reserve `dWorldStep` for small high-accuracy systems | `dWorldStep` is O(m²) memory and slow; `dWorldQuickStep` is O(m·N) and fast but less accurate near singular configs (include/ode/objects.h:380-427). |
| Expecting gravity without `dWorldSetGravity` | `dWorldSetGravity(world, 0, 0, -9.81)` after `dWorldCreate` | Default gravity is (0,0,0); bodies float until set (include/ode/objects.h:88-93). |
| `dWorldSetAutoDisableFlag` after creating bodies, expecting them to change | Set world defaults before creating bodies, or set per-body via the `dBody*` equivalents | World auto-disable/damping setters only seed defaults at body-creation time (include/ode/objects.h:662). |
| `dWorldQuickStep(w, dt);` (return ignored) | `if (!dWorldQuickStep(w, dt)) { /* retry when memory available */ }` | A 0 return means allocation failed and the step did NOT advance; treating it as success corrupts the loop (include/ode/objects.h:380-384). |

## Named constants

| Name | Value | Source | Purpose |
| --- | --- | --- | --- |
| dInitFlagManualThreadCleanup | `0x00000001` | include/ode/odeinit.h:77 | Init flag requiring explicit `dCleanupODEAllDataForThread` per thread instead of automatic tracking. |
| dAllocateFlagBasicData | `0` | include/ode/odeinit.h:147 | Basic data for normal operation; always implicitly included. |
| dAllocateFlagCollisionData | `0x00000001` | include/ode/odeinit.h:149 | Allocates collision-detection data; required before any collision call. |
| dAllocateMaskAll | `~0` | include/ode/odeinit.h:151 | Allocate all possible data; must not be combined with other flags. |
| dWORLDQUICKSTEP_ITERATION_COUNT_DEFAULT | `20U` | include/ode/objects.h:451 | Default QuickStep iterations per step. |
| dWORLDQUICKSTEP_ITERATION_PREMATURE_EXIT_DELTA_DEFAULT | `1e-8f` | include/ode/objects.h:473 | Default premature-exit delta for QuickStep dynamic iterations. |
| dWORLDSTEP_THREADCOUNT_UNLIMITED | `= dTHREADING_THREAD_COUNT_UNLIMITED` | include/ode/objects.h:137 | Sentinel for no island-stepping thread limit. |
| dInfinity | infinity in `dReal` precision | include/ode/odeconfig.h:154 | Infinity constant (default `dWorldSetMaxAngularSpeed`, contact max correcting vel). |
| dNaN | NaN macro (`#define dNaN`) | include/ode/odeconfig.h:181 | NaN constant. |
| dODE_VERSION | build-substituted string (`@ODE_VERSION@`; `ODE_VERSION=0.16` in configure.ac) | include/ode/version.h.in:4 | Version string; literal is generated at build time, not committed. |
| ODE_API | dllexport on Windows DLL builds, empty otherwise | include/ode/odeconfig.h:47 | Linkage/export macro on all public functions. |
| ODE_NORETURN | noreturn attribute macro | include/ode/odeconfig.h:75 | Applied to `dError` and `dDebug`. |

Config tokens reported by `dGetConfiguration` (presence is build-dependent): `ODE`, `ODE_single_precision`, `ODE_double_precision`, `ODE_EXT_no_debug`, `ODE_EXT_trimesh`, `ODE_EXT_opcode`, `ODE_EXT_gimpact`, `ODE_OPC_16bit_indices`, `ODE_OPC_new_collider`, `ODE_EXT_libccd`, `ODE_CCD_IMPL_internal`, the `ODE_CCD_COLL_*` variants, `ODE_EXT_mt_collisions`, `ODE_EXT_threading`, `ODE_THR_builtin_impl` (include/ode/common.h:540).

## Things to never invent

These are real ODE renames that get mis-recalled — use the real symbol on the right:

- `dCleanupODEDataForThread` — the real name is `dCleanupODEAllDataForThread`.
- `dInitFlagAutoThreadCleanup` — only `dInitFlagManualThreadCleanup` exists.
- `dAllocateFlagAll` — the real name is `dAllocateMaskAll`.
- `dSetMemoryHandler` — the real hooks are `dSetAllocHandler`/`dSetReallocHandler`/`dSetFreeHandler`.
- `dSetWarningHandler` — the real name is `dSetMessageHandler`; there is no `dWarning` (use `dMessage`).
- `dWorldSetIterations` — the real name is `dWorldSetQuickStepNumIterations`.
- `dWorldSetTimeStep` / `dWorldGetStepSize` — `stepsize` is a per-call argument to `dWorldStep`/`dWorldQuickStep`, not stored on the world.
- `dWorldSetSOR` — the SOR over-relaxation knob is `dWorldSetQuickStepW`.
- `dWorldStepFast` / `dWorldStepFast1` — a removed legacy integrator; use `dWorldQuickStep`.
- `dWorldSetGravityVector` — it is `dWorldSetGravity` with `x, y, z` scalars.
- `dWorldSetAutoDisableEnable` — it is `dWorldSetAutoDisableFlag`.
