# Multithreading & Island Stepping

> Source of truth: include/ode/threading.h, include/ode/threading_impl.h, include/ode/cooperative.h, include/ode/objects.h. Every rule cites real ODE source by file:line; headers win over the wiki on conflict. C symbols start with 'd'; do not invent symbols.

## Mental model
- ODE parallelizes stepping by splitting the world into **islands** (connected components of joint-linked bodies) automatically inside `dWorldStep`/`dWorldQuickStep`. You never call the splitter yourself — it is internal (ode/src/ode.cpp:1835-1838).
- To get parallel work you supply a **threading implementation** + a **thread pool**, then attach the implementation to the world. The pool's worker threads serve the implementation; the steppers post island tasks (and lower-level sub-calls) onto it.
- **Default is single-threaded.** A world with no implementation assigned uses a global self-threaded implementation that runs everything on the caller's thread — you do nothing for serial behavior (include/ode/threading_impl.h:63-67).
- A multi-threaded implementation is inert until **served**: allocating a pool is not enough, you must call `dThreadingThreadPoolServeMultiThreadedImplementation` (include/ode/threading_impl.h:227-229).
- Lifecycle is strict and ordered: allocate impl → allocate pool → serve → attach-to-world; teardown reverses it (shutdown → free pool → detach → free impl).
- The **Cooperative** API (`dCooperative*` + `dResourceRequirements*` + `dResourceContainer*`) is a separate, optional layer for cooperative algorithms (e.g. the LDLT factor/solve helpers) that run over the same threading implementation.

## When to use
- Use the threading API only when you want `dWorldQuickStep`/`dWorldStep` to run across multiple cores. For a single-threaded simulation, do nothing — the default self-threaded implementation already backs every world.
- Reach for the **multi-threaded** path when the world contains many independent islands (stacks, scattered objects); parallelism scales with island count, not with body count inside one island (ode/src/ode.cpp:1835-1838).
- Use `dWorldSetStepIslandsProcessingMaxThreadCount` to trade memory for island parallelism (each island thread needs its own buffer sized to the largest island).
- Use the Cooperative + ResourceRequirements/ResourceContainer trio only when calling the cooperative matrix helpers (`dCooperativelyFactorLDLT` and friends) directly.

## Public API (C)

### Threading implementation lifecycle (include/ode/threading_impl.h)

| Symbol | Signature | Source | Purpose |
|---|---|---|---|
| `dThreadingAllocateSelfThreadedImplementation` | `dThreadingImplementationID dThreadingAllocateSelfThreadedImplementation(void)` | include/ode/threading_impl.h:75 | Allocate a self-threaded (runs on caller's thread) implementation. Rarely needed — a global one is used by default. Returns NULL on failure. |
| `dThreadingAllocateMultiThreadedImplementation` | `dThreadingImplementationID dThreadingAllocateMultiThreadedImplementation(void)` | include/ode/threading_impl.h:92 | Allocate a multi-threaded implementation that **must be served** by a pool (or external threads) before it does parallel work. Returns NULL on failure. |
| `dThreadingImplementationGetFunctions` | `const dThreadingFunctionsInfo *dThreadingImplementationGetFunctions(dThreadingImplementationID impl)` | include/ode/threading_impl.h:107 | Get the function-pointer table for a **built-in** impl. Pass result as `functions_info` to `dWorldSetStepThreadingImplementation`/`dCooperativeCreate`. Do not use with custom impls. |
| `dThreadingImplementationShutdownProcessing` | `void dThreadingImplementationShutdownProcessing(dThreadingImplementationID impl)` | include/ode/threading_impl.h:135 | Release the threads serving the impl so they return to their origin. All dependent ODE calls must have returned first. No effect on a self-threaded impl. |
| `dThreadingImplementationCleanupForRestart` | `void dThreadingImplementationCleanupForRestart(dThreadingImplementationID impl)` | include/ode/threading_impl.h:153 | Restore a multi-threaded impl's state after shutdown so it can be served again. No effect on a self-threaded impl. |
| `dThreadingFreeImplementation` | `void dThreadingFreeImplementation(dThreadingImplementationID impl)` | include/ode/threading_impl.h:167 | Delete a built-in impl. First unassign from every object and ensure no threads still serve it. |
| `dExternalThreadingServeMultiThreadedImplementation` | `void dExternalThreadingServeMultiThreadedImplementation(dThreadingImplementationID impl, dThreadReadyToServeCallback *readiness_callback/*=NULL*/, void *callback_context/*=NULL*/)` | include/ode/threading_impl.h:194 | Entry point for a user-owned external thread to dedicate itself to serving a multi-threaded impl (instead of ODE's pool). Blocks until shutdown; optional `readiness_callback` fires after the thread registers. |

### Built-in thread pool (include/ode/threading_impl.h)

| Symbol | Signature | Source | Purpose |
|---|---|---|---|
| `dThreadingAllocateThreadPool` | `dThreadingThreadPoolID dThreadingAllocateThreadPool(unsigned thread_count, dsizeint stack_size, unsigned int ode_data_allocate_flags, void *reserved/*=NULL*/)` | include/ode/threading_impl.h:223 | Create a pool of `thread_count` workers. `stack_size=0` → system default. `ode_data_allocate_flags` is passed to `dAllocateODEDataForThread` per worker (demos pass `dAllocateFlagBasicData`). Returns NULL on failure. POSIX: call with signals masked. |
| `dThreadingThreadPoolServeMultiThreadedImplementation` | `void dThreadingThreadPoolServeMultiThreadedImplementation(dThreadingThreadPoolID pool, dThreadingImplementationID impl)` | include/ode/threading_impl.h:246 | Attach the pool to an impl so its threads start serving. One impl per pool at a time. Blocks until all workers register, so the impl is usable on return. |
| `dThreadingThreadPoolWaitIdleState` | `void dThreadingThreadPoolWaitIdleState(dThreadingThreadPoolID pool)` | include/ode/threading_impl.h:270 | Block until all pool threads are released from any impl. Use after shutdown to confirm threads are free. Not required before destroy (free waits implicitly); returns immediately if not serving. |
| `dThreadingFreeThreadPool` | `void dThreadingFreeThreadPool(dThreadingThreadPoolID pool)` | include/ode/threading_impl.h:285 | Delete a pool. Threads must be released from any impl first (call shutdown), else this blocks until they return. |

### World wiring (include/ode/objects.h)

| Symbol | Signature | Source | Purpose |
|---|---|---|---|
| `dWorldSetStepThreadingImplementation` | `void dWorldSetStepThreadingImplementation(dWorldID w, const dThreadingFunctionsInfo *functions_info, dThreadingImplementationID threading_impl)` | include/ode/objects.h:364 | Assign the impl + its functions table the world uses to parallelize `[quick]step`. Pass `dThreadingImplementationGetFunctions(impl)` and `impl`. Pass `(NULL, NULL)` to detach and revert to the default self-threaded behavior. |
| `dWorldSetStepIslandsProcessingMaxThreadCount` | `void dWorldSetStepIslandsProcessingMaxThreadCount(dWorldID w, unsigned count)` | include/ode/objects.h:160 | Cap parallel island threads. Effective = min(this, pool thread count). Default `dWORLDSTEP_THREADCOUNT_UNLIMITED`. `count=1` forces sequential islands. Does not affect lower-level sub-call parallelism. |
| `dWorldGetStepIslandsProcessingMaxThreadCount` | `unsigned dWorldGetStepIslandsProcessingMaxThreadCount(dWorldID w)` | include/ode/objects.h:172 | Read the current island-stepping thread-count limit (`0` == unlimited). |

### Cooperative algorithm container (include/ode/cooperative.h)

| Symbol | Signature | Source | Purpose |
|---|---|---|---|
| `dCooperativeCreate` | `dCooperativeID dCooperativeCreate(const dThreadingFunctionsInfo *functionInfo/*=NULL*/, dThreadingImplementationID threadingImpl/*=NULL*/)` | include/ode/cooperative.h:108 | Create the shared-context container cooperative algorithms run on. Pass the threading table + impl (`NULL`/`NULL` → default self-threading). Per-instance calls must be serialized; must outlive everything referencing it. |
| `dCooperativeDestroy` | `void dCooperativeDestroy(dCooperativeID cooperative)` | include/ode/cooperative.h:119 | Destroy a Cooperative — only after every referencing object is destroyed. NULL allowed. |
| `dResourceRequirementsCreate` | `dResourceRequirementsID dResourceRequirementsCreate(dCooperativeID cooperative)` | include/ode/cooperative.h:144 | Create an empty (descriptive) ResourceRequirements tied to a Cooperative. Estimate functions add requirements to it. Returns NULL on failure. |
| `dResourceRequirementsDestroy` | `void dResourceRequirementsDestroy(dResourceRequirementsID requirements)` | include/ode/cooperative.h:156 | Destroy a ResourceRequirements — any time, holds no real resources. NULL allowed. |
| `dResourceRequirementsClone` | `dResourceRequirementsID dResourceRequirementsClone(dResourceRequirementsID requirements)` | include/ode/cooperative.h:174 | Copy a ResourceRequirements (contents + Cooperative relation preserved); source unchanged. Destroy the clone. |
| `dResourceRequirementsMergeIn` | `void dResourceRequirementsMergeIn(dResourceRequirementsID summaryRequirements, dResourceRequirementsID extraRequirements)` | include/ode/cooperative.h:191 | Grow `summaryRequirements` so it also covers `extraRequirements` (per-field maxima); `extraRequirements` unchanged. Both from the same Cooperative. |
| `dResourceContainerAcquire` | `dResourceContainerID dResourceContainerAcquire(dResourceRequirementsID requirements)` | include/ode/cooperative.h:212 | Allocate the actual resources described by a ResourceRequirements; this is what algorithms consume. Requirements reusable and deletable immediately. Returns NULL on failure. |
| `dResourceContainerDestroy` | `void dResourceContainerDestroy(dResourceContainerID resources)` | include/ode/cooperative.h:221 | Destroy a ResourceContainer and release its resources. NULL allowed. |

### Cooperative estimate helper used in the demo (include/ode/matrix_coop.h)

| Symbol | Signature | Source | Purpose |
|---|---|---|---|
| `dEstimateCooperativelyFactorLDLTResourceRequirements` | `void dEstimateCooperativelyFactorLDLTResourceRequirements(dResourceRequirementsID requirements, ...)` | include/ode/matrix_coop.h:61 | Adds the resource needs of a cooperative LDLT factorization to `requirements` so a sufficient ResourceContainer can be acquired (see demo_ode.cpp). |

### Opaque types & implementor-facing typedefs

| Symbol | Definition | Source | Purpose |
|---|---|---|---|
| `dThreadingImplementationID` | `struct dxThreadingImplementation *` | include/ode/threading.h:45-46 | Handle to a threading implementation (built-in self/multi-threaded, or custom). |
| `dThreadingThreadPoolID` | `struct dxThreadingThreadPool *` | include/ode/threading_impl.h:45-46 | Handle to a built-in thread pool. |
| `dThreadingFunctionsInfo` | function-pointer table struct | include/ode/threading.h:375-405 | The interface an implementation provides (mutex group, call-wait, post/wait/alter calls, retrieve_thread_count, preallocate). Built-in: get via `dThreadingImplementationGetFunctions`; custom: supply your own. |
| `dThreadReadyToServeCallback` | `typedef void (dThreadReadyToServeCallback)(void *callback_context)` | include/ode/threading_impl.h:170 | Optional readiness callback for `dExternalThreadingServeMultiThreadedImplementation`. |
| `dCooperativeID` | `struct dxCooperative *` | include/ode/cooperative.h:42,57 | Shared-context container for cooperative algorithms; per-instance use must be serialized. |
| `dResourceRequirementsID` | `struct dxResourceRequirements *` | include/ode/cooperative.h:43,77 | Descriptive accumulator of cooperative resource needs; cloneable/mergeable. |
| `dResourceContainerID` | `struct dxResourceContainer *` | include/ode/cooperative.h:44,90 | Holds resources actually allocated per a ResourceRequirements; passed into algorithm calls. |
| `dMutexGroupID` | `struct dxMutexGroup *` | include/ode/threading.h:49-50 | Group of non-recursive mutexes (implementor-facing; via the functions table). |
| `dmutexindex_t` | `typedef unsigned` | include/ode/threading.h:48 | Index/count type for mutexes within a `dMutexGroup`. |
| `dCallReleaseeID` | `struct dxCallReleasee *` | include/ode/threading.h:147-148 | Posted-call dependency handle for wiring sub-call dependencies (implementor-facing). |
| `dCallWaitID` | `struct dxCallWait *` | include/ode/threading.h:150-151 | Wait handle for a posted threaded call (implementor-facing). |
| `dThreadedCallFunction` | `int (...)(void *call_context, dcallindex_t instance_index, dCallReleaseeID this_releasee)` | include/ode/threading.h:156-157 | Callback signature for a posted threaded call (implementor-facing). |
| `dThreadedWaitTime` | `struct { time_t wait_sec; unsigned long wait_nsec; }` | include/ode/threading.h:159-164 | Timeout for `wait_call`: NULL = infinite, zero value = non-blocking check (implementor-facing). |
| `ddependencycount_t` / `ddependencychange_t` / `dcallindex_t` | integer typedefs | include/ode/threading.h:153-155 | Task-graph counters/index types (implementor-facing). |

## Key rules
- Canonical multi-threaded setup: allocate a multi-threaded impl, allocate a pool, **serve** the pool to the impl, then attach the impl to the world with `dWorldSetStepThreadingImplementation(world, dThreadingImplementationGetFunctions(impl), impl)` — `dWorldQuickStep` then parallelizes across islands (ode/demo/demo_boxstack.cpp:601-605).
- Teardown reverses setup: `dThreadingImplementationShutdownProcessing(impl)` → `dThreadingFreeThreadPool(pool)` → `dWorldSetStepThreadingImplementation(world, NULL, NULL)` → `dThreadingFreeImplementation(impl)` (ode/demo/demo_boxstack.cpp:610-613).
- For built-in implementations always pass `dThreadingImplementationGetFunctions(impl)` as the functions table, never a hand-built one — both for the world and for `dCooperativeCreate` (include/ode/threading_impl.h:107).
- A multi-threaded implementation does nothing until a pool (or external threads) is told to serve it; allocating the pool is not enough, and Serve blocks until workers register (include/ode/threading_impl.h:227-237).
- If no implementation is assigned to a world, ODE uses a global default self-threaded (single-threaded) implementation, so you only allocate/assign one when you actually want parallel stepping (include/ode/threading_impl.h:63-67).
- Each world stepped in parallel needs its OWN implementation instance; sharing one across parallel-stepped worlds is not recommended — the built-in impl is likely to crash and per-call resource preallocation loses its sense (include/ode/objects.h:353-356).
- `dWorldSetStepIslandsProcessingMaxThreadCount` trades memory for island parallelism: each island thread allocates its own stepping buffer sized for the LARGEST island; default is unlimited and `count=1` forces sequential island stepping (include/ode/objects.h:146-153).
- On POSIX, call `dThreadingAllocateThreadPool` with signals masked — pool threads start with all signals blocked, inherit caller priority, and commit their stacks immediately on start (include/ode/threading_impl.h:204-216).
- Releasing/freeing order matters: before `dThreadingFreeImplementation` the impl must be unassigned from all objects and no thread may still serve it; before `dThreadingFreeThreadPool` the threads must be released from any impl or the call blocks (include/ode/threading_impl.h:157-167).
- To reuse a multi-threaded implementation after `dThreadingImplementationShutdownProcessing` (and after its threads exit), call `dThreadingImplementationCleanupForRestart` before serving it again (include/ode/threading_impl.h:153).
- Calls referring to a single Cooperative instance must be serialized — never invoke functions on the same Cooperative in parallel; the Cooperative must outlive every ResourceRequirements/ResourceContainer that references it (include/ode/cooperative.h:53-55).
- Never invoke the wait function from inside a threaded call — it blocks a physical worker thread and risks pool starvation; express ordering with call dependencies instead (include/ode/threading.h:306-309).
- **Stepping-threading is a 3-level control — and the API differs by ODE version.** In the **released 0.16.6** (what Homebrew ships) the full struct API exists: `dWorldSetSteppingThreadingParameters(world, &p)` with `dWorldSteppingThreadingParameters{ param_set; world_islands_iteration_max_threads (L1); island_stepping_max_threads (L2); lcp_solving_max_threads (L3); }` and the `dWSTP_*` select-flags (`dWSTP_WorldIslandsIterationMaxThreads`/`IslandStepping`/`LCPSolving`) — installed `objects.h:141-207`. Total threads ≈ `L1 × MAX(L2, L3)`. **Defaults:** L1 and L2 unlimited, **L3 (lcp_solving) = 1**. The L1 count is also settable via `dWorldSet/GetStepIslandsProcessingMaxThreadCount` (present in both versions). **Version note:** this 0.16 dev repo does NOT have the struct API (only the L1 knob) — `grep dWorldSetSteppingThreadingParameters` your **installed** header (or run `scripts/check-citations.py`) to confirm what you have.
- **`dWorldStep`'s LCP solve is single-threaded — header-stated in 0.16.6.** Installed `objects.h:175`: *"in current implementation `dWorldStep` supports single threaded LCP solving only and the `lcp_solving_max_threads` parameter has no effect for it."* So `dWorldStep` scales only with the number of **independent islands** (parallelism scales with island count, not solver size); `dWorldQuickStep`'s iterative solver is the one whose sub-steps can thread. (This 0.16 dev repo lacks that sentence — cite the **installed** header for it.)
- **Determinism (field-measured): the nondeterminism source is PARALLEL ISLAND ITERATION (L1), not the sub-island work.** With ≥2 threads iterating islands concurrently, floating-point reductions reorder and the run is not bit-reproducible — the drift was *identical* across every L2/L3 setting, isolating L1 as the cause. **Bit-exactness IS recoverable:** set `world_islands_iteration_max_threads = 1` (`dWorldSetStepIslandsProcessingMaxThreadCount(world,1)`) — even with 8 pool workers the digest then matches the serial run exactly. **Tuning win:** the *fastest* config is **island iteration parallel + `island_stepping_max_threads = 1`** (L3 defaults to 1) — field-measured **8.95× at 16 threads** for many small islands, vs the naive all-unlimited default that peaked ~3.25× and *degraded* to 2×. So: parallelize islands (L1), serialize the sub-island level (L2=1), and drop L1 to 1 only when you need bit-exact reproducibility. (To hash a run, fold every body's final pose into one digest and compare — see `references/foundations/verifying-simulations.md` determinism fingerprint.)

## Common mistakes

| Bad | Good | Why |
|---|---|---|
| `dThreadingAllocateThreadPool(4, 0, dAllocateFlagBasicData, NULL); dWorldSetStepThreadingImplementation(world, fns, impl); /* never Served */` | `dThreadingThreadPoolServeMultiThreadedImplementation(pool, impl); dWorldSetStepThreadingImplementation(world, dThreadingImplementationGetFunctions(impl), impl);` | A multi-threaded impl is inert until a pool (or external threads) serves it; Serve also blocks until workers register so the impl is ready (include/ode/threading_impl.h:227-237). |
| `dWorldSetStepThreadingImplementation(world, someHandBuiltFnTable, impl);` | `dWorldSetStepThreadingImplementation(world, dThreadingImplementationGetFunctions(impl), impl);` | For built-in impls the correct functions table comes only from `dThreadingImplementationGetFunctions`; custom tables must be bundled with their own custom impl (include/ode/threading_impl.h:103-107). |
| `dThreadingFreeImplementation(impl); dThreadingFreeThreadPool(pool); /* still attached + served */` | `dThreadingImplementationShutdownProcessing(impl); dThreadingFreeThreadPool(pool); dWorldSetStepThreadingImplementation(world, NULL, NULL); dThreadingFreeImplementation(impl);` | Freeing while threads still serve it (or it is still attached to the world) is undefined; unassign and stop the threads first (include/ode/threading_impl.h:157-167). |
| `t = dThreadingAllocateMultiThreadedImplementation(); SetStep(worldA, fns, t); SetStep(worldB, fns, t); /* step A,B in parallel */` | Allocate a separate (impl, pool) per world stepped in parallel. | Assigning the same impl to worlds stepped in parallel is not recommended — built-in impl likely crashes and resource preallocation loses meaning (include/ode/objects.h:353-356). |
| Calling `dThreadingAllocateThreadPool` on POSIX with signals unmasked, or allocating a self-threaded impl just for serial use. | Mask signals around pool allocation on POSIX; for serial use leave the world unassigned (default self-threaded impl is used). | Pool threads start with all signals blocked and the call must run with signals masked; a global self-threading object already backs any unassigned world (include/ode/threading_impl.h:204-216; include/ode/threading_impl.h:63-67). |

## Named constants

| Name | Value | Source | Purpose |
|---|---|---|---|
| `dTHREADING_THREAD_COUNT_UNLIMITED` | `0U` | include/ode/threading.h:53 | Sentinel meaning "no thread-count limit". |
| `dWORLDSTEP_THREADCOUNT_UNLIMITED` | `dTHREADING_THREAD_COUNT_UNLIMITED` (== `0U`) | include/ode/objects.h:137 | Default island-stepping thread limit (unlimited) for `dWorldSet/GetStepIslandsProcessingMaxThreadCount`. |
| `dAllocateFlagBasicData` | `0` | include/ode/odeinit.h:147 | Per-thread ODE-data allocation flag passed as `ode_data_allocate_flags` to `dThreadingAllocateThreadPool` in every threaded demo. Related: `dAllocateFlagCollisionData` (`0x1`), `dAllocateMaskAll` (`~0`). |

## Canonical setup code

### Multi-threaded QuickStep setup and teardown (built-in pool)
```c
/* setup: islands are built implicitly by stepping; allocate impl + pool; attach to world */
dThreadingImplementationID threading = dThreadingAllocateMultiThreadedImplementation();
dThreadingThreadPoolID pool = dThreadingAllocateThreadPool(4, 0, dAllocateFlagBasicData, NULL);
dThreadingThreadPoolServeMultiThreadedImplementation(pool, threading);
// dWorldSetStepIslandsProcessingMaxThreadCount(world, 1); // optional: cap island threads (1 == sequential)
dWorldSetStepThreadingImplementation(world, dThreadingImplementationGetFunctions(threading), threading);

// ... run simulation: dWorldQuickStep(world, dt) now parallelizes across islands ...

/* teardown (reverse order) */
dThreadingImplementationShutdownProcessing(threading);
dThreadingFreeThreadPool(pool);
dWorldSetStepThreadingImplementation(world, NULL, NULL);
dThreadingFreeImplementation(threading);
```
(ode/demo/demo_boxstack.cpp:601-613)

### Threading impl as file-scope globals (demo_crash pattern)
```c
static dThreadingImplementationID threading;
static dThreadingThreadPoolID pool;

/* in setup */
threading = dThreadingAllocateMultiThreadedImplementation();
pool = dThreadingAllocateThreadPool(4, 0, dAllocateFlagBasicData, NULL);
dThreadingThreadPoolServeMultiThreadedImplementation(pool, threading);
// dWorldSetStepIslandsProcessingMaxThreadCount(world, 1);
dWorldSetStepThreadingImplementation(world, dThreadingImplementationGetFunctions(threading), threading);

/* on exit */
dThreadingImplementationShutdownProcessing(threading);
dThreadingFreeThreadPool(pool);
dWorldSetStepThreadingImplementation(world, NULL, NULL);
dThreadingFreeImplementation(threading);
```
(ode/demo/demo_crash.cpp:86-87,275-279,241-244)

### Cooperative + ResourceRequirements/ResourceContainer over a pool (cooperative LDLT helpers)
```c
const unsigned threadCountMaximum = 8;
dThreadingImplementationID threading = dThreadingAllocateMultiThreadedImplementation();
dCooperativeID cooperative = dCooperativeCreate(dThreadingImplementationGetFunctions(threading), threading);
dThreadingThreadPoolID pool = dThreadingAllocateThreadPool(threadCountMaximum, 0, dAllocateFlagBasicData, NULL);
dThreadingThreadPoolServeMultiThreadedImplementation(pool, threading);

dResourceRequirementsID requirements = dResourceRequirementsCreate(cooperative);
dEstimateCooperativelyFactorLDLTResourceRequirements(requirements, threadCountMaximum, COOP_MSIZE);
dResourceContainerID resources = dResourceContainerAcquire(requirements);
// ... cooperative computations using `resources` (e.g. dCooperativelyFactorLDLT) ...
dResourceContainerDestroy(resources);
dResourceRequirementsDestroy(requirements);

dThreadingImplementationShutdownProcessing(threading);
dThreadingFreeThreadPool(pool);
dCooperativeDestroy(cooperative);
dThreadingFreeImplementation(threading);
```
(ode/demo/demo_ode.cpp:492-542)

## Things to never invent
- Do NOT invent a `dThreadingGetThreadCount` / `dThreadingImplementationGetThreadCount` public free function — thread-count retrieval exists only as the `dThreadingImplThreadCountRetrieveFunction` pointer (`retrieve_thread_count`) inside `dThreadingFunctionsInfo` (include/ode/threading.h:344,392); there is no `ODE_API` wrapper.
- Do NOT claim a public trylock function exists — `dMutexGroupMutexTryLockFunction` and the `trylock_group_mutex` slot are deliberately commented out (include/ode/threading.h:128,402).
- Do NOT invent thread-affinity, thread-priority, or thread-count setters on the pool — `dThreadingAllocateThreadPool` fixes `thread_count` at creation; affinity is the system default and priority is inherited (include/ode/threading_impl.h:204).
- Do NOT present the low-level task-graph functions (`dThreadedCallPostFunction`, `dThreadedCallWaitFunction`, `dThreadedCallDependenciesCountAlterFunction`, `dMutexGroup*` functions, `dThreadedCallWaitAlloc/Reset/Free`) as `ODE_API` free functions — they are typedefs for function pointers inside `dThreadingFunctionsInfo`, meant for implementors of a custom backend, not callers.
- Do NOT invent island-stepping helpers like `dWorldStepIsland` or `dWorldSetIslandsProcessing*` beyond `dWorldSet/GetStepIslandsProcessingMaxThreadCount` (include/ode/objects.h:160,172).
- **Version-check the struct API — it is version-dependent, not a fabrication.** The `dWorldSetSteppingThreadingParameters` struct API (with the `dWSTP_*` flags and the `*_max_threads` fields) **exists in the released 0.16.6** (`grep` your installed `objects.h:141-207`) but is **absent in this 0.16 dev repo**. Confirm against your installed header (`scripts/check-citations.py`) before using it. The L1 knob `dWorldSetStepIslandsProcessingMaxThreadCount` is present in both.
- Do NOT add parameters to `dThreadingAllocateSelfThreadedImplementation` / `dThreadingAllocateMultiThreadedImplementation` — both take no arguments (include/ode/threading_impl.h:75,92).
