# ODE Utilities: Random, Timing, DIF Export, Memory & Error Hooks

> Source of truth: include/ode/misc.h, include/ode/timer.h, include/ode/export-dif.h, include/ode/memory.h, include/ode/error.h (assertion macros from ode/src/error.h). Every rule cites real ODE source by file:line; headers win over the wiki on conflict. C symbols start with 'd'; do not invent symbols.

## Mental model

- The random API is **one process-global stream** (a single static seed in `misc.cpp`). `dRand`, `dRandInt`, `dRandReal`, `dMakeRandomVector`, and `dMakeRandomMatrix` all draw from it; there is no per-world or per-object RNG.
- The timing API has two flavors: **code timers** (`dTimerStart/Now/End/Report`, a global fixed array of 100 slots) and **stopwatches** (`dStopwatch` + `dStopwatchReset/Start/Stop/Time`, caller-owned structs that accumulate).
- `dWorldExportDIF` writes a world to an **already-opened FILE\*** in text Dynamics Interchange Format; you own the `fopen`/`fclose`. There is no DIF *reader* in the public API.
- Memory and error/handler hooks are **process-global function pointers**; install them before any allocation or simulation, and pass `0` to restore the built-in default.
- The `dIASSERT/dUASSERT/dAASSERT/dDEBUGMSG` family is **library-private** (only in `ode/src/error.h`, not the public header); app code uses `dError/dDebug/dMessage` instead.

## When to use

- You need reproducible simulations or test data → seed and draw with the random API.
- You are profiling collision/step/draw phases → code timers (`dTimerStart`…) or stopwatches.
- You want to serialize a world for inspection/regression diff → `dWorldExportDIF`.
- You are comparing two matrices in a test → `dMaxDifference` / `dMaxDifferenceLowerTriangle`.
- You must route ODE's allocations or fatal-error behavior through your own runtime → memory/error handlers.

## Public API (C)

### Random number generation (include/ode/misc.h)

| Symbol | Signature | Source | Purpose |
| --- | --- | --- | --- |
| `dTestRand` | `int dTestRand(void)` | include/ode/misc.h:37 | Returns 1 if the RNG is working (validates the first outputs from seed 0), else 0; saves/restores the seed so it does not perturb the stream. |
| `dRand` | `unsigned long dRand(void)` | include/ode/misc.h:42 | Next 32-bit pseudo-random number from a linear congruential generator (`1664525*seed + 1013904223`); header warns it is "a not-very-random linear congruential method". Thread-safe via atomic compare-exchange on the static seed. |
| `dRandGetSeed` | `unsigned long dRandGetSeed(void)` | include/ode/misc.h:45 | Returns the current RNG seed (the internal static `duint32`). |
| `dRandSetSeed` | `void dRandSetSeed(unsigned long s)` | include/ode/misc.h:46 | Sets the current RNG seed; call once at startup for a reproducible run, or to restore a saved checkpoint. |
| `dRandInt` | `int dRandInt(int n)` | include/ode/misc.h:51 | Random integer in `[0, n-1]` via bit-folding/multiply-shift (not modulo); header warns the distribution "will get worse as n approaches 2^32". |
| `dRandReal` | `dReal dRandReal(void)` | include/ode/misc.h:54 | Random `dReal` in `[0,1]`, computed as `dRand()/(double)0xffffffff` (1.0 is attainable). |

### Random matrix/vector & matrix helpers (include/ode/misc.h)

| Symbol | Signature | Source | Purpose |
| --- | --- | --- | --- |
| `dPrintMatrix` | `void dPrintMatrix(const dReal *A, int n, int m, const char *fmt, FILE *f)` | include/ode/misc.h:57 | Prints an n-by-m matrix `A` to `f` using printf format `fmt`. C++-only overload defaults `fmt="%10.4f "` and `f=stdout`. |
| `dMakeRandomVector` | `void dMakeRandomVector(dReal *A, int n, dReal range)` | include/ode/misc.h:60 | Fills the n-element array `A` with random values in `+/- range`. |
| `dMakeRandomMatrix` | `void dMakeRandomMatrix(dReal *A, int n, int m, dReal range)` | include/ode/misc.h:63 | Fills an n-by-m matrix `A` (size `n*m`) with random values in `+/- range`. |
| `dClearUpperTriangle` | `void dClearUpperTriangle(dReal *A, int n)` | include/ode/misc.h:66 | Zeroes the strict upper triangle of the n-by-n matrix `A`. |
| `dMaxDifference` | `dReal dMaxDifference(const dReal *A, const dReal *B, int n, int m)` | include/ode/misc.h:69 | Maximum absolute element-wise difference between two n-by-m matrices — a testing/verification helper. |
| `dMaxDifferenceLowerTriangle` | `dReal dMaxDifferenceLowerTriangle(const dReal *A, const dReal *B, int n)` | include/ode/misc.h:73 | Maximum absolute difference between the lower triangles of two n-by-n matrices. |

### Stopwatch (include/ode/timer.h)

| Symbol | Signature | Source | Purpose |
| --- | --- | --- | --- |
| `dStopwatch` (type) | `typedef struct dStopwatch { double time; unsigned long cc[2]; } dStopwatch` | include/ode/timer.h:35 | Accumulating stopwatch: `time` is total clock count, `cc[2]` holds the clock count captured at the last Start. |
| `dStopwatchReset` | `void dStopwatchReset(dStopwatch *)` | include/ode/timer.h:40 | Zeroes the accumulated time/clock-count fields — call before a fresh measurement. |
| `dStopwatchStart` | `void dStopwatchStart(dStopwatch *)` | include/ode/timer.h:41 | Records the current hardware clock count as the start of an interval; does **not** reset accumulated time. |
| `dStopwatchStop` | `void dStopwatchStop(dStopwatch *)` | include/ode/timer.h:42 | Adds `(now - last start)` to the accumulated clock count; Start/Stop pairs accumulate across calls. |
| `dStopwatchTime` | `double dStopwatchTime(dStopwatch *)` | include/ode/timer.h:43 | Returns total accumulated time in seconds. |

### Code timers (include/ode/timer.h)

| Symbol | Signature | Source | Purpose |
| --- | --- | --- | --- |
| `dTimerStart` | `void dTimerStart(const char *description)` | include/ode/timer.h:48 | Begins a code-timing session, recording slot 0 with `description`. Header: "pass a static string here" (pointer is stored, not copied). |
| `dTimerNow` | `void dTimerNow(const char *description)` | include/ode/timer.h:49 | Records an intermediate checkpoint labelled `description` (static string). Silently does nothing once the slot array (MAXNUM=100) is full. |
| `dTimerEnd` | `void dTimerEnd(void)` | include/ode/timer.h:50 | Closes the session, recording a final TOTAL slot. |
| `dTimerReport` | `void dTimerReport(FILE *fout, int average)` | include/ode/timer.h:56 | Prints a per-slot timing report (time + percentage between checkpoints) to `fout`. `average != 0` prints average per slot, meaningful only when the same Start-Now-End sequence repeats. |
| `dTimerTicksPerSecond` | `double dTimerTicksPerSecond(void)` | include/ode/timer.h:64 | Timer ticks per second implied by the timing hardware/API; header notes actual resolution may be coarser. |
| `dTimerResolution` | `double dTimerResolution(void)` | include/ode/timer.h:69 | Estimate of the actual timer resolution in seconds; may be larger than `1/dTimerTicksPerSecond()`. |

### DIF export (include/ode/export-dif.h)

| Symbol | Signature | Source | Purpose |
| --- | --- | --- | --- |
| `dWorldExportDIF` | `void dWorldExportDIF(dWorldID w, FILE *file, const char *world_name)` | include/ode/export-dif.h:33 | Writes world `w` (bodies, joints, and geoms attached to bodies) to an already-opened `FILE*` in text DIF, prefixing exported symbol names with `world_name`. Does **not** export spaces or standalone geoms (source TODO). |

### Memory hooks (include/ode/memory.h)

| Symbol | Signature | Source | Purpose |
| --- | --- | --- | --- |
| `dAllocFunction` (type) | `typedef void *dAllocFunction(dsizeint size)` | include/ode/memory.h:35 | Signature for a custom allocation callback. |
| `dReallocFunction` (type) | `typedef void *dReallocFunction(void *ptr, dsizeint oldsize, dsizeint newsize)` | include/ode/memory.h:36 | Signature for a custom reallocation callback; receives both old and new sizes for bookkeeping. |
| `dFreeFunction` (type) | `typedef void dFreeFunction(void *ptr, dsizeint size)` | include/ode/memory.h:37 | Signature for a custom free callback; receives the block size. |
| `dSetAllocHandler` | `void dSetAllocHandler(dAllocFunction *fn)` | include/ode/memory.h:41 | Installs a custom allocator; `0` restores the default (`malloc`). |
| `dSetReallocHandler` | `void dSetReallocHandler(dReallocFunction *fn)` | include/ode/memory.h:42 | Installs a custom reallocator; `0` restores the default (`realloc`). |
| `dSetFreeHandler` | `void dSetFreeHandler(dFreeFunction *fn)` | include/ode/memory.h:43 | Installs a custom free; `0` restores the default (`free`). |
| `dGetAllocHandler` | `dAllocFunction *dGetAllocHandler(void)` | include/ode/memory.h:46 | Returns the current alloc handler, or `0` if the default is in place. |
| `dGetReallocHandler` | `dReallocFunction *dGetReallocHandler(void)` | include/ode/memory.h:47 | Returns the current realloc handler, or `0` if the default is in place. |
| `dGetFreeHandler` | `dFreeFunction *dGetFreeHandler(void)` | include/ode/memory.h:48 | Returns the current free handler, or `0` if the default is in place. |
| `dAlloc` | `void *dAlloc(dsizeint size)` | include/ode/memory.h:51 | Allocates `size` bytes via the installed alloc handler, falling back to `malloc`. |
| `dRealloc` | `void *dRealloc(void *ptr, dsizeint oldsize, dsizeint newsize)` | include/ode/memory.h:52 | Reallocates `ptr` (`oldsize` bytes) to `newsize` via the installed handler; default falls back to `realloc(ptr,newsize)` (ignores `oldsize`). |
| `dFree` | `void dFree(void *ptr, dsizeint size)` | include/ode/memory.h:53 | Frees a `size`-byte block via the installed free handler, falling back to `free`. `ptr==0` is a safe no-op. |

### Error / debug / message handlers (include/ode/error.h)

| Symbol | Signature | Source | Purpose |
| --- | --- | --- | --- |
| `dMessageFunction` (type) | `typedef void dMessageFunction(int errnum, const char *msg, va_list ap)` | include/ode/error.h:37 | Signature for error/debug/message handlers; header states error and debug handlers "should not return". |
| `dSetErrorHandler` | `void dSetErrorHandler(dMessageFunction *fn)` | include/ode/error.h:42 | Installs a handler for fatal errors (`dError`); `0` restores the default (print to stderr, then `exit(1)`). |
| `dSetDebugHandler` | `void dSetDebugHandler(dMessageFunction *fn)` | include/ode/error.h:43 | Installs a handler for debug traps (`dDebug`, raised by failed assertions); `0` restores the default (print to stderr, then `abort()`). |
| `dSetMessageHandler` | `void dSetMessageHandler(dMessageFunction *fn)` | include/ode/error.h:44 | Installs a handler for non-fatal warnings (`dMessage`); `0` restores the default (print to stderr, return). |
| `dGetErrorHandler` | `dMessageFunction *dGetErrorHandler(void)` | include/ode/error.h:49 | Returns the current error handler, or `0` if the default is in place. |
| `dGetDebugHandler` | `dMessageFunction *dGetDebugHandler(void)` | include/ode/error.h:50 | Returns the current debug handler, or `0` if the default is in place. |
| `dGetMessageHandler` | `dMessageFunction *dGetMessageHandler(void)` | include/ode/error.h:51 | Returns the current message handler, or `0` if the default is in place. |
| `dError` | `void ODE_NORETURN dError(int num, const char *msg, ...)` | include/ode/error.h:54 | Raises a fatal error: invokes the error handler (or default) with a printf-style message + number, then `exit(1)`. Marked `ODE_NORETURN`. |
| `dDebug` | `void ODE_NORETURN dDebug(int num, const char *msg, ...)` | include/ode/error.h:55 | Raises a debug trap (used by the assertion macros): invokes the debug handler (or default) then `abort()`. Marked `ODE_NORETURN`. |
| `dMessage` | `void dMessage(int num, const char *msg, ...)` | include/ode/error.h:56 | Emits a non-fatal warning via the message handler (or default print to stderr) and returns normally. |

### Library-private assertion macros (ode/src/error.h — NOT in the public header)

| Symbol | Form | Source | Purpose |
| --- | --- | --- | --- |
| `dIASSERT(a)` | `if (!(a)) dDebug(d_ERR_IASSERT, ...)` | ode/src/error.h:48 | Internal consistency check; traps via `dDebug`. Collapses to `((void)0)` under `dNODEBUG`. |
| `dUASSERT(a,msg)` | `if (!(a)) dDebug(d_ERR_UASSERT, msg ...)` | ode/src/error.h:50 | User assertion with a message; traps via `dDebug`. |
| `dDEBUGMSG(msg)` | `dMessage(d_ERR_UASSERT, msg ...)` | ode/src/error.h:52 | Emits a message only (via `dMessage`), no trap. |
| `dAASSERT(a)` | `dUASSERT(a, "Bad argument(s)")` | ode/src/error.h:46 | Argument assert; a special case of `dUASSERT`. |

## Key rules

- The random API is a single shared global stream (a static `volatile duint32 seed` in `misc.cpp`); `dRand`/`dRandInt`/`dRandReal`/`dMakeRandomVector`/`dMakeRandomMatrix` all draw from it. For reproducible runs call `dRandSetSeed(s)` once at startup; to checkpoint/restore the stream, save `dRandGetSeed()` and restore with `dRandSetSeed()` (ode/src/misc.cpp:33).
- Do not rely on `dRand` for cryptographic or high-quality randomness; the header calls it "a not-very-random linear congruential method" and warns `dRandInt`'s distribution "will get worse as n approaches 2^32" (include/ode/misc.h:39).
- `dRandReal` divides by `0xffffffff`, so its range is the **closed** interval `[0,1]` — 1.0 is attainable; do not assume `[0,1)` (include/ode/misc.h:54).
- Pass only static (string-literal or otherwise persistent) strings to `dTimerStart`/`dTimerNow`; the timer stores the `const char*` pointer and does not copy the string, so a stack/temporary buffer dangles by `dTimerReport` time (include/ode/timer.h:48).
- Use code timers as a Start -> (Now ...) -> End sequence, then `dTimerReport`. A session is limited to MAXNUM (100) checkpoints; `dTimerNow`/`dTimerEnd` silently do nothing once the array is full (ode/src/timer.cpp:370).
- `dStopwatchStart`/`Stop` accumulate across calls; call `dStopwatchReset` first for a fresh measurement and read it with `dStopwatchTime` (seconds). Reported resolution is only as fine as `dTimerResolution()`, which can be coarser than `1/dTimerTicksPerSecond()` (include/ode/timer.h:69).
- `dWorldExportDIF` takes an already-opened `FILE*` (you `fopen`/`fclose` it yourself) and only serializes bodies, joints, and geoms attached to bodies; standalone geoms in spaces and the space hierarchy are NOT exported (ode/src/export-dif.cpp:27).
- Memory and error/handler hooks are process-global and must be installed BEFORE any allocation or simulation; passing `fn=0` to any `dSet*Handler` restores ODE's built-in default (include/ode/memory.h:39).
- A custom `dFreeFunction`/`dReallocFunction` receives the block size; if you install a custom allocator you must honor that contract because ODE always passes the size it believes the block has. `dFree(ptr,size)` treats `ptr==0` as a no-op (ode/src/memory.cpp:91).
- Error and debug handlers (`dMessageFunction` for `dError`/`dDebug`) must not return — the header states this and the defaults end in `exit(1)` (`dError`) or `abort()` (`dDebug`). Only `dMessage` handlers may return normally (include/ode/error.h:34).
- The `dIASSERT`/`dUASSERT`/`dAASSERT`/`dDEBUGMSG` macros are library-private (declared in `ode/src/error.h`, NOT the public `include/ode/error.h`) and compile to `((void)0)` under `dNODEBUG`; application code should call `dError`/`dDebug`/`dMessage` or install handlers instead (ode/src/error.h:46).

## Common mistakes

| Bad | Good | Why |
| --- | --- | --- |
| `char buf[64]; sprintf(buf,"step %d",i); dTimerNow(buf);` | `dTimerNow("step"); /* string literal / static string */` | `dTimerStart`/`dTimerNow` store the `const char*` without copying; a stack buffer dangles by the time `dTimerReport` dereferences it (include/ode/timer.h:48). |
| `dStopwatch sw; dStopwatchStart(&sw); ... dStopwatchStop(&sw); double t = dStopwatchTime(&sw);` | `dStopwatch sw; dStopwatchReset(&sw); dStopwatchStart(&sw); ... dStopwatchStop(&sw); double t = dStopwatchTime(&sw);` | An uninitialized `dStopwatch` has garbage in `time`; `Start` does not reset it, so the total is meaningless without a preceding `dStopwatchReset` (include/ode/timer.h:41). |
| `dSetAllocHandler(myAlloc); /* leaving free/realloc default */` | `dSetAllocHandler(myAlloc); dSetReallocHandler(myRealloc); dSetFreeHandler(myFree);` | Blocks from your allocator may be passed to the default `realloc`/`free`, mixing allocators — undefined behavior. Install a consistent set or none (ode/src/memory.cpp:91). |
| `void myError(int n, const char *m, va_list ap){ vprintf(m, ap); /* returns */ }` | `void myError(int n, const char *m, va_list ap){ vfprintf(stderr,m,ap); abort(); }` | The error/debug handler contract is "should not return"; ODE calls them where continuing corrupts state, and the defaults end in `exit`/`abort` (include/ode/error.h:34). |
| `#include <ode/error.h>` … `dIASSERT(x);` | `if (!x) dError(d_ERR_UNKNOWN, "x must hold");  /* or standard assert() */` | `dIASSERT`/`dUASSERT`/`dAASSERT`/`dDEBUGMSG` are not declared in the public header — they live in `ode/src/error.h` and are internal; app code cannot include that file (ode/src/error.h:48). |
| `dWorldExportDIF(world, "world.dif", "sim");` | `FILE *f = fopen("world.dif","w"); dWorldExportDIF(world, f, "sim"); fclose(f);` | The second parameter is a `FILE*`, not a filename; passing a string literal is a type error that crashes on the `fprintf` calls (include/ode/export-dif.h:33). |

## Named constants

| Name | Value | Source | Purpose |
| --- | --- | --- | --- |
| `d_ERR_UNKNOWN` | `0` | include/ode/common.h:375 | "Unknown error" number passed as the first arg to error/debug/message functions. |
| `d_ERR_IASSERT` | `1` (enum) | include/ode/common.h:376 | Internal consistency-check assertion failure; passed to `dDebug` by `dIASSERT`. |
| `d_ERR_UASSERT` | `2` (enum) | include/ode/common.h:377 | User assertion / bad-argument failure; passed to `dDebug` by `dUASSERT`/`dAASSERT` and to `dMessage` by `dDEBUGMSG`. |
| `d_ERR_LCP` | `3` (enum) | include/ode/common.h:378 | Error number reported by the LCP solver when it fails. |
| `ODE_NORETURN` | `__attribute__((noreturn))` / `__declspec(noreturn)` | include/ode/odeconfig.h:75 | Compiler attribute applied to `dError`/`dDebug` to mark them non-returning. |
| `dsizeint` (type) | `typedef size_t dsizeint` | include/ode/odeconfig.h:102 | Unsigned size type used throughout the memory API (`dAlloc`/`dRealloc`/`dFree`). (Build-variant typedefs to `duint64`/`duint32` also exist for other ABIs in the same header.) |
| `MAXNUM` | `100` | ode/src/timer.cpp:330 | Internal (non-public) limit: max `dTimerNow` checkpoints in one `dTimerStart`..`dTimerEnd` session; extra calls are silently dropped. Documented as a hard usage limit, not an exported symbol. |

## Worked examples

```c
/* Deterministic randomness: seed once, save/restore the stream (include/ode/misc.h:45) */
#include <ode/misc.h>

dRandSetSeed(12345);          /* reproducible run */
int r = dRandInt(6);          /* 0..5 */
dReal u = dRandReal();        /* 0..1 (inclusive) */

/* checkpoint and restore the RNG stream */
unsigned long saved = dRandGetSeed();
/* ... draw some numbers ... */
dRandSetSeed(saved);          /* rewind to the checkpoint */
```

```c
/* Code timing: Start -> Now... -> End -> Report (include/ode/timer.h:48) */
#include <ode/timer.h>

dTimerStart("collision");      /* static strings only */
  /* ... collision detection ... */
dTimerNow("step");
  /* ... dWorldQuickStep ... */
dTimerNow("draw");
  /* ... render ... */
dTimerEnd();
dTimerReport(stdout, 0);       /* average=0: per-call times */
```

```c
/* Stopwatch for an accumulated interval (include/ode/timer.h:40) */
dStopwatch sw;
dStopwatchReset(&sw);          /* required: zero the accumulator */
dStopwatchStart(&sw);
  /* ... work ... */
dStopwatchStop(&sw);
printf("%.6f s\n", dStopwatchTime(&sw));
```

```c
/* Custom memory allocator: install before any world/body creation (include/ode/memory.h:41) */
#include <ode/memory.h>

static void *myAlloc(dsizeint size)                    { return malloc(size); }
static void *myRealloc(void *p, dsizeint o, dsizeint n) { (void)o; return realloc(p, n); }
static void  myFree(void *p, dsizeint size)            { (void)size; free(p); }

dSetAllocHandler(myAlloc);
dSetReallocHandler(myRealloc);
dSetFreeHandler(myFree);
/* pass 0 to any of these to restore ODE's malloc/realloc/free defaults */
```

```c
/* Custom error handler that does not return (include/ode/error.h:42) */
#include <ode/error.h>
#include <stdarg.h>

static void onError(int errnum, const char *msg, va_list ap) {
    fprintf(stderr, "ODE error %d: ", errnum);
    vfprintf(stderr, msg, ap);
    fputc('\n', stderr);
    abort();              /* must NOT return */
}

dSetErrorHandler(onError);
/* dSetDebugHandler / dSetMessageHandler are analogous; pass 0 to reset */
```

```c
/* Serialize a world to a DIF file; caller owns the FILE* (include/ode/export-dif.h:33) */
#include <ode/ode.h>          /* pulls in export-dif.h */

FILE *f = fopen("world.dif", "w");
if (f) {
    dWorldExportDIF(world, f, "mysim");  /* prefix exported names with "mysim" */
    fclose(f);
}
```

## Things to never invent

- Do NOT invent a filename-taking variant of `dWorldExportDIF`; the only signature is `dWorldExportDIF(dWorldID, FILE*, const char*)` (include/ode/export-dif.h:33).
- Do NOT claim there is a `dWorldImportDIF` / DIF reader in the public API — no such symbol exists in `include/ode` (only `dWorldExportDIF`).
- Do NOT present `dIASSERT`/`dUASSERT`/`dAASSERT`/`dDEBUGMSG` as part of `include/ode/error.h` — they are only in the private `ode/src/error.h` (ode/src/error.h:48).
- Do NOT invent a per-world or seedable-object RNG; the seed is a single process-global static (ode/src/misc.cpp:33).
- Do NOT claim `dRandReal` returns `[0,1)` exclusive — it divides by `0xffffffff` so 1.0 is attainable (include/ode/misc.h:54).
- Do NOT assert `dStopwatchStart` resets accumulated time — it does not; only `dStopwatchReset` does (include/ode/timer.h:41).
- Do NOT invent a thread-local timer or a way to raise MAXNUM at runtime; the code-timer state is a fixed global array of 100 slots (ode/src/timer.cpp:330).
