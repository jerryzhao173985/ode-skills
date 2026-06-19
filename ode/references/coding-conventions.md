# ODE Coding Conventions

> Source of truth: CSR.txt (Coding Style Requirements, Oleh Derevenko), include/ode/odeconfig.h, include/ode/common.h, include/ode/error.h, ode/src/error.h. Every rule cites real ODE source by file:line; headers win over the wiki on conflict. C symbols start with 'd'; do not invent symbols.

## Mental model

- Two naming tiers: public symbols use the `d` prefix (`dBodyCreate`, `dReal`); internal object structs use the `dx` prefix (`dxBody`, `dxWorld`) and are exposed to users only as opaque `d...ID` handles.
- Precision is a compile-time choice. Exactly one of `dSINGLE`/`dDOUBLE` is defined per build; you write `dReal` + the `d`-prefixed math macros and the same source compiles for both. Raw `float`/`double`, `sqrtf`, or `0.5` silently break one build.
- DLL/visibility decoration is explicit: every public declaration in `include/ode/*.h` is prefixed `ODE_API`.
- Errors and assertions route through one funnel: `dError`/`dDebug`/`dMessage` call installed handlers; the `dIASSERT`/`dUASSERT`/... macros are built on `dDebug` and all compile out under `dNODEBUG`.
- CSR.txt governs *structure* (one complex construct per function, single exit, no goto, accessor-only fields, canonical do/while(false) validation), not naming or math — those live in the headers.

## When to use

Read this before writing or modifying any C/C++ in `include/ode/**` or `ode/src/**`: adding a public API function, writing math in a `.cpp`, adding assertions, reporting errors, or refactoring a function to house style. Header conventions (d/dx, `ODE_API`, `dReal`, math macros, assertions) are mandatory in the actual codebase; CSR.txt structural rules apply when matching the source's house style.

## Public API (C)

### Error / debug / message reporting (include/ode/error.h)

| Symbol | Signature | Source | Purpose |
|---|---|---|---|
| `dError` | `ODE_API void ODE_NORETURN dError (int num, const char *msg, ...)` | include/ode/error.h:54 | Report a fatal user-visible error and abort. `num` is a `d_ERR_*` code. `ODE_NORETURN`: does not return; routed through the installed error handler. |
| `dDebug` | `ODE_API void ODE_NORETURN dDebug (int num, const char *msg, ...)` | include/ode/error.h:55 | Report an internal-consistency/debug failure and abort (`ODE_NORETURN`). Called by the `dIASSERT`/`dUASSERT`/`dICHECK` macros on failure. Routed through the debug handler. |
| `dMessage` | `ODE_API void dMessage (int num, const char *msg, ...)` | include/ode/error.h:56 | Print a non-fatal informational message and RETURN (NOT `ODE_NORETURN`). Used by `dDEBUGMSG`. Routed through the message handler. |
| `dSetErrorHandler` | `ODE_API void dSetErrorHandler (dMessageFunction *fn)` | include/ode/error.h:42 | Install handler invoked by `dError`. Pass `0` to restore the default. |
| `dSetDebugHandler` | `ODE_API void dSetDebugHandler (dMessageFunction *fn)` | include/ode/error.h:43 | Install handler invoked by `dDebug` (and therefore failed assertions). `0` restores the default. |
| `dSetMessageHandler` | `ODE_API void dSetMessageHandler (dMessageFunction *fn)` | include/ode/error.h:44 | Install handler invoked by `dMessage`/`dDEBUGMSG`. `0` restores the default. |
| `dGetErrorHandler` | `ODE_API dMessageFunction *dGetErrorHandler(void)` | include/ode/error.h:49 | Return currently installed error handler (chain/restore). |
| `dGetDebugHandler` | `ODE_API dMessageFunction *dGetDebugHandler(void)` | include/ode/error.h:50 | Return currently installed debug handler. |
| `dGetMessageHandler` | `ODE_API dMessageFunction *dGetMessageHandler(void)` | include/ode/error.h:51 | Return currently installed message handler. |

### Types

| Symbol | Signature | Source | Purpose |
|---|---|---|---|
| `dReal` | `typedef float dReal;` / `typedef double dReal;` | include/ode/common.h:57,62 | Precision-neutral scalar: `float` under `dSINGLE`, `double` under `dDOUBLE`. ALWAYS write `dReal`, never raw `float`/`double`. |
| `dMessageFunction` | `typedef void dMessageFunction (int errnum, const char *msg, va_list ap)` | include/ode/error.h:37 | Signature every error/debug/message handler must have. |
| `dVector3 / dVector4 / dMatrix3 / dMatrix4 / dMatrix6 / dQuaternion` | `typedef dReal dVector3[dV3E__MAX]` (and siblings) | include/ode/common.h:270-275 | Math types are plain `dReal` arrays; index via the named element enums (`dVec3Element`, `dMat3Element`, common.h:134-267), and note vectors are padded (`dV3E_PAD`) for alignment. |
| `dxBody / dxWorld / dxGeom / dxSpace` (internal struct pattern) | `struct dxBody : public dObject { ... }` etc. | ode/src/objects.h:221 (dxBody), :251 (dxWorld), ode/src/collision_kernel.h:103 (dxGeom), :227 (dxSpace) | Internal `dx`-prefixed structs in `ode/src`, exposed only as opaque `d...ID` handles (e.g. `typedef struct dxBody *dBodyID`). Do NOT reach into `dx` fields from API-level code; use `d`-prefixed accessors. |

## Key rules

- Public API uses the `d` prefix; internal object structs use the `dx` prefix and are exposed only as opaque `d...ID` handles — public-level code goes through `d`-prefixed accessors, never `dx` struct fields (include/ode/collision.h:65 public `dGeomDestroy` vs ode/src/objects.h:221 `struct dxBody`).
- Every public function declaration in `include/ode/*.h` must be prefixed with `ODE_API` for correct DLL export/import (include/ode/odeconfig.h:47-55).
- Write precision-neutral code: use `dReal` for all scalars, `REAL(x)` for every float literal, and the `d`-prefixed math macros (`dSqrt`/`dRecip`/`dFabs`/`dSin`/...) instead of raw float ops or f-suffixed libm calls (include/ode/common.h:56-65).
- Exactly one of `dSINGLE`/`dDOUBLE` must be defined per build: defining both is a `#error`, defining neither is a `#error` in this tree (include/ode/common.h:59) (include/ode/common.h:64).
- Wrap every floating-point literal that feeds a `dReal` expression in `REAL(...)` so single-precision builds do not silently promote to double (`REAL(x)` expands to `(x##f)` under `dSINGLE`) (include/ode/common.h:282).
- Use `dInfinity` / `dNaN` and `dIsNan(x)` for special float values rather than hand-rolling `1.0/0.0` or magic numbers (include/ode/odeconfig.h:153) (include/ode/odeconfig.h:194).
- Assertion discipline: `dIASSERT(a)` for internal-consistency checks (emits `d_ERR_IASSERT` via `dDebug`); `dUASSERT(a,msg)` for user-facing precondition messages (`d_ERR_UASSERT`); `dAASSERT(a)` = `dUASSERT(a,"Bad argument(s)")` for argument validation (ode/src/error.h:46) (ode/src/error.h:97).
- All `dIASSERT`/`dUASSERT`/`dDEBUGMSG` compile to `((void)0)` when `dNODEBUG` is defined — the condition must have NO side effects; use `dIVERIFY`/`dUVERIFY`/`dAVERIFY` when the expression must still evaluate in release builds (ode/src/error.h:65).
- `dICHECK(a)` is a hard internal check that, on failure, calls `dDebug` then crashes via `*(int *)0 = 0` (ode/src/error.h:88).
- Assertions call `dDebug`, never the standard-library `assert()`, so they honor the installed debug handler and the `d_ERR_*` codes and can all be disabled together via `dNODEBUG` (ode/src/error.h:48).
- Compile-time checks use `dSASSERT(e)` / `dSMSGASSERT(e,msg)` — `static_assert` under C++11, a negative-array-size typedef fallback otherwise (ode/src/error.h:78).
- Header layering: public API in `include/ode/*.h`; private/internal helpers in `ode/src/*.h` guarded as private (the assertion macros live behind guard `_ODE__PRIVATE_ERROR_H_`, not in the public API) (ode/src/error.h:25).
- Prefer the portable inline macros `ODE_PURE_INLINE` (`static __inline`, file-local) and `ODE_INLINE` (`__inline`) over a bare `inline` keyword (include/ode/odeconfig.h:65).
- Wrap C API in `ODE_EXTERN_C` (`extern "C"` under C++, empty under C) so symbols are not C++-mangled and link from both languages (include/ode/odeconfig.h:69).
- Decorate compatibility-only API with `ODE_API_DEPRECATED` so new code is steered away from it (include/ode/odeconfig.h:57).
- Decorate non-returning functions with `ODE_NORETURN`; it is applied to `dError` and `dDebug` (include/ode/odeconfig.h:75).

### CSR.txt structural house style (apply when modifying ODE source)

- One complex construct per function: at most one `for`/`while`/`do..while`/`switch`/`try..catch`/`try..finally`, and it must not sit inside a conditional operator (exceptions: nested loop iterating a uniform multi-dimensional array linearly; `try..finally` grouping several resource allocations) (CSR.txt:10).
- No jumps: `goto` and `continue` must not be used (CSR.txt:24).
- Single exit point: a function has exactly one exit at the end; if it returns a value, the `return` is the last syntactic construction (CSR.txt:30).
- Zero means failure/invalid: results use binary-zero as the failure value and types use binary-zero as the invalid element — realized as the error enum starting at `d_ERR_UNKNOWN = 0` and `NULL` opaque IDs (CSR.txt:41) (include/ode/common.h:375).
- Initialize with zero: variables, class fields, and default parameters get the binary-zero representation; public enums have a zero default element (CSR.txt:47).
- Do not reuse variables: a variable holds only the single value described by its name (CSR.txt:56).
- Treat by-value parameters as `const` (exception: when the value loses meaning, e.g. a pointer to an object being deleted) (CSR.txt:64).
- Result-assignment idiom: declare one result variable, assign it as the last meaningful operator in each branch, then return it; copy simple by-reference params to locals at entry and write back at exit (CSR.txt:74).
- Class design: all fields private and accessed only via accessor methods (except in ctors/dtors/init-finalize), ctors/dtors do only trivial zero-init and delegate to methods, callbacks only validate/convert args then call non-public methods, virtual methods are never public (CSR.txt:100).
- No logical-level violations: protected/private methods must not call public methods of their own/ancestor classes; lower-level classes call higher-level ones only via dedicated callbacks (CSR.txt:156).
- Use the canonical function structures for fallible logic: Boolean Function, Validation Function (`do{...break on failure...bResult=true;}while(false)`), side-effect-rollback Validation, and Loop Validation Function (CSR.txt:168).

## Common mistakes

| Bad | Good | Why |
|---|---|---|
| `dReal r = 0.5 * x; float s = sqrtf(x); if (a < 1.0/0.0)` | `dReal r = REAL(0.5) * x; dReal s = dSqrt(x); if (a < dInfinity)` | Raw double literals, f-suffixed libm calls, and hand-rolled infinity break precision neutrality: under `dDOUBLE` `sqrtf` truncates to float, under `dSINGLE` `0.5` forces a double promotion (include/ode/common.h:282). |
| `void dDoThing(dGeomID g);` (public header, no decoration) | `ODE_API void dDoThing(dGeomID g);` | Public functions in `include/ode/*.h` without `ODE_API` are not exported from the Windows DLL and fail to link for consumers (include/ode/odeconfig.h:47). |
| `dIASSERT(pBody = FindBody(id));` (side effect in condition) | `pBody = FindBody(id); dIASSERT(pBody != NULL);` | `dIASSERT`/`dUASSERT`/`dAASSERT` compile to `((void)0)` under `dNODEBUG`, so a side effect in the condition silently vanishes in release; use `dIVERIFY` if it must still run (ode/src/error.h:65). |
| `assert(idx < count);` (standard C assert in ODE source) | `dIASSERT(idx < count);` | ODE assertions go through `dDebug` so they honor the installed debug handler and the `d_ERR_IASSERT` code, and disable together via `dNODEBUG`; plain `assert()` bypasses this (ode/src/error.h:48). |
| `g->body->pos[0] = x;` (reaching into a `dx` struct) | `const dReal *p = dBodyGetPosition(body); dBodySetPosition(body, x, p[1], p[2]);` | `dx`-prefixed structs are internal; API-level code must use the opaque `d...ID` handle and `d`-prefixed accessors, never dereference fields (ode/src/objects.h:221). |
| `int Foo() { for(...){ if(err) return -1; } return 0; }` | `bool Foo() { bool bResult=false; for(...){ if(err){ bAnyFailure=true; break; } } bResult=!bAnyFailure; return bResult; }` | CSR.txt mandates a single exit at the function end and forbids `goto`/`continue`; mid-function returns violate house style (CSR.txt:30). |

## Named constants

### Export / linkage / inline macros (include/ode/odeconfig.h)

| Name | Value | Source | Purpose |
|---|---|---|---|
| `ODE_API` | `__declspec(dllexport)` (MSVC/MinGW + `ODE_DLL`), else empty | include/ode/odeconfig.h:47 | DLL export/visibility decoration prefixing EVERY public function declaration. |
| `ODE_API_DEPRECATED` | `__declspec(deprecated)` / `__attribute__((__deprecated__))` / empty | include/ode/odeconfig.h:57 | Marks a public symbol deprecated; kept for compatibility, not for new code. |
| `ODE_NORETURN` | `__attribute__((noreturn))` (GCC) / `__declspec(noreturn)` (MSVC) / empty | include/ode/odeconfig.h:75 | Decoration for functions that never return; applied to `dError`/`dDebug`. |
| `ODE_PURE_INLINE` | `static __inline` | include/ode/odeconfig.h:65 | File-local inline helper in headers. |
| `ODE_INLINE` | `__inline` | include/ode/odeconfig.h:66 | Portable inline keyword. |
| `ODE_EXTERN_C` | `extern "C"` (C++) / empty (C) | include/ode/odeconfig.h:69 | C linkage so symbols are not C++-mangled across both languages. |

### Special float values (include/ode/odeconfig.h)

| Name | Value | Source | Purpose |
|---|---|---|---|
| `dInfinity` | Positive infinity of active precision (platform `#if/#elif` cascade: `(float)/(double)INFINITY`, else `HUGE_VALF/HUGE_VAL`, else `1.0/0.0`) | include/ode/odeconfig.h:153 | Use for unlimited joint stops / force limits rather than a magic large number. |
| `dNaN` | `(dInfinity - dInfinity)` (cast to active precision) | include/ode/odeconfig.h:194 | Portable quiet NaN; pair with `dIsNan(x)` to test for NaN. |

### Precision-neutral math layer (include/ode/common.h)

| Name | Value | Source | Purpose |
|---|---|---|---|
| `REAL(x)` | `(x##f)` (single) / `(x)` (double) | include/ode/common.h:282 | Precision-correct literal constructor; wrap every float literal. Real usage: ode/src/box.cpp:65 `REAL(0.5) * ...`. |
| `dRecip(x)` | `(1.0f/(x))` / `(1.0/(x))` | include/ode/common.h:283 | Reciprocal. |
| `dSqrt(x)` | `sqrtf(x)` / `sqrt(x)` | include/ode/common.h:284 | Square root. |
| `dRecipSqrt(x)` | `(1.0f/sqrtf(x))` / double form | include/ode/common.h:285 | Reciprocal square root. |
| `dSin(x)` | `sinf(x)` / `sin(x)` | include/ode/common.h:286 | Sine. |
| `dCos(x)` | `cosf(x)` / `cos(x)` | include/ode/common.h:287 | Cosine. |
| `dFabs(x)` | `fabsf(x)` / `fabs(x)` | include/ode/common.h:288 | Absolute value. |
| `dAtan2(y,x)` | `atan2f(y,x)` / `atan2(y,x)` | include/ode/common.h:289 | Arc tangent, 2 args. |
| `dAsin(x)` | `asinf(x)` / `asin(x)` | include/ode/common.h:290 | Arc sine. |
| `dAcos(x)` | `acosf(x)` / `acos(x)` | include/ode/common.h:291 | Arc cosine. |
| `dFMod(a,b)` | `fmodf(a,b)` / `fmod(a,b)` | include/ode/common.h:292 | Modulo. |
| `dFloor(x)` | `floorf(x)` / `floor(x)` | include/ode/common.h:293 | Floor. |
| `dCeil(x)` | `ceilf(x)` / `ceil(x)` | include/ode/common.h:294 | Ceil. |
| `dCopySign(a,b)` | `_ode_copysignf(a,b)` / double form | include/ode/common.h:295 | Copy value sign. |
| `dNextAfter(x,y)` | `_ode_nextafterf(x,y)` / double form | include/ode/common.h:296 | Next representable value. |
| `dMax(a,b)` | `_ode_fmaxf(a,b)` / double form | include/ode/common.h:297 | Maximum. |
| `dMin(a,b)` | `_ode_fminf(a,b)` / double form | include/ode/common.h:298 | Minimum. |
| `dIsNan(x)` | chooses `__isnanf`/`_isnanf`/`isnanf`/`_isnan` via `HAVE_*` macros | include/ode/common.h:300 (single) / 339 (double) | NaN test; prefer over platform-specific spellings. |
| `dPAD(a)` | `(((a) > 1) ? (((a) + 3) & (int)(~3)) : (a))` | include/ode/common.h:91 | Round an int up to a multiple of 4 (0/1 unchanged) for matrix leading dimensions. |

### Error-number enum (include/ode/common.h)

| Name | Value | Source | Purpose |
|---|---|---|---|
| `d_ERR_UNKNOWN` | `0` | include/ode/common.h:375 | Unknown error; canonical zero-means-unknown value (matches CSR.txt:41). |
| `d_ERR_IASSERT` | `1` | include/ode/common.h:376 | Internal assertion failed (emitted by `dIASSERT`/`dICHECK`). |
| `d_ERR_UASSERT` | `2` | include/ode/common.h:377 | User assertion failed (emitted by `dUASSERT`/`dAASSERT`/`dDEBUGMSG`). |
| `d_ERR_LCP` | `3` | include/ode/common.h:378 | LCP solver failure. |

## Things to never invent

- Do NOT claim CSR.txt documents `dReal`/`ODE_API`/`dSqrt`/precision/naming/assertions — it does not (grep over CSR.txt returns none). Those are header conventions, cited to the headers, not to CSR.txt.
- Do not invent assertion macro names beyond the verified set: `dIASSERT`, `dUASSERT`, `dAASSERT`, `dDEBUGMSG`, `dICHECK`, `dIVERIFY`, `dUVERIFY`, `dAVERIFY`, `dSASSERT`, `dSMSGASSERT` (ode/src/error.h).
- Do not invent precision-neutral math macros: the exact set is `REAL`, `dRecip`, `dSqrt`, `dRecipSqrt`, `dSin`, `dCos`, `dFabs`, `dAtan2`, `dAsin`, `dAcos`, `dFMod`, `dFloor`, `dCeil`, `dCopySign`, `dNextAfter`, `dMax`, `dMin`, `dIsNan` (include/ode/common.h:282-347). There is no `dTan` or `dPow` macro defined there.
- Do not invent error-number enum members beyond `d_ERR_UNKNOWN`, `d_ERR_IASSERT`, `d_ERR_UASSERT`, `d_ERR_LCP` (include/ode/common.h:375-378).
- Do not state that ODE removed the `dSINGLE`/`dDOUBLE` `#define` requirement in THIS tree: `common.h:64` still `#error`s when neither is defined. (The web note about `ode/precision.h` applies to the 0.13 release line; verify per-tree.)
- Do not invent CSR.txt section numbers/titles — there are exactly three numbered sections (I General, II Class Design, III Canonical Function Structures) plus a section-III preamble item 0.
- The b-prefix on booleans (`bResult`, `bAnyFailure`) and n-prefix on counts in CSR.txt examples are NOT stated naming rules — they are illustrative copy-style and out-of-scope; mirror surrounding code rather than rely on a rule. Comment phrasing (e.g. `// Some linear code`) is likewise out-of-scope prose, not a hard rule.
