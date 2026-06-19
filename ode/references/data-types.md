# ODE Data Types & Precision

> Source of truth: include/ode/common.h, include/ode/odeconfig.h, include/ode/precision.h.in. Every rule cites real ODE source by file:line; headers win over the wiki on conflict. C symbols start with 'd'; do not invent symbols.

## Mental model

- ODE is built at **one** precision. `dReal` is `float` under `dSINGLE` and `double` under `dDOUBLE`; you MUST define exactly one, and your code and the linked `libode` must agree or you get silent corruption (include/ode/common.h:56).
- All ODE math runs through `dReal` and the fixed-width array typedefs (`dVector3`, `dMatrix3`, `dQuaternion`); never hand ODE raw `float`/`double` arrays (include/ode/common.h:57).
- Two layout gotchas dominate: **`dVector3` is 4 `dReal`s wide** (index 3 is alignment pad, not a usable component), and **`dQuaternion` is `[w,x,y,z]`** with the real part first (include/ode/common.h:270, include/ode/common.h:256).
- Object handles (`dWorldID`, `dBodyID`, ...) are **opaque pointers** to internal `dx*` structs — pass them around, never dereference (include/ode/common.h:364).
- "No limit" / "infinite force" is the precision-correct `dInfinity` macro, and literal constants want `REAL(x)` so they match the active precision (include/ode/odeconfig.h:153).
- The full math/rotation function surface (90+ helpers: `dCalc*`, `dMultiply*`, `dR*`, `dQ*`) lives in `references/math-and-rotation.md` — this file covers the **types**, precision switch, and the handful of helpers that disambiguate the type contracts.

## When to use

- Declaring any variable ODE will read or write: positions, velocities, rotations, mass, contact buffers.
- Setting up the build: choosing single vs double and matching it to the prebuilt/linked library.
- Storing or copying poses (vector/matrix/quaternion layout, padding, element order).
- Holding object references (world/body/joint/space/geom IDs) in your own structs.
- Passing "unlimited" joint stops, force caps, or ERP/CFM-style magnitudes that need `dInfinity` / `REAL()`.

## Public API (C)

### Precision-correct scalar & precision detection

| Symbol | Signature | Source | Purpose |
| --- | --- | --- | --- |
| `dReal` | `typedef float dReal;` (dSINGLE) / `typedef double dReal;` (dDOUBLE) | include/ode/common.h:57 | The scalar float type all ODE math uses; resolved by the precision macros. |
| `dGetConfiguration` | `const char* dGetConfiguration (void)` | include/ode/common.h:563 | Returns the build-config token string (e.g. `ODE_single_precision` / `ODE_double_precision`) to detect precision at runtime. |
| `dCheckConfiguration` | `int dCheckConfiguration( const char* token )` | include/ode/common.h:573 | Returns 1 if the given (case-sensitive) config token is present in the build. |

### Math value types (fixed-width `dReal` arrays)

| Symbol | Signature | Source | Purpose |
| --- | --- | --- | --- |
| `dVector3` | `typedef dReal dVector3[dV3E__MAX];` | include/ode/common.h:270 | 4 `dReal`s: X,Y,Z + 1 PAD for alignment. **Index 3 is padding, not a 4th component.** |
| `dVector4` | `typedef dReal dVector4[dV4E__MAX];` | include/ode/common.h:271 | 4 `dReal`s (x,y,z,o); a true 4-element vector. |
| `dMatrix3` | `typedef dReal dMatrix3[dM3E__MAX];` | include/ode/common.h:272 | 12 `dReal`s: a 3x3 rotation matrix, row-major, **4 `dReal`s per row** (4th column is pad). |
| `dMatrix4` | `typedef dReal dMatrix4[dM4E__MAX];` | include/ode/common.h:273 | 16 `dReal`s (4x4 matrix). |
| `dMatrix6` | `typedef dReal dMatrix6[(dMD__MAX * dV3E__MAX) * (dMD__MAX * dSA__MAX)];` | include/ode/common.h:274 | 36 `dReal`s; spatial 6x6 matrix (linear+angular), mostly internal. |
| `dQuaternion` | `typedef dReal dQuaternion[dQUE__MAX];` | include/ode/common.h:275 | 4 `dReal`s ordered `[w(R), x(I), y(J), z(K)]` — **real part first.** |

### Opaque object ID handles (never dereference)

| Symbol | Signature | Source | Purpose |
| --- | --- | --- | --- |
| `dWorldID` | `typedef struct dxWorld *dWorldID;` | include/ode/common.h:364 | Handle to a dynamics world. |
| `dSpaceID` | `typedef struct dxSpace *dSpaceID;` | include/ode/common.h:365 | Handle to a collision space. |
| `dBodyID` | `typedef struct dxBody *dBodyID;` | include/ode/common.h:366 | Handle to a rigid body / dynamics object. |
| `dGeomID` | `typedef struct dxGeom *dGeomID;` | include/ode/common.h:367 | Handle to a collision geometry. |
| `dJointID` | `typedef struct dxJoint *dJointID;` | include/ode/common.h:368 | Handle to a joint. |
| `dJointGroupID` | `typedef struct dxJointGroup *dJointGroupID;` | include/ode/common.h:369 | Handle to a joint group (batch-free contacts/joints). |

### Aggregate / enum types

| Symbol | Signature | Source | Purpose |
| --- | --- | --- | --- |
| `dMass` | `struct dMass { dReal mass; dVector3 c; dMatrix3 I; ... };` | include/ode/mass.h:88 | Mass struct: total mass, center of gravity `c`, inertia tensor `I`. |
| `dJointFeedback` | `typedef struct dJointFeedback { dVector3 f1; dVector3 t1; dVector3 f2; dVector3 t2; } dJointFeedback;` | include/ode/common.h:516 | Forces/torques applied to the two bodies of a joint (f1/t1 to body 1, f2/t2 to body 2). |
| `dJointType` | `typedef enum { dJointTypeNone=0, dJointTypeBall, dJointTypeHinge, ... } dJointType;` | include/ode/common.h:384 | Enum of joint kinds (Ball, Hinge, Slider, Contact, Universal, Hinge2, Fixed, AMotor, LMotor, ...). |
| `dTriIndex` | `typedef duint32 dTriIndex;` | include/ode/common.h:85 | Index type for trimesh data (`duint32` or `duint16` per build flags). |
| `duint32` / `duint16` | `typedef uint32_t duint32;` (and `duint16`) | include/ode/odeconfig.h:93 | Fixed-width unsigned integer typedefs used across ODE. |

### Type-disambiguating helpers (full math surface → math-and-rotation.md)

| Symbol | Signature | Source | Purpose |
| --- | --- | --- | --- |
| `dNormalize3` | `void dNormalize3 (dVector3 a)` | include/ode/odemath.h:523 | Scale a 3-vector to unit length in place; **potentially asserts** on a zero vector. |
| `dNormalize4` | `void dNormalize4 (dVector4 a)` | include/ode/odemath.h:524 | Scale a 4-vector (incl. quaternion-as-4-vector) to unit length; **potentially asserts** on zero. |
| `dSafeNormalize3` | `int dSafeNormalize3 (dVector3 a)` | include/ode/odemath.h:521 | Normalize a 3-vector, returning a status int instead of asserting on a zero vector. |
| `dSafeNormalize4` | `int dSafeNormalize4 (dVector4 a)` | include/ode/odemath.h:522 | Normalize a 4-vector, returning a status int instead of asserting. |
| `dQSetIdentity` | `void dQSetIdentity (dQuaternion q)` | include/ode/rotation.h:46 | Set a quaternion to the identity rotation `[1,0,0,0]` (real part first). |
| `dRSetIdentity` | `void dRSetIdentity (dMatrix3 R)` | include/ode/rotation.h:34 | Set a 3x3 matrix to the identity rotation. |

## Key rules

- Define **exactly one** of `dSINGLE` or `dDOUBLE` for the build: defining both is a compile `#error` and defining neither is also a compile `#error` (include/ode/common.h:56).
- `dReal` is `float` under `dSINGLE` and `double` under `dDOUBLE`; application code and any data shared with ODE must be typed `dReal`, and the **library you link must be built with the same precision** as your code — mixing is a classic crash (include/ode/common.h:57).
- `precision.h.in` is configured at build time: `dIDESINGLE`→`dSINGLE`, `dIDEDOUBLE`→`dDOUBLE`, else `@ODE_PRECISION@` is substituted; the chosen macro propagates through `common.h` (include/ode/precision.h.in:8).
- Use `dInfinity` (precision-correct positive infinity) for "no limit" values; write floating constants with `REAL(x)` so they match the active precision (include/ode/odeconfig.h:153).
- `dVector3` is 4 `dReal`s: elements 0..2 are X,Y,Z and element 3 (`dV3E_PAD`) is alignment padding — allocate/copy 4 `dReal`s; matrix rows are likewise 4-wide, so `dMatrix3` is 12 `dReal`s with stride 4 per row (include/ode/common.h:270).
- `dQuaternion` order is `[w,x,y,z]` (real part is element 0 = `dQUE_R`); identity is `[1,0,0,0]` via `dQSetIdentity` (include/ode/common.h:256).
- For the matrix-multiply (`dMultiply*`) and cross helpers, **never pass the same buffer as both an input operand and the result** — it is NOT equivalent to `A*=B` / in-place (include/ode/odemath.h:349).
- `dNormalize3`/`dNormalize4` may assert on a zero-length vector; use `dSafeNormalize3`/`dSafeNormalize4` (which return an int status) when the input might be zero (include/ode/odemath.h:521).
- ID handles (`dWorldID`, `dBodyID`, `dGeomID`, `dJointID`, `dSpaceID`, `dJointGroupID`) are opaque pointers to `dx*` structs — never dereference them; pass them to the `d*` API and destroy via the owning subsystem's destroy call (include/ode/common.h:364).
- The `dCalc*`/`dMultiply*` helpers are `ODE_PURE_INLINE` (static inline, header-only); the `d`-prefixed normalize/plane/rotation entry points are `ODE_API` exported symbols (include/ode/odeconfig.h:65).

## Common mistakes

Precision mismatch (app vs linked `libode`) and quaternion order (`[w,x,y,z]`, real first) are SKILL hard rules 7 and 10 — see `SKILL.md` "Hard rules". The two below are type-layout traps unique to this card:

| Bad | Good | Why |
| --- | --- | --- |
| `dReal v[3]; memcpy(dst, v, 3*sizeof(dReal));` — passing 3-wide arrays as `dVector3`. | Declare `dVector3 v;` (4 `dReal`s) and copy 4 elements; remember `dMatrix3` rows are 4 wide (`a+4`, `a+8`). | `dVector3` is 4 `dReal`s (index 3 is alignment padding), not 3 — treating it as 3 breaks alignment, copying, and matrix-row stride (include/ode/common.h:270). |
| `dJointSetHingeParam(j, dParamFMax, 1e30);` (or `DBL_MAX` / `HUGE_VAL`). | Use `dInfinity` (and write other constants with `REAL(x)`) so values match the active precision. | Hardcoding a "big number" for infinite limits/forces is precision-wrong and can overflow a `float` build; the API expects the `dInfinity` macro (include/ode/odeconfig.h:153). |

## Named constants

| Name | Value | Source | Purpose |
| --- | --- | --- | --- |
| `dSINGLE` | build macro (defined → `dReal`=`float`) | include/ode/common.h:56 | Single-precision build; mutually exclusive with `dDOUBLE` (compile `#error` if both). |
| `dDOUBLE` | build macro (defined → `dReal`=`double`) | include/ode/common.h:61 | Double-precision build; you MUST define exactly one of `dSINGLE`/`dDOUBLE` or compilation `#error`s. |
| `dInfinity` | precision-correct `+INF` (float/double cast) | include/ode/odeconfig.h:153 | Positive infinity; use for unlimited joint stops/forces, never a raw double literal. |
| `dNaN` | precision-correct quiet NaN | include/ode/odeconfig.h:181 | Precision-correct NaN macro. |
| `REAL(x)` | `(x##f)` under `dSINGLE`, `(x)` under `dDOUBLE` | include/ode/common.h:282 | Write a precision-correct floating constant. |
| `M_PI` | `REAL(3.1415926535897932384626433832795029)` | include/ode/common.h:44 | Pi as a `REAL()` constant, defined here if `<math.h>` lacks it (also `M_PI_2`, `M_SQRT1_2`). |
| `dDOT` (legacy) | `#define dDOT(a,b) dCalcVectorDot3(a,b)` | include/ode/odemath_legacy.h:77 | Legacy macro aliasing `dCalcVectorDot3`; `dDOT13/31/33/14/41/44` alias strided variants. Prefer `dCalc*` names. |
| `dCROSS` (legacy) | `#define dCROSS(a,op,b,c) ...` | include/ode/odemath_legacy.h:93 | Legacy cross-product macro with an operator (`=`,`+=`,`-=`); plus `dCROSSpqr`/`dCROSS114...444` forms. Prefer `dCalcVectorCross3`. |
| `dMULTIPLY0_331` (legacy) | `#define dMULTIPLY0_331(A,B,C) dMultiply0_331(A,B,C)` | include/ode/odemath_legacy.h:140 | Legacy uppercase macros aliasing the `dMultiply*` inline matrix-multiply functions. |
| `dV3E_X` / `dV3E_Y` / `dV3E_Z` / `dV3E_PAD` | X=0,Y=1,Z=2,PAD=3 (`dV3E__MAX`=4) | include/ode/common.h:139 | Named indices into a `dVector3`; `dV3E__MAX`=4 is the array length, confirming the 4th slot is padding. |
| `dQUE_R` / `dQUE_I` / `dQUE_J` / `dQUE_K` | R=0,I=1,J=2,K=3 | include/ode/common.h:256 | Named quaternion indices; R=real(0) confirms the real part is element 0. |
| `dParamLoStop` / `dParamHiStop` / `dParamVel` / `dParamFMax` / ... | enum via `D_ALL_PARAM_NAMES` | include/ode/common.h:439 | Joint-parameter id enum; add `dParamGroup` (0x100) per extra axis (e.g. `dParamFMax2`). |
| `dAMotorUser` / `dAMotorEuler` | 0 and 1 | include/ode/common.h:500 | Angular-motor mode constants. |

## Things to never invent

These symbols do **not** exist — do not emit them; the grounded replacements are given:

- `dVec3` — the type is `dVector3`, with index names `dV3E_X`/`dV3E_Y`/`dV3E_Z`.
- `dRealInfinity` / `dInfinityf` — the only macro is `dInfinity`.
- `dRToQ` / `dQToR` — actual names are `dRfromQ` and `dQfromR` (note the lowercase `from`).
- `dQtoR` / `dRtoQ` — do NOT exist; use `dQfromR` / `dRfromQ`.
- `dEpsilon` — no such constant in any `include/ode` header.
- `dPI` — does not exist; pi is `M_PI` via `REAL()`, and `dPI` is not defined.
- `dVector3Set` / `dSetVector3` — use `dAssignVector3` or `dZeroVector3`.
- `dCross3` / `dDot3` as standalone functions — the C-API inline names are `dCalcVectorCross3` / `dCalcVectorDot3`; `dCROSS`/`dDOT` are legacy macros.
- `dRealMax` / `dRealMin` — per-element helpers are `dMax`/`dMin` macros over scalars.
- `dQuaternionNormalize` — normalize quaternions via the `dNormalize4` path on the 4-vector; there is no dedicated quaternion-normalize public symbol.
