# Math & Rotation

> Source of truth: include/ode/odemath.h, include/ode/matrix.h, include/ode/rotation.h, include/ode/odemath_legacy.h, include/ode/common.h. Every rule cites real ODE source by file:line; headers win over the wiki on conflict. C symbols start with 'd'; do not invent symbols.

## Mental model

- **Angles are RADIANS.** Every rotation/quaternion builder feeds its angle into `dSin`/`dCos` (which wrap `sinf`/`cosf`). Pass `M_PI`, never `90` (ode/src/rotation.cpp:73-78; include/ode/common.h:286-287).
- **Quaternions are W-FIRST `[w,x,y,z]`.** Index 0 (`dQUE_R`) is the real/scalar part; identity is `{1,0,0,0}`. This is the opposite of glm/Eigen/Bullet's `[x,y,z,w]` (include/ode/common.h:256-262).
- **`dMatrix3` is a 4x3-padded, ROW-MAJOR 3x3.** It is 12 `dReal`s (3 rows of 4); element `(i,j)` is at index `i*4+j`; the 4th column is padding kept at 0. Never `memcpy` it as 9 reals (include/ode/common.h:170-204,272).
- **`dVector3` is 4 reals, not 3.** Index 3 (`dV3E_PAD`) is alignment pad; `sizeof(dVector3) == 4*sizeof(dReal)` (include/ode/common.h:139-147,270).
- **Two API tiers.** Fixed-size **inline** ops in `odemath.h` (the `*3`/`*331`/`*333` family, hot path) versus general **n-sized** dense linear algebra in `matrix.h` (Cholesky / LDLT / solve, flat row-major arrays with `nskip` stride).
- **Transpose digits are an encoding, not magic:** trailing `0`/`1`/`2` selects which operand is transposed; the `_pqr` suffix gives result/operand row counts (include/ode/matrix.h:61-68; include/ode/odemath.h:349-352).

## When to use

- Building a body orientation from an axis+angle, Euler angles, or a target Z/2-axis frame (`dRFrom*`, `dQFrom*`).
- Converting between rotation matrices and quaternions (`dRfromQ`/`dQfromR`) or integrating orientation (`dDQfromW`).
- Rotating / inverse-rotating vectors and composing rotations without round-tripping through a quaternion (`dMultiply*_331`, `dMultiply*_333`).
- Cleaning up numerical drift in an accumulated rotation matrix (`dOrthogonalizeR`) or safely normalizing possibly-degenerate vectors (`dSafeNormalize3/4`).
- Solving small dense symmetric positive-definite systems inside custom constraints/solvers (`dFactorLDLT`/`dSolveLDLT`, `dFactorCholesky`/`dSolveCholesky`).
- Porting old ODE code that still uses the `dDOT`/`dCROSS`/`dMULTIPLY*` legacy macros.

## Public API (C)

### Vectors — set / copy / arithmetic (`odemath.h`, all inline)

| Symbol | Signature | Source | Purpose |
| --- | --- | --- | --- |
| dZeroVector3 | `void dZeroVector3(dVector3 res)` | include/ode/odemath.h:43 | Zero the 3 axis components (pad untouched). |
| dAssignVector3 | `void dAssignVector3(dVector3 res, dReal x, dReal y, dReal z)` | include/ode/odemath.h:50 | Set `res = (x,y,z)`. |
| dCopyVector3 | `void dCopyVector3(dReal *res, const dReal *a)` | include/ode/odemath.h:132 | Copy 3 components `a -> res`. |
| dCopyScaledVector3 | `void dCopyScaledVector3(dReal *res, const dReal *a, dReal nScale)` | include/ode/odemath.h:141 | `res = a * nScale`. |
| dCopyNegatedVector3 | `void dCopyNegatedVector3(dReal *res, const dReal *a)` | include/ode/odemath.h:150 | `res = -a`. |
| dCopyVector4 | `void dCopyVector4(dReal *res, const dReal *a)` | include/ode/odemath.h:159 | Copy 4 components (a matrix row incl. pad). |
| dScaleVector3 | `void dScaleVector3(dReal *res, dReal nScale)` | include/ode/odemath.h:118 | In-place `res *= nScale` (scalar). Not the same as matrix.h `dScaleVector`. |
| dNegateVector3 | `void dNegateVector3(dReal *res)` | include/ode/odemath.h:125 | In-place `res = -res`. |
| dAddVectors3 | `void dAddVectors3(dReal *res, const dReal *a, const dReal *b)` | include/ode/odemath.h:73 | `res = a + b` (aliasing-safe). |
| dSubtractVectors3 | `void dSubtractVectors3(dReal *res, const dReal *a, const dReal *b)` | include/ode/odemath.h:82 | `res = a - b` (aliasing-safe). |
| dAddVectorScaledVector3 | `void dAddVectorScaledVector3(dReal *res, const dReal *a, const dReal *b, dReal b_scale)` | include/ode/odemath.h:91 | `res = a + b_scale*b`. |
| dAddScaledVectors3 | `void dAddScaledVectors3(dReal *res, const dReal *a, const dReal *b, dReal a_scale, dReal b_scale)` | include/ode/odemath.h:100 | `res = a_scale*a + b_scale*b`. |
| dAddThreeScaledVectors3 | `void dAddThreeScaledVectors3(dReal *res, const dReal *a, const dReal *b, const dReal *c, dReal a_scale, dReal b_scale, dReal c_scale)` | include/ode/odemath.h:109 | `res = a_scale*a + b_scale*b + c_scale*c`. |

### Vectors — dot / cross / length / normalize

| Symbol | Signature | Source | Purpose |
| --- | --- | --- | --- |
| dCalcVectorDot3 | `dReal dCalcVectorDot3(const dReal *a, const dReal *b)` | include/ode/odemath.h:219 | Standard 3-vector dot (strides 1,1). = legacy `dDOT`. |
| dCalcVectorDot3_13 | `dReal dCalcVectorDot3_13(const dReal *a, const dReal *b)` | include/ode/odemath.h:220 | Dot, `a` stride 1, `b` stride 3. |
| dCalcVectorDot3_31 | `dReal dCalcVectorDot3_31(const dReal *a, const dReal *b)` | include/ode/odemath.h:221 | Dot, `a` stride 3, `b` stride 1. |
| dCalcVectorDot3_33 | `dReal dCalcVectorDot3_33(const dReal *a, const dReal *b)` | include/ode/odemath.h:222 | Dot, both stride 3. |
| dCalcVectorDot3_14 | `dReal dCalcVectorDot3_14(const dReal *a, const dReal *b)` | include/ode/odemath.h:223 | Dot, `a` stride 1, `b` stride 4 (padded-matrix column). |
| dCalcVectorDot3_41 | `dReal dCalcVectorDot3_41(const dReal *a, const dReal *b)` | include/ode/odemath.h:224 | Dot, `a` stride 4, `b` stride 1. |
| dCalcVectorDot3_44 | `dReal dCalcVectorDot3_44(const dReal *a, const dReal *b)` | include/ode/odemath.h:225 | Dot, both stride 4 (two padded-matrix columns). |
| _dCalcVectorDot3 | `dReal _dCalcVectorDot3(const dReal *a, const dReal *b, unsigned step_a, unsigned step_b)` | include/ode/odemath.h:213 | Internal strided-dot core (leading underscore); the `_XY` wrappers call it. |
| dCalcVectorCross3 | `void dCalcVectorCross3(dReal *res, const dReal *a, const dReal *b)` | include/ode/odemath.h:245 | `res = a x b` (all stride 1). = legacy `dCROSS`. |
| dCalcVectorCross3_114 | `void dCalcVectorCross3_114(dReal *res, const dReal *a, const dReal *b)` | include/ode/odemath.h:246 | Cross, res/a stride 1, b stride 4. |
| dCalcVectorCross3_141 | `void dCalcVectorCross3_141(dReal *res, const dReal *a, const dReal *b)` | include/ode/odemath.h:247 | Cross, res stride 1, a stride 4, b stride 1. |
| dCalcVectorCross3_144 | `void dCalcVectorCross3_144(dReal *res, const dReal *a, const dReal *b)` | include/ode/odemath.h:248 | Cross, res stride 1, a/b stride 4. |
| dCalcVectorCross3_411 | `void dCalcVectorCross3_411(dReal *res, const dReal *a, const dReal *b)` | include/ode/odemath.h:249 | Cross, res stride 4 (write into matrix column), a/b stride 1. |
| dCalcVectorCross3_414 | `void dCalcVectorCross3_414(dReal *res, const dReal *a, const dReal *b)` | include/ode/odemath.h:250 | Cross, res stride 4, a stride 1, b stride 4. |
| dCalcVectorCross3_441 | `void dCalcVectorCross3_441(dReal *res, const dReal *a, const dReal *b)` | include/ode/odemath.h:251 | Cross, res/a stride 4, b stride 1. |
| dCalcVectorCross3_444 | `void dCalcVectorCross3_444(dReal *res, const dReal *a, const dReal *b)` | include/ode/odemath.h:252 | Cross, res/a/b all stride 4. |
| _dCalcVectorCross3 | `void _dCalcVectorCross3(dReal *res, const dReal *a, const dReal *b, unsigned step_res, unsigned step_a, unsigned step_b)` | include/ode/odemath.h:234 | Internal strided-cross core; the `_XYZ` wrappers call it. |
| dAddVectorCross3 | `void dAddVectorCross3(dReal *res, const dReal *a, const dReal *b)` | include/ode/odemath.h:254 | `res += a x b` (accumulate). |
| dSubtractVectorCross3 | `void dSubtractVectorCross3(dReal *res, const dReal *a, const dReal *b)` | include/ode/odemath.h:261 | `res -= a x b`. |
| dSetCrossMatrixPlus | `void dSetCrossMatrixPlus(dReal *res, const dReal *a, unsigned skip)` | include/ode/odemath.h:277 | Write skew matrix `[a]x` (stride `skip`/row) so `res*b = a x b`. Assumes target pre-zeroed. |
| dSetCrossMatrixMinus | `void dSetCrossMatrixMinus(dReal *res, const dReal *a, unsigned skip)` | include/ode/odemath.h:288 | Write `-[a]x` so `res*b = b x a`. Assumes target pre-zeroed. |
| dCalcVectorLength3 | `dReal dCalcVectorLength3(const dReal *a)` | include/ode/odemath.h:192 | Euclidean length `sqrt(a.a)`. = legacy `dLENGTH`. |
| dCalcVectorLengthSquare3 | `dReal dCalcVectorLengthSquare3(const dReal *a)` | include/ode/odemath.h:197 | Squared length `a.a` (no sqrt). = legacy `dLENGTHSQUARED`. |
| dCalcPointsDistance3 | `dReal dCalcPointsDistance3(const dReal *a, const dReal *b)` | include/ode/odemath.h:304 | `|a - b|`. = legacy `dDISTANCE`. |
| dCalcPointDepth3 | `dReal dCalcPointDepth3(const dReal *test_p, const dReal *plane_p, const dReal *plane_n)` | include/ode/odemath.h:202 | Signed plane depth `dot(plane_p - test_p, plane_n)`; positive = penetrating. |
| dSafeNormalize3 | `int dSafeNormalize3(dVector3 a)` | include/ode/odemath.h:521 | Normalize to unit length, return success flag (0 on zero vec). Impl ode/src/odemath.cpp:37. |
| dSafeNormalize4 | `int dSafeNormalize4(dVector4 a)` | include/ode/odemath.h:522 | Safe 4-vector normalize, return success. Impl ode/src/odemath.cpp:42. |
| dNormalize3 | `void dNormalize3(dVector3 a)` | include/ode/odemath.h:523 | In-place unit normalize; may ASSERT on zero. Impl ode/src/odemath.cpp:47. |
| dNormalize4 | `void dNormalize4(dVector4 a)` | include/ode/odemath.h:524 | In-place 4-vector normalize; may ASSERT on zero. Impl ode/src/odemath.cpp:52. |
| dPlaneSpace | `void dPlaneSpace(const dVector3 n, dVector3 p, dVector3 q)` | include/ode/odemath.h:534 | From unit normal `n`, build orthonormal `p,q` (`q = n x p`). Impl ode/src/odemath.cpp:58. |

### Matrices — copy / get / fixed-size multiply (`odemath.h`, all inline)

| Symbol | Signature | Source | Purpose |
| --- | --- | --- | --- |
| dZeroMatrix3 | `void dZeroMatrix3(dMatrix3 res)` | include/ode/odemath.h:57 | Zero the 9 real entries (pad columns untouched). |
| dZeroMatrix4 | `void dZeroMatrix4(dMatrix4 res)` | include/ode/odemath.h:64 | Zero all 16 entries of a `dMatrix4`. |
| dCopyMatrix4x4 | `void dCopyMatrix4x4(dReal *res, const dReal *a)` | include/ode/odemath.h:169 | Copy 3 rows of 4 (12 reals incl. pad). |
| dCopyMatrix4x3 | `void dCopyMatrix4x3(dReal *res, const dReal *a)` | include/ode/odemath.h:176 | Copy only the 3 real columns of each row (pad not copied). |
| dGetMatrixColumn3 | `void dGetMatrixColumn3(dReal *res, const dReal *a, unsigned n)` | include/ode/odemath.h:183 | Extract column `n`: `(a[n], a[n+4], a[n+8])`. |
| dMultiply0_331 | `void dMultiply0_331(dReal *res, const dReal *a, const dReal *b)` | include/ode/odemath.h:354 | `res(3x1) = a(3x3) * b(3x1)`. Rotate a vector by R. |
| dMultiply1_331 | `void dMultiply1_331(dReal *res, const dReal *a, const dReal *b)` | include/ode/odemath.h:359 | `res = a' * b`. Inverse-rotate a vector (R'). |
| dMultiply0_133 | `void dMultiply0_133(dReal *res, const dReal *a, const dReal *b)` | include/ode/odemath.h:364 | `res(1x3) = a(1x3) * b(3x3)`. Row-vector times matrix. |
| dMultiply0_333 | `void dMultiply0_333(dReal *res, const dReal *a, const dReal *b)` | include/ode/odemath.h:369 | `res(3x3) = a(3x3) * b(3x3)`. Compose two rotations. |
| dMultiply1_333 | `void dMultiply1_333(dReal *res, const dReal *a, const dReal *b)` | include/ode/odemath.h:376 | `res = a' * b` (3x3). |
| dMultiply2_333 | `void dMultiply2_333(dReal *res, const dReal *a, const dReal *b)` | include/ode/odemath.h:383 | `res = a * b'` (3x3). |
| dMultiplyAdd0_331 | `void dMultiplyAdd0_331(dReal *res, const dReal *a, const dReal *b)` | include/ode/odemath.h:390 | `res += a(3x3) * b(3x1)`. |
| dMultiplyAdd1_331 | `void dMultiplyAdd1_331(dReal *res, const dReal *a, const dReal *b)` | include/ode/odemath.h:397 | `res += a' * b`. |
| dMultiplyAdd0_133 | `void dMultiplyAdd0_133(dReal *res, const dReal *a, const dReal *b)` | include/ode/odemath.h:404 | `res(1x3) += a(1x3) * b(3x3)`. |
| dMultiplyAdd0_333 | `void dMultiplyAdd0_333(dReal *res, const dReal *a, const dReal *b)` | include/ode/odemath.h:411 | `res(3x3) += a(3x3) * b(3x3)`. |
| dMultiplyAdd1_333 | `void dMultiplyAdd1_333(dReal *res, const dReal *a, const dReal *b)` | include/ode/odemath.h:422 | `res(3x3) += a' * b`. |
| dMultiplyAdd2_333 | `void dMultiplyAdd2_333(dReal *res, const dReal *a, const dReal *b)` | include/ode/odemath.h:433 | `res(3x3) += a * b'`. |
| dCalcMatrix3Det | `dReal dCalcMatrix3Det(const dReal *mat)` | include/ode/odemath.h:444 | Determinant of a 4x3-padded 3x3 (skips pad columns). |
| dInvertMatrix3 | `dReal dInvertMatrix3(dReal *dst, const dReal *ma)` | include/ode/odemath.h:463 | Closed-form 3x3 inverse into `dst`; returns det. Returns 0 and writes nothing if det==0. |
| dOrthogonalizeR | `int dOrthogonalizeR(dMatrix3 m)` | include/ode/odemath.h:536 | Re-orthonormalize `m` into a proper rotation; returns boolean status. Impl ode/src/odemath.cpp:63. |

### Dense linear algebra — n-sized, flat row-major (`matrix.h`)

| Symbol | Signature | Source | Purpose |
| --- | --- | --- | --- |
| dSetZero | `void dSetZero(dReal *a, int n)` | include/ode/matrix.h:38 | Zero an n-element flat array (n = element count). |
| dSetValue | `void dSetValue(dReal *a, int n, dReal value)` | include/ode/matrix.h:39 | Set all n elements to `value`. |
| dDot | `dReal dDot(const dReal *a, const dReal *b, int n)` | include/ode/matrix.h:46 | Dot of two n*1 vectors; returns 0 if n<=0. Variable length, unlike `dCalcVectorDot3`. |
| dMultiply0 | `void dMultiply0(dReal *A, const dReal *B, const dReal *C, int p, int q, int r)` | include/ode/matrix.h:70 | `A = B*C`, row-major; A:p*r B:p*q C:q*r. Impl ode/src/matrix.cpp:533. |
| dMultiply1 | `void dMultiply1(dReal *A, const dReal *B, const dReal *C, int p, int q, int r)` | include/ode/matrix.h:71 | `A = B'*C`; A:p*r B:q*p C:q*r. Impl ode/src/matrix.cpp:539. |
| dMultiply2 | `void dMultiply2(dReal *A, const dReal *B, const dReal *C, int p, int q, int r)` | include/ode/matrix.h:72 | `A = B*C'`; A:p*r B:p*q C:r*q. Impl ode/src/matrix.cpp:545. |
| dFactorCholesky | `int dFactorCholesky(dReal *A, int n)` | include/ode/matrix.h:81 | In-place Cholesky of lower triangle: `L*L'=A`. Returns 1 ok, 0 if not PD. |
| dSolveCholesky | `void dSolveCholesky(const dReal *L, dReal *b, int n)` | include/ode/matrix.h:88 | Solve `L*L'*x=b` in place (overwrites `b`); uses lower triangle of L. |
| dInvertPDMatrix | `int dInvertPDMatrix(const dReal *A, dReal *Ainv, int n)` | include/ode/matrix.h:96 | Invert n*n PD `A` into `Ainv`. Returns 1 ok, 0 if not PD. |
| dIsPositiveDefinite | `int dIsPositiveDefinite(const dReal *A, int n)` | include/ode/matrix.h:105 | 1/0: is `A` positive definite (attempts Cholesky, A unaltered). |
| dFactorLDLT | `void dFactorLDLT(dReal *A, dReal *d, int n, int nskip)` | include/ode/matrix.h:115 | Factor `A = L*D*L'`; L into strict lower triangle of A, RECIPROCALS of D into `d`. Leading dim `nskip`. Impl ode/src/fastldltfactor.cpp:438. |
| dSolveL1 | `void dSolveL1(const dReal *L, dReal *b, int n, int nskip)` | include/ode/matrix.h:122 | Solve `L*x=b` (unit lower-tri L); `b` overwritten. Impl ode/src/fastlsolve.cpp:197. |
| dSolveL1T | `void dSolveL1T(const dReal *L, dReal *b, int n, int nskip)` | include/ode/matrix.h:129 | Solve `L'*x=b` (unit lower-tri L, transposed); `b` overwritten. Impl ode/src/fastltsolve.cpp:196. |
| dScaleVector | `void dScaleVector(dReal *a, const dReal *d, int n)` | include/ode/matrix.h:135 | Elementwise `a = a .* d` over n elements. Impl ode/src/fastvecscale.cpp:173. Not the same as `dScaleVector3`. |
| dVectorScale | `ODE_API_DEPRECATED void dVectorScale(dReal *a, const dReal *d, int n)` | include/ode/matrix.h:140 | DEPRECATED alias of `dScaleVector`. Use `dScaleVector`. Impl ode/src/fastvecscale.cpp:179. |
| dSolveLDLT | `void dSolveLDLT(const dReal *L, const dReal *d, dReal *b, int n, int nskip)` | include/ode/matrix.h:149 | Solve `L*D*L'*x=b` given L and RECIPROCAL-diagonal `d`; `b` overwritten. Pairs with `dFactorLDLT`. Impl ode/src/fastldltsolve.cpp:188. |
| dLDLTAddTL | `void dLDLTAddTL(dReal *L, dReal *d, const dReal *a, int n, int nskip)` | include/ode/matrix.h:165 | Rank update of an L*D*L' factorization by a top-left `[[b,a'],[a,0]]` block. Internal to the LCP solver. |
| dLDLTRemove | `void dLDLTRemove(dReal **A, const int *p, dReal *L, dReal *d, int n1, int n2, int r, int nskip)` | include/ode/matrix.h:185 | Update an L*D*L' factorization of a permuted matrix when row/col `r` is removed (O(n^2)). |
| dRemoveRowCol | `void dRemoveRowCol(dReal *A, int n, int nskip, int r)` | include/ode/matrix.h:193 | Remove row/col `r` of n*n `A` (leading dim `nskip` preserved). |

### Rotation & quaternion builders (`rotation.h`)

| Symbol | Signature | Source | Purpose |
| --- | --- | --- | --- |
| dRSetIdentity | `void dRSetIdentity(dMatrix3 R)` | include/ode/rotation.h:34 | Set `R` to 3x3 identity (pad zeroed). Impl ode/src/rotation.cpp:52. |
| dRFromAxisAndAngle | `void dRFromAxisAndAngle(dMatrix3 R, dReal ax, dReal ay, dReal az, dReal angle)` | include/ode/rotation.h:36 | Build `R` from axis + angle in RADIANS (axis normalized for you). Impl ode/src/rotation.cpp:59. |
| dRFromEulerAngles | `void dRFromEulerAngles(dMatrix3 R, dReal phi, dReal theta, dReal psi)` | include/ode/rotation.h:39 | Build `R` from Euler angles in RADIANS. Impl ode/src/rotation.cpp:69. |
| dRFrom2Axes | `void dRFrom2Axes(dMatrix3 R, dReal ax, dReal ay, dReal az, dReal bx, dReal by, dReal bz)` | include/ode/rotation.h:41 | Col 1 = normalized `a`, col 2 = `b` Gram-Schmidt'd against `a`, col 3 = `a x b`. Returns silently (R unset) if either input is zero length. Impl ode/src/rotation.cpp:94. |
| dRFromZAxis | `void dRFromZAxis(dMatrix3 R, dReal ax, dReal ay, dReal az)` | include/ode/rotation.h:44 | Col 3 (z) = normalized `(ax,ay,az)`; x,y cols an arbitrary orthonormal basis (`dPlaneSpace`). Impl ode/src/rotation.cpp:136. |
| dQSetIdentity | `void dQSetIdentity(dQuaternion q)` | include/ode/rotation.h:46 | Set `q = (1,0,0,0)` i.e. w=1. Impl ode/src/rotation.cpp:159. |
| dQFromAxisAndAngle | `void dQFromAxisAndAngle(dQuaternion q, dReal ax, dReal ay, dReal az, dReal angle)` | include/ode/rotation.h:48 | `q[0]=cos(angle/2)`, vector part = normalized axis * sin(angle/2). Angle in RADIANS; zero axis -> identity. Impl ode/src/rotation.cpp:169. |
| dQMultiply0 | `void dQMultiply0(dQuaternion qa, const dQuaternion qb, const dQuaternion qc)` | include/ode/rotation.h:53 | `qa = qb * qc` (rotate by qc, then qb). Impl ode/src/rotation.cpp:191. |
| dQMultiply1 | `void dQMultiply1(dQuaternion qa, const dQuaternion qb, const dQuaternion qc)` | include/ode/rotation.h:55 | `qa = qb' * qc` (rotate by qc, then inverse of qb). Impl ode/src/rotation.cpp:201. |
| dQMultiply2 | `void dQMultiply2(dQuaternion qa, const dQuaternion qb, const dQuaternion qc)` | include/ode/rotation.h:57 | `qa = qb * qc'` (rotate by inverse of qc, then qb). Impl ode/src/rotation.cpp:211. |
| dQMultiply3 | `void dQMultiply3(dQuaternion qa, const dQuaternion qb, const dQuaternion qc)` | include/ode/rotation.h:59 | `qa = qb' * qc'` (both inverted). Impl ode/src/rotation.cpp:221. |
| dRfromQ | `void dRfromQ(dMatrix3 R, const dQuaternion q)` | include/ode/rotation.h:61 | Quaternion (w-first) -> 3x3 `R`. Note lowercase `from`; alias macro `dQtoR(q,R)`. Impl ode/src/rotation.cpp:236. |
| dQfromR | `void dQfromR(dQuaternion q, const dMatrix3 R)` | include/ode/rotation.h:62 | 3x3 `R` -> quaternion (w-first). Alias macro `dRtoQ(R,q)`. Impl ode/src/rotation.cpp:258. |
| dDQfromW | `void dDQfromW(dReal dq[4], const dVector3 w, const dQuaternion q)` | include/ode/rotation.h:63 | Quaternion time-derivative `dq = 0.5 * omega_quat * q` from angular velocity `w` and orientation `q`. Impl ode/src/rotation.cpp:310. |

### Legacy macros (`odemath_legacy.h`) — backward-compat forwarders only

| Symbol | Signature | Source | Purpose |
| --- | --- | --- | --- |
| dDOT / dDOT13 / dDOT31 / dDOT33 / dDOT14 / dDOT41 / dDOT44 | `#define dDOT(a,b) dCalcVectorDot3(a,b)` (and `_13`.._44 variants) | include/ode/odemath_legacy.h:66-72 | Expand to the `dCalcVectorDot3*` inline family. |
| dCROSS / dCROSSpqr / dCROSS114..444 | `#define dCROSS(a,op,b,c) ...`, `dCROSSpqr(a,op,b,c,p,q,r)` | include/ode/odemath_legacy.h:84-110 | Inline cross-product macros; `op` is normally `=` but can be `+=`/`-=`. `dCROSS` = `dCROSS111`. |
| dCROSSMAT | `#define dCROSSMAT(A,a,skip,plus,minus) ...` | include/ode/odemath_legacy.h:120-128 | Skew cross-product submatrix (assumes A pre-zeroed); plus/minus pick sign. |
| dLENGTH / dLENGTHSQUARED / dDISTANCE | `#define dLENGTH(a) dCalcVectorLength3(a)` etc. | include/ode/odemath_legacy.h:72-74 | Map to `dCalcVectorLength3`/`dCalcVectorLengthSquare3`/`dCalcPointsDistance3`. |
| dMULTIPLY0_331 .. dMULTIPLY2_333 | `#define dMULTIPLY0_331(A,B,C) dMultiply0_331(A,B,C)` (and 1_331, 0_133, 0_333, 1_333, 2_333) | include/ode/odemath_legacy.h:141-146 | Forward to the `dMultiply*_***` inline functions. |
| dMULTIPLYADD0_331 .. dMULTIPLYADD2_333 | `#define dMULTIPLYADD0_331(A,B,C) dMultiplyAdd0_331(A,B,C)` (and 1_331, 0_133, 0_333, 1_333, 2_333) | include/ode/odemath_legacy.h:147-152 | Forward to the `dMultiplyAdd*_***` inline functions. |
| dQtoR / dRtoQ | `#define dQtoR(q,R) dRfromQ((R),(q))`, `#define dRtoQ(R,q) dQfromR((q),(R))` | include/ode/compatibility.h:35-36 | Compat aliases for quaternion<->matrix conversion (note reversed argument order). |

## Key rules

- All rotation and quaternion builders take angles in **RADIANS** (fed to `dSin`/`dCos`, which wrap `sinf`/`cosf`); passing degrees silently produces a wrong rotation (ode/src/rotation.cpp:73-78; include/ode/common.h:286-287).
- Quaternions are **W-FIRST** `[w,x,y,z]`: `q[0]` (`dQUE_R`) is the real part; `dQFromAxisAndAngle` sets `q[0]=cos(angle/2)` and identity is `(1,0,0,0)` (ode/src/rotation.cpp:176-180; include/ode/common.h:256-262).
- `dMatrix3` is row-major 3x3 stored as 3 rows of 4 `dReal`s (12 total); element `(i,j)` is at index `i*4+j` and the 4th column is padding that must stay 0 — never treat it as a tight 9-element array (ode/src/rotation.cpp:35; include/ode/common.h:170-204,272).
- `dVector3` occupies 4 `dReal`s; the 4th (`dV3E_PAD`) is alignment padding to ignore — do not assume `sizeof(dVector3)==3*sizeof(dReal)` (include/ode/common.h:145,270).
- Use `dRfromQ`/`dQfromR` (public spellings, lowercase `from`); `dQtoR(q,R)` and `dRtoQ(R,q)` are compatibility-header macro aliases with reversed argument order (include/ode/rotation.h:61-62; include/ode/compatibility.h:35-36).
- The trailing-digit multiply naming is fixed: `0` = no transpose, `1` = first matrix transposed (`B'`), `2` = second transposed (`C'`); the `_pqr` suffix gives result/operand row counts (e.g. `_331` = res 3x1, a 3x3, b 3x1) (include/ode/matrix.h:61-68; include/ode/odemath.h:349-352).
- Never pass the same buffer for result `A` and input `C` of any `dMultiply*`/`dMultiplyAdd*` routine; in-place `A*=B` is NOT what they compute (include/ode/odemath.h:349-352; include/ode/odemath_legacy.h:135-138).
- `dFactorLDLT` writes the **RECIPROCALS** of D's diagonal into `d` (not D itself); `dSolveLDLT` expects that same reciprocal-diagonal `d`, and `L` is unit-lower-triangular with ones implied (not stored) (include/ode/matrix.h:108-115,143-149).
- LDLT/solve routines use a leading dimension `nskip` = n rounded up to a multiple of 4 (via `dPAD`); pass the matching `nskip` you allocated with, not `n` (include/ode/matrix.h:108-115; include/ode/common.h:91).
- `dSetCrossMatrixPlus`/`Minus` assume the destination submatrix is ALREADY zeroed (they skip the diagonal/zero entries); zero it first or you get garbage on the diagonal (include/ode/odemath.h:269-275).
- Prefer `dSafeNormalize3`/`4` (return a success flag) over `dNormalize3`/`4` when the input might be a zero vector; `dNormalize3`/`4` may assert/abort on zero length (include/ode/odemath.h:521-524).
- `dInvertMatrix3` returns 0 and writes NOTHING when the determinant is zero; always check the return before using `dst` (include/ode/odemath.h:485-505).
- After accumulating many incremental rotations, re-orthonormalize with `dOrthogonalizeR` to remove numerical drift before relying on `R` being a proper rotation (include/ode/odemath.h:535-536).

## Common mistakes

| Bad | Good | Why |
| --- | --- | --- |
| `dRFromAxisAndAngle(R, 0,0,1, 90); // expecting 90 degrees` | `dRFromAxisAndAngle(R, 0,0,1, M_PI*0.5); // angle is RADIANS` | All ODE angle arguments are radians (fed to sinf/cosf via dSin/dCos). Passing degrees silently produces a wildly wrong rotation (ode/src/rotation.cpp:73-78). |
| `dQuaternion q = {0,0,0,1}; // x,y,z,w like glm/Eigen` | `dQuaternion q = {1,0,0,0}; // w,x,y,z -- w (real) is FIRST` | ODE quaternions are w-first (`dQUE_R=0`). Copying an x,y,z,w quaternion from glm/Eigen/Bullet without reordering rotates incorrectly (include/ode/common.h:256-262). |
| `dReal R9[9]; memcpy(R9, bodyR, 9*sizeof(dReal)); // treat dMatrix3 as tight 3x3` | `for(i;i<3)for(j;j<3) R9[i*3+j]=bodyR[i*4+j]; // skip pad column` | `dMatrix3` is 3 rows of 4 (12 reals) with a padding column; a tight 9-element copy mixes rows with pad and corrupts the matrix for a renderer/library expecting 3x3 (include/ode/common.h:170-204,272). |
| `dMultiply0_333(R, R, B); // in-place R = R*B` | `dMatrix3 tmp; dMultiply0_333(tmp, R, B); dCopyMatrix4x3(R, tmp);` | The header warns NEVER to alias the result with the second operand C; the routines read C while writing res, so in-place gives wrong values (include/ode/odemath_legacy.h:135-138). |
| `dFactorLDLT(A,d,n,n); dSolveLDLT(L,d,b,n,n); // nskip = n` | `int nskip = dPAD(n); ... dFactorLDLT(A,d,n,nskip); dSolveLDLT(A,d,b,n,nskip);` | LDLT routines use a leading dimension rounded up to a multiple of 4; nskip=n mismatches the row stride the matrix was laid out with (include/ode/matrix.h:108-115; include/ode/common.h:91). |
| `dSetCrossMatrixPlus(M, a, 4); // M not zeroed` | `dMatrix3 M; dZeroMatrix3(M); dSetCrossMatrixPlus(M, a, 4);` | `dSetCrossMatrixPlus`/`Minus` only write the off-diagonal cross-product entries and assume the rest is already zero; an uninitialized M keeps garbage on the diagonal (include/ode/odemath.h:269-275). |
| `dNormalize3(v); // v might be (0,0,0)` | `if (!dSafeNormalize3(v)) { /* handle degenerate direction */ }` | `dNormalize3` may assert/abort on a zero-length vector; `dSafeNormalize3` returns a status so you can handle the degenerate case gracefully (include/ode/odemath.h:521-524). |

## Named constants

| Name | Value | Source | Purpose |
| --- | --- | --- | --- |
| dSA_X / dSA_Y / dSA_Z / dSA__MAX | 0 / 1 / 2 / 3 | include/ode/common.h:96 | Separate-axis index enum; the base axis ordering all other element enums derive from. |
| dV3E_X / dV3E_Y / dV3E_Z / dV3E_PAD / dV3E__MAX | 0 / 1 / 2 / 3 / 4 | include/ode/common.h:139 | `dVector3` element indices; `dV3E_PAD` is the alignment pad slot. |
| dV4E_X / dV4E_Y / dV4E_Z / dV4E_O | 0 / 1 / 2 / 3 | include/ode/common.h:155 | `dVector4` element indices (`O` = 4th/origin component). |
| dM3E_XX..dM3E_ZZ (+ *PAD) | XX=0,XY=1,XZ=2,XPAD=3,YX=4,YY=5,YZ=6,YPAD=7,ZX=8,ZY=9,ZZ=10,ZPAD=11 | include/ode/common.h:170 | `dMatrix3` named element indices, row-major with pad columns. |
| dM4E_XX..dM4E_OO | 0..15 (4 rows x 4 cols, all used) | include/ode/common.h:216 | `dMatrix4` named element indices. |
| dQUE_R / dQUE_I / dQUE_J / dQUE_K | 0 / 1 / 2 / 3 | include/ode/common.h:256 | Quaternion element indices: `R=0` (real/scalar w), then I,J,K. Confirms w-first layout. |
| dPAD(a) | `(((a)+3)&~3)` for a>1 | include/ode/common.h:91 | Round `a` up to the next multiple of 4 (why matrix leading dimensions / `nskip` round to 4). |
| dRecip / dSqrt / dRecipSqrt / dSin / dCos / dFabs | 1/x, sqrt, 1/sqrt, sin, cos, fabs (dSINGLE/dDOUBLE-dispatched) | include/ode/common.h:283 | Scalar math macros; `dSin`/`dCos` take RADIANS, underpinning the radians convention of all rotation builders (single at 283-288, double at 322-327). |
| dACCESS33(A,i,j) | `((A)[(i)*4+(j)])` | include/ode/odemath.h:32 | Index element `(i,j)` of a 4-stride (padded 3x3) matrix. |

## Types

| Name | Definition | Source | Notes |
| --- | --- | --- | --- |
| dReal | `float` (dSINGLE) or `double` (dDOUBLE) | include/ode/common.h:57 | Base scalar; every vector/matrix/quaternion element is a `dReal`. |
| dVector3 | `dReal[dV3E__MAX]` = `dReal[4]` | include/ode/common.h:270 | 3-vector padded to 4; index 3 (`dV3E_PAD`) is alignment-only. |
| dVector4 | `dReal[dV4E__MAX]` = `dReal[4]` | include/ode/common.h:271 | True 4-vector; also a normalize-4 / quaternion target in some code paths. |
| dMatrix3 | `dReal[dM3E__MAX]` = `dReal[12]` | include/ode/common.h:272 | Row-major 3x3 as 3 rows of 4 (pad column = 0); `(i,j)` at `i*4+j`. |
| dMatrix4 | `dReal[dM4E__MAX]` = `dReal[16]` | include/ode/common.h:273 | Full 4x4 row-major matrix. |
| dMatrix6 | `dReal[6*8]` = `dReal[48]` | include/ode/common.h:274 | 6x6 spatial matrix padded to 6x8; used internally for mass/inertia spatial algebra. |
| dQuaternion | `dReal[dQUE__MAX]` = `dReal[4]` | include/ode/common.h:275 | W-FIRST `[w,x,y,z]`: `dQUE_R=0` real, then I,J,K; identity `(1,0,0,0)`. |

## Things to never invent

- Do not invent a `dRFromQuaternion` or `dQuaternionToMatrix` name; the real spellings are `dRfromQ` and `dQfromR` (lowercase `from`), with macro aliases `dQtoR`/`dRtoQ`.
- Do not claim quaternions are `x,y,z,w`; ODE is w-first (`dQUE_R` index 0).
- Do not state `dMatrix3` is 9 contiguous reals; it is 12 (3 rows of 4 with a pad column).
- Do not invent degree-taking overloads; there is no `dRFromAxisAndAngleDeg` etc. — everything is radians.
- Do not invent a `dMultiply3` or a `_334`/non-listed size suffix; only the `0`/`1`/`2` transpose digits and the suffixes present in odemath.h (`_331`,`_133`,`_333`) and matrix.h (general `p,q,r`) exist.
- Do not assume `dScaleVector` (matrix.h, elementwise `a.*d`) and `dScaleVector3` (odemath.h, scalar*vector) are the same function — they are different signatures in different headers.
- Do not claim `dFactorLDLT` stores D's diagonal; it stores the RECIPROCAL of the diagonal.
- Do not invent a return value for the `void` `dMultiply*`/`dNormalize3`/`dRFrom*` functions; only `dSafeNormalize3/4`, `dOrthogonalizeR`, `dInvertMatrix3`, `dCalcMatrix3Det`, `dFactorCholesky`, `dInvertPDMatrix`, `dIsPositiveDefinite`, `dDot` return values.

## Worked examples

```c
/* Orient a body from an axis-angle (radians) and from a quaternion */
dMatrix3 R;
/* 45 degrees about the world Z axis -- note RADIANS */
dRFromAxisAndAngle(R, 0, 0, 1, M_PI / 4);
dBodySetRotation(body, R);

/* same thing via a w-first quaternion */
dQuaternion q;
dQFromAxisAndAngle(q, 0, 0, 1, M_PI / 4); /* q[0]=cos(angle/2) */
dBodySetQuaternion(body, q);
```
(include/ode/rotation.h:36,48; ode/src/rotation.cpp:169-188)

```c
/* Convert between quaternion and rotation matrix (w-first) */
dQuaternion q = {1, 0, 0, 0}; /* identity: w,x,y,z */
dMatrix3 R;
dRfromQ(R, q);          /* quaternion -> 3x3 (alias: dQtoR(q,R)) */

dQuaternion q2;
dQfromR(q2, R);         /* 3x3 -> quaternion (alias: dRtoQ(R,q2)) */
```
(include/ode/rotation.h:61-62; include/ode/compatibility.h:35-36)

```c
/* Compose rotations and rotate a vector (mind aliasing & padding) */
dMatrix3 Rab;
dMultiply0_333(Rab, Ra, Rb);     /* Rab = Ra * Rb (do NOT pass Rab as Rb) */

dVector3 v_world;
dMultiply0_331(v_world, Rab, v_local);   /* v_world = Rab * v_local */

dVector3 v_back;
dMultiply1_331(v_back, Rab, v_world);    /* inverse-rotate: Rab' * v_world */
```
(include/ode/odemath.h:354-388)

```c
/* Solve a small symmetric PD system with LDLT (correct nskip) */
int n = 6;
int nskip = dPAD(n);            /* leading dimension rounded up to 4 */
dReal *A = (dReal*)dAlloc(nskip * n * sizeof(dReal));
dReal d[6], b[6];
/* ...fill A (row-major, stride nskip) and b... */
dFactorLDLT(A, d, n, nskip);    /* d receives RECIPROCAL diagonal of D */
dSolveLDLT(A, d, b, n, nskip);  /* solves A*x=b, x overwrites b */
```
(include/ode/matrix.h:108-149; include/ode/common.h:91)

```c
/* Re-orthonormalize a drifted rotation matrix */
/* after many incremental updates R may drift from a proper rotation */
if (!dOrthogonalizeR(R)) {
    /* matrix was too degenerate to fix */
}
```
(include/ode/odemath.h:535-536; ode/src/odemath.cpp:63)

## [VERIFY] markers

- **`dRFromEulerAngles` axis order (roll/pitch/yaw):** the (phi,theta,psi) -> roll-about-x / pitch-about-y / yaw-about-z reading is inferred from the matrix form at ode/src/rotation.cpp:79-90; the manual section 8.3 was not reachable in the fetched wiki excerpt. [VERIFY] against the official manual if precise axis-order wording is needed.
- **`dDQfromW` semantics:** described here as `dq = 0.5 * omega_quat * q` from the implementation at ode/src/rotation.cpp:310-317; the header (rotation.h:63) gives only the signature. [VERIFY] the exact derivative form against the implementation if used for integration.
- **`dMatrix6` usage:** the "spatial mass/inertia algebra" purpose is inferred from its 6x8 shape (common.h:274); concrete consumers live in ODE internals (out of public-header scope) and were not exhaustively traced. [VERIFY] if you need its exact internal layout.
