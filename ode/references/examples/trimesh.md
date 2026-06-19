# Walkthrough: trimesh-vs-primitive collision (`demo_trimesh.cpp`)

This example shows how to put a **triangle mesh** into an ODE collision pipeline:
build the vertex/index data, wrap it in a `dTriMeshDataID`, create a trimesh geom
with `dCreateTriMesh`, and collide dynamic primitives (boxes, spheres, capsules,
convex hulls) against it in the `nearCallback`. It also fires a **ray** at the
mesh to demonstrate the ray-vs-trimesh branch.

The trimesh in this demo is a static, downward-pointing **square pyramid** that
acts as the floor — there is no ground plane. Dynamic shapes drop onto it.

Excerpts are verbatim from `ode/demo/demo_trimesh.cpp` and cited by line. The
data-lifecycle rules at the end are grounded in the public header
`include/ode/collision_trimesh.h`. The "What to copy" section distills the recipe.

---

## 1. World and space — `dSimpleSpace`, no ground plane

Unlike `demo_buggy`/`demo_boxstack` (which use a hash space + a ground plane),
the trimesh demo uses a brute-force simple space and **no** plane — the mesh
itself is the floor.

> `ode/demo/demo_trimesh.cpp:511-518`

```c
dInitODE2(0);
world = dWorldCreate();
space = dSimpleSpaceCreate(0);     // simple (brute-force) space, not hash
contactgroup = dJointGroupCreate (0);
dWorldSetGravity (world,0,0,-0.5);
dWorldSetCFM (world,1e-5);
//dCreatePlane (space,0,0,1,0);    // no ground plane: the trimesh pyramid is the floor
```

(`dSimpleSpaceCreate` is the brute-force O(n²) space;
`ode/demo/demo_trimesh.cpp:514`. `dHashSpaceCreate` scales better with many
geoms — `ode/demo/demo_boxstack.cpp:580` — but the trimesh demo has few objects.)

---

## 2. Build the trimesh data and create the geom

This is the core sequence. Fill a vertex array and an index array, allocate a
`dTriMeshDataID`, **build** it from your data, **preprocess** it, then create the
geom from it.

> `ode/demo/demo_trimesh.cpp:521-567`

```c
Size[0]=5.0f; Size[1]=5.0f; Size[2]=2.5f;
// 5 vertices: 4-corner top plane + apex at origin (a square pyramid pointing down)
Vertices[0][0]=-Size[0]; Vertices[0][1]=-Size[1]; Vertices[0][2]=Size[2];
// ... Vertices[1..3] = other top corners, Vertices[4] = {0,0,0} apex ...
Indices[0]=0; Indices[1]=1; Indices[2]=4;   // 4 triangles, 12 indices
Indices[3]=1; Indices[4]=2; Indices[5]=4;
Indices[6]=2; Indices[7]=3; Indices[8]=4;
Indices[9]=3; Indices[10]=0; Indices[11]=4;

dTriMeshDataID Data = dGeomTriMeshDataCreate();
//dGeomTriMeshDataBuildSimple(Data, (dReal*)Vertices, VertexCount, Indices, IndexCount);
dGeomTriMeshDataBuildSingle(Data, Vertices[0], 3 * sizeof(float), VertexCount,
                            &Indices[0], IndexCount, 3 * sizeof(dTriIndex));
dGeomTriMeshDataPreprocess2(Data, (1U << dTRIDATAPREPROCESS_BUILD_FACE_ANGLES), NULL);
TriMesh = dCreateTriMesh(space, Data, 0, 0, 0);  // callbacks (Callback,ArrayCallback,RayCallback)=0
```

Step by step:

1. **`dGeomTriMeshDataCreate()`** allocates an empty data object. You own it,
   not ODE. (`ode/demo/demo_trimesh.cpp:561`;
   `include/ode/collision_trimesh.h:64`.)
2. **`dGeomTriMeshDataBuildSingle(...)`** loads the data. The args are
   `(Data, Vertices, VertexStride, VertexCount, Indices, IndexCount, TriStride)`.
   Here the strides are `3 * sizeof(float)` for vertices and
   `3 * sizeof(dTriIndex)` for triangles — **computed with `sizeof`, never
   hardcoded**, because `dTriIndex` may be 16-bit. The `Single` in the name
   means the vertices are read as `float[3]`. (`ode/demo/demo_trimesh.cpp:564`;
   `include/ode/collision_trimesh.h:120`.) Note the commented-out
   `dGeomTriMeshDataBuildSimple` alternative, which picks single/double from
   `dReal` and takes no strides.
3. **`dGeomTriMeshDataPreprocess2(Data, (1U << dTRIDATAPREPROCESS_BUILD_FACE_ANGLES), NULL)`**
   precomputes face-angle data that aids trimesh-vs-convex collision. The flag
   is passed as `1U << flag`. Call this **after** the build and **before**
   creating the geom. (`ode/demo/demo_trimesh.cpp:565`;
   `include/ode/collision_trimesh.h:195`.)
4. **`dCreateTriMesh(space, Data, 0, 0, 0)`** creates the geom. The three
   trailing zeros are the optional per-triangle / per-array / ray callbacks
   (none here). (`ode/demo/demo_trimesh.cpp:567`;
   `include/ode/collision_trimesh.h:253`.)

**Critical lifecycle rule:** the `Data` object — and the `Vertices`/`Indices`
arrays it references — are **not copied**; they must outlive the geom. Destroy
the geom first, then the data (`dGeomTriMeshDataDestroy`). (Rules:
`include/ode/collision_trimesh.h:64-65`, `:100-103`.)

---

## 3. A ray geom to probe the mesh

The demo also creates a ray and aims it at the pyramid, to show the
ray-vs-trimesh collision branch.

> `ode/demo/demo_trimesh.cpp:571-584`

```c
Ray = dCreateRay(space, 0.9);              // length 0.9
dVector3 Origin, Direction;
Origin[0]=0.0; Origin[1]=0; Origin[2]=0.5; Origin[3]=0;
Direction[0]=0; Direction[1]=1.1f; Direction[2]=-1; Direction[3]=0;
dNormalize3(Direction);                    // ray direction must be unit length
dGeomRaySet(Ray, Origin[0], Origin[1], Origin[2],
                 Direction[0], Direction[1], Direction[2]);
```

- **`dCreateRay(space, 0.9)`** makes a ray geom of length 0.9.
  (`ode/demo/demo_trimesh.cpp:571`.)
- **`dNormalize3(Direction)`** normalizes the direction in place — a ray
  direction must be unit length before `dGeomRaySet`.
  (`ode/demo/demo_trimesh.cpp:582`.)
- **`dGeomRaySet(Ray, px,py,pz, dx,dy,dz)`** sets the ray's origin and
  direction. (`ode/demo/demo_trimesh.cpp:584`.)

---

## 4. The `nearCallback`: ray branch vs. solid-contact branch

The callback first applies the standard joint-exclusion guard and fills a
contact surface, then — for each contact point — **branches on geom class**: if
either geom is a ray, draw the hit and `continue` (no contact joint); otherwise
create a contact joint as usual.

> `ode/demo/demo_trimesh.cpp:133-178`

```c
static void nearCallback (void *, dGeomID o1, dGeomID o2)
{
  dBodyID b1 = dGeomGetBody(o1);
  dBodyID b2 = dGeomGetBody(o2);
  if (b1 && b2 && dAreConnectedExcluding (b1,b2,dJointTypeContact)) return;

  dContact contact[MAX_CONTACTS];
  for (i=0; i<MAX_CONTACTS; i++) {
    contact[i].surface.mode = dContactBounce | dContactSoftCFM;
    contact[i].surface.mu = dInfinity; contact[i].surface.mu2 = 0;
    contact[i].surface.bounce = 0.1; contact[i].surface.bounce_vel = 0.1;
    contact[i].surface.soft_cfm = 0.01;
  }
  if (int numc = dCollide (o1,o2,MAX_CONTACTS,&contact[0].geom,sizeof(dContact))) {
    for (i=0; i<numc; i++) {
      if (dGeomGetClass(o1) == dRayClass || dGeomGetClass(o2) == dRayClass){
        // RAY hit: draw a tiny sphere at the contact point and a line along the normal*depth
        dsDrawSphere(contact[i].geom.pos, Rotation, REAL(0.01));
        // End = pos + normal*depth (per component) ; dsDrawLine(pos, End);
        continue;                     // do NOT make a contact joint for ray hits
      }
      dJointID c = dJointCreateContact (world,contactgroup,contact+i);
      dJointAttach (c,b1,b2);
      if (show_contacts) dsDrawBox (contact[i].geom.pos,RI,ss);
    }
  }
}
```

Key points to internalize:

- **`dAreConnectedExcluding(b1,b2,dJointTypeContact)`** stops two bodies that are
  already joined (by a non-contact joint) from generating contacts — but does
  not suppress new contacts just because they share a contact joint.
  (`ode/demo/demo_trimesh.cpp:133-178`; constant
  `ode/demo/demo_boxstack.cpp:140`.)
- **One surface config drives all contacts**: `dContactBounce | dContactSoftCFM`
  with `mu=dInfinity`, `bounce=0.1`, `bounce_vel=0.1`, `soft_cfm=0.01`. The
  trimesh-vs-primitive case uses the **same** dContact surface machinery as box-
  vs-box; the mesh is "just another geom" to `dCollide`.
- **The ray branch** keys off `dGeomGetClass(...) == dRayClass`. A ray hit is
  visualized (a sphere at `contact[i].geom.pos`, a line along the contact
  normal × depth) and then `continue`s — **no contact joint is created for ray
  hits**, because a ray is a query, not a solid body.
  (`dRayClass`: `ode/demo/demo_trimesh.cpp:158`.)
- For real solid pairs, the usual `dJointCreateContact` +
  `dJointAttach(c,b1,b2)` runs. (`MAX_CONTACTS` here is 40, the demo's array
  cap; `ode/demo/demo_trimesh.cpp:98`.)

---

## 5. simLoop: step, then draw the mesh triangle-by-triangle

The simulation loop is the standard `dSpaceCollide` → `dWorldStep` →
`dJointGroupEmpty`, followed by drawing each dynamic geom and then the static
trimesh face-by-face (drawstuff has no "draw a trimesh geom" call, so the demo
walks `Vertices`/`Indices` itself).

> `ode/demo/demo_trimesh.cpp:436-496`

```c
static void simLoop (int pause)
{
  dsSetColor (0,0,2);
  dSpaceCollide (space,0,&nearCallback);
  if (!pause) dWorldStep (world,0.05);
  //if (!pause) dWorldStepFast (world,0.05, 1);
  dJointGroupEmpty (contactgroup);

  dsSetColor (1,1,0); dsSetTexture (DS_WOOD);
  for (int i=0; i<num; i++)
    for (int j=0; j < GPB; j++) { /* color by selected/disabled */ drawGeom (obj[i].geom[j],0,0,show_aabb); }

  // draw the static trimesh triangles directly from Vertices/Indices
  const dReal* Pos = dGeomGetPosition(TriMesh);
  const dReal* Rot = dGeomGetRotation(TriMesh);
  for (int i = 0; i < IndexCount / 3; i++){
    const float *p = Vertices[Indices[i*3+0]]; const dVector3 v0 = {p[0],p[1],p[2]};
    p = Vertices[Indices[i*3+1]]; const dVector3 v1 = {p[0],p[1],p[2]};
    p = Vertices[Indices[i*3+2]]; const dVector3 v2 = {p[0],p[1],p[2]};
    dsDrawTriangle(Pos, Rot, v0, v1, v2, 0);
  }
  if (Ray){
    dVector3 Origin, Direction; dGeomRayGet(Ray, Origin, Direction);
    dReal Length = dGeomRayGetLength(Ray);
    // End = Origin + Direction*Length ; dsDrawLine(Origin, End);
  }
}
```

- The step uses **`dWorldStep(world,0.05)`** (the exact big-matrix solver);
  `dWorldStepFast` is commented out. (`ode/demo/demo_trimesh.cpp:440`.)
- The mesh is drawn by reading its world transform
  (`dGeomGetPosition`/`dGeomGetRotation(TriMesh)`) and emitting one
  `dsDrawTriangle` per triangle from the source arrays.
- The ray is drawn by reading it back with **`dGeomRayGet`** and
  **`dGeomRayGetLength`** (`ode/demo/demo_trimesh.cpp:484`, `:486`).

---

## What to copy — recipe for a static trimesh collider

1. **Allocate, build, preprocess, create — in that order:**
   ```c
   dTriMeshDataID Data = dGeomTriMeshDataCreate();
   dGeomTriMeshDataBuildSingle(Data, Vertices[0], 3*sizeof(float), VertexCount,
                               &Indices[0], IndexCount, 3*sizeof(dTriIndex));
   dGeomTriMeshDataPreprocess2(Data, (1U << dTRIDATAPREPROCESS_BUILD_FACE_ANGLES), NULL);
   dGeomID TriMesh = dCreateTriMesh(space, Data, 0, 0, 0);
   ```
   (`ode/demo/demo_trimesh.cpp:561-567`.)

2. **Match the build function to your vertex precision, not to `dReal`.**
   `dGeomTriMeshDataBuildSingle` expects `float[3]`;
   `dGeomTriMeshDataBuildDouble` expects `double[3]`;
   `dGeomTriMeshDataBuildSimple` picks single/double from how ODE was compiled
   and takes `dReal` vertices with no stride args. The name selects how bytes
   are read. (`include/ode/collision_trimesh.h:120-145`.)

3. **Always stride with `sizeof`:** `VertexStride = 3*sizeof(float)`,
   `TriStride = 3*sizeof(dTriIndex)`. `dTriIndex` can be 16-bit, so a hardcoded
   `12`/`4` silently mis-strides the arrays.
   (`ode/demo/demo_trimesh.cpp:564`; `include/ode/common.h:78-86`.)

4. **Keep the data and its source arrays alive for the geom's whole lifetime.**
   They are referenced, not copied. On shutdown, `dGeomDestroy(geom)` first, then
   `dGeomTriMeshDataDestroy(Data)`. (`include/ode/collision_trimesh.h:64-65`,
   `:100-103`.)

5. **Collide it like any other geom.** In `nearCallback`, the same `dCollide` +
   `dContact` surface + `dJointCreateContact` flow handles
   trimesh-vs-box/sphere/capsule/convex. No special case is needed unless one
   geom is a **ray** — then branch on `dGeomGetClass(...) == dRayClass`, read the
   hit out of `contact[i].geom`, and **do not** create a contact joint.
   (`ode/demo/demo_trimesh.cpp:133-178`.)

6. **For a *moving* trimesh**, push the previous transform each step with
   `dGeomTriMeshSetLastTransform(geom, last_trans)` so ODE can estimate
   per-triangle velocity for accurate contacts; this is stored per geom, not per
   data. (`include/ode/collision_trimesh.h:114`; pattern in
   `ode/demo/demo_moving_trimesh.cpp`.)

---

## Never invent

- The only build entrypoints are `dGeomTriMeshDataBuildSingle` / `Single1` /
  `Double` / `Double1` / `Simple` / `Simple1`
  (`include/ode/collision_trimesh.h:120-150`). **There is no bare
  `dGeomTriMeshDataBuild`** and no single-call "build + create" helper. This
  demo calls `dGeomTriMeshDataBuildSingle` (line 564), with
  `dGeomTriMeshDataBuildSimple` commented out (line 563) — do not cite a bare
  `dGeomTriMeshDataBuild` to this file.
- Do **not** claim the Build* calls copy the vertex/index arrays — they are
  referenced (`include/ode/collision_trimesh.h:100-103`).
- Do **not** assume `dTriIndex` is always 32-bit / 4 bytes; it can be `duint16`
  (`include/ode/common.h:78-86`).
- In this demo the `c` key creates a **capsule** (`dCreateCapsule`), not a
  cylinder, despite the start() help text saying "c for cylinder"; the cylinder
  branches are commented out. Don't attribute a cylinder shape to this file.
- Whether `dGeomTriMeshDataBuildSingle` (vs a bare `dGeomTriMeshDataBuild`) is
  the currently-recommended public name should be confirmed against
  `include/ode/collision_trimesh.h`. [VERIFY]
- Exact `dsDrawTriangle` / `dsDrawLine` signatures are in
  `include/drawstuff/drawstuff.h`, not in this demo. [VERIFY]
- If you need a trimesh or collision symbol not shown here, grep
  `include/ode/collision_trimesh.h` / `include/ode/collision.h` for it first;
  never paste an API name from memory.
