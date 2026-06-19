// probe_trimesh.cpp — confirm the installed ODE has what a trimesh/heightfield build needs,
// BEFORE you write the program. dCreateTriMesh aborts at runtime if no trimesh backend was
// compiled in, so probe the build instead of assuming it.
//
//   c++ -O2 $(ode-config --cflags) probe_trimesh.cpp $(ode-config --libs) -o probe_trimesh && ./probe_trimesh
//   exit 0 = trimesh + double ok | 1 = no trimesh backend | 2 = wrong precision
#include <ode/ode.h>
#include <cstdio>
int main(void) {
    printf("ODE config: %s\n", dGetConfiguration());          // full build-config token string
    int dbl = dCheckConfiguration("ODE_double_precision");
    int tri = dCheckConfiguration("ODE_EXT_trimesh");
    int opc = dCheckConfiguration("ODE_EXT_opcode");
    int gim = dCheckConfiguration("ODE_EXT_gimpact");
    printf("double_precision = %d\n", dbl);
    printf("trimesh          = %d   (opcode=%d  gimpact=%d)\n", tri, opc, gim);
    printf("sizeof(dReal)    = %zu  (8 = double, 4 = single)\n", sizeof(dReal));
    if (!dbl) { printf("FAIL: not double precision — do NOT hand-define dSINGLE/dDOUBLE; match the lib\n"); return 2; }
    if (!tri) { printf("FAIL: no trimesh backend — dCreateTriMesh would abort. Rebuild ODE with OPCODE/GIMPACT\n"); return 1; }
    printf("OK: trimesh backend present, double precision.\n");
    return 0;
}
