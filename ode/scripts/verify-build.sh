#!/usr/bin/env sh
# verify-build.sh — prove the shipped ODE example programs BUILD and RUN against YOUR installed ODE.
# Reproduces this skill's "all examples build and run" claim. Needs ode-config (brew install ode) + a c++ compiler.
#   sh scripts/verify-build.sh        # exit 0 = all PASS, 1 = a build/run failed, 2 = no ode-config
cd "$(dirname "$0")/.." || exit 2                      # skill root
command -v ode-config >/dev/null 2>&1 || { echo "ode-config not found — install ODE (brew install ode)"; exit 2; }
CF=$(ode-config --cflags); LB=$(ode-config --libs); CXX=${CXX:-c++}
echo "ODE $(ode-config --version) | $CF $LB"
fail=0
run() {  # label  src  flags (each program carries its own -O2/-std; the .c uses -x c, NOT a c++ std)
  if $CXX $CF $3 "assets/$2" $LB -o "/tmp/odev_$1" 2>/tmp/odev_err; then
    if /tmp/odev_$1 >/tmp/odev_out 2>&1; then echo "  PASS  $2"; else echo "  FAIL(run)   $2"; tail -1 /tmp/odev_out; fail=1; fi
  else echo "  FAIL(build) $2"; sed 's/^/    /' /tmp/odev_err; fail=1; fi
}
echo "Building + running the shipped example programs:"
run hs      headless_example.c "-O2 -x c"
run buggy   buggy_ramp.cpp     "-O2 -std=c++17 -DHEADLESS"
run arm6dof arm6dof.cpp        "-O2 -std=c++17"
run jenga   jenga.cpp          "-O2 -std=c++17"
run dbuggy  demo_buggy.cpp     "-O2 -std=c++17"
run ptm     probe_trimesh.cpp  "-O2 -std=c++17"
echo "Citation drift vs your installed ODE:"
python3 scripts/check-citations.py 2>&1 | sed -n '1,3p' | sed 's/^/  /'
[ "$fail" = 0 ] && { echo "RESULT: all examples build+run PASS"; exit 0; } || { echo "RESULT: FAIL"; exit 1; }
