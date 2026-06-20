#!/usr/bin/env python3
"""verify-build.py — build + run every shipped ODE example against YOUR installed ODE.

Reproduces this skill's "all examples build and run" claim. Portable: finds ODE via `ode-config` OR
`pkg-config ode` (same discovery as check-citations.py), uses arg-lists (no shell-quoting fragility) and
a real temp dir (no hardcoded /tmp), and any C++ compiler ($CXX, else clang++/g++/c++).

  python3 scripts/verify-build.py      # exit 0 = all PASS, 1 = a build/run failed, 2 = ODE/compiler missing
"""
import os, sys, shlex, shutil, tempfile, subprocess

SKILL = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

def discover_ode():
    """Return (cflags_list, libs_list, version, via) using ode-config or pkg-config — whichever works."""
    for via, base, verarg in (("ode-config", "ode-config", "--version"),
                              ("pkg-config", "pkg-config ode", "--modversion")):
        if not shutil.which(base.split()[0]):
            continue
        cf = subprocess.run(f"{base} --cflags", shell=True, capture_output=True, text=True)
        lf = subprocess.run(f"{base} --libs", shell=True, capture_output=True, text=True)
        if cf.returncode == 0 and "-I" in cf.stdout:
            verbase = "ode-config" if via == "ode-config" else "pkg-config ode"
            ver = subprocess.run(f"{verbase} {verarg}", shell=True, capture_output=True, text=True).stdout.strip()
            return shlex.split(cf.stdout), shlex.split(lf.stdout), ver or "?", via
    return None, None, None, None

cf, lf, ver, via = discover_ode()
if cf is None:
    print("ODE not found via ode-config or pkg-config — install ODE (macOS: brew install ode; "
          "Debian/Ubuntu: apt install libode-dev).")
    sys.exit(2)
CXX = os.environ.get("CXX") or next((c for c in ("c++", "clang++", "g++") if shutil.which(c)), None)
if not CXX:
    print("No C++ compiler found — set $CXX or install clang++/g++.")
    sys.exit(2)
print(f"ODE {ver} via {via} | CXX={CXX} | {' '.join(cf)} {' '.join(lf)}")

# (source, compile flags). The .c is built with the c++ DRIVER + `-x c` (libode is C++) — NOT a c++ -std.
EXAMPLES = [
    ("headless_example.c", ["-O2", "-x", "c"]),
    ("buggy_ramp.cpp",     ["-O2", "-std=c++17", "-DHEADLESS"]),
    ("arm6dof.cpp",        ["-O2", "-std=c++17"]),
    ("jenga.cpp",          ["-O2", "-std=c++17"]),
    ("demo_buggy.cpp",     ["-O2", "-std=c++17"]),
    ("probe_trimesh.cpp",  ["-O2", "-std=c++17"]),
    ("probe_sign.cpp",     ["-O2", "-std=c++17"]),
    ("probe_sign2.cpp",    ["-O2", "-std=c++17"]),
]
print("Building + running the shipped example programs:")
tmp = tempfile.mkdtemp(prefix="odeverify_")
fail = 0
try:
    for src, flags in EXAMPLES:
        path = os.path.join(SKILL, "assets", src)
        if not os.path.exists(path):
            print(f"  SKIP (missing)  {src}"); continue
        out = os.path.join(tmp, os.path.splitext(src)[0])
        b = subprocess.run([CXX, *cf, *flags, path, *lf, "-o", out], capture_output=True, text=True)
        if b.returncode != 0:
            print(f"  FAIL(build)  {src}")
            print("    " + b.stderr.strip()[:400].replace("\n", "\n    ")); fail = 1; continue
        try:
            r = subprocess.run([out], capture_output=True, text=True, timeout=120)
            tail = ((r.stdout + r.stderr).strip().splitlines() or [""])[-1]
            print(f"  {'PASS ' if r.returncode == 0 else 'FAIL(run) '} {src}   {tail[-60:]}")
            if r.returncode != 0:
                fail = 1
        except subprocess.TimeoutExpired:
            print(f"  FAIL(timeout)  {src}"); fail = 1
finally:
    shutil.rmtree(tmp, ignore_errors=True)

cc = os.path.join(SKILL, "scripts", "check-citations.py")
if os.path.exists(cc):
    out = subprocess.run([sys.executable, cc], capture_output=True, text=True).stdout.splitlines()
    line = next((l for l in out if l.startswith("Symbols cited")), "")
    if line:
        print(f"Citation drift vs your installed ODE: {line}")

print("RESULT:", "all examples build+run PASS" if not fail else "FAIL")
sys.exit(1 if fail else 0)
