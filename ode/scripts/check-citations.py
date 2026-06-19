#!/usr/bin/env python3
"""check-citations.py — validate THIS skill's API citations against YOUR installed ODE.

The skill's `file:line` citations are pinned to the ODE **0.16 source**; your installed library may
differ — line numbers drift (e.g. `dParamVel` is `objects.h:443` in the 0.16 repo but `:442` in 0.16.6),
and a symbol could be removed across versions. This script finds your installed headers via `ode-config`,
then reports:
  (1) any symbol the skill cites that is NOT in your installed ODE (review — version-removed or a typo), and
  (2) a count of line-number drift,
so you know to **trust the symbol, re-grep the installed header for an exact line**. It does NOT modify the
skill. Run from the skill directory:  python3 scripts/check-citations.py
"""
import os, re, glob, subprocess, sys, bisect

SKILL = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))   # this file is <skill>/scripts/

def ode_include():
    for cmd in ("ode-config --cflags", "pkg-config --cflags ode"):
        out = subprocess.run(cmd, shell=True, capture_output=True, text=True).stdout
        m = re.search(r'-I(\S+)', out)
        if m and os.path.isdir(os.path.join(m.group(1), "ode")):
            return m.group(1)
    out = subprocess.run("pkg-config --variable=includedir ode", shell=True, capture_output=True, text=True).stdout.strip()
    if out and os.path.isdir(os.path.join(out, "ode")):
        return out
    return None

inc = ode_include()
if not inc:
    print("Could not locate installed ODE headers via ode-config / pkg-config. Is ODE installed (brew install ode)?")
    sys.exit(2)
hdr = os.path.join(inc, "ode")
ver = subprocess.run("ode-config --version", shell=True, capture_output=True, text=True).stdout.strip() or "?"
print(f"Installed ODE headers: {hdr}   (version {ver})")

ds_dir = os.path.join(inc, "drawstuff")
declared = set(subprocess.run(
    f"grep -rhoE '(d[A-Z][A-Za-z0-9_]+|ds[A-Z][A-Za-z0-9_]+)' {hdr}" + (f" {ds_dir}" if os.path.isdir(ds_dir) else ""),
    shell=True, capture_output=True, text=True).stdout.split())
decl_sorted = sorted(declared)
def is_prefix(t):
    j = bisect.bisect_left(decl_sorted, t)
    return j < len(decl_sorted) and decl_sorted[j].startswith(t)

SYM = re.compile(r'(?<![A-Za-z0-9_])(d[A-Z][A-Za-z0-9_]+|ds[A-Z][A-Za-z0-9_]+)')
NEG = ('never invent', 'do not use', 'removed', 'there is no', 'not a symbol', 'instead of', '->', '→')
md_files = glob.glob(SKILL + "/**/*.md", recursive=True)

# these files survey ODE internals / build macros that are NOT in the public installed headers — skip them
EXCLUDE = {'internals-map.md', 'ecosystem.md', 'build-and-backends.md', 'coding-conventions.md', 'timing-random-dif.md'}
PLACEHOLDER = {'dXxx', 'dXxxID', 'dDoThing', 'dCollideXY', 'dArray', 'dMatrix', 'dReals', 'dYyy', 'dRFrom', 'dQFrom'}
claimed = {}
for f in md_files:
    if os.path.basename(f) in EXCLUDE:
        continue
    skip = False
    for i, line in enumerate(open(f, encoding='utf-8', errors='replace'), 1):
        s = line.strip()
        if s.startswith('#'):
            skip = bool(re.search(r'never invent|do not (invent|use)', s, re.I)); continue
        if skip or any(n in line.lower() for n in NEG):
            continue
        for m in SYM.finditer(line):
            if m.group(1) in PLACEHOLDER:
                continue
            claimed.setdefault(m.group(1), (os.path.relpath(f, SKILL), i))

missing = {t: v for t, v in claimed.items() if t not in declared and not is_prefix(t)}
print(f"\nSymbols cited by the skill: {len(claimed)}   present in your ODE: {len(claimed) - len(missing)}")
if missing:
    print(f"NOT found in your installed ODE {ver} ({len(missing)} — review; version-removed, a typo, or a pattern placeholder):")
    for t, (f, i) in sorted(missing.items()):
        print(f"  {t:36s} {f}:{i}")
else:
    print("All cited symbols exist in your installed ODE.  OK")

cite = re.compile(r'\b([a-z0-9_]+\.h):(\d+)')
checked = drift = 0
for f in md_files:
    for line in open(f, encoding='utf-8', errors='replace'):
        ctx = set(SYM.findall(line))
        if not ctx:
            continue
        for h, ln in cite.findall(line):
            p = os.path.join(hdr, h)
            if not os.path.exists(p):
                continue
            lines = open(p, encoding='utf-8', errors='replace').read().splitlines()
            ln = int(ln)
            if ln > len(lines):
                continue
            checked += 1
            if not (ctx & set(SYM.findall(" ".join(lines[max(0, ln-3):ln+2])))):
                drift += 1
print(f"\nLine cites checked against your installed headers: {checked};  off by >2 lines: {drift}")
print("Reminder: cites are version-approximate — trust the SYMBOL and re-grep the installed header for an exact line.")
sys.exit(1 if missing else 0)
