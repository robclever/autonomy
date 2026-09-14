#!/usr/bin/env python3
"""Enforce module dependency layering for slam_sim.

The simulation flow (see .README) is a *runtime* data-flow cycle:

    environment -> sensors -> fusion -> autonomy -> platform -> environment

To keep *compile-time* dependencies acyclic, each module may only #include
project headers from modules it is allowed to depend on.  Modules communicate
through data types defined in core/ (e.g. Pose, Observation), and the sim/
orchestrator wires the actual loop.  The environment receives the platform's
pose as a parameter rather than #including the platform header.

This script scans every #include "..." and flags violations.  Exit code is
non-zero if any are found, so it can gate CI.
"""

import os
import re
import sys
from pathlib import Path

# Module -> set of modules it MAY depend on.  stdlib / third_party headers
# (no project module prefix) are always allowed and not listed here.
ALLOWED = {
    "core": set(),
    "environment": {"core"},
    "sensors": {"core", "environment"},
    "fusion": {"core", "sensors"},
    "autonomy": {"core", "fusion"},
    "platform": {"core", "autonomy"},
    "sim": {
        "core", "environment", "sensors", "fusion", "autonomy", "platform",
    },
    "evaluation": {
        "core", "environment", "sensors", "fusion", "autonomy", "platform",
    },
    "visualization": {
        "core", "environment", "sensors", "fusion", "autonomy", "platform",
    },
    "config": {"core"},
    "tests": {
        "core", "environment", "sensors", "fusion", "autonomy", "platform",
        "sim", "evaluation", "visualization", "config",
    },
}

# core's include layout is flattened: core/include/{math,kinematics,time,measurement}/
# so an include like "math/Vec3.h" is a core-to-core include.
CORE_SUBDIRS = {"math", "kinematics", "time", "measurement"}

# Matches: #include "module/subpath.h"
INC_RE = re.compile(r'^\s*#\s*include\s+"([^"]+)"')

# Directories to skip while walking the tree.
SKIP_DIRS = {"build", "third_party", ".git", "__pycache__"}


def module_of(path_str):
    """Return the top-level module for a project path, else None."""
    parts = Path(path_str).parts
    if not parts:
        return None
    if parts[0] in ALLOWED:
        return parts[0]
    if parts[0] in CORE_SUBDIRS:
        return "core"
    return None


def scan_file(root, filepath):
    """Return (line_no, from_module, to_module, include) for each violation."""
    rel = Path(filepath).relative_to(root)
    from_mod = module_of(rel)
    if from_mod is None:
        return []

    violations = []
    with open(filepath) as f:
        for i, line in enumerate(f, 1):
            m = INC_RE.match(line)
            if not m:
                continue
            inc = m.group(1)
            to_mod = module_of(inc)
            if to_mod is None:
                continue                # stdlib / third_party — allowed
            if to_mod == from_mod:
                continue                # same-module include — fine
            if to_mod not in ALLOWED.get(from_mod, set()):
                violations.append((i, from_mod, to_mod, inc))
    return violations


def main():
    root = Path(sys.argv[1] if len(sys.argv) > 1 else ".").resolve()
    exts = {".cpp", ".h", ".hpp"}
    all_violations = []

    for dirpath, dirnames, filenames in os.walk(root):
        dirnames[:] = [d for d in dirnames if d not in SKIP_DIRS]
        for fn in filenames:
            if Path(fn).suffix in exts:
                fp = Path(dirpath) / fn
                for line_no, frm, to, inc in scan_file(root, fp):
                    all_violations.append((fp, line_no, frm, to, inc))

    if not all_violations:
        print("dependency check: OK")
        return 0

    print("dependency check: FAILED")
    print("  (module -> module it is not allowed to depend on)")
    for fp, line_no, frm, to, inc in all_violations:
        print(f"  {fp}:{line_no}: {frm} -> {to}  ({inc})")
    return 1


if __name__ == "__main__":
    sys.exit(main())
