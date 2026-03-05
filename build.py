#!/usr/bin/env python3
"""
VEX Robot build script
======================
Merges source modules in src/modules/ into a single src/main.py for upload.

Usage:
    python build.py x_robot          # build one robot
    python build.py x_robot tank_robot   # build multiple robots
    python build.py                  # defaults to current directory

Each robot directory must contain:
    src/modules/     – the split source files
    build_order.txt  – module filenames to merge, one per line (relative to
                       src/modules/).  Lines starting with # are comments.

Output is written to src/main.py in the robot directory.
The original module files are never modified.
"""

import os
import sys

GENERATED_HEADER = """\
# ---------------------------------------------------------------------------- #
#   AUTO-GENERATED FILE – do not edit directly.                                #
#   Edit files in src/modules/ and run:                                        #
#       python build.py <robot_dir>                                            #
# ---------------------------------------------------------------------------- #
"""

# Lines that must appear at most once at the very top of the merged file.
DEDUPE_PATTERNS = {
    "from vex import *",
    "import math",
}

# Lines tagged with this comment are kept for editor LSP support but removed
# from the merged output.  Usage:  from sensors import gps  # lsp-only
LSP_IMPORT_MARKER = "# lsp-only"

# The VEX IDE banner delimiter string (leading part is enough)
_VEX_BANNER = "# ------------"


def _strip_vex_header(lines: list) -> list:
    """Remove the VEX IDE auto-generated file header (the 9-line banner comment)."""
    if not lines:
        return lines
    # Banner starts AND ends with a dashes line; skip everything up to
    # (and including) the closing dashes line.
    if lines[0].startswith(_VEX_BANNER):
        closing = 1
        while closing < len(lines):
            if lines[closing].startswith(_VEX_BANNER):
                return lines[closing + 1:]
            closing += 1
    return lines


def merge(robot_dir: str) -> None:
    modules_dir = os.path.join(robot_dir, "src")
    order_file  = os.path.join(robot_dir, "build_order.txt")
    output_file = os.path.join(robot_dir, "src", "main.py")

    if not os.path.isfile(order_file):
        print(f"ERROR: {order_file} not found.")
        return
    if not os.path.isdir(modules_dir):
        print(f"ERROR: {modules_dir} not found.")
        return

    with open(order_file) as f:
        order = [line.strip() for line in f
                 if line.strip() and not line.startswith("#")]

    seen_dedupes    = set()
    deduped_imports = []   # collected in first-encounter order
    sections        = []   # list of (section_name, body_lines)

    for filename in order:
        path = os.path.join(modules_dir, filename)
        if not os.path.isfile(path):
            print(f"WARNING: {path} not found, skipping.")
            continue

        with open(path, encoding="utf-8") as f:
            raw_lines = f.read().splitlines()

        lines     = _strip_vex_header(raw_lines)
        body_lines = []

        for line in lines:
            stripped = line.strip()
            # Drop LSP-only imports entirely from the merged output
            if stripped.endswith(LSP_IMPORT_MARKER):
                continue
            if stripped in DEDUPE_PATTERNS:
                if stripped not in seen_dedupes:
                    seen_dedupes.add(stripped)
                    deduped_imports.append(line)
                # Always skip from section body – it lives at the top
                continue
            body_lines.append(line)

        # Trim leading/trailing blank lines
        while body_lines and not body_lines[0].strip():
            body_lines.pop(0)
        while body_lines and not body_lines[-1].strip():
            body_lines.pop()

        if body_lines:
            section_name = os.path.splitext(filename)[0]
            sections.append((section_name, body_lines))

    # ---- Write output ----
    with open(output_file, "w", encoding="utf-8") as out:
        out.write(GENERATED_HEADER)
        out.write("\n")

        if deduped_imports:
            out.write("\n".join(deduped_imports) + "\n")

        for section_name, body_lines in sections:
            out.write(f"\n\n# ===== {section_name} =====\n\n")
            out.write("\n".join(body_lines))
            out.write("\n")

    print(f"[{robot_dir}] Built {output_file}  ({len(sections)} modules)")


if __name__ == "__main__":
    targets = sys.argv[1:] if len(sys.argv) > 1 else ["."]
    for target in targets:
        merge(target)
