#!/usr/bin/env python3
"""Generate compile_commands.json for the firmware.

clangd otherwise has no idea how this tree is built: it cannot find Inc/,
the CubeMX HAL, FreeRTOS or LwIP, so every source reports hundreds of bogus
"file not found" and "unknown type name" errors that bury real ones. bear is
not available here, so the flags are read straight out of the Makefile.

Run from the firmware directory:  python3 tools/gen_compile_commands.py
"""

import json
import os
import re
import shlex
import subprocess
import sys

FIRMWARE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def make_variable(name):
    """Ask make itself for a variable, so this never drifts from the build."""
    result = subprocess.run(
        ["make", "--no-print-directory", "-f", "-", "print"],
        cwd=FIRMWARE_DIR,
        input=f"include Makefile\nprint:\n\t@echo $({name})\n",
        capture_output=True,
        text=True,
        check=False,
    )
    if result.returncode != 0:
        sys.exit(f"could not read {name} from the Makefile:\n{result.stderr}")
    return result.stdout.strip()


def main():
    sources = make_variable("C_SOURCES").split()
    cflags = shlex.split(make_variable("CFLAGS"))

    # clangd drives a host clang, which rejects the ARM-only code generation
    # flags. Keep the includes and defines, drop the rest, and let clangd parse
    # as a generic target.
    keep = []
    skip_next = False
    for flag in cflags:
        if skip_next:
            skip_next = False
            continue
        if flag.startswith(("-I", "-D")):
            keep.append(flag)
        elif flag in ("-MF", "-o"):
            skip_next = True

    keep += ["-std=gnu17", "-DUSE_HAL_DRIVER", "-DSTM32F407xx"]

    entries = []
    for source in sources:
        entries.append(
            {
                "directory": FIRMWARE_DIR,
                "file": os.path.join(FIRMWARE_DIR, source),
                "arguments": ["clang"] + keep + ["-c", source],
            }
        )

    out_path = os.path.join(FIRMWARE_DIR, "compile_commands.json")
    with open(out_path, "w", encoding="utf-8") as handle:
        json.dump(entries, handle, indent=2)
        handle.write("\n")

    print(f"wrote {out_path} ({len(entries)} translation units)")


if __name__ == "__main__":
    main()
