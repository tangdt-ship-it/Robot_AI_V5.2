"""Verify the STM32 application does not use the reserved MAP/calibration tail."""

import re
import shlex
import subprocess
import sys
import os
from pathlib import Path

Import("env")

RESERVED_START = 0x0807D000
RESERVED_END = 0x08080000


def _check_map_flash_layout(source, target, env):
    elf = Path(str(target[0]))
    objdump_text = env.subst("$OBJDUMP").strip()
    objdump = objdump_text.strip('"')
    if not objdump or not Path(objdump).exists():
        candidates = [
            Path(sys.executable).parent.parent.parent
            / "packages/toolchain-gccarmnoneeabi/bin/arm-none-eabi-objdump.exe",
        ]
        core_dir = os.environ.get("PLATFORMIO_CORE_DIR", "")
        if core_dir:
            candidates.append(
                Path(core_dir)
                / "packages/toolchain-gccarmnoneeabi/bin/arm-none-eabi-objdump.exe"
            )
        for candidate in candidates:
            if candidate.exists():
                objdump = str(candidate)
                break
    if not objdump or not Path(objdump).exists():
        # PlatformIO may expose only the tool basename to the Python
        # subprocess while SCons itself has already resolved the full path.
        cc_text = env.subst("$CC").strip()
        try:
            cc_name = shlex.split(cc_text, posix=False)[0].strip('"')
            candidate = Path(cc_name).with_name("arm-none-eabi-objdump.exe")
            if candidate.exists():
                objdump = str(candidate)
        except (IndexError, ValueError):
            pass
    if not objdump or not Path(objdump).exists():
        raise RuntimeError("MAP flash layout check: arm-none-eabi-objdump not found")
    output = subprocess.check_output(
        [objdump, "-h", str(elf)], text=True, errors="replace"
    )
    sections = []
    section_re = re.compile(
        r"^\s*\d+\s+(\S+)\s+([0-9a-fA-F]+)\s+"
        r"([0-9a-fA-F]+)\s+([0-9a-fA-F]+)\s+"
    )
    for line in output.splitlines():
        match = section_re.match(line)
        if not match:
            continue
        name, size_text, vma_text, lma_text = match.groups()
        size = int(size_text, 16)
        vma = int(vma_text, 16)
        lma = int(lma_text, 16)
        if size:
            sections.append((name, size, vma, lma))

    if not sections:
        raise RuntimeError("MAP flash layout check: no ELF sections found")

    conflicts = []
    flash_ends = []
    for name, size, vma, lma in sections:
        for label, address in (("VMA", vma), ("LMA", lma)):
            if address < RESERVED_END and address + size > RESERVED_START:
                conflicts.append(
                    f"{name} {label}=0x{address:08X} size=0x{size:X}"
                )
        if RESERVED_START > lma >= 0x08000000:
            flash_ends.append(lma + size)

    if conflicts:
        raise RuntimeError(
            "MAP flash layout check: application overlaps reserved tail: "
            + ", ".join(conflicts)
        )

    app_end = max(flash_ends) if flash_ends else 0x08000000
    print(
        "MAP_FLASH_LAYOUT,APP_END=0x%08X,RESERVED=0x%08X-0x%08X,"
        "STATUS=PASS" % (app_end, RESERVED_START, RESERVED_END)
    )


env.AddPostAction("$BUILD_DIR/${PROGNAME}.elf", _check_map_flash_layout)
