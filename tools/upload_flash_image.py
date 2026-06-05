"""
PlatformIO post-build upload script for [env:bms-can] / [env:hv-ecu].

Replaces the default ST-Link upload with a two-stage OpenOCD flash:
  1. bootloader_with_meta.bin at 0x08000000  (64KB, with embedded app CRC+size)
  2. app.bin                   at 0x08010000  (application binary)

The bootloader ELF must exist at .pio/build/bootloader/firmware.elf.
"""

import os
import sys
import subprocess
import shutil

import SCons.Node.Alias
from SCons.Script import AlwaysBuild

Import("env")  # noqa: F821


def _elf_to_bin(objcopy, elf_path, bin_path):
    stale = (
        not os.path.exists(bin_path)
        or os.path.getmtime(elf_path) > os.path.getmtime(bin_path)
    )
    if stale:
        subprocess.run([objcopy, "-O", "binary", elf_path, bin_path], check=True)


def _upload(source, target, env):  # noqa: ANN001
    build_dir = env.subst("$BUILD_DIR")
    project_dir = env.subst("$PROJECT_DIR")
    objcopy = env.subst("$OBJCOPY")

    bl_build = os.path.join(project_dir, ".pio", "build", "bootloader")
    bl_elf = os.path.join(bl_build, "firmware.elf")
    bl_bin = os.path.join(bl_build, "firmware.bin")
    app_elf = os.path.join(build_dir, "firmware.elf")
    app_bin = os.path.join(build_dir, "firmware.bin")
    app_out = os.path.join(build_dir, "app.bin")
    bl_meta = os.path.join(build_dir, "bootloader_with_meta.bin")

    if not os.path.exists(bl_elf):
        print(
            "upload: bootloader ELF not found"
            " — run 'pio run -e bootloader' first"
        )
        env.Exit(1)

    _elf_to_bin(objcopy, bl_elf, bl_bin)
    _elf_to_bin(objcopy, app_elf, app_bin)

    sys.path.insert(0, os.path.join(project_dir, "tools"))
    from gen_flash_image import build_image  # noqa: PLC0415
    shutil.copy2(app_bin, app_out)
    app_size, app_crc = build_image(bl_bin, app_bin, bl_meta)
    print(f"upload: size=0x{app_size:08X} crc=0x{app_crc:08X}")

    openocd = os.path.expanduser(
        "~/.platformio/packages/tool-openocd/bin/openocd"
    )
    if not os.path.exists(openocd):
        openocd = "openocd"

    rel = lambda p: os.path.relpath(p, project_dir)  # noqa: E731
    print(f"upload: flashing {rel(bl_meta)} -> 0x08000000")
    print(f"upload: flashing {rel(app_out)} -> 0x08010000")

    result = subprocess.run([
        openocd,
        "-f", "interface/stlink.cfg",
        "-f", "target/stm32f4x.cfg",
        "-c", (
            f"program {bl_meta} 0x08000000 verify; "
            f"program {app_out} 0x08010000 verify reset exit"
        ),
    ])
    if result.returncode != 0:
        print(f"upload: OpenOCD failed with code {result.returncode}")
        env.Exit(result.returncode)


# Clear the platform's upload alias (which depends on firmware.bin) and
# replace it with ours, which depends on firmware.elf and runs objcopy itself.
_alias = SCons.Node.Alias.default_ans.lookup("upload")
if _alias is not None:
    _alias.sources = []
    _alias.sources_set = set()
    _alias.builder = None
    _alias.reset_executor()

_elf = os.path.join(env.subst("$BUILD_DIR"), "firmware.elf")  # noqa: F821
AlwaysBuild(env.Alias("upload", _elf, _upload))  # noqa: F821
