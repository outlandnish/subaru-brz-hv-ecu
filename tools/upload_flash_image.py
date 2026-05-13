"""
PlatformIO upload script for [env:hv-ecu].

Replaces the default ST-Link upload with a two-stage OpenOCD flash:
  1. bootloader_with_meta.bin at 0x08000000  (64KB, with embedded app CRC+size)
  2. app.bin                   at 0x08010000  (application binary)

Requires gen_flash_image.py to have run first (happens automatically via the
post-build hook).  If bootloader_with_meta.bin is missing, falls back to
running gen_flash_image.py build_image() inline.
"""

import os
import sys
import subprocess
import shutil

Import("env")  # noqa: F821


def _upload(source, target, env):  # noqa: ANN001
    build_dir   = env.subst("$BUILD_DIR")
    project_dir = env.subst("$PROJECT_DIR")

    bl_meta = os.path.join(build_dir, "bootloader_with_meta.bin")
    app_bin = os.path.join(build_dir, "app.bin")

    # If gen_flash_image hasn't run yet, do it now
    if not os.path.exists(bl_meta) or not os.path.exists(app_bin):
        sys.path.insert(0, os.path.join(project_dir, "tools"))
        from gen_flash_image import build_image
        bl_src = os.path.join(project_dir, ".pio", "build", "bootloader", "firmware.bin")
        fw_bin = os.path.join(build_dir, "firmware.bin")
        if not os.path.exists(bl_src):
            print("upload: bootloader binary not found — run 'pio run -e bootloader' first")
            env.Exit(1)
        shutil.copy2(fw_bin, app_bin)
        app_size, app_crc = build_image(bl_src, fw_bin, bl_meta)
        print(f"upload: size=0x{app_size:08X} crc=0x{app_crc:08X}")

    openocd = os.path.expanduser("~/.platformio/packages/tool-openocd/bin/openocd")
    if not os.path.exists(openocd):
        openocd = "openocd"

    print(f"upload: flashing {os.path.relpath(bl_meta, project_dir)} -> 0x08000000")
    print(f"upload: flashing {os.path.relpath(app_bin, project_dir)} -> 0x08010000")

    result = subprocess.run([
        openocd,
        "-f", "interface/stlink.cfg",
        "-f", "target/stm32f4x.cfg",
        "-c", (
            f"program {bl_meta} 0x08000000 verify; "
            f"program {app_bin} 0x08010000 verify reset exit"
        ),
    ])
    if result.returncode != 0:
        print(f"upload: OpenOCD failed with code {result.returncode}")
        env.Exit(result.returncode)


env.Replace(UPLOADCMD=_upload)  # noqa: F821
