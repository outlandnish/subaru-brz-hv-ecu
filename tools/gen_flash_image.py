"""
Post-build script and standalone CLI tool.

As a PlatformIO post-build script (attached to [env:hv-ecu]):
  Automatically runs after the app build, reads the bootloader binary from
  .pio/build/bootloader/firmware.bin, embeds the app CRC+size into the last
  8 bytes of the padded 64KB bootloader region, and writes two output files:

    .pio/build/hv-ecu/bootloader_with_meta.bin  — 64KB, ready to flash at 0x08000000
    .pio/build/hv-ecu/app.bin                   — app binary, flash at 0x08010000

As a standalone CLI:
    python3 tools/gen_flash_image.py [--flash]

  --flash   also programs both images via OpenOCD + ST-Link
"""

import os
import struct
import subprocess
import sys

# ── CRC-32/ISO-HDLC (matches flash_crc32() in lib/bootloader/flash.cpp) ──────
def crc32_iso_hdlc(data: bytes) -> int:
    crc = 0xFFFFFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = (crc >> 1) ^ 0xEDB88320 if crc & 1 else crc >> 1
    return (~crc) & 0xFFFFFFFF


def build_image(bl_bin: str, app_bin: str, out_bin: str) -> tuple[int, int]:
    """
    Pad bootloader to 64KB, embed app_size + app_crc32 at offset 0xFFF8, write
    out_bin.  Returns (app_size, app_crc).
    """
    BOOTLOADER_SIZE = 64 * 1024
    META_OFFSET     = 0xFFF8       # 0x0800FFF8 - 0x08000000

    with open(bl_bin, "rb") as f:
        bl_data = f.read()
    with open(app_bin, "rb") as f:
        app_data = f.read()

    if len(bl_data) > META_OFFSET:
        raise RuntimeError(
            f"Bootloader binary ({len(bl_data)} B) overflows reserved region "
            f"(max {META_OFFSET} B).  Reduce bootloader size."
        )

    app_size = len(app_data)
    app_crc  = crc32_iso_hdlc(app_data)

    padded = bytearray(b"\xFF" * BOOTLOADER_SIZE)
    padded[:len(bl_data)] = bl_data
    padded[META_OFFSET:META_OFFSET + 4] = struct.pack("<I", app_size)
    padded[META_OFFSET + 4:META_OFFSET + 8] = struct.pack("<I", app_crc)

    with open(out_bin, "wb") as f:
        f.write(padded)

    return app_size, app_crc


def flash_images(bl_with_meta: str, app_bin: str, openocd: str) -> None:
    """Flash bootloader+meta at 0x08000000 then app at 0x08010000."""
    cmds = [
        openocd,
        "-f", "interface/stlink.cfg",
        "-f", "target/stm32f4x.cfg",
        "-c", (
            f"program {bl_with_meta} 0x08000000 verify; "
            f"program {app_bin} 0x08010000 verify reset exit"
        ),
    ]
    result = subprocess.run(cmds, capture_output=False)
    if result.returncode != 0:
        raise RuntimeError(f"OpenOCD exited with code {result.returncode}")


# ── PlatformIO post-build hook ────────────────────────────────────────────────
def _pio_post_build(source, target, env):          # noqa: ANN001
    build_dir  = env.subst("$BUILD_DIR")
    project_dir = env.subst("$PROJECT_DIR")

    bl_bin  = os.path.join(project_dir, ".pio", "build", "bootloader", "firmware.bin")
    app_bin = os.path.join(build_dir, "firmware.bin")
    out_bin = os.path.join(build_dir, "bootloader_with_meta.bin")
    app_out = os.path.join(build_dir, "app.bin")

    if not os.path.exists(bl_bin):
        print(f"gen_flash_image: bootloader binary not found at {bl_bin}")
        print("gen_flash_image: run 'pio run -e bootloader' first")
        return

    # Copy app binary alongside the combined image for convenience
    import shutil
    shutil.copy2(app_bin, app_out)

    app_size, app_crc = build_image(bl_bin, app_bin, out_bin)

    print(f"gen_flash_image: size=0x{app_size:08X} crc=0x{app_crc:08X} -> {os.path.relpath(out_bin, project_dir)}")


# Register with PlatformIO when imported as a build script
try:
    Import("env")                      # noqa: F821
    _bin = os.path.join(env.subst("$BUILD_DIR"), "firmware.bin")
    env.AddPostAction(_bin, _pio_post_build)  # noqa: F821
except Exception:
    pass  # not running inside PlatformIO — CLI mode below


# ── Standalone CLI ────────────────────────────────────────────────────────────
if __name__ == "__main__":
    import argparse, shutil

    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--flash", action="store_true", help="Flash both images via OpenOCD after building")
    parser.add_argument("--bl-bin",  default=".pio/build/bootloader/firmware.bin", help="Bootloader binary")
    parser.add_argument("--app-bin", default=".pio/build/hv-ecu/firmware.bin",     help="App binary")
    parser.add_argument("--out-dir", default=".pio/build/hv-ecu",                  help="Output directory")
    parser.add_argument("--openocd", default=None,                                  help="Path to openocd binary")
    args = parser.parse_args()

    # Resolve repo root relative to this script
    script_dir  = os.path.dirname(os.path.abspath(__file__))
    project_dir = os.path.dirname(script_dir)

    bl_bin  = os.path.join(project_dir, args.bl_bin)
    app_bin = os.path.join(project_dir, args.app_bin)
    out_dir = os.path.join(project_dir, args.out_dir)
    out_bin = os.path.join(out_dir, "bootloader_with_meta.bin")
    app_out = os.path.join(out_dir, "app.bin")

    for path, label in [(bl_bin, "bootloader"), (app_bin, "app")]:
        if not os.path.exists(path):
            print(f"error: {label} binary not found: {path}", file=sys.stderr)
            sys.exit(1)

    os.makedirs(out_dir, exist_ok=True)
    shutil.copy2(app_bin, app_out)

    app_size, app_crc = build_image(bl_bin, app_bin, out_bin)

    print(f"App size : 0x{app_size:08X}  ({app_size} bytes)")
    print(f"App CRC  : 0x{app_crc:08X}")
    print(f"Output   : {os.path.relpath(out_bin, project_dir)}")
    print(f"App copy : {os.path.relpath(app_out, project_dir)}")

    if args.flash:
        # Auto-detect OpenOCD bundled with PlatformIO
        if args.openocd is None:
            pio_openocd = os.path.expanduser(
                "~/.platformio/packages/tool-openocd/bin/openocd"
            )
            args.openocd = pio_openocd if os.path.exists(pio_openocd) else "openocd"

        print(f"\nFlashing via {args.openocd} ...")
        flash_images(out_bin, app_out, args.openocd)
        print("Done.")
