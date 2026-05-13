# Hardening Plan — BRZ HV-ECU

Generated after full code review on 2026-05-13.
Base commit: `b07c38b` (Add IVT-S error decoding and bring simple_charger onto v1)

This plan covers everything surfaced by the review. Each branch has its own
"Start context" section so it can be picked up cold by Claude or a human.

No new features land until branch 1 is merged. Branches 2 and 3 can be worked
in parallel after branch 1 is merged. Branch 4 is housekeeping and can be
done at any time.

---

## Branch 1 — `hardening/safety-critical`

**Merge requirement:** must be reviewed and merged before any other work.

### Start context

- STM32F413 running FreeRTOS. Main app entrypoint: `src/main.cpp`. BMS logic in `src/bms/bms.cpp` / `src/bms/bms.h`.
- HV contactor driver: DRV8874 H-bridge (PMODE floating = independent half-bridge). IN1=PD12, IN2=PD13, nSLEEP=PD10, nFAULT=PC0.
- AC contactor driver: same topology. IN1=PD14, IN2=PD15, nSLEEP=PD11, nFAULT=PC1. (Not yet driven by BMS — stub only.)
- HVIL interlock: HVIL_DETECT_PIN = PC2. Currently defined but never read anywhere.
- IVT-S current/voltage shunt on **HV CAN** (PA11/PA12, `hv_can` instance). CAN IDs 0x521–0x528.
- CHAdeMO controller (Foccci) on **M3 CAN** (PB12/PB13, `m3_can` instance). CAN IDs 0x100/0x102/0x108/0x109.
- FreeRTOS tasks (all in main.cpp + bms.cpp): `can_rx_task` (pri 4), `ivt_process_task` (pri 3), `canopen_periodic_task` (pri 2), BMS `master_task` (pri 2), BMS `bcc0_monitor_task` / `bcc1_monitor_task` (pri 2), BMS `hv_can_task` (pri 2), `console_task` (pri 1).
- The BMS state enum: `BMS_Initialization, BMS_Idle, BMS_Charging, BMS_CellBalancing, BMS_Sleep, BMS_Error`.
- The HV state enum: `HV_Disabled, HV_Precharge, HV_Active, HV_Fault, HV_Shutdown`.
- Cell voltages stored in `uint32_t cell_voltages_uv[BCC_MAX_CELLS]` written by `bcc0_monitor_task` and `bcc1_monitor_task`, read by `master_task` and `update_spot_values`.

### Items

**S1. Move HV contactor pin init to the very top of `setup()`** ✓
- File: `src/main.cpp` (~line 902 is where AC contactors are set up, HV is missing)
- Before any BMS/CAN/task init: `pinMode(HV_CONTACTOR_1_PIN, OUTPUT)`, `pinMode(HV_CONTACTOR_2_PIN, OUTPUT)`, `pinMode(HV_CONTACTOR_NSLEEP_PIN, OUTPUT)`, `pinMode(HV_CONTACTOR_FAULT_PIN, INPUT)`, `digitalWrite` all outputs LOW. nSLEEP=LOW keeps DRV8874 asleep. Must happen before the 2s serial delay.

**S2. Wire HVIL_DETECT_PIN into the HV state machine** ✓
- File: `src/main.cpp` setup (add `pinMode(HVIL_DETECT_PIN, INPUT_PULLUP)`)
- File: `src/bms/bms.cpp` `update_hv_state()` and `master_task_loop()`
- Debounce: require 3 consecutive reads of HVIL open before acting (poll cadence is measurement_interval_ms, so 3×100ms = 300ms debounce).
- On HVIL break: call `hv_disconnect()`, force `hv_state = HV_Fault`, log.
- The pin (PC2) is not shared with anything else in the v1 header so no conflict.

**S3. Fix IVT/HV CAN queue mis-routing** ✓
- `ivt_process_task` was reading from `m3_can_queue` despite both IVT and CHAdeMO being on HV CAN. Fixed: task now reads `hv_can_queue`; CHAdeMO `begin()` call updated to pass `hv_can`; comment updated. `m3_can_queue` is now unused — wire it up or remove it when M3 CAN gets a consumer.

**S4. Fix stack voltage to include BCC1** ✓
- File: `src/bms/bms.cpp`
- `stack_voltage_uv` is only written by `bcc0_monitor_task_loop` (~line 453). Add `stack_voltage_bcc1_uv` written by `bcc1_monitor_task_loop`.
- `get_hv_state()` check at line 324, and anywhere `packVoltage` is set, must sum both: `(stack_voltage_uv + stack_voltage_bcc1_uv) / 1000000`.
- Same fix needed in `has_reached_target_voltage` / `get_max_cell_voltage_diff_mv` / `calculate_cell_balance_requirements` — all currently loop only `bcc0_config->cell_count`. Change to `bcc0_config->device_count * bcc0_config->cell_count + bcc1_config->device_count * bcc1_config->cell_count`.
- `CELL_OFFSET` hardcoded as `6` in `bcc1_monitor_task_loop` (~line 511). Replace with `bcc0_config->device_count * bcc0_config->cell_count`.

**S5. Fix PWM/digitalWrite conflict on HV contactor pins** ✓
- File: `src/bms/bms.cpp` `hv_connect()`, `hv_disconnect()`, `update_hv_state()`
- Lines ~694–696, ~705–706, ~768 call `digitalWrite(positive_contactor_pin, ...)` / `digitalWrite(negative_contactor_pin, ...)` directly. When `contactors_use_pwm = true` the HardwareTimer still owns the pin alt-function.
- Funnel ALL transitions through `control_contactors(bool, bool)` which already branches on `contactors_use_pwm`. Remove the bare `digitalWrite` calls.

**S6. Add task-level locking for shared cell voltage state** ✓
- Files: `src/bms/bms.cpp`, `src/bms/bms.h`
- Declare a `SemaphoreHandle_t cell_voltage_mutex` (or use `portENTER_CRITICAL`/`portEXIT_CRITICAL` for the short regions).
- Writers: `bcc0_monitor_task_loop` and `bcc1_monitor_task_loop` when updating `cell_voltages_uv[]` and `stack_voltage_uv`/`stack_voltage_bcc1_uv`.
- Readers: `update_spot_values()`, `has_reached_target_voltage()`, `get_max_cell_voltage_diff_mv()`. Take mutex to copy the array into a local snapshot, then release before processing.
- `current_state` and `hv_state` (uint8_t) are single-word and are written only by `master_task`; declare `volatile` but don't need a mutex. `contactor_fault` (bool) same treatment.

**S7. Verify `vTaskStartScheduler()` call at end of `setup()`** ✓
- File: `src/main.cpp:1182`
- STM32FreeRTOS port typically wraps `setup()` in a task that's started by the scheduler before `setup()` runs — calling `vTaskStartScheduler` a second time is at best a no-op, at worst UB.
- Check the port's `main.c` / `stm32freertos.cpp` in `lib/`. If the scheduler is already running when `setup()` is called: remove the `vTaskStartScheduler()` call and the "ERROR" block after it entirely. If not: keep it. Add a comment explaining which case applies.

**S8. `enable_sleep_mode()` must open contactors first** ✓
- File: `src/bms/bms.cpp` `enable_sleep_mode()` (~line 1340)
- Prepend `hv_disconnect()` before putting BCC into sleep. The `reboot` path in main already calls `stop_charging()` first, but direct callers of `enable_sleep_mode()` do not.

**S9. Add `BMS_Error` state transition on precharge timeout** ✓
- File: `src/bms/bms.cpp` (~line 753)
- After `hv_disconnect()` + `hv_state = HV_Fault`, also set `current_state = BMS_Error`. Currently `current_state` is left in whatever it was (typically `BMS_Charging`).

**S10. Gate `start_charging()` / `hv_connect()` on `hv_state != HV_Fault`** ✓
- File: `src/bms/bms.cpp` `start_charging()` and `hv_connect()`
- Currently only checks `current_state != BMS_Error`. After a fault the operator must explicitly clear it; add `if (hv_state == HV_Fault) return;` guard.

**S11. Guard `total_cells == 0` as a hard fault** ✓
- File: `src/main.cpp` (~line 891)
- Currently prints "ERROR: No BCC devices configured!" but continues. BMS tasks still start and the master task loops forever on `hardware_initialized`. Change to: blink all LEDs red + halt (infinite `vTaskDelay` loop, no tasks created, contactors confirmed open).

**S12. IVT offline must stop charging** ✓
- File: `src/bms/bms.cpp` `master_task_loop()` or `update_spot_values()`
- After IVT `is_alive()` returns false (>2s without a frame), if `hv_state == HV_Active` or `current_state == BMS_Charging`: force `safeChargeCurrent = 0`, call `stop_charging()`, `current_state = BMS_Error`.
- Currently `packCurrent` is silently set to 0 but charging continues.

---

## Branch 2 — `hardening/io-and-logging`

**Prerequisite:** branch 1 merged.

### Start context

- `DebugSerial` is declared in `src/debug_serial.cpp` AND again in `src/main.cpp:14`. Both compile into `hv-ecu` env — this is duplicate definition UB.
- `src/debug_serial.h:4` has `#define Serial DebugSerial` — a textual macro pulled into every TU via `main.h`. Works today because include order happens to align, but will silently break if any lib uses a `Serial` symbol name.
- `src/main.cpp:86` unconditionally `Serial.printf`s every HV CAN frame inside `can_rx_task` — at 500 kbps this floods USART1 and causes frame loss.
- No mutex protects `HardwareSerial` across the five tasks that call `Serial.printf`.
- `src/main.cpp:36–38`: `m3_can_test_enabled`, `m3_can_test_interval_ms`, `m3_can_test_count` declared but no code uses them.
- `src/main.cpp:816`: `delay(2000)` boot stall for serial monitor.
- `src/bms/bms.h` has two new fields `led_display_mode` and `led_mode_switch_time` adjacent to the existing private block but `update_hv_led()` is NOT called in SOC display mode, meaning during 3/6 s of every cycle there's no HV status visible.

### Items

**L1. Fix duplicate `DebugSerial` definition**
- Remove `HardwareSerial DebugSerial(USART1_RX, USART1_TX);` from `src/main.cpp:14`. The definition in `src/debug_serial.cpp` is canonical; `src/debug_serial.h` has the `extern` declaration. `main.h` already includes `debug_serial.h` so `main.cpp` still sees the symbol.

**L2. Remove `#define Serial DebugSerial`; replace callsites**
- Remove the macro from `src/debug_serial.h:4`.
- Run a targeted replace of `Serial.` → `DebugSerial.` in every file under `src/` that uses it (`main.cpp`, `bms/bms.cpp`, `battery_char.cpp`, `chademo/chademo.cpp`, `ivt-s/ivt_shunt.cpp`, `hal/dma_config.cpp`, `simple_charger.cpp`).
- Leave `lib/` alone for now (lib/libopeninv-arduino has its own Serial usage; that's a separate change).

**L3. Add a FreeRTOS mutex around `DebugSerial`**
- File: `src/debug_serial.h` / `src/debug_serial.cpp`
- Create a `SemaphoreHandle_t debug_serial_mutex` initialised in `debug_serial_init()` (a new function called from `setup()` before task creation).
- Provide `debug_printf(const char *fmt, ...)` and `debug_println(const char *str)` wrappers that take the mutex with a 10ms timeout, call `DebugSerial.printf`/`println`, then release.
- Replace `DebugSerial.printf` callsites with `debug_printf`.

**L4. Gate HV CAN frame printing on `hv_can_monitor_enabled`**
- File: `src/main.cpp` `can_rx_task` (~line 86)
- The help text already documents `hv can monitor on/off` commands. Wire the existing `hv_can_monitor_enabled` bool to actually guard the `Serial.printf` block. This already exists — it just isn't being checked.

**L5. Remove dead CAN test state**
- File: `src/main.cpp:36–38`
- `m3_can_test_enabled`, `m3_can_test_interval_ms`, `m3_can_test_count` are declared but never written or read by any task. Either implement the test generator (a simple periodic task sending IVT-shaped frames on M3) or remove the variables and the console commands that reference them (`m3 can test` entries in help at ~line 233).

**L6. Gate boot serial delay on a compile flag**
- File: `src/main.cpp:816`
- Replace `delay(2000)` with `#ifdef DEBUG_WAIT_FOR_SERIAL\n delay(2000);\n #endif`. Add `-D DEBUG_WAIT_FOR_SERIAL` to `[env:hv-ecu]` debug build but not to a future `[env:hv-ecu-release]`.

**L7. Call `update_hv_led()` in SOC display mode**
- File: `src/bms/bms.cpp` `update_status_leds()` (~line 1498)
- After `led_pattern_soc()`, before the early `return`, call `update_hv_led()` so the last pixel always shows HV state regardless of display mode.

**L8. Fix `state_str[]` out-of-bounds in console**
- File: `src/main.cpp:286`
- Array has 7 entries but includes `"Cooldown"` which doesn't exist in `BMS_State` (6 members: Initialization, Idle, Charging, CellBalancing, Sleep, Error). Either remove `"Cooldown"` or add a `BMS_Cooldown` state. Also add a bounds check before indexing.

**L9. Fix `led_pattern_soc()` divide-by-zero guard**
- File: `src/bms/bms.cpp` (~line 1422)
- `uint16_t per_led = 100 / count;` — add `if (count == 0) return;` before this line.

**L10. CAN init failure must not silently continue**
- File: `src/main.cpp` (~line 917–932)
- If `m3_can->begin()` or `hv_can->begin()` returns false: log, blink error LEDs, and halt (or set a `can_failed` flag that is checked in `master_task_loop` and forces `BMS_Error` immediately). Continuing with a null CAN bus means the IVT shunt will never report alive and charging will not engage — but there's no user-visible failure indication.

---

## Branch 3 — `infra/build-and-ci`

**Prerequisite:** branch 1 merged. Can be worked alongside branch 2.

### Start context

- PlatformIO project. Environments: `hv-ecu` (main production app), `simple-charger` (single-device bench tool), `battery-char` (characterization tool), `bms-test` (unity test runner).
- `lib/bcc` is a git submodule. Parent pointer records `4f05bff`; the working tree (and the pushed HEAD of that submodule's `main` branch) is `8c432e5`. The pointer needs bumping.
- `lib_deps` use caret semver (`^`). For an HV safety device, reproducible builds require exact pinning.
- No CI exists. `test_framework = unity` declared in all envs but `test/` contains only a README.
- `web/` is an untracked Node.js project (Express server + static UI). Has `node_modules/` in tree.
- `hv-ecu-v0-pins.h` is still in `src/hal/` but not referenced by anything.
- `.gitignore` has a typo: `lib/iso14429` should be `lib/iso14229`. `node_modules/` is not listed.
- `compile_commands.json` may be tracked (it should not be). `README.md` is sparse.
- FW version `FW_VERSION_*` in `src/main.h:21–24` is hand-edited.

### Items

**I1. Bump `lib/bcc` submodule pointer**
- `lib/bcc` HEAD in parent is `4f05bff`; actual HEAD on the submodule remote is `8c432e5` ("added debug-serial").
- Confirm `lib/bcc` remote has `8c432e5` pushed. Then: `git add lib/bcc && git commit -m "Bump lib/bcc to 8c432e5"`.

**I2. Pin all `lib_deps` to exact versions**
- File: `platformio.ini`
- Change `@^x.y.z` → `@x.y.z` for every dependency. Audit each lib for any known-breaking changes at the exact version before pinning.
- Deps to pin: `gustice/FreeRtosCppAPI`, `stm32duino/STM32duino FreeRTOS`, `khoih-prog/STM32_PWM`, `collin80/can_common`, Adafruit NeoPixel, ArduinoJson.

**I3. Lift shared `lib_deps` to `[env]`**
- File: `platformio.ini`
- Currently `simple-charger` has its own `lib_deps` that shadow `[env]`'s (which is empty). Add a `lib_deps` block to `[env]` with the common set. Each environment extends via `${env.lib_deps}\n extra-dep@x.y.z`.

**I4. Add build warning flags and map file output**
- File: `platformio.ini` `[env]` `build_flags`
- Add: `-Wall -Wextra -Wno-unused-parameter -Werror=return-type -Werror=format -Wdouble-promotion -fstack-usage -Wl,-Map=${BUILD_DIR}/${PIOENV}.map`
- Fix any new warnings that surface (expect a few from lib code — use `#pragma GCC diagnostic` suppressions scoped tightly).

**I5. Add a GitHub Actions CI workflow**
- File: `.github/workflows/build.yml` (new)
- Jobs: (a) `pio run -e hv-ecu`, (b) `pio run -e simple-charger`, (c) `pio run -e battery-char`, (d) `git submodule status | grep -q '^+'` fails if pointer drifts.
- Trigger: push to `main`, any PR.

**I6. Add at least two unity smoke tests**
- File: `test/test_params/test_params.cpp` (new)
- Test 1: `Param::LoadDefaults()` → key safety-critical params are within safe ranges (target cell voltage ≤ 4.2V, max charge current ≤ configured limit).
- Test 2: CAN frame encode/decode round-trip for IVT 0x521 frame (muxid=0, status=0, value=1000 mA → `get_current() == 1.0f`).
- Wire `[env:bms-test]` to actually build and run these.

**I7. Commit `web/` into source control**
- Add `web/.gitignore` containing `node_modules/`.
- `git add web/server.js web/package.json web/package-lock.json web/public/ web/README.md`.
- Commit as `"Add web operator interface"`.
- Add a brief section to top-level `README.md` explaining how to start it.

**I8. Delete `hv-ecu-v0-pins.h`**
- File: `src/hal/hv-ecu-v0-pins.h`
- Confirm zero references (`grep -r v0-pins src/`). Delete. If v0 boards must be supported in future, add a `HV_ECU_BOARD_REV` compile flag and select the correct header in a wrapper.

**I9. Clean up `hv-ecu-v1-pins.h` — remove `DebugSerial` declaration**
- File: `src/hal/hv-ecu-v1-pins.h:57–58`
- Remove the `#include "HardwareSerial.h"` and `extern HardwareSerial DebugSerial;` lines. Pin headers are pure pin maps; the declaration belongs only in `debug_serial.h`.

**I10. Fix `.gitignore` and untrack stray files**
- Add `node_modules/` to `.gitignore`.
- Fix typo `lib/iso14429` → `lib/iso14229`.
- Run `git rm --cached .DS_Store compile_commands.json` if they are tracked.
- Add `*.map`, `*.su` (stack-usage files), `.cache/`.

**I11. Inject firmware version from git at build time**
- Add a PlatformIO `extra_scripts = pre:tools/gen_version.py` script that:
  - Runs `git describe --tags --long --dirty` to get version string + SHA + dirty flag.
  - Writes `src/version_gen.h` with `#define FW_GIT_SHA "..."`, `#define FW_GIT_DIRTY 0|1`, and optionally parses a semver tag to fill `FW_VERSION_MAJOR/MINOR/PATCH`.
  - Add `src/version_gen.h` to `.gitignore`.
- Update `src/main.h` to include `version_gen.h` instead of hand-edited defines.

**I12. Update README**
- Sections to add: hardware overview (link to `hardware/`), BCC submodule init instructions (`git submodule update --init --recursive`), build & flash commands per env, first-boot procedure, web UI instructions, SAFETY notice.
- Fix the stray triple-backtick on line 9.

**I13. Add a LICENSE file**
- The README has a placeholder `[Your License Here]`. Choose a license and commit a `LICENSE` file.

---

## Branch 4 — `hardening/watchdog-and-observability`

**Prerequisite:** branches 1 and 2 merged.

### Start context

- The IWDG (independent watchdog timer) on STM32F413 runs off the LSI (~32 kHz). It is never initialized; a hung task (e.g. BCC SPI busy-wait at `bms.cpp:552–557` has no timeout) will never trigger a reset.
- `bms.cpp:552–557`: `do { bcc->is_converting(...) } while (!completed)` — no timeout. A SPI glitch here hangs the `bcc0_monitor_task` forever with no recovery.
- FreeRTOS `configCHECK_FOR_STACK_OVERFLOW` is not enabled. Stack overruns are silent.
- `uxTaskGetStackHighWaterMark` is never printed; no way to know if any task is close to overflow.
- No fault logging to non-volatile storage. After a power cycle there is no record of why the system faulted.
- `debug_enabled = true` is the default in `IVTShunt` constructor (`ivt_shunt.cpp:16`). This causes `print_frame` to emit 8× `printf` per CAN cycle in the IVT processing task — significant USART pressure in production.

### Items

**W1. Enable IWDG with a 500ms timeout, kicked from `master_task`**
- File: `src/main.cpp` setup, `src/bms/bms.cpp` `master_task_loop()`
- Initialize IWDG in `setup()` before creating tasks. `master_task` kicks the dog at the top of each loop iteration. If the master task hangs (BCC SPI, etc.) the MCU resets within 500ms and the DRV8874 nSLEEP pin is back to its reset default (LOW = safe) within the same window.
- Task priorities mean: if CAN RX, IVT, or CanOpen tasks hang but master does not, dog is still kicked. If master hangs, everything resets. Acceptable tradeoff.

**W2. Add timeout to `is_converting` busy-wait in BCC measure**
- File: `src/bms/bms.cpp` `measure_cell_voltages()` (~line 552)
- Add `uint32_t timeout_start = millis();` before the loop. If `millis() - timeout_start > 50` (50ms, well above any normal conversion time), break and return `false`.
- Propagate `false` up through `master_task_loop` to increment `communication_lost` counter.

**W3. Enable FreeRTOS stack overflow detection**
- File: `lib/` FreeRTOS port config (likely `lib/FreeRTOS/src/FreeRTOSConfig.h` or a project-local override).
- Set `configCHECK_FOR_STACK_OVERFLOW 2`. Implement `vApplicationStackOverflowHook(TaskHandle_t, char*)`: log the task name, open contactors via direct GPIO write (no BMS layer — must be safe even if heap is corrupt), then halt.

**W4. Print FreeRTOS stack watermarks on `status` command**
- File: `src/main.cpp` console `status` handler (~line 284)
- After the existing spot-value dump, add a section that iterates known task handles and prints `uxTaskGetStackHighWaterMark(handle)` for each. This surfaces stack pressure during development without adding production overhead.

**W5. Default `IVTShunt::debug_enabled` to `false`**
- File: `src/ivt-s/ivt_shunt.cpp` constructor (~line 16)
- Change `debug_enabled(true)` → `debug_enabled(false)`. `print_frame` should only emit when explicitly enabled. Add `set_debug(bool)` accessor if not already present, and wire it to a console command (`ivt debug on/off`).

**W6. Add minimal fault-log to flash**
- File: new `src/fault_log.h` / `src/fault_log.cpp`
- Store a 32-byte struct in a dedicated flash sector (STM32F413 has 16 KB sectors): last reset cause (IWDG / soft / power-on), last `BMS_State`, last `HV_State`, last fault bitmask, timestamp (uptime seconds), fw version. Written on any `BMS_Error` transition and on clean shutdown.
- Read and print on boot before anything else.

---

## Deferred / won't-do-now

The following items from the review are acknowledged but deferred to a later
milestone (likely when the AC charging flow is implemented):

- **Pluggable CAN dispatch** (ID registration API). The current if/else chain is fine for the current set of subsystems. Revisit when a third CAN participant is added.
- **Console extraction to `src/console/`**. main.cpp is large but functional; refactor when it crosses 1500 lines or when a new feature requires reusing the console from a different build env.
- **CHAdeMO voltage-verified precharge**. The current 5-second timer is intentional for now (the hardware isn't producing real EVSE handshakes yet). Move this to the AC charging feature branch.
- **`constexpr` pin definitions**. The `#define` approach works with all Arduino HAL macros; converting to `constexpr` risks breaking macro-based Arduino APIs. Defer.
- **`pin_def` for PC0/PC1 conflict between v0 and v1 headers**. Resolved by deleting v0 header in I8.
- **`STM32_UNIQUE_ID_BASE` STM32F7/H7 portability**. Current target is F413; if MCU changes, revisit.

---

## Working notes

When starting a branch, tell Claude:

> "Start `hardening/<branch-name>`. Base: `b07c38b` (or latest main). See HARDENING_PLAN.md for the branch's Start context and item list."

After each item is done, mark it here by appending `✓` to the item header line. When all items in a branch are done, open a PR against `main` and reference this file.
