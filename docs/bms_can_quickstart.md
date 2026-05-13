# BMS CAN Broadcaster — Integrator Quickstart

**DBC file:** [`bms_can_protocol.dbc`](../bms_can_protocol.dbc)

---

## 1. Hardware Setup

| Signal | Pin | Notes |
|---|---|---|
| HV CAN RX | `HV_CAN_RX` | 500 kbps, 120 Ω termination required at each end |
| HV CAN TX | `HV_CAN_TX` | |
| IVT-S shunt | HV CAN bus | Responds to frames 0x521–0x528 and 0x511 |
| BCC chain 0 | SPI (`BCC0_TX_CS`, `BCC0_INTB`, `BCC0_ENABLE`) | MC33772B, up to 15 modules |
| BCC chain 1 | SPI (`BCC1_TX_CS`, `BCC1_INTB`, `BCC1_ENABLE`) | MC33772B, up to 15 modules |
| Debug serial | 115200 baud | Available in `DEBUG` builds |

All CAN broadcast frames (0x351–0x405) are transmitted on **HV CAN at 500 kbps**.

---

## 2. Startup Sequence

On power-on the BMS transmits **0x404 (BMSConfig)** immediately before any other custom frames. Receivers should wait for this frame to determine the active module count before parsing 0x400–0x403.

The 0x404 frame then repeats every **5 seconds**.

---

## 3. Frame Summary

| CAN ID | Name | Rate | Purpose |
|---|---|---|---|
| 0x351 | ChargeVoltageLimit | 1000 ms | Pack charge/discharge voltage and current limits — ecosystem format |
| 0x355 | StateOfCharge | 1000 ms | SOC and SOH |
| 0x356 | PackSummary | 500 ms | Pack voltage, current, temperature |
| 0x359 | FaultFlags | 500 ms | Protection and warning flags, active module count |
| 0x35C | ChargerControl | 1000 ms | Charge enable / discharge enable |
| 0x400 | CellVoltages_1_3 | 100 ms/module | Cells 1–3, MUX = module index |
| 0x401 | CellVoltages_4_6 | 100 ms/module | Cells 4–6, MUX = module index |
| 0x402 | BalancingStatus | 500 ms/module | Per-module balance mask, mode, target |
| 0x403 | Temperature | 500 ms/module | Temp_A, Temp_B, Temp_Die per module |
| 0x404 | BMSConfig | 5000 ms + on change | Chain topology, module count, BMS state |
| 0x405 | ContactorStatus | 100 ms | K1–K4 state, pre-charge state and voltages |
| 0x406 | ExtPackSummary | 500 ms | High-resolution pack voltage (1 mV/LSB) and current (1 mA/LSB) — non-ecosystem |

Frames 0x351, 0x355, and 0x35C are the **minimum required** for charger compatibility with Victron, SMA, and Fronius systems.

### Module indexing

Modules are addressed globally across both chains:

- **Chain 0:** module index 0–14
- **Chain 1:** module index 15–29

The active count for each chain is broadcast in 0x404 (`Chain0_Modules`, `Chain1_Modules`). Only indices below the active count produce frames.

---

## 4. Signal Encodings

Key encoding decisions that differ from common BMS implementations:

| Frame | Signal | Encoding | Why |
|---|---|---|---|
| 0x356 | PackVoltage | int16, **10 mV/LSB** | Ecosystem standard — overflows above 327 V, use 0x406 for high-voltage packs |
| 0x406 | PackVoltage | uint32, **1 mV/LSB** | Extended frame — 1 mV resolution, supports up to ~4294 V |
| 0x406 | PackCurrent | int32, **1 mA/LSB** | Extended frame — 1 mA resolution |
| 0x405 | Precharge_Voltage | uint16, **100 mV/LSB** | uint16 at 1 mV/LSB overflows at 65 V |
| 0x405 | Pack_Voltage | uint16, **100 mV/LSB** | Same reason |
| 0x351 | ChargeVoltage | uint16, **10 mV/LSB** | Ecosystem standard — Victron/SMA/Fronius require this encoding |
| 0x351 | DischargeVoltage | uint16, **10 mV/LSB** | Ecosystem standard. Max encodable: 655.35 V |
| 0x356 | PackTemp | int16, 0.1°C/LSB | **0x7FFF = invalid** — BCC NTC readback not yet wired in firmware |
| 0x403 | Temp_A/B/Die | int16, 0.01°C/LSB | **0x7FFF = invalid** — same reason |

---

## 5. CRC

Frames 0x400–0x405 include a **CRC-8/AUTOSAR** checksum in byte 7, calculated over bytes 0–6.

| Parameter | Value |
|---|---|
| Polynomial | 0x2F |
| Initial value | 0xFF |
| Input/output reflection | None |
| Final XOR | 0xFF |
| Check value | 0xDF (CRC of "123456789") |

Frames 0x351–0x35C do **not** include a CRC (ecosystem format).

---

## 6. Configuration

### 6.1 Compile-time parameters (set before flashing)

Parameters are stored in non-volatile memory and loaded at boot. They can be updated by writing to parameter storage before flashing. There is currently **no runtime configuration interface** — UDS (§10 of the protocol spec) is planned but not yet implemented.

#### BCC Hardware

| Parameter | Unit | Range | Default | Description |
|---|---|---|---|---|
| `bcc0DeviceCount` | — | 0–15 | 8 | Number of MC33772B devices on chain 0 |
| `bcc0DeviceType` | 0=MC33771, 1=MC33772 | 0–1 | 1 | IC type for chain 0 |
| `bcc1DeviceCount` | — | 0–15 | 8 | Number of MC33772B devices on chain 1 |
| `bcc1DeviceType` | 0=MC33771, 1=MC33772 | 0–1 | 1 | IC type for chain 1 |

#### Battery

| Parameter | Unit | Range | Default | Description |
|---|---|---|---|---|
| `targetCellVolt` | mV | 3000–4300 | 3600 | Per-cell charge target voltage |
| `batteryCapacity` | Ah | 10–200 | 50 | Total pack capacity (used for SOC coulomb counting) |
| `minSocPercent` | % | 0–50 | 10 | SOC floor — BMS will not discharge below this |
| `maxSocPercent` | % | 50–100 | 100 | SOC ceiling |
| `initSocPercent` | % | 0–100 | 50 | Initial SOC estimate on first boot before coulomb counting converges |

#### Charging

| Parameter | Unit | Range | Default | Description |
|---|---|---|---|---|
| `maxChargeCurrent` | A | 5–200 | 30 | Maximum charge current limit (broadcast in 0x351 `ChargeCurrentLimit`) |
| `balanceThreshold` | mV | 10–200 | 50 | Cell delta-V above which balancing activates (also broadcast in 0x402 `Balance_Param`) |
| `balanceTarget` | mV | 5–100 | 10 | Cell delta-V below which balancing deactivates and charging resumes |
| `balanceTimerMin` | min | 1–60 | 5 | Maximum duration of a single balancing pass |
| `measureInterval` | ms | 10–1000 | 20 | Voltage measurement interval |

#### HV System

| Parameter | Unit | Range | Default | Description |
|---|---|---|---|---|
| `prechargeMargin` | V | 1–50 | 10 | Acceptable V_pack − V_inverter delta for precharge completion |
| `prechargeTimeout` | ms | 1000–30000 | 5000 | Maximum precharge duration before fault |
| `prechargeCheckInt` | ms | 10–1000 | 100 | How often precharge voltage is checked |

#### Contactor PWM (economizer mode)

| Parameter | Unit | Range | Default | Description |
|---|---|---|---|---|
| `pwmFrequency` | Hz | 1000–50000 | 25000 | Contactor PWM frequency |
| `engageDuty` | % | 50–100 | 100 | Duty cycle during initial engagement |
| `holdDuty` | % | 10–80 | 30 | Hold duty cycle after engagement |
| `engageTime` | ms | 10–500 | 100 | Duration of full-duty engage phase before switching to hold |

#### Timing

| Parameter | Unit | Range | Default | Description |
|---|---|---|---|---|
| `commTimeout` | ms | 1000–30000 | 5000 | BCC communication loss timeout before fault |
| `faultCheckInt` | ms | 100–30000 | 5000 | How often fault registers are read from BCC |

### 6.2 UDS configuration (planned — not yet implemented)

The protocol spec defines a full UDS interface (0x7E0 / 0x7E8) for runtime configuration of balancing parameters, protection thresholds, contactor roles, SOH, and chain topology. This is not implemented in the current firmware. All configuration is currently done at flash time.

See `EXTERNAL_BMS_PLAN.md §11` for the planned DID map.

---

## 7. Read-only diagnostic values

These values are maintained internally by the BMS and readable via the parameter interface (not broadcast over CAN).

| Value | Unit | Description |
|---|---|---|
| `packVoltage` | V | IVT-S V1 pack-side voltage |
| `packCurrent` | A | IVT-S current |
| `soc` | % | State of charge (integer) |
| `socPrecise` | % | State of charge (float) |
| `cellVoltMin` | mV | Minimum cell voltage across all cells |
| `cellVoltMax` | mV | Maximum cell voltage across all cells |
| `cellVoltDiff` | mV | Max − Min cell voltage |
| `safeChargeCurrent` | A | Computed safe charge current |
| `bmsState` | — | Internal BMS state enum |
| `hvState` | — | HV state machine state |
| `bcc0Initialized` | — | BCC chain 0 init status |
| `bcc1Initialized` | — | BCC chain 1 init status |
| `faultStatus` | — | Raw BCC fault register bitmask |
| `lasterr` | — | Last fault code |
| `ivtCurrent` | A | IVT-S current (raw CAN value) |
| `ivtVoltage1` | V | IVT-S V1 (pack side) |
| `ivtVoltage2` | V | IVT-S V2 (inverter side) |
| `ivtTemperature` | C | IVT-S temperature |
| `ivtPower` | kW | IVT-S computed power |

---

## 8. Fault flags (0x359)

| Bit | Signal | Source |
|---|---|---|
| Byte 0, bit 0 | OVP | BCC `CELL_OV` fault register |
| Byte 0, bit 1 | UVP | BCC `CELL_UV` fault register |
| Byte 0, bit 2 | OTP | BCC `AN_OT_UT` register, over-temp bits |
| Byte 0, bit 3 | UTP | BCC `AN_OT_UT` register, under-temp bits |
| Byte 1, bit 0 | OCP_Charge | IVT current > 0 while `has_faults()` |
| Byte 1, bit 1 | OCP_Discharge | IVT current < 0 while `has_faults()` |
| Byte 2, bits 0–2 | WarnHighV, WarnLowV, WarnHighT | Same sources as protection flags |
| Byte 4 | ActiveModules | 0 if BCC comm fault, else total module count |
| Byte 5 | BalancingModules | Total modules when in `BMS_CellBalancing` state, else 0 |

---

## 9. BMS state machine

States broadcast in 0x404 `BMS_State` and 0x405 `Precharge_State`:

```
              ┌─────────┐
   boot       │  Init   │
─────────────►│  0x00   │
              └────┬────┘
                   │ BCC chains enumerated
              ┌────▼────┐
              │  Ready  │◄──── HV_Shutdown
              │  0x01   │
              └────┬────┘
                   │ start_charging() / start_drive_mode()
          ┌────────▼────────┐
          │   Precharge     │
          │     0x06        │
          └────────┬────────┘
                   │ V_inverter ≈ V_pack (within prechargeMargin)
          ┌────────▼────────┐
          │ Charging/Active │
          │   0x04 / 0x02   │
          └────────┬────────┘
                   │ cell delta-V > balanceThreshold
          ┌────────▼────────┐
          │   Balancing     │
          │     0x03        │
          └─────────────────┘
                   
  Any state ──fault──► Fault (0x05)
```

---

## 10. Charger integration (ecosystem frames)

For Victron/SMA/Fronius chargers, the minimum subscription is:

1. **0x351** — pack charge voltage and current limits (sent every 1 s)
2. **0x355** — SOC and SOH (sent every 1 s)
3. **0x35C** — charge enable flag (sent every 1 s)

The charger must see all three frames within its timeout window (typically 3–5 s) to remain enabled. The `ChargeEnable` bit in 0x35C goes high when `BMS_State` is `Charging` or `Idle` and no faults are active. `DischargeEnable` goes high when `HV_State` is `Active` and no faults are active.
