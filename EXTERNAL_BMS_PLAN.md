# BMS CAN Protocol v0.1
**Status:** Draft  
**Target Hardware:** Dual-chain MC33772B BMS with Taycan 6S2P modules  
**CAN Bus Speed:** 500 kbps  
**CRC Algorithm:** CRC-8/AUTOSAR (polynomial 0x2F, init 0xFF, reflect in/out none, final XOR 0xFF)

---

## Table of Contents

1. [Overview](#1-overview)
2. [Physical Layer](#2-physical-layer)
3. [Frame Map](#3-frame-map)
4. [Ecosystem Broadcast Frames (0x351–0x35C)](#4-ecosystem-broadcast-frames)
5. [Cell Voltage Frames (0x400–0x401)](#5-cell-voltage-frames-0x400-0x401)
6. [Balancing Status Frame (0x402)](#6-balancing-status-frame-0x402)
7. [Temperature Frame (0x403)](#7-temperature-frame-0x403)
8. [BMS Config Frame (0x404)](#8-bms-config-frame-0x404)
9. [Contactor Status Frame (0x405)](#9-contactor-status-frame-0x405)
10. [UDS Diagnostic Interface](#10-uds-diagnostic-interface)
    - [Sessions and Security](#101-sessions-and-security)
    - [Data Identifiers](#102-data-identifiers)
    - [Fault Codes (DTCs)](#103-fault-codes-dtcs)
    - [Routine Control](#104-routine-control)
    - [Firmware Update](#105-firmware-update)
11. [Timing](#11-timing)
12. [CRC Calculation](#12-crc-calculation)

---

## 1. Overview

This document defines the CAN communication protocol for a dual-chain BMS supporting Porsche Taycan battery modules. Each module contains one MC33772B IC managing 6 cells in a 6S2P configuration. The protocol operates at 500 kbps and is structured in three layers:

| Layer | Frames | Purpose |
|---|---|---|
| Ecosystem Broadcast | 0x351–0x35C | Pack-level data. Compatible with SimpBMS, Victron, SMA, Fronius, and most aftermarket EV conversion chargers and inverters |
| Custom Broadcast | 0x400–0x405 | Per-module cell voltages, balancing status, temperatures, BMS config, contactor state |
| UDS Diagnostic | 0x7E0 / 0x7E8 | Configuration, fault codes, balancing parameters, contactor config, firmware update |

### Hardware Assumptions

- **Chains:** 2 (Chain 0 and Chain 1)
- **Max modules per chain:** 15 (30 total)
- **Cells per module:** 6 (always — Taycan 6S2P)
- **Max total cells:** 180
- **Temperature sensors per module:** 2x external NTC (Temp_A, Temp_B) + 1x MC33772B internal die (Temp_Die)
- **Contactors:** Up to 4 (K1–K4)
- **Current/voltage sensing:** IVT-S

---

## 2. Physical Layer

| Parameter | Value |
|---|---|
| Bus speed | 500 kbps |
| Frame format | CAN 2.0B, 11-bit standard IDs |
| Termination | 120 Ω at each end of bus |
| BMS UDS request ID | 0x7E0 |
| BMS UDS response ID | 0x7E8 |
| CRC algorithm | CRC-8/AUTOSAR |

---

## 3. Frame Map

| CAN ID | Name | Type | Rate | Description |
|---|---|---|---|---|
| 0x351 | ChargeVoltageLimit | Broadcast | 1000 ms | Max charge voltage and current limits |
| 0x355 | StateOfCharge | Broadcast | 1000 ms | SOC and SOH |
| 0x356 | PackSummary | Broadcast | 500 ms | Pack voltage, current, temperature |
| 0x359 | FaultFlags | Broadcast | 500 ms | Aggregate protection fault flags |
| 0x35C | ChargerControl | Broadcast | 1000 ms | Charger enable/disable flags |
| 0x400 | CellVoltages_1_3 | Broadcast (MUX) | 100 ms/module | Cells 1–3 per module. MUX = module index (0–29) |
| 0x401 | CellVoltages_4_6 | Broadcast (MUX) | 100 ms/module | Cells 4–6 per module. MUX = module index (0–29) |
| 0x402 | BalancingStatus | Broadcast (MUX) | 500 ms/module | Per-module balancing bitmask, mode, and target. MUX = module index (0–29) |
| 0x403 | Temperature | Broadcast (MUX) | 500 ms/module | Per-module Temp_A, Temp_B, Temp_Die. MUX = module index (0–29) |
| 0x404 | BMSConfig | Broadcast | On change + 5 s | Chain topology, module counts, BMS state, protocol version |
| 0x405 | ContactorStatus | Broadcast | 100 ms | State of K1–K4 and pre-charge status |
| 0x406 | ExtPackSummary | Broadcast | 500 ms | High-resolution pack voltage (1 mV/LSB) and current (1 mA/LSB) for high-voltage packs |
| 0x7E0 | UDS Request | Request/Response | On demand | ISO 14229 diagnostic requests from external tool |
| 0x7E8 | UDS Response | Request/Response | On demand | ISO 14229 diagnostic responses from BMS |

---

## 4. Ecosystem Broadcast Frames

These frames follow the SimpBMS/BYD/Pylontech broadcast format. Compatible with:

- **Chargers:** Victron MultiPlus/Quattro, SMA Sunny Island, Fronius Symo/Gen24, KEBA P30
- **Inverters:** Victron VE.Can ecosystem, OpenInverter-based systems
- **Monitors:** SimpBMS, Batrium WatchMon (BYD/Pylontech mode)

The minimum required frames for charger compatibility are **0x351, 0x355, and 0x35C**.

### 0x351 — Charge Voltage & Current Limits

Transmitted every 1000 ms.

| Byte | Signal | Bits | Encoding | Description |
|---|---|---|---|---|
| 0–1 | ChargeVoltage | 16 | 10 mV/LSB, uint16 | Maximum charge voltage. Ecosystem standard (Victron/SMA/Fronius). Max encodable: 655.35 V — clamp to UINT16_MAX for packs above 655 V |
| 2–3 | ChargeCurrentLimit | 16 | 100 mA/LSB, uint16 | Maximum charge current |
| 4–5 | DischargeCurrentLimit | 16 | 100 mA/LSB, uint16 | Maximum discharge current |
| 6–7 | DischargeVoltage | 16 | 10 mV/LSB, uint16 | Minimum discharge voltage. Ecosystem standard. Same 655 V ceiling applies |

### 0x355 — State of Charge / Health

Transmitted every 1000 ms.

| Byte | Signal | Bits | Encoding | Description |
|---|---|---|---|---|
| 0–1 | SOC | 16 | 0.01%/LSB, uint16 | State of charge (0–10000 = 0.00–100.00%) |
| 2–3 | SOH | 16 | 0.01%/LSB, uint16 | State of health (0–10000 = 0.00–100.00%) |
| 4–5 | SOC_High | 16 | 0.01%/LSB, uint16 | High precision SOC (same scale, same value) |
| 6–7 | reserved | 16 | — | Set to 0x0000 |

> **Note:** SOH is user-configurable via UDS DID 0xD105. DCIR-based automatic SOH estimation is reserved for a future revision.

### 0x356 — Pack Summary

Transmitted every 500 ms.

| Byte | Signal | Bits | Encoding | Description |
|---|---|---|---|---|
| 0–1 | PackVoltage | 16 | 10 mV/LSB, int16 | Total pack voltage. Ecosystem standard. Overflows above 327 V — use 0x406 for high-voltage packs |
| 2–3 | PackCurrent | 16 | 100 mA/LSB, int16 | Pack current. Positive = charging, negative = discharging |
| 4–5 | PackTemp | 16 | 0.1°C/LSB, int16 | Highest temperature across all NTC sensors. 0x7FFF = invalid/not yet available |
| 6–7 | reserved | 16 | — | Set to 0x0000 |

### 0x359 — Fault Flags

Transmitted every 500 ms.

| Byte | Signal | Bits | Encoding | Description |
|---|---|---|---|---|
| 0 | ProtectionFlags_0 | 8 | Bitmask | Bit 0: OVP, Bit 1: UVP, Bit 2: OTP, Bit 3: UTP |
| 1 | ProtectionFlags_1 | 8 | Bitmask | Bit 0: OCP charge, Bit 1: OCP discharge |
| 2 | WarnFlags_0 | 8 | Bitmask | Bit 0: High V warn, Bit 1: Low V warn, Bit 2: High T warn |
| 3 | WarnFlags_1 | 8 | Bitmask | Bit 0: High charge current warn, Bit 1: High discharge current warn |
| 4 | ActiveModules | 8 | Count | Number of active modules |
| 5 | BalancingModules | 8 | Count | Number of modules currently balancing |
| 6–7 | reserved | 16 | — | Set to 0x0000 |

### 0x35C — Charger Control

Transmitted every 1000 ms.

| Byte | Signal | Bits | Encoding | Description |
|---|---|---|---|---|
| 0 | ChargerFlags | 8 | Bitmask | Bit 0: Charge enable, Bit 1: Force charge, Bit 2: Discharge enable |
| 1–7 | reserved | 56 | — | Set to 0x00 |

---

## 5. Cell Voltage Frames (0x400–0x401)

Cell voltages for each module are split across two independent CAN IDs, each carrying 3 of the 6 cells. Both frames use a standard MUX byte equal to the global module index (0–29) and are transmitted independently at 100 ms per module.

**Global module index convention:**
- Chain 0 modules: index 0–14
- Chain 1 modules: index 15–29

### 0x400 — Cell Voltages, Cells 1–3

Transmitted every 100 ms per module.

| Byte | Signal | Bits | Encoding | Description |
|---|---|---|---|---|
| 0 | MUX | 8 | Module index (0–29) | Global module index |
| 1–2 | Cell1_Voltage | 16 | 1 mV/LSB, uint16 | Cell 1 voltage |
| 3–4 | Cell2_Voltage | 16 | 1 mV/LSB, uint16 | Cell 2 voltage |
| 5–6 | Cell3_Voltage | 16 | 1 mV/LSB, uint16 | Cell 3 voltage |
| 7 | CRC8 | 8 | CRC-8/AUTOSAR | CRC over bytes 0–6 |

### 0x401 — Cell Voltages, Cells 4–6

Transmitted every 100 ms per module.

| Byte | Signal | Bits | Encoding | Description |
|---|---|---|---|---|
| 0 | MUX | 8 | Module index (0–29) | Global module index |
| 1–2 | Cell4_Voltage | 16 | 1 mV/LSB, uint16 | Cell 4 voltage |
| 3–4 | Cell5_Voltage | 16 | 1 mV/LSB, uint16 | Cell 5 voltage |
| 5–6 | Cell6_Voltage | 16 | 1 mV/LSB, uint16 | Cell 6 voltage |
| 7 | CRC8 | 8 | CRC-8/AUTOSAR | CRC over bytes 0–6 |

> **Note:** 0x400 and 0x401 are transmitted independently at the same rate. A receiver can expect both frames for each module index within the same 100 ms cycle. Frames for inactive modules (index >= Total_Modules from 0x404) are not transmitted.

---

## 6. Balancing Status Frame (0x402)

Broadcasts per-module balancing state. One frame per module per cycle, MUX = global module index.

### Frame Layout

Transmitted every 500 ms per module.

| Byte | Signal | Bits | Encoding | Description |
|---|---|---|---|---|
| 0 | MUX | 8 | Module index (0–29) | Global module index |
| 1 | Balance_Mask | 8 | Bitmask | Bits 0–5: cell N+1 currently balancing. Bits 6–7 reserved, set to 0 |
| 2 | Balance_Mode | 8 | Enum | 0x00 = Disabled, 0x01 = Delta-V, 0x02 = Absolute target |
| 3–4 | Balance_Param | 16 | 1 mV/LSB, uint16 | Mode 0x01: delta-V threshold in mV. Mode 0x02: absolute target voltage in mV |
| 5–6 | reserved | 16 | — | Set to 0x0000 |
| 7 | CRC8 | 8 | CRC-8/AUTOSAR | CRC over bytes 0–6 |

### Balance_Mode Encoding

| Value | Mode | Balance_Param Meaning |
|---|---|---|
| 0x00 | Disabled | Ignored — set to 0x0000 |
| 0x01 | Delta-V | Threshold in mV above minimum cell voltage. Balance any cell where V_cell > V_min + threshold |
| 0x02 | Absolute | Target voltage in mV. Balance any cell where V_cell > target |

> **Note:** Balance_Mode and Balance_Param are global settings repeated in every module frame so any single frame is self-contained. Balancing is controlled autonomously by the BMS. Use UDS Routine Control service 0xD001 to trigger a manual balance cycle.

---

## 7. Temperature Frame (0x403)

Broadcasts per-module temperature readings. One frame per module per cycle, MUX = global module index.

### Frame Layout

Transmitted every 500 ms per module.

| Byte | Signal | Bits | Encoding | Description |
|---|---|---|---|---|
| 0 | MUX | 8 | Module index (0–29) | Global module index |
| 1–2 | Temp_A | 16 | 0.01°C/LSB, int16 | NTC thermistor A. Range: −327.68°C to +327.67°C |
| 3–4 | Temp_B | 16 | 0.01°C/LSB, int16 | NTC thermistor B. Range: −327.68°C to +327.67°C |
| 5–6 | Temp_Die | 16 | 0.01°C/LSB, int16 | MC33772B internal die temperature |
| 7 | CRC8 | 8 | CRC-8/AUTOSAR | CRC over bytes 0–6 |

> **Note:** A value of 0x7FFF (32767) indicates an invalid or disconnected sensor. Die temperature has approximately 1°C internal resolution; sub-degree digits will be zero unless interpolation is applied.

---

## 8. BMS Config Frame (0x404)

Broadcasts BMS topology and state as a single fixed frame with no MUX. Transmitted on power-on, on any configuration change, and periodically every 5 seconds. Receivers should wait for this frame before parsing 0x400–0x403 to determine the active module count.

Configuration is **read-only via broadcast**. Write configuration via UDS only.

### Frame Layout

| Byte | Signal | Bits | Encoding | Description |
|---|---|---|---|---|
| 0 | Chain0_Modules | 8 | 0–15 | Number of active modules on chain 0 |
| 1 | Chain1_Modules | 8 | 0–15 | Number of active modules on chain 1 |
| 2 | Total_Modules | 8 | 0–30 | Total active modules across both chains |
| 3 | Protocol_Version | 8 | 0x01 | Protocol version. This document = 0x01 |
| 4 | BMS_State | 8 | Enum | See BMS_State encoding below |
| 5–6 | reserved | 16 | — | Set to 0x0000 |
| 7 | CRC8 | 8 | CRC-8/AUTOSAR | CRC over bytes 0–6 |

### BMS_State Encoding

| Value | State | Description |
|---|---|---|
| 0x00 | Init | BMS initialising, chain communication not yet established |
| 0x01 | Ready | BMS operational, HV system open |
| 0x02 | Active | HV system closed, normal operation |
| 0x03 | Balancing | Balancing in progress |
| 0x04 | Charging | Charge contactor closed, charging in progress |
| 0x05 | Fault | One or more active faults. See 0x359 and UDS DTCs for details |
| 0x06 | Precharge | Pre-charge sequence in progress |

---

## 9. Contactor Status Frame (0x405)

Broadcasts the state of all four contactors and the pre-charge sequence. Transmitted every 100 ms.

### Frame Layout

| Byte | Signal | Bits | Encoding | Description |
|---|---|---|---|---|
| 0 | Contactor_State | 8 | Bitmask | Bit 0: K1, Bit 1: K2, Bit 2: K3, Bit 3: K4. 1 = closed, 0 = open |
| 1 | Precharge_State | 8 | Enum | See Precharge_State encoding below |
| 2–3 | Precharge_Voltage | 16 | **100 mV/LSB, uint16** | IVT-S inverter-side voltage during pre-charge. Changed from 1 mV/LSB to avoid overflow above 65 V |
| 4–5 | Pack_Voltage | 16 | **100 mV/LSB, uint16** | IVT-S pack-side voltage. Changed from 1 mV/LSB to avoid overflow above 65 V |
| 6 | reserved | 8 | — | Set to 0x00 |
| 7 | CRC8 | 8 | CRC-8/AUTOSAR | CRC over bytes 0–6 |

### Precharge_State Encoding

| Value | State | Description |
|---|---|---|
| 0x00 | Idle | Pre-charge not active |
| 0x01 | In Progress | Pre-charge contactor closed, voltage rising |
| 0x02 | Complete | Delta-V threshold met, ready to close main positive |
| 0x03 | Timeout | Pre-charge did not complete within configured timeout |
| 0x04 | Fault | Pre-charge fault (welded contactor, voltage sensor fault, etc.) |

### Default Contactor Assignment

| Contactor | Default Role | Configurable via UDS |
|---|---|---|
| K1 | Main negative | Yes |
| K2 | Main positive | Yes |
| K3 | Pre-charge | Yes |
| K4 | Charge path | Yes |

> **Note:** Contactor roles and pre-charge completion threshold are configurable via UDS DIDs 0xD200–0xD206. Pre-charge completion is determined by IVT-S voltage delta (V_pack − V_inverter < threshold), not by a fixed timer.

---

## 10. Extended Pack Summary Frame (0x406)

High-resolution pack voltage and current for receivers that need sub-100 mV precision or support packs above 327 V. Not part of the ecosystem broadcast format — not parsed by Victron/SMA/Fronius chargers. Transmitted at the same rate as 0x356.

### Frame Layout

Transmitted every 500 ms.

| Byte | Signal | Bits | Encoding | Description |
|---|---|---|---|---|
| 0–3 | PackVoltage | 32 | 1 mV/LSB, uint32 | Total pack voltage, little-endian. Range 0–4294967 V |
| 4–7 | PackCurrent | 32 | 1 mA/LSB, int32 | Pack current, little-endian. Positive = charging, negative = discharging |

> **Note:** No CRC — frame is 8 bytes, all payload. Temperature is omitted until BCC NTC readback is wired in; use 0x403 for per-module temperatures in the meantime.

---

## 11. UDS Diagnostic Interface

The UDS interface provides configuration, fault management, and firmware update over ISO 14229. Accessible with any UDS-capable tool (Vector CANalyzer, PEAK PCAN, python-udsoncan, etc.).

| Parameter | Value |
|---|---|
| Request ID | 0x7E0 |
| Response ID | 0x7E8 |
| Transport layer | ISO 15765-2 CAN TP |
| Addressing | Physical (1:1) |

---

### 10.1 Sessions and Security

#### Diagnostic Sessions (Service 0x10)

| Session | ID | Capabilities | Timeout |
|---|---|---|---|
| Default | 0x01 | Read all DIDs, read DTCs | No timeout |
| Extended Diagnostic | 0x03 | Read + write all DIDs, clear DTCs, routine control | 5 s inactivity |
| Programming | 0x02 | Firmware update only | Must unlock Security Level 2 first |

#### Security Access (Service 0x27)

| Level | RequestSeed | SendKey | Required For |
|---|---|---|---|
| Level 1 | 0x01 | 0x02 | Write DIDs in extended session |
| Level 2 | 0x11 | 0x12 | Programming session (firmware update) |

> **Note:** Seed/key algorithm TBD. Minimum recommendation: HMAC-SHA256 with a device-unique seed. Three consecutive failed attempts must lock security access for 10 minutes.

---

### 10.2 Data Identifiers

#### Read-Only DIDs (Service 0x22 — Default Session)

| DID | Name | Size | Encoding | Description |
|---|---|---|---|---|
| 0xF190 | DeviceSerial | 17 B | ASCII | Device serial number |
| 0xF187 | PartNumber | 10 B | ASCII | BMS hardware part number |
| 0xF189 | SoftwareVersion | 4 B | uint8 × 4 | Major.Minor.Patch.Build |
| 0xF195 | HardwareVersion | 2 B | uint8 × 2 | Major.Minor |
| 0xD000 | BMS_State | 1 B | Enum | See BMS_State encoding in §8 |
| 0xD001 | Pack_Voltage | 4 B | int32, 1 mV/LSB | Total pack voltage from IVT-S |
| 0xD002 | Pack_Current | 4 B | int32, 1 mA/LSB | Pack current. Positive = charging |
| 0xD003 | Cell_Voltage_Min | 2 B | uint16, 1 mV/LSB | Minimum cell voltage across all cells |
| 0xD004 | Cell_Voltage_Max | 2 B | uint16, 1 mV/LSB | Maximum cell voltage across all cells |
| 0xD005 | Cell_Voltage_Delta | 2 B | uint16, 1 mV/LSB | Max − Min cell voltage |
| 0xD006 | Cell_Voltage_Avg | 2 B | uint16, 1 mV/LSB | Average cell voltage across all cells |
| 0xD007 | Temp_Max | 2 B | int16, 0.01°C/LSB | Highest NTC temperature across all modules |
| 0xD008 | Temp_Min | 2 B | int16, 0.01°C/LSB | Lowest NTC temperature across all modules |
| 0xD009 | SOC | 2 B | uint16, 0.01%/LSB | State of charge |
| 0xD00A | SOH | 2 B | uint16, 0.01%/LSB | State of health (user-configured) |
| 0xD00B | Cell_Voltages_All | 2 B × N×6 | uint16, 1 mV/LSB | All cell voltages sequentially. N = Total_Modules, 6 cells per module |
| 0xD00C | Temps_All | 2 B × N×3 | int16, 0.01°C/LSB | All temperatures sequentially: Temp_A, Temp_B, Temp_Die per module |
| 0xD00D | Balance_Status_All | 1 B × N | Bitmask | Per-module balance mask. N = Total_Modules |
| 0xD00E | Chain0_Modules | 1 B | uint8 | Active module count on chain 0 |
| 0xD00F | Chain1_Modules | 1 B | uint8 | Active module count on chain 1 |
| 0xD010 | Total_Modules | 1 B | uint8 | Total modules across both chains |
| 0xD011 | Contactor_State | 1 B | Bitmask | Current state of K1–K4. See §9 |
| 0xD012 | Precharge_State | 1 B | Enum | Current pre-charge state. See §9 |
| 0xD013 | IVT_Pack_Voltage | 4 B | int32, 1 mV/LSB | IVT-S pack-side voltage |
| 0xD014 | IVT_Inverter_Voltage | 4 B | int32, 1 mV/LSB | IVT-S inverter-side voltage |

#### Read/Write DIDs (Service 0x2E — Extended Session + Security Level 1)

**Balancing Configuration**

| DID | Name | Size | Encoding | Description |
|---|---|---|---|---|
| 0xD100 | Balance_Mode | 1 B | Enum | 0x00 = Disabled, 0x01 = Delta-V, 0x02 = Absolute |
| 0xD101 | Balance_DeltaV_Threshold | 2 B | uint16, 1 mV/LSB | Delta-V threshold in mV. Active when Balance_Mode = 0x01 |
| 0xD102 | Balance_Absolute_Target | 2 B | uint16, 1 mV/LSB | Absolute target voltage in mV. Active when Balance_Mode = 0x02 |
| 0xD103 | Balance_Inhibit_Pack_Voltage | 2 B | uint16, 10 mV/LSB | Minimum pack voltage below which balancing is inhibited |
| 0xD104 | Balance_Min_Cell_Voltage | 2 B | uint16, 1 mV/LSB | Minimum cell voltage below which that cell will not be balanced |

**SOC / SOH Configuration**

| DID | Name | Size | Encoding | Description |
|---|---|---|---|---|
| 0xD105 | SOH_Override | 2 B | uint16, 0.01%/LSB | User-configured SOH value written to 0x355 directly |
| 0xD106 | SOC_Method | 1 B | Enum | 0x00 = Coulomb counting, 0x01 = OCV lookup (reserved for future) |

**Protection Thresholds**

| DID | Name | Size | Encoding | Description |
|---|---|---|---|---|
| 0xD110 | OVP_Threshold | 2 B | uint16, 1 mV/LSB | Cell overvoltage protection trip threshold |
| 0xD111 | OVP_Warning | 2 B | uint16, 1 mV/LSB | Cell overvoltage warning threshold |
| 0xD112 | UVP_Threshold | 2 B | uint16, 1 mV/LSB | Cell undervoltage protection trip threshold |
| 0xD113 | UVP_Warning | 2 B | uint16, 1 mV/LSB | Cell undervoltage warning threshold |
| 0xD114 | OTP_Threshold | 2 B | int16, 0.01°C/LSB | Overtemperature protection trip threshold |
| 0xD115 | OTP_Warning | 2 B | int16, 0.01°C/LSB | Overtemperature warning threshold |
| 0xD116 | UTP_Threshold | 2 B | int16, 0.01°C/LSB | Undertemperature protection trip threshold |
| 0xD117 | UTP_Warning | 2 B | int16, 0.01°C/LSB | Undertemperature warning threshold |
| 0xD118 | OCP_Charge_Threshold | 4 B | uint32, 1 mA/LSB | Overcurrent protection threshold for charge |
| 0xD119 | OCP_Discharge_Threshold | 4 B | uint32, 1 mA/LSB | Overcurrent protection threshold for discharge |

**Contactor Configuration**

| DID | Name | Size | Encoding | Description |
|---|---|---|---|---|
| 0xD200 | K1_Role | 1 B | Enum | 0x00 = Main negative, 0x01 = Main positive, 0x02 = Pre-charge, 0x03 = Charge path, 0xFF = Disabled |
| 0xD201 | K2_Role | 1 B | Enum | Same encoding as K1_Role |
| 0xD202 | K3_Role | 1 B | Enum | Same encoding as K1_Role |
| 0xD203 | K4_Role | 1 B | Enum | Same encoding as K1_Role |
| 0xD204 | Precharge_Completion_Threshold | 2 B | uint16, 1 mV/LSB | Delta-V between pack and inverter side below which pre-charge is considered complete |
| 0xD205 | Precharge_Timeout | 2 B | uint16, 1 ms/LSB | Maximum pre-charge duration before fault |
| 0xD206 | Precharge_Min_Voltage | 2 B | uint16, 1 mV/LSB | Minimum inverter-side voltage to consider pre-charge complete |

**Chain Configuration**

| DID | Name | Size | Encoding | Description |
|---|---|---|---|---|
| 0xD300 | Chain0_Module_Count | 1 B | uint8, 0–15 | Number of modules on chain 0 |
| 0xD301 | Chain1_Module_Count | 1 B | uint8, 0–15 | Number of modules on chain 1 |

> **Note:** Writing chain module counts triggers a BMS re-initialisation sequence. The BMS transitions to Init state, re-enumerates the chain, then returns to Ready state. A 0x404 config broadcast is sent immediately after re-initialisation completes.

---

### 10.3 Fault Codes (DTCs)

Fault codes are read using UDS Service 0x19 (ReadDTCInformation) and cleared using Service 0x14 (ClearDiagnosticInformation). Clearing DTCs requires Extended Session + Security Level 1.

| DTC | Name | Description |
|---|---|---|
| 0xC00001 | Cell_Overvoltage | One or more cells exceeded OVP_Threshold |
| 0xC00002 | Cell_Undervoltage | One or more cells fell below UVP_Threshold |
| 0xC00003 | Overtemperature | One or more sensors exceeded OTP_Threshold |
| 0xC00004 | Undertemperature | One or more sensors fell below UTP_Threshold |
| 0xC00005 | OCP_Charge | Charge current exceeded OCP_Charge_Threshold |
| 0xC00006 | OCP_Discharge | Discharge current exceeded OCP_Discharge_Threshold |
| 0xC00007 | Precharge_Timeout | Pre-charge did not complete within Precharge_Timeout |
| 0xC00008 | Precharge_Fault | Pre-charge fault (welded contactor suspected or voltage sensor fault) |
| 0xC00009 | Contactor_Fault | Contactor feedback mismatch (commanded state ≠ measured state) |
| 0xC0000A | Chain0_Comm_Fault | SPI communication fault on chain 0 |
| 0xC0000B | Chain1_Comm_Fault | SPI communication fault on chain 1 |
| 0xC0000C | IVT_Comm_Fault | IVT-S communication fault |
| 0xC0000D | Cell_Voltage_Sensor_Fault | MC33772B reported internal voltage measurement fault |
| 0xC0000E | Temp_Sensor_Fault | One or more temperature sensors reading 0x7FFF (disconnected or out of range) |
| 0xC0000F | Config_Invalid | Stored configuration checksum failure — BMS running with defaults |

---

### 10.4 Routine Control

Routines are invoked using UDS Service 0x31 (RoutineControl). All routines require Extended Session + Security Level 1.

| Routine ID | Name | Description |
|---|---|---|
| 0xD001 | Manual_Balance_Cycle | Triggers a single balancing pass using current Balance_Mode and Balance_Param. Returns when complete or on fault |
| 0xD002 | Balance_Stop | Immediately halts any active balancing on all modules |
| 0xD003 | Contactor_Open_All | Opens all contactors. For service/safety use only |
| 0xD004 | Precharge_Sequence | Manually triggers the pre-charge and close sequence |
| 0xD005 | Chain_Reinit | Re-initialises SPI chain enumeration without full BMS restart |
| 0xD006 | Clear_All_DTCs | Clears all stored DTCs. Equivalent to Service 0x14 with group 0xFFFFFF |
| 0xD007 | Save_Config | Persists all current DID values to non-volatile storage |

---

### 10.5 Firmware Update

Firmware update uses the standard UDS programming sequence. Programming session requires Security Level 2.

**Sequence:**

```
1. 0x10 0x02                         — Enter programming session
2. 0x27 0x11 → 0x27 0x12 <key>      — Unlock Security Level 2
3. 0x34 ...                          — Request download (address, length, compression)
4. 0x36 ... (repeat)                 — Transfer data blocks
5. 0x37                              — Request transfer exit
6. 0x31 0xFF 0x01                    — Validate firmware checksum (routine 0xFF01)
7. 0x11 0x01                         — ECU reset
```

> **Note:** The BMS will not close contactors while in programming session. If contactors are already closed when a programming session is requested, the request will be rejected with NRC 0x22 (conditionsNotCorrect).

---

## 11. Timing

| Frame | Interval | Notes |
|---|---|---|
| 0x351 | 1000 ms | |
| 0x355 | 1000 ms | |
| 0x356 | 500 ms | |
| 0x359 | 500 ms | |
| 0x35C | 1000 ms | |
| 0x400 | 100 ms/module | Full cycle = Total_Modules × 100 ms |
| 0x401 | 100 ms/module | Full cycle = Total_Modules × 100 ms. Transmitted independently of 0x400 |
| 0x402 | 500 ms/module | Full cycle = Total_Modules × 500 ms |
| 0x403 | 500 ms/module | Full cycle = Total_Modules × 500 ms |
| 0x404 | 5000 ms + on change | Always transmitted on power-on before other custom frames |
| 0x405 | 100 ms | |

> **Note:** At maximum configuration (30 modules), 0x400 and 0x401 each complete a full cycle in 3000 ms (30 frames × 100 ms). Bus load at maximum configuration at 500 kbps is well under 10%.

---

## 12. CRC Calculation

All custom broadcast frames (0x400–0x405) include a CRC-8/AUTOSAR checksum in byte 7, calculated over bytes 0–6.

**Algorithm parameters:**

| Parameter | Value |
|---|---|
| Polynomial | 0x2F |
| Initial value | 0xFF |
| Input reflection | No |
| Output reflection | No |
| Final XOR | 0xFF |
| Check value | 0xDF (CRC of ASCII "123456789") |

**Reference C implementation:**

```c
static const uint8_t crc8_autosar_table[256] = {
    0x00, 0x2F, 0x5E, 0x71, 0xBC, 0x93, 0xE2, 0xCD,
    /* ... full 256-entry table ... */
};

uint8_t crc8_autosar(const uint8_t *data, size_t len) {
    uint8_t crc = 0xFF;
    for (size_t i = 0; i < len; i++) {
        crc = crc8_autosar_table[crc ^ data[i]];
    }
    return crc ^ 0xFF;
}

/* Usage: byte 7 = crc8_autosar(frame, 7) */
```

> **Note:** The MC33772B uses CRC-8/AUTOSAR internally on its SPI bus. The same lookup table used for SPI communication can be reused directly for CAN frame CRCs.

---

*BMS CAN Protocol v0.1 — Draft. All frame definitions, DID addresses, and DTC codes are subject to change prior to v1.0 release.*