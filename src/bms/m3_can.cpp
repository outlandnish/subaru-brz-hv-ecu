#include "m3_can.h"
#include <string.h>

M3CANManager::M3CANManager(CANBus *bus)
    : can(bus), tick_count(0), bms_mux(0), charge_enabled(false),
      charge_power_w(0), cp_pilot_current_a(0.0f) {}

void M3CANManager::set_charge_enabled(bool enabled) {
    charge_enabled = enabled;
    if (!enabled) charge_power_w = 0;
}

void M3CANManager::set_charge_power_watts(uint16_t watts) {
    charge_power_w = watts;
}

// ── 0x20A HVP_contactorState — 6 bytes ───────────────────────────────────────
// Signal packing (little-endian 48-bit word):
//   bits  2:0   negContactorState   OPEN=1, PULLED_IN=4, ECONOMIZED=6
//   bits  5:3   posContactorState   OPEN=1, PRECHARGE=2, ECONOMIZED=6
//   bits 10:8   contactorSetState   OPEN=1, CLOSING=2, CLOSED=5
//   bit  35     packCtrsClosingAllowed
//   bit  36     dcLinkAllowedToEnergize
//   bit  40     hvilStatus          STATUS_OK=1
void M3CANManager::encode_contactor_state(M3ContactorStage stage, uint8_t out[6]) {
    uint8_t neg, pos, setst, closing_allowed, energize;
    switch (stage) {
    case M3_CONTACTOR_PRECHARGE:
        neg = 4; pos = 2; setst = 2; closing_allowed = 1; energize = 1;
        break;
    case M3_CONTACTOR_CLOSED:
        neg = 6; pos = 6; setst = 5; closing_allowed = 1; energize = 1;
        break;
    default: // OPEN
        neg = 1; pos = 1; setst = 1; closing_allowed = 0; energize = 0;
        break;
    }
    uint64_t w = 0;
    w |= (uint64_t)neg          <<  0;
    w |= (uint64_t)pos          <<  3;
    w |= (uint64_t)setst        <<  8;
    w |= (uint64_t)closing_allowed << 35;
    w |= (uint64_t)energize     << 36;
    w |= (uint64_t)1            << 40;  // hvilStatus=STATUS_OK
    memcpy(out, &w, 6);
}

// ── 0x22A HVP_pcsControl — 4 bytes ───────────────────────────────────────────
// Signal packing (little-endian 32-bit word):
//   bits 15:0   HVP_dcLinkVoltageRequest  scale=0.1 V, signed
//   bits 17:16  HVP_pcsControlRequest     0=SHUTDOWN 1=SUPPORT 2=PRECHARGE 3=DISCHARGE
//   bit  18     HVP_pcsChargeHwEnabled
//   bit  19     HVP_pcsDcdcHwEnabled
void M3CANManager::encode_pcs_control(M3PCSControl ctrl, float hv_voltage_v,
                                       bool charge_hw, bool dcdc_hw, uint8_t out[4]) {
    uint16_t v_raw = (uint16_t)((int16_t)(hv_voltage_v / 0.1f)) & 0xFFFF;
    uint32_t w = 0;
    w |= (uint32_t)v_raw;
    w |= (uint32_t)ctrl      << 16;
    w |= (uint32_t)charge_hw << 18;
    w |= (uint32_t)dcdc_hw   << 19;
    memcpy(out, &w, 4);
}

// ── 0x212 BMS_status — 8 bytes ────────────────────────────────────────────────
// Signal packing (little-endian 64-bit word):
//   bit  0     hvacPowerRequest  = 1
//   bit  4     updateAllowed     = 1
//   bit  7     pcsPwmEnabled     = 1
//   bits 10:8  contactorState    BMS_CTRSET_OPEN=1, CLOSED=4
//   bits 13:11 uiChargeStatus    BMS_NO_POWER=1, CHARGING=3
//   bits 18:16 hvState           HV_DOWN=0, HV_COMING_UP=1, HV_UP=6, HV_UP_FOR_CHARGE=4
//   bit  29    chargeRequest
//   bits 35:32 state             BMS_STANDBY=0, SUPPORT=2, CHARGE=3
//   bits 59:56 smStateRequest    = state
void M3CANManager::encode_bms_status(bool hv_active, bool precharging,
                                      bool charging, bool charge_request,
                                      uint8_t out[8]) {
    uint8_t ctrs, ui_chg, hv_st, state;
    if (precharging) {
        ctrs = 4; ui_chg = 3; hv_st = 1; state = charge_request ? 3 : 2;
    } else if (hv_active && charging) {
        ctrs = 4; ui_chg = 3; hv_st = 4; state = 3;
    } else if (hv_active) {
        ctrs = 4; ui_chg = 1; hv_st = 6; state = 2;
    } else {
        ctrs = 1; ui_chg = 1; hv_st = 0; state = 0;
    }
    uint64_t w = 0;
    w |= (uint64_t)1           <<  0;  // hvacPowerRequest
    w |= (uint64_t)1           <<  4;  // updateAllowed
    w |= (uint64_t)1           <<  7;  // pcsPwmEnabled
    w |= (uint64_t)ctrs        <<  8;
    w |= (uint64_t)ui_chg      << 11;
    w |= (uint64_t)hv_st       << 16;
    w |= (uint64_t)(charge_request ? 1ULL : 0ULL) << 29;
    w |= (uint64_t)state       << 32;
    w |= (uint64_t)state       << 56;  // smStateRequest = state
    memcpy(out, &w, 8);
}

// ── 0x21D CP_evseStatus receiver ─────────────────────────────────────────────
// CP_pilotCurrent : bit 8, 8-bit unsigned, scale=0.5 A/LSB
// J1772 pilot frequency encodes the current limit; CP ECU decodes and publishes here.
void M3CANManager::process_can_frame(const CAN_FRAME *frame) {
    if (!frame) return;
    if (frame->id == 0x21D && frame->length >= 2) {
        uint8_t raw = frame->data.uint8[1];  // byte 1 = bits 8..15
        cp_pilot_current_a = raw * 0.5f;
    }
}

// ── Public senders ────────────────────────────────────────────────────────────

void M3CANManager::send_contactor_state(M3ContactorStage stage) {
    if (!can) return;
    uint8_t buf[6];
    encode_contactor_state(stage, buf);
    can->sendMessage(0x20A, buf, 6);
}

void M3CANManager::send_pcs_control(M3PCSControl ctrl, float hv_voltage_v,
                                     bool charge_hw, bool dcdc_hw) {
    if (!can) return;
    uint8_t buf[4];
    encode_pcs_control(ctrl, hv_voltage_v, charge_hw, dcdc_hw, buf);
    can->sendMessage(0x22A, buf, 4);
}

void M3CANManager::send_bms_status(bool hv_active, bool precharging,
                                    bool charging, bool charge_request) {
    if (!can) return;
    uint8_t buf[8];
    encode_bms_status(hv_active, precharging, charging, charge_request, buf);
    can->sendMessage(0x212, buf, 8);
}

void M3CANManager::send_bms_mia_keepalive() {
    if (!can) return;
    // Alternates between two payload variants (mux)
    static const uint8_t payloads[2][8] = {
        {0xE5, 0x0D, 0xEB, 0xFF, 0x0C, 0x66, 0xBB, 0x11},
        {0xE3, 0x5D, 0xFB, 0xFF, 0x0C, 0x66, 0xBB, 0x06},
    };
    uint8_t buf[8];
    memcpy(buf, payloads[bms_mux], 8);
    can->sendMessage(0x3B2, buf, 8);
    bms_mux ^= 1;
}

// ── 0x2B2 charge power request — 5 bytes ─────────────────────────────────────
// bytes 0-1: power in watts, little-endian uint16
// bytes 2-4: fixed 0x00
void M3CANManager::send_charge_power_request() {
    if (!can) return;
    uint8_t buf[5] = {
        (uint8_t)(charge_power_w & 0xFF),
        (uint8_t)(charge_power_w >> 8),
        0x00, 0x00, 0x00
    };
    can->sendMessage(0x2B2, buf, 5);
}

// ── Periodic tick (call every 10 ms) ─────────────────────────────────────────
// Rate schedule:
//   Every tick (10 ms):  0x22A, 0x3B2
//   Every 10 ticks (100 ms): 0x20A, 0x212
//   Every 10 ticks (100 ms, charge_enabled): 0x2B2
void M3CANManager::tick(M3PCSControl ctrl, float hv_voltage_v,
                         bool charge_hw, bool dcdc_hw,
                         bool hv_active, bool precharging,
                         bool charging, bool charge_request) {
    send_pcs_control(ctrl, hv_voltage_v, charge_hw, dcdc_hw);
    send_bms_mia_keepalive();

    if (tick_count % 10 == 0) {
        M3ContactorStage stage;
        if (precharging)      stage = M3_CONTACTOR_PRECHARGE;
        else if (hv_active)   stage = M3_CONTACTOR_CLOSED;
        else                  stage = M3_CONTACTOR_OPEN;
        send_contactor_state(stage);
        send_bms_status(hv_active, precharging, charging, charge_request);
        if (charge_enabled) send_charge_power_request();
    }
    tick_count++;
}
