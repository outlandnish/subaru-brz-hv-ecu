#pragma once
#include <stdint.h>
#include "can.h"

/**
 * M3 CAN (PCS/BMS protocol) message builders.
 *
 * Signal layout derived from Model3_ETH.compact.json.
 * All multi-byte fields are little-endian on the wire.
 *
 * Key periodic messages (10 ms unless noted):
 *   0x20A  HVP_contactorState   — contactor stage to PCS
 *   0x22A  HVP_pcsControl       — PCS mode request + DC link voltage target
 *   0x212  BMS_status           — BMS/HV state heartbeat (100 ms)
 *   0x3B2  BMS_log2             — MIA keepalive (10 ms)
 */

// Contactor stage encoding for 0x20A
enum M3ContactorStage : uint8_t {
    M3_CONTACTOR_OPEN       = 0,  // All open
    M3_CONTACTOR_PRECHARGE  = 1,  // Neg pulled in, pos precharge
    M3_CONTACTOR_CLOSED     = 2,  // Both economized
};

// PCS control mode encoding for 0x22A bits[16:17]
enum M3PCSControl : uint8_t {
    M3_PCS_SHUTDOWN   = 0,
    M3_PCS_SUPPORT    = 1,
    M3_PCS_PRECHARGE  = 2,
    M3_PCS_DISCHARGE  = 3,
};

class M3CANManager {
public:
    explicit M3CANManager(CANBus *bus);

    // Called by BMS when charging starts/stops.
    void set_charge_enabled(bool enabled);

    // Update charge power request — call whenever safe charge current or pack voltage changes.
    // watts = 0 tells PCS to stop delivering power (e.g. balancing pause).
    void set_charge_power_watts(uint16_t watts);

    // Send individual frames
    void send_contactor_state(M3ContactorStage stage);
    void send_pcs_control(M3PCSControl ctrl, float hv_voltage_v,
                          bool charge_hw, bool dcdc_hw);
    void send_bms_status(bool hv_active, bool precharging, bool charging,
                         bool charge_request);
    void send_bms_mia_keepalive();

    // Process an incoming M3 CAN frame (call from M3 CAN RX loop).
    // Currently handles: 0x21D CP_evseStatus (J1772 pilot current limit).
    void process_can_frame(const CAN_FRAME *frame);

    // Returns the J1772 pilot current limit from the last 0x21D frame (0 if unseen).
    float get_cp_current_limit_a() const { return cp_pilot_current_a; }

    // Periodic tick — call every 10 ms from M3 CAN task.
    void tick(M3PCSControl ctrl, float hv_voltage_v,
              bool charge_hw, bool dcdc_hw,
              bool hv_active, bool precharging,
              bool charging, bool charge_request);

private:
    CANBus *can;
    uint16_t tick_count;
    uint8_t  bms_mux;

    bool     charge_enabled;    // set by set_charge_enabled()
    uint16_t charge_power_w;    // set by set_charge_power_watts()
    volatile float cp_pilot_current_a;  // from 0x21D CP_evseStatus

    void send_charge_power_request();   // 0x2B2

    static void encode_contactor_state(M3ContactorStage stage, uint8_t out[6]);
    static void encode_pcs_control(M3PCSControl ctrl, float hv_voltage_v,
                                   bool charge_hw, bool dcdc_hw, uint8_t out[4]);
    static void encode_bms_status(bool hv_active, bool precharging,
                                  bool charging, bool charge_request, uint8_t out[8]);
};
