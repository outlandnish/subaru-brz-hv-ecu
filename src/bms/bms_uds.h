#pragma once
/*
 * BMS UDS server — ISO 14229 diagnostic interface per EXTERNAL_BMS_PLAN §10.
 *
 * Sessions:
 *   0x01  Default            — ReadDID, ReadDTC
 *   0x03  Extended Diag      — WriteDID, ClearDTC, RoutineCtrl (needs SecLvl 1)
 *   0x02  Programming        — ECUReset to bootloader (needs SecLvl 2)
 *
 * Security levels:
 *   0x01/0x02  Level 1 — write DIDs, clear DTCs, routine control
 *   0x11/0x12  Level 2 — programming session (ECU reset to bootloader)
 *
 * This class owns the UDSServer_t state and exposes start_tasks() /
 * poll() (called from a FreeRTOS task in main_bms_can.cpp).
 */

#include "iso14229.h"
#include "bms_uds_tp.h"
#include "bms_can.h"
#include "bms.h"
#include "ivt-s/ivt_shunt.h"

// Active DTC bitmask indices — one bit per DTC in §10.3
#define DTC_OVP           0x00
#define DTC_UVP           0x01
#define DTC_OTP           0x02
#define DTC_UTP           0x03
#define DTC_OCP_CHG       0x04
#define DTC_OCP_DIS       0x05
#define DTC_PRECHARGE_TO  0x06
#define DTC_PRECHARGE_FLT 0x07
#define DTC_CONTACTOR_FLT 0x08
#define DTC_CHAIN0_COMM   0x09
#define DTC_CHAIN1_COMM   0x0A
#define DTC_IVT_COMM      0x0B
#define DTC_VCELL_SENSOR  0x0C
#define DTC_TEMP_SENSOR   0x0D
#define DTC_CFG_INVALID   0x0E
#define DTC_COUNT         15

struct BMSDTCRecord {
    uint32_t dtc_code;    // 3-byte DTC (e.g. 0xC00001)
    uint8_t  status;      // ISO 14229 DTC status byte
};

class BMSUDSServer {
public:
    BMSUDSServer(BatteryManagementSystem *bms, IVTShunt *ivt, BMSCANConfig *can_cfg);

    // Call once after bms_uds_tp_init()
    void init();

    // Start the FreeRTOS poll task
    bool start_tasks();

    // Expose DTC setting so broadcast layer / BMS can flag faults
    void set_dtc(uint8_t dtc_idx, bool active);
    void clear_all_dtcs();

    // Let the broadcaster update runtime-writable fields it mirrors
    BMSCANConfig *get_can_cfg() { return can_cfg; }

private:
    BatteryManagementSystem *bms;
    IVTShunt                *ivt;
    BMSCANConfig            *can_cfg;

    UDSServer_t srv;

    BMSDTCRecord dtc_table[DTC_COUNT];

    TaskHandle_t task_handle;

    // Security state
    uint8_t  sec_fail_count;
    uint32_t sec_lockout_until_ms;

    // Runtime-writable config (persisted in can_cfg / charging_config)
    // Stored locally so RDBI can read them back after WDBI
    uint16_t soh_x100;                   // DID 0xD105
    uint8_t  balance_mode;               // DID 0xD100
    uint16_t balance_delta_mv;           // DID 0xD101
    uint16_t balance_abs_mv;             // DID 0xD102
    uint16_t balance_inhibit_pack_mv;    // DID 0xD103
    uint16_t balance_min_cell_mv;        // DID 0xD104
    uint8_t  soc_method;                 // DID 0xD106
    uint16_t ovp_threshold_mv;           // DID 0xD110
    uint16_t ovp_warning_mv;             // DID 0xD111
    uint16_t uvp_threshold_mv;           // DID 0xD112
    uint16_t uvp_warning_mv;             // DID 0xD113
    int16_t  otp_threshold_cdeg;         // DID 0xD114
    int16_t  otp_warning_cdeg;           // DID 0xD115
    int16_t  utp_threshold_cdeg;         // DID 0xD116
    int16_t  utp_warning_cdeg;           // DID 0xD117
    uint32_t ocp_charge_ma;              // DID 0xD118
    uint32_t ocp_discharge_ma;           // DID 0xD119
    uint8_t  aux_contactor_mode;         // DID 0xD200
    uint8_t  aux_pin0_role;              // DID 0xD201
    uint8_t  aux_pin1_role;              // DID 0xD202
    uint8_t  nacs_pin;                   // DID 0xD203
    uint16_t precharge_completion_mv;    // DID 0xD204
    uint16_t precharge_timeout_ms_val;   // DID 0xD205
    uint16_t precharge_min_voltage_mv;   // DID 0xD206
    uint8_t  nacs_dc_level;             // DID 0xD207
    uint8_t  chain0_module_count;        // DID 0xD300
    uint8_t  chain1_module_count;        // DID 0xD301
    uint8_t  bcc0_device_type;           // DID 0xD302
    uint8_t  bcc1_device_type;           // DID 0xD303
    // Hardware config — DID 0xD400 range
    uint8_t  can_node_id;                // DID 0xD400
    uint16_t pwm_frequency_hz;           // DID 0xD401
    uint8_t  engage_duty_pct;            // DID 0xD402
    uint8_t  contactor_aux0_hold_duty_pct; // DID 0xD403
    uint8_t  contactor_aux1_hold_duty_pct; // DID 0xD404
    uint16_t engage_time_ms;             // DID 0xD405
    // Battery / timing config — DID 0xD500 range
    uint16_t battery_capacity_dah;       // DID 0xD500  (0.1 Ah units)
    uint16_t min_soc_pct_x10;           // DID 0xD501  (0.1 % units)
    uint16_t max_soc_pct_x10;           // DID 0xD502
    uint16_t init_soc_pct_x10;          // DID 0xD503
    uint16_t balance_timer_min;          // DID 0xD504
    uint16_t measure_interval_ms;        // DID 0xD505
    uint16_t balance_hv_off_min;         // DID 0xD506
    uint8_t  ivt_configured;             // DID 0xD507
    uint16_t precharge_check_int_ms;     // DID 0xD508
    uint16_t comm_timeout_ms_val;        // DID 0xD509
    uint16_t fault_check_int_ms;         // DID 0xD50A

    // iso14229 server callback
    static UDSErr_t uds_callback(UDSServer_t *srv, UDSEvent_t evt, void *arg);
    UDSErr_t handle_event(UDSEvent_t evt, void *arg);

    // Service handlers
    UDSErr_t handle_sess_ctrl(UDSDiagSessCtrlArgs_t *a);
    UDSErr_t handle_ecu_reset(UDSECUResetArgs_t *a);
    UDSErr_t handle_clear_dtc(UDSCDIArgs_t *a);
    UDSErr_t handle_read_dtc(UDSRDTCIArgs_t *a);
    UDSErr_t handle_rdbi(UDSRDBIArgs_t *a);
    UDSErr_t handle_wdbi(UDSWDBIArgs_t *a);
    UDSErr_t handle_sec_seed(UDSSecAccessRequestSeedArgs_t *a);
    UDSErr_t handle_sec_key(UDSSecAccessValidateKeyArgs_t *a);
    UDSErr_t handle_routine(UDSRoutineCtrlArgs_t *a);
    UDSErr_t handle_scheduled_reset(uint8_t reset_type);

    // Helpers
    uint32_t compute_key(uint8_t level, const uint8_t *seed, uint16_t seed_len) const;
    void     generate_seed(uint8_t level, uint8_t *seed_out, uint8_t seed_len) const;

    static void task_wrapper(void *pv);
    void        task_loop();
};
