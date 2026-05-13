#include "bms_uds.h"
#include "boot_shared.h"
#include "debug_serial.h"
#include "params.h"
#include "param_save.h"
#include "version_gen.h"
#include "Arduino.h"
#include <string.h>

// ── DTC table: 3-byte codes per EXTERNAL_BMS_PLAN §10.3 ──────────────────────
static const uint32_t k_dtc_codes[DTC_COUNT] = {
    0xC00001, // OVP
    0xC00002, // UVP
    0xC00003, // OTP
    0xC00004, // UTP
    0xC00005, // OCP_Charge
    0xC00006, // OCP_Discharge
    0xC00007, // Precharge_Timeout
    0xC00008, // Precharge_Fault
    0xC00009, // Contactor_Fault
    0xC0000A, // Chain0_Comm
    0xC0000B, // Chain1_Comm
    0xC0000C, // IVT_Comm
    0xC0000D, // Cell_Voltage_Sensor
    0xC0000E, // Temp_Sensor
    0xC0000F, // Config_Invalid
};

// ── Seed/key: XOR-based placeholder (plan says HMAC-SHA256; swap in later) ───
// Level 1: key = ~seed  (4-byte seed)
// Level 2: key = seed ^ 0xA5A5A5A5
static constexpr uint8_t SEED_LEN = 4;

// ── Constructor ───────────────────────────────────────────────────────────────
BMSUDSServer::BMSUDSServer(BatteryManagementSystem *bms_, IVTShunt *ivt_,
                           BMSCANConfig *can_cfg_)
    : bms(bms_), ivt(ivt_), can_cfg(can_cfg_),
      task_handle(nullptr),
      sec_fail_count(0), sec_lockout_until_ms(0) {
    memset(&srv, 0, sizeof(srv));

    for (uint8_t i = 0; i < DTC_COUNT; i++) {
        dtc_table[i].dtc_code = k_dtc_codes[i];
        dtc_table[i].status   = 0x00;
    }

    // Writable DID defaults derived from can_cfg / charging_config at init time
    soh_x100               = can_cfg->soh_percent_x100;
    balance_mode           = (uint8_t)can_cfg->balance_mode;
    balance_delta_mv       = can_cfg->balance_param_mv;
    balance_abs_mv         = can_cfg->balance_param_mv;
    balance_inhibit_pack_mv = 0;
    balance_min_cell_mv    = 2500;
    soc_method             = 0x00;   // Coulomb counting

    const BMSChargingConfig &cc = bms->get_charging_config();
    // Convert V/A to threshold mV/mA for DID storage
    ovp_threshold_mv  = (uint16_t)(cc.target_cell_voltage * 1000.0f);
    ovp_warning_mv    = (uint16_t)(cc.target_cell_voltage * 1000.0f - 50);
    uvp_threshold_mv  = 2500;
    uvp_warning_mv    = 2600;
    otp_threshold_cdeg = 4500;  //  45.00°C
    otp_warning_cdeg   = 4000;  //  40.00°C
    utp_threshold_cdeg = -1000; // -10.00°C
    utp_warning_cdeg   = -500;  //  -5.00°C
    ocp_charge_ma      = (uint32_t)(cc.max_charge_current_a * 1000.0f);
    ocp_discharge_ma   = (uint32_t)(cc.max_charge_current_a * 1000.0f);

    k_role[0] = 0x00; // K1 = Main negative
    k_role[1] = 0x01; // K2 = Main positive
    k_role[2] = 0x02; // K3 = Pre-charge
    k_role[3] = 0x03; // K4 = Charge path

    precharge_completion_mv  = 5000;   // 5 V delta
    precharge_timeout_ms_val = 10000;  // 10 s
    precharge_min_voltage_mv = 0;

    chain0_module_count = can_cfg->chain0_modules;
    chain1_module_count = can_cfg->chain1_modules;
}

// ── init ─────────────────────────────────────────────────────────────────────
void BMSUDSServer::init() {
    UDSErr_t err = UDSServerInit(&srv);
    if (err != UDS_OK) {
        debug_printf("UDS: UDSServerInit failed: %d\r\n", err);
        return;
    }
    srv.tp      = &g_bms_uds_tp.hdl;
    srv.fn      = uds_callback;
    srv.fn_data = this;
    srv.s3_ms   = 5100; // 5 s session timeout for extended session
    debug_println("UDS: server initialised (0x7E0/0x7E8)");
}

// ── start_tasks ───────────────────────────────────────────────────────────────
bool BMSUDSServer::start_tasks() {
    BaseType_t r = xTaskCreate(task_wrapper, "UDS_SRV", 4096, this, 2, &task_handle);
    if (r != pdPASS) {
        debug_println("UDS: failed to create server task");
        return false;
    }
    debug_println("UDS: server task started");
    return true;
}

// ── DTC helpers ──────────────────────────────────────────────────────────────
void BMSUDSServer::set_dtc(uint8_t dtc_idx, bool active) {
    if (dtc_idx >= DTC_COUNT) return;
    if (active) {
        dtc_table[dtc_idx].status |= 0x09; // testFailed | confirmedDTC
    } else {
        dtc_table[dtc_idx].status &= ~0x01; // clear testFailed
    }
}

void BMSUDSServer::clear_all_dtcs() {
    for (uint8_t i = 0; i < DTC_COUNT; i++)
        dtc_table[i].status = 0x00;
}

// ── Static task wrapper ───────────────────────────────────────────────────────
void BMSUDSServer::task_wrapper(void *pv) {
    static_cast<BMSUDSServer *>(pv)->task_loop();
}

void BMSUDSServer::task_loop() {
    while (true) {
        bms_uds_tp_poll();
        UDSServerPoll(&srv);
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}

// ── Static callback dispatch ─────────────────────────────────────────────────
UDSErr_t BMSUDSServer::uds_callback(UDSServer_t *srv_ptr, UDSEvent_t evt, void *arg) {
    BMSUDSServer *self = static_cast<BMSUDSServer *>(srv_ptr->fn_data);
    return self->handle_event(evt, arg);
}

// ── Main event dispatcher ─────────────────────────────────────────────────────
UDSErr_t BMSUDSServer::handle_event(UDSEvent_t evt, void *arg) {
    switch (evt) {
    case UDS_EVT_DiagSessCtrl:
        return handle_sess_ctrl(static_cast<UDSDiagSessCtrlArgs_t *>(arg));
    case UDS_EVT_EcuReset:
        return handle_ecu_reset(static_cast<UDSECUResetArgs_t *>(arg));
    case UDS_EVT_ClearDiagnosticInfo:
        return handle_clear_dtc(static_cast<UDSCDIArgs_t *>(arg));
    case UDS_EVT_ReadDTCInformation:
        return handle_read_dtc(static_cast<UDSRDTCIArgs_t *>(arg));
    case UDS_EVT_ReadDataByIdent:
        return handle_rdbi(static_cast<UDSRDBIArgs_t *>(arg));
    case UDS_EVT_WriteDataByIdent:
        return handle_wdbi(static_cast<UDSWDBIArgs_t *>(arg));
    case UDS_EVT_SecAccessRequestSeed:
        return handle_sec_seed(static_cast<UDSSecAccessRequestSeedArgs_t *>(arg));
    case UDS_EVT_SecAccessValidateKey:
        return handle_sec_key(static_cast<UDSSecAccessValidateKeyArgs_t *>(arg));
    case UDS_EVT_RoutineCtrl:
        return handle_routine(static_cast<UDSRoutineCtrlArgs_t *>(arg));
    case UDS_EVT_DoScheduledReset:
        return handle_scheduled_reset(*(static_cast<uint8_t *>(arg)));
    case UDS_EVT_SessionTimeout:
        debug_println("UDS: session timeout");
        return UDS_OK;
    default:
        return UDS_NRC_ServiceNotSupported;
    }
}

// ── 0x10 Diagnostic Session Control ──────────────────────────────────────────
UDSErr_t BMSUDSServer::handle_sess_ctrl(UDSDiagSessCtrlArgs_t *a) {
    switch (a->type) {
    case UDS_LEV_DS_DS:     // 0x01 Default
        return UDS_OK;
    case UDS_LEV_DS_EXTDS:  // 0x03 Extended — no precondition needed
        return UDS_OK;
    case UDS_LEV_DS_PRGS:   // 0x02 Programming — reject if contactors closed
        if (bms->get_hv_state() == HV_Active || bms->get_hv_state() == HV_Precharge)
            return UDS_NRC_ConditionsNotCorrect;
        return UDS_OK;
    default:
        return UDS_NRC_SubFunctionNotSupported;
    }
}

// ── 0x11 ECU Reset ────────────────────────────────────────────────────────────
UDSErr_t BMSUDSServer::handle_ecu_reset(UDSECUResetArgs_t *a) {
    // Hard reset and soft reset both jump to bootloader only in prog session
    // For safety: any ECU reset opens contactors first
    bms->stop_hv_system();

    if (srv.sessionType == UDS_LEV_DS_PRGS) {
        // Signal bootloader stay on next boot
        *(volatile uint32_t *)BOOTLOADER_FLAG_ADDR = BOOTLOADER_MAGIC;
    }
    // powerDownTimeMillis controls delay before UDS_EVT_DoScheduledReset fires
    a->powerDownTimeMillis = 200;
    return UDS_OK;
}

UDSErr_t BMSUDSServer::handle_scheduled_reset(uint8_t /*reset_type*/) {
    NVIC_SystemReset();
    return UDS_OK; // unreachable
}

// ── 0x14 Clear Diagnostic Information ────────────────────────────────────────
UDSErr_t BMSUDSServer::handle_clear_dtc(UDSCDIArgs_t *a) {
    // Requires extended session + security level 1
    if (srv.sessionType != UDS_LEV_DS_EXTDS)
        return UDS_NRC_ServiceNotSupportedInActiveSession;
    if (srv.securityLevel < 1)
        return UDS_NRC_SecurityAccessDenied;

    // groupOfDTC 0xFFFFFF = clear all
    if (a->groupOfDTC == 0xFFFFFF) {
        clear_all_dtcs();
        return UDS_OK;
    }
    // Clear specific DTC by matching code
    for (uint8_t i = 0; i < DTC_COUNT; i++) {
        if (dtc_table[i].dtc_code == a->groupOfDTC) {
            dtc_table[i].status = 0x00;
            return UDS_OK;
        }
    }
    return UDS_NRC_RequestOutOfRange;
}

// ── 0x19 Read DTC Information ─────────────────────────────────────────────────
// Only subFunc 0x02 (reportDTCByStatusMask) implemented.
UDSErr_t BMSUDSServer::handle_read_dtc(UDSRDTCIArgs_t *a) {
    if (a->type != 0x02) // reportDTCByStatusMask
        return UDS_NRC_SubFunctionNotSupported;

    uint8_t mask = a->subFuncArgs.dtcStatusByMaskArgs.mask;
    for (uint8_t i = 0; i < DTC_COUNT; i++) {
        if (!(dtc_table[i].status & mask)) continue;

        uint8_t rec[4];
        rec[0] = (dtc_table[i].dtc_code >> 16) & 0xFF;
        rec[1] = (dtc_table[i].dtc_code >>  8) & 0xFF;
        rec[2] = (dtc_table[i].dtc_code >>  0) & 0xFF;
        rec[3] = dtc_table[i].status;
        if (a->copy(&srv, rec, sizeof(rec)) != sizeof(rec))
            return UDS_NRC_ResponseTooLong;
    }
    return UDS_OK;
}

// ── Helpers for RDBI copy ─────────────────────────────────────────────────────
#define RDBI_COPY_U8(val)  do { uint8_t _v = (val); return a->copy(&srv, &_v, 1) == 1 ? UDS_OK : UDS_NRC_ResponseTooLong; } while(0)
#define RDBI_COPY_U16(val) do { uint16_t _v = __builtin_bswap16(val); return a->copy(&srv, &_v, 2) == 2 ? UDS_OK : UDS_NRC_ResponseTooLong; } while(0)
#define RDBI_COPY_I16(val) do { uint16_t _v = __builtin_bswap16((uint16_t)(val)); return a->copy(&srv, &_v, 2) == 2 ? UDS_OK : UDS_NRC_ResponseTooLong; } while(0)
#define RDBI_COPY_U32(val) do { uint32_t _v = __builtin_bswap32(val); return a->copy(&srv, &_v, 4) == 4 ? UDS_OK : UDS_NRC_ResponseTooLong; } while(0)
#define RDBI_COPY_I32(val) do { uint32_t _v = __builtin_bswap32((uint32_t)(val)); return a->copy(&srv, &_v, 4) == 4 ? UDS_OK : UDS_NRC_ResponseTooLong; } while(0)

// ── 0x22 Read Data By Identifier ─────────────────────────────────────────────
UDSErr_t BMSUDSServer::handle_rdbi(UDSRDBIArgs_t *a) {
    const uint16_t did = a->dataId;

    // ── Read-only standard IDs ────────────────────────────────────────────────
    if (did == 0xF190) {  // DeviceSerial — 17 ASCII bytes
        char serial[18]; // 17 chars + null
        const uint32_t *uid = (const uint32_t *)0x1FFF7A10UL;
        snprintf(serial, sizeof(serial), "%08lX%08lX0", uid[0], uid[1]);
        return a->copy(&srv, serial, 17) == 17 ? UDS_OK : UDS_NRC_ResponseTooLong;
    }
    if (did == 0xF187) {  // PartNumber — 10 ASCII bytes
        const uint8_t pn[10] = {'H','V','-','E','C','U','-','0','0','1'};
        return a->copy(&srv, pn, 10) == 10 ? UDS_OK : UDS_NRC_ResponseTooLong;
    }
    if (did == 0xF189) {  // SoftwareVersion — 4 × uint8
        const uint8_t ver[4] = { FW_VERSION_MAJOR, FW_VERSION_MINOR,
                                  FW_VERSION_PATCH, 0 };
        return a->copy(&srv, ver, 4) == 4 ? UDS_OK : UDS_NRC_ResponseTooLong;
    }
    if (did == 0xF195) {  // HardwareVersion — 2 × uint8
        const uint8_t hw[2] = { 1, 0 };
        return a->copy(&srv, hw, 2) == 2 ? UDS_OK : UDS_NRC_ResponseTooLong;
    }

    // ── Read-only live data DIDs ──────────────────────────────────────────────
    if (did == 0xD000) {  // BMS_State
        // Map internal state to external protocol state
        ExtBMSState ext = EXT_BMS_INIT;
        switch (bms->get_state()) {
        case BMS_Initialization: ext = EXT_BMS_INIT;      break;
        case BMS_Idle:           ext = EXT_BMS_READY;     break;
        case BMS_Charging:       ext = EXT_BMS_CHARGING;  break;
        case BMS_CellBalancing:  ext = EXT_BMS_BALANCING; break;
        case BMS_Error:          ext = EXT_BMS_FAULT;     break;
        default:                 ext = EXT_BMS_READY;     break;
        }
        RDBI_COPY_U8((uint8_t)ext);
    }
    if (did == 0xD001) {  // Pack_Voltage — int32 mV
        int32_t mv = ivt ? (int32_t)(ivt->get_voltage() * 1000.0f) : 0;
        RDBI_COPY_I32((uint32_t)mv);
    }
    if (did == 0xD002) {  // Pack_Current — int32 mA
        int32_t ma = ivt ? (int32_t)(ivt->get_current() * 1000.0f) : 0;
        RDBI_COPY_I32((uint32_t)ma);
    }
    if (did == 0xD003 || did == 0xD004 || did == 0xD005 || did == 0xD006) {
        uint8_t cnt = 0;
        uint32_t vbuf[BCC_MAX_CELLS];
        bms->get_cell_voltages(vbuf, &cnt);
        if (cnt == 0) return UDS_NRC_ConditionsNotCorrect;

        uint32_t vmin = vbuf[0], vmax = vbuf[0], vsum = 0;
        for (uint8_t i = 0; i < cnt; i++) {
            uint32_t v = vbuf[i] / 1000; // µV → mV
            if (v < vmin) vmin = v;
            if (v > vmax) vmax = v;
            vsum += v;
        }
        uint32_t vavg = vsum / cnt;

        if (did == 0xD003) { RDBI_COPY_U16((uint16_t)vmin); }
        if (did == 0xD004) { RDBI_COPY_U16((uint16_t)vmax); }
        if (did == 0xD005) { RDBI_COPY_U16((uint16_t)(vmax - vmin)); }
        if (did == 0xD006) { RDBI_COPY_U16((uint16_t)vavg); }
    }
    if (did == 0xD007 || did == 0xD008) {
        // Temp not yet wired from BCC readback — return 0x7FFF (invalid)
        RDBI_COPY_I16(0x7FFF);
    }
    if (did == 0xD009) {  // SOC
        RDBI_COPY_U16((uint16_t)(bms->get_soc_precise() * 100.0f));
    }
    if (did == 0xD00A) {  // SOH
        RDBI_COPY_U16(soh_x100);
    }
    if (did == 0xD00B) {  // Cell_Voltages_All
        uint8_t cnt = 0;
        uint32_t vbuf[BCC_MAX_CELLS];
        bms->get_cell_voltages(vbuf, &cnt);
        for (uint8_t i = 0; i < cnt; i++) {
            uint16_t mv_be = __builtin_bswap16((uint16_t)(vbuf[i] / 1000));
            if (a->copy(&srv, &mv_be, 2) != 2) return UDS_NRC_ResponseTooLong;
        }
        return UDS_OK;
    }
    if (did == 0xD00C) {  // Temps_All — return 0x7FFF × N×3
        const uint8_t total = can_cfg->chain0_modules + can_cfg->chain1_modules;
        const uint16_t invalid_be = __builtin_bswap16(0x7FFF);
        for (uint8_t i = 0; i < total * 3; i++) {
            if (a->copy(&srv, &invalid_be, 2) != 2) return UDS_NRC_ResponseTooLong;
        }
        return UDS_OK;
    }
    if (did == 0xD00D) {  // Balance_Status_All
        const uint8_t total = can_cfg->chain0_modules + can_cfg->chain1_modules;
        for (uint8_t i = 0; i < total; i++) {
            uint8_t mask = bms->get_balance_mask(i);
            if (a->copy(&srv, &mask, 1) != 1) return UDS_NRC_ResponseTooLong;
        }
        return UDS_OK;
    }
    if (did == 0xD00E) { RDBI_COPY_U8(can_cfg->chain0_modules); }
    if (did == 0xD00F) { RDBI_COPY_U8(can_cfg->chain1_modules); }
    if (did == 0xD010) { RDBI_COPY_U8((uint8_t)(can_cfg->chain0_modules + can_cfg->chain1_modules)); }
    if (did == 0xD011) { RDBI_COPY_U8(can_cfg->contactor_closed_mask); }
    if (did == 0xD012) {  // Precharge_State
        uint8_t s = 0x00; // Idle
        switch (bms->get_hv_state()) {
        case HV_Precharge: s = 0x01; break;
        case HV_Active:    s = 0x02; break;
        default:           s = 0x00; break;
        }
        RDBI_COPY_U8(s);
    }
    if (did == 0xD013) {  // IVT_Pack_Voltage — int32 mV
        int32_t mv = ivt ? (int32_t)(ivt->get_voltage() * 1000.0f) : 0;
        RDBI_COPY_I32((uint32_t)mv);
    }
    if (did == 0xD014) {  // IVT_Inverter_Voltage — int32 mV
        int32_t mv = ivt ? (int32_t)(ivt->get_voltage2() * 1000.0f) : 0;
        RDBI_COPY_I32((uint32_t)mv);
    }

    // ── Read/Write DIDs — balancing ───────────────────────────────────────────
    if (did == 0xD100) { RDBI_COPY_U8(balance_mode); }
    if (did == 0xD101) { RDBI_COPY_U16(balance_delta_mv); }
    if (did == 0xD102) { RDBI_COPY_U16(balance_abs_mv); }
    if (did == 0xD103) { RDBI_COPY_U16(balance_inhibit_pack_mv); }
    if (did == 0xD104) { RDBI_COPY_U16(balance_min_cell_mv); }

    // SOC/SOH
    if (did == 0xD105) { RDBI_COPY_U16(soh_x100); }
    if (did == 0xD106) { RDBI_COPY_U8(soc_method); }

    // Protection thresholds
    if (did == 0xD110) { RDBI_COPY_U16(ovp_threshold_mv); }
    if (did == 0xD111) { RDBI_COPY_U16(ovp_warning_mv); }
    if (did == 0xD112) { RDBI_COPY_U16(uvp_threshold_mv); }
    if (did == 0xD113) { RDBI_COPY_U16(uvp_warning_mv); }
    if (did == 0xD114) { RDBI_COPY_I16(otp_threshold_cdeg); }
    if (did == 0xD115) { RDBI_COPY_I16(otp_warning_cdeg); }
    if (did == 0xD116) { RDBI_COPY_I16(utp_threshold_cdeg); }
    if (did == 0xD117) { RDBI_COPY_I16(utp_warning_cdeg); }
    if (did == 0xD118) { RDBI_COPY_U32(ocp_charge_ma); }
    if (did == 0xD119) { RDBI_COPY_U32(ocp_discharge_ma); }

    // Contactor config
    if (did >= 0xD200 && did <= 0xD203) { RDBI_COPY_U8(k_role[did - 0xD200]); }
    if (did == 0xD204) { RDBI_COPY_U16(precharge_completion_mv); }
    if (did == 0xD205) { RDBI_COPY_U16(precharge_timeout_ms_val); }
    if (did == 0xD206) { RDBI_COPY_U16(precharge_min_voltage_mv); }

    // Chain config
    if (did == 0xD300) { RDBI_COPY_U8(chain0_module_count); }
    if (did == 0xD301) { RDBI_COPY_U8(chain1_module_count); }

    return UDS_NRC_RequestOutOfRange;
}

// ── 0x2E Write Data By Identifier ────────────────────────────────────────────
UDSErr_t BMSUDSServer::handle_wdbi(UDSWDBIArgs_t *a) {
    if (srv.sessionType != UDS_LEV_DS_EXTDS)
        return UDS_NRC_ServiceNotSupportedInActiveSession;
    if (srv.securityLevel < 1)
        return UDS_NRC_SecurityAccessDenied;

    const uint16_t did = a->dataId;
    const uint8_t *d   = a->data;
    const uint16_t len  = a->len;

    // Inline big-endian decode helpers
#define WR_U8(field, need)  do { if (len < (need)) return UDS_NRC_IncorrectMessageLengthOrInvalidFormat; (field) = d[0]; } while(0)
#define WR_U16(field, need) do { if (len < (need)) return UDS_NRC_IncorrectMessageLengthOrInvalidFormat; (field) = (uint16_t)((d[0] << 8) | d[1]); } while(0)
#define WR_I16(field, need) do { if (len < (need)) return UDS_NRC_IncorrectMessageLengthOrInvalidFormat; (field) = (int16_t)((d[0] << 8) | d[1]); } while(0)
#define WR_U32(field, need) do { if (len < (need)) return UDS_NRC_IncorrectMessageLengthOrInvalidFormat; (field) = ((uint32_t)d[0]<<24)|((uint32_t)d[1]<<16)|((uint32_t)d[2]<<8)|d[3]; } while(0)

    if (did == 0xD100) { WR_U8(balance_mode, 1);
        if (balance_mode > 2) return UDS_NRC_RequestOutOfRange;
        can_cfg->balance_mode = (BalanceMode)balance_mode;
        return UDS_OK;
    }
    if (did == 0xD101) { WR_U16(balance_delta_mv, 2); return UDS_OK; }
    if (did == 0xD102) { WR_U16(balance_abs_mv,   2); return UDS_OK; }
    if (did == 0xD103) { WR_U16(balance_inhibit_pack_mv, 2); return UDS_OK; }
    if (did == 0xD104) { WR_U16(balance_min_cell_mv,     2); return UDS_OK; }

    if (did == 0xD105) {
        WR_U16(soh_x100, 2);
        if (soh_x100 > 10000) return UDS_NRC_RequestOutOfRange;
        can_cfg->soh_percent_x100 = soh_x100;
        return UDS_OK;
    }
    if (did == 0xD106) { WR_U8(soc_method, 1);
        if (soc_method > 1) return UDS_NRC_RequestOutOfRange;
        return UDS_OK;
    }

    if (did == 0xD110) { WR_U16(ovp_threshold_mv, 2); return UDS_OK; }
    if (did == 0xD111) { WR_U16(ovp_warning_mv,   2); return UDS_OK; }
    if (did == 0xD112) { WR_U16(uvp_threshold_mv, 2); return UDS_OK; }
    if (did == 0xD113) { WR_U16(uvp_warning_mv,   2); return UDS_OK; }
    if (did == 0xD114) { WR_I16(otp_threshold_cdeg, 2); return UDS_OK; }
    if (did == 0xD115) { WR_I16(otp_warning_cdeg,   2); return UDS_OK; }
    if (did == 0xD116) { WR_I16(utp_threshold_cdeg, 2); return UDS_OK; }
    if (did == 0xD117) { WR_I16(utp_warning_cdeg,   2); return UDS_OK; }
    if (did == 0xD118) { WR_U32(ocp_charge_ma,    4); return UDS_OK; }
    if (did == 0xD119) { WR_U32(ocp_discharge_ma, 4); return UDS_OK; }

    if (did >= 0xD200 && did <= 0xD203) {
        WR_U8(k_role[did - 0xD200], 1);
        if (k_role[did - 0xD200] > 0x04 && k_role[did - 0xD200] != 0xFF)
            return UDS_NRC_RequestOutOfRange;
        return UDS_OK;
    }
    if (did == 0xD204) { WR_U16(precharge_completion_mv,  2); return UDS_OK; }
    if (did == 0xD205) { WR_U16(precharge_timeout_ms_val, 2); return UDS_OK; }
    if (did == 0xD206) { WR_U16(precharge_min_voltage_mv, 2); return UDS_OK; }

    if (did == 0xD300) {
        WR_U8(chain0_module_count, 1);
        if (chain0_module_count > 15) return UDS_NRC_RequestOutOfRange;
        can_cfg->chain0_modules = chain0_module_count;
        return UDS_OK;
    }
    if (did == 0xD301) {
        WR_U8(chain1_module_count, 1);
        if (chain1_module_count > 15) return UDS_NRC_RequestOutOfRange;
        can_cfg->chain1_modules = chain1_module_count;
        return UDS_OK;
    }

#undef WR_U8
#undef WR_U16
#undef WR_I16
#undef WR_U32

    return UDS_NRC_RequestOutOfRange;
}

// ── 0x27 Security Access — seed ──────────────────────────────────────────────
UDSErr_t BMSUDSServer::handle_sec_seed(UDSSecAccessRequestSeedArgs_t *a) {
    const uint32_t now = millis();

    // Brute-force lockout: 3 consecutive failures → 10 min lockout
    if (sec_fail_count >= 3) {
        if (now < sec_lockout_until_ms)
            return UDS_NRC_RequiredTimeDelayNotExpired;
        sec_fail_count = 0; // lockout expired
    }

    // If already at this level, return all-zeros seed (already unlocked)
    if ((a->level == 0x01 && srv.securityLevel >= 1) ||
        (a->level == 0x11 && srv.securityLevel >= 0x11)) {
        const uint8_t zero[SEED_LEN] = {0, 0, 0, 0};
        return a->copySeed(&srv, zero, SEED_LEN) == SEED_LEN
            ? UDS_OK : UDS_NRC_ResponseTooLong;
    }

    uint8_t seed[SEED_LEN];
    generate_seed(a->level, seed, SEED_LEN);
    return a->copySeed(&srv, seed, SEED_LEN) == SEED_LEN
        ? UDS_OK : UDS_NRC_ResponseTooLong;
}

// ── 0x27 Security Access — key ───────────────────────────────────────────────
UDSErr_t BMSUDSServer::handle_sec_key(UDSSecAccessValidateKeyArgs_t *a) {
    if (a->len < SEED_LEN) return UDS_NRC_InvalidKey;

    const uint32_t now = millis();
    if (sec_fail_count >= 3 && now < sec_lockout_until_ms)
        return UDS_NRC_RequiredTimeDelayNotExpired;

    // Re-derive the seed we sent (deterministic from level + device UID)
    uint8_t seed[SEED_LEN];
    generate_seed(a->level, seed, SEED_LEN);
    uint32_t expected = compute_key(a->level, seed, SEED_LEN);

    uint32_t received = ((uint32_t)a->key[0] << 24) | ((uint32_t)a->key[1] << 16) |
                        ((uint32_t)a->key[2] <<  8) |  (uint32_t)a->key[3];

    if (received != expected) {
        sec_fail_count++;
        if (sec_fail_count >= 3)
            sec_lockout_until_ms = now + 600000UL; // 10 min
        return UDS_NRC_InvalidKey;
    }

    sec_fail_count = 0;
    return UDS_OK;
}

// ── 0x31 Routine Control ─────────────────────────────────────────────────────
UDSErr_t BMSUDSServer::handle_routine(UDSRoutineCtrlArgs_t *a) {
    if (srv.sessionType != UDS_LEV_DS_EXTDS)
        return UDS_NRC_ServiceNotSupportedInActiveSession;
    if (srv.securityLevel < 1)
        return UDS_NRC_SecurityAccessDenied;
    if (a->ctrlType != UDS_LEV_RCTP_STR) // only StartRoutine supported
        return UDS_NRC_SubFunctionNotSupported;

    switch (a->id) {
    case 0xD001: // Manual_Balance_Cycle
        bms->force_balance_cells();
        return UDS_OK;

    case 0xD002: // Balance_Stop
        bms->stop_charging(); // stop charging also halts balancing
        return UDS_OK;

    case 0xD003: // Contactor_Open_All
        bms->stop_hv_system();
        return UDS_OK;

    case 0xD004: // Precharge_Sequence
        bms->start_drive_mode();
        return UDS_OK;

    case 0xD005: // Chain_Reinit — re-init BMS hardware
        // No direct BCC re-enum API yet; do a soft restart instead
        *(volatile uint32_t *)BOOTLOADER_FLAG_ADDR = 0; // skip stay flag
        NVIC_SystemReset();
        return UDS_OK; // unreachable

    case 0xD006: // Clear_All_DTCs
        clear_all_dtcs();
        return UDS_OK;

    case 0xD007: // Save_Config — persist libopeninv params to flash
        parm_save();
        return UDS_OK;

    case 0xFF01: // Validate firmware checksum (called by UDS flashing tool)
        // The bootloader verifies CRC on next boot; nothing to do here.
        return UDS_OK;

    default:
        return UDS_NRC_RequestOutOfRange;
    }
}

// ── Seed/key algorithm ────────────────────────────────────────────────────────
// Deterministic seed: mix device UID with level so it's unique per ECU.
// Key algorithm: Level 1 → bitwise NOT; Level 2 → XOR 0xA5A5A5A5.
// Replace with HMAC-SHA256 for production.
void BMSUDSServer::generate_seed(uint8_t level, uint8_t *seed_out,
                                  uint8_t seed_len) const {
    const uint32_t *uid = (const uint32_t *)0x1FFF7A10UL;
    uint32_t raw = uid[0] ^ uid[1] ^ (uint32_t)level;
    for (uint8_t i = 0; i < seed_len && i < 4; i++)
        seed_out[i] = (raw >> (i * 8)) & 0xFF;
}

uint32_t BMSUDSServer::compute_key(uint8_t level, const uint8_t *seed,
                                    uint16_t seed_len) const {
    (void)seed_len;
    uint32_t s = ((uint32_t)seed[0] << 24) | ((uint32_t)seed[1] << 16) |
                 ((uint32_t)seed[2] <<  8) |  (uint32_t)seed[3];
    if (level == 0x01) return ~s;
    if (level == 0x11) return s ^ 0xA5A5A5A5UL;
    return ~s;
}
