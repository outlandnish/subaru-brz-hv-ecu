/**
 * @file param_prj.h
 * @brief Project-specific parameter definitions for libopeninv
 */

#ifndef PARAM_PRJ_H
#define PARAM_PRJ_H

/**
 * Parameter definition macros:
 * PARAM_ENTRY(category, name, unit, min, max, default, id)
 * VALUE_ENTRY(name, unit, id) - Read-only values
 *
 * ID should be unique across all parameters
 */

#define PARAM_LIST \
    /* CAN Configuration */ \
    PARAM_ENTRY("CAN", canNodeId, "", 1, 127, 22, 1) \
    \
    /* BCC Hardware Configuration */ \
    PARAM_ENTRY("BCC", bcc0DeviceCount, "", 1, 15, 8, 10) \
    PARAM_ENTRY("BCC", bcc0CellCount, "", 1, 14, 6, 11) \
    PARAM_ENTRY("BCC", bcc1DeviceCount, "", 1, 15, 8, 12) \
    PARAM_ENTRY("BCC", bcc1CellCount, "", 1, 14, 6, 13) \
    \
    /* Battery Protection Limits */ \
    PARAM_ENTRY("Battery", cellVoltMin, "mV", 2500, 3500, 3000, 2) \
    PARAM_ENTRY("Battery", cellVoltMax, "mV", 3500, 4300, 4200, 3) \
    PARAM_ENTRY("Battery", targetCellVolt, "mV", 3000, 4300, 3600, 4) \
    PARAM_ENTRY("Battery", batteryCapacity, "Ah", 10.0, 200.0, 50.0, 5) \
    PARAM_ENTRY("Battery", minSocPercent, "%", 0.0, 50.0, 10.0, 6) \
    PARAM_ENTRY("Battery", maxSocPercent, "%", 50.0, 100.0, 100.0, 7) \
    PARAM_ENTRY("Battery", initSocPercent, "%", 0.0, 100.0, 50.0, 8) \
    PARAM_ENTRY("Battery", socMinVoltage, "mV", 2500, 3500, 3000, 9) \
    \
    /* Charging Configuration */ \
    PARAM_ENTRY("Charging", maxChargeCurrent, "A", 5.0, 200.0, 30.0, 20) \
    PARAM_ENTRY("Charging", balanceThreshold, "mV", 10.0, 200.0, 50.0, 21) \
    PARAM_ENTRY("Charging", balanceTarget, "mV", 5.0, 100.0, 10.0, 22) \
    PARAM_ENTRY("Charging", balanceTimerMin, "min", 1, 60, 5, 23) \
    PARAM_ENTRY("Charging", measureInterval, "ms", 10, 1000, 20, 24) \
    \
    /* HV System Configuration */ \
    PARAM_ENTRY("HV", prechargeMargin, "V", 1.0, 50.0, 10.0, 30) \
    PARAM_ENTRY("HV", prechargeTimeout, "ms", 1000, 30000, 5000, 31) \
    PARAM_ENTRY("HV", prechargeCheckInt, "ms", 10, 1000, 100, 32) \
    \
    /* Contactor PWM Configuration */ \
    PARAM_ENTRY("Contactor", pwmFrequency, "Hz", 1000, 50000, 25000, 40) \
    PARAM_ENTRY("Contactor", engageDuty, "%", 50, 100, 100, 41) \
    PARAM_ENTRY("Contactor", holdDuty, "%", 10, 80, 30, 42) \
    PARAM_ENTRY("Contactor", engageTime, "ms", 10, 500, 100, 43) \
    \
    /* Filtering and Timing */ \
    PARAM_ENTRY("Filter", voltageFilterAlpha, "", 0.01, 1.0, 0.2, 50) \
    PARAM_ENTRY("Filter", commTimeout, "ms", 1000, 30000, 5000, 51) \
    PARAM_ENTRY("Filter", faultCheckInt, "ms", 100, 30000, 5000, 52) \
    \
    /* Read-only spot values */ \
    VALUE_ENTRY(version, "", 1000) \
    VALUE_ENTRY(serialNumber, "", 999) \
    VALUE_ENTRY(packVoltage, "V", 1001) \
    VALUE_ENTRY(packVoltFilt, "V", 1002) \
    VALUE_ENTRY(packCurrent, "A", 1003) \
    VALUE_ENTRY(soc, "%", 1004) \
    VALUE_ENTRY(socPrecise, "%", 1005) \
    VALUE_ENTRY(bmsState, "", 1006) \
    VALUE_ENTRY(hvState, "", 1007) \
    VALUE_ENTRY(maxCellVolt, "mV", 1008) \
    VALUE_ENTRY(minCellVolt, "mV", 1009) \
    VALUE_ENTRY(cellVoltDiff, "mV", 1010) \
    VALUE_ENTRY(safeChargeCurrent, "A", 1011) \
    VALUE_ENTRY(bcc0Initialized, "", 1012) \
    VALUE_ENTRY(bcc1Initialized, "", 1013) \
    VALUE_ENTRY(faultStatus, "", 1014) \
    VALUE_ENTRY(commLost, "", 1015) \
    VALUE_ENTRY(contactorFault, "", 1016) \
    /* IVT Shunt values (received directly from CAN) */ \
    VALUE_ENTRY(ivtCurrent, "A", 1020) \
    VALUE_ENTRY(ivtVoltage1, "V", 1021) \
    VALUE_ENTRY(ivtVoltage2, "V", 1022) \
    VALUE_ENTRY(ivtVoltage3, "V", 1023) \
    VALUE_ENTRY(ivtTemperature, "C", 1024) \
    VALUE_ENTRY(ivtPower, "kW", 1025) \

#endif // PARAM_PRJ_H
