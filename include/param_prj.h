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
    PARAM_ENTRY("BCC", bcc0DeviceCount, "", 0, 15, 0, 10) \
    PARAM_ENTRY("BCC", bcc0DeviceType, "0=MC33771, 1=MC33772", 0, 1, 1, 14) \
    PARAM_ENTRY("BCC", bcc1DeviceCount, "", 0, 15, 0, 12) \
    PARAM_ENTRY("BCC", bcc1DeviceType, "0=MC33771, 1=MC33772", 0, 1, 1, 15) \
    \
    /* Battery Configuration */ \
    PARAM_ENTRY("Battery", batteryCapacity, "Ah", 10.0, 200.0, 50.0, 5) \
    PARAM_ENTRY("Battery", minSocPercent, "%", 0.0, 50.0, 10.0, 6) \
    PARAM_ENTRY("Battery", maxSocPercent, "%", 50.0, 100.0, 100.0, 7) \
    PARAM_ENTRY("Battery", initSocPercent, "%", 0.0, 100.0, 50.0, 8) \
    \
    /* Charging Configuration */ \
    PARAM_ENTRY("Charging", balanceTimerMin, "min", 1, 60, 5, 23) \
    PARAM_ENTRY("Charging", measureInterval, "ms", 10, 1000, 20, 24) \
    PARAM_ENTRY("Charging", balanceHvOffMin, "min", 0, 120, 5, 25) \
    \
    /* IVT-S Configuration */ \
    PARAM_ENTRY("IVT", ivtConfigured, "0=unconfigured, 1=configured", 0, 1, 0, 60) \
    \
    /* HV System Configuration */ \
    PARAM_ENTRY("HV", prechargeCheckInt, "ms", 10, 1000, 100, 32) \
    \
    /* Contactor PWM Configuration */ \
    PARAM_ENTRY("Contactor", pwmFrequency, "Hz", 1000, 50000, 25000, 40) \
    PARAM_ENTRY("Contactor", engageDuty, "%", 50, 100, 100, 41) \
    PARAM_ENTRY("Contactor", holdDuty0, "%", 10, 80, 30, 42) \
    PARAM_ENTRY("Contactor", engageTime, "ms", 10, 500, 100, 43) \
    PARAM_ENTRY("Contactor", holdDuty1, "%", 10, 80, 30, 45) \
    \
    /* Timing */ \
    PARAM_ENTRY("Timing", commTimeout, "ms", 1000, 30000, 5000, 51) \
    PARAM_ENTRY("Timing", faultCheckInt, "ms", 100, 30000, 5000, 52) \
    \
    /* UDS writable protection thresholds (DID 0xD110-0xD119) */ \
    PARAM_ENTRY("Protect", ovpThresholdMv, "mV", 2000, 4500, 4200, 70) \
    PARAM_ENTRY("Protect", ovpWarningMv, "mV", 2000, 4500, 4150, 71) \
    PARAM_ENTRY("Protect", uvpThresholdMv, "mV", 1000, 4000, 2500, 72) \
    PARAM_ENTRY("Protect", uvpWarningMv, "mV", 1000, 4000, 2600, 73) \
    PARAM_ENTRY("Protect", otpThresholdCdeg, "0.01°C", -5000, 8500, 4500, 74) \
    PARAM_ENTRY("Protect", otpWarningCdeg, "0.01°C", -5000, 8500, 4000, 75) \
    PARAM_ENTRY("Protect", utpThresholdCdeg, "0.01°C", -5000, 8500, -1000, 76) \
    PARAM_ENTRY("Protect", utpWarningCdeg, "0.01°C", -5000, 8500, -500, 77) \
    PARAM_ENTRY("Protect", ocpChargeMa, "mA", 0, 500000, 30000, 78) \
    PARAM_ENTRY("Protect", ocpDischargeMa, "mA", 0, 500000, 30000, 79) \
    \
    /* UDS writable balance config (DID 0xD100-0xD106) */ \
    PARAM_ENTRY("Balance", balanceMode, "0=off,1=delta,2=abs", 0, 2, 1, 80) \
    PARAM_ENTRY("Balance", balanceDeltaMv, "mV", 0, 1000, 50, 81) \
    PARAM_ENTRY("Balance", balanceAbsMv, "mV", 0, 5000, 3600, 82) \
    PARAM_ENTRY("Balance", balanceInhibitPackMv, "mV", 0, 500000, 0, 83) \
    PARAM_ENTRY("Balance", balanceMinCellMv, "mV", 0, 5000, 2500, 84) \
    PARAM_ENTRY("Balance", sohX100, "", 0, 10000, 10000, 85) \
    PARAM_ENTRY("Balance", socMethod, "0=coulomb,1=voltage", 0, 1, 0, 86) \
    \
    /* Auxiliary contactor configuration (DID 0xD200-0xD207) */ \
    PARAM_ENTRY("Contactor", auxContactorMode, "0=precharge+main,1=ac_dc", 0, 1, 0, 90) \
    PARAM_ENTRY("Contactor", auxPin0Role, "mode0:0=precharge,1=main mode1:0=ac,1=dc", 0, 1, 0, 91) \
    PARAM_ENTRY("Contactor", auxPin1Role, "mode0:0=precharge,1=main mode1:0=ac,1=dc", 0, 1, 1, 92) \
    PARAM_ENTRY("Contactor", nacsPin, "Arduino pin, 255=disabled", 0, 255, 255, 93) \
    PARAM_ENTRY("Contactor", prechargeCompletionMv, "mV", 0, 50000, 5000, 94) \
    PARAM_ENTRY("Contactor", prechargeTimeoutMs, "ms", 0, 60000, 10000, 95) \
    PARAM_ENTRY("Contactor", prechargeMinVoltageMv, "mV", 0, 500000, 0, 96) \
    PARAM_ENTRY("Contactor", nacsDcLevel, "logic level = DC mode", 0, 1, 1, 97) \
    \
    /* Read-only spot values */ \
    VALUE_ENTRY(version, "", 1000) \
    VALUE_ENTRY(hwver, "0=HV ECU V1", 1026) \
    VALUE_ENTRY(opmode, "0=Initialization, 1=Idle, 2=Charging, 3=Balancing, 4=Cooldown, 5=Sleep, 6=Error", 1027) \
    VALUE_ENTRY(lasterr, "0=None, 1=CellOvervoltage, 2=CellUndervoltage, 3=CellOvertemp, 4=CellBalanceOpen, 5=CellBalanceShort, 6=BCC0CommFault, 7=BCC1CommFault, 8=IVTCommLoss, 9=IVTOvervoltage, 10=IVTUndervoltage, 11=IVTOvertemp, 12=PrechargeTimeout, 13=PrechargeFailed, 14=ContactorFault, 15=PackOvervoltage, 16=PackUndervoltage", 1028) \
    VALUE_ENTRY(status, "0=Disabled, 1=Precharge, 2=Active, 3=Fault, 4=Shutdown", 1029) \
    VALUE_ENTRY(serial, "", 1030) \
    VALUE_ENTRY(faultModule, "", 1031) \
    VALUE_ENTRY(faultCell, "", 1032) \
    VALUE_ENTRY(cellVoltMin, "mV", 2) \
    VALUE_ENTRY(cellVoltMax, "mV", 3) \
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
