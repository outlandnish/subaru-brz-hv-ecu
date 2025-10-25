#include "taycan.h"

const uint16_t TAYCAN_CONFIG[REG_CONF_CNT_MC33772] = {
    0x9001, // SYS_CFG1
    0x0330, // SYS_CFG2
    0x0000, // SYS_DIAG
    0x0717, // ADC_CFG
    0x4000, // ADC2_OFFSET_COMP
    0x003F, // OV_UV_EN
    0x0000, // CB1_CFG
    0x0000, // CB2_CFG
    0x0000, // CB3_CFG
    0x0000, // CB4_CFG
    0x0000, // CB5_CFG
    0x0000, // CB6_CFG
    0x0000, // GPIO_CFG1
    0x0000, // GPIO_CFG2
    0x0000, // FAULT_MASK1
    0x0000, // FAULT_MASK2
    0x0000, // FAULT_MASK3
    0x0000, // WAKEUP_MASK1
    0x0000, // WAKEUP_MASK2
    0x0000, // WAKEUP_MASK3
    0xD780, // TH_ALL_CT
    0xD780, // TH_CT6
    0xD780, // TH_CT5
    0xD780, // TH_CT4
    0xD780, // TH_CT3
    0xD780, // TH_CT2
    0xD780, // TH_CT1
    0x00ED, // TH_AN6_OT
    0x00ED, // TH_AN5_OT
    0x00ED, // TH_AN4_OT
    0x00ED, // TH_AN3_OT
    0x00ED, // TH_AN2_OT
    0x00ED, // TH_AN1_OT
    0x00ED, // TH_AN0_OT
    0x030E, // TH_AN6_UT
    0x030E, // TH_AN5_UT
    0x030E, // TH_AN4_UT
    0x030E, // TH_AN3_UT
    0x030E, // TH_AN2_UT
    0x030E, // TH_AN1_UT
    0x030E, // TH_AN0_UT
    0x0000, // TH_ISENSE OC
    0x0000, // TH_COULOMB_CNT_MSB
    0x0000  // TH_COULOMB_CNT_LSB
};

/*
Config dumped from battery:
Reading all configuration from battery...

  -------------------------------
  | Register           | Value  |
  -------------------------------
  | INIT               | 0x0001 |
  | SYS_CFG1           | 0x9001 |
  | SYS_CFG2           | 0x0330 |
  | SYS_DIAG           | 0x0000 |
  | ADC_CFG            | 0x0717 |
  | ADC2_OFFSET_COMP   | 0x4000 |
  | OV_UV_EN           | 0x003F |
  | CB1_CFG            | 0x0000 |
  | CB2_CFG            | 0x0000 |
  | CB3_CFG            | 0x0000 |
  | CB4_CFG            | 0x0000 |
  | CB5_CFG            | 0x0000 |
  | CB6_CFG            | 0x0000 |
  | GPIO_CFG1          | 0x0000 |
  | GPIO_CFG2          | 0x0000 |
  | FAULT_MASK1        | 0x0000 |
  | FAULT_MASK2        | 0x0000 |
  | FAULT_MASK3        | 0x0000 |
  | WAKEUP_MASK1       | 0x0000 |
  | WAKEUP_MASK2       | 0x0000 |
  | WAKEUP_MASK3       | 0x0000 |
  | TH_ALL_CT          | 0xD780 |
  | TH_CT6             | 0xD780 |
  | TH_CT5             | 0xD780 |
  | TH_CT4             | 0xD780 |
  | TH_CT3             | 0xD780 |
  | TH_CT2             | 0xD780 |
  | TH_CT1             | 0xD780 |
  | TH_AN6_OT          | 0x00ED |
  | TH_AN5_OT          | 0x00ED |
  | TH_AN4_OT          | 0x00ED |
  | TH_AN3_OT          | 0x00ED |
  | TH_AN2_OT          | 0x00ED |
  | TH_AN1_OT          | 0x00ED |
  | TH_AN0_OT          | 0x00ED |
  | TH_AN6_UT          | 0x030E |
  | TH_AN5_UT          | 0x030E |
  | TH_AN4_UT          | 0x030E |
  | TH_AN3_UT          | 0x030E |
  | TH_AN2_UT          | 0x030E |
  | TH_AN1_UT          | 0x030E |
  | TH_AN0_UT          | 0x030E |
  | TH_ISENSE OC       | 0x0000 |
  | TH_COULOMB_CNT_MSB | 0x0000 |
  | TH_COULOMB_CNT_LSB | 0x0000 |
  -------------------------------
*/