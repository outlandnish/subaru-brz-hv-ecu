/**
 * @file errormessage_prj.h
 * @brief Project-specific error message definitions for libopeninv
 *
 * Phase 1: Minimal stub to get building
 * Phase 4: Full error handling for BMS faults
 */

#ifndef ERRORMESSAGE_PRJ_H
#define ERRORMESSAGE_PRJ_H

/**
 * @brief Error message IDs for the HV-ECU BMS
 *
 * Phase 1: Just a placeholder enum
 * Later phases will add:
 * - Cell overvoltage/undervoltage
 * - Overcurrent
 * - Overtemperature
 * - Contactor faults
 * - Communication errors
 */

/**
 * @brief Error message list macro (empty for Phase 1)
 *
 * Later phases will use: ERROR_MESSAGE_ENTRY(CELL_OVERVOLT, "Cell Overvoltage")
 */
#define ERROR_MESSAGE_LIST

#endif // ERRORMESSAGE_PRJ_H
