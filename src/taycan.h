#include "bcc/bcc_config.h"
#include "BatteryCellController.h"

extern const uint16_t TAYCAN_CONFIG[REG_CONF_CNT_MC33772];

// Read cell voltage limits from BCC hardware configuration
void read_pack_voltage_limits(BatteryCellController *bcc, uint16_t *min_mv, uint16_t *max_mv);