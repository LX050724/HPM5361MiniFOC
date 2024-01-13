#pragma once

#include "hpm_mcan_drv.h"
#ifdef __cplusplus
extern "C" {
#endif

#define CAN_BAUDRATE_1M 1000000
#define CAN_BAUDRATE_500K 500000

void board_init_canfd(uint32_t baudrate, mcan_node_mode_t mode);

#ifdef __cplusplus
}
#endif