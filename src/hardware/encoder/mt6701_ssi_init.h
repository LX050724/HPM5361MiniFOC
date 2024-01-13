#pragma once
#include "hpm_sei_drv.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

int mt6701_ssi_init(uint32_t baud, sei_trigger_input_config_t *trigger_config, void (*callback)(uint32_t));

#ifdef __cplusplus
}
#endif