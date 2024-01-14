#pragma once
#include "hpm_sei_drv.h"
#include <stdint.h>
#include "encoder.h"

#ifdef __cplusplus
extern "C" {
#endif

int mt6701_ssi_init(uint32_t baud, sei_trigger_input_config_t *trigger_config);
void mt6701_set_callback(ecnoder_callback_t cb);

#ifdef __cplusplus
}
#endif