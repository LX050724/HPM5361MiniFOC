#pragma once

#include <stdint.h>
#include "foc/foc_core.h"

#ifdef __cplusplus
extern "C" {
#endif

void current_init();
void current_get_cal(uint16_t raw_v, uint16_t raw_w, foc_uvw_current_t *cur);
void current_set_calibration(int32_t v_value, int32_t w_value);

#ifdef __cplusplus
}
#endif