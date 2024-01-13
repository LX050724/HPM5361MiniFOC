#pragma once
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

int pwm_init(uint32_t pwm_freq, uint32_t ch8_cmp, uint32_t dead_time_ns);
void disable_all_pwm_output(void);
void enable_all_pwm_output(void);

#ifdef __cplusplus
}
#endif