#pragma once

#include <stdint.h>
#include "fast_sin.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    float iu;
    float iv;
    float iw;
} foc_uvw_current_t;

typedef struct
{
    float i_alpha;
    float i_beat;
} foc_alpha_beta_current_t;

typedef struct
{
    float iq;
    float id;
} foc_qd_current_t;

typedef struct
{
    float v_alpha;
    float v_beta;
} foc_alpha_beta_volt_t;

typedef struct
{
    uint32_t pwm_u;
    uint32_t pwm_v;
    uint32_t pwm_w;
} foc_pwm_t;

typedef struct
{
    float sinx;
    float cosx;
} foc_sin_cos_t;

static inline void foc_sin_cos(float x, foc_sin_cos_t *ang)
{
    fast_sin_cos(x, &ang->sinx, &ang->cosx);
}

void foc_svpwm(const foc_alpha_beta_volt_t *volt, foc_pwm_t *pwm, int pwm_max);
void foc_park(const foc_alpha_beta_current_t *in, const foc_sin_cos_t *ang, foc_qd_current_t *out);
void foc_inv_park(const foc_qd_current_t *in, const foc_sin_cos_t *ang, foc_alpha_beta_volt_t *out);
void foc_clarke(const foc_uvw_current_t *in, foc_alpha_beta_current_t *out);

#ifdef __cplusplus
}
#endif
