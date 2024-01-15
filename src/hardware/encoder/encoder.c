#include "encoder.h"
#include "foc/fast_sin.h"
#include "project_config.h"
#include <stdint.h>
#include <stdlib.h>

typedef struct
{
    int16_t pole_pairs;
    uint16_t ang_offset;
    uint16_t a_max;
    uint16_t a_min;
    uint16_t b_max;
    uint16_t b_min;
} encoder_t;

static encoder_t encoder;

#if GET_ENCODER_INTERFACE(ENCODER_TYPE) == ENCODER_LINEAR_HALL
float encoder_get_eleAngle()
{
    return 0.0f;
}

void encoder_get_eleAngle_sincos(foc_sin_cos_t *sincos)
{
}
#else
float encoder_get_eleAngle()
{
    uint16_t raw_ang = encoder_get_rawAngle();
    float ele_ang = ((int32_t)(raw_ang - encoder.ang_offset) * encoder.pole_pairs) * (2.0f * F_PI / 65536.0f);
    return ele_ang;
}

void encoder_get_eleAngle_sincos(foc_sin_cos_t *sincos)
{
    foc_sin_cos(encoder_get_eleAngle(), sincos);
}

void encoder_set_param(int16_t pole_pairs, uint16_t ang_offset)
{
    encoder.pole_pairs = pole_pairs;
    encoder.ang_offset = ang_offset;
}

int16_t encoder_get_pole_pairs()
{
    return encoder.pole_pairs;
}

#endif