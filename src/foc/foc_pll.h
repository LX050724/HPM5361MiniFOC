#pragma once
#include "foc/foc_core.h"
#include "foc_pid.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    foc_pid_contrl_t pi;
    float _epsilon;
    float theta;
    float speed;
} foc_pll_t;

void foc_pll_init(foc_pll_t *pll);
void foc_pll(foc_pll_t *pll, const foc_sin_cos_t *input);

#ifdef __cplusplus
}
#endif