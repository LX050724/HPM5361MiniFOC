#pragma once
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
    float speed_limit;
} foc_pll_t;

#ifdef __cplusplus
}
#endif