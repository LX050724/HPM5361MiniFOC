#pragma once

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    float kp;
    float ki;
    float kd;
    float err;
    float integral;
} foc_pid_contrl_t;


static inline float foc_pid_limit(float in, float limit)
{
    if (in > limit)
        return limit;
    if (in < -limit)
        return -limit;
    return in;
}

static inline float foc_pid_limit2(float in, float max, float min)
{
    if (in > max)
        return max;
    if (in < min)
        return min;
    return in;
}

#ifdef __cplusplus
}
#endif