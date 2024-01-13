#include "foc/foc_pll.h"
#include "foc/foc_core.h"


void foc_pll_init(foc_pll_t *pll)
{
    pll->speed = 0;
    pll->_epsilon = 0;
    pll->theta = 0;
}

void foc_pll(foc_pll_t *pll, const foc_sin_cos_t *input)
{
    foc_sin_cos_t theta_sin_cos;
    foc_sin_cos(pll->theta, &theta_sin_cos);
    pll->_epsilon = input->sinx * theta_sin_cos.cosx - input->cosx * theta_sin_cos.sinx;
    // pi
    // pll->speed = foc_pid_limit(piout);
    pll->theta += pll->speed;
}