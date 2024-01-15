#include "board.h"
#include "foc/foc_core.h"
#include "project_config.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef void (*ecnoder_callback_t)(uint32_t flag);

float encoder_get_eleAngle();
void encoder_get_eleAngle_sincos(foc_sin_cos_t *sincos);
void encoder_set_param(int16_t pole_pairs, uint16_t ang_offset);
int16_t encoder_get_pole_pairs();

#if ENCODER_TYPE == ENCODER_MT6701
#include "hpm_sei_drv.h"

static inline uint16_t encoder_get_rawAngle()
{
    return sei_get_data_value(BOARD_SEI, SEI_DAT_2);
}


static inline uint16_t encoder_get_status()
{
    return sei_get_data_value(BOARD_SEI, SEI_DAT_3);
}

static inline uint16_t encoder_get_crc()
{
    return sei_get_data_value(BOARD_SEI, SEI_DAT_4);
}

static inline void encoder_set_callback(ecnoder_callback_t cb)
{
    void mt6701_set_callback(ecnoder_callback_t cb);
    mt6701_set_callback(cb);
}

#endif

#ifdef __cplusplus
}
#endif