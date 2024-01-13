#include "board.h"
#include "project_config.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#if ENCODER_TYPE == ENCODER_MT6701
#include "mt6701_ssi_init.h"

static inline uint16_t encoder_getAngle()
{
    return sei_get_data_value(BOARD_SEI, SEI_DAT_2);
}

static inline uint16_t encoder_getStatus()
{
    return sei_get_data_value(BOARD_SEI, SEI_DAT_3);
}

static inline uint16_t encoder_getCrc()
{
    return sei_get_data_value(BOARD_SEI, SEI_DAT_4);
}

#endif

#ifdef __cplusplus
}
#endif