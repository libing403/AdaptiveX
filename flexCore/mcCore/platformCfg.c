
#include "axisControl.h"
#include "groupControl.h"
#include "platformCfg.h"
#include <stdint.h>

#if defined(WINDOWS_ENV)

    AxisControl_t _adx_AX [MAX_AXIS_COUNT];
    AxisGroupControl_t _adx_GRP [MAX_GROUP_COUNT];
    Queue _adx_ErrQ;
    int errorCode[MAX_ERROR_COUNT];

    int LogFile; 

#elif defined (BARE_METAL_ENV)
    #define BOARD_SDRAM_ADDRESS            (0x40000000)
    AxisControl_t *_adx_AX = (AxisControl_t *)BOARD_SDRAM_ADDRESS;
    AxisGroupControl_t* _adx_GRP = (AxisGroupControl_t* )(BOARD_SDRAM_ADDRESS + 0x400000);

#endif // BARE_METAL_ENV