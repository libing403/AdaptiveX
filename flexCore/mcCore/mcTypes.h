/**
 * @file mcTypes.h
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2025-02-18
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#ifndef __MCTYPE_H__
#define __MCTYPE_H__

#ifdef __cplusplus
extern "C"{
#endif

#include "platformCfg.h"
#include "axisControl.h"
#include "groupControl.h"
#include "errorDefine.h"
//版本号
#define MAINVERSION  3
#define SUBVERSION0  0
#define SUBVERSION1  5
#define STR(s)       #s  
#define VERSION(a,b,c)   "motionlib-" STR(a) "." STR(b) "." STR(c)
#ifndef  float_def 
	#define float_def double
#endif

//系统常量
#define deg2rad			0.017453292519943
#define rad2deg			57.295779513082323

#define logFile if(LogFile)fprintf
/*==================== 公共全局数据定义 ====================*/
extern  int LogFile;
extern  GantryControl_t _gantry_Ctrl[MAX_GANTRY_COUNT]; //龙门同步控制器

#if defined(WINDOWS_ENV)
	extern  AxisControl_t _adx_AX [MAX_AXIS_COUNT];
	extern  AxisGroupControl_t _adx_GRP [MAX_GROUP_COUNT];
#elif defined (BARE_METAL_ENV)
extern  AxisControl_t *_adx_AX;
extern  AxisGroupControl_t *_adx_GRP;
#endif // BARE_METAL_ENV

#ifdef __cplusplus
}
#endif

#endif