/*****************************************************************//**
 * \file   envconfig.h
 * \brief  环境配置，主要根据运行平台的不同进行不同的配置。
 *
 * \author Administrator
 * \date   March 2022
 *********************************************************************/
#ifndef ENVCONFIG_H_
#define ENVCONFIG_H_
#ifdef __cplusplus
extern "C" {
#endif

//#define __bare_metal__
#define __windows__

#ifdef __windows__
    #define WINDOWS_ENV
#elif defined (__linux__)
    #define LINUX_ENV
#elif defined( __arm__ )
    #define ARM_ENV

#elif defined( __bare_metal__ )
    #define BARE_METAL_ENV
#endif


#if defined(WINDOWS_ENV)
    #include <windows.h>
    #include <time.h>

#elif defined(ARM_ENV)
    #include "semphr.h"
    #define max(a, b)				((a) > (b) ? (a) : (b))
    #define INFINITE						portMAX_DELAY
    #define CreateMutex(a, b, c)	(xSemaphoreCreateMutex())
    //获取互斥量
    #define WaitForSingleObject		        xSemaphoreTake
    //给出互斥量
    #define ReleaseMutex					xSemaphoreGive
    //定义互斥量句柄
    #define HANDLE						   SemaphoreHandle_t
    //获取时间戳
    #define clock						   HAL_GetTick
    //时间戳定义
    typedef int clock_t;
    //延时
    #define Sleep(x)							vTaskDelay(x*1000)

#elif defined(BARE_METAL_ENV)
    //在裸机环境下定义相关宏
    #define HANDLE int
    #define max(a, b)			((a) > (b) ? (a) : (b))
    #define INFINITE	                 0
    #define CreateMutex(a,b,c)           1

    #define WaitForSingleObject(a,b)    (void)a      	
    #define ReleaseMutex(a)             (void)a
    #define NULL                         0    
    #define Sleep(x)                     (void)x

#else
    #error "Please define the platform environment!"

#endif

#ifdef __cplusplus
}
#endif

#endif // !ENVCONFIG_H_
