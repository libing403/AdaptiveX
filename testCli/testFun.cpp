#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "flexCore_if.h"
#include "testFun.h"
#include <windows.h>


/* ================== 工具函数 ================== */

int show_help(void)
{
    printf("----------------------------------------------------------------\n");
    printf("Available commands:\n");
    for (int i = 0; i < maxCmdCount; i++)
    {
        printf("  %-32s - %32s\n",
               cmd_table[i].cmd,
               cmd_table[i].help);
    }
    printf("----------------------------------------------------------------\n");
    return 0;
}
/* ================== 测试函数示例 ================== */
int test_adx_ConnetModbus(void)
{

    // int ret=adx_ConnetModbus("192.168.3.10", 4002);
    // if(ret!=0)
    // {
    //     printf("connect modbus error ret=%d\n",ret);
    //     return 1;
    // }
    printf("connect modbus success\n");
    return 0;
}

int test_adx_CoreInit()
{
    int ret=adx_CoreInit();
    if(ret!=0)
    {
        printf("adx_CoreInit error ret=%d\n",ret);
        return 1;
    }
    printf("adx_CoreInit success\n");
    return 0;
}

int test_adx_CoreClose()
{
    int ret=adx_CoreClose();
    if(ret!=0)
    {
        printf("adx_CoreClose error ret=%d\n",ret);
        return 1;
    }
    printf("adx_CoreClose success\n");
    return 0;
}

int test_adx_Power()
{
    int axisId=1;
    int enable=1;
    int startMode=0;
    int stopMode=0;
    for(int i=0;i<4;i++)
    {
        axisId=i;
        int ret=adx_Power(axisId,enable,startMode,stopMode);
        if(ret!=0)
        {
            printf("adx_Power error ret=%d\n",ret);
            return 1;
        }
         Sleep(10);
    }
    printf("adx_Power success\n");


    return 0;
}


int test_adx_ReadMultiParameter()
{
    int axisId=1;
    int index=0;
    axisParam_t param;
    int ret=adx_ReadMultiParameters(axisId,index,&param);
    if(ret!=0)
    {
        printf("adx_ReadMultiParameters error ret=%d\n",ret);
        return 1;
    }
    printf("adx_ReadMultiParameters success\n");
    return 0;
}

int test_adx_WriteMultiParameter()
{
    int axisId=1;
    int index=2;
    int bufferMode=1;
    axisParam_t param;
    param.ppr=1000;
    param.dirPol=1;
    param.speed=500;
    param.acc=1000;
    param.dpr=0.01;
    param.org=1;
    param.mdis=5.0;
    param.pdis=5.0;
    param.resetDir=2;
    param.resetSpeed=200;

    int ret=adx_WriteMultiParameters(axisId,index,&param,bufferMode);
    if(ret!=0)
    {
        printf("adx_WriteMultiParameters error ret=%d\n",ret);
        return 1;
    }
    printf("adx_WriteMultiParameters success\n");
    return 0;
}


int test_adx_MoveAbsolute()
{
    int axisId=1;
    int index=0;
    double position=100.0;
    double velocity=200.0;
    double acceleration=500.0;
    double deceleration=500.0;
    double  jerk=1000.0;
    int direction=3;
    int bufferMode=0;
    for(int i=0;i<4;i++)
    {
        axisId=i;
        position=i*100;
        int ret=adx_MoveAbsolute(axisId,index,position,velocity,acceleration,deceleration,jerk,direction,bufferMode);
        if(ret!=0)
        {
            printf("adx_MoveAbsolute error ret=%d\n",ret);
            return 1;
        }
        Sleep(100);
    }
 
    printf("adx_MoveAbsolute success\n");

    return 0;
}

int test_adx_ReadTargetPos()
{
    int axisId=1;
    int index=0;
    double position=0.0;
    for(int i=0;i<4;i++)
    {
        axisId=i;
        int ret=adx_ReadTargetPos(axisId,index,&position);
        if(ret!=0)
        {
            printf("adx_ReadTargetPos error ret=%d\n",ret);
            return 1;
        }
        
        printf("adx_ReadTargetPos success position=%.3f\n",position);
        Sleep(10);
    }
    return 0;
}

int test_adx_ReadTargetVel()
{
    int axisId=1;
    int index=0;
    double velocity=0.0;
    int ret=adx_ReadTargetVel(axisId,index,&velocity);
    if(ret!=0)
    {
        printf("adx_ReadTargetVel error ret=%d\n",ret);
        return 1;
    }
    printf("adx_ReadTargetVel success velocity=%.3f\n",velocity);
    return 0;
}

int test_adx_ReadLogicalPos()
{
    int axisId=1;
    int index=0;
    double position=0.0;
    int ret=adx_ReadLogicalPos(axisId,index,&position);
    if(ret!=0)
    {
        printf("adx_ReadLogicalPos error ret=%d\n",ret);
        return 1;
    }
    printf("adx_ReadLogicalPos success position=%.3f\n",position);
    return 0;
}

int test_adx_ReadLogicalVel()
{
    int axisId=1;
    int index=0;
    double velocity=0.0;
    int ret=adx_ReadLogicalVel(axisId,index,&velocity);
    if(ret!=0)
    {
        printf("adx_ReadLogicalVel error ret=%d\n",ret);
        return 1;
    }
    printf("adx_ReadLogicalVel success velocity=%.3f\n",velocity);
    return 0;
}

int test_adx_ReadActualPos()
{
    int axisId=1;
    int index=0;
    double position=0.0;
    int ret=adx_ReadActualPos(axisId,index,&position);
    if(ret!=0)
    {
        printf("adx_ReadActualPos error ret=%d\n",ret);
        return 1;
    }
    printf("adx_ReadActualPos success position=%.3f\n",position);
    return 0;
}

 

int test_adx_SetMotorScale()
{
    int axisId=1;
    double den=1.0;
    int num=1000;
	for (int i = 0; i < 4; i++)
	{
		axisId = i;
		int ret = adx_SetMotorEncoderScale(axisId, 0, num, den);
		if (ret != 0)
		{
			printf("adx_SetMotorScale error ret=%d\n", ret);
			return 1;
		}
		Sleep(10);
	}
    printf("adx_SetMotorScale success\n");
    return 0;
}

int test_adx_SetGantryEnable()
{
    int gantryId=0;
    int enable=1;
    int ret=adx_SetGantryEnable(gantryId,enable);
    if(ret!=0)
    {
        printf("adx_SetGantryEnable error ret=%d\n",ret);
        return 1;
    }
    printf("adx_SetGantryEnable success\n");
    return 0;
}

int test_adx_SetGantryConfig()
{
    int gantryId=0;
    GantryConfig_t config;
    config.masterId=0;
    config.slaveId=1;
    config.masterOffset_mm=0.0;
    config.slaveOffset_mm=0.0;
    int ret=adx_SetGantryConfig(gantryId,&config);
    if(ret!=0)
    {
        printf("adx_SetGantryConfig error ret=%d\n",ret);
        return 1;
    }
    printf("adx_SetGantryConfig success\n");
    return 0;
}

int test_adx_GantryMove()
{
    test_adx_CoreInit();
    test_adx_Power();
    test_adx_SetMotorScale();
    test_adx_SetGantryConfig();
    test_adx_SetGantryEnable();

    int axisId=0;
    int index=0;
    double position=100.0;
    double velocity=200.0;
    double acceleration=500.0;
    double deceleration=500.0;
    double  jerk=1000.0;
    int direction=3;
    int bufferMode=0;
    int ret=adx_MoveAbsolute(axisId,index,position,velocity,acceleration,deceleration,jerk,direction,bufferMode);
    if(ret!=0)
    {
        printf("adx_MoveAbsolute error ret=%d\n",ret);
        return 1;
    }
    axisId=2;
    position=200.0;
     ret=adx_MoveAbsolute(axisId,index,position,velocity,acceleration,deceleration,jerk,direction,bufferMode);
    if(ret!=0)
    {
        printf("adx_MoveAbsolute error ret=%d\n",ret);
        return 1;
    }
    axisId=3;
    position=300.0;
     ret=adx_MoveAbsolute(axisId,index,position,velocity,acceleration,deceleration,jerk,direction,bufferMode);
    if(ret!=0)
    {
        printf("adx_MoveAbsolute error ret=%d\n",ret);
        return 1;
    }
    return 0;
}
