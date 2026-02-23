#include<stddef.h>
#include<stdio.h>
//自定义头文件
 
#include "cmdNo.h"
#include "mcCore/axisControl.h"
#include"mcTypes.h"
#include"axisControl.h"
#include"groupControl.h"
#include"mcCore.h"
#include"Queue.h"
#include"errorDefine.h"
#include"flexCore_if.h"


//第三方库
#include"ulog.h"

/*==================== 辅助函数 ====================*/

// #define ANSI_COLOR_GREEN " \x1b[32m"
// #define ANSI_COLOR_YELLOW " \x1b[33m"
// #define ANSI_COLOR_RED " \x1b[31m"
// #define ANSI_COLOR_RESET " \x1b[0m"
#define ANSI_COLOR_GREEN "  "
#define ANSI_COLOR_YELLOW " "
#define ANSI_COLOR_RED "  "
#define ANSI_COLOR_RESET "  "


void my_console_logger(ulog_level_t severity, char *msg)
{
    //time_t now;
    //time(&now);
    //struct tm *local_time = localtime(&now);
    //char time_str[20];
    //strftime(time_str, sizeof(time_str), "%Y-%m-%d %H:%M:%S", local_time);
    char prefix[20]="adx";
    const char *color = ANSI_COLOR_GREEN;
    if (severity == ULOG_WARNING_LEVEL) {
        color = ANSI_COLOR_YELLOW;
    } else if (severity == ULOG_ERROR_LEVEL) {
        color = ANSI_COLOR_RED;
    }

    printf("%s[%s]:%s%s%s\n",
           prefix,
           ulog_level_name(severity),
           color,
           msg,
           ANSI_COLOR_RESET);
}

void setLogFile(int mode)
{
    LogFile = mode;
}




int openLogFile(void)
{
    AxisGroupControl_t *grp=_adx_GRP;
    char filename[64];
    printf("open data file\n");
    for(int i=0;i<MAX_GROUP_COUNT;i++)
    {
        //临时文件初始化
        sprintf(filename,"cmdFile%d.txt",i);
        grp[i].cmdFile=fopen(filename,"w");
        if(grp[i].cmdFile==NULL)
        {
            ULOG_ERROR("cmdFile%d.txt open error",i);
            return 1;
        }
        sprintf(filename,"group%d.txt",i);
        grp[i].grpFile=fopen(filename,"w");
        if(grp[i].grpFile==NULL)
        {
            ULOG_ERROR("group%d.txt open error",i);
            return 1;
        }
        sprintf(filename,"cart%d.txt",i);
        grp[i].cartFile=fopen(filename,"w");
        if(grp[i].cartFile==NULL)
        {
            ULOG_ERROR("cartFile%d.txt open error",i);
            return 1;
        }
        sprintf(filename,"actFile%d.txt",i);
        grp[i].actFile=fopen(filename,"w");
        if(grp[i].actFile==NULL)
        {
            ULOG_ERROR("actFile%d.txt open error",i);
            return 1;
        }
    }
    char fileName[64];
    AxisControl_t *axis=_adx_AX;
    for(int i=0;i<MAX_AXIS_COUNT;i++ )
    {
    
        sprintf(fileName, "drv%d.txt", i);
        axis[i].drvFile=fopen(fileName,"w");
        if(!axis[i].drvFile)
        {
            ULOG_ERROR("open file %s errror!",fileName);
            break;
        }
    }

    setLogFile(1);
    return 0;
}

int closeLogFile(void)
{
    AxisGroupControl_t *grp=_adx_GRP;
    printf("close data file\n");
    for(int i=0;i<MAX_GROUP_COUNT;i++)
    {
        
        fclose(grp[i].cmdFile);
        fclose(grp[i].grpFile);
        fclose(grp[i].cartFile);
        fclose(grp[i].actFile);
    }
    AxisControl_t *axis=_adx_AX;
    for(int i=0;i<MAX_AXIS_COUNT;i++)
    {
         
        fclose(axis[i].drvFile);
 
    }
    return 0;
}



/**
 * @brief 对日志，硬件接口，轴控制及轴组控制进行初始化
 * 
 * @return int 
 */
int adx_CoreInit(void)
{
 
    ULOG_INIT();
    ULOG_SUBSCRIBE(my_console_logger, ULOG_DEBUG_LEVEL);
    ULOG_DEBUG("This is a debug message!");
    ULOG_INFO("This is a normal message!!");
    ULOG_WARNING("This is a warning message!");
    ULOG_ERROR("This is an error message!");
    int dataSize=0;
    dataSize=sizeof(adx_Control_t);
    ULOG_INFO("controller data size = %d kb", dataSize/1024);
     
    //初始化轴控制器
    int ret=axisControlInit();
    if(ret)
    {
        ULOG_ERROR("axisControlInit error ret=%d",ret);
        return ret;
    }
    ret=GroupControlInit();
    if(ret)
    {
        ULOG_ERROR("GroupControlInit error ret=%d",ret);
        return ret;
    }

#if 0
    openLogFile();
   //创建主循环
    HANDLE handle_motionPlan;
    handle_motionPlan = CreateThread(NULL, 0, (LPTHREAD_START_ROUTINE)motionPlan, NULL, 0, NULL);
    ULOG_INFO("motion thread create ok!");
    // 设置线程优先级（相对于进程优先级）
    SetThreadPriority(handle_motionPlan, THREAD_PRIORITY_HIGHEST);
    
   //设置定时器
    DWORD_PTR dwUser = 0;
    MMRESULT timerID = timeSetEvent(1, 1, (LPTIMECALLBACK)realTimeControl, dwUser, TIME_PERIODIC);
    if (timerID != 0)
    ULOG_INFO("timer thread create ok!");
    // HANDLE handle_realTimeControl;
    // handle_realTimeControl = CreateThread(NULL, 0, (LPTHREAD_START_ROUTINE)realTimeControl, NULL, 0, NULL);
    // ULOG_INFO("real time control thread create ok!");
    // // 设置线程优先级（相对于进程优先级）
    // SetThreadPriority(handle_realTimeControl, THREAD_PRIORITY_HIGHEST);

#endif
    

   return 0;
}

int adx_CoreClose()
{
    
    ULOG_INFO("adx_Core close!");
    #if defined(WINDOWS_ENV)
    //关闭日志
    closeLogFile();
    #elif defined(ARM_ENV)
   
    #endif
   
    return 0;
}

/***************************************************************
 * 以下是轴控制相关接口定义
 *************************************************************/

 int adx_Power(int axisId,int enable,int startMode,int  stopMode)
{
    if(axisId<0 || axisId>MAX_AXIS_COUNT)
    {
        return SETPARAM_ERR;
    }
    int ret=0;
    FsmEvent_t event={.name="adx_Power",.type=cmd_adx_Power};
    event.type = cmd_adx_Power;
    struct {
        int axisId;
        int enable ;
        int startMode;
        int stopMode;
    }evdata={.axisId=axisId,.enable=enable,.startMode=startMode,.stopMode=stopMode};
    event.data = &evdata;
    ret=AxisControl_HandleEvent(&_adx_AX[axisId],&event);
    return ret;
}


int adx_ReadParameter(int axisId,int index,int paramId,double *value)
{
    if(axisId<0 || axisId>MAX_AXIS_COUNT)
    {
        return SETPARAM_ERR;
    }
    int ret=0;
    FsmEvent_t event={.name="adx_ReadParameter",.type=cmd_adx_WriteFloatParam};
    event.type = cmd_adx_WriteFloatParam;
    struct {
        int axisId;
        int index;
        int paramId;
        double *value;
    }evdata={.axisId=axisId,.paramId=paramId,.value=value};
    event.data = &evdata;
    ULOG_DEBUG("adx_WriteParameter: grpId=%d,index=%d,paramId=%d", axisId, index, paramId);
    // memcpy(event.msg,&evdata,sizeof(evdata));
    // ret=EnQueue(&cmdQueue,&event);
    ret=AxisControl_HandleEvent(&_adx_AX[axisId],&event);
    return ret;
}

int adx_WriteParameter(int axisId,int index,int paramId,double value,int bufferMode)
{
    if(axisId<0 || axisId>MAX_AXIS_COUNT)
    {
        return SETPARAM_ERR;
    }
    int ret=0;
    FsmEvent_t event={.name="adx_WriteParameter",.type=cmd_adx_WriteFloatParam};
    event.type = cmd_adx_WriteFloatParam;
    struct {
        int axisId;
        int index;
        int paramId;
        double value;
        int bufferMode;
    }evdata={.axisId=axisId,.index=index,.paramId=paramId,.value=value,.bufferMode=bufferMode};
    event.data = &evdata;
    ULOG_DEBUG("adx_WriteParameter: grpId=%d,index=%d,paramId=%d, value=%f", axisId, index, paramId, value);
    // memcpy(event.msg,&evdata,sizeof(evdata));
    // ret=EnQueue(&cmdQueue,&event);
    ret=AxisControl_HandleEvent(&_adx_AX[axisId],&event);
    return ret;
}

int adx_WriteBoolParameter(int axisId,int index,int paramId,bool value,int bufferMode)
{
    if(axisId<0 || axisId>MAX_AXIS_COUNT)
    {
        return SETPARAM_ERR;
    }
    int ret=0;
    FsmEvent_t event={.name="adx_WriteParameter",.type=cmd_adx_GroupWriteIntParam};
    event.type = cmd_adx_GroupWriteIntParam;
    struct {
        int axisId;
        int index;
        int paramId;
        bool value;
        int bufferMode;
    }evdata={.axisId=axisId,.index=index,.paramId=paramId,.value=value,.bufferMode=bufferMode};
    event.data = &evdata;
    ULOG_DEBUG("adx_WriteParameter: grpId=%d,index=%d,paramId=%d, value=%d", axisId, index, paramId, value);
    // memcpy(event.msg,&evdata,sizeof(evdata));
    // ret=EnQueue(&cmdQueue,&event);
    ret=AxisControl_HandleEvent(&_adx_AX[axisId],&event);
    return ret;
}

int adx_WriteMultiParameters(int axisId,int index,axisParam_t *param,int bufferMode)
{
    if(axisId<0 || axisId>MAX_AXIS_COUNT)
    {
        return SETPARAM_ERR;
    }
    int ret=0;
    FsmEvent_t event={.name="adx_WriteMultiParameters",.type=cmd_adx_WriteMultiParam            ,
};
    
    struct {
        int axisId;
        int index;
        axisParam_t *param;
        int bufferMode;
    }evdata={.axisId=axisId,.index=index,.param=param,.bufferMode=bufferMode};
    event.data = &evdata;
    ULOG_DEBUG("adx_WriteMultiParameters: axisId=%d,index=%d", axisId, index);
    ret=AxisControl_HandleEvent(&_adx_AX[axisId],&event);
    return ret;
}

int adx_MoveAbsolute(int axisId,int index,double position,double velocity,double acceleration,double deceleration,
    double jerk,int direction,int bufferMode)
{
    if(axisId<0 || axisId>MAX_AXIS_COUNT)
    {
        return SETPARAM_ERR;
    }
    FsmEvent_t event={.name="adx_MoveAbsolute",.type=cmd_adx_MoveAbsolute};
    struct 
    {
        int axisId;
        int index;
        double position;
        double velocity;
        double acceleration;
        double deceleration;
        double jerk;
        int direction;
        int bufferMode;
    }evdata={
        .axisId=axisId,
        .index=index,
        .position=position,
        .velocity=velocity,
        .acceleration=acceleration,
        .deceleration=deceleration,
        .jerk=jerk,
        .direction=direction,
        .bufferMode=bufferMode

    };
    event.data=&evdata;
    int ret=AxisControl_HandleEvent(&_adx_AX[axisId],&event);
    return ret;

}

int adx_MoveRelative(int axisId,int index,double position,double velocity,double acceleration,double deceleration,
    double jerk,int direction,int bufferMode)
{
    if(axisId<0 || axisId>MAX_AXIS_COUNT)
    {
        return SETPARAM_ERR;
    }
    FsmEvent_t event={.name="adx_MoveRelative",.type=cmd_adx_MoveRelative};
    struct 
    {
        int axisId;
        int index;
        double position;
        double velocity;
        double acceleration;
        double deceleration;
        double jerk;
        int direction;
        int bufferMode;
    }evdata={
        .axisId=axisId,
        .index=index,
        .position=position,
        .velocity=velocity,
        .acceleration=acceleration,
        .deceleration=deceleration,
        .jerk=jerk,
        .direction=direction,
        .bufferMode=bufferMode
    };
    event.data=&evdata;
    int ret=AxisControl_HandleEvent(&_adx_AX[axisId],&event);
    return ret;
}

int adx_SetOverride(int axisId,int index,double velFactor,double accFactor,double jerkFactor,int bufferMode)
{
    if(axisId<0 || axisId>MAX_AXIS_COUNT)
    {
        return SETPARAM_ERR;
    }
    FsmEvent_t event={.name="adx_SetOverride",.type=cmd_adx_SetOverride};
    struct 
    {
        int axisId;
        int index;
        double velFactor;
        double accFactor;
        double jerkFactor;
        int bufferMode;
    }evdata={
        .axisId=axisId,
        .index=index,
        .velFactor=velFactor,
        .accFactor=accFactor,
        .jerkFactor=jerkFactor,
        .bufferMode=bufferMode
    };
    event.data=&evdata;
    int ret=AxisControl_HandleEvent(&_adx_AX[axisId],&event);
    return ret;

}

int adx_MoveVelocity(int axisId,int index,double velocity,double acceleration,double deceleration,double jerk,int direction,int bufferMode)
{
    if(axisId<0 || axisId>MAX_AXIS_COUNT ||direction==0)
    {
        return SETPARAM_ERR;
    }
    FsmEvent_t event={.name="adx_MoveVelocity",.type=cmd_adx_MoveVelocity};
    struct 
    {
        int axisId;
        int index;
        double velocity;
        double acceleration;
        double deceleration;
        double jerk;
        int direction; //方向，0-正方向，2-负方向，3-当前方向
        int bufferMode;
    }evdata={
        .axisId=axisId,
        .index=index,
        .velocity=velocity,
        .acceleration=acceleration,
        .deceleration=deceleration,
        .jerk=jerk,
        .direction=direction,
        .bufferMode=bufferMode
    };
    event.data=&evdata;
    int ret=AxisControl_HandleEvent(&_adx_AX[axisId],&event);
    return ret;

}


int adx_Halt(int axisId,int index,double deceleration,double jerk,int bufferMode)
{
    if(axisId<0 || axisId>MAX_AXIS_COUNT)
    {
        return SETPARAM_ERR;
    }
    int ret=0;
    FsmEvent_t event={.name="adx_Halt",.type=cmd_adx_Stop};
    event.type = cmd_adx_Stop;
    struct {
        int axisId;
        int index;
        double deceleration;
        double jerk;
        int bufferMode;
    }evdata={.axisId=axisId,.index=index,.deceleration=deceleration,.jerk=jerk,.bufferMode=bufferMode};
    event.data = &evdata;
    ret=AxisControl_HandleEvent(&_adx_AX[axisId],&event);
    return ret;
}


int adx_Stop(int axisId,int index,double deceleration,double jerk,int bufferMode)
{
    if(axisId<0 || axisId>MAX_AXIS_COUNT)
    {
        return SETPARAM_ERR;
    }
    int ret=0;
    FsmEvent_t event={.name="adx_Stop",.type=cmd_adx_Stop};
    event.type = cmd_adx_Stop;
    struct {
        int axisId;
        int index;
        double deceleration;
        double jerk;
        int bufferMode;
    }evdata={.axisId=axisId,.index=index,.deceleration=deceleration,.jerk=jerk,.bufferMode=bufferMode};
    event.data = &evdata;
    // memcpy(event.msg,&evdata,sizeof(evdata));
    // ret=EnQueue(&cmdQueue,&event);
    ret=AxisControl_HandleEvent(&_adx_AX[axisId],&event);
    return ret;
}



/**
 * @brief 设置龙门架同步启用/禁用
 * 启用后，主轴运动时从轴跟随主轴运动，保持相对位置不变。
 * 禁用后，主轴运动时从轴不跟随运动。
 *
 * @param axisId 轴ID,主轴或从轴均可
 * @param enable 启用标志，1-启用，0-禁用。
 * @return 
 */
int adx_SetGantryEnable(int axisId,int enable)
{
    if(axisId<0 || axisId>MAX_AXIS_COUNT)
    {
        return SETPARAM_ERR;
    }
    int ret=0;
    FsmEvent_t event={.name="adx_SetGantryEnable",.type=cmd_adx_SetGantryEnable};
    event.type = cmd_adx_SetGantryEnable;
    struct {
        int axisId;
        int enable ;
    }evdata={.axisId=axisId,.enable=enable};
    event.data = &evdata;
    ret=AxisControl_HandleEvent(&_adx_AX[axisId],&event);
    return ret;
}

/**
 * @brief 设置龙门架同步配置
 * 
 * @param masterId 主轴ID
 * @param slaveId  从轴ID
 * @param config   同步配置参数
 * @return 
 */
int adx_SetGantryConfig(int gantryId, GantryConfig_t *config)
{
    if(gantryId<0 || gantryId>MAX_GANTRY_COUNT )
    {
        return SETPARAM_ERR;
    }
    int ret=0;
    FsmEvent_t event={.name="adx_SetGantryConfig",.type=cmd_adx_SetGantryConfig};
    event.type = cmd_adx_SetGantryConfig;
    struct {
        int gantryId;
        GantryConfig_t *config;
    }evdata={.gantryId=gantryId,.config=config};
    event.data = &evdata;
    int masterId=config->masterId;
    ULOG_INFO("adx_SetGantryConfig:gantryId=%d, masterId=%d,slaveId=%d", gantryId, config->masterId, config->slaveId);
    ret=AxisControl_HandleEvent(&_adx_AX[masterId],&event);
    return ret;
}

int adx_SetHomeParam(int axisId,HomeParameter_t *param,int bufferMode)
{
  (void)bufferMode;
    if (axisId<0 || axisId>MAX_AXIS_COUNT)
    {
        return SETPARAM_ERR;
    }
    struct{
        int axisId;
        HomeParameter_t *param;
    }evdata=
    {
        .axisId=axisId,
        .param=param
    };
    FsmEvent_t event={
    .name="adx_SetGantryConfig",
    .type=cmd_adx_SetHomeParam,
    .data=&evdata
    };
    ULOG_INFO("adx_SetHomeParam: axisId=%d", axisId); 
    int ret = AxisControl_HandleEvent(&_adx_AX[axisId], &event);
    return ret;
}


int adx_GoHome(int axisId, int index, int bufferMode)
{
    if (axisId<0 || axisId>MAX_AXIS_COUNT)
    {
        return SETPARAM_ERR;
    }
    struct
    {
        int axisId;
        int index;
        int bufferMode;
    } data = {
        .axisId = axisId,
        .index = index,
        .bufferMode = bufferMode
    };
    FsmEvent_t event = {
        .name = "adx_GoHome",
        .type = cmd_adx_GoHome,
        .data = &data
    };
    ULOG_INFO("adx_GoHome: axisId=%d,index=%d", axisId, index);
    int ret = AxisControl_HandleEvent(&_adx_AX[axisId], &event);
    return ret;
}


int adx_ReadTargetPos(int axisId, int index, double* pos)
{
    if (axisId<0 || axisId>MAX_AXIS_COUNT)
    {
        return SETPARAM_ERR;
    }
    struct
    {
        int axisId;
        int index;
        double* pos;
    } data = {
        .axisId = axisId,
        .index = index,
        .pos = pos
    };
    FsmEvent_t event = {
        .name = "adx_ReadTargetPos",
        .type = cmd_adx_ReadTargetPos,
        .data = &data
    };
    ULOG_INFO("adx_ReadTargetPos: axisId=%d,index=%d", axisId, index);
    int ret = AxisControl_HandleEvent(&_adx_AX[axisId], &event);
    return ret;

}

int adx_ReadTargetVel(int axisId, int index, double* vel)
{
	if (axisId<0 || axisId>MAX_AXIS_COUNT)
	{
		return SETPARAM_ERR;
	}
	struct
	{
		int axisId;
		int index;
		double* vel;
	} data = {
		.axisId = axisId,
		.index = index,
		.vel = vel
	};
	FsmEvent_t event = {
		.name = "adx_ReadTargetVel",
		.type = cmd_adx_ReadTargetVel,
		.data = &data
	};
	ULOG_DEBUG("adx_ReadTargetVel: axisId=%d,index=%d", axisId, index);
	int ret = AxisControl_HandleEvent(&_adx_AX[axisId], &event);
	return ret;
}

int adx_ReadTargetAcc(int axisId, int index, double* acc)
{
	if (axisId<0 || axisId>MAX_AXIS_COUNT)
	{
		return SETPARAM_ERR;
	}
	struct
	{
		int axisId;
		int index;
		double* acc;
	} data = {
		.axisId = axisId,
		.index = index,
		.acc = acc
	};
	FsmEvent_t event = {
		.name = "adx_ReadTargetAcc",
		.type = cmd_adx_ReadTargetAcc,
		.data = &data
	};
	ULOG_DEBUG("adx_ReadTargetAcc: axisId=%d,index=%d", axisId, index);
	int ret = AxisControl_HandleEvent(&_adx_AX[axisId], &event);
	return ret;
}

int adx_ReadTargetJerk(int axisId, int index, double* jerk)
{
	if (axisId<0 || axisId>MAX_AXIS_COUNT)
	{
		return SETPARAM_ERR;
	}
	struct
	{
		int axisId;
		int index;
		double* jerk;
	} data = {
		.axisId = axisId,
		.index = index,
		.jerk = jerk
	};
	FsmEvent_t event = {
		.name = "adx_ReadTargetJerk",
		.type = cmd_adx_ReadTargetJerk,
		.data = &data
	};
	ULOG_DEBUG("adx_ReadTargetJerk: axisId=%d,index=%d", axisId, index);
	int ret = AxisControl_HandleEvent(&_adx_AX[axisId], &event);
	return ret;
}

int adx_ReadLogicalPos(int axisId, int index, double* pos)
{
    if (axisId<0 || axisId>MAX_AXIS_COUNT)
    {
        return SETPARAM_ERR;
    }
    struct
    {
        int axisId;
        int index;
        double* pos;
    } data = {
        .axisId = axisId,
        .index = index,
        .pos = pos
    };
    FsmEvent_t event = {
        .name = "adx_ReadLogicalPos",
        .type = cmd_adx_ReadLogicalPos,
        .data = &data
    };
    //ULOG_DEBUG("adx_ReadLogicalPos: axisId=%d,index=%d", axisId, index);
    int ret = AxisControl_HandleEvent(&_adx_AX[axisId], &event);
    return ret;
}


int adx_ReadMultiLogicalPos(int axisId[], int index,int axisNum, double *pos)
{

    for(int i=0;i<axisNum;i++)
    {
        if (axisId[i]<0 || axisId[i] > MAX_AXIS_COUNT)
        {
            return SETPARAM_ERR;
        }
    }
    struct
    {
        int *axisId;
        int index;
        int axisNum;
        double* pos;
    } data = {
        .axisId = axisId,
        .index = index,
        .axisNum=axisNum,
        .pos = pos
    };
    FsmEvent_t event = {
        .name = "adx_ReadMultiLogicalPos",
        .type = cmd_adx_ReadMultiLogicalPos,
        .data = &data
    };
    ULOG_DEBUG("adx_ReadMultiLogicalPos: axisNum=%d  ",axisNum);
    int ret = AxisControl_HandleEvent(&_adx_AX[0], &event);
    return ret;
}

int adx_ReadLogicalVel(int axisId, int index, double* vel)
{
	if (axisId<0 || axisId>MAX_AXIS_COUNT)
	{
		return SETPARAM_ERR;
	}
	struct
	{
		int axisId;
		int index;
		double* vel;
	} data = {
		.axisId = axisId,
		.index = index,
		.vel = vel
	};
	FsmEvent_t event = {
		.name = "adx_ReadLogicalVel",
		.type = cmd_adx_ReadLogicalVel,
		.data = &data
	};
	ULOG_DEBUG("adx_ReadLogicalVel: axisId=%d,index=%d", axisId, index);
	int ret = AxisControl_HandleEvent(&_adx_AX[axisId], &event);
	return ret;
}


int adx_ReadLogicalAcc(int axisId, int index, double* acc)
{
	if (axisId<0 || axisId>MAX_AXIS_COUNT)
	{
		return SETPARAM_ERR;
	}
	struct
	{
		int axisId;
		int index;
		double* acc;
	} data = {
		.axisId = axisId,
		.index = index,
		.acc = acc
	};
	FsmEvent_t event = {
		.name = "adx_ReadLogicalAcc",
		.type = cmd_adx_ReadLogicalAcc,
		.data = &data
	};
	ULOG_DEBUG("adx_ReadLogicalAcc: axisId=%d,index=%d", axisId, index);
	int ret = AxisControl_HandleEvent(&_adx_AX[axisId], &event);
	return ret;
}

int adx_ReadLogicalJerk(int axisId, int index, double* jerk)
{
	if (axisId<0 || axisId>MAX_AXIS_COUNT)
	{
		return SETPARAM_ERR;
	}
	struct
	{
		int axisId;
		int index;
		double* jerk;
	} data = {
		.axisId = axisId,
		.index = index,
		.jerk = jerk
	};
	FsmEvent_t event = {
		.name = "adx_ReadLogicalJerk",
		.type = cmd_adx_ReadLogicalJerk,
		.data = &data
	};
	ULOG_DEBUG("adx_ReadLogicalJerk: axisId=%d,index=%d", axisId, index);
	int ret = AxisControl_HandleEvent(&_adx_AX[axisId], &event);
	return ret;
}

int adx_ReadActualPos(int axisId, int index, double* pos)
{
    if (axisId<0 || axisId>MAX_AXIS_COUNT)
    {
        return SETPARAM_ERR;
    }
    struct
    {
        int axisId;
        int index;
        double* pos;
    } data = {
        .axisId = axisId,
        .index = index,
        .pos = pos
    };
    FsmEvent_t event = {
        .name = "adx_ReadActualPos",
        .type = cmd_adx_ReadActualPos,
        .data = &data
    };
    ULOG_DEBUG("adx_ReadActualPos: axisId=%d,index=%d", axisId, index);
    int ret = AxisControl_HandleEvent(&_adx_AX[axisId], &event);
    return ret;
}


int adx_ReadMultiActualPos(int axisId[], int index,int axisNum, double* pos)
{
    for(int i=0;i<MAX_AXIS_COUNT;i++)
    {
        if (axisId[i]<0 || axisId[i]>MAX_AXIS_COUNT)
        {
            return SETPARAM_ERR;
        }
    }
    struct
    {
        int *axisId;
        int index;
        int axisNum;
        double* pos;
    } data = {
        .axisId = axisId,
        .index = index,
        .axisNum=axisNum,
        .pos = pos
    };
    FsmEvent_t event = {
        .name = "adx_ReadMultiActualPos",
        .type = cmd_adx_ReadMultiActualPos,
        .data = &data
    };
    ULOG_DEBUG("adx_ReadMultiActualPos: axisNum=%d", axisNum);
    int ret = AxisControl_HandleEvent(&_adx_AX[0], &event);
    return ret;
}


int adx_ReadActualVel(int axisId, int index, double* vel)
{
	if (axisId<0 || axisId>MAX_AXIS_COUNT)
	{
		return SETPARAM_ERR;
	}
	struct
	{
		int axisId;
		int index;
		double* vel;
	} data = {
		.axisId = axisId,
		.index = index,
		.vel = vel
	};
	FsmEvent_t event = {
		.name = "adx_ReadActualVel",
		.type = cmd_adx_ReadActualVel,
		.data = &data
	};
	ULOG_DEBUG("adx_ReadActualVel: axisId=%d,index=%d", axisId, index);
	int ret = AxisControl_HandleEvent(&_adx_AX[axisId], &event);
	return ret;
}

int adx_ReadActualAcc(int axisId, int index, double* acc)
{
	if (axisId<0 || axisId>MAX_AXIS_COUNT)
	{
		return SETPARAM_ERR;
	}
	struct
	{
		int axisId;
		int index;
		double* acc;
	} data = {
		.axisId = axisId,
		.index = index,
		.acc = acc
	};
	FsmEvent_t event = {
		.name = "adx_ReadActualAcc",
		.type = cmd_adx_ReadActualAcc,
		.data = &data
	};
	ULOG_DEBUG("adx_ReadActualAcc: axisId=%d,index=%d", axisId, index);
	int ret = AxisControl_HandleEvent(&_adx_AX[axisId], &event);
	return ret;
}

int adx_ReadActualJerk(int axisId, int index, double* jerk)
{
    if (axisId<0 || axisId>MAX_AXIS_COUNT)
    {
        return SETPARAM_ERR;
    }
    FsmEvent_t event = { .name = "adx_ReadStatus",.type = cmd_adx_ReadStatus };
    struct
    {
        int axisId;
        int index;
        double *jerk;
    }evdata = {
        .axisId = axisId,
        .index = index,
        .jerk = jerk,

    };
    event.data = &evdata;
    int ret = AxisControl_HandleEvent(&_adx_AX[axisId], &event);
    return ret;
}



int adx_ReadStatus(int axisId,int index,int *status)
{
    if(axisId<0 || axisId > MAX_AXIS_COUNT)
    {
        return SETPARAM_ERR;
    }
    FsmEvent_t event={.name="adx_ReadStatus",.type=cmd_adx_ReadStatus};
    struct 
    {
        int axisId;
        int index;
        int *status;
    }evdata={
        .axisId=axisId,
        .index=index,
        .status=status,

    };
    event.data=&evdata;
    int ret=AxisControl_HandleEvent(&_adx_AX[axisId],&event);
    return ret;
}

int adx_ReadMultiStatus(int axisId[],int index,int axisNum,int *status)
{
    for(int i=0;i<axisNum;i++)
    {    
        if(axisId[i]<0 || axisId[i] > MAX_AXIS_COUNT)
        {
            return SETPARAM_ERR;
        }
    }
    FsmEvent_t event={.name="adx_ReadMultiStatus",.type=cmd_adx_ReadMultiStatus};
    struct 
    {
        int *axisId;
        int index;
        int axisNum;
        int *status;
    }evdata={
        .axisId=axisId,
        .index=index,
        .axisNum=axisNum,
        .status=status,

    };
    event.data=&evdata;
    int ret=AxisControl_HandleEvent(&_adx_AX[0],&event);
    return ret;
}


int adx_GroupReadTargetPos(int grpId, int index, double* pos)
{
	if (grpId<0 || grpId>MAX_GROUP_COUNT)
	{
		return SETPARAM_ERR;
	}
	FsmEvent_t event = { .name = "adx_GroupReadTargetPos",.type = cmd_adx_GroupReadTargetPos };
	struct
	{
		int grpId;
		int index;
		double* pos;
	}evdata = {
		.grpId = grpId,
		.index = index,
		.pos = pos,
	};
	event.data = &evdata;
	ULOG_DEBUG("adx_GroupReadTargetPos: grpId=%d,index=%d", grpId, index);
	int ret = GroupControl_HandleEvent(&_adx_GRP[grpId], &event);
	return ret;
}


int adx_GroupReadLogicalPos(int grpId, int index, double* pos)
{
	if (grpId<0 || grpId>MAX_GROUP_COUNT)
	{
		return SETPARAM_ERR;
	}
	FsmEvent_t event = { .name = "adx_GroupReadLogicalPos",.type = cmd_adx_GroupReadLogicalPos };
	struct
	{
		int grpId;
		int index;
		double* pos;
	}evdata = {
		.grpId = grpId,
		.index = index,
		.pos = pos,
	};
	event.data = &evdata;
	ULOG_DEBUG("adx_GroupReadLogicalPos: grpId=%d,index=%d", grpId, index);
	int ret = GroupControl_HandleEvent(&_adx_GRP[grpId], &event);
	return ret;
}

int adx_GroupReadSpeed(int grpId, int index, double* speed)
{
	if (grpId<0 || grpId>MAX_GROUP_COUNT)
	{
		return SETPARAM_ERR;
	}
	FsmEvent_t event = { .name = "adx_GroupReadSpeed",.type = cmd_adx_GroupReadSpeed };
	struct
	{
		int grpId;
		int index;
		double* speed;
	}evdata = {
		.grpId = grpId,
		.index = index,
		.speed = speed,
	};
	event.data = &evdata;
	ULOG_DEBUG("adx_GroupReadSpeed: grpId=%d,index=%d", grpId, index);
	int ret = GroupControl_HandleEvent(&_adx_GRP[grpId], &event);
	return ret;
}

int adx_GroupReadStatus(int grpId, int index, int* status )
{
	if (grpId<0 || grpId > MAX_GROUP_COUNT)
	{
		return SETPARAM_ERR;
	}
	FsmEvent_t event = { .name = "adx_GroupReadStatus",.type = cmd_adx_GroupReadStatus };
	struct
	{
		int grpId;
		int index;
		int* status;
 
	}evdata = {
		.grpId = grpId,
		.index = index,
		.status = status,
 
	};
	event.data = &evdata;
	int ret = GroupControl_HandleEvent(&_adx_GRP[grpId], &event);
	return ret;
}


/**************************************************** 
 * 以下为轴组相关接口定义
 * **************************************************/
int adx_GroupDisable(int grpId)
{
    if(grpId<0 || grpId>MAX_GROUP_COUNT)
    {
        return SETPARAM_ERR;
    }
 
    FsmEvent_t event={.name="adx_GroupDisable",.type=cmd_adx_GroupDisable};
    struct 
    {
        int grpId;
    }evdata={
        .grpId=grpId
        
    };
    event.data=&evdata;
    // memcpy(event.msg,&evdata,sizeof(evdata));
    // ret=EnQueue(&cmdQueue,&event);
    int ret=GroupControl_HandleEvent(&_adx_GRP[grpId],&event);
    return ret;
}


 int adx_GroupEnable(int grpId)
 {
     if(grpId<0 || grpId>MAX_GROUP_COUNT)
     {
         return SETPARAM_ERR;
     }
  
     FsmEvent_t event={.name="adx_GroupEnable",.type=cmd_adx_GroupEnable};
     struct 
     {
         int grpId;
     }evdata={
         .grpId=grpId
     };
     event.data=&evdata;
    int  ret=GroupControl_HandleEvent(&_adx_GRP[grpId],&event);
    return ret;
 }

int adx_AadAxisToGroup(int grpId,int axisId)
{
    if(grpId<0 || grpId>MAX_GROUP_COUNT)
    {
        return SETPARAM_ERR;
    }
    if(axisId<0 || axisId>MAX_AXIS_COUNT)
    {
        return SETPARAM_ERR;
    }
    AxisControl_t *axis=&_adx_AX[axisId];
    if(axis->grpId>=0)
    {
       return UNSUPPORTEDCMD_ERR;//轴设置冲突
    }
 
    FsmEvent_t event={.name="adx_AddAxisToGroup",.type=cmd_adx_AddAxisToGroup};
     
    struct {
        int grpId;
        int axisId;
    }evdata={.grpId=grpId,.axisId=axisId};
    event.data = &evdata;
    // memcpy(event.msg,&evdata,sizeof(evdata));
    //int ret=EnQueue(&cmdQueue,&event);
    int ret=GroupControl_HandleEvent(&_adx_GRP[grpId],&event);
    return ret;
}

int adx_CreateAxisGroup(int grpId,int index,int coordNum,int axisNum,int *axisId)
{
    if(grpId<0 || grpId>MAX_GROUP_COUNT)
    {
        return SETPARAM_ERR;
    }
    if(axisNum<0 || axisNum>MAX_AXIS_COUNT ||coordNum<0||coordNum>MAX_AXIS_COUNT)
    {
        return SETPARAM_ERR;
    }
    
    FsmEvent_t event={.name="adx_CreateAxisGroup",.type=cmd_adx_CreateAxisGroup};
     
    struct {
        int grpId;
        int coordNum;
        int axisNum;
        int *axisId;
    }evdata={.grpId=grpId,.coordNum=coordNum,.axisNum=axisNum,.axisId=axisId};
    event.data = &evdata;
    ULOG_DEBUG("adx_CreateAxisGroup: grpId=%d,index=%d,coordNum=%d,axisNum=%d", grpId,index,coordNum,axisNum);
    printf("axisId={ ");
    for (int i=0; i < axisNum; i++)
        printf("%d ", axisId[i]);
    printf(" }\n");

    int ret=GroupControl_HandleEvent(&_adx_GRP[grpId],&event);
    return ret;
}

int adx_MoveLinearAbsolute(int grpId, int index,double *pos, double velocity,double acc,double dec,double jerk,
    int direction,int coordSys,  int bufferMode,int transitionMode,double *transitionParameter)
{
    if(grpId<0 || grpId>MAX_GROUP_COUNT  \
    || velocity<1.0e-5 ||acc<1.0e-5 || dec<1.0e-5 || jerk<1.0e-5 \
    || bufferMode<0 || bufferMode>3 )
    {
        ULOG_ERROR("adx_MoveLinearAbsolute: Invalid parameter ");
        return SETPARAM_ERR;
    }
    AxisGroupControl_t *grp=&_adx_GRP[grpId];
    struct 
    {
        int grpId;
        int index;
        double *pos;
        double velocity;
        double acc;
        double dec;
        double jerk;
        int direction;
        int coordSys;
        int bufferMode;
        int transitionMode;
        double *transitionParameter;      
    }evdata={
        .grpId=grpId,
        .index=index,
        .pos=pos,
        .velocity=velocity,
        .acc=acc,
        .dec=dec,
        .jerk=jerk,
        .direction=direction,
        .coordSys=coordSys,
        .bufferMode=bufferMode,
        .transitionMode=transitionMode,
        .transitionParameter=transitionParameter
    };
    FsmEvent_t event={
        .name="adx_MoveLinearAbsolute",
        .type=cmd_adx_MoveLinearAbsolute,
        .data=&evdata
    }; 
    
    //ULOG_INFO("%s %d %d %lf %lf %lf \n%lf %lf %lf %lf %d %d %d %d %lf ",__func__, grpId, index, pos[0],pos[1],pos[2],velocity,acc,dec,jerk,direction,
    //coordSys,bufferMode,transitionMode,transitionParameter[0]);

    //logFile(grp->cmdFile,"%s %d %d %lf %lf %lf %lf %lf %lf %lf %d %d %d %d %lf  \n",__func__, grpId, index, pos[0],pos[1],pos[2],velocity,acc,dec,jerk,direction,
    //coordSys,bufferMode,transitionMode,transitionParameter[0]);
    int ret=GroupControl_HandleEvent(grp,&event);
    return ret;
}

 
 /**
  * @brief 注册自定义运动学转换
  * 
  * @param grpId 轴组编号
  * @param pinverse 逆解函数指针
  * @param pforward 正解函数指针
  * @return int 
  */
int adx_SetKinematics(int grpId,int index, void *pinverse, void *pforward)
{
    if(grpId<0 || grpId>MAX_GROUP_COUNT || pinverse==NULL || pforward==NULL)
    {
        return SETPARAM_ERR;
    }
    struct 
    {
        int grpId;
        int index;
        void *p_inverse;
        void *p_forward;
 
    }evdata={
        .grpId=grpId,
        .index=index,
        .p_inverse=pinverse,
        .p_forward=pforward
    };
    FsmEvent_t event={
        .name="adx_SetKinematics",
        .type=cmd_adx_SetKinematics,
        .data=&evdata
    }; 
    ULOG_DEBUG("adx_SetKinematics: grpId=%d,index=%d", grpId, index);
    // memcpy(event.msg,&evdata,sizeof(evdata));
    // int ret=EnQueue(&cmdQueue,&event);
    int ret=GroupControl_HandleEvent(&_adx_GRP[grpId],&event);
    return ret;
}


int adx_RegisterSetDeviceFun(int grpId,int index, int devId,void *pSetDeviceFun)
{
    if(grpId<0 || grpId>MAX_GROUP_COUNT || pSetDeviceFun==NULL)
    {
        return SETPARAM_ERR;
    }
    AxisGroupControl_t *grp=&_adx_GRP[grpId];
    struct 
    {
        int grpId;
        int index;
        int devId;
        void *pSetDeviceFun;

    }data={

        .grpId=grpId,
        .index=index,
        .devId=devId,
        .pSetDeviceFun=pSetDeviceFun
    }; 
    FsmEvent_t event={
        .name="adx_RegisterSetDeviceFun",
        .type=cmd_adx_RegisterSetDeviceFun,
        .data=&data
    };
    ULOG_DEBUG("adx_RegisterSetDeviceFun: grpId=%d,index=%d,devId=%d,pSetDeviceFun=%p", grpId, index, devId,pSetDeviceFun);
    int ret=GroupControl_HandleEvent(grp,&event);
    return ret;
}


int adx_RegisterReadDeviceFun(int grpId,int index, int devId,void *pReadDeviceFun)
{
    if(grpId<0 || grpId>MAX_GROUP_COUNT || pReadDeviceFun==NULL)
    {
        return SETPARAM_ERR;
    }
    AxisGroupControl_t *grp=&_adx_GRP[grpId];
    struct 
    {
        int grpId;
        int index;
        int devId;
        void *pReadDeviceFun;

    }data={

        .grpId=grpId,
        .index=index,
        .devId=devId,
        .pReadDeviceFun=pReadDeviceFun
    }; 
    FsmEvent_t event={
        .name="adx_RegisterReadDeviceFun",
        .type=cmd_adx_RegisterReadDeviceFun,
        .data=&data
    };
    ULOG_DEBUG("adx_RegisterReadDeviceFun: grpId=%d,index=%d,devId=%d,pReadDeviceFun=%p", grpId, index, devId,pReadDeviceFun);
    int ret=GroupControl_HandleEvent(grp,&event);
    return ret;
}


int adx_RegisterSetMotorEnableFun(int axisId,int index,void *p_enableMotor)
{
    if(axisId<0 || axisId>MAX_AXIS_COUNT || p_enableMotor==NULL)
    {
        return SETPARAM_ERR;
    }
    AxisControl_t *axis=&_adx_AX[axisId];
    struct 
    {
        int axisId;
        int index;
        void *p_enableMotor;

    }data={

        .axisId=axisId,
        .index=index,
        .p_enableMotor=p_enableMotor
    }; 
    FsmEvent_t event={
        .name="adx_RegisterMotorEnableFun",
        .type=cmd_adx_RegisterSetMotorEnableFun,
        .data=&data
    };
    ULOG_DEBUG("adx_RegisterMotorEnableFun: axisId=%d,index=%d,pEnableMotor=%p", axisId, index,p_enableMotor);
    int ret=AxisControl_HandleEvent(axis,&event);
    return ret;
}



int adx_RegisterSetMotorRefPosFun(int axisId,int index,void *p_setMotorPos)
{
    if(axisId<0 || axisId>MAX_AXIS_COUNT || p_setMotorPos==NULL)
    {
        return SETPARAM_ERR;
    }
    AxisControl_t *axis=&_adx_AX[axisId];
    struct 
    {
        int axisId;
        int index;
        void *p_setMotorPos;

    }data={

        .axisId=axisId,
        .index=index,
        .p_setMotorPos=p_setMotorPos
    }; 
    FsmEvent_t event={
        .name="adx_RegisterSetMotorRefPosFun",
        .type=cmd_adx_RegisterSetMotorRefPosFun,
        .data=&data
    };
    ULOG_DEBUG("adx_RegisterSetMotorRefPosFun: axisId=%d,index=%d,pSetMotorRefPos=%p", axisId, index,p_setMotorPos);
    int ret=AxisControl_HandleEvent(axis,&event);
    return ret;

}

int adx_RegisterGetMotorActPosFun(int axisId,int index,void *p_getMotorPos)
{
    if(axisId<0 || axisId>MAX_AXIS_COUNT || p_getMotorPos==NULL)
    {
        return SETPARAM_ERR;
    }
    AxisControl_t *axis=&_adx_AX[axisId];
    struct 
    {
        int axisId;
        int index;
        void *p_getMotorPos;
    }data={
        .axisId=axisId,
        .index=index,
        .p_getMotorPos=p_getMotorPos
    };
    FsmEvent_t event={
        .name="adx_RegisterGetMotorActPosFun",
        .type=cmd_adx_RegisterGetMotorActPosFun,
        .data=&data
    };
    ULOG_DEBUG("adx_RegisterGetMotorActPosFun: axisId=%d,index=%d,pGetMotorActPos=%p", axisId, index,p_getMotorPos);
    int ret=AxisControl_HandleEvent(axis,&event);
    return ret;

}

int adx_SetMotorEncoderScale(int axisId,int index, int num,int den)
{
    if(axisId<0 || axisId>MAX_AXIS_COUNT || num<=0 || den<=0)
    {
        return SETPARAM_ERR;
    }
    AxisControl_t *axis=&_adx_AX[axisId];
    struct 
    {
        int axisId;
        int index;
        int num;
        int den;

    }data={

        .axisId=axisId,
        .index=index,
        .num=num,
        .den=den
    }; 
    FsmEvent_t event={
        .name="adx_SetMotorEncoderScale",
        .type=cmd_adx_SetMotorEncoderScale,
        .data=&data
    };
    ULOG_DEBUG("adx_SetMotorEncoderScale: axisId=%d,index=%d,num=%d,den=%d", axisId, index,num,den);
    int ret=AxisControl_HandleEvent(axis,&event);
    return ret;

}




int adx_ReadMultiParameters(int axisId, int index, axisParam_t *param)
{
    if(axisId<0 || axisId>MAX_AXIS_COUNT || param==NULL)
    {
        return SETPARAM_ERR;
    }
    AxisControl_t *axis=&_adx_AX[axisId];
    struct 
    {
        int axisId;
        int index;
        axisParam_t *param;

    }data={

        .axisId=axisId,
        .index=index,
        .param=param
    }; 
    FsmEvent_t event={
        .name="adx_ReadMultiParameters",
        .type=cmd_adx_ReadMultiParam,
        .data=&data
    };
    ULOG_DEBUG("adx_ReadMultiParameters: axisId=%d,index=%d", axisId, index);
    int ret=AxisControl_HandleEvent(axis,&event);
    return ret;
}

int adx_GroupWriteParameter(int grpId,int index,Mc_Parameter_enum paramId,double value,int bufferMode)
{
    if(grpId<0 || grpId>MAX_GROUP_COUNT )
    {
        return SETPARAM_ERR;
    }
    
    if(value<0.0)value=0.0;
    if(value>100.0)value=100.0;
    AxisGroupControl_t *grp=&_adx_GRP[grpId];
    struct 
    {
        int grpId;
        int index;
        int paramId;        
        double value;
        int bufferMode;
    }evdata={

        .grpId=grpId,
        .index=index,
        .paramId=paramId,
        .value=value,
        .bufferMode=bufferMode
    }; 
    FsmEvent_t event={
        .name="adx_GroupWriteParameter",
        .type=cmd_adx_GroupWriteMultiParam,
        .data=&evdata
    };
    ULOG_DEBUG("adx_GroupWriteParameter: grpId=%d,index=%d,paramId=%d,value=%lf", grpId, index, paramId,value);
    int ret=GroupControl_HandleEvent(grp,&event);
    return ret;
}

int adx_GroupReadParameter(int grpId,int index,Mc_Parameter_enum paramId,double *value)
{
    if(grpId<0 || grpId>MAX_GROUP_COUNT || value==NULL)
    {
        return SETPARAM_ERR;
    }
    AxisGroupControl_t *grp=&_adx_GRP[grpId];
    struct 
    {
        int grpId;
        int index;
        double paramId;
        double *value;

    }data={

        .grpId=grpId,
        .index=index,
        .paramId=paramId,
        .value=value
    }; 
    FsmEvent_t event={
        .name="adx_GroupReadParameter",
        .type=cmd_adx_GroupReadFloatParam,
        .data=&data
    };
    ULOG_DEBUG("adx_GroupReadParameter: grpId=%d,index=%d,paramId=%d", grpId, index, paramId);
    int ret=GroupControl_HandleEvent(grp,&event);
    return ret;
}

int adx_GroupWriteMultiParameters(int grpId,int index,GrpParam_t *param,int bufferMode)
{
    if(grpId<0 || grpId>MAX_GROUP_COUNT || param==NULL)
    {
        return SETPARAM_ERR;
    }
    AxisGroupControl_t *grp=&_adx_GRP[grpId];
    struct 
    {
        int grpId;
        int index;
        GrpParam_t *param;
        int bufferMode;

    }data={

        .grpId=grpId,
        .index=index,
        .param=param,
        .bufferMode=bufferMode
    }; 
    FsmEvent_t event={
        .name="adx_GroupWriteMultiParameters",
        .type=cmd_adx_GroupWriteMultiParam,
        .data=&data
    };
    ULOG_DEBUG("adx_GroupWriteMultiParameters: grpId=%d,index=%d", grpId, index);
    int ret=GroupControl_HandleEvent(grp,&event);
    return ret;
}

int adx_GroupReadMultiParameters(int grpId,int index,GrpParam_t *param,int bufferMode)
{
    if(grpId<0 || grpId>MAX_GROUP_COUNT || param==NULL)
    {
        return SETPARAM_ERR;
    }
    AxisGroupControl_t *grp=&_adx_GRP[grpId];
    struct 
    {
        int grpId;
        int index;
        GrpParam_t *param;
        int bufferMode;

    }data={

        .grpId=grpId,
        .index=index,
        .param=param,
        .bufferMode=bufferMode
    }; 
    FsmEvent_t event={
        .name="adx_GroupReadMultiParameters",
        .type=cmd_adx_GroupReadMultiParam,
        .data=&data
    };
    ULOG_DEBUG("adx_GroupReadMultiParameters: grpId=%d,index=%d", grpId, index);
    int ret=GroupControl_HandleEvent(grp,&event);
    return ret;
}




int adx_GroupSetOverride(int  grpId,int index,double velFactor,double accFactor,double jerkFactor,int bufferMode)
{
    if(grpId<0 || grpId>MAX_GROUP_COUNT \
        || velFactor>1.0 ||velFactor<-1.0)
    {
        return SETPARAM_ERR;
    }
    AxisGroupControl_t *grp=&_adx_GRP[grpId];
    struct 
    {
        int grpId;
        int index;
        double velFactor;
        double accFactor;
        double jerkFactor;
        int bufferMode;

    }data={

        .grpId=grpId,
        .index=index,
        .velFactor=velFactor,
        .accFactor=accFactor,
        .jerkFactor=jerkFactor,
        .bufferMode=bufferMode,
    }; 
    FsmEvent_t event={
        .name="adx_GroupSetOverride",
        .type=cmd_adx_SetOverride,
        .data=&data
    };
    ULOG_INFO("%s %d %d %lf %lf %lf %d ",__func__, grpId, index, velFactor,accFactor,jerkFactor,bufferMode);
    //logFile(grp->cmdFile,"%s %d %d %lf %lf %lf %d \n",__func__, grpId, index, velFactor,accFactor,jerkFactor,bufferMode);
    int ret=GroupControl_HandleEvent(grp,&event);
    return ret;
}


int adx_GroupInterrupt(int grpId,int index,double deceleration,double jerk,int bufferMode)
{
    if(grpId<0 || grpId>MAX_GROUP_COUNT)
    {
        return SETPARAM_ERR;
    }
    AxisGroupControl_t *grp=&_adx_GRP[grpId];
    struct 
    {
        int grpId;
        int index;
        double deceleration;
        double jerk;
        int bufferMode;

    }data={

        .grpId=grpId,
        .index=index,
        .deceleration=deceleration,
        .jerk=jerk,
        .bufferMode=bufferMode
    }; 
    FsmEvent_t event={
        .name="adx_GroupInterrupt",
        .type=cmd_adx_GroupInterrupt,
        .data=&data
    };
     
    //logFile(grp->cmdFile,"%s %d %d %lf %lf %d \n",__func__, grpId, index, deceleration, jerk, bufferMode);
    ULOG_INFO("%s %d %d %lf %lf %d",__func__ , grpId, index, deceleration, jerk, bufferMode);
    int ret=GroupControl_HandleEvent(grp,&event);
    return ret;
}

int adx_GroupContinue(int grpId,int index,double acceleration,double jerk,int bufferMode)
{

    if(grpId<0 || grpId>MAX_GROUP_COUNT)
    {
        return SETPARAM_ERR;
    }
    AxisGroupControl_t *grp=&_adx_GRP[grpId];
    struct 
    {
        int grpId;
        int index;
        double acceleration;
        double jerk;
        int bufferMode;

    }data={

        .grpId=grpId,
        .index=index,
        .acceleration=acceleration,
        .jerk=jerk,
        .bufferMode=bufferMode
    }; 
    FsmEvent_t event={
        .name="adx_GroupContinue",
        .type=cmd_adx_GroupContinue,
        .data=&data
    };
    //logFile(grp->cmdFile,"%s %d %d %lf %lf %d \n",__func__, grpId, index, acceleration, jerk, bufferMode);
    ULOG_INFO("%s %d %d %lf %lf %d",__func__ , grpId, index, acceleration, jerk, bufferMode);
    int ret=GroupControl_HandleEvent(grp,&event);
    return ret;
}


int adx_GroupStop(int grpId,int index,double deceleration,double jerk,int bufferMode)
{
    if(grpId<0 || grpId>MAX_GROUP_COUNT)
    {
        return SETPARAM_ERR;
    }
    AxisGroupControl_t *grp=&_adx_GRP[grpId];
    struct 
    {
        int grpId;
        int index;
        double deceleration;
        double jerk;
        int bufferMode;

    }data={

        .grpId=grpId,
        .index=index,
        .deceleration=deceleration,
        .jerk=jerk,
        .bufferMode=bufferMode
    }; 
    FsmEvent_t event={
        .name="adx_GroupStop",
        .type=cmd_adx_GroupStop,
        .data=&data
    };
    //logFile(grp->cmdFile,"%s %d %d %lf %lf %d \n",__func__, grpId, index, deceleration, jerk, bufferMode);
    ULOG_INFO("%s %d %d %lf %lf %d",__func__ , grpId, index, deceleration, jerk, bufferMode);
    int ret=GroupControl_HandleEvent(grp,&event);
    return ret;
}

int adx_GroupSetMcode(int grpId,int index,int *forwardAddr,int *reverseAddr,int forwardValue,int reverseValue,int bufferMode)
{
    if(grpId<0 || grpId>MAX_GROUP_COUNT || (forwardAddr==NULL && reverseAddr==NULL))
    {
        return SETPARAM_ERR;
    }
    AxisGroupControl_t *grp=&_adx_GRP[grpId];
    struct 
    {
        int grpId;
        int index;
        int *forwardValAddr;
        int *reverseValAddr;
        int forwardValue;
        int reverseValue;
        int bufferMode;

    }data={

        .grpId=grpId,
        .index=index,
        .forwardValAddr=forwardAddr,
        .reverseValAddr=reverseAddr,
        .forwardValue=forwardValue,
        .reverseValue=reverseValue,
        .bufferMode=bufferMode
    }; 
    FsmEvent_t event={
        .name="adx_GroupSetMcode",
        .type=cmd_adx_GroupSetMcode,
        .data=&data
    };
    //ULOG_DEBUG("adx_GroupSetMcode: grpId=%d,index=%d,forwardValAddr=%p,reverseValAddr=%p,
    //    forwardValue=%d,reverseValue=%d,bufferMode=%d", grpId, index, forwardAddr,reverseAddr,forwardValue,reverseValue,bufferMode);
    logFile(grp->cmdFile,"%s %d %d %p %p %d %d %d \n",__func__, grpId, index, forwardAddr,reverseAddr,forwardValue,reverseValue,bufferMode);
    int ret=GroupControl_HandleEvent(grp,&event);
    return ret;
}





int adx_SetDeviceStatus(int grpId,int index, int devId,int direction,int forwardStatus,int reverseStatus,int bufferMode)
{
    if(grpId<0 || grpId>MAX_GROUP_COUNT || devId<0 || devId>=MAX_DEVICE_COUNT)
    {
        return SETPARAM_ERR;
    }
    AxisGroupControl_t *grp=&_adx_GRP[grpId];
    struct 
    {
        int grpId;
        int index;
        int devId;
        int forwardStatus;
        int reverseStatus;
        int direction;
        int bufferMode;

    }data={

        .grpId=grpId,
        .index=index,
        .devId=devId,
        .forwardStatus=forwardStatus,
        .reverseStatus=reverseStatus,
        .direction=direction,
        .bufferMode=bufferMode
    }; 
    FsmEvent_t event={
        .name="adx_SetDeviceStatus",
        .type=cmd_adx_SetDeviceStatus,
        .data=&data
    };
    ULOG_DEBUG("adx_SetDeviceStatus: grpId=%d,index=%d,devId=%d,forwardStatus=%d,reverseStatus=%d,direction=%d,bufferMode=%d", grpId, index, devId,forwardStatus,reverseStatus,direction,bufferMode);
    //logFile(grp->cmdFile,"%s %d %d %d %d %d %d %d\n",__func__, grpId, index, devId,forwardStatus,reverseStatus,direction,bufferMode);
    int ret=GroupControl_HandleEvent(grp,&event);
    return ret;
}

int adx_ReadDeviceStatus(int grpId,int index, int devId,int direction,int *status)
{
    if(grpId<0 || grpId>MAX_GROUP_COUNT || status==NULL||devId<0||devId>=MAX_DEVICE_COUNT)
    {
        return SETPARAM_ERR;
    }
    AxisGroupControl_t *grp=&_adx_GRP[grpId];
    struct 
    {
        int grpId;
        int index;
        int devId;
        int direction;
        int *status;
    }data={
        .grpId=grpId,
        .index=index,
        .devId=devId,
        .direction=direction,
        .status=status
    }; 
    FsmEvent_t event={
        .name="adx_ReadDeviceStatus",
        .type=cmd_adx_ReadDeviceStatus,
        .data=&data
    };
    ULOG_DEBUG("adx_ReadDeviceStatus: grpId=%d,index=%d,devId=%d,status=%p", grpId, index, devId,status);
    int ret=GroupControl_HandleEvent(grp,&event);
    //*status=*event.data.status;
    return ret;
}




/*****************************************
 * 以下是非PLCopen标准接口
 * ***************************************/
int adx_ReadGroupBufferLength(int grpId,int *length)
{
    if(grpId<0 || grpId>MAX_GROUP_COUNT)
    {
        return SETPARAM_ERR;
    }
    if(length==NULL)
    {
        return SETPARAM_ERR;
    }
    *length=QueueFreeSize(&_adx_GRP[grpId].trajectoryQ);
    return 0;
}