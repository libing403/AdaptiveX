
#include <stdio.h>
#include <math.h>
#include <string.h>
//自定义头文件
#include "flexCore_if.h"
#include "axisControl.h"
#include "mcTypes.h"
#include "onlinetrapeprofile.h"

//第三方库
 #include"ulog.h"
int LogFile;
double globalReferenceTime=0.0;//参考时间
double syncInpTime=0.0;//插补时间
//驱动同步时钟
double syncDrvTime=0.0;
 GantryControl_t _gantry_Ctrl[MAX_GANTRY_COUNT]; //龙门同步控制器
/*==================== 状态转换辅助函数 ====================*/


int syncAxisStateTogroup(AxisControl_t *axis) {

    if(axis->grpId<0)return 0;
    AxisGroupControl_t *group=&_adx_GRP[axis->grpId];
    // 单轴状态影响轴组状态
    bool allAxesStandstill = true;
    //bool anyAxisError      = false;
    //bool anyAxisHoming     = false;
    switch (axis->state->status)
    {
        case AXIS_DISCRETEMOTION:          //断续运动,点位运动
        case AXIS_CONTINUOUSMOTION:       // 连续运动
        case AXIS_SYNCMOTION:      // 同步运动
        case AXIS_HOMING:            // 原点回归状态
            // 轴组处于同步运动状态
            if(group->state!=&GroupMovingState)
                groupTransitionStateNoSync(group,&GroupMovingState);
            break;
        case GRP_STANDBY:
            // 轴组处于待机状态
            for(int i=0;i<group->kine.axisCount;i++)
            {
                //axisTransitionState(group->kine.axis[i], &StandstillState);
                if(group->kine.axis[i]->state!= &StandstillState)
                {
                    allAxesStandstill = false;
                    break;
                }
            }
            if(allAxesStandstill)
            {
                 
                groupTransitionStateNoSync(group,&GroupStandbyState);
            } 
            break;   
        case AXIS_ERRORSTOP:
            // 单轴处于错误停止状态
            //group->state = &GroupErrorStopState;
            groupTransitionStateNoSync(group,&GroupStandbyState);
            break;
        default:
            break;
    }
    return 0;
}

int axisTransitionState(AxisControl_t *axis, const AxisState_t *newState) {
    int ret = 0;
    if (axis->state && axis->state->onExit) {
        ret=axis->state->onExit(axis);
   }
   axis->preState= axis->state; // 保存上一个状态
   axis->state = newState;
   if (axis->state && axis->state->onEnter) {
        ret=axis->state->onEnter(axis);
   }
   syncAxisStateTogroup(axis);
   return ret;
}

int axisTransitionStateAndNoSync(AxisControl_t *axis, const AxisState_t *newState) {
    int ret = 0;
    if (axis->state && axis->state->onExit) {
        ret=axis->state->onExit(axis);
   }
   axis->preState= axis->state; // 保存上一个状态
   axis->state = newState;
   if (axis->state && axis->state->onEnter) {
        ret=axis->state->onEnter(axis);
   }
 
   return ret;
}


int setMotorEnable(unsigned short int axisId, int enable,int timeOut_us)
{

    #if defined(WINDOWS_ENV)
    AxisControl_t *axis=&_adx_AX[axisId];
    char fileName[64];
    sprintf(fileName,"drv%d.txt",axisId);
    axis->drvFile=fopen(fileName,"w");
    if(axis->drvFile==NULL)
    {
        ULOG_ERROR("open drv file error\n");
        return -1;
    }
    #endif
   (void)timeOut_us;
    //ULOG_INFO("default setMotorEnable: axisId=%d, enable=%d\n", axisId, enable);
    return 0;
}

int getMotorEnable(unsigned short int axisId, int *enable,int timeOut_us)
{
    (void)timeOut_us;
    ULOG_INFO("getMotorEnable: axisId=%d, enable=%d\n", axisId, *enable);
    return 0;
}

//模拟设置电机参考位置函数
int setMotorRefPos(unsigned short int axisId, int refPos,int timeOut_us)
{
    #if defined(WINDOWS_ENV)
    AxisControl_t *axis=&_adx_AX[axisId];
    if(axis->drvFile!=NULL)
    {
        fprintf(axis->drvFile,"%d\n", refPos);
        fflush(axis->drvFile);
    }
    axis->act.pos=refPos;
    #endif
    (void)timeOut_us;
    // printf("setMotorRefPos: axisId=%d, refPos=%d\n", axisId, refPos);
     return 0;
}

int getMotorActPos(unsigned short int axisId, int *actPos,int timeOut_us)
{
    #if defined(WINDOWS_ENV)
    AxisControl_t *axis=&_adx_AX[axisId];
    //axis->act.pos=axis->drv.pos; //模拟实际位置等于驱动位置
    *actPos=axis->act.pos;
    #endif
    (void)timeOut_us;
     //printf("getMotorActPos: axisId=%d, actPos=%d\n", axisId, *actPos);
     return 0;
}

/*==================== 公共 API 实现 ====================*/
int axisControlInit(void) {
    //轴初始化
    static  int firstTime=0;
    int ret=0;
    AxisControl_t *axis=_adx_AX;
    memset(axis, 0, sizeof(AxisControl_t)*MAX_AXIS_COUNT);
    if (firstTime == 0)
    {

        for (int i = 0; i < MAX_AXIS_COUNT; i++)
        {         
            axis[i].hMutex=CreateMutex(NULL, FALSE, NULL);
            if (NULL == axis[i].hMutex)
            {
                ULOG_ERROR("axis->hMutex create error \n");
                return CREATE_MUTEX_ERR;
            }
            axis[i].axisTimeQ.hMutex = CreateMutex(NULL, FALSE, NULL);
            if (NULL == axis[i].axisTimeQ.hMutex)
            {
                ULOG_ERROR("axis->axisTimeQ.hMutex create error \n");
                return CREATE_MUTEX_ERR;
            }
            axis[i].addTimeQ.hMutex = CreateMutex(NULL, FALSE, NULL);
            if (NULL == axis[i].addTimeQ.hMutex)
            {
                ULOG_ERROR("axis->axisTimeQ.hMutex create error \n");
                return CREATE_MUTEX_ERR;
            }
            axis[i].grpTimeQ.hMutex=CreateMutex(NULL, FALSE, NULL);
            if (NULL == axis[i].grpTimeQ.hMutex)
            {
                ULOG_ERROR("axis->grpTimeQ.hMutex create error \r\n");
                return CREATE_MUTEX_ERR;
            }
            
        }

        firstTime = 1;
    }

    for (int i = 0; i < MAX_AXIS_COUNT; i++) {
        axis[i].axisId = i;
        axis[i].grpId=-1;//初始轴不属于任何轴组
        //初始化状态
        axis[i].state = &DisableState;
        if (axis[i].state->onEnter)
            axis[i].state->onEnter(&axis[i]);
        axis[i].inp.syncInpTime=0;
        axis[i].inp.syncDrvTime=0;
        axis[i].refTime=&globalReferenceTime;
        //初始化默认参数
        axis[i].axisParam.acc=1000.0;
        axis[i].axisParam.dec=1000.0;
        axis[i].axisParam.vs=0.0;
        axis[i].axisParam.ve=0.0;
        axis[i].axisParam.vel=100;
        axis[i].axisParam.jerk=10000;
        //电机参数初始化默认值
        axis[i].scale.num=1;
        axis[i].scale.den=1;
        //初始化轴规划器
        axis[i].inp.type=0;
        axis[i].inp.cmdNo=0;
        axis[i].inp.index=0;
        axis[i].inp.time=0.0;
        axis[i].inp.state=1;
        axis[i].inp.dt=CONTROL_PERIOD_SEC;
        //初始化在线规划器
        axis[i].inp.otp.count=0;
        axis[i].inp.otp.state=1;
        axis[i].inp.otp.param.Ts=CONTROL_PERIOD_SEC;
        axis[i].inp.otp.param.velMax=10.0;
        axis[i].inp.otp.param.accMax=100.0;
        axis[i].inp.otp.tolerance=fmax(1.1*0.5*axis[i].inp.otp.param.accMax*CONTROL_PERIOD_SEC*CONTROL_PERIOD_SEC,1.0e-4);
        
        axis[i].inp.velCtl.count=0;
        axis[i].inp.velCtl.state=1;
        axis[i].inp.velCtl.param.Ts=CONTROL_PERIOD_SEC;
        axis[i].inp.velCtl.param.velMax=10.0;
        axis[i].inp.velCtl.param.accMax=100.0;
        axis[i].inp.velCtl.tolerance=fmax(1.1*0.5*axis[i].inp.otp.param.accMax*CONTROL_PERIOD_SEC*CONTROL_PERIOD_SEC,1.0e-4);

        axis[i].inp.osp.count=0;
        axis[i].inp.osp.state=1;
        axis[i].inp.osp.param.Ts=CONTROL_PERIOD_SEC;
        axis[i].inp.osp.param.velMax=10.0;
        axis[i].inp.osp.param.accMax=100.0;
        axis[i].inp.osp.param.jerkMax=1000.0;
        axis[i].inp.osp.tolerance=fmax(1.1*1.0/6.0*axis[i].inp.osp.param.jerkMax*CONTROL_PERIOD_SEC*CONTROL_PERIOD_SEC*CONTROL_PERIOD_SEC, 1.0e-5);
        //注册硬件操作函数        
        axis[i].setMotorRefPos=setMotorRefPos;
        axis[i].getMotorActPos=getMotorActPos;
        axis[i].setMotorEnable=setMotorEnable;
        axis[i].getMotorEnable=getMotorEnable;


        //轨迹队列初始化
        ret = InitQueue(&(axis[i].trajectoryQ), axis[i].axisTrajBuffer, MAX_AXISTRAJECT_COUNT+1, sizeof(AxisTrajectory_t));
        if (ret)
        {
            ULOG_ERROR("file %s,fuction line %s  error!",__FILE__,__FUNCTION__,__LINE__);
            return QUEUEINIT_ERR;
        }
        //轴插补队列初始化
        ret = InitQueue(&(axis[i].axisTimeQ), axis[i].axisTimeQBuffer, MAX_TIMEQUEUE_COUNT+1, sizeof(InpTimeData_t));
        if (ret)
        {
            ULOG_ERROR("file %s,fuction line %s  error!",__FILE__,__FUNCTION__,__LINE__);
            return QUEUEINIT_ERR;
        }
        //轴叠加运动队列初始化
        ret = InitQueue(&(axis[i].addTimeQ), axis[i].addTimeQBuffer, MAX_TIMEQUEUE_COUNT+1, sizeof(InpTimeData_t));
        if (ret)
        {
            ULOG_ERROR("file %s,fuction line %s  error!",__FILE__,__FUNCTION__,__LINE__);
            return QUEUEINIT_ERR;
        }
        //轴组插补队列初始化
        ret = InitQueue(&(axis[i].grpTimeQ), axis[i].grpTimeQBuffer, MAX_TIMEQUEUE_COUNT+1, sizeof(InpTimeData_t));
        if (ret)
        {
            ULOG_ERROR("file %s,fuction line %s  error!",__FILE__,__FUNCTION__,__LINE__);
            return QUEUEINIT_ERR;
        }

    }
    ULOG_INFO("axis control initialized with %d axis.", MAX_AXIS_COUNT);

    //初始化倍率
    for (int i = 0; i < MAX_AXIS_COUNT; i++) {
        ret=initSpeedRate(&axis[i].speedRate, 1.0, -1.0, 1.0,\
            10,100,100);
        if(ret)
        {
            ULOG_ERROR("file %s,fuction line %s  error!",__FILE__,__FUNCTION__,__LINE__);
            return QUEUEINIT_ERR;
        }
        ret=initSpeedRate(&axis[i].stopRate, 1.0, -1.0, 1.0,\
        10,100,100);
        if(ret)
        {
            ULOG_ERROR("file %s,fuction line %s  error!",__FILE__,__FUNCTION__,__LINE__);
            return QUEUEINIT_ERR;
        }
    }
    return 0;

}



int axisSyncToPosition(AxisControl_t *axis, double position)
{
    
    //时间同步
    axis->inp.syncInpTime=axis->axisCurTime.syncTime;
    axis->inp.time=axis->axisCurTime.time;
    axis->inp.syncDrvTime=axis->axisCurTime.time;
    //插补器位置同步
    axis->inp.prePos=position;
    axis->inp.curPos=position;
    axis->inp.preVelocity=0.0;
    axis->inp.curVelocity=0.0;
    axis->inp.preAcceleration=0.0;
    axis->inp.curAcceleration=0.0;
    axis->inp.preJerk=0.0;
    axis->inp.curJerk=0.0;
    axis->inp.targetPos=position;
    
    //梯形在线规划器位置同步
    axis->inp.otp.state=1;
    axis->inp.otp.count=0;
    axis->inp.otp.param.rk=position;
    axis->inp.otp.param.drk=0.0;
    axis->inp.otp.param.qLast=position;
    axis->inp.otp.param.dqLast=0.0;
    axis->inp.otp.param.ddqLast=0.0;
    axis->inp.otp.qtNext=position;
    axis->inp.otp.dqtNext=0.0;
    axis->inp.otp.ddqtNext=0.0;
    //S形在线规划器位置同步
    axis->inp.osp.state=1;
    axis->inp.osp.count=0;
    axis->inp.osp.param.rk=position;
    axis->inp.osp.param.drk=0.0;
    axis->inp.osp.param.qLast=position;
    axis->inp.osp.param.dqLast=0.0;
    axis->inp.osp.param.ddqLast=0.0;
    axis->inp.osp.qtNext=position;
    axis->inp.osp.dqtNext=0.0;
    axis->inp.osp.ddqtNext=0.0;
    //出队数据同步
    axis->axisCurTime.cmdNo=axis->inp.cmdNo;
    axis->axisCurTime.index=axis->inp.index;  
    axis->axisCurTime.syncTime=axis->inp.syncInpTime;
    axis->axisCurTime.time=axis->inp.time;
    axis->axisCurTime.lastPos=position;
    axis->axisCurTime.curPos=position;
    //驱动指令位置同步
    axis->drv.pos=position;
    axis->drv.vel=0.0;
    axis->drv.acc=0.0;
    axis->drv.jerk=0.0;

    axis->cmd.pos=position;
    axis->cmd.vel=0.0;
    axis->cmd.acc=0.0;
    axis->cmd.jerk=0.0;
    //清空队列
    ClearQueueISR(&axis->axisTimeQ);
    ClearQueueISR(&axis->trajectoryQ);
 

    return 0;
}
 

/*==================== 事件处理入口 ====================*/
int AxisControl_HandleEvent(AxisControl_t *axis, const FsmEvent_t *event) {
    int ret=0;
    WaitForSingleObject(axis->hMutex, INFINITE);
    if (axis->state && axis->state->onEvent) {
        //ULOG_INFO("Axis %d: execute cmd [%s] ",axis->axisId,event->name);
        ret= axis->state->onEvent(axis, event);
        //ULOG_INFO("Axis %d: fihish cmd [%s] ret=%d",axis->axisId,event->name,ret);
        
    }
    else
    {
        ULOG_ERROR("%s [cmdNo=%d],%s ,Line %d : Axis.state==NULL,axis.state.OnEvent==NULL is null.\n",event->name,event->type,__func__,__LINE__);
        ret=CMDDISPATCH_ERR;
    }
    ReleaseMutex(axis->hMutex);
    return ret;
}



int axisControlUpdate(void)
{
    AxisControl_t *axis=_adx_AX;
    int ret=0;
    for(int i=0;i<MAX_AXIS_COUNT;i++)
    {
        if(axis[i].state&&axis[i].state->onUpdate)
        {
            ret=axis[i].state->onUpdate(&axis[i],CONTROL_PERIOD_SEC);
            if(ret)
            {
                ULOG_ERROR("Axis %d: axisControlUpdate error ret=%d.",i,ret);
                break;
            }

        }
            
    }
    return ret;
}

int axisControlRTUpdate(void)
{
    int ret=0;
	AxisControl_t *axis= _adx_AX;
	for(int i=0;i<MAX_AXIS_COUNT;i++)
	{
		
		if(axis[i].state && axis[i].state->onRtUpdate)
		{
			//调用实时处理函数
			ret=axis[i].state->onRtUpdate(&axis[i], CONTROL_PERIOD_SEC);
			if(ret)
			{
				ULOG_ERROR("axis[%d] onRtUpdate error ret=%d\n",i,ret);
                return ret;
			}
		}
		 
	}
    return ret;
}


bool isAxisStandstill(AxisControl_t *axis) {
    
    if (axis == NULL) return false;
    bool isAxisStandstill = false;
    //bool isGroupFinish = false;
 
    // 检查轴是否处于静止状态
    double disErr=axis->cmd.pos-axis->inp.targetPos;
    if (!QueueLength(&axis->axisTimeQ) \
    && !QueueLength(&axis->trajectoryQ) \
    && 1==axis->inp.otp.state \
    && 1==axis->inp.osp.state \
    && 1==axis->inp.velCtl.state\
    && fabs(disErr)<MAX_OTG_POS_ERR\
    && axis->waitCount++ > STATE_WAIT_COUNT \
    //&& axis->gantryCtrl->config.slaveId!=axis->axisId//从轴不可能转化为静止，除非解除龙门同步
    ) 
    {      
        isAxisStandstill = true;
        axis->waitCount = 0; // 重置等待计数器
    }
    return isAxisStandstill  ;
}

int writeAxisTimeQ(AxisControl_t *axis)
{
	int ret;
	InpTimeData_t inpTime;
	inpTime.cmdNo=axis->inp.cmdNo;
	inpTime.index=axis->inp.index;
	inpTime.syncTime=axis->inp.syncInpTime;
	inpTime.time=axis->inp.time;
    inpTime.lastPos=axis->inp.prePos;
    inpTime.curPos=axis->inp.curPos;
    
    //RTOS需要关中断，避免读写冲突
    ret=EnQueue(&axis->axisTimeQ,&inpTime);
    if(ret)
    {
        ULOG_ERROR("writeAxisTimeQ error\n");
        return ret;
    }
	return 0;
}

int moveAbsoluteBufferMode(AxisControl_t *axis, const FsmEvent_t *event)
{
    int ret=0;
    //提取事件数据paramId
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
    }data;
    memcpy(&data,event->data,sizeof(data));
    if(event->type==cmd_adx_MoveRelative) 
    {
        if(axis->inp.type==0)
            data.position=axis->inp.otp.param.rk+data.position;
        else if(axis->inp.type==1)
            data.position=axis->inp.osp.param.rk+data.position;    
         
    }
    //速度合法性检查
    if(fabs(data.velocity)<0.001)//=0.0
    {
        ULOG_ERROR("Axis %d: state %s:  cmd [%d] %s error, velocity is zero.",axis->axisId,axis->state->name,event->type,event->name );
        ret=SETPARAM_ERR ;
        return ret;
    }
    else if(data.velocity<0.0)//<0.0
    {
        data.velocity=axis->axisParam.vel; 
        ULOG_WARNING("Axis %d: state %s:  cmd [%d] %s warning, velocity adjusted to %f.", axis->axisId, axis->state->name, event->type, event->name, data.velocity); 
    }
    //加速度合法性检查
    if(fabs(data.acceleration)<0.001)//=0.0
    {
        ULOG_ERROR("Axis %d: state %s:  cmd [%d] %s error, acceleration is zero.",axis->axisId,axis->state->name,event->type,event->name );
        ret=SETPARAM_ERR ;
        return ret;
    }
    else if(data.acceleration<0.0)//<0.0
    {
        data.acceleration=axis->axisParam.acc; 
        ULOG_WARNING("Axis %d: state %s:  cmd [%d] %s warning, acceleration adjusted to %f.", axis->axisId, axis->state->name, event->type, event->name, data.acceleration); 
    }
    //减速度合法性检查
    if(fabs(data.deceleration)<0.001)//=0.0
    {
        ULOG_ERROR("Axis %d: state %s:  cmd [%d] %s error, deceleration is zero.",axis->axisId,axis->state->name,event->type,event->name );
        ret=SETPARAM_ERR ;
        return ret;
    }
    else if(data.deceleration<0.0)//<0.0
    {
        data.deceleration=axis->axisParam.acc; 
        ULOG_WARNING("Axis %d: state %s:  cmd [%d] %s warning, deceleration adjusted to %f.", axis->axisId, axis->state->name, event->type, event->name, data.deceleration); 
    }
    //加加速度合法性检查
    if(data.jerk>=0.001&&data.jerk<100)
    {
        data.jerk=axis->axisParam.jerk; 
        ULOG_WARNING("Axis %d: state %s:  cmd [%d] %s warning, jerk adjusted to %f.", axis->axisId, axis->state->name, event->type, event->name, data.jerk); 
    }

    //根据轴的混成方式处理
    switch (data.bufferMode)
    {
    case mcAborting:
        //中止模式，打断已有指令，执行当前指令
        ret=setAxisPlanerTarget(axis,data.position,data.velocity,data.acceleration,data.deceleration,data.jerk);    
        if(ret)
        {
            ULOG_ERROR("Aixs %d: state %s:  setAxisPlanerTarget error.",axis->axisId,axis->state->name );
            break;
        }
        axis->inp.cmdNo=event->type;
        axis->inp.index=data.index;             
        break;
    case mcBuffered:
        //缓冲模式，等待已有指令完成后，执行当前指令
        AxisTrajectory_t cmd;
        memcpy(&cmd,&data,sizeof(AxisTrajectory_t));
        ret=EnQueue(&axis->trajectoryQ,&cmd);
        if(ret)
        {
            ULOG_ERROR("Axis %d: EnQueue failed.", axis->axisId);     
            break;               
        }
        break;
    case mcBlendingLow:
        //混成模式，低速合并，执行当前指令
        if(axis->inp.type==0)
            data.velocity=fmin(data.velocity,axis->inp.otp.param.velMax); 
        else
            data.velocity=fmin(data.velocity,axis->inp.osp.param.velMax);
        ret=setAxisPlanerTarget(axis,data.position,data.velocity,data.acceleration,data.deceleration,data.jerk);    
        if(ret)
        {
            ULOG_ERROR("Axis[%d]: state %s:  cmd [%d] %s error.",axis->axisId,axis->state->name,event->type,event->name );
            break;
        }

        axis->inp.cmdNo=event->type;
        axis->inp.index=data.index;          
        break;

    case mcBlendingPrevious:
        //混成模式，前一个速度合并，执行当前指令
        if(axis->inp.type==0)
            data.velocity= axis->inp.otp.param.velMax; 
        else
            data.velocity=axis->inp.osp.param.velMax;
        ret=setAxisPlanerTarget(axis,data.position,data.velocity,data.acceleration,data.deceleration,data.jerk);    
        if(ret)
        {
            ULOG_ERROR("Axis[%d]: state %s:  cmd [%d] %s error.",axis->axisId,axis->state->name,event->type,event->name );
        }

        axis->inp.cmdNo=event->type;
        axis->inp.index=data.index;        

        break;
    case mcBlendingHigh:
        //混成模式，高速合并，执行当前指令
        if(axis->inp.type==0)
            data.velocity=fmax(data.velocity,axis->inp.otp.param.velMax); 
        else
            data.velocity=fmax(data.velocity,axis->inp.osp.param.velMax);
        ret=setAxisPlanerTarget(axis,data.position,data.velocity,data.acceleration,data.deceleration,data.jerk);    
        if(ret)
        {
            ULOG_ERROR("Axis[%d]: state %s:  cmd [%d] %s error.",axis->axisId,axis->state->name,event->type,event->name );                   
            break;
        }
 
        axis->inp.cmdNo=event->type;
        axis->inp.index=data.index; 
        break;

    default:
        ULOG_ERROR("Axis %d: state %s:  cmd [%d] %s error, bufferMode is invalid.",axis->axisId,axis->state->name,event->type,event->name );
        ret= SETPARAM_ERR;
        break;
    }
    return ret;
}


int axisInterpolation(AxisControl_t *axis,double dt)
{
    int ret=0;
    //插补队列满了或者无缓存指令且插补器已完成插补
    if(!QueueFreeSize(&(axis->axisTimeQ)) || (!QueueLength(&axis->trajectoryQ) && 1==axis->inp.otp.state && 1==axis->inp.osp.state ))
        return 0;
    if(QueueLength(&axis->trajectoryQ)&& 1==axis->inp.otp.state && 1==axis->inp.osp.state )
    {
        //从轨迹队列中取出一个轨迹点
        AxisTrajectory_t data;
        int eventType=cmd_adx_MoveAbsolute;         
        ret=DeQueue(&axis->trajectoryQ,&data);
        if(ret)
        {
            ULOG_ERROR("Axis %d: DeQueue error ret=%d.",axis->axisId,ret);
            return ret;
        }
        ret=setAxisPlanerTarget(axis,data.position,data.velocity,data.acceleration,data.deceleration,data.jerk);    
        if(ret)
        {
            ULOG_ERROR("Aixs %d: state %s:  setAxisPlanerTarget error.",axis->axisId,axis->state->name );
            return ret;
        }
        ret=axisTransitionState(axis,&DisceteMotionState);
        if(ret)
        {
            ULOG_ERROR("Axis %d: axisTransitionState error ret=%d",axis->axisId,ret);
            return ret;
        }
        axis->inp.cmdNo=eventType;
        axis->inp.index=data.index;  
    }   
    //从物理轴更新轴状态(位置、速度、加速度)
    double nextState[3]={0.0};
    if(axis->inp.type==0)
    {   
        ret=onlineTrapePlannerUpdate(&axis->inp.otp,nextState);
        if(ret)
        {
            ULOG_ERROR("Axis %d: onlineTrapePlannerUpdate error ret=%d.",axis->axisId,ret);
            return ret;
        }
        axis->inp.state=axis->inp.otp.state;
    }
    else if(axis->inp.type==1)
    {
        ret=onlineScurvePlannerUpdate(&axis->inp.osp,nextState);
        if(ret)
        {
            ULOG_ERROR("Axis %d: onlineScurvePlannerUpdate error ret=%d.",axis->axisId,ret);
            return ret;
        }
        axis->inp.state=axis->inp.osp.state;
    }
    axis->inp.prePos=axis->inp.curPos;
    axis->inp.preVelocity=axis->inp.curVelocity;
    axis->inp.preAcceleration=axis->inp.curAcceleration;
    axis->inp.curPos=nextState[0];
    axis->inp.curVelocity=nextState[1];
    axis->inp.curAcceleration=nextState[2];
    axis->inp.syncInpTime=axis->inp.time;
    axis->inp.time=axis->inp.syncInpTime+dt;
    
    ret=writeAxisTimeQ(axis);
    if(ret)
    {
        
        //ULOG_ERROR("Axis %d: writeAxisTimeQ error ret=%d.",axis->axisId,ret);
        ULOG_ERROR("Axis %d: writeAxisTimeQ error ret=%d. QueueLength=%d\n",axis->axisId,ret,QueueLength(&axis->axisTimeQ));
        return ret;
    }
    return ret;

}





int initSpeedRate(SpeedRate_t *sr,double targetRate,  double minRate, double maxRate,
    double maxVel,double maxAcc,double maxJerk)
{
    if (sr == NULL) return 1;
    sr->targetRate = targetRate;
    sr->currentRate = targetRate;
    sr->deltaRate = 0.0;
    sr->minRate = minRate;
    sr->maxRate = maxRate;
    sr->maxVel=maxVel;
    sr->maxAcc=maxAcc;
    sr->maxJerk=maxJerk;
    return 0;
}
int setSpeedRate(SpeedRate_t *sr,double targetRate,double maxVel,double maxAcc,double maxJerk,double Ts)
{
    //SpeedRate_t *sr=&axis->speedRate;
     
    if (sr == NULL) return 1;
    if (targetRate > sr->maxRate) targetRate = sr->maxRate;
    if (targetRate < sr->minRate) targetRate = sr->minRate;
    sr->targetRate=targetRate;
    sr->maxVel=maxVel;
    sr->maxAcc=maxAcc;
    sr->maxJerk=maxJerk;
    sr->Ts=Ts;
   

    // double sig=sign(targetRate-sr->currentRate);
    // if (fabs(curVelocity)>1.0e-3 )
    // {
    //     //加速
    //     sr->targetRate = targetRate;
    //     sr->deltaRate = sig*accMax/curVelocity*Ts;
         
    // }
    // else
    // {
    //     //保持
    //     sr->targetRate = targetRate;
    //     sr->deltaRate = (targetRate-sr->currentRate);
    // }
    return 0;
}



double updateSpeedRate(SpeedRate_t *sr,double curVel,double curAcc)
{
    if (sr == NULL) return 1.0;
    if(fabs(sr->targetRate-sr->currentRate)<1.0e-5)
    {
        return sr->targetRate;
    }
    double sig=sign(sr->targetRate-sr->currentRate);

    double accErr=(sig*sr->maxAcc- curAcc )/sr->maxAcc;
    sr->e_integral+=accErr;
    sr->deltaRate=(0.001*accErr+0.001*sr->e_integral);
    //double jerkLimit=sr->maxJerk-curJerk*curJerk*curJerk*sr->currentRate*sr->currentRate*sr->currentRate;
    double rateVel;//rateAcc,rateVelMax, rateAccMax;

    if(fabs(curVel)>1.0e-5)
        rateVel=sig*fabs(sr->maxAcc*sr->currentRate/curVel);
    else
        rateVel=(sr->targetRate-sr->currentRate)/ sr->Ts;

    //如果加速度过大，限制加速度
 
    // if(fabs(curVel)>1.0e-5)
    //     rateAccMax= fabs(sr->maxJerk*sr->currentRate/curVel);
    // else
    //     rateAccMax=fabs(rateVel-sr->rateVelocity)/ sr->Ts;
    // if((rateVel-sr->rateVelocity)/sr->Ts<-rateAccMax)
    //     rateVel=-rateAccMax*sr->Ts+sr->rateVelocity;
    // else if((rateVel-sr->rateVelocity)/sr->Ts>rateAccMax)
    //     rateVel=rateAccMax*sr->Ts+sr->rateVelocity;

    if(fabs(sr->targetRate-sr->currentRate)<fabs(rateVel * sr->Ts))
    {
        rateVel= (sr->targetRate - sr->currentRate)/sr->Ts;
        sr->rateVelocity=rateVel;
        sr->currentRate=sr->targetRate;
       
    }
    else
    {
        sr->currentRate += rateVel * sr->Ts;
        sr->rateVelocity=rateVel;
    }
    if (sr->currentRate > sr->maxRate)
        sr->currentRate = sr->maxRate;
    else if (sr->currentRate < sr->minRate) 
        sr->currentRate = sr->minRate;
    return sr->currentRate;
}

double getSpeedRate(SpeedRate_t *sr)
{
    if (sr == NULL) return 1.0;
    return sr->currentRate;
}




//根据倍率更新单轴位置
int updatePositionAtsyncDrvTime(AxisControl_t *axis, double syncDrvTime,double dt, InpTimeData_t *atq) {
    (void)dt;
    int ret=0;
    double axisPos=0.0;
   
    //根据倍率进行二次插补
    InpTimeData_t tq;
    if(!GetHeadElemISR(&axis->axisTimeQ, &tq) )
    {
        //单轴插补时队列会有数据
        while ( syncDrvTime > tq.time &&!GetHeadElemISR(&axis->axisTimeQ, &tq))
        {

            ret = DeQueueISR(&axis->axisTimeQ, &tq);
            if (ret)
            {
                ULOG_ERROR("DeQueueISR error %d\r\n", ret);
                return ret;
            }
        }
        if (tq.time - tq.syncTime < 1.0e-5|| QueueLengthISR(&axis->axisTimeQ)==0)//时间小于10us
        {
            axisPos = tq.curPos;
        }
        else
        {
            axisPos  = tq.lastPos + (tq.curPos - tq.lastPos) * (syncDrvTime - tq.syncTime) / (tq.time - tq.syncTime);
        }
        atq->cmdNo=tq.cmdNo;
        atq->index=tq.index;
        atq->syncTime=tq.syncTime;
        atq->time=syncDrvTime;
        atq->lastPos=atq->curPos;
        atq->curPos=axisPos;
    }
    else
    {
        //轴组插补时或队列无数据，取轴组更新的位置
        memcpy(atq,&axis->axisCurTime,sizeof(InpTimeData_t));
    }

    return 0;
}


//获取单轴规划位置
int getAxisTimeQueue(AxisControl_t *axis,double dt,InpTimeData_t *atq)
{
    int ret = 0;
    //更新单轴插补位置
    memcpy(atq,&axis->axisCurTime,sizeof(InpTimeData_t));
    //倍率更新
    double axisRate=updateSpeedRate(&axis->speedRate,axis->cmd.vel,axis->cmd.acc);
    double stopRate=updateSpeedRate(&axis->stopRate,axis->cmd.vel,axis->cmd.acc);
    double rate=axisRate*stopRate;
    if(QueueLengthISR(&axis->axisTimeQ) )
    {
        double syncDrvTime=axis->inp.syncDrvTime;
       // double axisPos=axis->drv.pos;
        //单轴运动时钟更新
        syncDrvTime = syncDrvTime + rate * dt; 
        axis->inp.syncDrvTime = syncDrvTime;
        ret = updatePositionAtsyncDrvTime(axis, syncDrvTime, dt, atq);
        if (ret)
        {
            ULOG_ERROR("Axis %d: updatePositionAtsyncDrvTime error ret=%d.", axis->axisId, ret);
            return ret;
        }
    }
    //更新轴规划信息
    double vel = (atq->curPos - axis->cmd.pos)/dt;
    double acc = (vel - axis->cmd.vel) / dt;
    double jerk = (acc - axis->cmd.acc) / dt;
    axis->cmd.jerk = jerk;
    axis->cmd.vel = vel;
    axis->cmd.acc = acc;
    axis->cmd.pos = atq->curPos;

    return ret;
}


//获取单轴驱动位置,进行限位和滤波平滑处理
int updateAxisDrivePosition(AxisControl_t *axis,double dt)
{
    (void)dt;
    int ret=0;
    //软限位
    //ret=applyAxisSoftLimit(axis, dt);
    if(ret)
    {
        ULOG_ERROR("Axis %d: state %s: applyAxisSoftLimit error.",axis->axisId,axis->state->name);
        return ret;
    }
    //位置滤波平滑
    axis->drv.pos=axis->cmd.pos;
    axis->drv.vel=axis->cmd.vel;
    axis->drv.acc=axis->cmd.acc;
    axis->drv.jerk=axis->cmd.jerk;
    return 0;
}




int setAxisPlanerTarget(AxisControl_t *axis,double targetPos,double velocity,
    double acceleration,double deceleration,double jerk)
{
    double qLast,dqLast,ddqLast;
    qLast=axis->inp.curPos;
    dqLast=axis->inp.curVelocity;
    ddqLast=axis->inp.curAcceleration;
    //插补器参数同步
    axis->inp.targetPos=targetPos;
    axis->inp.targetVel=velocity;
    axis->inp.targetAcc=acceleration;
    axis->inp.targetJerk=jerk;

    if(fabs(jerk)<0.001)//梯形
    {
        axis->inp.type=0;

        axis->inp.otp.param.qLast=qLast;
        axis->inp.otp.param.dqLast=dqLast;
        axis->inp.otp.param.ddqLast=ddqLast;     
        axis->inp.otp.param.Ts=CONTROL_PERIOD_SEC;

        axis->inp.otp.qtNext=qLast;
        axis->inp.otp.dqtNext=dqLast;
        axis->inp.otp.ddqtNext=ddqLast;
        onlineTrapePlannerSetTarget(&axis->inp.otp,targetPos,velocity,acceleration,deceleration);
        axis->inp.state=axis->inp.otp.state;
    }
    else if(jerk<-0.001)
    {
        axis->inp.type=1;
        axis->inp.targetPos=targetPos;
        axis->inp.osp.param.qLast=qLast;
        axis->inp.osp.param.dqLast=dqLast;
        axis->inp.osp.param.ddqLast=ddqLast;
        axis->inp.osp.param.Ts=CONTROL_PERIOD_SEC;

        axis->inp.osp.qtNext=qLast;
        axis->inp.osp.dqtNext=dqLast;
        axis->inp.osp.ddqtNext=ddqLast;
        onlineScurePlannerSetTarget(&axis->inp.osp,targetPos,velocity,acceleration,deceleration,axis->axisParam.jerk);
        axis->inp.state=axis->inp.osp.state;
    }
    else 
    {
        axis->inp.type=1;
        axis->inp.targetPos=targetPos;
        axis->inp.osp.param.qLast=qLast;
        axis->inp.osp.param.dqLast=dqLast;
        axis->inp.osp.param.ddqLast=ddqLast;
        axis->inp.osp.param.Ts=CONTROL_PERIOD_SEC;

        axis->inp.osp.qtNext=qLast;
        axis->inp.osp.dqtNext=dqLast;
        axis->inp.osp.ddqtNext=ddqLast;
        onlineScurePlannerSetTarget(&axis->inp.osp,targetPos,velocity,acceleration,deceleration,jerk);
    }
    return 0;    
}



int setAxisVelocityPlanerTarget(AxisControl_t *axis,double targetVel,
    double acceleration,double deceleration,double jerk)
{
  (void)deceleration;
    //速度控制器参数更新
    axis->inp.targetPos=targetVel;
    axis->inp.velCtl.param.qLast=axis->cmd.vel;
    axis->inp.velCtl.param.dqLast=axis->cmd.acc;
    axis->inp.velCtl.param.ddqLast=axis->cmd.jerk;
    axis->inp.velCtl.param.Ts=CONTROL_PERIOD_SEC;
    axis->inp.velCtl.qtNext=axis->cmd.vel;
    axis->inp.velCtl.dqtNext=axis->cmd.acc;
    axis->inp.velCtl.ddqtNext=axis->cmd.jerk;
    //插补器参数同步
    axis->inp.syncInpTime=axis->axisCurTime.syncTime;
    axis->inp.time=axis->axisCurTime.time;
    axis->inp.syncDrvTime=axis->axisCurTime.time;
    axis->inp.prePos=axis->cmd.pos;
    axis->inp.curPos=axis->cmd.pos;
    axis->inp.preVelocity=axis->cmd.vel;
    axis->inp.curVelocity=axis->cmd.vel;
    axis->inp.preAcceleration=axis->cmd.acc;
    axis->inp.curAcceleration=axis->cmd.acc;
    axis->inp.preJerk=axis->cmd.jerk;
    axis->inp.curJerk=axis->cmd.jerk;

    onlineTrapePlannerSetTarget(&axis->inp.velCtl,targetVel,acceleration,jerk,jerk);
    axis->inp.state=axis->inp.velCtl.state;

    return 0;
}



int axisVelocityInterpolation(AxisControl_t *axis,double dt)
{
    int ret=0;
     //倍率更新
    double axisRate=updateSpeedRate(&axis->speedRate,axis->cmd.vel,axis->cmd.acc);
    double stopRate=updateSpeedRate(&axis->stopRate,axis->cmd.vel,axis->cmd.acc);
    double rate=axisRate*stopRate;
    double dtime=rate*dt;
    double nextState[3]={0.0};
    ret=onlineTrapePlannerUpdate(&axis->inp.velCtl,nextState);
    if(ret)
    {
        ULOG_ERROR("Axis %d: onlineScurvePlannerUpdate error ret=%d.",axis->axisId,ret);
        return ret;
    }
    //更新轴插补器状态
    axis->inp.preJerk=axis->cmd.jerk;
    axis->inp.curJerk=nextState[2];
    axis->inp.preAcceleration=axis->cmd.acc;
    axis->inp.curAcceleration=nextState[1];
    axis->inp.preVelocity=axis->cmd.vel;
    axis->inp.curVelocity=nextState[0];
    axis->inp.prePos=axis->cmd.pos;
    axis->inp.curPos+=0.5*(axis->inp.preVelocity+axis->inp.curVelocity)*dtime;
    axis->inp.syncInpTime=axis->inp.time;;
    axis->inp.time=axis->inp.syncInpTime+dtime;
    axis->inp.syncDrvTime=axis->inp.time;

    InpTimeData_t atq;
    atq.cmdNo=axis->inp.cmdNo;
    atq.index=axis->inp.index;
    atq.syncTime=axis->inp.syncInpTime;
    atq.time=axis->inp.time;
    atq.lastPos=axis->inp.prePos;
    atq.curPos=axis->inp.curPos;
    memcpy(&axis->axisCurTime,&atq,sizeof(InpTimeData_t));
    //更新轴驱动信息
    double ds = atq.curPos - axis->cmd.pos;
    double vel = ds / dt;
    double acc = (vel - axis->cmd.vel) / dt;
    double jerk = (acc - axis->cmd.acc) / dt;
    axis->cmd.jerk = jerk;
    axis->cmd.vel = vel;
    axis->cmd.acc = acc;
    axis->cmd.pos = atq.curPos;
    //写入调试文件
    //logFile(axis->drvFile, "%.5f %.5f %.5f %.5f %.5f \n ", *axis->refTime, axis->cmd.pos, axis->cmd.vel, axis->cmd.acc, axis->cmd.jerk);
    //printf("%.5f %.5f\n ", axis->refTime,drvPos);
    //fflush(axis->drvFile); 
    ret=updateMotorInfo(axis);
    if(ret)
    {
        ULOG_ERROR("Axis %d: updateMotorInfo error ret=%d.",axis->axisId,ret);
        return ret;
    }
    return 0;
}

int setAxisSoftPosiLmt(AxisControl_t *axis, double value)
{
    if (axis == NULL) return 1;
    //设置最大限位
    axis->axisParam.posiLmt = value;
    //setCubicCurveParam(&axis->lmtCtrl[1], value - 5.0, value, value - 5.0, value, 1.0, 0.0);
    axis->axisLmt.x1=value;
    axis->axisLmt.xa=5;
    axis->axisLmt.amax=1.0/ axis->axisLmt.xa;
    axis->axisLmt.q1=value- axis->axisLmt.xa+1.0/(2.0* axis->axisLmt.amax);
   
    return 0;
}

int setAxisSoftNegaLmt(AxisControl_t *axis, double value)
{
    if (axis == NULL) return 1;
    //设置最小限位
    axis->axisParam.negaLmt = value;
    //setCubicCurveParam(&axis->lmtCtrl[0], value , value + 5.0, value , value + 5.0, 0.0, 1.0);
    axis->axisLmt.x0=value;
    axis->axisLmt.xa=5;
    axis->axisLmt.amax=1.0/ axis->axisLmt.xa;
    axis->axisLmt.q0=value + axis->axisLmt.xa-1.0/(2.0* axis->axisLmt.amax);
 
    return 0;
}



/**
 * @brief 限制各关节的运动范围,
 *
 * @param axis 轴结构体
 * @param pos 轴坐标
 * @return int 返回0，执行成功，返回1，参数输入错误。
 */
int checkAxisSoftLimit(AxisControl_t *axis,double dt)
{
    double drvPos,drvVel,drvAcc,drvJerk;
    drvPos=axis->cmd.pos;
    drvVel=axis->cmd.vel;
    drvAcc=axis->cmd.acc;
    drvJerk=axis->cmd.jerk;
    //最小限制
    if (axis->axisParam.enableNegaLmt && drvPos - axis->lmtCtrl[0].q1 < 0.0 )
    {
        calcCubicCurvePos(&axis->lmtCtrl[0], drvPos, &drvPos);
        if (drvPos - axis->axisParam.negaLmt < 1.0e-3)
        {	//靠近限位
            drvPos = axis->axisParam.negaLmt;
        }
        else
        {
            ;//远离限位
        }
    }
    
    //最大限制
    if ( axis->axisParam.enablePosiLmt&& drvPos - axis->lmtCtrl[1].q0 > 0.0 )
    {
        calcCubicCurvePos(&axis->lmtCtrl[1], drvPos, &drvPos);
        if (drvPos - axis->axisParam.posiLmt > 1.0e-3)
        {
            //靠近限位
            drvPos = axis->axisParam.posiLmt;

        }
        else
        {
            ;//远离限位
        }
    }
    drvVel=(drvPos-axis->drv.pos)/dt;
    drvAcc=(drvVel-axis->drv.vel)/dt;
    drvJerk=(drvAcc-axis->drv.acc)/dt;
    axis->drv.pos=drvPos;
    axis->drv.vel=drvVel;
    axis->drv.acc=drvAcc;
    axis->drv.jerk=drvJerk;
    if(drvAcc>50000)
        printf("drvPos=%f, drvVel=%f, drvAcc=%f, drvJerk=%f\n",drvPos,drvVel,drvAcc,drvJerk);
    return 0;
}


int applyAxisSoftLimit(AxisControl_t *axis,double dt)
{
    double drvPos,drvVel,drvAcc,drvJerk;
    drvPos=axis->cmd.pos;
    drvVel=axis->cmd.vel;
    drvAcc=axis->cmd.acc;
    drvJerk=axis->cmd.jerk;
    //最小限制
    if (axis->axisParam.enableNegaLmt )
    {
        if(drvPos<axis->axisLmt.x0)
            drvPos=axis->axisLmt.q0;
        else if(drvPos< axis->axisLmt.x0 + axis->axisLmt.xa)
        {
            drvPos=axis->axisLmt.q0 + 0.5*axis->axisLmt.amax*(drvPos-axis->axisLmt.x0)*(drvPos-axis->axisLmt.x0);
        }
    }
    //最大限制
    if ( axis->axisParam.enablePosiLmt)
    {

        if( drvPos>axis->axisLmt.x1)
            drvPos=axis->axisLmt.q1;
        else if(drvPos>axis->axisLmt.x1 - axis->axisLmt.xa)
        {
            drvPos=axis->axisLmt.q1 - 0.5*axis->axisLmt.amax*(drvPos-axis->axisLmt.x1)*(drvPos-axis->axisLmt.x1);
        }
    }
    drvVel=(drvPos-axis->drv.pos)/dt;
    drvAcc=(drvVel-axis->drv.vel)/dt;
    drvJerk=(drvAcc-axis->drv.acc)/dt;
    axis->drv.pos=drvPos;
    axis->drv.vel=drvVel;
    axis->drv.acc=drvAcc;
    axis->drv.jerk=drvJerk;
    if(drvAcc>50000)
        printf("drvPos=%f, drvVel=%f, drvAcc=%f, drvJerk=%f\n",drvPos,drvVel,drvAcc,drvJerk);
    return 0;
}


int writeAxisParam_Handle(AxisControl_t *axis, const FsmEvent_t *event) {
    if (axis == NULL || event == NULL) return 1;
    int ret = 0;
    //提取事件数据paramId
    struct {
        int axisId;
        int index;
        int paramId;
        double value;
        int bufferMode;

    }*ptr=event->data;
    switch (ptr->paramId)
    {   
        case mcSWLimitNeg:
        {
            setAxisSoftNegaLmt(axis, ptr->value);
            break;
        }

        case mcSWLimitPos:
        {

            setAxisSoftPosiLmt(axis, ptr->value);
            break;
        }

        default:
            // 处理其他参数
            ret=UNSUPPORTEDPARAM_ERR;
            break;
    }
    return ret;
}


int writeAxisBoolParam_Handle(AxisControl_t *axis, const FsmEvent_t *event) {
    if (axis == NULL || event == NULL) return 1;
    int ret = 0;
    //提取事件数据paramId
    struct {
        int axisId;
        int index;
        int paramId;
        bool value;
        int bufferMode;

    }*ptr=event->data;
    switch (ptr->paramId)
    {   
        case mcEnableLimitNeg:
        {
            axis->axisParam.enableNegaLmt = ptr->value;
            break;
        }

        case mcEnableLimitPos:
        {

            axis->axisParam.enablePosiLmt = ptr->value;
            break;
        }
        
        default:
            // 处理其他参数
            ret=UNSUPPORTEDPARAM_ERR;
            break;
    }
    return ret;
}

int readAxisParam_Handle(AxisControl_t *axis, const FsmEvent_t *event) {
    if (axis == NULL || event == NULL) return 1;
    int ret = 0;
    // 假设 event->data 是一个指向参数数据的指针
    // 这里假设数据结构为 { axisId, paramId, value }
    struct {
        int axisId;
        int index;
        int paramId;
    }*ptr=event->data;
    // 处理参数写入事件
    // 根据 paramId 和 value 更新轴的参数
    // 这里可以添加具体的参数处理逻辑
    switch (ptr->paramId)
    {   
        case mcVelocityRate:
        {
            struct {
                int axisId;
                int index;
                int paramId;
                double *value;
            }*data=event->data;
            *(data->value)=getSpeedRate(&axis->speedRate);
            ret=0;
            break;
        }
        case mcCmdPosition:
        {
            struct {
                int axisId;
                int index;
                int paramId;
                double *value;
            }*data=event->data;
            if(axis->inp.type==0)
            {
                *(data->value)=axis->inp.otp.param.rk;
            }
            else if(axis->inp.type==1)
            {
                *(data->value)=axis->inp.osp.param.rk;
            }
            ret=0;
            break;
        }
        

        default:
            // 处理其他参数
            ret=UNSUPPORTEDPARAM_ERR;
            break;
    }
    return ret;
}

/*==================== 龙门双轴同步API实现 =====================*/
// 配置龙门的同步参数，包括主轴和从轴的ID、偏移量以及偏移速度等。
// 龙门参数配置
int gantrySyncConfig(int gantryId, GantrySyncConfig_t *gantConfig)
 {
    if (gantConfig->masterId < 0 || gantConfig->masterId >= MAX_AXIS_COUNT 
        ||gantConfig->slaveId < 0 || gantConfig->slaveId >= MAX_AXIS_COUNT 
        || gantConfig->masterId == gantConfig->slaveId
        || gantryId < 0 || gantryId >= MAX_GANTRY_COUNT) 
        {
        ULOG_ERROR("Invalid motor ID: master=%d, slave=%d", gantConfig->masterId, gantConfig->slaveId);
        return 1;//参数错误
    }
    if(_gantry_Ctrl[gantryId].isCoupled)
    {
        ULOG_ERROR("Gantry ID %d is isCoupling.", gantryId);
        return 2;//龙门组已耦合，无法配置
    }

    // 检查轴是否已在其他龙门组中耦合
    for (int i = 0; i < MAX_GANTRY_COUNT; i++) {
        if ((_gantry_Ctrl[i].config.masterId == gantConfig->masterId 
            || _gantry_Ctrl[i].config.masterId == gantConfig->slaveId
            || _gantry_Ctrl[i].config.slaveId == gantConfig->masterId
            || _gantry_Ctrl[i].config.slaveId == gantConfig->slaveId) 
            && _gantry_Ctrl[i].isCoupled == 1) {
        ULOG_ERROR("Motor ID already in gantry coupling: master=%d, slave=%d", gantConfig->masterId, gantConfig->slaveId);
        return 3; // 轴已在其他龙门组中耦合
        }
    }
    
    // 配置龙门同步参数
    _gantry_Ctrl[gantryId].config.masterId = gantConfig->masterId;// 主轴ID
    _gantry_Ctrl[gantryId].config.slaveId = gantConfig->slaveId;// 从轴ID
    AxisControl_t *masterAxis=&_adx_AX[gantConfig->masterId];
    AxisControl_t *slaveAxis=&_adx_AX[gantConfig->slaveId];

    _gantry_Ctrl[gantryId].config.masterOffset_mm = gantConfig->masterOffset_mm;// 主轴偏移量（mm）
    _gantry_Ctrl[gantryId].config.slaveOffset_mm = gantConfig->slaveOffset_mm;// 从轴偏移量（mm）
    _gantry_Ctrl[gantryId].config.offsetspeed_mm_s = gantConfig->offsetspeed_mm_s;// 偏移速度（mm/s）
    _gantry_Ctrl[gantryId].isCoupled = 1; // 默认解除耦合
    
    // 初始化状态
    _gantry_Ctrl[gantryId].positionError = 0.0f;// 位置误差
    _gantry_Ctrl[gantryId].maxPositionError = 0.1f;// 最大位置误差（mm）
    _gantry_Ctrl[gantryId].syncFactor = 1.0f;// 同步因子
    _gantry_Ctrl[gantryId].isHoming = false;// 是否在对准状态
    _gantry_Ctrl[gantryId].isSynced = false;// 是否同步状态
    _gantry_Ctrl[gantryId].errorCount = 0;// 错误计数
     
    masterAxis->cfg.axisType=AXIS_TYPE_GANTRY_MASTER;
    slaveAxis->cfg.axisType=AXIS_TYPE_GANTRY_SLAVE;
    masterAxis->gantryCtrl=&_gantry_Ctrl[gantryId];
    slaveAxis->gantryCtrl=&_gantry_Ctrl[gantryId];
    ULOG_INFO("Gantry sync configured: gantryId=%d, master=%d, slave=%d", gantryId,
               gantConfig->masterId,gantConfig->slaveId);
    
    return 0;
}

// 基于主从控制的龙门同步实现
int gantryMasterSlaveControl(GantryControl_t *gantryCtrl, double dt) {
    
    (void)dt;
    int ret=0;
    AxisControl_t *masterAxis=&_adx_AX[gantryCtrl->config.masterId];
    AxisControl_t *slaveAxis=&_adx_AX[gantryCtrl->config.slaveId];
    
    //插补位置更新
    memcpy(&slaveAxis->inp,&masterAxis->inp,sizeof(AxisInterpolation_t));
    //规划位置更新
    memcpy(&slaveAxis->cmd,&masterAxis->cmd,sizeof(AxisCmd_t));

    //从轴驱动位置计算
    slaveAxis->drv.pos=masterAxis->drv.pos+(gantryCtrl->config.slaveOffset_mm-gantryCtrl->config.masterOffset_mm);
    slaveAxis->drv.vel=masterAxis->drv.vel;
    slaveAxis->drv.acc=masterAxis->drv.acc;
    slaveAxis->drv.jerk=masterAxis->drv.jerk;

    ret=updateMotorInfo(slaveAxis);
    if(ret)
    {
        ULOG_ERROR("Gantry ID %d: updateMotorInfo error ret=%d.", gantryCtrl->config.slaveId,ret);
    }
    return ret;
}

 int axisGoHome(AxisControl_t *axis)
 {
    int ret=0;
    //回原点运动
    switch (axis->homeParam.homeMode)
    {
    case 0: //当前位置为原点
        //插补器状态更新
    



        break;
    
    default:
        break;
    }
    return ret;
 }


 int updateMotorInfo(AxisControl_t *axis)
 {
    int ret=0;
    axis->drvStatus=P_OT;
    //获取电机位置
    int pos=0;
    ret=axis->getMotorActPos(axis->axisId, &pos, 1000);
     if (ret)
     {
         ULOG_ERROR("getMotorActPos error ret=%d\n", ret);
         return ret;
     }
     //axis->act.pos=(double)pos*axis->scale.den/(double)axis->scale.num;
    //设置电机位置
    pos=axis->drv.pos*axis->scale.num/axis->scale.den;
    ret = axis->setMotorRefPos(axis->axisId,  pos, 1000);
     if (ret)
     {
         ULOG_ERROR("motorSetPos error ret=%d\n", ret);
         return ret;
     }
    return ret;
 }