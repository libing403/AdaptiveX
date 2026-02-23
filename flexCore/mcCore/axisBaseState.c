
#include <stdio.h>
#include <math.h>
#include <string.h>
//自定义头文件
#include "axisControl.h"
#include "mcTypes.h"
#include "Queue.h"
#include "flexCore_if.h"
//第三方库
#include"ulog.h"

/*==================== BaseState 状态实现 ====================*/
int Base_onEnter(AxisControl_t *axis) {
    (void)axis;
    return 0;
}

int Base_onExit(AxisControl_t *axis) {
(void)axis;
    return 0;
}


int Base_onEvent(AxisControl_t *axis, const FsmEvent_t *event) {
    if (!event) return 0;
    int ret=0;
    switch(event->type) {
        case cmd_adx_Power:
        {
            struct {
                int axisId;
                int enable;
                int startMode;
                int stopMode;
            }*data=event->data;
            if(data->enable)
            {
                //使能轴
                ret=axis->setMotorEnable(axis->axisId,1,1000);
                if(ret)
                {
                    ULOG_ERROR("setMotorEnable error ret=%d", ret);
                    break;
                }
                ret=axisTransitionState(axis,&StandstillState);
                if(ret)
                {
                    ULOG_ERROR("Axis %d: axisTransitionState error ret=%d",axis->axisId,ret);
                    break;
                }
            }              
            else
            {   
                //关闭轴
                 ret=axis->setMotorEnable(axis->axisId,0,1000);
                 {
                    ULOG_ERROR("setMotorEnable error ret=%d", ret);
                    break;
                 }
                ret=axisTransitionState(axis,&DisableState);
                {
                    ULOG_ERROR("Axis %d: axisTransitionState error ret=%d",axis->axisId,ret);
                    break;
                }
            }
            break;
        }
        case cmd_adx_WriteFloatParam:
        {
            ret=writeAxisParam_Handle(axis, event);
            break;
        }
        case cmd_adx_GroupWriteIntParam:
        {
            ret=writeAxisBoolParam_Handle(axis, event);
            break;
        }
        case cmd_adx_ReadFloatParam:
        {
            ret=readAxisParam_Handle(axis, event);
            break;
        }
        case cmd_adx_MoveAbsolute:
        case cmd_adx_MoveRelative:        
        {          
            ret=moveAbsoluteBufferMode(axis, event);
            if(ret)
            {
                ULOG_ERROR("Axis %d: axisTransitionState error ret=%d",axis->axisId,ret);
                break;
            }
            ret=axisTransitionState(axis,&DisceteMotionState); 
            if(ret)
            {
                ULOG_ERROR("Axis %d: axisTransitionState error ret=%d",axis->axisId,ret);
                break;
            }
            break;
        }
        case cmd_adx_SetGantryConfig:
        {
            struct {
                int gantryId;
                GantrySyncConfig_t *gantryConfig;
            }*data=event->data;
            ret=gantrySyncConfig(data->gantryId, data->gantryConfig); 
            if (ret)
            {
                ULOG_ERROR("gantrySyncConfig error ret=%d\n", ret);
            }
            break;
        }
        case cmd_adx_SetGantryEnable:
        {
            struct{
                int gantryId;
                int enable;
            }*data=event->data;
            axis->gantryCtrl[data->gantryId].isCoupled=data->enable;
            int slaveId=axis->gantryCtrl[data->gantryId].config.slaveId;
            AxisControl_t *slaveAxis=&_adx_AX[slaveId];
            if(data->enable)
            {   
                ret=axisTransitionState(axis,&DisceteMotionState); 
                if(ret)
                {
                    ULOG_ERROR("Axis %d: axisTransitionState error ret=%d",axis->axisId,ret);
                    break;
                }
                ret=axisTransitionState(slaveAxis,&SyncMotionState); 
                if(ret)
                {
                    ULOG_ERROR("Axis %d: axisTransitionState error ret=%d",slaveAxis->axisId,ret);
                    break;
                }
            }
            else
            {
                ret=axisTransitionState(axis,&StandstillState);
                if(ret)
                {
                    ULOG_ERROR("Axis %d: axisTransitionState error ret=%d",axis->axisId,ret);
                    break;
                }
                ret=axisTransitionState(slaveAxis,&StandstillState); 
                if(ret)
                {
                    ULOG_ERROR("Axis %d: axisTransitionState error ret=%d",slaveAxis->axisId,ret);
                    break;
                }

            }
            break;
        }
        case cmd_adx_SetOverride:
        {
            struct {
                int axisId;
                int index;
                double velFactor;
                double accFactor;
                double jerkFactor;
                int bufferMode;
            }*data=event->data;
            ret=setSpeedRate(&axis->speedRate, data->velFactor, axis->axisParam.vel,\
                 axis->axisParam.acc, axis->axisParam.jerk,axis->inp.dt);
            if (ret)
            {
                ULOG_ERROR("setSpeedRate error ret=%d\n", ret);
            }
            break;
        }
        case cmd_adx_MoveVelocity:
        {
            struct {
                int axisId;
                int index;
                double velocity;
                double acceleration;
                double deceleration;
                double jerk;
                int direction;
                int bufferMode;
            }*data=event->data;

            ret=setAxisVelocityPlanerTarget(axis,data->velocity,data->acceleration,data->deceleration,data->jerk);
            //ret=setAxisPlanerTarget(axis,data->velocity,data->acceleration,data->jerk, data->jerk,data->jerk);    
            if(ret)
            {
                ULOG_ERROR("Aixs %d: state %s:  setAxisVelocityPlanerTarget error.",axis->axisId,axis->state->name );
                break;
            }
            axis->inp.cmdNo=event->type;
            axis->inp.index=data->index;  
            ret=axisTransitionState(axis,&ContinusMotionState); 
            if(ret)
            {
                ULOG_ERROR("Axis %d: axisTransitionState error ret=%d",axis->axisId,ret);
                break;
            }
            ClearQueueISR(&axis->trajectoryQ);
            ClearQueueISR(&axis->axisTimeQ);
            break;
     
        }
        case cmd_adx_ReadTargetPos:
        {
            struct 
            {
                int axisId;
                int index;
                double* pos;
            }*data=event->data;
            *data->pos=axis->inp.targetPos;
            break;
        }
        case cmd_adx_ReadTargetVel:
        {
            struct 
            {
                int axisId;
                int index;
                double* vel;
            }*data=event->data;
            *data->vel=axis->inp.targetVel;
            break;
        }
        case cmd_adx_ReadTargetAcc:
        {
            struct 
            {
                int axisId;
                int index;
                double* acc;
            }*data=event->data;
            *data->acc=axis->inp.targetAcc;
            break;
        }
        case cmd_adx_ReadTargetJerk:
        {
            struct 
            {
                int axisId;
                int index;
                double* jerk;
            }*data=event->data;
            *data->jerk=axis->inp.targetJerk;
            break;
        }
        case cmd_adx_ReadLogicalPos:
        {
            struct 
            {
                int axisId;
                int index;
                double* pos;
            }*data=event->data;
            *data->pos=axis->drv.pos;
            break;
        }
        case cmd_adx_ReadMultiLogicalPos:
        {

            struct 
            {
                int *axisId;
                int index;
                int axisNum;
                double* pos;
            }*data=event->data;
            for(int i=0;i<data->axisNum;i++)
            {
                
                data->pos[i]=axis[data->axisId[i]].drv.pos;
            }
                
            break;
        }
        case cmd_adx_ReadActualPos:
        {
            struct 
            {
                int axisId;
                int index;
                double* pos;
            }*data=event->data;
            *data->pos=axis->act.pos;
            break;
        }
        case cmd_adx_ReadMultiActualPos:
        {
            struct 
            {
                int *axisId;
                int index;
                int axisNum;
                double* pos;
            }*data=event->data;
            for(int i=0;i<data->axisNum;i++)
            {
                data->pos[i]=axis[data->axisId[i]].act.pos;
            }
            
            break;
        }


        case cmd_adx_Stop:
        {
            struct {
                int axisId;
                int index;
                double deceleration;
                double jerk;
                int bufferMode;
            }*data=event->data;
            double deceleration=data->deceleration;
            //double jerk=data->jerk;
            //减速度合法性检查
            if(fabs(deceleration)<0.001)//=0.0
            {
                ULOG_ERROR("Axis %d: state %s:  cmd [%d] %s error, deceleration is zero.",axis->axisId,axis->state->name,event->type,event->name );
                ret=SETPARAM_ERR ;
                return ret;
            }
            else if(deceleration<0.0)//<0.0
            {
                deceleration=axis->axisParam.acc; 
                ULOG_WARNING("Axis %d: state %s:  cmd [%d] %s warning, deceleration adjusted to %f.", axis->axisId, axis->state->name, event->type, event->name, deceleration); 
            }


            ret=setSpeedRate(&axis->stopRate, 0.0,axis->axisParam.vel,\
                deceleration,data->jerk,axis->inp.dt);
            if (ret)
            {
                ULOG_ERROR("setSpeedRate error ret=%d\n", ret);
            }
            ret=axisTransitionState(axis,&StoppingState); 
            if(ret)
            {
                ULOG_ERROR("Axis %d: axisTransitionState error ret=%d",axis->axisId,ret);
                break;
            }
            break;
        }

        case cmd_adx_ReadStatus:
        {
            struct 
            {
                int axisId;
                int index;
                int *status; //指向状态数据的指针
            }*data=event->data;
            *data->status=axis->state->status;
            break;
        }
        case cmd_adx_ReadMultiStatus:
        {
            struct 
            {
                int *axisId;
                int index;
                int axisNum;
                int *status; //指向状态数据的指针
            }*data=event->data;

            for(int i=0;i<data->axisNum;i++)
            {
                data->status[i]=axis[data->axisId[i]].state->status;
            }

            break;
        }
        case cmd_adx_RegisterSetMotorRefPosFun:
        {
            struct 
            {
                int axisId;
                int index;
                int (*setMotorRefPosFun)( unsigned short int axisId, int pos,int timeOut_us);
            }*data=event->data;
            axis->setMotorRefPos=data->setMotorRefPosFun;
            break;
        }
        case cmd_adx_RegisterGetMotorActPosFun:
        {
            struct 
            {
                int axisId;
                int index;
                void *getMotorActPosFun;
            }*data=event->data;
            (void)data; 
            break;
        }
        case cmd_adx_RegisterSetMotorEnableFun:
        {
            struct 
            {
                int axisId;
                int index;
                void *p_motorEnableFun;
            }*data=event->data;
            axis->setMotorEnable=data->p_motorEnableFun;
            break;
        }
        case cmd_adx_SetMotorEncoderScale:
        {
            struct 
            {
                int axisId;
                int index;
                int num;
                int den;
            }*data=event->data;
            axis->scale.num=data->num;
            axis->scale.den=data->den;
            break;
        }
        case cmd_adx_SetHomeParam:
        {
            struct 
            {
                int axisId;
                HomeParameter_t* param;
            }*data=event->data;
            memcpy(&axis->homeParam, data->param, sizeof(HomeParameter_t));
            break;
        }
        case cmd_adx_GoHome:
        {
            struct 
            {
                int axisId;
                int index;
                int bufferMode;
            }*data=event->data;
            (void)data;
            ret=axisTransitionState(axis,&HomingState); 
            if(ret)
            {
                ULOG_ERROR("Axis %d: axisTransitionState error ret=%d",axis->axisId,ret);
                break;
            }
            break;
        }
        default:
        ULOG_ERROR("Axis %d: state %s: undefined or unsupported cmd [%d] %s.", 
            axis->axisId, axis->state->name, event->type, event->name);
        break;
    }
    return ret;
}



int Base_onUpdate(AxisControl_t *axis, double dt) {
    
    int ret=0;
    for(int i=0;i<5;i++)
    {
        ret=axisInterpolation(axis,dt);
        if(ret)
        {
            ULOG_ERROR("Axis %d: axisInterpolation error ret=%d",axis->axisId,ret);
            return ret;
        }
    }
    return ret;
}


int Base_onRtUpdate(AxisControl_t *axis, double dt)
{
    int ret=0;
    InpTimeData_t atq;
    //获取单轴规划位置并应用倍率
    ret=getAxisTimeQueue(axis,dt,&atq);
    if(ret)
    {
        ULOG_ERROR("Axis %d: state %s: getAxisTimeQueue error [%d].",axis->axisId,axis->state->name,ret);
        return ret;
    }
    memcpy(&axis->axisCurTime,&atq,sizeof(InpTimeData_t));

    //更新驱动位置，限位，滤波，力控等
    ret=updateAxisDrivePosition(axis, dt);
    if(ret)
    {
        ULOG_ERROR("Axis %d: state %s: updateAxisDrivePosition error.",axis->axisId,axis->state->name);
        return ret;
    }

    //写入调试文件
    //logFile(axis->drvFile, "%.5f %.5f %.5f %.5f %.5f \n ", *axis->refTime, axis->drv.pos, axis->drv.vel, axis->drv.acc, axis->drv.jerk);
    //fflush(axis->drvFile);
    //获取电机状态信息
    ret=updateMotorInfo(axis);
    if(ret)
    {
        ULOG_ERROR("Axis %d: state %s: updateMotorInfo error.",axis->axisId,axis->state->name);
        return ret;
    }
    return 0;
}

const AxisBaseState_t BaseState = {
    .name = "BaseState",
    .status = AXIS_NONE,
    .onEnter = Base_onEnter,
    .onExit  = Base_onExit,
    .onUpdate = Base_onUpdate,
    .onEvent = Base_onEvent,
    .onRtUpdate=Base_onRtUpdate,
};

