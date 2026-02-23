
#include <stdio.h>
#include <math.h>
#include <string.h>
//自定义头文件
#include "axisControl.h"

#include "mcTypes.h"
//第三方库
 #include"ulog.h"


/*==================== Stopping 状态实现 ====================*/
int Stopping_onEnter(AxisControl_t *axis) {
    //ULOG_INFO("Axis %d: Enter Stopping state.", axis->axisId);
     (void)axis;

    return 0;
}

int Stopping_onExit(AxisControl_t *axis) {
    (void)axis;

    //ULOG_INFO("Axis %d: Exit Stopping state.", axis->axisId);
    return 0;
}



 int Stopping_onEvent(AxisControl_t *axis, const FsmEvent_t *event) {
    if (!event) return 0;
    int ret=0;
    switch(event->type) {
        case cmd_adx_Power:
        case cmd_adx_WriteFloatParam:
        case cmd_adx_ReadFloatParam:
        case cmd_adx_ReadIntParam:
        case cmd_adx_GroupWriteIntParam:
        case cmd_adx_Stop:
        case cmd_adx_ReadTargetPos:
        case cmd_adx_ReadTargetVel:
        case cmd_adx_ReadTargetAcc:
        case cmd_adx_ReadTargetJerk:
        case cmd_adx_ReadLogicalPos:
        case cmd_adx_ReadLogicalVel:
        case cmd_adx_ReadLogicalAcc:
        case cmd_adx_ReadLogicalJerk:
        case cmd_adx_ReadActualPos:
        case cmd_adx_ReadActualVel:
        case cmd_adx_ReadActualAcc:
        case cmd_adx_ReadActualJerk:
        case cmd_adx_ReadMultiActualPos:
        case cmd_adx_ReadMultiLogicalPos:
        case cmd_adx_ReadMultiStatus:

        case cmd_adx_ReadStatus:
        {
            ret=axis->state->baseState->onEvent(axis,event);
            if(ret)
            {
                ULOG_ERROR("Aixs %d: state %s:  cmd [%d] %s error ret=%d.",
                    axis->axisId,axis->state->name,event->type,event->name, ret);
            }
            break;
        }
       
        default:
             ULOG_ERROR("Aixs %d: state %s, undefine or unsupport cmd [%d] %s.", 
            axis->axisId, axis->state->name, event->type, event->name);
            break;
    }
    return ret;

}

int Stopping_onUpdate(AxisControl_t *axis, double dt) {
    int ret=0;
    static int waitCount=0;
    switch(axis->preState->status) {
        case AXIS_STANSTILL:
        // 处理静止状态下的更新

            break;
        case AXIS_DISCRETEMOTION:
        // 处理离散运动状态下的更新
        {
            if(axis->state &&axis->state->baseState )
            {
                ret=axis->state->baseState->onUpdate(axis,dt);//调用基类的更新函数，可能正常插补停止
                if(ret)
                {
                    ULOG_ERROR("Axis %d: state %s: baseState->onUpdate error ret=%d.",axis->axisId,axis->state->name,ret);
                    return ret;
                }
            }
            //倍率为0.0时，也是停止完成
            if(fabs(axis->stopRate.currentRate) < 1.0e-5 && axis->state!=&StandstillState &&waitCount++ >3)
            {
                waitCount=0;
                //监护条件[运动完成]导致的状态切换             
                ret=axisTransitionState(axis,&StandstillState);
                if(ret)
                {
                    ULOG_ERROR("Axis %d: axisTransitionState error ret=%d",axis->axisId,ret);
                    return ret;
                }
                ClearQueueISR(&(axis->trajectoryQ)); //清除轨迹队列
                ClearQueueISR(&(axis->axisTimeQ)); //清除时间队列
                axis->inp.state=1; //插补器状态设置为空闲
                axis->stopRate.currentRate=1.0; //停止倍率归一
                axis->stopRate.targetRate=1.0; //目标倍率归一

                axis->inp.curPos=axis->cmd.pos; //当前指令位置同步
                axis->inp.prePos=axis->cmd.pos; //前一位置同步
                 

                axis->inp.preVelocity=0.0; //前一速度归零
                axis->inp.preAcceleration=0.0; //前一加速度归零
                axis->inp.preJerk=0.0; //前一加加速度归零
                axis->inp.curVelocity=0.0; //当前速度归零
                axis->inp.curAcceleration=0.0; //当前加速度归零

                axis->inp.otp.state=1; //离散插补器状态设置为空闲
                axis->inp.otp.dqtNext=0.0; //离散插补器前一速度归零
                axis->inp.otp.ddqtNext=0.0; //离散插补器前一加速度归零

                axis->inp.otp.param.dqLast=0.0; //离散插补器前一速度归零
                axis->inp.otp.param.ddqLast=0.0; //离散插补器前一加速度归零
                
                axis->inp.osp.state=1; //连续插补器状态设置为空闲
                axis->inp.osp.dqtNext=0.0; //连续插补器前一速度归零
                axis->inp.osp.ddqtNext=0.0; //连续插补器前一加速度归零
                
                axis->inp.osp.param.dqLast=0.0; //连续插补器前一速度归零
                axis->inp.osp.param.ddqLast=0.0; //连续插补器前一加速度归零
                
            }
           break; 
        }
            
        case AXIS_CONTINUOUSMOTION:
            // 处理连续运动状态下的更新
            {
                if(fabs(axis->inp.curVelocity) < 1.0e-5 && waitCount++ > 3)
                {
                    waitCount=0;
                    //监护条件[运动完成]导致的状态切换
                    ret=axisTransitionState(axis,&StandstillState);
                    if(ret)
                    {
                        ULOG_ERROR("Axis %d: axisTransitionState error ret=%d",axis->axisId,ret);
                        return ret;
                    }                   
                    axis->inp.state=1; //插补器状态设置为空闲
                }
                
            }
            break;
        case AXIS_STOPPING:
            // 处理停止状态下的更新
            break;
        default:
            break;
    }
    return 0;
}

int Stopping_onRtUpdate(AxisControl_t *axis, double dt)
{
  (void)dt;
    int ret=0;
    switch(axis->preState->status) {
        case AXIS_STANSTILL:
            // 处理静止状态下的更新

            break;
        case AXIS_DISCRETEMOTION:
        // 处理离散运动状态下的更新
        if(axis && axis->state && axis->state->baseState)
        {
            //调用基类的实时更新函数
            axis->state->baseState->onRtUpdate(axis, dt);
            
        }
        break;
        case AXIS_CONTINUOUSMOTION:
        {
            ret=axisVelocityInterpolation(axis, dt);
            if(ret)
            {
                ULOG_ERROR("Axis %d: state %s: axisVelocityInterpolation error ret=%d.",axis->axisId,axis->state->name,ret);
                return ret;
            }
            break;
        }

        case AXIS_STOPPING:
            // 处理停止状态下的更新
            break;
        default:
            ULOG_ERROR("Axis %d: state %s: undefine or unsupport status [%d].", 
                axis->axisId, axis->state->name, axis->preState->status);
            break;
    }


    return 0;
}

const AxisState_t StoppingState = {
    .baseState = &BaseState,
    .name = "STOPPING",
    .status=AXIS_STOPPING,
    .onEnter = Stopping_onEnter,
    .onExit  = Stopping_onExit,
    .onUpdate = Stopping_onUpdate,
    .onRtUpdate = Stopping_onRtUpdate,
    .onEvent = Stopping_onEvent
};