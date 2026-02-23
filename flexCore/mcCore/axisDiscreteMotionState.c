

#include <stdio.h>
#include <math.h>
#include <string.h>
//自定义头文件
#include "axisControl.h"
#include "mcTypes.h"
//第三方库
 #include"ulog.h"

/*==================== Moving 状态实现 ====================*/
int Discrete_onEnter(AxisControl_t *axis) {
(void)axis;
    //ULOG_INFO("Axis %d: Enter DisceteMotionState ",axis->axisId);         
    return 0;
}

int Discrete_onExit(AxisControl_t *axis) {
  (void)axis;
    //ULOG_INFO("Axis %d: Exit DisceteMotionState .", axis->axisId);
    return 0;
}



int Discrete_onEvent(AxisControl_t *axis, const FsmEvent_t *event) {
    if (!axis) return 0;
    if (!event) return 0;
    int ret=0;
    switch(event->type) {
        case cmd_adx_Power:
        case cmd_adx_WriteFloatParam:
        case cmd_adx_ReadFloatParam:
        case cmd_adx_MoveAbsolute:
        case cmd_adx_MoveRelative:
        case cmd_adx_SetOverride:
        case cmd_adx_MoveVelocity:
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
        case cmd_adx_Stop:
        case cmd_adx_Halt:
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
            ret=UNSUPPORTEDCMD_ERR;
            break;
    }
    return ret;
}

int Discrete_onUpdate(AxisControl_t *axis, double dt) {
    int ret=0;
    if(axis->state &&axis->state->baseState )
    {
        ret=axis->state->baseState->onUpdate(axis, dt);
        if(ret)
        {
            ULOG_ERROR("Axis %d: SyncMotion onUpdate error ret=%d", axis->axisId, ret);
            return ret;
        }
    }
    //监护条件[运动完成]导致的状态切换
    if(axis->state!=&StandstillState &&isAxisStandstill(axis))
    {
        ret=axisTransitionState(axis,&StandstillState);
        if(ret)
        {
            ULOG_ERROR("Axis %d: axisTransitionState error ret=%d",axis->axisId,ret);
            return ret;
        }
    }
    return ret;
}

int Discrete_onRtUpdate(AxisControl_t *axis, double dt)
{
    int ret=0;
    if(axis && axis->state && axis->state->baseState)
    {
        //调用基类的实时更新函数
        ret=axis->state->baseState->onRtUpdate(axis, dt);
        if(ret)
        {
            ULOG_ERROR("Axis %d: DisceteMotion onRtUpdate error ret=%d", axis->axisId, ret);
            return ret;
        }
    }

    return 0;
}

const AxisState_t DisceteMotionState = {
    .baseState = &BaseState,
    .status=AXIS_DISCRETEMOTION,
    .name = "DISCRETEMOTION",
    .onEnter = Discrete_onEnter,
    .onExit  = Discrete_onExit,
    .onUpdate = Discrete_onUpdate,
    .onEvent = Discrete_onEvent,
    .onRtUpdate=Discrete_onRtUpdate,
};

