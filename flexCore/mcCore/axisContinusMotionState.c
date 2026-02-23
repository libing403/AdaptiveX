

#include <stdio.h>
#include <math.h>
#include <string.h>
//自定义头文件
#include "axisControl.h"

#include "mcTypes.h"
//第三方库
#include"ulog.h"
 
/*==================== Moving 状态实现 ====================*/
int ContinusMotion_onEnter(AxisControl_t *axis) {
    ULOG_INFO("Axis %d: Enter ContinusMotion state.",axis->axisId);
    return 0;
}

int ContinusMotion_onExit(AxisControl_t *axis) {
    ULOG_INFO("Axis %d: Exit ContinusMotion state. ", axis->axisId);
    return 0;
}


int ContinusMotion_onEvent(AxisControl_t *axis, const FsmEvent_t *event) {
    if (!event) return 0;
    int ret=0;
    switch(event->type) {
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
        case cmd_adx_Stop:
        {
            struct {
                int axisId;
                int index;
                double deceleration;
                double jerk;
                int bufferMode;
            }*data=event->data;
            ret=setAxisVelocityPlanerTarget(axis, 0.0,data->deceleration, data->deceleration,data->jerk);
            if(ret)
            {
                ULOG_ERROR("Axis %d: state %s: cmd [%d] %s error ret=%d.",
                    axis->axisId, axis->state->name, event->type, event->name, ret);
                return ret;
            } 
            ret=axisTransitionState(axis, &StoppingState);
            if(ret)
            {
                ULOG_ERROR("Axis %d: state %s: axisTransitionState error ret=%d.",
                    axis->axisId, axis->state->name, ret);
                return ret;
            }  
            break;
        }
        default:
        ULOG_ERROR("Axis %d: state %s: undefined or unsupported cmd [%d] %s.", 
            axis->axisId, axis->state->name, event->type, event->name);
        break;
    }
    return 0;
}


 int ContinusMotion_onUpdate(AxisControl_t *axis, double dt) {

    (void)dt;
    int ret=0;
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
    return 0;
}


int ContinusMotion_onRtUpdate(AxisControl_t *axis, double dt)
{
    int ret=0;
    ret=axisVelocityInterpolation(axis, dt);
    if(ret)
    {
        ULOG_ERROR("Axis %d: state %s: axisVelocityInterpolation error ret=%d.",axis->axisId,axis->state->name,ret);
        return ret;
    }

    return 0;
}

const AxisState_t ContinusMotionState = {
    .baseState = &BaseState,
    .status=AXIS_CONTINUOUSMOTION,
    .name = "AXIS_CONTINUOUSMOTION",
    .onEnter = ContinusMotion_onEnter,
    .onExit  = ContinusMotion_onExit,
    .onUpdate = ContinusMotion_onUpdate,
    .onEvent = ContinusMotion_onEvent,
    .onRtUpdate=ContinusMotion_onRtUpdate,
};

