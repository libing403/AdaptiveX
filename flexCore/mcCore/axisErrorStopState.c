

#include <stdio.h>
#include <math.h>
#include <string.h>
//自定义头文件
#include "axisControl.h"

 
#include "mcTypes.h"
 
//第三方库
 #include"ulog.h"

/*==================== Error 状态实现 ====================*/
int Error_onEnter(AxisControl_t *axis) {
  (void)axis;
    //ULOG_INFO("Axis %d: Enter Error state!\n", axis->axisId);
    return 0;
}

 int Error_onExit(AxisControl_t *axis) {
   (void)axis;
   // ULOG_INFO("Axis %d: Exit Error state.\n", axis->axisId);
    return 0;
}

 int Error_onUpdate(AxisControl_t *axis, double dt) {
    (void)axis;
    (void)dt;
    // 错误状态下可以实现错误恢复逻辑
    return 0;
}

 int Error_onEvent(AxisControl_t *axis, const FsmEvent_t *event) {
    if (!axis) return 0;
    if (!event) return 0;
    int ret=0;
    switch(event->type) {
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
            ret=UNSUPPORTEDCMD_ERR;
            break;

    }

    return ret;
}
int Error_onRtUpdate(AxisControl_t *axis, double dt) {
    (void)axis;
    (void)dt;
    // 错误状态下可以实现错误恢复逻辑
    return 0;
}



const AxisState_t ErrorStopState = {
    .baseState = &BaseState,
    .status = AXIS_ERRORSTOP,
    .name = "ErrorStop",
    .onEnter = Error_onEnter,
    .onExit  = Error_onExit,
    .onEvent = Error_onEvent,
    .onUpdate = Error_onUpdate,
    .onRtUpdate = Error_onRtUpdate,
};