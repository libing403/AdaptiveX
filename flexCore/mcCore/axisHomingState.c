
#include <stdio.h>
#include <math.h>
#include <string.h>
//自定义头文件
#include "axisControl.h"

#include "mcTypes.h"
//第三方库
 #include"ulog.h"


/*==================== Homing 状态实现 ====================*/
int Homing_onEnter(AxisControl_t *axis) {
(void)axis;
    //ULOG_INFO("Axis %d: Enter Homing state.\n", axis->axisId);
 
    return 0;
}

int Homing_onExit(AxisControl_t *axis) {
(void)axis;
    //ULOG_INFO("Axis %d: Exit Homing state.\n", axis->axisId);
    return 0;
}

int Homing_onUpdate(AxisControl_t *axis, double dt) {

    (void)axis;
    (void)dt;
    return 0;
}

 int Homing_onEvent(AxisControl_t *axis, const FsmEvent_t *event) {
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


int Homing_onRtUpdate(AxisControl_t *axis, double dt)

{
    (void)axis;
    (void)dt;
    return 0;
}


const AxisState_t HomingState = {
    .name = "Homing",
    .onEnter = Homing_onEnter,
    .onExit  = Homing_onExit,
    .onUpdate = Homing_onUpdate,
    .onEvent = Homing_onEvent
};