
#include <stdio.h>
#include <math.h>
#include <string.h>
//自定义头文件
#include "axisControl.h"


#include "mcTypes.h"

//第三方库
 #include"ulog.h"

/*==================== Disable 状态实现 ====================*/
int Disable_onEnter(AxisControl_t *axis) {
(void)axis;
    //ULOG_INFO("Axis %d: Enter Disable state.", axis->axisId);
    return 0;
}

int Disable_onExit(AxisControl_t *axis) {
    //ULOG_INFO("Axis %d: Exit Disable state.", axis->axisId);
    (void)axis;
    return 0;
}

int Disable_onUpdate(AxisControl_t *axis, double dt) {
    (void)axis;
    (void)dt;



    // 空闲状态下，无更新操作
    return 0;
}

int Disable_onEvent(AxisControl_t *axis, const FsmEvent_t *event) {
    if (!event) return 0;
    int ret=0;
    switch(event->type) {
        case cmd_adx_Power:
        case cmd_adx_WriteFloatParam:
        case cmd_adx_GroupWriteIntParam:
        case cmd_adx_ReadFloatParam:
        case cmd_adx_MoveAbsolute:
        case cmd_adx_RegisterSetMotorRefPosFun:
        case cmd_adx_RegisterGetMotorActPosFun:
        case cmd_adx_RegisterSetMotorEnableFun: 
        case cmd_adx_SetMotorEncoderScale:
        case cmd_adx_SetGantryConfig:
        case cmd_adx_SetGantryEnable:
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
        case cmd_adx_MoveVelocity:
        {
            //调用基类的事件处理函数
            ret=axis->state->baseState->onEvent(axis,event);
            if(ret)
            {
                ULOG_ERROR("Aixs %d: state %s:  cmd [%d] %s error ret=%d.",
                    axis->axisId,axis->state->name,event->type,event->name, ret);
            }
            break;
        }

        default:
            break;
    }
    return ret;
}


int Disable_onRtUpdate(AxisControl_t *axis, double dt) {
(void)axis;
(void)dt;
 
    // 空闲状态下，无更新操作
    return 0;
}


const AxisState_t DisableState = {
    .baseState = &BaseState,
    .name = "DISABLED",
    .status=AXIS_DISABLED,
    .onEnter = Disable_onEnter,
    .onExit  = Disable_onExit,
    .onUpdate = Disable_onUpdate,
    .onEvent = Disable_onEvent,
    .onRtUpdate=Disable_onRtUpdate,
};
