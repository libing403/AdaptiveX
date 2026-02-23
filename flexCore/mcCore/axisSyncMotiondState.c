

#include <stdio.h>
#include <math.h>
#include <string.h>
//自定义头文件
#include "axisControl.h"

#include "mcTypes.h"
//第三方库
 #include"ulog.h"

/*==================== Moving 状态实现 ====================*/
int SyncMotion_onEnter(AxisControl_t *axis) {
  (void)axis;
    //ULOG_INFO("Axis %d: Enter SyncMotion state.",axis->axisId);
    return 0;
}

int SyncMotion_onExit(AxisControl_t *axis) {
  (void)axis;
    //ULOG_INFO("Axis %d: Exit SyncMotion state.", axis->axisId);
    return 0;
}



int SyncMotion_onEvent(AxisControl_t *axis, const FsmEvent_t *event) {
    if (!event) return 0;
    int ret=0;
    switch(event->type) {
        case cmd_adx_Power:
        case cmd_adx_WriteFloatParam:
        case cmd_adx_ReadFloatParam:
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
            ret=axis->state->baseState->onEvent(axis,event);
            if(ret)
            {
                ULOG_ERROR("Aixs %d: state %s:  cmd [%d] %s error ret=%d.",
                    axis->axisId,axis->state->name,event->type,event->name, ret);
            }
            break;
        }
 
        default:
             ULOG_ERROR("aixs %d: state %s, undefine or unsupport cmd [%d] %s.", 
            axis->axisId, axis->state->name, event->type, event->name);
            break;
    }
    return 0;
}

 int SyncMotion_onUpdate(AxisControl_t *axis, double dt) {

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

    return 0;
}


int SyncMotion_onRtUpdate(AxisControl_t *axis, double dt)
{
    int ret=0;
    if(axis->cfg.axisType==AXIS_TYPE_GANTRY_SLAVE && axis->gantryCtrl->isCoupled)
    {

        ret=gantryMasterSlaveControl(axis->gantryCtrl, dt);
        if(ret)
        {
            ULOG_ERROR("Axis %d: gantryMasterSlaveControl error ret=%d",axis->axisId,ret);
            return ret;
        }
    }


    return 0;
}

const AxisState_t SyncMotionState = {
    .baseState = &BaseState,
    .status=AXIS_SYNCMOTION,
    .name = "SYNCMOTION",
    .onEnter = SyncMotion_onEnter,
    .onExit  = SyncMotion_onExit,
    .onUpdate = SyncMotion_onUpdate,
    .onEvent = SyncMotion_onEvent,
    .onRtUpdate=SyncMotion_onRtUpdate,
};

