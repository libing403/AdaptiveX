

#include <stdio.h>
#include <math.h>
#include <string.h>
//自定义头文件
#include "axisControl.h"

#include "mcTypes.h"
//第三方库
 #include"ulog.h"

/*==================== Standstill 状态实现 ====================*/
int Standstill_onEnter(AxisControl_t *axis)
{
  (void)axis;
    //ULOG_INFO("Axis %d: Enter Standstill state.", axis->axisId);
    return 0;
}

int Standstill_onExit(AxisControl_t *axis)
{
  (void)axis;
    //ULOG_INFO("Axis %d: Exit Standstill state.", axis->axisId);
    return 0;
}





int Standstill_onEvent(AxisControl_t *axis,const FsmEvent_t* event)
{
    if (!event) return 0;
    int ret=0;
    switch(event->type) {
        case cmd_adx_Power:
        case cmd_adx_WriteFloatParam:
        case cmd_adx_GroupWriteIntParam:
        case cmd_adx_ReadFloatParam:
        case cmd_adx_MoveAbsolute:
        case cmd_adx_SetGantryConfig: //龙门配置只有静止才能使用。
        case cmd_adx_SetGantryEnable: //龙门配置只有静止才能使用。
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
        case cmd_adx_ReadStatus:
        case cmd_adx_ReadMultiActualPos:
        case cmd_adx_ReadMultiLogicalPos:
        case cmd_adx_ReadMultiStatus:
        case cmd_adx_MoveVelocity:
        case cmd_adx_RegisterSetMotorRefPosFun:
        case cmd_adx_RegisterGetMotorActPosFun:
        case cmd_adx_RegisterSetMotorEnableFun:   
        case cmd_adx_SetMotorEncoderScale:    
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
             ULOG_ERROR("Aixs %d: state %s, undefine or unsupport cmd [%d] %s.", 
            axis->axisId, axis->state->name, event->type, event->name);
            break;
    }
    return ret;

}

int Standstill_onUpdate(AxisControl_t *axis,double dt)
{
    int ret=0;
    if(axis->state &&axis->state->baseState )
    {
        ret=axis->state->baseState->onUpdate(axis,dt);
    }
    return ret;
}

int Standstill_onRtUpdate(AxisControl_t *axis,double dt)
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

const AxisState_t StandstillState = {
    .baseState = &BaseState,
    .name = "Standstill",
    .status=AXIS_STANSTILL,
    .onEnter = Standstill_onEnter,
    .onExit  = Standstill_onExit,
    .onEvent = Standstill_onEvent,
    .onUpdate = Standstill_onUpdate,
    .onRtUpdate=Standstill_onRtUpdate,
};


