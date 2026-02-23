#include <stdio.h>
#include <string.h>
#include<math.h>
//自定义头文件
#include "axisControl.h"  // 为 adx_HandleAxisEvent()、adx_IsAxisIdle() 提供接口
#include "groupControl.h"
#include "interpolation.h"
#include"mcTypes.h"
#include "Queue.h"
#include "ulog.h"

 

/*==================== 组静止状态 =====================*/
int GroupStandby_onEnter(AxisGroupControl_t *group) {
    //ULOG_INFO("Group %d: Enter GroupStandby state.", group->cfg.grpId);
        (void)group;
    return 0;
}
int GroupStandby_onExit(AxisGroupControl_t *group) {
    //ULOG_INFO("Group %d: Exit GroupStandby state.", group->cfg.grpId);
    (void)group;
    return 0;
}



int GroupStandby_onEvent(AxisGroupControl_t *group, const FsmEvent_t *event) {
    if (!event) return 0;
    int ret=0;
    switch(event->type) {
        case cmd_adx_GroupDisable: 
           
            ret=groupTransitionState(group, &GroupDisableState);
            
            break;
        
        case cmd_adx_GroupEnable: 
            
            ret=groupTransitionState(group, &GroupStandbyState);
            break;
        case cmd_adx_GroupWriteMultiParam: 
        case cmd_adx_SetKinematics:
        case cmd_adx_MoveLinearAbsolute: 
        case cmd_adx_SetOverride:
        case cmd_adx_GroupInterrupt:
        case cmd_adx_GroupContinue:
        case cmd_adx_GroupStop:
        {
            ret=group->state->baseState->onEvent(group, event);
            if(ret)
            {
                ULOG_ERROR("Group %d: state %s:  cmd [%d] %s error ret=%d.",
                group->cfg.grpId,group->state->name,event->type,event->name, ret);
            }
            break;
        }

        default:
        ULOG_ERROR("Group %d: state %s: undefine or unsupport cmd [%d] %s.", 
            group->cfg.grpId, group->state->name, event->type, event->name);
            //此状态下不允许或未定义的指令
            ret=UNSUPPORTEDCMD_ERR;
            break;
    }
    return ret;
}

int GroupStandby_onUpdate(AxisGroupControl_t *group, double dt) {
    (void)group;
    (void)dt;   

    //ULOG_DEBUG("Group %d: onUpdate GroupStandby state.", group->cfg.grpId);
    return 0;
}

int GroupStandby_onRtUpdate(AxisGroupControl_t *group, double dt) {
    int ret=0;
    if(group->state && group->state->baseState && group->state->baseState->onRtUpdate)
    {
        //调用基类的实时更新函数
        ret=group->state->baseState->onRtUpdate(group, dt);
        if(ret)
        {
            ULOG_ERROR("Group %d: groupMoving_onRtUpdate error ret=%d",group->cfg.grpId,ret);
            return ret;
        }
    }

    return 0;
}

 const AxisGroupState_t GroupStandbyState = {
    .baseState=&groupBaseState,
    .name = "GRP_STANDBY",
    .status=GRP_STANDBY,
    .onEnter = GroupStandby_onEnter,
    .onExit  = GroupStandby_onExit,
    .onUpdate = GroupStandby_onUpdate,
    .onEvent = GroupStandby_onEvent,
    .onRtUpdate=GroupStandby_onRtUpdate,
};
