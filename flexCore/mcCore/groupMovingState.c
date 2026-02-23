
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


/*==================== 组运动状态 =====================*/
int GroupMoving_onEnter(AxisGroupControl_t *group) {
    //ULOG_INFO("Group %d: Enter Moving state.", group->cfg.grpId);
        (void)group;
    return 0;
}

int GroupMoving_onExit(AxisGroupControl_t *group) {
    //ULOG_INFO("Group %d: Exit Moving state.", group->cfg.grpId);
        (void)group;
    return 0;
}




int GroupMoving_onEvent(AxisGroupControl_t *group, const FsmEvent_t *event) {
    if (!event) return 0;
    // 在组运动状态下允许接收停止事件以中断运动
    int ret=0;
    switch (event->type)
    {
    case cmd_adx_GroupDisable:
        /* code */
        break;
    case cmd_adx_GroupEnable:
        /* code */
        break;
    case cmd_adx_MoveLinearAbsolute:
    case cmd_adx_GroupWriteMultiParam: 
    case cmd_adx_SetOverride:
    case cmd_adx_GroupInterrupt:
    case cmd_adx_GroupContinue:
    case cmd_adx_GroupStop:
    case cmd_adx_GroupSetMcode:
    case cmd_adx_SetDeviceStatus:
    case cmd_adx_ReadDeviceStatus:
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
    ULOG_ERROR("Group %d: state %s: undefine or unsupport cmd[%d] %s.", 
        group->cfg.grpId, group->state->name, event->type, event->name);
        //此状态下不允许或未定义的指令
        ret=UNSUPPORTEDCMD_ERR;
        break;
    }
    return ret;
}

int GroupMoving_onUpdate(AxisGroupControl_t *group, double dt) {
    
    int ret=0;
    if(group->state && group->state->baseState && group->state->baseState->onUpdate)
    {
        //调用基类的更新函数
        ret=group->state->baseState->onUpdate(group, dt);
        if(ret)
        {
            ULOG_ERROR("Group %d: groupMoving_onUpdate error ret=%d",group->cfg.grpId,ret);
            return ret;
        }
    }
    //监护条件[运动完成]导致的状态切换
    if(group->state!=&GroupStandbyState &&isGroupStandby(group))
    {
        ret=groupTransitionState(group,&GroupStandbyState);
        if(ret)
        {
            ULOG_ERROR("Group %d: groupTransitionState error ret=%d",group->cfg.grpId,ret);
            return ret;
        }
    }

    return 0;   
}


int GroupMoving_onRtUpdate(AxisGroupControl_t* group, double dt) {
    //printf("Group %d: onRtUpdate Moving state.\n", group->cfg.grpId);
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


const AxisGroupState_t GroupMovingState = {
    .baseState=&groupBaseState,
    .name = "GRP_MOVING",
    .status= GRP_MOVING,
    .onEnter = GroupMoving_onEnter,
    .onExit  = GroupMoving_onExit,
    .onUpdate = GroupMoving_onUpdate,
    .onRtUpdate = GroupMoving_onRtUpdate,
    .onEvent = GroupMoving_onEvent,

};

