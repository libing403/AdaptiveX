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



/*==================== 组关闭状态 =====================*/
int GroupDisable_onEnter(AxisGroupControl_t *group) {
  (void)group;
    //ULOG_INFO("Group %d: Enter Disable state.", group->cfg.grpId);
    
    return 0;
}
int GroupDisable_onExit(AxisGroupControl_t *group) {
    //ULOG_INFO("Group %d: Exit Disable state.", group->cfg.grpId);
    (void)group;
 
    return 0;
}

int GroupDisable_onUpdate(AxisGroupControl_t *group, double dt) {
        (void)group;
    (void)dt;   
    //ULOG_DEBUG("Group %d: onUpdate Disable state.", group->cfg.grpId);

    return 0;
}

int GroupDisable_onEvent(AxisGroupControl_t *group, const FsmEvent_t *event) {
    if (!event) return 0;
    int ret=0;
    switch(event->type) {
        case cmd_adx_GroupDisable: {

            ret=groupTransitionState(group, &GroupDisableState);
            break;
        }
        case cmd_adx_GroupEnable: {

            ret=groupTransitionState(group, &GroupStandbyState);
            break;
        }
        case cmd_adx_AddAxisToGroup: 
        case cmd_adx_CreateAxisGroup:
        {
            if(group->state->baseState->onEvent)
            {
                //调用基类的事件处理函数
                ret=group->state->baseState->onEvent(group, event);
                if(ret)
                {
                ULOG_ERROR("Group %d: state %s:  cmd [%d] %s error ret=%d.",
                    group->cfg.grpId,group->state->name,event->type,event->name, ret);
                }
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

int GroupDisable_onRtUpdate(AxisGroupControl_t *group, double dt) {
    (void)group;
    (void)dt;    

    return 0;
}
 const AxisGroupState_t GroupDisableState = {
    .baseState=&groupBaseState,
    .name = "GRP_DISABLE",
    .status=GRP_DISABLE,
    .onEnter = GroupDisable_onEnter,
    .onExit  = GroupDisable_onExit,
    .onUpdate = GroupDisable_onUpdate,
    .onEvent = GroupDisable_onEvent,
    .onRtUpdate=GroupDisable_onRtUpdate,
};