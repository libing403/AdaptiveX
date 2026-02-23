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



/*==================== 组回零状态 =====================*/
int GroupHoming_onEnter(AxisGroupControl_t *group) {
    //printf("Group %d: Enter Homing state.\n", group->cfg.grpId);
        (void)group;

    return 0;
}

int GroupHoming_onExit(AxisGroupControl_t *group) {
    //printf("Group %d: Exit Homing state.\n", group->cfg.grpId);
        (void)group;
    return 0;
}

int GroupHoming_onUpdate(AxisGroupControl_t *group, double dt) {
     
    (void)dt;
    int allIdle = 1;
    for (int i = 0; i < group->kine.axisCount; i++) {

    }
    if (allIdle) {
        printf("Group %d: All axes homed.\n", group->cfg.grpId);
        groupTransitionState(group, &GroupDisableState);
    }
    return 0;
}

int GroupHoming_onEvent(AxisGroupControl_t *group, const FsmEvent_t *event) {
    if (!event) return 0;
    int ret=0;
    switch (event->type) {

    default:
        ULOG_ERROR("Group %d: state %s: undefine or unsupport cmd %d.", group->cfg.grpId, group->state->name);
        //此状态下不允许或未定义的指令
        ret=UNSUPPORTEDCMD_ERR;
        break;
    }
    
    return ret;
}

const AxisGroupState_t GroupHomingState = {
    .baseState=&groupBaseState,
    .name = "GRP_HOMING",
    .status=GRP_HOMING,
    .onEnter = GroupHoming_onEnter,
    .onExit  = GroupHoming_onExit,
    .onUpdate = GroupHoming_onUpdate,
    .onEvent = GroupHoming_onEvent
};
