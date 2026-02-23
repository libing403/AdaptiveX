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

 


/*==================== 组错误状态 =====================*/
int GroupErrorStop_onEnter(AxisGroupControl_t *group) {
    // printf("Group %d: Enter Error state!\n", group->groupId);
        (void)group;
    return 0;
}

int GroupErrorStop_onExit(AxisGroupControl_t *group) {
    // printf("Group %d: Exit Error state.\n", group->groupId);
        (void)group;
    return 0;
}

int GroupErrorStop_onUpdate(AxisGroupControl_t *group, double dt) {
    (void)group;
    (void)dt;
    // 错误状态下可尝试错误恢复
    return 0;
}

int GroupErrorStop_onEvent(AxisGroupControl_t *group, const FsmEvent_t *event) {
    (void)group;
    (void)event;
    // 错误状态下不处理事件
    return 0;
}

const AxisGroupState_t GroupErrorStopState = {
    .baseState=&groupBaseState,
    .name = "GRP_ERRORSTOP",
    .status=GRP_ERRORSTOP,
    .onEnter = GroupErrorStop_onEnter,
    .onExit  = GroupErrorStop_onExit,
    .onUpdate = GroupErrorStop_onUpdate,
    .onEvent = GroupErrorStop_onEvent
};