
#include <stdio.h>
#include <string.h>
#include<math.h>
//自定义头文件
#include "axisControl.h"   
#include "groupControl.h"
#include "interpolation.h"
#include"mcTypes.h"
#include "Queue.h"
#include "ulog.h"




/*==================== 组正在停止状态 =====================*/
int GroupStopping_onEnter(AxisGroupControl_t *group) {
    // printf("Group %d: Enter Stopping state!\n", group->groupId);
    (void)group;
    return 0;
}

int GroupStopping_onExit(AxisGroupControl_t *group) {
    // printf("Group %d: Exit Stopping state.\n", group->groupId);
    (void)group;
    return 0;
}


int GroupStopping_onEvent(AxisGroupControl_t *group, const FsmEvent_t *event) {

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
    case cmd_adx_GroupWriteMultiParam: 
    {
 
        ret=group->state->baseState->onEvent(group, event);
        if(ret)
        {
            ULOG_ERROR("Group %d: state %s:  cmd [%d] %s error ret=%d.",
            group->cfg.grpId,group->state->name,event->type,event->name, ret);
        }
        break;
    }
    case cmd_adx_MoveLinearAbsolute:
    case cmd_adx_GroupInterrupt:
    case cmd_adx_GroupContinue:
    case cmd_adx_SetOverride:
    case cmd_adx_GroupStop:
    default:
    ULOG_ERROR("Group %d: state %s: undefine or unsupport cmd[%d] %s.", 
        group->cfg.grpId, group->state->name, event->type, event->name);
        //此状态下不允许或未定义的指令
        ret=UNSUPPORTEDCMD_ERR;
        break;
    }
    return ret;

}

int GroupStopping_onUpdate(AxisGroupControl_t *group, double dt) {
    //int ret=0;
    static int waitCount=0;
    switch(group->preState->status) {

        case GRP_DISABLE:
            // 处理禁用状态下的更新
            break;
        case GRP_STANDBY:
            // 处理待机状态下的更新
            break;

        case GRP_MOVING:
        {
            // 处理静止状态下的更新
            int ret=group->state->baseState->onUpdate(group, dt);
            if(ret)
            {
                ULOG_ERROR("Group %d: GroupStopping_onUpdate error ret=%d",group->cfg.grpId,ret);
                return ret;
            }
            //倍率为0.0切换到是停止状态
            if(fabs(group->stopRate.currentRate)<1.0e-5 && group->state!=&GroupStandbyState && waitCount++>3)
            {
                waitCount=0;
                group->inp.state=0; //插补器状态设置为空闲
                group->inp.preLi=0.0;
                group->inp.li=0.0;
                group->inp.ti=0.0;
                group->inp.initTime=0.0; //插补器初始时间归零
                group->inp.linearVelocity=0.0; //线速度归零
                group->inp.linearAcceleration=0.0; //线加速度归零
                group->inp.linearJerk=0.0; //线加加速度归零
                
                group->smoothIndex=0; //前瞻插补索引归零
                group->planIndex=0; //速度规划索引归零
                group->inpIndex=0; //插补器索引归零
                group->smoothIndex=0; //前瞻插补索引归零
                group->endTime=group->inp.time; //路径终点时刻同步
                group->stopRate.currentRate=1.0; //停止倍率归一
                group->stopRate.targetRate=1.0; //目标倍率归一
                ClearQueueISR(&(group->trajectoryQ)); //清除轨迹队列
                for(int i=0;i<group->kine.coordDim;i++)
                {
                    
                    group->curInpTraj.pos[i]=group->inp.curPos[i]; //同步当前指令位置
                    group->curInpTraj.startPos[i]=group->inp.curPos[i]; //同步当前指令位置
                    group->inp.prePos[i]=group->inp.curPos[i]; //同步当前指令位置
                    group->inp.startPos[i]=group->inp.curPos[i]; //同步当前指令位置
                    group->inp.endPos[i]=group->inp.curPos[i]; //同步当前指令位置
                }
                //监护条件[停止完成]导致的状态切换
                ret=groupTransitionState(group,&GroupStandbyState);
                if(ret)
                {
                    ULOG_ERROR("groupTransitionState error ret=%d",ret);
                    return ret;
                }

                
            }
            break;

        }

        case GRP_HOMING:
            // 处理回原点状态下的更新
            break;
        case GRP_STOPPING:
            // 处理停止状态下的更新
            break;

        case GRP_ERRORSTOP:
            // 处理错误停止状态下的更新
            break;    
        default:
            // 处理其他状态下的更新
            ULOG_ERROR("Group %d: state %s: undefine or unsupport status [%d].", 
                group->cfg.grpId, group->state->name, group->preState->status);
            break;
    }
    return 0;
 
}
int GroupStopping_onRtUpdate(AxisGroupControl_t *group, double dt) {

    int ret=group->state->baseState->onRtUpdate(group, dt);
    if(ret)
    {
        ULOG_ERROR("Group %d: GroupStopping_onRtUpdate error ret=%d",group->cfg.grpId,ret);
        return ret;
    }

    return 0;
}

const AxisGroupState_t GroupStoppingState = {
    .baseState=&groupBaseState,
    .name = "GRP_STOPPING",
    .status=GRP_STOPPING,
    .onEnter = GroupStopping_onEnter,
    .onExit  = GroupStopping_onExit,
    .onEvent = GroupStopping_onEvent,
    .onUpdate = GroupStopping_onUpdate,
    .onRtUpdate = GroupStopping_onRtUpdate,
};
