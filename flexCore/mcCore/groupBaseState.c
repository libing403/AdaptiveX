
#include <stdio.h>
#include <string.h>
#include<math.h>
//自定义头文件
#include "axisControl.h"  
#include "groupControl.h"
#include "interpolation.h"
#include"mcTypes.h"
#include "Queue.h"
#include "onlinetrapeprofile.h"
#include "flexCore_if.h"
#include "ulog.h"

int writeGrpParam_Handle(AxisGroupControl_t *grp, const FsmEvent_t *event);
int readGrpParam_Handle(AxisGroupControl_t *grp, const FsmEvent_t *event);
/*==================== 组运动状态 =====================*/
int groupBaseState_onEnter(AxisGroupControl_t *group) {
    //ULOG_DEBUG("Group %d: Enter Moving state.", group->cfg.grpId);
    (void)group;
    return 0;
}

int groupBaseState_onExit(AxisGroupControl_t *group) {
    //ULOG_DEBUG("Group %d: Exit Moving state.", group->cfg.grpId);
    (void)group;
    return 0;
}



int groupBaseState_onEvent(AxisGroupControl_t *group, const FsmEvent_t *event) {
    if (!event) return 0;
    int ret=0;
    switch(event->type) {
        case cmd_adx_GroupDisable: 
           
            break;
        
        case cmd_adx_GroupEnable: 
            
            ret=groupTransitionState(group, &GroupStandbyState);
            break;
        case cmd_adx_AddAxisToGroup: 
        {
            struct {
                int groupId;
                int axisId;

            } *data =  event->data;
            int index = group->kine.axisCount;
            //group->kine.axisId[index] = evdata->axisId;
            group->kine.axisCount++;
            group->kine.axis[index]=&_adx_AX[data->axisId];
            //状态不变
            ret=groupTransitionState(group, group->state);
            break;
        }
        case cmd_adx_CreateAxisGroup:
        {
            struct {
                int grpId;
                int coordNum;
                int axisNum;
                int *axisId;
            } *evdata =  event->data;
            group->kine.axisCount=evdata->axisNum;
            group->kine.coordDim=evdata->coordNum;
            for(int i=0;i<evdata->axisNum;i++)
            {
                //group->kine.axisId[i]=evdata->axisId[i];
                group->kine.axis[i]=&_adx_AX[evdata->axisId[i]];
            }
            group->cfg.grpEnable=1;
            break;
        }
        case cmd_adx_MoveLinearAbsolute: 
        {
            struct 
            {
            int grpId;
            int index;
            double *pos;
            double velocity;
            double acc;
            double dec;
            double jerk;
            int direction;
            int coordSys;
            int bufferMode;
            int transitionMode;
            double *transitionParameter;  
            }*data =  event->data;            
            CmdTrajectory_t cmd={0};
            cmd.cmdNo=event->type;
            cmd.index=data->index;
            cmd.pathType=1;
            cmd.state=0;
            cmd.syncCnt=group->enSyncCnt+1;
            for(int i=0;i<group->kine.coordDim;i++)
            {
                cmd.pos[i]=data->pos[i];
                cmd.endPose[i]=data->pos[i];
                cmd.startPose[i]=group->preEnqTraj.endPose[i];
            }
            cmd.vs=group->usrParam.vs;
            cmd.ve=group->usrParam.ve;
            for(int i=0;i<group->kine.coordDim;i++)
            {
                cmd.vimax[i]=data->velocity;

            }
            cmd.vel=data->velocity;
            cmd.acc=data->acc;
            cmd.dec=data->dec;
            cmd.jerk=data->jerk; 
            cmd.bufferMode=data->bufferMode;
            cmd.dim=group->kine.coordDim;

            cmd.syncCnt=group->enSyncCnt+1;
            ret=EnQueue(&group->trajectoryQ,&cmd);
            if(ret)
            {
                ULOG_ERROR("Group %d: EnQueue failed.", group->cfg.grpId);
                break;
            }
            group->enSyncCnt++;
            ret=groupTransitionState(group, &GroupMovingState);
            if(ret)
            {
                ULOG_ERROR("Group %d: Transition to GroupMovingState failed.", group->cfg.grpId);
                break;
            }
         
            ret=trajectoryPretreatment(group);
            if(ret)
            {
                ULOG_ERROR("trajectoryPretreatment error ret=%d",ret);
                return ret;
            }
            lookAhead(group);
            memcpy(&group->preEnqTraj,&cmd,sizeof(CmdTrajectory_t));


            break;
        }

        case cmd_adx_SetKinematics:
        {
            struct{
                int grpId;
                int index;
                void *p_inverse;
                void *p_forward;
            }*data = event->data;
            group->kine.pInverseKin = data->p_inverse;
            group->kine.pForwardKin = data->p_forward;

            //无需状态转换
            break;
        }

        case cmd_adx_GroupWriteMultiParam:
        {
            ret=writeGrpParam_Handle(group, event);
            if (ret)
            {
                ULOG_ERROR("writeGrpParam_Handle error ret=%d\n", ret);
                return ret;
            }
            //ret=setSpeedRate(&group->speedRate,data->speedRate,group->usrParam.velMax,group->usrParam.accMax,0.001);      
            break;
        }
        case cmd_adx_GroupReadFloatParam:
        {
            struct{
                int grpId;
                int index;
                double paramId;
                double *value;
            }*data = event->data;
            //*(data->speedRate)=getSpeedRate(&group->speedRate);
            (void)data;
            ret=readGrpParam_Handle(group, event);
            break;
        }


        case cmd_adx_GroupReadMultiParam:
        {

            break;
        }
        case cmd_adx_SetOverride:
        {
            struct{
                int grpId;
                int index;
                double velFactor;
                double accFactor;
                double jerkFactor;
                int bufferMode;
            }*data = event->data;
            ret=setSpeedRate(&group->speedRate,data->velFactor,group->usrParam.velMax,\
                group->usrParam.accMax,group->usrParam.jerkMax,group->inp.dt);
            if(ret)
            {
                ULOG_ERROR("setSpeedRate error ret=%d\n", ret);
                return ret;
            }
            break;
        }
        case cmd_adx_GroupInterrupt:
        {
            struct{
                int grpId;
                int index;
                double deceleration;
                double jerk;
                int bufferMode;
            }*data = event->data;
            //中断运动
            ret=setSpeedRate(&group->speedRate,0.0,group->usrParam.velMax,\
                data->deceleration,data->jerk,group->inp.dt);
            if(ret)
            {
                ULOG_ERROR("setSpeedRate error ret=%d\n", ret);
                return ret;
            }
 
            break;
        }

        case cmd_adx_GroupContinue:
        {
            struct{
                int grpId;
                int index;
                double acceleration;
                double jerk;
                int bufferMode;
            }*data = event->data;
            //继续运动
            ret=setSpeedRate(&group->speedRate,1.0,group->usrParam.velMax,\
                data->acceleration,data->jerk,group->inp.dt);
            if(ret)
            {
                ULOG_ERROR("setSpeedRate error ret=%d\n", ret);
                return ret;
            }

            break;
        }

        case cmd_adx_GroupStop:
        {
            struct{
                int grpId;
                int index;
                double deceleration;
                double jerk;
                int bufferMode;
            }*data = event->data;
            //停止运动
            ret=setSpeedRate(&group->stopRate,0.0,group->usrParam.velMax,\
                data->deceleration,data->jerk,group->inp.dt);
            if(ret)
            {
                ULOG_ERROR("setSpeedRate error ret=%d\n", ret);
                return ret;
            }
            ret=groupTransitionState(group,&GroupStoppingState); 
            if(ret)
            {
                ULOG_ERROR("Group %d: groupTransitionState error ret=%d",group->cfg.grpId,ret);
                return ret;
            }
            break;
        }

        case cmd_adx_GroupSetMcode:
        {
            struct{
                int grpId;
                int index;
                int *forwardValAddr;
                int *reverseValAddr;
                int forwardValue;
                int reverseValue;
                int bufferMode;
            }*data = event->data;
           
            if(data->bufferMode==0) //立即执行模式
            {

                //立即执行
                FifoEvent_t evt;

                evt.cmdNo=event->type;
                evt.index=data->index;
                evt.forwardValAddr=data->forwardValAddr;
                evt.reverseValAddr=data->reverseValAddr;
                evt.forwardValue=data->forwardValue;
                evt.reverseValue=data->reverseValue;
                ret=executeEvent(group,&evt);
                if(ret)
                {
                    ULOG_ERROR("Group %d: executeEvent failed.", group->cfg.grpId);
                    break;
                }

            }
            else if(data->bufferMode==1)//缓存模式
            {

                //加入到指令队列
                FifoEvent_t evt;
                evt.syncCnt=group->enSyncCnt+1;
                evt.cmdNo=event->type;
                evt.index=data->index;
                evt.forwardValAddr=data->forwardValAddr;
                evt.reverseValAddr=data->reverseValAddr;
                evt.forwardValue=data->forwardValue;
                evt.reverseValue=data->reverseValue;
                ret=EnQueue(&group->eventQ,&evt);
                if(ret)
                {
                    ULOG_ERROR("Group %d: EnQueue eventQ failed.", group->cfg.grpId);
                    break;
                }
                group->enSyncCnt++;
                
            }
            else
            {
                ULOG_ERROR("Group %d: Invalid bufferMode %d.", group->cfg.grpId,data->bufferMode);
                ret=SETPARAM_ERR;
                break;
            }
            
            break;
        }
        case cmd_adx_RegisterSetDeviceFun:
        {
            struct{
                int grpId;
                int index;
                int devId;
                void *pSetDeviceFun;
            }*data = event->data;
            group->setDeviceStatus[data->devId] = data->pSetDeviceFun;
            break;
        }
        case cmd_adx_RegisterReadDeviceFun:
        {
            struct{
                int grpId;
                int index;
                int devId;
                void *pReadDeviceFun;
            }*data = event->data;
            group->readDeviceStatus[data->devId] = data->pReadDeviceFun;
            break;
        }
        case cmd_adx_SetDeviceStatus:
        {
            struct{
                int grpId;
                int index;
                int devId;
                int forwardStatus;
                int reverseStatus;
                int direction;
                int bufferMode;
            }*data = event->data;
            if(data->bufferMode==0) //缓存模式
            {
                //加入到指令队列
                FifoEvent_t evt;
                evt.syncCnt=group->enSyncCnt+1;
                evt.cmdNo=event->type;
                evt.index=data->index;
                evt.deviceId=data->devId;
                evt.forwardValue=data->forwardStatus;
                evt.reverseValue=data->reverseStatus;
                evt.direction=data->direction;
                int ret=EnQueue(&group->eventQ,&evt);
                if(ret)
                {
                    ULOG_ERROR("Group %d: EnQueue eventQ failed.", group->cfg.grpId);
                    break;
                }
                group->enSyncCnt++;
            }
            else if(data->bufferMode==1)//立即执行模式
            {
                //立即执行
                FifoEvent_t evt;
                evt.cmdNo=event->type;
                evt.index=data->index;
                evt.deviceId=data->devId;
                evt.forwardValue=data->forwardStatus;
                evt.reverseValue=data->reverseStatus;
                evt.direction=data->direction;
                ret=executeEvent(group,&evt);
                if(ret)
                {
                    ULOG_ERROR("Group %d: executeEvent failed.", group->cfg.grpId);
                    break;
                }
            }
            else
            {
                ULOG_ERROR("Group %d: Invalid bufferMode %d.", group->cfg.grpId,data->bufferMode);
                ret=SETPARAM_ERR;
                break;
            }
            break;
        }
        case cmd_adx_ReadDeviceStatus:
        {
            struct{
                int grpId;
                int index;
                int devId;
                int *status;
            }*data = event->data;
            if(data->status==NULL)
            {
                ULOG_ERROR("Group %d: Invalid status pointer.", group->cfg.grpId);
                ret=SETPARAM_ERR;
                break;
            }
            //立即执行
            FifoEvent_t evt;
            evt.cmdNo=event->type;
            evt.index=data->index;
            evt.deviceId=data->devId;
            evt.forwardStatus=0;
            ret=executeEvent(group,&evt);
            if(ret)
            {
                ULOG_ERROR("Group %d: executeEvent failed.", group->cfg.grpId);
                break;
            }
            *data->status=evt.forwardStatus;
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

int groupBaseState_onUpdate(AxisGroupControl_t *group, double dt) {
    //速度规划    
    CmdTrajectory_t *p_traj;
    int ret;
    p_traj= getCurrentElem(&group->trajectoryQ,group->planIndex);
    //printf("Group %d: p_traj->index=%d, p_traj->state=%d, planIndex=%d.\n", group->cfg.grpId,p_traj->index,p_traj->state,group->planIndex);
    if(p_traj&&p_traj->state>=2)
    {
        
        if(p_traj->state==2)
        {
            double vs=p_traj->vs;
            double ve=p_traj->ve;
            ret=trapezoidal_profile(&p_traj->tp,p_traj->eqLen,vs,p_traj->vel,\
                ve,p_traj->acc,p_traj->dec,dt);
            if(ret)
            {
                ULOG_ERROR("trapezoidal_profile error ret=%d",ret);
                return ret;
            }
            group->endTime=group->endTime+p_traj->tp.t;//路径终点时刻
            p_traj->time=group->endTime;               //该段路径结束时刻
            p_traj->state=3;
            //更新速度规划索引
            group->planIndex=getNextIndex(&group->trajectoryQ,group->planIndex);
        }
        else
        {
            //更新速度规划索引
            group->planIndex=getNextIndex(&group->trajectoryQ,group->planIndex);
        }

    }

    //监护条件导致的状态切换
    if(group->state!=&GroupStandbyState && isGroupStandby(group))
    {
        ret=groupTransitionState(group,&GroupStandbyState);
        if(ret)
        {
            ULOG_ERROR("groupTransitionState error ret=%d",ret);
            return ret;
        }
    }

    //出队完成的轨迹段
    while(QueueFreeSize(&group->trajectoryQ)<10)
    {
        CmdTrajectory_t *p_traj;
        p_traj= (CmdTrajectory_t*)getElemFromRear(&group->trajectoryQ,group->inpIndex);
        if(p_traj&&p_traj->state==5)
        {
            //出队
            CmdTrajectory_t traj;
            ret=DeQueue(&group->trajectoryQ,&traj);
            if(ret)
            {
                ULOG_ERROR("DeQueueISR error ret=%d",ret);
                return ret;
            }

        }
        else
        {
            break;
        }
    }

    //出队完成的事件
    while(QueueFreeSize(&group->eventQ)<10)
    {
        FifoEvent_t *p_evt;
        p_evt= (FifoEvent_t*)getElemFromRear(&group->eventQ,group->evtIndex);
        if(p_evt&&p_evt->state>0)
        {
            //出队
            FifoEvent_t evt;
            ret=DeQueue(&group->eventQ,&evt);
            if(ret)
            {
                ULOG_ERROR("DeQueueISR error ret=%d",ret);
                return ret;
            }

        }
        else
        {
            break;
        }
    }
    // int num=QueueLength(&group->trajectoryQ);
    // if(num==12)
    // {
    //     for(int i=0;i<num;i++)
    //     {
    //         CmdTrajectory_t *p_traj=(CmdTrajectory_t *)getElemFromHead(&group->trajectoryQ,i);
    //         printf("Group %d:  index=%d, state=%d, vs=%lf, ve=%lf.\n", group->cfg.grpId,p_traj->index,p_traj->state,p_traj->vs,p_traj->ve);
    //     }
    // }


    return 0;   
}

int groupBaseState_onRtUpdate(AxisGroupControl_t* group, double dt) {
 
    (void)dt;
    int ret=0;
    //轴组插补
    //printf("Group %d: groupBaseState_onRtUpdate state=%s .\n", group->cfg.grpId, group->state->name);
    ret=groupInterpolation(group);
    if(ret)
    {
        ULOG_ERROR("groupInterpolation error ret=%d",ret);
        return ret;
    }

    //缓存事件执行
    FifoEvent_t *pEvt=NULL;
    while(1 && group->inp.state==0)
    {
        ret=getExecutedFifoEvent(group,&pEvt);
        if(ret)
        {
            ULOG_ERROR("Group: %d, state: %d, getExecutedFifoEvent error ret=%d", group->cfg.grpId,group->state->status,ret);
            return ret;
        }
        if(pEvt==NULL|| pEvt->state>0) break;
        printf("Group: %d, executeEvent, index=%d, syncCnt=%d\n", group->cfg.grpId, pEvt->index, pEvt->syncCnt);
        ret=executeEvent(group,pEvt);
        if(ret)
        {
            ULOG_ERROR("Group: %d, state: %d, executeEvent error ret=%d", group->cfg.grpId,group->state->status,ret);
            return ret;
        }
    }


    return 0;
}

const AxisGroupBaseState_t groupBaseState = {
    .name = "GroupBaseState",
    .status= GRP_BASESTATE,
    .onEnter = groupBaseState_onEnter,
    .onExit  = groupBaseState_onExit,
    .onUpdate = groupBaseState_onUpdate,
    .onRtUpdate = groupBaseState_onRtUpdate,
    .onEvent = groupBaseState_onEvent,
};
