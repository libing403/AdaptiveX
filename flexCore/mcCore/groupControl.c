
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
#include "flexCore_if.h"
#include "errorDefine.h"
#include "MathLib.h"

 GantryControl_t gantryCtrl[MAX_GANTRY_COUNT]; //龙门同步控制器
/*==================== 内部声明：组状态机状态 =====================*/
extern const AxisGroupState_t GroupDisableState;
extern const AxisGroupState_t GroupStandbyState;
extern const AxisGroupState_t GroupMovingState;
extern const AxisGroupState_t GroupHomingState;
extern const AxisGroupState_t GroupStoppingState;
extern const AxisGroupState_t GroupErrorStopState;
/*-------------------- 组状态转换辅助函数 --------------------*/


int syncGroupStateToAxis(AxisGroupControl_t *group) {
    int ret=0;
    // 轴组状态影响单轴状态
    switch (group->state->status) {
        case GRP_MOVING:
            // 单轴处于同步运动状态
            for(int i=0;i<group->kine.axisCount;i++)
            {
                ret=axisTransitionStateAndNoSync(group->kine.axis[i], &SyncMotionState);
                // group->kine.axis[i]->preState= group->kine.axis[i]->state; // 保存上一个状态
                // group->kine.axis[i]->state = &SyncMotionState;  
                
            }
            break;
        case GRP_STANDBY:
            // 单轴处于待机状态
            for(int i=0;i<group->kine.axisCount;i++)
            {
                ret=axisTransitionStateAndNoSync(group->kine.axis[i], &StandstillState);
                // group->kine.axis[i]->preState= group->kine.axis[i]->state; // 保存上一个状态
                // group->kine.axis[i]->state = &StandstillState;
            }
            break;
        case GRP_STOPPING:
            //单轴处于同步运动状态
            for(int i=0;i<group->kine.axisCount;i++)
            {
                ret=axisTransitionStateAndNoSync(group->kine.axis[i], &SyncMotionState);
                // group->kine.axis[i]->preState= group->kine.axis[i]->state; // 保存上一个状态
                // group->kine.axis[i]->state = &SyncMotionState;

            }
            break;
        case GRP_HOMING:
            // 单轴处于同步运动状态
            for(int i=0;i<group->kine.axisCount;i++)
            {
                ret=axisTransitionStateAndNoSync(group->kine.axis[i], &SyncMotionState);
                // group->kine.axis[i]->preState= group->kine.axis[i]->state; // 保存上一个状态
                // group->kine.axis[i]->state = &SyncMotionState;
            }
            break;
        default:
            ULOG_ERROR("Group %d: state %s: undefine or unsupport state sync.",
                group->cfg.grpId, group->state->name);
            break;
        // 其他情况...
        return ret;
    }
    
    return 0;
}

int groupTransitionState(AxisGroupControl_t *group, const AxisGroupState_t *newState) {
    int ret=0;
    if (group->state && group->state->onExit)
        group->state->onExit(group);
    group->preState = group->state; // 保存上一个状态
    group->state = newState;
    
    if (group->state && group->state->onEnter)
        group->state->onEnter(group);
    // 同步轴组状态到单轴
    syncGroupStateToAxis(group);
    return ret;
}

int groupTransitionStateNoSync(AxisGroupControl_t *group, const AxisGroupState_t *newState) {
    int ret=0;
    if (group->state && group->state->onExit)
        group->state->onExit(group);
    group->preState = group->state; // 保存上一个状态
    group->state = newState;
    
    if (group->state && group->state->onEnter)
        group->state->onEnter(group);
    return ret;
}


/*==================== 公共 API 实现 =====================*/
int  GroupControlInit() {

    static int firstTime = 0;
    AxisGroupControl_t* grp = _adx_GRP;
    memset(grp,0,sizeof(AxisGroupControl_t)*MAX_GROUP_COUNT);
    if (firstTime == 0)
    {
        
        for (int i = 0; i < MAX_GROUP_COUNT; i++)
        {
            
            //memset(grp,0,sizeof(AxisGroupControl_t));
            
            grp[i].hMutex = CreateMutex(NULL, FALSE, NULL);
            if (NULL == grp[i].hMutex)
            {
                ULOG_ERROR("grp->hMutex creat error \r\n");
                return CREATE_MUTEX_ERR;
            }
            grp[i].trajectoryQ.hMutex = CreateMutex(NULL, FALSE, NULL);
            if (NULL == grp[i].trajectoryQ.hMutex)
            {
                ULOG_ERROR("grp->trajectoryQ.hMutex creat error \r\n");
                return CREATE_MUTEX_ERR;
            }
        }
        firstTime = 1;
    }
    for(int i=0;i<MAX_GROUP_COUNT;i++)
    {
        
        //运动队列初始化
        int ret = InitQueue(&grp[i].trajectoryQ, grp[i].trajectoryBuffer, MAX_GRPTRAJECT_COUNT+1, sizeof(CmdTrajectory_t));
        if (ret)
        {
            ULOG_ERROR("file %s,fuction line %s  error!",__FILE__,__FUNCTION__,__LINE__);
            return QUEUEINIT_ERR;
        }
        //初始化事件队列
        ret = InitQueue(&grp[i].eventQ, grp[i].eventBuffer, MAX_GRPcmd_COUNT+1, sizeof(FifoEvent_t));
        if (ret)
        {
            ULOG_ERROR("file %s,fuction line %s  error!",__FILE__,__FUNCTION__,__LINE__);
            return QUEUEINIT_ERR;
        }
        //用户参数初始化默认值
        grp[i].usrParam.vs=0;
        grp[i].usrParam.ve=0;
        grp[i].usrParam.smoothLevel=20;
        grp[i].usrParam.velMax=100;
        grp[i].usrParam.accMax=200;
        grp[i].usrParam.jerkMax=1000;
        grp[i].usrParam.wMax=PM_PI;
        grp[i].usrParam.aMax=PM_PI*2;
        grp[i].usrParam.jMax=PM_PI*10;
        grp[i].usrParam.lookAheadEnabled=1; //默认启用前瞻平滑

        //配置初始化
        grp[i].cfg.grpId=i;
        grp[i].cfg.grpEnable=0;
        grp[i].cfg.execID=0;
        //运动学初始化
        grp[i].kine.grpType=0;
        grp[i].kine.axisCount=0;
        grp[i].kine.coordDim=0;
        for(int j=0;j<MAX_AXIS_COUNT;j++)
        {
            
            grp[i].kine.axis[j]=NULL;
        }
        grp[i].kine.pInverseKin=inverseTransformQuat;
        grp[i].kine.pForwardKin=forwardTransformQuat;

        grp[i].refTime=&globalReferenceTime;
        //专用队列索引初始化
        grp[i].smoothIndex=0;
        grp[i].planIndex=0;
        grp[i].inpIndex=0;

 
        //状态初始化
        grp[i].state = &GroupDisableState;
        if (grp[i].state->onEnter)
            grp[i].state->onEnter(grp);    
            
        //插补器初始化
        grp[i].inp.dt=CONTROL_PERIOD_SEC;
        grp[i].inp.cmdNo=0;
        grp[i].inp.index=0;
        grp[i].inp.syncTime=0;
        grp[i].inp.state=0;
        grp[i].inp.time=0;
        grp[i].cyclePeriod=CONTROL_PERIOD_SEC;
        grp[i].enSyncCnt=0;
        grp[i].delSyncCnt=0;
        grp[i].usrParam.accnMax=grp[i].usrParam.smoothLevel*100.0;

        for(int j=0;j<MAX_AXIS_COUNT;j++)
        {
            grp[i].curInpTraj.pos[j]=0;
        }
        //速度倍率控制器初始化
        ret=initSpeedRate(&grp[i].speedRate, 1.0, -1.0, 1.0,  
            grp[i].usrParam.velMax, grp[i].usrParam.accMax, grp[i].usrParam.jerkMax);
        if(ret)
        {
            ULOG_ERROR("file %s,fuction line %s  error!",__FILE__,__FUNCTION__,__LINE__);
            return SPEEDRATEINIT_ERR;
        }

        //停止倍率控制器初始化
        ret=initSpeedRate(&grp[i].stopRate, 1.0, -1.0, 1.0,  
            grp[i].usrParam.velMax, grp[i].usrParam.accMax, grp[i].usrParam.jerkMax);
        if(ret)
        {
            ULOG_ERROR("file %s,fuction line %s  error!",__FILE__,__FUNCTION__,__LINE__);
            return SPEEDRATEINIT_ERR;
        }

    }

    return 0;
}

bool isGroupStandby(AxisGroupControl_t *group) {
    
    bool isStandby = false;
    double disErr=0.0;
    for(int i=0;i<group->kine.coordDim;i++)
    {   

        double di=group->inp.curPos[i]-group->curInpTraj.pos[i];
        disErr=disErr+di*di;
    }
    disErr=sqrt(disErr);
    if(isQueueRear(&group->trajectoryQ,group->inpIndex) \
    && group->inp.state == 0 \
    && (group->curTraj_ptr ? group->curTraj_ptr->state == 5 : 0) \
    && fabs(group->inp.time-group->endTime)<MAX_TIME_ERR\
    && group->speedRate.targetRate>0.0\
    && disErr<MAX_DISTANCE_ERR\
    && group->waitCount++>STATE_WAIT_COUNT)
    {

        // printf("disErr=%.5f,group->inp.time=%.5f,group->endTime=%.5f\n",disErr,group->inp.time,group->endTime);
        // printf("isQueeueRear=%d,group->inp.state=%d,group->speedRate.targetRate=%.5f\n",
        //     isQueueRear(&group->trajectoryQ,group->inpIndex),group->inp.state,group->speedRate.targetRate);
        // printf("waitCount=%d,STATE_WAIT_COUNT=%d\n",group->waitCount,STATE_WAIT_COUNT);
        isStandby = true;
        group->waitCount = 0; // 重置等待计数器
    }
    return isStandby;
}


int GroupControl_Update() {

    int ret=0;
    AxisGroupControl_t *grp=_adx_GRP;
    for(int i=0;i<MAX_GROUP_COUNT;i++)
    {        
        if (grp[i].state && grp[i].state->onUpdate)
        {
            ret=grp[i].state->onUpdate(&grp[i],CONTROL_PERIOD_SEC); // 使用与单轴控制相同的周期 dt
            if(ret)
            {
                ULOG_ERROR("group %d :GroupControl_Update error ret=%d.",i,ret);
                break;
            }
        }
            
        // ReleaseMutex(grp[i].hMutex);
    }
    return 0;
}

int GroupControl_RtUpdate() {

    int ret;
	AxisGroupControl_t *grp= _adx_GRP;
	for(int i=0;i<MAX_GROUP_COUNT;i++)
	{
		if(grp[i].state&&grp[i].state->onRtUpdate)
        {
            ret = grp[i].state->onRtUpdate(&grp[i],CONTROL_PERIOD_SEC);
            if(ret)
            {
                ULOG_ERROR("group[%d] onRtUpdate error ret=%d\n",i,ret);
                //原子操作，进入到错误状态
                return ret;
            }
        }

	}
    return 0;

}

int GroupControl_HandleEvent(AxisGroupControl_t *group, const FsmEvent_t *event) {
    
    int ret;
    WaitForSingleObject(group->hMutex, INFINITE);
    if (group->state && group->state->onEvent)
    {
        ret=group->state->onEvent(group, event);
        ULOG_INFO("group %d : %s ret=%d",group->cfg.grpId,event->name,ret);
    }
    else
    {
        ULOG_ERROR("%s [cmdNo=%d],%s ,Line %d : group.state==NULL,group.state.OnEvent==NULL is null.",event->name,event->type,__func__,__LINE__);
        ret= CMDDISPATCH_ERR;
    }
    ReleaseMutex(group->hMutex);
    return ret;
}



int writeGrpParam_Handle(AxisGroupControl_t *grp, const FsmEvent_t *event) {
    if (grp == NULL || event == NULL) return 1;
    int ret = 0;
    //提取事件数据paramId
    struct {
        int grpId;
        int index;
        int paramId;
        double value;
        int bufferMode;


    }*ptr=event->data;
    switch (ptr->paramId)
    {   

        case mcSmoothLevel: //平滑级别
        {
            grp->usrParam.smoothLevel=ptr->value;
            grp->usrParam.accnMax=grp->usrParam.smoothLevel*100.0;
            printf("grp->usrParam.smoothLevel=%.2f,grp->usrParam.accnMax=%.2f\n",grp->usrParam.smoothLevel,grp->usrParam.accnMax);
            ret=0;
            break;
        }


        default:
            // 处理其他参数
            ULOG_ERROR("Group %d: state %s: undefine or unsupport parameter [%d].", 
                grp->cfg.grpId, grp->state->name, ptr->paramId);
            ret=UNSUPPORTEDPARAM_ERR;
            break;
    }
    return ret;
}


int readGrpParam_Handle(AxisGroupControl_t *grp, const FsmEvent_t *event) {
    if (grp == NULL || event == NULL) return 1;
    int ret = 0;
    // 假设 event->data 是一个指向参数数据的指针
    // 这里假设数据结构为 { axisId, paramId, value }
    struct {
        int axisId;
        int index;
        int paramId;
    }*ptr=event->data;
    // 处理参数写入事件
    // 根据 paramId 和 value 更新轴的参数
    // 这里可以添加具体的参数处理逻辑
    switch (ptr->paramId)
    {   
        case mcGrpSpeedRate:
        {
            struct {
                int axisId;
                int index;
                int paramId;
                double *value;
            }*data=event->data;
            *(data->value)=getSpeedRate(&grp->speedRate);
            ret=0;
            break;
        }
        

        default:
            // 处理其他参数
            ret=UNSUPPORTEDPARAM_ERR;
            break;
    }
    return ret;
}


int inverseTransform(int grpId,double *cartPos,double *refPos, double *axisPos)
{
    //运动学逆解
    (void)refPos;
    int ret=0;
    AxisGroupControl_t *grp=&_adx_GRP[grpId];
    int axisNum=grp->kine.axisCount;
    for(int i=0;i<axisNum;i++)
    {
        axisPos[i]=cartPos[i];
    }
    return ret;
}

int inverseTransformQuat(int grpId,double pose[7],double refPose[7],double jointPos[6])
{
    jointPos[0]=pose[0];
    jointPos[1]=pose[1];
    jointPos[2]=pose[2];
    //四元数转xyz欧拉角
    double rx,ry,rz;
    quat2xyzEulerAngle(pose+3,&rx,&ry,&rz);
    jointPos[3]=rx;
    jointPos[4]=ry;
    jointPos[5]=rz;
    return 0;
}


int forwardTransformQuat(int grpId,double jointPos[6],double pose[7])
{
    pose[0]=jointPos[0];
    pose[1]=jointPos[1];
    pose[2]=jointPos[2];
    //xyz欧拉角转四元数
    double quat[4];
    xyzEulerAngle2quat(jointPos[3],jointPos[4],jointPos[5],quat);
    pose[3]=quat[0];
    pose[4]=quat[1];
    pose[5]=quat[2];
    pose[6]=quat[3];
     return 0;
}


int forwardTransform(int grpId,double *axisPos,double *cartPos)
{
    //运动学正解
    int ret=0;
    AxisGroupControl_t *grp=&_adx_GRP[grpId];
    int axisNum=grp->kine.axisCount;
    for(int i=0;i<axisNum;i++)
    {
        cartPos[i]=axisPos[i];
    }
    return ret;
}


int lookAhead(AxisGroupControl_t *grp)
{
    CmdTrajectory_t *p_traj,*pT0,*pT1;
    int lookIndex,prevIndex,nextIndex;
    if(grp->usrParam.lookAheadEnabled==0)
    {
        pT1=getElemFromRear(&grp->trajectoryQ,0);
        pT1->state=2; // 设置状态为预处理完成
        return 0; //前瞻平滑未启用
    }

    if(QueueLengthToRear(&grp->trajectoryQ,grp->smoothIndex)<1)
    {
        //没有轨迹指令
        return 0;
    }
    else if(QueueLengthToRear(&grp->trajectoryQ,grp->smoothIndex)<2)
    {
        p_traj=getElemFromRear(&grp->trajectoryQ,0);
        if (p_traj&&p_traj->state==1) 
        {            
            p_traj->state=2; 
        }
        return 0;
    }
    else if(QueueLengthToRear(&grp->trajectoryQ,grp->smoothIndex)<3)
    {

        p_traj=getElemFromRear(&grp->trajectoryQ,0);
        if (p_traj&&p_traj->state==1) 
        {
            p_traj->state=2; 
        }
    }
    else 
    {
        lookIndex=getRearElemIndex(&grp->trajectoryQ);
        prevIndex=getPreviousIndex(&grp->trajectoryQ,lookIndex);
        //回溯
        while(prevIndex!=grp->smoothIndex)
        {
            
            pT1=getCurrentElem(&grp->trajectoryQ,lookIndex);
            pT0=getCurrentElem(&grp->trajectoryQ,prevIndex);
            if(pT1->state==0)break; //如果当前指令状态为0，退出
            double vi1=pT1->ve;
            double vi0=sqrt(vi1*vi1+2.0*pT1->acc*pT1->len);
            if(vi0> pT0->velLimit)
            {            
                vi0=pT0->velLimit;
                pT0->ve=vi0;
                pT1->vs=vi0;
                break; //退出回溯
            }
            else
            {
                pT0->ve=vi0;
                pT1->vs=vi0;
            }
            //vi0=fmin(vi0,pT0->velLimit);
            lookIndex=getPreviousIndex(&grp->trajectoryQ,lookIndex);
            prevIndex=getPreviousIndex(&grp->trajectoryQ,prevIndex);
        }
        //向后调整
        nextIndex=lookIndex;
        lookIndex=prevIndex;
        while(!isQueueRear(&grp->trajectoryQ,nextIndex))
        {
            pT1=getCurrentElem(&grp->trajectoryQ,nextIndex);
            pT0=getCurrentElem(&grp->trajectoryQ,lookIndex);
            if(pT1->state==0)break; //如果当前指令状态为0，退出
            double vi0=pT0->ve;
            double vi1=sqrt(vi0*vi0+2.0*pT1->acc*pT1->len);
            vi1=fmin(vi1,pT1->velLimit);
            pT1->ve=vi1;
            pT1->vs=vi0;
            pT1->state=2;
            lookIndex=getNextIndex(&grp->trajectoryQ,lookIndex);
            nextIndex=getNextIndex(&grp->trajectoryQ,nextIndex);

        }

    }
    pT1=getElemFromRear(&grp->trajectoryQ,0);
    pT1->state=2; // 设置状态为预处理完成
    return 0;
}

static double getPoseRotScale(AxisGroupControl_t *grp, CmdTrajectory_t *traj)
{
    const double eps = 1.0e-5;
    double vRef;
    double wRef;

    if (grp == NULL)
    {
        return 1.0;
    }

    vRef = (traj != NULL && traj->vel > eps) ? traj->vel : grp->usrParam.velMax;
    if (vRef < 1.0)
    {
        vRef = 1.0;
    }

    wRef = (grp->usrParam.wMax > eps) ? grp->usrParam.wMax : vRef;
    if (wRef <= eps)
    {
        wRef = eps;
    }

    return vRef / wRef;
}

static void calcTrajectoryGeometry(AxisGroupControl_t *grp,
                                   CmdTrajectory_t *traj,
                                   const double *from,
                                   const double *to)
{
    const double eps = 1.0e-5;
    const int poseDim = MAX_CARTESIAN_COUNT;
    const int posDimMax = 3;
    double rotScale;
    double posLen2 = 0.0;
    double posLen;
    double rotEqLen;
    double eqLen;
    // double nextPosVec[3] = { 0.0, 0.0, 0.0 };
    // double nextPosLen2 = 0.0;
    // double nextPosLen;
    int posDim;
    double qStart[4] = { 1.0, 0.0, 0.0, 0.0 };
    double qEnd[4] = { 1.0, 0.0, 0.0, 0.0 };
    double qEndAligned[4] = { 1.0, 0.0, 0.0, 0.0 };
    double qStartConj[4] = { 1.0, 0.0, 0.0, 0.0 };
    double qRel[4] = { 1.0, 0.0, 0.0, 0.0 };
    double rpyStart[3] = { 0.0, 0.0, 0.0 };
    double rpyEnd[3] = { 0.0, 0.0, 0.0 };
    double dot;
    double sinHalf;
    int dim;
    int k;

    if (grp == NULL || traj == NULL || from == NULL || to == NULL)
    {
        return;
    }

    dim = grp->kine.coordDim;
    posDim = (dim < posDimMax) ? dim : posDimMax;
    rotScale = getPoseRotScale(grp, traj);

    for (k = 0; k < MAX_AXIS_COUNT; k++)
    {
        traj->vec[k] = 0.0;
    }
    for (k = 0; k < poseDim; k++)
    {
        traj->dirVec[k] = 0.0;
        traj->startQua[k] = 0.0;
        traj->endQua[k] = 0.0;
        traj->rotVec[k] = 0.0;
    }

    // for (k = 0; k < dim && k < poseDim; k++)
    // {
    //     traj->startPose[k] = from[k];
    //     traj->endPose[k] = to[k];
    // }

    /* 1) 先计算位置几何参数 */
    for (k = 0; k < posDim; k++)
    {
        double dPos = to[k] - from[k];
        traj->dirVec[k] = dPos;
        traj->vec[k] = dPos;
        posLen2 += dPos * dPos;
    }

    posLen = sqrt(posLen2);
    traj->posLength = posLen;
    if (posLen <= eps)
    {
        for (k = 0; k < posDim; k++)
        {
            traj->vec[k] = 0.0;
            traj->dirVec[k] = 0.0;
        }
    }
    else
    {
        for (k = 0; k < posDim; k++)
        {
            traj->vec[k] /= posLen;
            traj->dirVec[k] /= posLen;
        }
    }

    /* 2) 再计算姿态几何参数 */
    if (dim > 3)
    {
        rpyStart[0] = from[3];
        rpyStart[1] = from[4];
        rpyStart[2] = from[5];
        rpyEnd[0] = to[3];
        rpyEnd[1] = to[4];
        rpyEnd[2] = to[5];
        rpy2quat(rpyStart, qStart);
        rpy2quat(rpyEnd, qEnd);

        dot = qStart[0] * qEnd[0] + qStart[1] * qEnd[1] + qStart[2] * qEnd[2] + qStart[3] * qEnd[3];
        if (dot < 0.0)
        {
            qEndAligned[0] = -qEnd[0];
            qEndAligned[1] = -qEnd[1];
            qEndAligned[2] = -qEnd[2];
            qEndAligned[3] = -qEnd[3];
            dot = -dot;
        }
        else
        {
            qEndAligned[0] = qEnd[0];
            qEndAligned[1] = qEnd[1];
            qEndAligned[2] = qEnd[2];
            qEndAligned[3] = qEnd[3];
        }

        dot = clamp_value(dot, -1.0, 1.0);
        traj->rotAngle = 2.0 * acos(dot);
        rotEqLen = traj->rotAngle * rotScale;

        quat_conjugate(qStart, qStartConj);
        quat_multiply(qEndAligned, qStartConj, qRel);
        if (quat_normalize(qRel, qRel) != 0)
        {
            qRel[0] = 1.0;
            qRel[1] = 0.0;
            qRel[2] = 0.0;
            qRel[3] = 0.0;
        }
        sinHalf = sqrt(fmax(0.0, 1.0 - qRel[0] * qRel[0]));
        if (sinHalf > eps)
        {
            traj->rotVec[0] = qRel[1] / sinHalf;
            traj->rotVec[1] = qRel[2] / sinHalf;
            traj->rotVec[2] = qRel[3] / sinHalf;
        }
        else
        {
            traj->rotVec[0] = 1.0;
            traj->rotVec[1] = 0.0;
            traj->rotVec[2] = 0.0;
        }

        traj->startQua[0] = qStart[0];
        traj->startQua[1] = qStart[1];
        traj->startQua[2] = qStart[2];
        traj->startQua[3] = qStart[3];
        traj->endQua[0] = qEndAligned[0];
        traj->endQua[1] = qEndAligned[1];
        traj->endQua[2] = qEndAligned[2];
        traj->endQua[3] = qEndAligned[3];

    }
    else
    {
        traj->rotAngle = 0.0;
        rotEqLen = 0.0;
        traj->startQua[0] = 1.0;
        traj->endQua[0] = 1.0;
        traj->rotVec[0] = 1.0;

    }
    eqLen = fmax(posLen, rotEqLen);
    traj->eqLen = eqLen;
    traj->len = eqLen;
    traj->theta = PM_PI;

}

int trajectoryPretreatment(AxisGroupControl_t *grp)
{
	CmdTrajectory_t* pT2 = NULL, * pT1 = NULL, * pT0 = NULL;
    Queue *pTQ=&grp->trajectoryQ;
	int k;
	double p0[MAX_AXIS_COUNT],p1[MAX_AXIS_COUNT], p2[MAX_AXIS_COUNT];
	double viMax = 0.0;
    double wiMax = 0.0;
    //int N = QueueLengthISR(pTQ);
    if(QueueLengthToRear(&grp->trajectoryQ,grp->inpIndex)<1 )
    {
         return 0;
    }
	else if(QueueLengthToRear(&grp->trajectoryQ,grp->inpIndex) < 2)
    {
        pT1 = getElemFromRear(pTQ, 0);
        pT0 = &grp->curInpTraj;
        
        //取出轴组的坐标
		for (k = 0; k < grp->kine.coordDim; k++)
		{	        
			p0[k] = pT0->pos[k];
			p1[k] = pT1->pos[k];     
		}
		//计算方向矢量和夹角
        if (pT1->len < 1.0E-5)
        {
            for (k = 0; k < grp->kine.coordDim; k++)
            {
                pT1->vec[k] = 0.0;              
                if (k < MAX_CARTESIAN_COUNT)
                {
                    pT1->dirVec[k] = 0.0;
                }
            }
            pT1->theta = PM_PI;
        }
        //计算位姿几何参数（位置+姿态）
        calcTrajectoryGeometry(grp, pT1, p0, p1);
        pT1->velLimit=grp->usrParam.ve;        
        pT1->state = 1; // 设置状态为预处理完成
        return 0; 
    }	
	else if(QueueLengthToRear(&grp->trajectoryQ,grp->inpIndex) <3)
	{
           
        pT2 = getElemFromRear(pTQ, 0);
        pT1 = getElemFromRear(pTQ, 1);
        pT0 = &grp->curInpTraj;
 
	}
	else 
	{
		pT2 = getElemFromRear(pTQ, 0);
		pT1 = getElemFromRear(pTQ, 1);
		pT0 = getElemFromRear(pTQ, 2);

	}
    //取出轴组的坐标
    for (k = 0; k < grp->kine.coordDim; k++)
    {    
        p0[k] = pT0->pos[k];
        p1[k] = pT1->pos[k];
        p2[k] = pT2->pos[k]; 
    }

    //计算位姿几何参数（位置+姿态）
    calcTrajectoryGeometry(grp, pT1, p1, p2);
    pT1->posTheta = calc_vector_angle(3, pT1->dirVec, pT2->dirVec);
    pT1->rotTheta = calc_vector_angle(3, pT1->rotVec, pT2->rotVec);
    //计算几何条件确定的拐角速度
    //各轴速度限制
    viMax = fmin(pT1->vel, pT2->vel);
    for(k = 0; k < grp->kine.coordDim; k++)
    {        
        if (fabs(pT1->vec[k]) > 1.0e-5)
        {
            viMax = fmin(viMax, fabs(pT1->vimax[k] / pT1->vec[k]));
        }
    }
    //位置拐角限制
    viMax = fmin(viMax,   grp->usrParam.accnMax* grp->cyclePeriod / (2.0 * sin(pT1->theta / 2.0)));
    
    //姿态方向拐角限制
    for(k = 0; k < grp->kine.coordDim; k++)
    {        
        if (fabs(pT1->rotVec[k]) > 1.0e-5)
        {
            wiMax = fmin(wiMax, fabs(pT1->wimax[k] / pT1->rotVec[k]));
        }
    }   
    wiMax = fmin(wiMax, grp->usrParam.aMax* grp->cyclePeriod / (2.0 * sin(pT1->rotTheta / 2.0)));
    //综合位置和姿态的拐角限制
    double scale = getPoseRotScale(grp, pT1);
    viMax = fmin(viMax, wiMax * scale);
    pT1->velLimit = viMax;
    pT2->state = 1; // 设置状态为预处理完成
	return 0;
}

int executeEvent(AxisGroupControl_t *grp,FifoEvent_t *pEvt)
{
    int ret=0;
    switch (pEvt->cmdNo)
    {
    case cmd_adx_GroupSetMcode:
        if(grp->speedRate.currentRate>=0.0 &&pEvt->forwardValAddr)
        {
            *pEvt->forwardValAddr=pEvt->forwardValue;
            ULOG_INFO("Group %d: set Mcode forward value %.2f.", 
            grp->cfg.grpId, *pEvt->forwardValAddr);
            pEvt->state=1; //正向运动处理事件完成
        }
        else if(grp->speedRate.currentRate<0.0 &&pEvt->reverseValAddr)
        {
            *pEvt->reverseValAddr=pEvt->reverseValue;
            ULOG_INFO("Group %d: set Mcode reverse value %.2f.", 
            grp->cfg.grpId, *pEvt->reverseValAddr);
            pEvt->state=2; //反向运动处理事件完成
        }
        break;
    
    case cmd_adx_SetDeviceStatus :
        if(grp->speedRate.currentRate >= 0.0 && (pEvt->direction==0 || pEvt->direction==2))//正向或双向
        {
            ret=grp->setDeviceStatus[pEvt->deviceId](pEvt->deviceId,pEvt->forwardStatus);
            ULOG_INFO("Group %d: set device %d status to %d.", 
            grp->cfg.grpId, pEvt->deviceId, pEvt->forwardStatus);
            pEvt->state=1; //正向运动处理事件完成
        }
        else if(grp->speedRate.currentRate < 0.0 && (pEvt->direction==1 || pEvt->direction==2))//反向或双向
        {
            ret=grp->setDeviceStatus[pEvt->deviceId](pEvt->deviceId,pEvt->reverseStatus);
            ULOG_INFO("Group %d: set device %d status to %d.", 
            grp->cfg.grpId, pEvt->deviceId, pEvt->reverseStatus);
            pEvt->state=2; //反向运动处理事件完成
        }
        break;
    case cmd_adx_ReadDeviceStatus:
        if(grp->speedRate.currentRate >= 0.0 && (pEvt->direction==0 || pEvt->direction==2))//正向或双向
        {
            ret=grp->readDeviceStatus[pEvt->deviceId](pEvt->deviceId,&pEvt->forwardStatus);
            ULOG_INFO("Group %d: read device %d status %d.", 
            grp->cfg.grpId, pEvt->deviceId, pEvt->forwardStatus);
            pEvt->state=1; //正向运动处理事件完成
        }
        else if(grp->speedRate.currentRate < 0.0 && (pEvt->direction==1 || pEvt->direction==2))//反向或双向
        {
            ret=grp->readDeviceStatus[pEvt->deviceId](pEvt->deviceId,&pEvt->reverseStatus);
            ULOG_INFO("Group %d: read device %d status %d.", 
            grp->cfg.grpId, pEvt->deviceId, pEvt->reverseStatus);
            pEvt->state=2; //反向运动处理事件完成
        }

        break;

    default:
        ULOG_ERROR("Group %d: state %s: undefine or unsupport event [%d].", 
        grp->cfg.grpId, grp->state->name, pEvt->cmdNo);
        ret=UNSUPPORTEDCMD_ERR;
        break;
    }

    return ret;
}

int getExecutedFifoEvent(AxisGroupControl_t *grp,FifoEvent_t **pEvt)
{
	//执行缓存事件
    *pEvt=NULL;
    int ret=0;
    double rate=getSpeedRate(&grp->speedRate);
	CmdTrajectory_t *p_traj= getCurrentElem(&grp->trajectoryQ,grp->inpIndex);
    CmdTrajectory_t *p_nextTraj= getCurrentElem(&grp->trajectoryQ,grp->inpIndex+1);
    //CmdTrajectory_t *p_prevTraj= getCurrentElem(&grp->trajectoryQ,grp->inpIndex-1);
	FifoEvent_t *p_evt= getCurrentElem(&grp->eventQ,grp->evtIndex);	
    if(p_evt!=NULL&& p_nextTraj!=NULL)//有事件和轨迹指令
    {
       if(rate>=0 )//正向运动
       {
            
            if(p_evt->state!=1)//正向未处理
            {
                
                //(1)下一个轨迹syncCnt大于当前事件syncCnt，说明当前事件可执行
                if( p_evt->syncCnt== grp->delSyncCnt+1 )
                {
                    grp->delSyncCnt=p_evt->syncCnt;
                    p_evt=getCurrentElem(&grp->eventQ,grp->evtIndex); 
                    grp->evtIndex=getNextIndex(&grp->eventQ,grp->evtIndex);  
                    *pEvt=p_evt;
                }
                //(2)下一个轨迹syncCnt小于当前事件syncCnt，说明当前事件不可执行，需要等待下一个轨迹
                else
                {
                    *pEvt=NULL;                  
                }
            }  
       }
        else if(rate < 0  )//反向运动
        {
            if(p_evt->state!=2)//反向未处理
            {
                
                //(1)上一个轨迹syncCnt小于当前事件syncCnt，说明当前事件可执行
                if( p_evt->syncCnt== p_traj->syncCnt -1)
                {
                    grp->delSyncCnt=p_evt->syncCnt;
                    p_evt=getCurrentElem(&grp->eventQ,grp->evtIndex); 
                    grp->evtIndex=getNextIndex(&grp->eventQ,grp->evtIndex);  
                    *pEvt=p_evt;
                }
                //(2)上一个轨迹syncCnt大于当前事件syncCnt，说明当前事件不可执行，需要等待上一个轨迹
                else
                {
                    *pEvt=NULL;
                   
                }
            }
        }
        else//不满足执行条件
        {
            *pEvt=NULL;
            return 0;
        }
    }
    else if(p_evt!=NULL && p_traj==NULL)//没有轨迹指令
    {
        if(rate>0  )//正向运动
       {
            if(p_evt->state!=1)//正向未处理
            {
                if( p_evt->syncCnt== p_traj->syncCnt +1)
                {
                    grp->delSyncCnt=p_evt->syncCnt;
                    p_evt=getCurrentElem(&grp->eventQ,grp->evtIndex); 
                    grp->evtIndex=getNextIndex(&grp->eventQ,grp->evtIndex);  
                    *pEvt=p_evt;
                }
                else
                {

                    *pEvt=NULL;

                }
            }
       }
        else//反向运动
        {
            if(p_evt->state!=2)//反向未处理
            {
                if( p_evt->syncCnt== p_traj->syncCnt -1)
                {
                    grp->delSyncCnt=p_evt->syncCnt;
                    grp->inpIndex=getPreviousIndex(&grp->trajectoryQ,grp->inpIndex);
                    p_evt=getCurrentElem(&grp->eventQ,grp->evtIndex);   
                    *pEvt=p_evt;
                }
                else
                {
                    *pEvt=NULL;
                }
            }
        }
    }
    else//没有事件
    {
        *pEvt=NULL;
        return 0;
    }
    return ret;
}
 
int groupInterpolation(AxisGroupControl_t *grp)
{
	int ret;
	//更新轴组倍率
    double speedRate=updateSpeedRate(&grp->speedRate,grp->inp.linearVelocity,grp->inp.linearAcceleration);
	double stopRate=updateSpeedRate(&grp->stopRate,grp->inp.linearVelocity,grp->inp.linearAcceleration);
	double rate=speedRate*stopRate;
	ret=trajectoryLinearInterpolation(grp,rate);
	if (ret)
	{
		ULOG_ERROR("Group: %d, state: %d, trajectoryLinearInterpolation error,ret=%d", grp->cfg.grpId,grp->state->status,ret);
		return ret;
	}
	//进行逆解
	ret = inverseKinematics(grp);
	if (ret)
	{
		ULOG_ERROR("Group: %d, state: %d, inverseKinematics error ret=%d", grp->cfg.grpId,grp->state->status,ret);
		return ret;
	}
 

    return ret;
}




int inverseKinematics(AxisGroupControl_t *grp)
{
	//运动学逆解
	double axisPos[MAX_AXIS_COUNT];
	double refPos[MAX_AXIS_COUNT];
	for(int i=0;i<grp->kine.axisCount;i++)
	{
		refPos[i]=grp->kine.axis[i]->inp.prePos;
	}
	int ret=grp->kine.pInverseKin(grp->cfg.grpId, grp->inp.curPos,refPos,axisPos);
	if(ret)
	{
		ULOG_ERROR("inverseKinematics() error ret= %d", ret);
		return ret;
	}

	//更新轴组中各轴位置
	for (int i = 0; i < grp->kine.axisCount; i++)
	{
		//更新轴插补器
		grp->kine.axis[i]->inp.cmdNo=grp->inp.cmdNo;
		grp->kine.axis[i]->inp.index=grp->inp.index;
		grp->kine.axis[i]->inp.prePos = grp->kine.axis[i]->inp.curPos;
		grp->kine.axis[i]->inp.curPos = axisPos[i];	
		grp->kine.axis[i]->inp.time = grp->inp.time;
		grp->kine.axis[i]->inp.syncDrvTime=grp->inp.time;
		grp->kine.axis[i]->inp.syncInpTime = grp->inp.syncTime;
		grp->kine.axis[i]->inp.state = 1;
		//更新驱动指令
		grp->kine.axis[i]->axisCurTime.cmdNo=grp->inp.cmdNo;
		grp->kine.axis[i]->axisCurTime.index=grp->inp.index;
		grp->kine.axis[i]->axisCurTime.syncTime=grp->inp.syncTime;
		grp->kine.axis[i]->axisCurTime.time=grp->inp.time;
		grp->kine.axis[i]->axisCurTime.lastPos=grp->kine.axis[i]->inp.prePos;
		grp->kine.axis[i]->axisCurTime.curPos=grp->kine.axis[i]->inp.curPos;
	}
	//写入调试文件
	//logFile(grp->grpFile,"%.5f %.5f %.5f %.5f %d\n",	*grp->refTime,grp->kine.pathLength,grp->inp.linearVelocity,grp->inp.linearAcceleration,grp->inp.index);
	//fflush(grp->grpFile);	
	for(int i=0;i<grp->kine.coordDim;i++)
	{
		logFile(grp->cartFile,"%.5f ",grp->inp.curPos[i]);	
	}
		
	//logFile(grp->cartFile,"\n");
	//fflush(grp->cartFile);		

	//printf("inverseKinematics time=%.5f pathlen=%.5f inp->state=%d\n",*grp->refTime,grp->kine.pathLength,grp->inp.state);
	//fflush(stdout);
	return 0;
}

int forwardKinematics(AxisGroupControl_t *grp)
{
	//运动学逆解
	double axisPos[MAX_AXIS_COUNT];
	double cartPos[MAX_AXIS_COUNT];
	for(int i=0;i<grp->kine.axisCount;i++)
	{
		axisPos[i]=grp->kine.axis[i]->inp.curPos;
	}
	int ret=grp->kine.pForwardKin(grp->cfg.grpId,axisPos,cartPos);
	if(ret)
	{
		ULOG_ERROR("forwardKinematics() error ret= %d\n", ret);
		return ret;
	}
	//更新轴组位置
	for(int i=0;i<grp->kine.coordDim;i++)
	{
		grp->inp.prePos[i]=grp->inp.curPos[i];
		grp->inp.curPos[i]=cartPos[i];
	}
	return 0;
}

int writeGrpTimeQ(AxisGroupControl_t *grp)
{
	int ret;
	InpTimeData_t inpTime;
	inpTime.cmdNo=grp->inp.cmdNo;
	inpTime.index=grp->inp.index;
	inpTime.syncTime=grp->inp.index;
	inpTime.time=grp->inp.time;
	for(int i=0;i<grp->kine.axisCount;i++)
	{
		inpTime.lastPos=grp->kine.axis[i]->inp.prePos;
		inpTime.curPos=grp->kine.axis[i]->inp.curPos;
		
		//RTOS需要关中断，避免读写冲突
		ret=EnQueue(&(grp->kine.axis[i]->axisTimeQ),&inpTime);
		if(ret)
		{
			ULOG_ERROR("writeGrpTimeQ error\n");
			return ret;
		}
	}
	return 0;
}