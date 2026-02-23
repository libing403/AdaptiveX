/**
 * @file Interpolation.c
 * @author li.bing(libinggalaxy@163.com)
 * @brief 
 * @version 0.1
 * @date 2025-03-29
 * 
 * Copyright (c)  2025 li.bing
 * 
 */
#include "mcTypes.h"
#include "errorDefine.h"
#include "interpolation.h"
#include "VelocityPlanning.h"
#include "string.h"
#include "groupControl.h"
#include "ulog.h"
#include <math.h>


int initLinearInp(AxisGroupControl_t *grp,double rate) 
{
	int i = 0;
	Inpterpolator_t *inpt=&grp->inp;
	CmdTrajectory_t *p_traj;

	p_traj=(CmdTrajectory_t *)getCurrentElem(&grp->trajectoryQ,grp->inpIndex);
	
	if(p_traj && p_traj->state>=3)
	{	
		if(rate>0)
		{	
			//倍率为正，要前进
			if(fabs(inpt->time-p_traj->time)<1.001e-3 )
			{
				//printf("at the end of the segment, time=%f, p_traj->time=%f\n",inpt->time,p_traj->time);
				//插补到该段的结尾
				if(!isQueueRear(&grp->trajectoryQ,grp->inpIndex) )
				{
					//不是队尾，可以取下一段
					inpt->preLi=inpt->li-p_traj->tp.L;//从新的起点计算位移量
					grp->inpIndex=getNextIndex(&grp->trajectoryQ,grp->inpIndex);
					grp->smoothIndex=grp->inpIndex;
					p_traj=(CmdTrajectory_t *)getCurrentElem(&grp->trajectoryQ,grp->inpIndex);
					if(p_traj->state>=3)
					{
						//已进行速度规划，下一步初始化
					}
					else
					{
						//未完成速度规划
						inpt->state=0;
						return 1;
					}
				}
				else
				{
					//到了轨迹终点，不能继续前进
					inpt->state=0;
					return 1;
				}
			}
			else
			{
				//不在该段结尾，可以沿该段继续前进
				//printf("continue forward, time=%f, p_traj->time=%f\n",inpt->time,p_traj->time);
				
			}
		}
		else
		{
			//倍率为负，要回退
			if(fabs(p_traj->time-inpt->time-p_traj->tp.t)<MAX_TIME_ERR )
			{

				//在该段的开始，
				if(!isQueueHead(&grp->trajectoryQ,grp->inpIndex))
				{
					//不在队头，可以取前一段
					grp->inpIndex=getPreviousIndex(&grp->trajectoryQ,grp->inpIndex);
					p_traj=(CmdTrajectory_t *)getCurrentElem(&grp->trajectoryQ,grp->inpIndex);
					inpt->ti=inpt->initTime + p_traj->tp.t;
					inpt->preLi=inpt->li+p_traj->tp.L;
				}
				else
				{
					//在队头，无法继续回退
					inpt->state=0;
					return 1;
				}

			}
			else
			{
				//不在该段的起始，还可以继续沿该段回退
			}
		}
	}
	else
	{
		//没有准备好的指令
		inpt->state=0;
		return 1;
	}

	memcpy(&inpt->tp, &p_traj->tp, sizeof(TrapeProfile_t));
	inpt->ti=inpt->initTime;
	inpt->cmdNo = p_traj->cmdNo;
	inpt->index = p_traj->index;
	inpt->dimension = p_traj->dim;

	for (i = 0; i < inpt->dimension; i++)
	{
		inpt->startPos[i] = p_traj->startPos[i];
		inpt->endPos[i] = p_traj->pos[i];
	}

	inpt->length = p_traj->len;
	if (inpt->length < 1.0e-5)
	{
		inpt->length = 0.0;
		for (i = 0; i < inpt->dimension; i++)
			inpt->vec[i] = 0.0;
	}
	else
	{
		for (i = 0; i < inpt->dimension; i++)
			inpt->vec[i]=p_traj->vec[i];
	}
	inpt->li = 0.0;
	p_traj->state=4;//插补计算中
	grp->curTraj_ptr=p_traj; 
	memcpy(&grp->curInpTraj, p_traj, sizeof(CmdTrajectory_t));
	grp->delSyncCnt=p_traj->syncCnt;
	return 0;
}

int linearInpterpolation(AxisGroupControl_t *grp,double rate)
{

	Inpterpolator_t *inpt=&grp->inp;
	int i;
	//double len = 0.0;
	double dtime = 0.0;
	//int ret=0;
	double ti = inpt->ti + inpt->dt * rate;

	if (ti > inpt->tp.t)//插补时间最大不能超过总时间
		ti = inpt->tp.t;
	else if (ti < 0.0)//插补时间最小不能小于0
		ti = 0.0;
	dtime = ti - inpt->ti;
	inpt->syncTime = inpt->time;
	inpt->time = inpt->syncTime + dtime;
	inpt->ti = ti;
	if(rate>0.0)
	{
		inpt->initTime = inpt->ti - inpt->tp.t;
	}
	else{
		inpt->initTime=ti;
	}
	
	inpt->li = trapezoidal_dis(&inpt->tp, inpt->ti);

	for (i = 0; i < inpt->dimension; i++)
	{
		inpt->prePos[i] = inpt->curPos[i];
		inpt->curPos[i] = inpt->startPos[i] + inpt->li * inpt->vec[i];
	}
	grp->kine.pathLength=grp->kine.pathLength+inpt->li-inpt->preLi;

	double linearVel=(grp->inp.li-grp->inp.preLi)/grp->inp.dt;
	double linearAcc=(linearVel-grp->inp.linearVelocity)/grp->inp.dt;
	double linearjerk=(linearAcc-grp->inp.linearAcceleration)/grp->inp.dt;
	inpt->preLi=inpt->li;
	grp->inp.linearVelocity=linearVel;
	grp->inp.linearAcceleration=linearAcc;
	grp->inp.linearJerk=linearjerk;

	return 0;
}

int trajectoryLinearInterpolation(AxisGroupControl_t *grp,double rate)
{
    int ret=0;
    //int flag=1;
    //printf("Group: %d, trajectoryLinearInterpolation, grp->inp.state=%d\n", grp->cfg.grpId,grp->inp.state);
    if(grp->inp.state==0)
    {
      ret=initLinearInp(grp,rate);
      if(ret)
      {
            
          double linearVel=(grp->inp.li-grp->inp.preLi)/grp->inp.dt;
          double linearAcc=(linearVel-grp->inp.linearVelocity)/grp->inp.dt;
          double linearjerk=(linearAcc-grp->inp.linearAcceleration)/grp->inp.dt;
          grp->inp.preLi=grp->inp.li;
          grp->inp.linearVelocity=linearVel;
          grp->inp.linearAcceleration=linearAcc;
          grp->inp.linearJerk=linearjerk;
       
      }
      else
      {
       grp->inp.state=1;
       printf("Group: %d, initLinearInp, index=%d, syncCnt=%d\n", grp->cfg.grpId,grp->curInpTraj.index,grp->curInpTraj.syncCnt);
    
      }
    }
    switch (grp->inp.state)
    {

        case 1:	
          ret=linearInpterpolation(grp,rate);
          if(grp->inp.ti+grp->inp.dt * rate >grp->inp.tp.t || grp->inp.ti+ grp->inp.dt*rate < 0.0 )
          { 

                  if(QueueLengthToRear(&grp->trajectoryQ,grp->inpIndex)>1)
                  {
                          //队列仍有指令
                          grp->inp.state=0;
                          grp->curTraj_ptr->state=5; //插补完成
                  }
                  else
                  {
                          grp->inp.state=2;//进入最后一步插补
                  }
          }
          //printf("Group: %d, inp.state= %d, rate=%.3f, ti=%.3f, tp.t=%.3f\n", grp->cfg.grpId,grp->inp.state,rate,grp->inp.ti,grp->inp.tp.t); 
          break;
	case 2:
		ret=linearInpterpolation(grp,rate);
		if(ret)break;
		grp->inp.state=0;
		grp->curTraj_ptr->state=5; //插补完成
		//printf("inp finish ------------------------------------------\n");
		break;
    default:
		ULOG_ERROR("Group: %d, undefined inp.state= %d", grp->cfg.grpId,grp->inp.state);
        return 0 ;
    }

	return 0;
}

