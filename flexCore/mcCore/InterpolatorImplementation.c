#include <math.h>
#include <string.h>
#include "InterpolatorImplementation.h"
#include "VelocityPlanning.h"
#include "groupControl.h"

#define LINE_EPS 1.0e-9

/**
 * \brief 将数值限制在闭区间[low, high]。
 */
static double clampValue(double value, double low, double high)
{
	if (value < low) return low;
	if (value > high) return high;
	return value;
}

int prepareTrajectory(AxisGroupControl_t *grp,double rate,CmdTrajectory_t **trajOut)
{
	CmdTrajectory_t *p_traj;
    Inpterpolator_t *inpt=&grp->inp;
    int ret=0;
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
	if(trajOut)
	{
		*trajOut=p_traj;
	}
	return 0;
}

static int initLineInterpolatorCore(LineInterpolator_t *interp,
								const double startPos[3],
								const double endPos[3],
								const double startQua[4],
								const double endQua[4],
								double vMax,
								double wMax,
								double accMax,
								double decMax,
								double dt)
{
	double deltaPos[3];
	double lPos;
	double dot;
	double theta;
	double lRot;
	double lTotal;
	double q0Conj[4];
	double qRel[4];
	double sinHalfTheta;
	double q0n[4];
	double q1n[4];
	int i;

	if (interp == NULL || startPos == NULL || endPos == NULL || startQua == NULL || endQua == NULL)
	{
		return 1;
	}
	if (dt <= 0.0 || vMax < 0.0 || accMax <= 0.0 || decMax <= 0.0)
	{
		return 2;
	}
	if (quat_normalize(startQua, q0n) != 0 || quat_normalize(endQua, q1n) != 0)
	{
		return 3;
	}

	/* 清空运行态，避免旧周期残留状态影响新段初始化。 */
	memset(interp, 0, sizeof(LineInterpolator_t));

	for (i = 0; i < 3; i++)
	{
		deltaPos[i] = endPos[i] - startPos[i];
		interp->position.startPos[i] = startPos[i];
		interp->position.endPos[i] = endPos[i];
		interp->position.deltaPos[i] = deltaPos[i];
		interp->position.curPos[i] = startPos[i];
		interp->geom.startPose[i] = startPos[i];
		interp->geom.endPose[i] = endPos[i];
	}

	lPos = vec_norm(3, deltaPos);
	interp->position.posLength = lPos;
	interp->position.invPosLength = (lPos > LINE_EPS) ? (1.0 / lPos) : 0.0;
	if (lPos > LINE_EPS)
	{
		/* 几何方向向量（单位化） */
		interp->geom.dirVec[0] = deltaPos[0] / lPos;
		interp->geom.dirVec[1] = deltaPos[1] / lPos;
		interp->geom.dirVec[2] = deltaPos[2] / lPos;
		interp->position.dirPos[0] = interp->geom.dirVec[0];
		interp->position.dirPos[1] = interp->geom.dirVec[1];
		interp->position.dirPos[2] = interp->geom.dirVec[2];
	}
	else
	{
		interp->geom.dirVec[0] = 0.0;
		interp->geom.dirVec[1] = 0.0;
		interp->geom.dirVec[2] = 0.0;
		interp->position.dirPos[0] = 0.0;
		interp->position.dirPos[1] = 0.0;
		interp->position.dirPos[2] = 0.0;
	}

	quat_copy(q0n, interp->attitude.startQua);
	quat_copy(q1n, interp->attitude.endQua);

	/* 通过同半球对齐避免跨越四元数双覆盖导致的大角度插值。 */
	dot = clampValue(quat_dot(q0n, q1n), -1.0, 1.0);
	if (dot < 0.0)
	{
		interp->attitude.endQuaAligned[0] = -q1n[0];
		interp->attitude.endQuaAligned[1] = -q1n[1];
		interp->attitude.endQuaAligned[2] = -q1n[2];
		interp->attitude.endQuaAligned[3] = -q1n[3];
		dot = -dot;
	}
	else
	{
		quat_copy(q1n, interp->attitude.endQuaAligned);
	}

	dot = clampValue(dot, -1.0, 1.0);
	theta = 2.0 * acos(dot);

	/*
	 * 位姿同步映射：
	 * L_total = max(L_pos, L_rot)
	 * L_rot = theta / wMax * vMax
	 */
	if (wMax <= LINE_EPS)
	{
		lRot = (theta <= LINE_EPS) ? 0.0 : 1.0e30;
	}
	else
	{
		lRot = theta / wMax * vMax;
	}

	lTotal = fmax(lPos, lRot);
	interp->geom.totalLength = lTotal;

	interp->attitude.quaDot = dot;
	interp->attitude.theta = theta;
	if (theta > LINE_EPS)
	{
		quat_conjugate(interp->attitude.startQua, q0Conj);
		quat_multiply(interp->attitude.endQuaAligned, q0Conj, qRel);
		sinHalfTheta = sqrt(qRel[1] * qRel[1] + qRel[2] * qRel[2] + qRel[3] * qRel[3]);
		if (sinHalfTheta > LINE_EPS)
		{
			interp->attitude.rotAxis[0] = qRel[1] / sinHalfTheta;
			interp->attitude.rotAxis[1] = qRel[2] / sinHalfTheta;
			interp->attitude.rotAxis[2] = qRel[3] / sinHalfTheta;
		}
		else
		{
			interp->attitude.rotAxis[0] = 0.0;
			interp->attitude.rotAxis[1] = 0.0;
			interp->attitude.rotAxis[2] = 0.0;
		}
	}
	else
	{
		interp->attitude.rotAxis[0] = 0.0;
		interp->attitude.rotAxis[1] = 0.0;
		interp->attitude.rotAxis[2] = 0.0;
	}
	interp->attitude.angularVelocity = 0.0;
	interp->attitude.angularVelocityVec[0] = 0.0;
	interp->attitude.angularVelocityVec[1] = 0.0;
	interp->attitude.angularVelocityVec[2] = 0.0;
	interp->attitude.rotLength = lRot;
	quat_copy(interp->attitude.startQua, interp->attitude.curQua);

	interp->base.pathType = 1;
	interp->base.dimension = 3;
	interp->base.dt = dt;
	interp->base.ti = 0.0;
	interp->base.initTime = 0.0;
	interp->base.time = 0.0;
	interp->base.syncTime = 0.0;
	interp->base.li = 0.0;
	interp->base.preLi = 0.0;
	interp->base.state = 1;

	for (i = 0; i < 3; i++)
	{
		interp->base.startPos[i] = startPos[i];
		interp->base.endPos[i] = endPos[i];
		interp->base.curPos[i] = startPos[i];
		interp->base.prePos[i] = startPos[i];
		interp->position.curVel[i] = 0.0;
	}
	interp->position.linearVelocity = 0.0;

	interp->s = 0.0;
	interp->preS = 0.0;

	if (lTotal <= LINE_EPS)
	{
		/* 位置和姿态都无需运动，直接置完成状态。 */
		interp->base.tp.L = 0.0;
		interp->base.tp.t = 0.0;
		interp->base.linearVelocity = 0.0;
		interp->base.linearAcceleration = 0.0;
		interp->base.linearJerk = 0.0;
		interp->position.curVel[0] = 0.0;
		interp->position.curVel[1] = 0.0;
		interp->position.curVel[2] = 0.0;
		interp->position.linearVelocity = 0.0;
		interp->attitude.angularVelocity = 0.0;
		interp->attitude.angularVelocityVec[0] = 0.0;
		interp->attitude.angularVelocityVec[1] = 0.0;
		interp->attitude.angularVelocityVec[2] = 0.0;
		interp->base.state = 2;
		return 0;
	}

	if (trapezoidal_profile(&interp->base.tp, lTotal, 0.0, vMax, 0.0, accMax, decMax, dt) != 0)
	{
		return 4;
	}

	return 0;
}

int executeLineInterpolator(LineInterpolator_t *interp,
							double rate,
							double outPos[3],
							double outQua[4],
							int *isFinished)
{
	double ti;
	double dtime;
	double lTotal;
	double linearVel;
	double linearAcc;
	double linearJerk;
	double ds;
	double dsdt;
	int i;

	if (interp == NULL)
	{
		return 1;
	}
	if (interp->base.dt <= 0.0)
	{
		return 2;
	}

	lTotal = interp->base.tp.L;
	if (lTotal <= LINE_EPS)
	{
		/* 退化段：直接输出终点位姿。 */
		for (i = 0; i < 3; i++)
		{
			interp->position.curPos[i] = interp->position.endPos[i];
			interp->base.prePos[i] = interp->base.curPos[i];
			interp->base.curPos[i] = interp->position.endPos[i];
		}
		quat_copy(interp->attitude.endQuaAligned, interp->attitude.curQua);
		interp->preS = interp->s;
		interp->s = 1.0;
		interp->position.curVel[0] = 0.0;
		interp->position.curVel[1] = 0.0;
		interp->position.curVel[2] = 0.0;
		interp->position.linearVelocity = 0.0;
		interp->attitude.angularVelocity = 0.0;
		interp->attitude.angularVelocityVec[0] = 0.0;
		interp->attitude.angularVelocityVec[1] = 0.0;
		interp->attitude.angularVelocityVec[2] = 0.0;
		interp->base.linearVelocity = 0.0;
		interp->base.linearAcceleration = 0.0;
		interp->base.linearJerk = 0.0;
		interp->base.state = 2;
		if (outPos != NULL)
		{
			outPos[0] = interp->position.curPos[0];
			outPos[1] = interp->position.curPos[1];
			outPos[2] = interp->position.curPos[2];
		}
		if (outQua != NULL)
		{
			quat_copy(interp->attitude.curQua, outQua);
		}
		if (isFinished != NULL)
		{
			*isFinished = 1;
		}
		return 0;
	}

	ti = interp->base.ti + interp->base.dt * rate;
	ti = clampValue(ti, 0.0, interp->base.tp.t);

	dtime = ti - interp->base.ti;
	interp->base.syncTime = interp->base.time;
	interp->base.time = interp->base.syncTime + dtime;
	interp->base.ti = ti;
	interp->base.initTime = (rate >= 0.0) ? (interp->base.ti - interp->base.tp.t) : interp->base.ti;

	interp->base.preLi = interp->base.li;
	interp->base.li = trapezoidal_dis(&interp->base.tp, interp->base.ti);
	interp->preS = interp->s;
	/* 标量路径长度归一化：s(t)=x(t)/L_total。 */
	interp->s = clampValue(interp->base.li / lTotal, 0.0, 1.0);
	ds = interp->s - interp->preS;
	dsdt = (fabs(dtime) > LINE_EPS) ? (ds / dtime) : 0.0;

	/* 位置线性插补：p(t)=p0+s*(p1-p0)。 */
	for (i = 0; i < 3; i++)
	{
		interp->base.prePos[i] = interp->base.curPos[i];
		interp->position.curPos[i] = interp->position.startPos[i] + interp->s * interp->position.deltaPos[i];
		interp->base.curPos[i] = interp->position.curPos[i];
		interp->position.curVel[i] = interp->position.deltaPos[i] * dsdt;
	}
	interp->position.linearVelocity = vec_norm(3, interp->position.curVel);

	/* 姿态同步插补：q(t)=SLERP(q0,q1,s)。 */
	if (quat_slerp(interp->attitude.startQua, interp->attitude.endQuaAligned, interp->s, interp->attitude.curQua) != 0)
	{
		return 3;
	}
	interp->attitude.angularVelocity = interp->attitude.theta * dsdt;
	interp->attitude.angularVelocityVec[0] = interp->attitude.rotAxis[0] * interp->attitude.angularVelocity;
	interp->attitude.angularVelocityVec[1] = interp->attitude.rotAxis[1] * interp->attitude.angularVelocity;
	interp->attitude.angularVelocityVec[2] = interp->attitude.rotAxis[2] * interp->attitude.angularVelocity;

	linearVel = (interp->base.li - interp->base.preLi) / interp->base.dt;
	linearAcc = (linearVel - interp->base.linearVelocity) / interp->base.dt;
	linearJerk = (linearAcc - interp->base.linearAcceleration) / interp->base.dt;

	interp->base.linearVelocity = linearVel;
	interp->base.linearAcceleration = linearAcc;
	interp->base.linearJerk = linearJerk;

	if (ti >= interp->base.tp.t - LINE_EPS)
	{
		interp->base.state = 2;
	}
	else if (ti <= LINE_EPS)
	{
		interp->base.state = 0;
	}
	else
	{
		interp->base.state = 1;
	}

	if (outPos != NULL)
	{
		outPos[0] = interp->position.curPos[0];
		outPos[1] = interp->position.curPos[1];
		outPos[2] = interp->position.curPos[2];
	}
	if (outQua != NULL)
	{
		quat_copy(interp->attitude.curQua, outQua);
	}
	if (isFinished != NULL)
	{
		*isFinished = (interp->base.state == 2) ? 1 : 0;
	}

	return 0;
}

/**
 * \brief 参考 Interpolation.c 的轨迹选段逻辑，定位当前可执行轨迹。
 */
static int selectLineTrajectory(AxisGroupControl_t *grp, double rate, CmdTrajectory_t **outTraj)
{
	CmdTrajectory_t *p_traj;

	if (grp == NULL || outTraj == NULL)
	{
		return 1;
	}

	p_traj = (CmdTrajectory_t *)getCurrentElem(&grp->trajectoryQ, grp->inpIndex);
	if (p_traj == NULL || p_traj->state < 3)
	{
		grp->inp.state = 0;
		return 2;
	}

	if (rate > 0.0)
	{
		if (fabs(grp->inp.time - p_traj->time) < 1.001e-3)
		{
			if (!isQueueRear(&grp->trajectoryQ, grp->inpIndex))
			{
				grp->inp.preLi = grp->inp.li - p_traj->tp.L;
				grp->inpIndex = getNextIndex(&grp->trajectoryQ, grp->inpIndex);
				grp->smoothIndex = grp->inpIndex;
				p_traj = (CmdTrajectory_t *)getCurrentElem(&grp->trajectoryQ, grp->inpIndex);
				if (p_traj == NULL || p_traj->state < 3)
				{
					grp->inp.state = 0;
					return 3;
				}
			}
			else
			{
				grp->inp.state = 0;
				return 4;
			}
		}
	}
	else
	{
		if (fabs(p_traj->time - grp->inp.time - p_traj->tp.t) < MAX_TIME_ERR)
		{
			if (!isQueueHead(&grp->trajectoryQ, grp->inpIndex))
			{
				grp->inpIndex = getPreviousIndex(&grp->trajectoryQ, grp->inpIndex);
				p_traj = (CmdTrajectory_t *)getCurrentElem(&grp->trajectoryQ, grp->inpIndex);
				if (p_traj == NULL)
				{
					grp->inp.state = 0;
					return 5;
				}
				grp->inp.ti = grp->inp.initTime + p_traj->tp.t;
				grp->inp.preLi = grp->inp.li + p_traj->tp.L;
			}
			else
			{
				grp->inp.state = 0;
				return 6;
			}
		}
	}

	*outTraj = p_traj;
	return 0;
}

int initLineInterpolator(AxisGroupControl_t *grp, double rate)
{
	CmdTrajectory_t *p_traj = NULL;
	LineInterpolator_t *interp;
	double p0[3];
	double p1[3];
	double q0[4] = { 1.0, 0.0, 0.0, 0.0 };
	double q1[4] = { 1.0, 0.0, 0.0, 0.0 };
	double dt;
	double vMax;
	double wMax;
	double accMax;
	double decMax;
	int i;
	int ret;

	if (grp == NULL)
	{
		return 1;
	}

	interp = &grp->unifiedInp.linear;

	ret = selectLineTrajectory(grp, rate, &p_traj);
	if (ret != 0)
	{
		return ret;
	}

	for (i = 0; i < 3; i++)
	{
		p0[i] = p_traj->startPos[i];
		p1[i] = p_traj->pos[i];
	}

	dt = (grp->inp.dt > 0.0) ? grp->inp.dt : grp->cyclePeriod;
	if (dt <= 0.0)
	{
		dt = CONTROL_PERIOD_SEC;
	}

	vMax = (p_traj->vel > 0.0) ? p_traj->vel : grp->usrParam.velMax;
	accMax = (p_traj->acc > 0.0) ? p_traj->acc : grp->usrParam.accMax;
	decMax = (p_traj->dec > 0.0) ? p_traj->dec : grp->usrParam.accMax;
	wMax = (grp->usrParam.wMax > LINE_EPS) ? grp->usrParam.wMax : grp->usrParam.velMax;
	if (wMax <= LINE_EPS)
	{
		wMax = 1.0;
	}

	ret = initLineInterpolatorCore(interp, p0, p1, q0, q1, vMax, wMax, accMax, decMax, dt);
	if (ret != 0)
	{
		return 10 + ret;
	}

	grp->inp.cmdNo = p_traj->cmdNo;
	grp->inp.index = p_traj->index;
	grp->inp.dimension = 3;
	grp->inp.pathType = p_traj->pathType;
	grp->inp.dt = interp->base.dt;
	grp->inp.ti = interp->base.ti;
	grp->inp.initTime = interp->base.initTime;
	grp->inp.time = interp->base.time;
	grp->inp.syncTime = interp->base.syncTime;
	grp->inp.length = interp->base.tp.L;
	grp->inp.li = interp->base.li;
	grp->inp.preLi = interp->base.preLi;
	grp->inp.state = 1;
	grp->inp.tp = interp->base.tp;

	for (i = 0; i < 3; i++)
	{
		grp->inp.startPos[i] = p0[i];
		grp->inp.endPos[i] = p1[i];
		grp->inp.prePos[i] = p0[i];
		grp->inp.curPos[i] = p0[i];
		grp->inp.vec[i] = interp->geom.dirVec[i];
	}

	p_traj->state = 4;
	grp->curTraj_ptr = p_traj;
	memcpy(&grp->curInpTraj, p_traj, sizeof(CmdTrajectory_t));
	grp->delSyncCnt = p_traj->syncCnt;

	return 0;
}

int initLinePoseInp(AxisGroupControl_t *grp,
					LineInterpolator_t *interp,
					double rate,
					const double startQua[4],
					const double endQua[4],
					double wMax)
{
	CmdTrajectory_t *p_traj = NULL;
	double p0[3];
	double p1[3];
	double q0[4] = { 1.0, 0.0, 0.0, 0.0 };
	double q1[4] = { 1.0, 0.0, 0.0, 0.0 };
	double dt;
	double vMax;
	double accMax;
	double decMax;
	int i;
	int ret;

	if (grp == NULL || interp == NULL)
	{
		return 1;
	}

	ret = selectLineTrajectory(grp, rate, &p_traj);
	if (ret != 0)
	{
		return ret;
	}

	for (i = 0; i < 3; i++)
	{
		p0[i] = p_traj->startPos[i];
		p1[i] = p_traj->pos[i];
	}

	if (startQua != NULL)
	{
		quat_copy(startQua, q0);
	}
	if (endQua != NULL)
	{
		quat_copy(endQua, q1);
	}

	dt = (grp->inp.dt > 0.0) ? grp->inp.dt : grp->cyclePeriod;
	if (dt <= 0.0)
	{
		dt = CONTROL_PERIOD_SEC;
	}

	vMax = (p_traj->vel > 0.0) ? p_traj->vel : grp->usrParam.velMax;
	accMax = (p_traj->acc > 0.0) ? p_traj->acc : grp->usrParam.accMax;
	decMax = (p_traj->dec > 0.0) ? p_traj->dec : grp->usrParam.accMax;
	if (wMax <= LINE_EPS)
	{
		wMax = grp->usrParam.velMax;
		if (wMax <= LINE_EPS)
		{
			wMax = 1.0;
		}
	}

	ret = initLineInterpolatorCore(interp, p0, p1, q0, q1, vMax, wMax, accMax, decMax, dt);
	if (ret != 0)
	{
		return 10 + ret;
	}

	grp->inp.cmdNo = p_traj->cmdNo;
	grp->inp.index = p_traj->index;
	grp->inp.dimension = 3;
	grp->inp.pathType = p_traj->pathType;
	grp->inp.dt = interp->base.dt;
	grp->inp.ti = interp->base.ti;
	grp->inp.initTime = interp->base.initTime;
	grp->inp.time = interp->base.time;
	grp->inp.syncTime = interp->base.syncTime;
	grp->inp.length = interp->base.tp.L;
	grp->inp.li = interp->base.li;
	grp->inp.preLi = interp->base.preLi;
	grp->inp.state = 1;
	grp->inp.tp = interp->base.tp;

	for (i = 0; i < 3; i++)
	{
		grp->inp.startPos[i] = p0[i];
		grp->inp.endPos[i] = p1[i];
		grp->inp.prePos[i] = p0[i];
		grp->inp.curPos[i] = p0[i];
		grp->inp.vec[i] = interp->geom.dirVec[i];
	}

	p_traj->state = 4;
	grp->curTraj_ptr = p_traj;
	memcpy(&grp->curInpTraj, p_traj, sizeof(CmdTrajectory_t));
	grp->delSyncCnt = p_traj->syncCnt;

	return 0;
}

int linePoseInterpolation(AxisGroupControl_t *grp,
						  LineInterpolator_t *interp,
						  double rate,
						  double outQua[4])
{
	double outPos[3] = { 0.0, 0.0, 0.0 };
	int isFinished = 0;
	int i;
	int ret;

	if (grp == NULL || interp == NULL)
	{
		return 1;
	}

	ret = executeLineInterpolator(interp, rate, outPos, outQua, &isFinished);
	if (ret != 0)
	{
		return ret;
	}

	grp->inp.syncTime = interp->base.syncTime;
	grp->inp.time = interp->base.time;
	grp->inp.ti = interp->base.ti;
	grp->inp.initTime = interp->base.initTime;
	grp->inp.preLi = interp->base.preLi;
	grp->inp.li = interp->base.li;
	grp->inp.linearVelocity = interp->base.linearVelocity;
	grp->inp.linearAcceleration = interp->base.linearAcceleration;
	grp->inp.linearJerk = interp->base.linearJerk;

	for (i = 0; i < 3; i++)
	{
		grp->inp.prePos[i] = interp->base.prePos[i];
		grp->inp.curPos[i] = interp->base.curPos[i];
	}

	grp->kine.pathLength = grp->kine.pathLength + interp->base.li - interp->base.preLi;
	grp->inp.state = isFinished ? 2 : 1;

	return 0;
}

int trajectoryLinePoseInterpolation(AxisGroupControl_t *grp,
									LineInterpolator_t *interp,
									double rate,
									const double startQua[4],
									const double endQua[4],
									double wMax,
									double outQua[4])
{
	int ret = 0;

	if (grp == NULL || interp == NULL)
	{
		return 1;
	}

	if (grp->inp.state == 0)
	{
		ret = initLinePoseInp(grp, interp, rate, startQua, endQua, wMax);
		if (ret != 0)
		{
			return ret;
		}
		grp->inp.state = 1;
	}

	switch (grp->inp.state)
	{
	case 1:
		ret = linePoseInterpolation(grp, interp, rate, outQua);
		if (ret != 0)
		{
			return ret;
		}
		if (grp->inp.ti + grp->inp.dt * rate > grp->inp.tp.t || grp->inp.ti + grp->inp.dt * rate < 0.0)
		{
			if (QueueLengthToRear(&grp->trajectoryQ, grp->inpIndex) > 1)
			{
				grp->inp.state = 0;
				if (grp->curTraj_ptr != NULL)
				{
					grp->curTraj_ptr->state = 5;
				}
			}
			else
			{
				grp->inp.state = 2;
			}
		}
		break;
	case 2:
		ret = linePoseInterpolation(grp, interp, rate, outQua);
		if (ret != 0)
		{
			return ret;
		}
		grp->inp.state = 0;
		if (grp->curTraj_ptr != NULL)
		{
			grp->curTraj_ptr->state = 5;
		}
		break;
	default:
		return 2;
	}

	return 0;
}
