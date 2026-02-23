#include "onlinetrapeprofile.h"
#include <math.h>
#include <string.h>
#include <stdio.h>
#include "axisControl.h"	
/**
 * \brief 符号函数.
 *
 * \param x	需要判断符号的变量
 * \return  与符号相关的值1,0,-1.
 */
double sign(double x)
{
	double y = 0;
	if (x > 1.0e-5)
		y = 1.0;
	else if (x < -1.0e-5)
		y = -1.0;
	else
		y = 0;
	return y;
}

/**
 * \brief 饱和截止函数.
 *
 * \param x  输入变量
 * \param u1 限制下限
 * \param u2 限制上限
 * \return   饱和限制后的输出值
 */
double saturation(double x, double u1, double u2)
{
	double y = 0;
	if (x < u1)
		y = u1;
	else if (x > u2)
		y = u2;
	else
		y = x;
	return y;
}

/**
 * \brief 参数更新函数.
 *
 * \param param	在线梯形规划参数结构体
 * \param tp 在线梯形速度规划器
 * \return
 */
int onlineTrapePlannerUpdateParam(OnlineTrapePlannerParam_t* param, OnlineTrapePlanner_t* tp)
{
	memcpy(&(tp->param), param, sizeof(OnlineTrapePlannerParam_t));
	tp->qtNext = param->qLast;
	tp->state = 1;
	return 0;

}

/**
 * \brief 更新梯形规划的目标位置.
 *
 * \param tp
 * \param rk
 * \return
 */
int onlineTrapePlannerSetRef(OnlineTrapePlanner_t* tp, double rk)
{
	tp->param.rk = rk;
	tp->state = 2;
	return 0;
}

/**
 * \brief 更新梯形规划的速度约束.
 *
 * \param tp
 * \param vel
 * \return
 */
int onlineTrapePlannerSetVel(OnlineTrapePlanner_t* tp, double vel)
{
	tp->param.velMax = vel;
	tp->state = 2;
	return 0;
}

/**
 * \brief 更新梯形规划的加速度约束.
 *
 * \param tp
 * \param vel
 * \return
 */
int onlineTrapePlannerSetAcc(OnlineTrapePlanner_t* tp, double acc)
{
	tp->param.accMax = acc;
	tp->tolerance=fmax(1.1*0.5*tp->param.accMax*tp->param.Ts*tp->param.Ts,1.0e-4);
	tp->state = 2;
	return 0;
}

int onlineTrapePlannerSetTarget(OnlineTrapePlanner_t* tp, double targetPos,double velocity,
	double acceleration,double deceleration)
{
	(void)deceleration;
	tp->param.rk=targetPos;
	tp->param.velMax=velocity;
	tp->param.accMax = acceleration;
	tp->tolerance=fmax(1.1*0.5*tp->param.accMax*tp->param.Ts*tp->param.Ts,1.0e-4);
	tp->state = 2;
	tp->count=0;
	return 0;
}


/**
 * \brief 按周期更新规划器的输出.
 *
 * \param tp 在线梯形速度规划器
 * \param dl 下个周期的位移
 * \param delta 下个周期的相对位移
 * \return	 返回0执行成功，返回非0执行失败。
 */
int onlineTrapePlannerUpdateOutput(OnlineTrapePlanner_t* tp, double* dl, double* delta)
{
	if (tp->state == 0)return 1;
	if (fabs(tp->param.rk - tp->qtNext) < tp->tolerance&& fabs(tp->dqtNext)<tp->tolerance)
	{
		tp->dqtNext = 0.0;
		tp->qtNext = tp->param.rk;

		tp->delta = 0.0;
		tp->param.dqLast = tp->dqtNext;
		tp->param.qLast = tp->qtNext;

		*dl = tp->qtNext;
		*delta = tp->delta;
		tp->state = 1;
		return 0;
	}

	tp->state = 2;
	double qtLast = tp->param.qLast;
	double dqtLast = tp->param.dqLast;
	double rk = tp->param.rk;
	double vmax = tp->param.velMax;
	double Ts = tp->param.Ts;
	double U = tp->param.accMax;
	double drk = tp->param.drk;
	double ek = (qtLast - rk) / U;
	double dek = (dqtLast - drk) / U;
	double zk = 1.0 / Ts * (ek / Ts + dek / 2.0);
	double dzk = dek / Ts;
	int m = (int)((1.0 + sqrt(1.0 + 8.0 * (double)fabs(zk))) / 2.0f);
	double delatak = dzk + zk / m + (m - 1.0) / 2.0f * sign(zk);
	double uk = -U * saturation(delatak, -1.0f, 1.0f) * (1.0f + sign(dqtLast * sign(delatak) + vmax - U * Ts)) / 2.0;
	tp->dqtNext = dqtLast + Ts * uk;
	tp->qtNext = qtLast + Ts / 2.0 * (tp->dqtNext + dqtLast);
	tp->delta = tp->qtNext - qtLast;
	tp->param.dqLast = tp->dqtNext;
	tp->param.qLast = tp->qtNext;

	*dl = tp->qtNext;
	*delta = tp->delta;
	return 0;
}

int onlineTrapePlannerUpdate(OnlineTrapePlanner_t* tp, double nextState[3])
{
	if (0==tp->state)return 1;
	if(1==tp->state)
	{
		nextState[0] = tp->qtNext;
		nextState[1] = tp->dqtNext;
		nextState[2] = 0.0;
		return 0;
	} 
	if (fabs(tp->param.rk - tp->qtNext) < tp->tolerance && fabs(tp->dqtNext)<0.01&& tp->count++>5)
	{
		tp->ddqtNext=0.0;
		tp->dqtNext = 0.0;
		tp->qtNext = tp->param.rk;

		tp->delta = 0.0;
		tp->param.dqLast = tp->dqtNext;
		tp->param.qLast = tp->qtNext;

		nextState[0] = tp->qtNext;
		nextState[1] = tp->dqtNext;
		nextState[2] = tp->ddqtNext;
		tp->state = 1;
		tp->count=0;
		return 0;
	}
	tp->state = 2;
	double qtLast = tp->param.qLast;
	double dqtLast = tp->param.dqLast;
	double rk = tp->param.rk;
	double vmax = tp->param.velMax;
	double Ts = tp->param.Ts;
	double U = tp->param.accMax;
	double drk = tp->param.drk;
	double ek = (qtLast - rk) / U;
	double dek = (dqtLast - drk) / U;
	double zk = 1.0 / Ts * (ek / Ts + dek / 2.0);
	double dzk = dek / Ts;
	int m = (int)((1.0 + sqrt(1.0 + 8.0 * (double)fabs(zk))) / 2.0f);
	double delatak = dzk + zk / m + (m - 1.0) / 2.0f * sign(zk);
	double uk = -U * saturation(delatak, -1.0f, 1.0f) * (1.0f + sign(dqtLast * sign(delatak) + vmax - U * Ts)) / 2;
	tp->dqtNext = dqtLast + Ts * uk;
	tp->qtNext = qtLast + Ts / 2.0 * (tp->dqtNext + dqtLast);

	nextState[0] = tp->qtNext;
	nextState[1] = tp->dqtNext;
	nextState[2] = uk;

	tp->delta = tp->qtNext - qtLast;
	tp->param.ddqLast = nextState[2];
	tp->param.dqLast = nextState[1];
	tp->param.qLast = nextState[0];
	return 0;
}


/**
 * \brief 直线插补器初始化
 * 
 * \param StartPos 起始位置。
 * \param EndPos	结束位置。
 * \param dim		插补坐标的维数，1-6维。
 * \param li		直线插补器结构体。
 * \return 
 */
int linearInterpolatorInit(double* StartPos, double* EndPos, double Ts,int dim, LinearInterpolator_t* li)
{
	int i = 0;
	double len = 0.0;
	li->dim = dim;
	li->Ts = Ts;
	for (i = 0; i < dim; i++)
	{
		li->sPos[i] = StartPos[i];
		li->ePos[i] = EndPos[i];
        
		li->lastPos[i] = li->sPos[i];//下一个位置赋值为初值，插补前保持在当前位置
	}
	for (i = 0; i < dim; i++)
		len = len + (EndPos[i] - StartPos[i]) * (EndPos[i] - StartPos[i]);
	if (len < 1.0e-5)
	{
		len = 0.0;
		li->totalLen = len;
		for (i = 0; i < dim; i++)
			li->vec[0] = 0.0;
	}
	else
	{
		len = sqrt(len);
		li->totalLen = len;
		for (i = 0; i < dim; i++)
			li->vec[i] = (EndPos[i] - StartPos[i]) / len;
	}
	li->nextLen = 0.0;
	li->state = 1;
	return 0;
}


/**
 * \brief 直线插补器更新
 * 
 * \param dl	下个周期相对位移。
 * \param nextPos 下个周期的坐标。
 * \param li      插补器结构体。
 * \return 
 */
int linearInterpolatorUpdate(double dl, double* nextPos, LinearInterpolator_t* li)
{
	int i;
	li->nextLen = li->nextLen + dl;
	for (i = 0; i < li->dim; i++)
	{
		nextPos[i] = li->sPos[i] + li->nextLen * li->vec[i];
		li->vel[i] = (nextPos[i] - li->lastPos[i]) / li->Ts;
		li->lastPos[i] = nextPos[i];
	}
	return 0;
}



/**
 * \brief 在线跟踪控制器始化。
 * 
 * \param ottp	在线目标跟踪控制器的参数。
 * \param ott	目标跟踪规划器结构体。
 * \return		返回0成功，非零失败。
 * \retval 1    已被初始化。
 */
int onlineTargetTrackingInit(OnlineTargetTrackParam_t* ottp, OnlineTargetTracking_t* ott)
{
	int i = 0;
	int ret;
	double sPos[6];
	double ePos[6];
	OnlineTrapePlannerParam_t param;
	//记录初始化参数到目标跟踪结构体
	memcpy(&ott->ottp, ottp, sizeof(OnlineTargetTrackParam_t));
	//路径参数更新
	//if (ott->state == 1)return 1;
	for (i = 0; i < 6; i++)
	{
		sPos[i] = ottp->curPos[i];
		ottp->refPos[i] = ottp->curPos[i];
		ePos[i] = ottp->refPos[i];
	}
	ret = linearInterpolatorInit(sPos, ePos, ottp->Ts, 6, & ott->li);
	if (ret)
	{
		printf("LinearInterpolatorInit error \n");
		return ret;
	}
	param.rk = ott->li.totalLen;
	param.drk = ottp->refVel;
	param.qLast = 0.0f;
	param.dqLast = ottp->curVel;
	param.accMax = ottp->accMax;
	param.velMax = ottp->velMax;
	param.Ts = ottp->Ts;
	ret = onlineTrapePlannerUpdateParam(&param, &ott->otp);
	if (ret)
	{
		return ret;
	}
	else
	{
		printf("OnlineTrapePlannerUpdateParam finish\n");
	}
	ott->state = 1;
	return 0;
}


/**
 * \brief 在线目标跟踪规划器参数在线更新
 * 
 * \param targetPos 新的目标位置。
 * \param ott		规划器结构体。
 * \return          返回0成功，非零失败。
 * \retval			返回1，规划器未初始化，请初始化。
 */
int onlineTargetTrackingUpdateParam(double *targetPos, OnlineTargetTracking_t* ott)
{
	int i;
	int ret;
	double sPos[6];
	if (ott->state == 0)return 1;

	//路径参数更新
	OnlineTrapePlannerParam_t param;
	for (i = 0; i < ott->li.dim; i++)
	{
		sPos[i] = ott->li.lastPos[i];
	}
		
	ret = linearInterpolatorInit(sPos, targetPos, ott->ottp.Ts,ott->li.dim, &ott->li);
	if (ret)
	{
		printf("LinearInterpolatorInit error ret=%d\n", ret);
		return ret;
	}

	//目标位置更新
	memcpy(&param, &ott->otp, sizeof(OnlineTrapePlannerParam_t));
	param.rk = ott->li.totalLen;
	param.qLast = 0;
	ret = onlineTrapePlannerUpdateParam(&param, &ott->otp);
	if (ret)
	{
		printf("OnlineTrapePlannerUpdateParam error ret=%d\n", ret);
		return ret;
	}
	return 0;
}

/**
 * \brief 目标跟踪控制器输出更新。
 * 
 * \param ott	跟踪控制器结构体。
 * \param nextPos 输出当前周期插补坐标。
 * \param vel     输出周期周期速度
 * \return 返回0执行成功，返回非零执行失败
 * \retval 1 目标跟踪控制器未初始化，请先初始化。
 */
int onlineTargetTrackingUpdateOutput(OnlineTargetTracking_t* ott, double nextPos[6], double* vel)
{
	double dl, delta;
	int ret;
	if (ott->state == 0)
		return 1;
	//正常跟踪状态
	ret = onlineTrapePlannerUpdateOutput(&ott->otp, &dl, &delta);
	if (ret)
	{
		printf("OnlineTrapePlannerUpdateOutput error!\n");
		return ret;
	}
	delta = fabs(delta);
	ret = linearInterpolatorUpdate(delta, nextPos, &ott->li);
	if (ret)
	{
		printf("LinearInterpolatorUpdate error!\n");
		return ret;
	}
	*vel = ott->otp.dqtNext;


	return 0;
}


/**
 * \brief 目标跟踪功能开启或关闭。
 *
 * \param ott 目标跟踪控制器结构体。
 * \param enable	使能标志，1--开启，0--立即停止
 * \return 返回0执行成功，返回非零执行失败
 * \retval 0 执行成功
 * \retval 1 目标跟踪控制结构体未初始化。
 */
int setOnlineTargetTrackingEnable(OnlineTargetTracking_t* ott, int enable)
{
	if (ott->state == 0)
		return 1;
	if (enable == 0)//立即停止跟踪
	{
		ott->enable = enable;
		ott->otp.dqtNext = 0;

	}
	else if (enable == 1)//开启跟踪
	{
		ott->enable = enable;
	}
	return 0;
}

/**
 * .\brief 获取在线跟踪控制器的当前规划位置。
 * 
 * \param ott	在线目标跟踪结构体
 * \param CurPos 当前规划位置
 * \return 
 */
int getOnlineTargetTrackingCurPos(OnlineTargetTracking_t* ott, double CurPos[6])
{
	int i = 0;
	for (i = 0; i < ott->li.dim; i++)
		CurPos[i] = ott->li.lastPos[i];
	return 0;
}

/**
 * \brief 获取当前的指令位置.
 * 
 * \param ott	在线目标跟踪结构体。
 * \param RefPos	返回当前指令位置（参考位置）。
 * \return 返回0执行成功，非0执行失败。
 */
int getOnlineTargetTrackingRefPos(OnlineTargetTracking_t* ott, double RefPos[6])
{
	int i = 0;
	for(i=0;i< ott->li.dim;i++)
		RefPos[i]=ott->li.ePos[i];
	return 0;
}

/**
 * .\brief 获取线跟踪控制器的当前规划速度
 * 
 * \param ott	在线目标跟踪结构体。
 * \param vel   当前规划位置。
 * \return 
 */
int getOnlineTargetTrackingVel(OnlineTargetTracking_t* ott, double *vel)
{
	*vel = ott->otp.dqtNext;
	return 0;
}


/**
 * \brief 参数更新函数.
 *
 * \param param	在线梯形规划参数结构体
 * \param tp 在线梯形速度规划器
 * \return
 */
int onlineScurvePlannerUpdateParam(OnlineScurvePlannerParam_t* param, OnlineScurvePlanner_t* tp)
{
	memcpy(&(tp->param), param, sizeof(OnlineScurvePlannerParam_t));
	tp->qtNext = param->qLast;
	tp->dqtNext = param->dqLast;
	tp->ddqtNext = param->ddqLast;
	tp->state = 1;
	return 0;

}

/**
 * \brief 更新S形规划的目标位置.
 *
 * \param tp
 * \param rk
 * \return
 */
int onlineScurvePlannerSetRef(OnlineScurvePlanner_t* sp, double rk)
{
	sp->param.rk = rk;
	sp->state=2;
	return 0;
}

/**
 * \brief 更新s形规划的速度约束.
 *
 * \param sp
 * \param vel
 * \return
 */
int onlineScurvePlannerSetVel(OnlineScurvePlanner_t* sp, double vel)
{
	sp->param.velMax = vel;
	sp->state=2;
	return 0;
}

/**
 * \brief 更新S形规划的加速度约束.
 *
 * \param sp
 * \param acc
 * \return
 */
int onlineScurvePlannerSetAcc(OnlineScurvePlanner_t* sp, double acc)
{
	sp->param.accMax = acc;
	sp->tolerance=fmax(1.1*1.0/6.0*sp->param.jerkMax*sp->param.Ts*sp->param.Ts*sp->param.Ts, MAX_OTG_POS_ERR);
	sp->state=2;
	return 0;
}

int onlineScurePlannerSetTarget(OnlineScurvePlanner_t* sp,double targtPos,double velocity,
	double acceleration,double deceleration,double jerk)
{
	(void)deceleration;
	sp->param.rk = targtPos;
	sp->param.velMax = velocity;
	sp->param.accMax = acceleration;
	sp->param.jerkMax=jerk;
	sp->tolerance=fmax(1.1*1.0/6.0*sp->param.jerkMax*sp->param.Ts*sp->param.Ts*sp->param.Ts, MAX_OTG_POS_ERR);
	sp->count=0;
	sp->qE_k_1=sp->param.rk - sp->qtNext;
	sp->qE_k=0.0;
	sp->state=2;
	return 0;

}

int onlineScurvePlannerUpdate(OnlineScurvePlanner_t* sp, double nextState[3])
{

	if (0==sp->state )return 1;
	else if (1==sp->state)
	{
		nextState[0] = sp->qtNext;
		nextState[1] = sp->dqtNext;
		nextState[2] = 0.0;
		return 0;
	}
	if (fabs(sp->param.rk - sp->qtNext) < sp->tolerance\
	||(fabs(sp->qE_k - sp->qE_k_1) < 1.0e-5 && sp->count++>5 )) //足够接近目标时，停止计算更新
	{
		sp->ddqtNext=0.0;
		sp->dqtNext = 0.0;
		sp->qtNext = sp->param.rk;
		sp->delta = 0.0;
		sp->param.dqLast = sp->dqtNext;
		sp->param.qLast = sp->qtNext;

		nextState[0] = sp->qtNext;
		nextState[1] = sp->dqtNext;
		nextState[2] = 0.0;
		sp->state = 1;
		sp->count=0;
		return 0;
	}
	 
	double rk= sp->param.rk;
	double drk= sp->param.drk;
	double ddrk= sp->param.ddrk;
	double qLast = sp->qtNext;
	double dqLast = sp->dqtNext;
	double ddqLast = sp->ddqtNext;
	double vmax = sp->param.velMax;
	double amax = sp->param.accMax;
	double U= sp->param.jerkMax;
	double Ts= sp->param.Ts;
	//3阶滤波实现在线轨迹跟踪
	double 	ek = (qLast - rk) / U; // ‘误差’
	double	dek = (dqLast - drk) / U; // ‘误差’导数
	double	ddek = (ddqLast - ddrk) / U;

	double demin = (-vmax - drk) / U;
	double demax = (vmax - drk) / U;
	double ddemin = (-amax - ddrk) / U;
	double ddemax = (amax - ddrk) / U;
	//开始计算sigma
	double delta = dek + 0.5 * ddek * fabs(ddek);
	double sigma = ek + dek * ddek * sign(delta) - ddek * ddek * ddek * (1.0 - 3.0 * fabs(sign(delta))) / 6.0 + sign(delta) * sqrt(2.0 * pow(ddek * ddek + 2.0 * dek * sign(delta), 3.0)) / 4.0;
	if (ddek <= ddemax && dek <= 0.5 * ddek * ddek - ddemax * ddemax)
	{
		sigma = ek - (ddemax * (ddek * ddek - 2.0 * dek)) / 4.0 - pow((ddek * ddek - 2.0 * dek), 2.0) / (8.0 * ddemax) - ddek * (3.0 * dek - ddek * ddek) / 3.0;
	}
	if (ddek >= ddemin && dek >= ddemin * ddemin - 0.5 * ddek * ddek)
	{
		sigma = ek - (ddemin * (ddek * ddek + 2.0 * dek)) / 4.0 - pow((ddek * ddek + 2.0 * dek), 2.0) / (8.0 * ddemin) + ddek * (3.0 * dek + ddek * ddek) / 3.0;
	}
	//开始计算采用的加加速度uk
	double uc = -U * sign(sigma + (1.0 - fabs(sign(sigma))) * (delta + (1.0 - fabs(sign(delta))) * ddek));
	double uaddemin = -U * sign(ddek - ddemin);
	double uaddemax = -U * sign(ddek - ddemax);
	double vvmin = ddek * fabs(ddek) + 2.0 * (dek - demin);
	double vvmax = ddek * fabs(ddek) + 2.0 * (dek - demax);
	double ucvdemin = -U * sign(vvmin + (1.0 - fabs(sign(vvmin))) * ddek);
	double ucvdemax = -U * sign(vvmax + (1.0 - fabs(sign(vvmax))) * ddek);
	double uvdemin = fmax(uaddemin, fmin(ucvdemin, uaddemax));
	double uvdemax = fmax(uaddemin, fmin(ucvdemax, uaddemax));
	double uk = fmax(uvdemin, fmin(uc, uvdemax));
	nextState[2] = ddqLast + Ts * uk;
	nextState[1] = dqLast + 0.5 * Ts * (ddqLast + nextState[2]);
	nextState[0] = qLast + 0.5 * Ts * (dqLast + nextState[1]);
	sp->qtNext= nextState[0];
	sp->dqtNext= nextState[1];
	sp->ddqtNext= nextState[2];

	sp->param.ddqLast=sp->ddqtNext;
	sp->param.dqLast=sp->dqtNext;
	sp->param.qLast=sp->qtNext;
	sp->qE_k_1= sp->qE_k;
	sp->qE_k = fabs(sp->param.rk - sp->qtNext);
	return 0;
}

