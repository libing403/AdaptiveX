/*****************************************************************//**
 * \file   VelocityPlanning.c
 * \brief  速度规划.
 * 
 * \author LiBing
 * \date   July 2020
 *********************************************************************/
#include <string.h>
#include <math.h>
#include "VelocityPlanning.h"
#include "MathLib.h"
/**
 * \brief 梯形加减速规划.
 * 
 * \param tp	梯形速度曲线结构体。
 * \param L		位移。
 * \param vs	初始速度。
 * \param vc	期望速度。
 * \param ve	末速度。
 * \param acc	加速度。
 * \param dec   减速度。
 * \return      返回零执行成功，非零值执行失败（目前暂时无错误代码）。
 */
int trapezoidal_profile(TrapeProfile_t* tp,float_def L, float_def Vs, float_def Vc, \
	float_def Ve, float_def Acc, float_def Dec, float_def dt)
{
	memset(tp, 0, sizeof(TrapeProfile_t));
	float_def vm=0.0,vs = Vs, vc = Vc, ve = Ve, acc = Acc, dec = -Dec;
	tp->acc = Acc;
	tp->dec = -Dec;
	tp->L = L;
	tp->dt = dt;
	if (vs < ve && L < ((double)ve * ve - (double)vs * vs) / (2.0 * acc))
	{
		//只有加速段，且需降低末速度
		ve = sqrt(fabs((double)vs * vs + 2.0 * acc * L));
		vm = ve;
		tp->t1 = (vm - vs) / acc;
		tp->t2 = 0.0;
		tp->t3 = 0.0;
		tp->t = tp->t1;
		tp->L1 = vs * tp->t1 + 0.5 * acc * tp->t1 * tp->t1;
		tp->L2 = 0.0;
		tp->L3 = 0.0;
		tp->vs = vs;
		tp->ve = ve;
		tp->vm = vm;
	}
	else if (vs > ve && L < ((double)ve * ve - (double)vs * vs) / (2.0 * dec))
	{
		//只有减速段，且需降低起步速度
		vs = sqrt(fabs((double)ve * ve - 2.0 * dec * L));
		vm = vs;
		tp->t1 = 0.0;
		tp->t2 = 0.0;
		tp->t3 = (ve - vm) / dec;
		tp->t = tp->t3;
		tp->L1 = 0.0;
		tp->L2 = 0.0;
		tp->L3 = vm * tp->t3 + 0.5 * dec * tp->t3 * tp->t3;
		tp->vs = vs;
		tp->ve = ve;
		tp->vm = vm;
	}
	else
	{
		//一般情况
		vm = sqrt(fabs((-2.0 * acc * dec * L - (double)vs * vs * dec + (double)ve * ve * acc)) / (acc - dec));
		if (vm < vc)
		{
			//无匀速段
			tp->vm = vm;
			tp->t1 = (vm - vs) / acc;
			tp->t3 = (ve - vm) / dec;
			tp->t2 = 0.0;
			tp->t = tp->t1 + tp->t2 + tp->t3;
			tp->L1 = ((double)vm * vm - (double)vs * vs) / (2.0 * acc);
			tp->L2 = 0.0;
			tp->L3 = ((double)ve * ve - (double)vm * vm) / (2.0 * dec);
			tp->vs = vs;
			tp->ve = ve;
			tp->vm = vm;
		}
		else
		{
			//有匀速段
			vm = vc;
			tp->vm = vc;
			tp->t1 = (vm - vs) / acc;
			tp->t3 = (ve - vm) / dec;
			tp->L1 = ((double)vm * vm - vs * vs) / (2.0 * acc);
			tp->L3 = ((double)ve * ve - vm * vm) / (2.0 * dec);
			tp->L2 = L - tp->L1 - tp->L3;
			tp->t2 = tp->L2 / vm;
			tp->t = tp->t1 + tp->t2 + tp->t3;
			tp->vs = vs;
			tp->ve = ve;
			tp->vm = vm;
		}

	}
	return 0;
}

float_def  trapezoidal_acc(TrapeProfile_t* tp, float_def t)
{
	float_def acc;
	if(t<1.0e-5)
		acc=0.0;
	else if(t<tp->t1)
		acc=tp->acc;
	else if(t<tp->t2)
		acc=0.0;
	else if(t<tp->t3)
		acc=tp->dec;
	return acc;
}


/**
 * \brief 计算梯形加减速任意时刻的速度.
 * 
 * \param tp	梯形速度曲线结构体。
 * \param t		相对于该段起点的时刻。
 * \return		对应时刻的位移。	
 */
float_def  trapezoidal_vel(TrapeProfile_t* tp, float_def t)
{
	float_def vt = 0.0;
	if (tp->L < 1.0E-5)
	{
		vt = 0.0;
	}
	else if (t < tp->t1)
	{
		vt = tp->vs + tp->acc * t;
	}
	else if (t < tp->t1 + tp->t2)
	{
		vt = tp->vm;
	}
	else
	{
		vt = tp->vm + tp->dec * (t - tp->t1 - tp->t2);
	}
	return vt;
}

/**
 * \brief 计算梯形加减速任意时刻的位移.
 * 
 * \param tp	梯形速度曲线结构体。
 * \param t		相对于该段起点的时刻。
 * \return		对应时刻的位移。
 */
float_def  trapezoidal_dis(TrapeProfile_t* tp, float_def t)
{
	float_def tmp = 0.0, Lt = 0.0;
	if (tp->L < 1.0E-5)
	{
		Lt = 0.0;
	}
	else if (t < tp->t1)
	{
		Lt = tp->vs * t + 0.5 * tp->acc * t * t;
	}
	else if (t < tp->t1 + tp->t2)
	{
		Lt = tp->L1 + tp->vm * (t - tp->t1);
	}
	else
	{
		tmp = t - tp->t1 - tp->t2;
		Lt = tp->L1 + tp->L2 + tp->vm * tmp + 0.5 * tp->dec * tmp * tmp;
	}
	return Lt;
}

/**
 * \brief 梯形速度曲线同步.
 * 
 * \param vs 起步速度
 * \param ve 终止速度
 * \param Armax 最大加速度
 * \param L		路程长度
 * \param ts	指定时间
 * \param tp	同步后的梯形曲线输出。
 * \return 
 */
int trapezoidal_profile_timesync(float_def vs, float_def ve, float_def amax, float_def dmax,float_def L, float_def ts, TrapeProfile_t* tp)
{

	float_def T1 = 0.0, T2 = 0.0, a, b, c,vm, t1, t2, t3, L1, L2, L3, acc, dec,tmp1;
	//float_def v1 = fmin(vs, ve);
	//float_def v2 = fmax(vs, ve);
	//float_def x = 0;

	if(vs<ve)
		if (ve < 1.0e-3)
		{
			T1 = 1.0e10;
			T2 = 1.0e10;
		}
		else if (vs < 1.0e-3)
		{
			T1 = (ve - vs) / amax + (L - (ve * ve - vs * vs) / (2 * amax)) / ve;
            T2 = 1.0e10;
		}
		else
		{
			T1 = (ve - vs) / amax + (L - (ve * ve - vs * vs) / (2 * amax)) / ve;
			T2 = (ve - vs) / amax + (L - (ve * ve - vs * vs) / (2 * amax)) / vs;
		}

	else//vs>=ve
	{
		if (vs < 1.0e-3)
		{
			T1 = 1.0e10;
			T2 = 1.0e10;
		}
		else if (vs < 1.0e-3)
		{
			T1 = (vs - ve) / dmax + (L - (vs * vs - ve * ve) / (2 * dmax)) / vs;
			T2 = 1.0e10;
		}
		else
		{
			T1 = (vs - ve) / dmax + (L - (vs * vs - ve * ve) / (2 * dmax)) / vs;
			T2 = (vs - ve) / dmax + (L - (vs * vs - ve * ve) / (2 * dmax)) / ve;
		}
	}

	if (ts < T1) //% 存在加速、匀速、减速段
	{
		acc = amax;
		dec = -dmax;
		a = acc-dec;
		b = 2 * acc * dec * ts + 2 * dec * vs - 2 * acc * ve;
		c = acc * ve * ve - dec * vs * vs - 2 * acc * dec * L;
		if (fabs(a) > 1.0e-3)
			vm = (-b - sqrt(fabs(b * b - 4 * a * c))) / (2 * a);
		else if (fabs(b) > 1.0e-3)
			vm = -c / b;
		else
			vm = 0.0;
	}

	else if( ts < T2) //
	{
		if (vs < ve) //% 加速，匀速，加速
		{ 

			dec = dmax;
			acc = amax;
			a = acc - dec;
			b = 2 * acc * dec * ts + 2 * dec * vs - 2 * acc * ve;
			c = acc * ve * ve - dec * vs * vs - 2 * acc * dec * L;
			if (fabs(a) > 1.0e-3)
				vm = (-b - sqrt(fabs(b * b - 4 * a * c))) / (2 * a);
			else if (fabs(b) > 1.0e-3)
				vm = -c / b;
			else
				vm = 0.0;
		}
		else//% 减速，匀速，减速
		{
			acc = -amax;
			dec = -dmax;
			a = acc - dec;
			b = 2 * acc * dec * ts + 2 * dec * vs - 2 * acc * ve;
			c = acc * ve * ve - dec * vs * vs - 2 * acc * dec * L;
			if (fabs(a) > 1.0e-3)
				vm = (-b - sqrt(fabs(b * b - 4 * a * c))) / (2 * a);
			else if (fabs(b) > 1.0e-3)
				vm = -c / b;
			else
				vm = 0.0;

		}

	}
	else//%ts>T2 减速，匀速，加速
	{

		acc = -amax;
		dec = dmax;
		a = acc - dec;
		b = 2 * acc * dec * ts + 2 * dec * vs - 2 * acc * ve;
		c = acc * ve * ve - dec * vs * vs - 2 * acc * dec * L;
		tmp1 = b * b - 4 * a * c;
		if (ts > vs / amax + ve / dmax)
		{
			if (L > vs * vs / (2 * amax) + ve * ve / (2 * dmax))
			{
				//存在大于0的解
				if (fabs(a) > 1.0e-3)
					vm = (-b - sqrt(fabs(tmp1))) / (2 * a);
				else if (fabs(b) > 1.0e-3)
					vm = -c / b;
				else
					vm = 0.0;
			}
			else if (L > vs * vs / (2 * amax))
			{
				//没有大于零的解，降低末速度
				ve = sqrt(fabs(2 * amax * dmax * L - dmax * vs * vs) / amax);
				b = 2 * acc * dec * ts + 2 * dec * vs - 2 * acc * ve;
				c = acc * ve * ve - dec * vs * vs - 2 * acc * dec * L;
				if (fabs(a) > 1.0e-3)
					vm = (-b - sqrt(fabs(b * b - 4 * a * c))) / (2 * a);
				else if (fabs(b) > 1.0e-3)
					vm = -c / b;
				else
					vm = 0.0;
			}
			else
			{
				//无解，降低起步速度
				vs = sqrt(2 * amax * L);
				vm = 0;
				ve = 0;
			}
		}
		else
		{
			//匀速处理
			vs = L / ts;
			ve = vs;
			vm = vs;
		}
	}
	t1 = (vm - vs) / acc;
	t3 = (ve - vm) / dec;
	t2 = ts - t1 - t3;
	L1 = vs * t1 + 0.5 * acc * t1 * t1;
	L2 = vm * t2;
	L3 = vm * t3 + 0.5 * dec * t3 * t3;
	tp->acc = acc;
	tp->dec = dec;
	tp->L = L;
	tp->L1 = L1;
	tp->L2 = L2;
	tp->L3 = L3;
	tp->t = t1 + t2 + t3;
	tp->t1 = t1;
	tp->t2 = t2;
	tp->t3 = t3;
	tp->vs = vs;
	tp->vm = vm;
	tp->ve = ve;
	return 0;
}




/**
 * \brief 设置三次多项式的边界条件.
 *
 * \param cc	三次多项式曲线结构体
 * \param t0	时间起点
 * \param t1	时间终点
 * \param q0	起点位置
 * \param q1	终点位置
 * \param v0	起点速度
 * \param v1	终点速度
 * \return
 */
int setCubicCurveParam(CubicCurve_t* cc, float_def t0, float_def t1, float_def q0, float_def q1, float_def v0, float_def v1)
{
	//a0 = q0;
	//a1 = v0;
	//a2 = (3 * h - (2 * v0 + v1) * T) / (T * T);
	//a3 = (-2 * h + (v0 + v1) * T) / (T * T * T);
	//ft = a0 + a1 * (tr - t0) + a2 * (tr - t0). ^ 2 + a3 * (tr - t0). ^ 3;
	float_def T = t1 - t0;
	float_def h = q1 - q0;
	cc->t0 = t0;
	cc->t1 = t1;
	cc->q0 = q0;
	cc->q1 = q1;
	cc->v0 = v0;
	cc->v1 = v1;

	cc->a0 = q0;
	cc->a1 = v0;
	cc->a2 = (3 * h - (2 * v0 + v1) * T) / (T * T);
	cc->a3 = (-2 * h + (v0 + v1) * T) / (T * T * T);
	return 0;
}

/**
 * \brief 计算三次多项式曲线的位置.
 *
 * \param cc	三次多项式曲线的结构体。
 * \param x		三次多项式曲线的时间值。
 * \param y		三次多项式曲线的位置值。
 * \return
 */
int calcCubicCurvePos(CubicCurve_t* cc, float_def x, float_def* y)
{
	//y=a0+a1*(tl-t0)+a2*(tl-t0).^2+a3*(tl-t0).^3;
	float_def tmp = 0.0;
	if (x < cc->t0)x = cc->t0;
	if (x > cc->t1)x = cc->t1;
	tmp = cc->a0 + cc->a1 * (x - cc->t0) + cc->a2 * (x - cc->t0) * (x - cc->t0) + cc->a3 * (x - cc->t0) * (x - cc->t0) * (x - cc->t0);
	*y = tmp;
	return 0;
}

/**
 * \brief 计算三次多项式曲线的一阶导数.
 *
 * \param cc	三次多项式曲线的结构体。
 * \param x		三次多项式曲线的时间值。
 * \param dy	三次多项式曲线的位置值。
 * \return
 */
int calcCubicCurveDerivative(CubicCurve_t* cc, float_def x, float_def* dy)
{
	//dx/dt=a1+2*a2*(tl-t0)+3*a3*(tl-t0).^2;
	float_def tmp = 0.0;
	tmp = cc->a1 + 2.0 * cc->a2 * (x - cc->t0) + 3 * cc->a3 * (x - cc->t0) * (x - cc->t0);
	*dy = tmp;
	return 0;
}


