#ifndef __VELOCITYPLANNING_H__
#define __VELOCITYPLANNING_H__
#ifdef __cplusplus
extern "C" {
#endif

#include "MathLib.h"


	/**
	 * \brief 梯形速度曲线结构体.
	 */
	typedef struct {
		float_def dt;
		float_def L;
		float_def t1;
		float_def t2;
		float_def t3;
		float_def t;
		float_def L1;
		float_def L2;
		float_def L3;
		float_def vs;
		float_def vm;
		float_def ve;
		float_def acc;
		float_def dec;
	}TrapeProfile_t;



	//三次多项式曲线，f(t)=a0+a1*(t-t0)+a1*(t-t0)*(t-t0)+a2*(t-t0)*(t-t0)
	typedef struct {
		float_def a0;
		float_def a1;
		float_def a2;
		float_def a3;
		float_def t0;
		float_def t1;
		float_def q0;
		float_def q1;
		float_def v0;
		float_def v1;
	}CubicCurve_t;
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
	 * \return		返回零执行成功，非零值执行失败（目前暂时无错误代码）。
	 */
	int trapezoidal_profile(TrapeProfile_t* tp, float_def L, float_def vs, float_def vc,\
		 float_def ve, float_def acc, float_def dec, float_def dt);


	float_def  trapezoidal_acc(TrapeProfile_t* tp, float_def t);

	/**
	 * \brief 计算梯形加减速任意时刻的速度.
	 *
	 * \param tp	梯形速度曲线结构体。
	 * \param t		相对于该段起点的时刻。
	 * \return		对应时刻的位移。
	 */
	float_def  trapezoidal_vel(TrapeProfile_t* tp, float_def t);

	/**
	 * \brief 计算梯形加减速任意时刻的位移.
	 *
	 * \param tp	梯形速度曲线结构体。
	 * \param t		相对于该段起点的时刻。
	 * \return		对应时刻的位移。
	 */
	float_def  trapezoidal_dis(TrapeProfile_t* tp, float_def t);

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
	int trapezoidal_profile_timesync(float_def vs, float_def ve, float_def acc,float_def dec, float_def L, float_def ts, TrapeProfile_t* tp);


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
	int setCubicCurveParam(CubicCurve_t* cc, float_def t0, float_def t1, float_def q0, float_def q1, float_def v0, float_def v1);

	/**
	 * \brief 计算三次多项式曲线的位置.
	 *
	 * \param cc	三次多项式曲线的结构体。
	 * \param x		三次多项式曲线的时间值。
	 * \param y		三次多项式曲线的位置值。
	 * \return
	 */
	int calcCubicCurvePos(CubicCurve_t* cc, float_def x, float_def* y);

	/**
	 * \brief 计算三次多项式曲线的一阶导数.
	 *
	 * \param cc	三次多项式曲线的结构体。
	 * \param x		三次多项式曲线的时间值。
	 * \param dy	三次多项式曲线的位置值。
	 * \return
	 */
	int calcCubicCurveDerivative(CubicCurve_t* cc, float_def x, float_def* dy);

#ifdef __cplusplus
}
#endif

#endif
