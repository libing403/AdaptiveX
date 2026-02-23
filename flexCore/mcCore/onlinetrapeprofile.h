/*****************************************************************//**
 * \file   OnlineTrapeProfile.h
 * \brief  在线梯形轨迹规划
 * 
 * \author Libing
 * \date   November 2021
 *********************************************************************/

#ifndef ONLINETRAPEPROFILE_H_
#define ONLINETRAPEPROFILE_H_
#ifdef __cplusplus
extern "C"{
#endif

	typedef struct {
		//给定输入,可随时变化
		double rk;//参考位置
		double drk;//参考位置速度
		double qLast;//当前位置
		double dqLast;//当前速度
		double ddqLast;////当前加速度
		double velMax;//速度约束
		double accMax;//加速度约束
		double Ts;    //控制周期
	}OnlineTrapePlannerParam_t;

	typedef struct {
		OnlineTrapePlannerParam_t param;
		double tolerance; //误差容限
		double qtNext;   //下一周期的位置
		double dqtNext;  //下一周期的速度
		double ddqtNext;  //下一周期的加速度
		double delta;
		int count;
		int state;   //初始化标志，0--未初始化 1--已初始化，已到达目标位置，处于静止状态，2-- 未到达目标处于跟踪状态。
	}OnlineTrapePlanner_t;

	typedef struct {
		double sPos[6];//起点位姿
		double ePos[6];//终点位姿
		double lastPos[6];//上次插补位姿
		double nextLen;   //本次插补广义位移
		double totalLen;  //总位移
		double vec[6];    //插补方向矢量
		double vel[6];	 //插补速度
		double Ts;		 //插补速度
		int dim;         //插补的维数
		int state;	 //插补标志，0--插补器未初始化，1--插补器已初始化
	}LinearInterpolator_t;

	typedef struct {
		int mode;
		double refPos[6];//目标参考位置坐标
		double refVel;//目标参考位置速度
		double curPos[6];//当前位置
		double curVel;//当前速度
		double velMax;//速度约束
		double accMax;//加速度约束
		double Ts;    //控制周期

	}OnlineTargetTrackParam_t;


	typedef struct {
		int enable;            //跟踪使能标志
		OnlineTargetTrackParam_t ottp;//目标跟踪参数结构体
		LinearInterpolator_t li;//直线插补结构体。
		OnlineTrapePlanner_t otp; //在线梯形速度规划器。
		int state;	//状态标志，0--未初始化 1--已被初始化，已到达目标位置，处于静止状态，2--未到达目标处于跟踪状态。
	}OnlineTargetTracking_t;

	typedef struct {
		//给定输入,可随时变化
		double rk;//参考位置
		double drk;//参考位置速度
		double ddrk;
		double qLast;//当前位置
		double dqLast;//当前速度
		double ddqLast;////当前加速度
		double velMax;//速度约束
		double accMax;//加速度约束
		double jerkMax;
		double Ts;    //控制周期
	}OnlineScurvePlannerParam_t;

	typedef struct {
		OnlineScurvePlannerParam_t param;
		double tolerance; //误差容限
		double qtNext;   //下一周期的位置
		double dqtNext;  //下一周期的速度
		double ddqtNext;  //下一周期的加速度
		double delta;
		double qE_k; //位置误差
		double qE_k_1;
		int count;
		int state;   //初始化标志，0--未初始化 1--已被初始化，已到达目标位置，处于静止状态，2--未到达目标处于跟踪状态。
	}OnlineScurvePlanner_t;

/**
 * \brief 符号函数.
 *
 * \param x	需要判断符号的变量
 * \return  与符号相关的值1,0,-1.
 */
double sign(double x);

	/**
	 * \brief 参数更新函数，.
	 *
	 * \param param	在线梯形规划参数结构体
	 * \param otpPlanner 在线梯形速度规划器
	 * \return
	 */
	int onlineTrapePlannerUpdateParam(OnlineTrapePlannerParam_t* param, OnlineTrapePlanner_t* otpPlanner);

	/**
	 * \brief 更新梯形规划的目标位置.
	 *
	 * \param otpPlanner
	 * \param rk
	 * \return
	 */
	int onlineTrapePlannerSetRef(OnlineTrapePlanner_t* otpPlanner, double rk);


	/**
	 * \brief 更新梯形规划的速度约束.
	 *
	 * \param tp
	 * \param vel
	 * \return
	 */
	int onlineTrapePlannerSetVel(OnlineTrapePlanner_t* tp, double vel);

	/**
	 * \brief 更新梯形规划的加速度约束.
	 *
	 * \param tp
	 * \param acc
	 * \return
	 */
	int onlineTrapePlannerSetAcc(OnlineTrapePlanner_t* tp, double acc);

	int onlineTrapePlannerSetTarget(OnlineTrapePlanner_t* tp, double targtPos,double velocity,
		double acceleration,double deceleration);


	/**
	 * \brief 按周期更新规划器的输出.
	 *
	 * \param tp 在线梯形速度规划器
	 * \param dl 下个周期的位移
	 * \param delta 下个周期的相对位移
	 * \return	 返回0执行成功，返回非0执行失败。
	 */
	int onlineTrapePlannerUpdateOutput(OnlineTrapePlanner_t* tp, double* dl, double* delta);

	/**
	 * \brief 在线梯形规划器更新函数.
	 *
	 * \param tp 在线梯形速度规划器
	 * \param nextState 下个周期的状态
	 * \return 返回0执行成功，返回非0执行失败。
	 */
	int onlineTrapePlannerUpdate(OnlineTrapePlanner_t* tp, double nextState[3]);


	/**
	 * \brief 直线插补器初始化
	 *
	 * \param StartPos 起始位置。
	 * \param EndPos	结束位置。
	 * \param dim		插补坐标的维数，1-6维。
	 * \param li		直线插补器结构体。
	 * \return
	 */
	int linearInterpolatorInit(double* StartPos, double* EndPos, double Ts,int dim, LinearInterpolator_t* li);


	/**
	 * \brief 直线插补器更新
	 *
	 * \param dl	下个周期相对位移。
	 * \param nextPos 下个周期的坐标。
	 * \param li      插补器结构体。
	 * \return
	 */
	int linearInterpolatorUpdate(double dl, double* nextPos, LinearInterpolator_t* li);

	/**
	 * \brief 在线跟踪控制器始化。
	 *
	 * \param ottp	在线目标跟踪控制器的参数。
	 * \param ott	目标跟踪规划器结构体。
	 * \return		返回0成功，非零失败。
	 * \retval 1    已被初始化。
	 */
	int onlineTargetTrackingInit(OnlineTargetTrackParam_t* ottp,OnlineTargetTracking_t* ott);


	/**
	 * \brief 在线目标跟踪规划器参数在线更新
	 *
	 * \param targetPos 新的目标位置。
	 * \param ott		规划器结构体。
	 * \return          返回0成功，非零失败。
	 * \retval			返回1，规划器未初始化，请初始化。
	 */
	int onlineTargetTrackingUpdateParam(double* targetPos, OnlineTargetTracking_t* ott);

	/**
	 * \brief 目标跟踪控制器输出更新。
	 *
	 * \param ott	跟踪控制器结构体。
	 * \param nextPos 输出当前周期插补坐标。
	 * \param vel     输出周期周期速度
	 * \return 返回0执行成功，返回非零执行失败
	 * \retval 1 目标跟踪控制器未初始化，请先初始化。
	 */
	int onlineTargetTrackingUpdateOutput(OnlineTargetTracking_t* ott, double nextPos[6],double *vel);

	

	/**
	 * \brief 目标跟踪功能开启或关闭。
	 *
	 * \param ott 目标跟踪控制器结构体。
	 * \param enable	使能标志：1--开启，0--关闭。
	 * \return 返回0执行成功，返回非零执行失败
	 * \retval 0 执行成功
	 * \retval 1 目标跟踪控制结构体未初始化。
	 */
	int setOnlineTargetTrackingEnable(OnlineTargetTracking_t* ott, int enable);

	/**
	 * .\brief 获取在线跟踪控制器的当前规划位置。
	 *
	 * \param ott	在线目标跟踪结构体
	 * \param CurPos 当前规划位置
	 * \return
	 */
	int getOnlineTargetTrackingCurPos(OnlineTargetTracking_t* ott, double CurPos[6]);

	/**
	 * \brief 获取当前的指令位置.
	 *
	 * \param ott	ott->li。
	 * \param RefPos	返回当前指令位置（参考位置）。
	 * \return 返回0执行成功，非0执行失败。
	 */
	int getOnlineTargetTrackingRefPos(OnlineTargetTracking_t* ott, double RefPos[6]);

	/**
	 * .\brief 获取线跟踪控制器的当前规划速度
	 *
	 * \param ott	在线目标跟踪结构体。
	 * \param vel   当前规划位置。
	 * \return
	 */
	int getOnlineTargetTrackingVel(OnlineTargetTracking_t* ott, double* vel);

	/**
	 * \brief 参数更新函数.
	 *
	 * \param param	在线梯形规划参数结构体
	 * \param otpPlanner 在线梯形速度规划器
	 * \return
	 */
	int onlineScurvePlannerUpdateParam(OnlineScurvePlannerParam_t* param, OnlineScurvePlanner_t* otpPlanner);

	/**
	 * \brief 更新S形规划的目标位置.
	 *
	 * \param otpPlanner
	 * \param rk
	 * \return
	 */
	int onlineScurvePlannerSetRef(OnlineScurvePlanner_t* sp, double rk);

	/**
	 * \brief 更新s形规划的速度约束.
	 *
	 * \param tp
	 * \param vel
	 * \return
	 */
	int onlineScurvePlannerSetVel(OnlineScurvePlanner_t* sp, double vel);

	/**
	 * \brief 更新S形规划的加速度约束.
	 *
	 * \param sp
	 * \param acc
	 * \return
	 */
	int onlineScurvePlannerSetAcc(OnlineScurvePlanner_t* sp, double acc);


	int onlineScurePlannerSetTarget(OnlineScurvePlanner_t* sp,double targetPos,double velocity,
		double acceleration,double deceleration,double jerk);

	int onlineScurvePlannerUpdate(OnlineScurvePlanner_t* sp,  double nextState[3]);

#ifdef __cplusplus
}
#endif // !cplusplus
#endif  //!ONLINETRAPEPROFILE_H_
