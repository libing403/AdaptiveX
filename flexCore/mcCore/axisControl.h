#ifndef AXISCONTROL_H
#define AXISCONTROL_H

#ifdef __cplusplus
extern "C" {
#endif

//解决__func__兼容性问题
#ifndef __func__
#if defined(_MSC_VER)
#define __func__ __FUNCTION__
#endif
#endif

#include <stdbool.h>
#include <stdio.h>
#include "VelocityPlanning.h"
 #include "onlinetrapeprofile.h"
#include "Queue.h"
#define STATE_WAIT_COUNT 3    //等待状态机更新次数


#define CONTROL_PERIOD_SEC 0.001  // 控制更新周期：1ms
#define MAX_OTG_POS_ERR 0.0001 // 在线目标跟踪到位容差
#define MAX_TIME_ERR    0.00001 //时间到位误差
#define MAX_DISTANCE_ERR  0.0005 //轴组到位判断容差

// 系统支持的最大轴数
#define MAX_CENTRAL_ACC 10000.0
#define MAX_AXISTRAJECT_COUNT 100
#define MAX_AXIS_COUNT 4
#define MAX_TIME_QUEUE 30
#define MAX_MSG_SIZE 256
#define MAX_CMDQUEUE_COUNT 100
#define MAX_TIMEQUEUE_COUNT 30
#define MAX_GANTRY_COUNT 2
#include "cmdNo.h"

// 事件/命令结构体，使用 void* 支持泛型参数传递
typedef struct FsmEvent{
    EventType_enum type;  // 事件类型
    const char *name;     // 事件名称
    void *data;           // 泛型数据指针（例如：目标位置、速度等）
    char msg[MAX_MSG_SIZE]; // 事件信息内容
} FsmEvent_t;

typedef enum AxisType_enum {

    AXIS_TYPE_LINEAR = 0,  // 普通线性轴
    AXIS_TYPE_ROTARY = 1,   // 普通旋转轴
    AXIS_TYPE_GANTRY_MASTER = 2, // 龙门主轴
    AXIS_TYPE_GANTRY_SLAVE = 3, // 龙门从轴

} AxisType_enum;


//单轴轨迹指令
typedef struct {
    int axisId;
    int index;
    double position;
    double velocity;
    double acceleration;
    double deceleration;
    double jerk;
    int direction;
    int bufferMode;

}AxisTrajectory_t;




/**
 * \brief 插补缓存数据.
 */
typedef struct {
    int index;            ///<正在驱动的目标点索引
    int cmdNo;
    double time;
    double syncTime;
    double lastPos;
    double curPos;
}InpTimeData_t;



/**
 * \brief 直线插补器.
 */
typedef struct {
    int index;
    int cmdNo;
    int dimension;
    int pathType;
    double startPos[MAX_AXIS_COUNT];				 ///< 本段起始位置.
    double endPos[MAX_AXIS_COUNT];
    double ti;			//局部时间坐标，相对于本段起点。
    double syncTime;	//该段绝对同步时间戳，相对于运动起点。
    double time;        //本周期插补时间步长，由倍率控制。                      
    double initTime;    //偏置时间，上一周期插补不足一个周期时，由该段补偿的时间
    double prePos[MAX_AXIS_COUNT];///< 本段目标位置.
    double curPos[MAX_AXIS_COUNT];
    double linearVelocity;
    double linearAcceleration;
    double linearJerk;
    // double velocity[MAX_AXIS_COUNT];
    // double acceleration[MAX_AXIS_COUNT];
    // double jerk[MAX_AXIS_COUNT];
    double length;		   ///<本段位移长度.
    double vec[MAX_AXIS_COUNT];	///< 本段插补矢量方向.
    double preLi;
    double li;      ///<本段已插补长度.	                     		
    int state;		///0-表示插补空闲，等待插补，1-表示以初始化，正在插补，2-正在进行最后一步插补 ,3-正在初始化，未完成    
                    // 0-表示插补空闲，等待插补，1-表示以初始化，正在插补，2-正在进行最后一步插补，3-已进行运动学转换，
    double dt;
    TrapeProfile_t tp;	 //速度曲线.
}Inpterpolator_t;

typedef struct SpeedRate_t{
    double e_integral ;
    double targetRate;
    double currentRate;
    double deltaRate;
    double rateVelocity;
 
    double minRate;
    double maxRate;
    double maxVel;
    double maxAcc;
    double maxJerk;
    double Ts;
}SpeedRate_t;

 

// 前向声明
typedef struct AxisControl_t AxisControl_t;
/*==================== 状态机定义 ====================*/

// 定义状态动作函数类型
typedef int (*StateOnEnter)(AxisControl_t *axis);
typedef int (*StateOnExit)(AxisControl_t *axis);
typedef int (*StateOnUpdate)(AxisControl_t *axis, double dt);
typedef int (*StateOnEvent)(AxisControl_t *axis, const FsmEvent_t *event);
typedef int (*StateOnRtUpdate)(AxisControl_t *axis, double dt);


typedef enum AxisStatus_enum {
    AXIS_NONE = 0,          // 未定义状态
    AXIS_DISABLED,           // 轴禁用状态
    AXIS_STANSTILL,         // 轴静止状态
    AXIS_DISCRETEMOTION,    //断续运动,点位运动
    AXIS_CONTINUOUSMOTION,  // 连续运动
    AXIS_SYNCMOTION,        // 同步运动
    AXIS_HOMING,            // 原点回归状态
    AXIS_STOPPING,          // 轴停止状态
    AXIS_ERRORSTOP,         // 错误停止状态
} AxisStatus_enum;
 
typedef struct AxisBaseState_t{
    const char *name;           // 状态名称
    AxisStatus_enum status;     // 状态标志
    StateOnEnter onEnter;        // 进入状态动作
    StateOnExit onExit;         // 退出状态动作
    StateOnUpdate onUpdate;     // 周期性更新动作
    StateOnEvent onEvent;       // 事件处理入口：处理来自上层的命令事件
    StateOnRtUpdate onRtUpdate; //中断实时更新动作
}AxisBaseState_t;

// 状态描述结构体
typedef struct {
    const AxisBaseState_t *baseState; // 基类状态
    const char *name;         // 状态名称，用于调试输出
    AxisStatus_enum status;   // 状态标志
    StateOnEnter onEnter;     // 进入状态动作
    StateOnExit onExit;       // 退出状态动作
    StateOnUpdate onUpdate;   // 周期性更新动作
    StateOnEvent onEvent;     // 事件处理入口：处理来自上层的命令事件
    StateOnRtUpdate onRtUpdate;//中断实时更新动作
} AxisState_t;





typedef struct AxisInterpolation_t{
    int type;//速度类型，0-梯形加减速，1-S形加减速；
    int cmdNo;
    int index;
    double dt;
    double syncInpTime;//指向控制器插补运动的时钟
    double syncDrvTime;//指向驱动器插补运动的时钟
    double time;       //插补数据时间戳
    double prePos;
    double preVelocity;
    double preAcceleration;
    double preJerk;
    double curPos;
    double curVelocity;
    double curAcceleration;
    double curJerk;
    double targetPos;
    double targetVel;
    double targetAcc;
    double targetJerk;
    int direction; //方向，0-正方向，1-最短路径，2-负方向，3-当前方向
    int state;//初始化标志，0--未初始化 1--已到达目标位置，处于静止状态，2-- 未到达目标处于跟踪状态。
    OnlineTrapePlanner_t otp;
    OnlineScurvePlanner_t osp;
    OnlineTrapePlanner_t velCtl;

}AxisInterpolation_t;

/*==================== 龙门双轴同步定义 =====================*/
//龙门双轴同步配置结构体
typedef struct {
    int masterId;        //主轴ID
    int slaveId;        //从轴ID
    double masterOffset_mm;     //主动轴原点感应器到0坐标的偏移
    double slaveOffset_mm;    //跟随轴原点感应器到0坐标的偏移
    double offsetspeed_mm_s;  //回原点时,从原点感应器到0坐标的运行速度
    
} GantrySyncConfig_t;

//龙门双轴同步状态结构体
typedef struct {
    int gantryId;              //龙门同步ID
    GantrySyncConfig_t config;  //配置参数
    double positionError;        //位置误差
    double maxPositionError;     //最大允许位置误差
    double syncFactor;           //同步因子
    bool isHoming;              //是否正在回原点
    bool isSynced;              //是否已同步
    int errorCount;             //错误计数
    int isCoupled;           //0:解除主动轴和跟随轴的耦合,1:主动轴和跟随轴进入耦合模式
} GantryControl_t;


typedef struct {
    double homeVel;       //原点速度
    double homeAcc;       //原点加速度
    double homeDec;       //原点减速度
    double homeOffset;    //原点偏移位置
    int homeDir;         //原点方向，1-正方向，-1-负方向
    int homeMode;       //原点模式，0-快速定位，1-慢速定位，2-接近原点，3-离开原点
}HomeParam_t;

    //规划指令位置
   typedef struct  {
        double pos;
        double vel;
        double acc;
        double dec;
        double jerk;
        double trq;
    }AxisCmd_t;


// typedef int (*SetMotorRefPosFunc)(int axisId, void *pSetMotorRefPos);
// typedef int (*GetMotorActPosFunc)(int axisId, void *pGetMotorActPos);  


typedef struct AxisControl_t {
    HANDLE hMutex; // 互斥锁,锁轴的所有数据
    int axisId;                 // 轴ID
    int grpId;
    //轴基本设定
    struct Cfg{
         int axisNo;
         int axisEnable;
         int axisType;
         unsigned int NodeAddress;//表示EtherCAT的从站地址。
         int execID;
    }cfg;

    struct Scale{
        int num;//电机旋转一圈的脉冲数
        double den;//电机旋转一圈的移动量；
        int units;//指令位置的显示单位；
        int countMode;//计数模式，0-线性模式，1-旋转模式
        double maxPos;//当前位置上限值，上溢出位置
        double minPos;//当前位置下限值，下溢出位置
    }scale;
    HomeParam_t homeParam;
    
    //用户设定默认轴参数
    struct AxisUserParam{
        //double pos;
        double vel;
        double acc;
        double dec;
        double vs;
        double ve;
        double jerk;
        double posiLmt; //正方向位置限制
        double negaLmt; //负方向位置限制
        bool enablePosiLmt; //是否启用正方向位置限制
        bool enableNegaLmt; //是否启用负方向位置限制
    }axisParam;
    //软限位约束
	struct {
		double x0;
		double x1;
		double q0;
		double q1;
		double amax;
		double xa;
	}axisLmt;

    GantryControl_t *gantryCtrl; //龙门同步控制器

    
    CubicCurve_t lmtCtrl[2];//关节限位三次多项式平滑曲线
    //实时状态参数
    const AxisState_t *preState; // 上个状态指针
    const AxisState_t *state;   // 当前状态指针

    //电机驱动接口函数指针
    int (*setMotorEnable)(unsigned short int axisId, int enable,int timeOut_us);
    int (*getMotorEnable)(unsigned short int axisId, int *enable,int timeOut_us);
    
    int (*setMotorRefPos)(unsigned short int axisId, int refPos,int timeOut_us);
    int (*getMotorActPos)(unsigned short int axisId, int *actPos,int timeOut_us);


    enum Details{
        Idle,//停止中
        InPosWainting,//到位等待
        Homed,        //原点确定
        InHome,       //原点停止
        VelLimit      //指令速度饱和
    }details;
    enum Dir{
        Nega=-1,
        Posi=1,
    }dir;
    enum DrvStatus
    {
        ServoOn,//伺服ON
        Ready,//伺服待机
        MainPower,//主电路电源
        P_OT,//正方向极限输入
        N_OT,//负方向极限输入
        HomeSw,//近原点输入
        Home,//原点输入
        ImdStop,//即停输入
        Latch1,//外部锁定输入1
        Latch2,//外部锁定输入2
        DrvAlarm,//驱动器错误输入
        DrvWarning,//驱动器警告输入
        ILA,//驱动器内部功能限制中
        CSP,//周期同步位置（CSP）模式；
        CSV,//周期同步速度（CSV）模式中
        CST,//周期同步转矩（CST）模式中
    }drvStatus;
    AxisCmd_t cmd;//规划指令位置
     //控制器向驱动器发的指令设定，增加软限位，力控限制后的位置
    struct Drv{
        double pos;
        double vel;
        double acc;
        double dec;
        double jerk;
        double trq;
    }drv;
    //驱动器向控制器反馈的状态信息
    struct Act{
        double pos;
        double vel;
        double trq;
        unsigned long int timeStamp;
    }act;
    // //轻度故障
    // struct MFaultLv1{
    //     bool active;
    //     int code;
    // }mFaultLv1;
    // //轴监视信息
    // struct Obsr{
    //     bool active;
    //     int code;
    // }obsr;


    int waitCount; //等待计数器，等待运动状态稳定

    // 可扩展其他成员，如错误码、报警信息等
    SpeedRate_t speedRate;  //速度倍率规划器
    SpeedRate_t stopRate;   //停止倍率规划器

    unsigned int enSyncCnt;//指令入队计数器
    unsigned int delSyncCnt;//指令出队计数器
    AxisInterpolation_t inp;//轴插补器;
    double *refTime;//参考时间戳，中断时间累加;

    double axisRate;//单轴插补倍率
    InpTimeData_t grpCurTime;//叠加运动的数据
    InpTimeData_t axisCurTime;//上个周期插补器数据
 
    Queue addTimeQ;
    InpTimeData_t addTimeQBuffer[MAX_TIME_QUEUE+1];
    
    Queue grpTimeQ;
    InpTimeData_t grpTimeQBuffer[MAX_TIME_QUEUE+1]; 

    Queue axisTimeQ;
    InpTimeData_t axisTimeQBuffer[MAX_TIME_QUEUE+1]; 

    Queue trajectoryQ;
    AxisTrajectory_t axisTrajBuffer[MAX_AXISTRAJECT_COUNT+1];
    FILE *drvFile;


}AxisControl_t;




/*====================  状态声明 ====================*/
extern   const AxisBaseState_t BaseState;
extern   const AxisState_t DisableState;
extern   const AxisState_t ContinusMotionState;
extern   const AxisState_t DisceteMotionState;
extern   const AxisState_t ErrorStopState;
extern   const AxisState_t HomingState;
extern   const AxisState_t StandstillState;
extern   const AxisState_t StoppingState;
extern   const AxisState_t SyncMotionState;


/*==================== 公共API ====================*/
extern double globalReferenceTime;



int axisTransitionState(AxisControl_t *axis, const AxisState_t *newState) ;
int axisTransitionStateAndNoSync(AxisControl_t *axis, const AxisState_t *newState) ;
/**
 * @brief 初始化单轴控制，所有轴及其状态机
 */
int axisControlInit(void);
int axisControlUpdate(void);   
int axisControlRTUpdate(void);
int axisInterpolation(AxisControl_t *axis,double dt);
/**
 * @brief 状态机事件处理入口，将事件分发到当前状态
 * @param axis 指向轴控制结构体的指针
 * @param event 事件及其参数封装
 */
int AxisControl_HandleEvent(AxisControl_t *axis, const FsmEvent_t *event);

int initSpeedRate(SpeedRate_t *sr,double targetRate,  double minRate, double maxRate,
    double maxVel,double maxAcc,double maxJerk);
int setSpeedRate(SpeedRate_t *sr,double targetRate,double maxVel,double maxAcc,double maxJerk,double Ts);
double updateSpeedRate(SpeedRate_t *sr,double curVel,double curAcc);
double getSpeedRate(SpeedRate_t *sr);
int setAxisPlanerTarget(AxisControl_t *axis,double targetPos,double velocity,
    double acceleration,double deceleration,double jerk);
int getAxisTimeQueue(AxisControl_t *axis,double dt,InpTimeData_t *atq);

bool isAxisStandstill(AxisControl_t *axis) ; 

int writeAxisParam_Handle(AxisControl_t *axis, const FsmEvent_t *event);
int writeAxisBoolParam_Handle(AxisControl_t *axis, const FsmEvent_t *event);
int readAxisParam_Handle(AxisControl_t *axis, const FsmEvent_t *event) ;
int moveAbsoluteBufferMode(AxisControl_t *axis, const FsmEvent_t *event);
int getAddTimeQueue(AxisControl_t *axis,double dt,InpTimeData_t *atq);
int setAxisVelocityPlanerTarget(AxisControl_t *axis,double targetVel,
    double acceleration,double deceleration,double jerk);
int axisVelocityInterpolation(AxisControl_t *axis,double dt);
 


int setAxisSoftPosiLmt(AxisControl_t *axis, double value);
int setAxisSoftNegaLmt(AxisControl_t *axis, double value);

/**
 * @brief 限制各关节的运动范围,
 *
 * @param axis 轴结构体
 * @param pos 轴坐标
 * @return int 返回0，执行成功，返回1，参数输入错误。
 */
int checkAxisSoftLimit(AxisControl_t *axis,double dt);

int applyAxisSoftLimit(AxisControl_t *axis,double dt);

//获取单轴驱动位置,进行限位和滤波平滑处理
int updateAxisDrivePosition(AxisControl_t *axis,double dt);


/*==================== 龙门双轴同步API =====================*/
/**
 * @brief 龙门双轴同步配置
 * 
 * @param gantConfig //龙门双轴同步配置结构体
 * @return int 
 */
int gantrySyncConfig(int gantryId,GantrySyncConfig_t *gantConfig);

int gantryMasterSlaveControl(GantryControl_t *gantryCtrl, double dt);

 int updateMotorInfo(AxisControl_t *axis);
#ifdef __cplusplus
}
#endif

#endif /* AXISCONTROL_H */
