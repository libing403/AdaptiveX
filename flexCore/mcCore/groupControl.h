#ifndef GROUP_CONTROL_H
#define GROUP_CONTROL_H

#ifdef __cplusplus
extern "C" {
#endif

//解决__func__兼容性问题
#ifndef __func__
#if defined(_MSC_VER)
#define __func__ __FUNCTION__
#endif
#endif
 
#include "axisControl.h"  // 引入单轴相关定义，包括 FsmEvent_t
#include "Queue.h"
#include "VelocityPlanning.h"
#include "onlinetrapeprofile.h"
#include <stdio.h>
#include "InterpolatorImplementation.h"

#define MAX_CTRL_COUNT        1 //最大控制器数量
#define MAX_GROUP_COUNT       2     //最大轴组数量
#define MAX_GRPcmd_COUNT      100 //最大事件数量
#define MAX_GRPTRAJECT_COUNT  100 //最大轨迹数量
#define MAX_DEVICE_COUNT      32 //最大设备数量
#define MAX_ERROR_COUNT       100
#define MAX_CARTESIAN_COUNT   6 //最大笛卡尔轴数量



//轴组轨迹指令
typedef struct {
    //轨迹通用参数
	int cmdNo;	///<命令号。
	int index;	///<指令索引。
	unsigned int syncCnt;
    int dim;    //有效坐标数
	int pathType;//0-未定义的类型，1-直线路径，2-圆弧路径，3-椭圆路径，4-B样条路径，5-关节单轴运动指令
    int state; //轨迹的状态：0-未处理，1-预处理完成，2-速度前瞻完成,3-速度规划完成，4-插补计算中，5-插补完成
    int bufferMode;//缓冲模式，0-中止模式，打断已有指令，执行当前指令；1-缓冲模式；2-低速合并；3-以前一个速度合并
    int transMode;//过渡模式，0-无过渡，1-线性过渡，2-梯形过渡，3-曲线过渡
	double startPos[MAX_AXIS_COUNT];
    double pos[MAX_AXIS_COUNT];
    double vimax[MAX_AXIS_COUNT];//各虚拟轴允许的最大速度。
    double velLimit;//速度限制
    double len;
    double vs;
    double ve;
    double vel;
    double acc;
    double dec;
    double jerk;
    double time;
    double vec[MAX_AXIS_COUNT];//方向矢量
    double startPose[MAX_CARTESIAN_COUNT];  ///< 起点位姿
    double endPose[MAX_CARTESIAN_COUNT];    ///< 终点位姿
    double dirVec[MAX_CARTESIAN_COUNT];     ///< 路径方向矢量
    double theta;//corner该点和前后两点的连线的角度。
    double posTheta; //位置方向拐角
    double posLength;//位置路径长度
    double startQua[MAX_CARTESIAN_COUNT];  ///< 起点姿态四元数
    double endQua[MAX_CARTESIAN_COUNT];    ///< 终点姿态四元数
    double rotVec[MAX_CARTESIAN_COUNT];    ///< 姿态旋转轴向量
    double wimax[MAX_AXIS_COUNT];//各姿态轴允许的最大角速度。
    double rotTheta; //姿态方向拐角
    double rotAngle; //姿态旋转角度
    double eqLen;//等效路径长度，考虑位置和姿态综合影响
    TrapeProfile_t tp;//梯形速度轮廓
    

}CmdTrajectory_t;

typedef struct{
    int cmdNo;	///<命令号。
	int index;	///<指令索引。
    int state; //事件的状态：0-未处理，1-已正向处理,2-已反向处理
	unsigned int syncCnt;
    //M代码设置地址
    int *forwardValAddr;
    int *reverseValAddr;
    int forwardValue;
    int reverseValue;

    //操作硬件
    int deviceId;   //设备ID
    int direction; //0-正向设置，1-反向设置，2-正/反向均设置
    int forwardStatus; //正向状态值
    int reverseStatus; //反向状态值


}FifoEvent_t;

/*==================== 组状态机定义 =====================*/
typedef enum GrpStatus
{
    GRP_BASESTATE,//状态基类
    GRP_DISABLE,//禁用轴组
    GRP_STANDBY,//停止待机中
    GRP_MOVING,//动作中，包括到位等待、以及因超调而使速度变为0的状态；
    GRP_STOPPING,//减速停止中，
    GRP_ERRORSTOP,//错误减速停止，
    GRP_HOMING,//轴组回原点中

    //子状态
    GRP_HALTING,//暂停减速中，仍处于运动中
    GRP_HALTED,//处于暂停静止中
    GRP_MOVE,//暂停中，接收其他运动指令中
    GRP_GOBACK,//暂停中，回退运动中
    GRP_RESUME,//暂停中，恢复运动中

}GrpStatus_enum;

//前置声明
typedef  struct AxisGroupControl_t AxisGroupControl_t;

typedef int (*GroupStateOnEnter)( AxisGroupControl_t  *group);
typedef int (*GroupStateOnExit)( AxisGroupControl_t *group);
typedef int (*GroupStateOnUpdate)( AxisGroupControl_t *group, double dt);
typedef int (*GroupStateOnEvent)( AxisGroupControl_t *group, const FsmEvent_t *event);
typedef int (*GroupStateOnRtUpdate)( AxisGroupControl_t *group, double dt);

typedef struct AxisGroupBaseState_t{
    const char *name;         // 状态名称
    GrpStatus_enum status;
    GroupStateOnEnter onEnter;
    GroupStateOnExit onExit;
    GroupStateOnUpdate onUpdate;
    GroupStateOnRtUpdate onRtUpdate;
    GroupStateOnEvent onEvent;
}AxisGroupBaseState_t;

typedef struct AxisGroupState_t {
    const AxisGroupBaseState_t *baseState; // 基类状态
    const char *name;         // 状态名称
    GrpStatus_enum status;
    GroupStateOnEnter onEnter;
    GroupStateOnExit onExit;
    GroupStateOnUpdate onUpdate;
    GroupStateOnRtUpdate onRtUpdate;
    GroupStateOnEvent onEvent;

} AxisGroupState_t;



//设备操作函数指针
typedef int (*p_setDeviceFun)(int deviceId,int status);
typedef int (*p_readDeviceFun)(int deviceId,int *status);

/*==================== 轴组控制结构体 =====================*/


typedef struct AxisGroupControl_t {
    HANDLE hMutex; // 互斥锁,锁轴组的所有数据
    struct GrpUsrParam{//插补参数
        double vs;
        double ve;
        double accnMax;     //法向加速度
        double accMax;
        double velMax;
        double jerkMax;
        double smoothLevel;//前瞻平滑等级
        int lookAheadEnabled; //前瞻是否启用
        double wMax; //最大角速度
        double aMax; //最大角加速度
        double jMax; //最大角加加速度
    }usrParam;

    struct GrpCfg{
        int grpId;
        //0-未创建轴组，没有添加任何轴到轴组
        //1-未使用轴组，已添加轴到轴组
        //2-使用轴组，轴组在运动。
        int grpEnable;
        int execID;//执行ID，0-未分配任务（未创建组）;1-分配至原始恒定周期任务;2-分配至固定周期任务

    }cfg;
    struct Kinematics{
        int grpType; //多轴协调控制的轴构成，0-坐标直接等效于物理轴，1-自定义轴变换;
        //int axisId[MAX_AXIS_COUNT]; // 组内轴ID列表（取自单轴模块）
        AxisControl_t *axis[MAX_AXIS_COUNT];
        int axisCount;//物理轴数量
        int coordDim;   //坐标维数
        //acs到mcs的坐标变换
        //double acs2mcs[4][4];
        //acs到mcs的动态坐标变换(自定义五轴，机械臂等)
        int (*pInverseKin)(int grpID,double *mcsPos,double *refPos,double *acsPos );
        int (*pForwardKin)(int grpID,double *acsPos,double *mcsPos );                 // 组内轴数量
        double pathLength;
    }kine;

    //实时状态参数
    const AxisGroupState_t *preState;       // 上个组状态
    const AxisGroupState_t *state;       // 当前组状态

    enum GrpDetails{
        GRP_Idle,//停止中，到位等待以及未进行指令运算时
        GRP_InPosWaiting,//到位等待，任意一个构成轴到位等待时
    }details;
    struct GrpCmd{
        double dis;//指令插补位置
        double vel;//指令插补速度
        double acc;//指令插补加速度
        double jerk;
    }cmd;
    //轴组轻度故障
    // struct GrpMFaultLv1{
    //     bool active;
    //     int code;
    // }mFaultLv1;
    // //轴组监视信息
    // struct GrpObsr{
    //     bool active;
    //     int code;
    // }obsr;
    //插补通道状态

    p_setDeviceFun setDeviceStatus[MAX_DEVICE_COUNT];
    p_readDeviceFun readDeviceStatus[MAX_DEVICE_COUNT];
     
    //拓展成员
    int waitCount; //等待状态更新次数， 
    double *refTime;
    SpeedRate_t speedRate;  //速度倍率规划器
    SpeedRate_t stopRate;   //停止倍率规划器

    Queue trajectoryQ;
    CmdTrajectory_t trajectoryBuffer[MAX_GRPTRAJECT_COUNT+1]; 
    Queue eventQ;
    FifoEvent_t eventBuffer[MAX_GRPcmd_COUNT+1]; 
    
    unsigned int enSyncCnt;//指令入队计数器
    unsigned int delSyncCnt;//指令出队计数器

    int trajCmdIdx;//轨迹类指令编号
    int evtCmdIdx;//非轨迹类指令编号

    int inpIndex;   //当前正在插补的指令索引(前进或回退时指向队列元素索引)
    CmdTrajectory_t curInpTraj;//当前插补指令
    CmdTrajectory_t preEnqTraj;//最近一次入队轨迹
    int evtIndex;    //上次正在执行的事件索引
    int smoothIndex; //平滑处理的指令索引(插补的到最前面的指令)
    int planIndex;   //当前速度规划的指令索引
    double endTime; //轨迹终点时间

    Inpterpolator_t inp;
    InterpolatorBase_t inpBase;
    UnifiedInterpolator_t unifiedInp;
    CmdTrajectory_t *curTraj_ptr;
    double cyclePeriod; //轴组控制周期
   

    int dbFlag;//调试标志
    FILE *cmdFile;//指令文件
    FILE *grpFile;//轴组插补文件
    FILE *cartFile;//笛卡尔轨迹文件
    FILE *drvFile;//驱动文件
    FILE *actFile;//实际文件
} AxisGroupControl_t;





typedef struct adx_Control_t{

    double cyclePeriod; //控制周期
    double syncInpTime; //同步时间
    double syncDrvTime; //驱动器同步时间
    AxisGroupControl_t grp[MAX_GROUP_COUNT]; // 轴组控制结构体
    AxisControl_t axis[MAX_AXIS_COUNT]; // 轴控制结构体
    int axisCount; // 轴数量
    int grpCount; // 组数量

} adx_Control_t;

/*==================== 前置声明：组状态机状态 =====================*/
extern const AxisGroupBaseState_t groupBaseState;
extern const AxisGroupState_t GroupDisableState;
extern const AxisGroupState_t GroupStandbyState;
extern const AxisGroupState_t GroupMovingState;
extern const AxisGroupState_t GroupHomingState;
extern const AxisGroupState_t GroupStoppingState;
extern const AxisGroupState_t GroupErrorStopState;
/*==================== 公共 API =====================*/

/**
 * @brief 轴组初始化，建议在系统初始化时调用
 * 
 * @return int 
 */
int  GroupControlInit(void);

/**
 * @brief 轴组状态机周期性更新函数，建议在实时任务中调用
 * @param group 指向轴组控制结构体的指针
 */
int GroupControl_Update(void);
int GroupControl_RtUpdate(void);
bool isGroupStandby(AxisGroupControl_t *group);

int groupTransitionState(AxisGroupControl_t *group, const AxisGroupState_t *newState);
int groupTransitionStateNoSync(AxisGroupControl_t *group, const AxisGroupState_t *newState) ;
/**
 * @brief 轴组状态机事件处理接口
 * @param group 指向轴组控制结构体的指针
 * @param event 指向组事件结构体的指针
 */
int GroupControl_HandleEvent(AxisGroupControl_t *group, const FsmEvent_t *event);

int inverseTransform(int grpId,double *cartPos,double *refPos, double *axisPos);
int forwardTransform(int grpId,double *axisPos,double *cartPos);
int inverseTransformQuat(int grpId,double pose[7],double refPose[7],double jointPos[6]);
int forwardTransformQuat(int grpId,double jointPos[6],double pose[7]);

int trajectoryPretreatment(AxisGroupControl_t* grp);
int lookAhead(AxisGroupControl_t *grp);

int executeEvent(AxisGroupControl_t *grp,FifoEvent_t *pEvt);

int getExecutedFifoEvent(AxisGroupControl_t *grp,FifoEvent_t **pEvt);

#ifdef __cplusplus
}
#endif

#endif /* GROUP_CONTROL_H */
