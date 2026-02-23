/**
 * @file flexCore_if.h 用户接口头文件
 * @author li.bing(libinggalaxy@163.com)
 * @brief 
 * @version 0.1
 * @date 2025-02-24
 * 
 * Copyright (c)  2025 li.bing
 * 
 */
#ifndef FLEXCORE_H
#define FLEXCORE_H

#ifdef __cplusplus
extern "C" {
#endif

//解决__func__兼容性问题
#ifndef __func__
#if defined(_MSC_VER)
#define __func__ __FUNCTION__
#endif
#endif

#include<stdbool.h>



typedef enum Mc_Parameter_enum
{

  AXIS_PARAMETER_ID = 1,  //轴参数ID；
  mcVelocityRate,         //轴速度倍率
  mcEncoderScale,        //轴编码器缩放比；
  mcCmdPosition,          //所命令位置；
  mcCmdVelocity,          //所命令的速度
  mcCmdAcceleration,      //所命令的加速度；
  mcCmdDeceleration,      //所命令的减速度；
  mcCmdJerk,              //所命令的加加速度
  mcSWLimitPos,           //正方向软件限位开关位置；
  mcSWLimitNeg,           //负方向软件限位开关位置；
  mcEnableLimitPos,       //允许正方向软件限位开关；
  mcEnableLimitNeg,       //允许负方向软件限位开关；
  mcEnablePosLagMonitoring,//允许监视位置偏差；
  mcMaxPositionLag,       //最大位置偏差；
  mcMaxVelocitySystem,    //最大系统速度；
  mcMaxVelocityAppl,      //最大应用速度；
  mcActualVelocity,       //实际速度；
  mcMaxAccSystem,         //最大系统加速度；
  mcMaxAccAppl,           //最大应用加速度；
  mcMaxDecSystem,         //最大系统加速度；
  mcMaxDecAppl,           //最大应用加速度；
  mcMaxJerkSystem,        //最大系统加加速度；
  mcMaxJerkAppl,          //最大应用加加速度；

  GRP_PARAMETER_ID = 1000,//轴组参数ID；
  mcGrpSpeedRate,         //轴组速度倍率；
  mcSmoothLevel           //前瞻平滑等级；

}Mc_Parameter_enum;
 
typedef enum adx_BufferMode_enum {
  mcAborting = 0, // 中止模式，打断已有指令，执行当前指令
  mcBuffered,    // 缓冲模式
  mcBlendingLow,// 低速合并
  mcBlendingPrevious,// 以前一个速度合并
  mcBlendingHigh,// 高速合并
} adx_BufferMode_enum;

typedef struct
{
    unsigned int ppr; // 每转脉冲数
    int dirPol; // 方向极性(1 正转, -1 反转)
    unsigned int speed; // rpm
    unsigned int acc; // rpm/s
    float dpr; // 导程(每转行程,mm/r)
    int org; // 原点(原点感应器接入的输入点序号), -1 表示没有原点感应器,复位时仅清零坐标
    float mdis; // 负向软限位,只能为 0 或负值
    float pdis; // 正向软限位,只能为 0 或正值
    int resetDir; // 复位方向,-1 表示复位时向负坐标方向运行,1 表示复位时向正坐标方向运行
    unsigned int resetSpeed; // 复位速度,rpm
}axisParam_t;

typedef struct
{
    int leftCmdBufferCount; // 插补通道剩余缓存区数量
    int totalCmdCount; // 插补通道已接收的指令数量
    int curRunningCmdIndex; // 插补通道当前正在运行的指令的序号
    int status; // 插补通道当前运行状态， 
} GrpParam_t;


//龙门双轴同步配置结构体
typedef struct {
    int masterId;        //主轴ID
    int slaveId;        //从轴ID
    double masterOffset_mm;     //主动轴原点感应器到0坐标的偏移
    double slaveOffset_mm;    //跟随轴原点感应器到0坐标的偏移
    double offsetspeed_mm_s;  //回原点时,从原点感应器到0坐标的运行速度
    
} GantryConfig_t;

typedef struct {
    double homeVel;       //原点速度
    double homeAcc;       //原点加速度
    double homeDec;       //原点减速度
    double homeOffset;    //原点偏移位置
    int homeDir;         //原点方向，1-正方向，-1-负方向
    int homeMode;       //原点模式，0-快速定位，1-慢速定位，2-接近原点，3-离开原点
}HomeParameter_t;


/**
 * @brief 初始化运动库相关资源，主业务启动时调用
 * 
 * @return int 
 */
int adx_CoreInit(void);

/**
 * @brief 关闭运动库相关资源，主业务退出时调用
 * 
 * @return int 
 */
int adx_CoreClose(void);


/**
 * @brief 
 * 
 * @param axisId 
 * @param enable 
 * @param startMode 
 * @param stopMode 
 * @return int 
 */
int adx_Power(int axisId,int enable,int startMode,int  stopMode);



/**
 * @brief 读取指定轴的参数
 * 
 * @param axidId 轴ID
 * @param index 指令索引
 * @param paramId 参数ID
 * @param value 存储读取值的指针
 * @return int 返回状态,0-成功，其他值失败
 */
int adx_ReadParameter(int axidId,int index,int paramId,double *value);

/**
 * @brief 读取指定轴的布尔参数
 * 
 * @param axidId 轴ID
 * @param index 指令索引
 * @param paramId 参数ID
 * @param value 存储读取值的指针
 * @return int 返回状态,0-成功，其他值失败
 */
int adx_ReadBoolParameter(int axidId,int index,int paramId,bool *value);


int adx_WriteMultiParameters(int axisId,int index,axisParam_t *param,int bufferMode);


/**
 * @brief 写入指定轴的参数
 * 
 * @param axidId 轴ID
 * @param index 指令索引
 * @param paramId 参数ID
 * @param value 写入值
 * @param mode 写入模式，0-立即生效，1-缓存模式,前一个指令执行完成后开始生效；
 * @return int 返回状态,0-成功，其他值失败
 */
int adx_WriteParameter(int axisId,int index,int paramId,double value,int bufferMode);

/**
 * @brief 写入指定轴的布尔参数
 * 
 * @param axidId 轴ID
 * @param index 指令索引
 * @param paramId 参数ID
 * @param value 写入值
 * @param mode 写入模式，0-立即生效，1-缓存模式,前一个指令执行完成后开始生效；
 * @return int 返回状态,0-成功，其他值失败
 */
int adx_WriteBoolParameter(int axidId,int index,int paramId,bool value,int bufferMode);

/**
 * @brief 写入指定轴的多个参数
 * 
 * @param axisId 轴Id；
 * @param index 指令索引；
 * @param param 参数结构体指针
 * @param bufferMode 缓冲模式
 * @return int 
 */
int adx_WriteMultiParameters(int axisId,int index,axisParam_t *param,int bufferMode);

/**
 * @brief 单轴绝对定位
 * 
 * @param axisId        轴Id；
 * @param index         指令索引；
 * @param position      绝对目标位置；
 * @param velocity      进行定位的速度设定值。>0.0-使用设定值，=0.0-不允许，<0,0-使用预先参数设置的值；
 * @param acceleration  加速度；>0.0-使用指定值，=0.0-不允许，<0,0-使用预先参数设置的值；
 * @param Deceleration  减速度；>0.0-使用指定值，=0.0-不允许，<0,0-使用预先参数设置的值；
 * @param jerk          加加速度；>0.0,恒定加加速度曲线，=0.0-梯形速度曲线，<0,0-使用预先参数设置的值；
 * @param direction     轴运动方向；1-正方向，2-负方向，3-最短距离；
 * @return int 
 */
int adx_MoveAbsolute(int axisId,int index,double position,double velocity,double acceleration,double deceleration,
double jerk,int direction,int bufferMode);

/**
 * @brief 单轴绝对定位
 * 
 * @param axisId        轴Id；
 * @param index         指令索引；
 * @param position      相对运动距离(静止时，相对于当前位置; 前1个指令未运行完成时,相对于前1个指令目标位置)；
 * @param velocity      进行定位的速度设定值。>0.0-使用设定值，=0.0-不允许，<0,0-使用预先参数设置的值；
 * @param acceleration  加速度；>0.0-使用指定值，=0.0-不允许，<0,0-使用预先参数设置的值；
 * @param Deceleration  减速度；>0.0-使用指定值，=0.0-不允许，<0,0-使用预先参数设置的值；
 * @param jerk          加加速度；>0.0,恒定加加速度曲线，=0.0-梯形速度曲线，<0,0-使用预先参数设置的值；
 * @param direction     轴运动方向；1-正方向，2-负方向，3-最短距离；
 * @param bufferMode    缓冲模式，0-立即执行，1-缓存执行；
 * @return int 
 */
int adx_MoveRelative(int axisId,int index,double position,double velocity,double acceleration,double deceleration,
double jerk,int direction,int bufferMode);

/**
 * @brief 停止轴运动
 * 
 * @param axisId     轴Id；
 * @param index      指令索引；
 * @param velFactor  速度倍率；>0.0-使用设定值，=0.0-不允许，<0,0-使用预先参数设置的值；
 * @param accFactor  加速度；>0.0-使用设定值，=0.0-不允许，<0,0-使用预先参数设置的值；
 * @param jerkFactor 加加速度；>0.0-使用设定值，=0.0-不允许，<0,0-使用预先参数设置的值；
 * @param bufferMode 缓冲模式，0-立即执行，1-缓存执行；
 * @return int       
 */
int adx_SetOverride(int axisId,int index,double velFactor,double accFactor,double jerkFactor,int bufferMode);


/**
 * @brief 单轴速度模式运动。单轴以给定的速度持续运动，直到接收到停止指令。
 * 
 * @param axisId       轴Id；
 * @param index        指令索引；
 * @param velocity     进行定位的速度设定值；>0.0-使用设定值，=0.0-不允许，<0,0-使用预先参数设置的值；
 * @param acceleration 加速度；>0.0-使用设定值，=0.0-不允许，<0,0-使用预先参数设置的值；
 * @param deceleration 减速度；>0.0-使用设定值，=0.0-不允许，<0,0-使用预先参数设置的值；
 * @param jerk         加加速度；>0.0-使用设定值，=0.0-不允许，<0,0-使用预先参数设置的值；
 * @param direction    运动方向；1-正方向，2-负方向，其他，停止；
 * @param bufferMode   缓冲模式，0-立即执行，1-缓存执行；
 * @return int         
 */
int adx_MoveVelocity(int axisId,int index,double velocity,double acceleration,double deceleration,double jerk,int direction,int bufferMode);

/**
 * @brief       设置龙门架同步启用/禁用
 * 启用后，主轴运动时从轴跟随主轴运动，保持相对位置不变。
 * 禁用后，主轴运动时从轴不跟随运动。
 *
 * @param axisId   轴ID,主轴或从轴均可
 * @param enable   启用标志，1-启用，0-禁用。
 * @return         
 */
int adx_SetGantryEnable(int axisId,int enable);


/**
 * @brief 设置龙门架同步配置
 * 
 * @param masterId 主轴ID
 * @param slaveId  从轴ID
 * @param config   同步配置参数
 * @return 
 */
int adx_SetGantryConfig(int gantryId, GantryConfig_t *config);



int adx_WriteMultiParameters(int axisId, int index, axisParam_t *param, int bufferMode);



int adx_ReadMultiParameters(int axisId, int index, axisParam_t *param);


/**
 * @brief 读取指定轴的目标位置
 * 
 * @param axisId 轴ID
 * @param index 指令索引
 * @param pos 存储读取位置的指针
 * @return int 返回状态,0-成功，其他值失败
 */
int adx_ReadTargetPos(int axisId, int index, double* pos);

/**
 * @brief 读取指定轴的目标速度
 * 
 * @param axisId 轴ID
 * @param index 指令索引
 * @param vel 存储读取速度的指针
 * @return int 返回状态,0-成功，其他值失败
 */

int adx_ReadTargetVel(int axisId, int index, double* vel);

/**
 * @brief 读取指定轴的目标加速度
 * 
 * @param axisId 轴ID
 * @param index 指令索引
 * @param acc 存储读取加速度的指针
 * @return int 返回状态,0-成功，其他值失败
 */
int adx_ReadTargetAcc(int axisId, int index, double* acc);

/**
 * @brief 读取指定轴的目标加加速度
 * 
 * @param axisId 轴ID
 * @param index 指令索引
 * @param jerk 存储读取加加速度的指针
 * @return int 返回状态,0-成功，其他值失败
 */
int adx_ReadTargetJerk(int axisId, int index, double* jerk);

/**
 * @brief 读取指定轴的逻辑位置;
 * 
 * @param axisId 轴ID;
 * @param index 指令索引;
 * @param pos 存储读取位置的指针;
 * @return int 返回状态,0-成功，其他值失败;
 */
int adx_ReadLogicalPos(int axisId, int index, double* pos);


/**
 * @brief 同时获取多个轴的逻辑位置;
 * 
 * @param axisId   轴的索引,例如axisId[3]={1,3,5};
 * @param index    指令索引;
 * @param axisNum  轴id的索引个数，例如axisId[3]={1,3,5}表示同时获取3个轴的逻辑位置;
 * @param pos      存放返回数据的数组;
 * @return int     返回状态,0-成功，其他值失败;
 */
int adx_ReadMultiLogicalPos(int axisId[], int index,int axisNum, double *pos);


/**
 * @brief 读取指定轴的逻辑速度
 * 
 * @param axisId 轴ID
 * @param index 指令索引
 * @param vel 存储读取速度的指针
 * @return int 返回状态,0-成功，其他值失败
 */
int adx_ReadLogicalVel(int axisId, int index, double* vel);


/**
 * @brief 读取指定轴的逻辑加速度
 * 
 * @param axisId 轴ID
 * @param index 指令索引
 * @param acc 存储读取加速度的指针
 * @return int 返回状态,0-成功，其他值失败
 */
int adx_ReadLogicalAcc(int axisId, int index, double* acc);


/**
 * @brief 读取指定轴的逻辑加加速度
 * 
 * @param axisId 轴ID;
 * @param index 指令索引;
 * @param jerk 存储读取加加速度的指针;
 * @return int 返回状态,0-成功，其他值失败;
 */
int adx_ReadLogicalJerk(int axisId, int index, double* jerk);


/**
 * @brief 读取指定轴的真实位置(编码器位置);
 * 
 * @param axisId 轴ID;
 * @param index  指令索引;
 * @param pos    存储读取的位置;
 * @return int   返回状态,0-成功，其他值失败;
 */
int adx_ReadActualPos(int axisId, int index, double* pos);


/**
 * @brief 读取多个轴的真实位置(编码器位置);
 * 
 * @param axisId   轴ID;
 * @param index    指令索引;
 * @param axisNum  要读取轴参数的位置;
 * @param pos      轴位置存储位置;
 * @return int     返回状态,0-成功，其他值失败;
 */
int adx_ReadMultiActualPos(int axisId[], int index,int axisNum, double* pos);


/**
 * @brief 读取指定轴的实际速度
 * 
 * @param axisId 轴ID
 * @param index 指令索引
 * @param vel 存储读取速度的指针
 * @return int 返回状态,0-成功，其他值失败
 */
int adx_ReadActualVel(int axisId, int index, double* vel);


/**
 * @brief 读取指定轴的实际加速度
 * 
 * @param axisId 轴ID
 * @param index 指令索引
 * @param acc 存储读取加速度的指针
 * @return int 返回状态,0-成功，其他值失败
 */
int adx_ReadActualAcc(int axisId, int index, double* acc);

/**
 * @brief 读取指定轴的实际加加速度
 * 
 * @param axisId 轴ID
 * @param index 指令索引
 * @param jerk 存储读取加加速度的指针
 * @return int 返回状态,0-成功，其他值失败
 */
int adx_ReadActualJerk(int axisId, int index, double* jerk);
/**
 * @brief 读取指定轴的状态
 * 
 * @param axisId 轴ID
 * @param index 指令索引
 * @param status 存储读取状态的指针;轴状态有以下几种：
 *   1-DISABLED,          // 轴禁用状态
 *   2-STANSTILL,         // 轴静止状态
 *   3-DISCRETEMOTION,    //断续运动,点位运动
 *   4-CONTINUOUSMOTION,  // 连续运动
 *   5-SYNCMOTION,        // 同步运动
 *   6-HOMING,            // 原点回归状态
 *   7-STOPPING,          // 轴停止状态
 *   8-ERRORSTOP,         // 错误停止状态
 * @param bufferMode 缓冲模式，0-立即执行，1-缓存执行；
 * @return int 返回状态,0-成功，其他值失败
 */
int adx_ReadStatus(int axisId,int index,int *status);

/**
 * @brief 读取多个轴的状态;
 * 
 * @param axisId   轴索引列表; 
 * @param index    指令索引;
 * @param axisNum  轴数量;
 * @param status   状态值存储数组;
 * @return 
 */
int adx_ReadMultiStatus(int axisId[],int index,int axisNum,int *status);

/**
 * @brief 读取指定轴组的目标位置，最后一个运动指令的目标位置
 * 
 * @param axisId 轴ID
 * @param index 指令索引
 * @param pos 存储读取位置的指针,一般笛卡尔X,Y,Z,A,B,C等坐标轴位置
 * @return int 返回状态,0-成功，其他值失败
 */
int adx_GroupReadTargetPos(int grpId, int index, double* pos);

/**
 * @brief 读取指定轴组的逻辑位置，当前插补点的位置
 * 
 * @param axisId 轴ID
 * @param index 指令索引
 * @param pos 存储读取坐标的指针,一般笛卡尔X,Y,Z,A,B,C等坐标轴位置
 * @return int 返回状态,0-成功，其他值失败
 */
int adx_GroupReadLogicalPos(int grpId, int index, double* pos);

/**
 * @brief 读取指定轴组的状态
 * 
 * @param axisId 轴ID
 * @param index 指令索引
 * @param status 存储读取状态的指针
 * @return int 返回状态,0-成功，其他值失败
 */
int adx_GroupReadStatus(int grpId, int index, int* status );

/**
 * @brief 紧急停止轴运动
 * 
 * @param axisId 轴Id；
 * @param index  指令索引；
 * @param deceleration 减速度；>0.0-使用设定值，=0.0-不允许，<0,0-使用预先参数设置的值；
 * @param jerk 加加速度；>0.0,恒定加加速度曲线，=0.0-梯形速度曲线，<0,0-使用预先参数设置的值；
 * @param bufferMode 缓冲模式，0-立即执行，1-缓存执行；
 * @return int 
 */
int adx_Halt(int axisId,int index,double deceleration,double jerk,int bufferMode);
/**
 * @brief 停止轴运动
 * 
 * @param axisId 轴Id；
 * @param index  指令索引；
 * @param deceleration 减速度；>0.0-使用设定值，=0.0-不允许，<0,0-使用预先参数设置的值；
 * @param jerk 加加速度；>0.0,恒定加加速度曲线，=0.0-梯形速度曲线，<0,0-使用预先参数设置的值；
 * @param bufferMode 缓冲模式，0-立即执行，1-缓存执行；
 * @return int 
 */
int adx_Stop(int axisId,int index,double deceleration,double jerk,int bufferMode);




/**
 * @brief 电子齿轮设置；
 *  
 * @param slave_axisId 从属轴ID
 * @param master_axisId 主轴ID
 * @param ratio_numerator 传动比分子
 * @param ratio_denominator 传动比分母
 * @param lend_time 平滑切入时间(ms)，0表示立即切入
 * @return int  返回状态,0-成功，其他值失败
 */
int adx_GearIn(int slave_axisId, int master_axisId, double ratio_numerator, double ratio_denominator, double lend_time);



/**************************************************** 
 * 以下为轴组相关接口声明
 * **************************************************/

 /**
  * @brief 轴组关闭；
  * 
  * @param grpId 轴组索引，取值0,或1，或...,或N；
  * @return int 
  */
 int adx_GroupDisable(int grpId);

 /**
  * @brief 轴组使能；
  * 
  * @param grpId 轴组索引，取值0,或1，或...,或N；
  * @return int 
  */
int adx_GroupEnable(int grpId);

/**
 * @brief 添加轴到轴组；
 * 
 * @param grpId 轴组索引，取值0,或1，或...,或N；
 * @param axisId 轴索引，取值0,或1，或...,或N；
 * @return int 
 */
int adx_AadAxisToGroup(int grpId,int axisId);

/**
 * @brief 创建轴组并向轴组添加轴
 * 
 * @param grpId grpId 轴组索引，取值0,或1，或...,或N；
 * @param index 指令索引，用于标识指令的顺序，方便用户查询缓存指令的执行进度。不需要时可设置为0；
 * @param coordNum 笛卡尔轴的坐标数目；
 * @param axisNum  物理轴的数量；
 * @param axisId   物理轴的id，一维数组；
 * @return int 
 */
int adx_CreateAxisGroup(int grpId,int index,int coordNum,int axisNum,int *axisId);

/**
  * @brief 注册自定义运动学转换
  * 
  * @param grpId 轴组编号
  * @param pinverse 逆解函数指针
  * @param pforward 正解函数指针
  * @return int 
  */
int adx_SetKinematics(int grpId, int index,void *pinverse, void *pforward);

/**
 * @brief 注册电机使能函数
 * 
 * @param axisId 轴ID
 * @param index 指令索引
 * @param p_motorEnable 电机使能函数指针
 * @return int 返回状态,0-成功，其他值失败
 */
int adx_RegisterSetMotorEnableFun(int axisId,int index,void *p_motorEnable);

/**
 * @brief 注册获取电机实际位置的函数
 * 
 * @param axisId 轴ID
 * @param index 指令索引
 * @param p_getMotorPos 获取电机实际位置的函数指针
 * @return int 返回状态,0-成功，其他值失败
 */
int adx_RegisterGetMotorActPosFun(int axisId,int index,void *p_getMotorPos);

/**
 * @brief 注册设置电机编码器缩放比的函数
 *
 * @param axisId 轴ID
 * @param index 指令索引
 * @param num 缩放比分子
 * @param den 缩放比分母
 * @return int 返回状态,0-成功，其他值失败
 */
int adx_SetMotorEncoderScale(int axisId,int index, int num,int den);

/**
 * @brief 注册设置电机目标位置的函数
 * 
 * @param axisId 轴ID
 * @param index 指令索引
 * @param p_setMotorPos 设置电机目标位置的函数指针
 * @return int 返回状态,0-成功，其他值失败
 */
int adx_RegisterSetMotorRefPosFun(int axisId,int index,void *p_setMotorPos);



/**
 * @brief 以绝对位置方式移动轴组内所有轴;
 * 
 * @param grpId 轴组索引，取值0,或1，或...,或N；
 * @param pos   各轴目标位置数组，pos[0]，pos[1]，...,pos[n]，n轴数；
 * @param velocity 统一的移动速度（正值）；
 * @param acc 合成加速度（正值）；
 * @param dec 合成减速度（正值）；
 * @param jerk 加加速度（正值）；
 * @param direction 方向，1-正方向，2-负方向，3-最短距离；
 * @param coordSys 坐标系标志，0-WCS,世界坐标系，1-OCS1,对象坐标系（工件坐标系），2-OCS2，对象坐标系（工件坐标系），
 * 3-OCS3，对象坐标系（工件坐标系）；
 * @param bufferMode 缓冲模式，0-立即执行，1-缓存执行；
 * @param transitionMode <0-使用默认值，0-无动态调整，1-轨迹分段动态调整，2-不进行轨迹分段动态调整；
 * @param transitionParameter 过渡参数；
 * @return int 
 */
int adx_MoveLinearAbsolute(int grpId, int index,double *pos, double velocity,double acc,double dec,double jerk,
  int direction,int coordSys,  int bufferMode,int transitionMode,double *transitionParameter);

/**
 * @brief 读取指定轴组的参数
 * 
 * @param grpId 轴组ID
 * @param index 指令索引
 * @param paramId 参数ID
 * @param value 存储读取值的指针
 * @return int 返回状态,0-成功，其他值失败
 */
int adx_GroupReadParameter(int grpId,int index,Mc_Parameter_enum paramId,double *value);

/**
 * @brief 读取指定轴组的布尔参数
 * 
 * @param grpId 轴组ID
 * @param index 指令索引
 * @param paramId 参数ID
 * @param value 存储读取值的指针
 * @return int 返回状态,0-成功，其他值失败
 */
int adx_GroupReadBoolParameter(int grpId,int index,Mc_Parameter_enum paramId,bool *value);

/**
 * @brief 写入指定轴组的布尔参数
 * 
 * @param grpId 轴组ID
 * @param index 指令索引
 * @param paramId 参数ID
 * @param value 写入值
 * @param mode 写入模式，0-立即生效，1-缓存模式,前一个指令执行完成后开始生效；
 * @return int 返回状态,0-成功，其他值失
 */
int adx_GroupWriteBoolParameter(int grpId,int index,int mode,Mc_Parameter_enum paramId,bool value);

/**
 * @brief 写入指定轴组的参数
 * 
 * @param grpId 轴组ID
 * @param index 指令索引
 * @param paramId 参数ID
 * @param value 写入值
 * @param bufferMode 写入模式，0-立即生效，1-缓存模式,前一个指令执行完成后开始生效；
 * @return int 返回状态,0-成功，其他值失败
 */
int adx_GroupWriteParameter(int grpId,int index,Mc_Parameter_enum paramId,double value,int bufferMode);


/**
 * @brief 写入指定轴组的多个参数
 * 
 * @param grpId 轴组ID
 * @param index 指令索引
 * @param param 参数结构体指针
 * @param bufferMode 写入模式，0-立即生效，1-缓存模式,前一个指令执行完成后开始生效；
 * @return int 返回状态,0-成功，其他值失败
 */
int adx_GroupWriteMultiParameters(int grpId,int index,GrpParam_t *param,int bufferMode);




/**
 * @brief 读取指定轴组的多个参数
 * 
 * @param grpId 轴组ID
 * @param index 指令索引
 * @param param 参数结构体指针
 * @param bufferMode 写入模式，0-立即生效，1-缓存模式,前一个指令执行完成后开始生效；
 * @return int 返回状态,0-成功，其他值失败
 */
int adx_GroupReadMultiParameters(int grpId,int index,GrpParam_t *param,int bufferMode);


/**
 * @brief 设置轴组的速度倍率
 * 
 * @param grpId 轴组ID
 * @param index 指令索引
 * @param velFactor 速度倍率，>0.0-使用设定值，=0.0-不允许，<0,0-使用预先参数设置的值；
 * @param accFactor 加速度倍率，>0.0-使用设定值，=0.0-不允许，<0,0-使用预先参数设置的值；
 * @param jerkFactor 加加速度倍率，>0.0-使用设定值，=0.0-不允许，<0,0-使用预先参数设置的值；
 * @param bufferMode 缓冲模式，0-立即执行，1-缓存执行；
 * @return int 返回状态,0-成功，其他值失败
 */
int adx_GroupSetOverride(int  grpId,int index,double velFactor,double accFactor,double jerkFactor,int bufferMode);


/**
 * @brief 暂停轴组；暂停轴组运动，只是把倍率变为0，运动状态不改变；通过执行 adx_GroupContinue回恢复运动。
 * 
 * @param grpId 轴组ID
 * @param index 指令索引
 * @param deceleration 减速度
 * @param jerk 加加速度
 * @param bufferMode 缓冲模式，0-立即执行，1-缓存执行；
 * @return int 返回状态,0-成功，其他值失败
 */
int adx_GroupInterrupt(int grpId,int index,double deceleration,double jerk,int bufferMode);

/** 
 * @brief 恢复暂停的轴组；
 * 
 * @param grpId 轴组ID
 * @param index 指令索引
 * @param acceleration 加速度
 * @param jerk 加加速度
 * @param bufferMode 缓冲模式，0-立即执行，1-缓存执行；
 */
int adx_GroupContinue(int grpId,int index,double acceleration,double jerk,int bufferMode);

/** 
 * @brief 轴组减速停止。
 * 
 * @param grpId 轴组ID
 * @param index 指令索引
 * @param deceleration 减速度
 * @param jerk 加加速度
 * @param bufferMode 缓冲模式，0-立即执行，1-缓存执行；
 */
int adx_GroupStop(int grpId,int index,double deceleration,double jerk,int bufferMode);

/**
 * @brief 设置M代码的值
 * 
 * @param grpId 轴组ID;
 * @param index 指令索引;
 * @param forwardAddr 正向运动设置的地址;
 * @param reverseAddr 反向运动设置的地址;
 * @param setValue 设置值;
 * @param bufferMode 缓冲模式，0-立即执行，1-缓存执行;
 * @return int 返回状态,0-成功，其他值失败.
 */
int adx_GroupSetMcode(int grpId,int index,int *forwardAddr,int *reverseAddr,int forwardValue,int reverseValue,int bufferMode);



/**
 * @brief 设置设备状态.
 * 
 * @param grpId       轴组ID;
 * @param index       指令索引;
 * @param devId       设备ID;
 * @param status      设备状态;
 * @param direction   运动方向，0-正方向运动时起作用，1-负方向运动时起作用，2-无方向要求,均起作用；
 * @param bufferMode  缓冲模式，0-立即执行，1-缓存执行;
 * @return int        返回状态,0-成功，其他值失败.
 */
int adx_SetDeviceStatus(int grpId,int index, int devId,int direction,int forwardStatus,int reverseStatus,int bufferMode);

/**
 * @brief 读取设备状态.
 * 
 * @param grpId       轴组ID;
 * @param index       指令索引;
 * @param devId       设备ID;
 * @param status      设备状态存储指针;
 * @return int        返回状态,0-成功，其他值失败.
 */
int adx_ReadDeviceStatus(int grpId,int index, int devId,int direction,int *status);


/***********************************************************
 * 以下为非PLCopen运动控制接口
 * 
 **********************************************************/

int adx_ReadGroupBufferLength(int grpId,int *length);
 
////////////////////////////////////////////////////////////
//下位机设备状态读写函数注册接口


/**
 * @brief 注册设置设备状态的函数.
 * 
 * @param grpId           轴组ID;
 * @param index           指令索引;
 * @param devId           设备ID;
 * @param pSetDeviceFun   设置设备状态的函数指针，函数原型为： int SetDeviceStatus(int devId,int status);
 * @return int            返回状态,0-成功，其他值失败.
 */
int adx_RegisterSetDeviceFun(int grpId,int index, int devId,void *pSetDeviceFun);

/**
 * @brief 注册读取设备状态的函数.
 * 
 * @param grpId           轴组ID;
 * @param index           指令索引;
 * @param devId           设备ID;
 * @param pReadDeviceFun  读取设备状态的函数指针，函数原型为： int ReadDeviceStatus(int devId,int *status);
 * @return int            返回状态,0-成功，其他值失败.
 */
int adx_RegisterReadDeviceFun(int grpId,int index, int devId,void *pReadDeviceFun);

#ifdef __cplusplus
}
#endif

#endif /* FLEXCORE_H */
