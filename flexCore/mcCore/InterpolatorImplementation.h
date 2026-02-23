/**
 * @file InterpolatorStructure.h
 * @brief 直线、圆弧、样条曲线插补器结构体定义
 * @date 2026-02-19
 */

#ifndef INTERPOLATOR_STRUCTURE_H
#define INTERPOLATOR_STRUCTURE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "axisControl.h"
#include "ArcGeometryCalculation.h"
#include "MathLib.h"

#ifndef MAX_CARTESIAN_COUNT
#define MAX_CARTESIAN_COUNT 6
#endif
/* ============================================================================
   基础插补器参数结构体（所有插补类型的公共部分）
   ============================================================================ */

/**
 * \brief 基础插补器信息
 * 包含所有插补类型都需要的参数
 */
typedef struct {
    int cmdNo;              ///< 命令号
    int index;              ///< 指令索引
    int dimension;          ///< 坐标维数
    int pathType;           ///< 路径类型：1-直线，2-圆弧，3-样条曲线
    
    double startPos[MAX_AXIS_COUNT];  ///< 起始位置
    double endPos[MAX_AXIS_COUNT];    ///< 终止位置
    double curPos[MAX_AXIS_COUNT];    ///< 当前位置
    double prePos[MAX_AXIS_COUNT];    ///< 前一位置
    double li;                   ///< 已插补长度（路径参数）
    double preLi;                ///< 前一插补长度
    double ti;                   ///< 局部时间坐标
    double initTime;             ///< 偏置时间
    double time;            ///< 插补时间戳
    double syncTime;        ///< 同步时间戳
    double dt;              ///< 控制周期
    
    double linearVelocity;      ///< 线速度
    double linearAcceleration;  ///< 线加速度
    double linearJerk;          ///< 线加加速度
    
    int state;              ///< 插补状态: 0-空闲, 1-插补中, 2-最后一步, 3-初始化中
    TrapeProfile_t tp;      ///< 速度轮廓
} InterpolatorBase_t;

/* ============================================================================
   直线插补器结构体
   ============================================================================ */
#define LINE_POS_DIM 3
#define LINE_QUA_DIM 4

typedef struct {
    double startPos[LINE_POS_DIM];          ///< 直线起点位置(x,y,z)
    double endPos[LINE_POS_DIM];            ///< 直线终点位置(x,y,z)
    double deltaPos[LINE_POS_DIM];          ///< 位移向量(end-start)
    double dirPos[LINE_POS_DIM];            ///< 位置插补单位方向向量
    double curPos[LINE_POS_DIM];            ///< 当前位置(x,y,z)
    double curVel[LINE_POS_DIM];            ///< 当前位置线速度向量(x,y,z)
    double linearVelocity;                  ///< 当前位置线速度标量
    double posLength;                       ///< 位置路径长度 ||end-start||
    double invPosLength;                    ///< 位置路径长度倒数（零长度时为0）
} LinePositionInterpData_t;

typedef struct {
    double startQua[LINE_QUA_DIM];          ///< 起始姿态四元数(w,x,y,z)
    double endQua[LINE_QUA_DIM];            ///< 目标姿态四元数(w,x,y,z)
    double endQuaAligned[LINE_QUA_DIM];     ///< 同半球对齐后的目标四元数
    double quaDot;                          ///< 起终四元数点积（用于判断符号翻转）
    double curQua[LINE_QUA_DIM];            ///< 当前姿态四元数(w,x,y,z)
    double theta;                           ///< 起终旋转角(弧度)
    double rotAxis[LINE_POS_DIM];           ///< 起终姿态相对旋转轴（单位向量）
    double angularVelocity;                 ///< 当前姿态角速度标量(rad/s)
    double angularVelocityVec[LINE_POS_DIM];///< 当前姿态角速度向量(rad/s)
    double rotLength;                       ///< 旋转等效长度
} LineAttitudeInterpData_t;

typedef struct {
    double startPose[MAX_CARTESIAN_COUNT];  ///< 起点位姿
    double endPose[MAX_CARTESIAN_COUNT];    ///< 终点位姿
    double dirVec[LINE_POS_DIM];            ///< 路径方向矢量
    double totalLength;                     ///< 位姿同步后的总路径长度
} LineGeometry_t;

/**
 * \brief 直线插补器
 * 基于参数方程：P(s) = P_start + s * vec，其中 0 <= s <= length
 */
typedef struct {
    InterpolatorBase_t base;     ///< 基础插补器参数
    LineGeometry_t geom;          ///< 底层几何信息
    LinePositionInterpData_t position;  ///< 位置插补参数（插补器层）
    LineAttitudeInterpData_t attitude;  ///< 姿态插补参数（插补器层）
    double s;                     ///< 归一化插补参数，范围[0,1]
    double preS;                  ///< 前一周期归一化插补参数

} LineInterpolator_t;

/* ============================================================================
   圆弧插补器结构体
============================================================================ */

/**
 * \brief 圆弧插补器参数
 */
typedef struct {
    double startPose[MAX_CARTESIAN_COUNT];  ///< 起点位姿
    double midPose[MAX_CARTESIAN_COUNT];   ///< 中间点（三点法确定圆弧）
    double endPose[MAX_CARTESIAN_COUNT];    ///< 终点位姿
    ArcGeometry_t geom;                ///< 圆弧几何参数

} ArcInterpolatorParam_t;

/**
 * \brief 圆弧插补器
 * 参数方程：P(θ) = center + radius * (cos(θ) * u1 + sin(θ) * u2)
 * 其中 θ ∈ [startAngle, endAngle]
 */
typedef struct {
    InterpolatorBase_t base;     ///< 基础插补器参数
    ArcInterpolatorParam_t param;
    double theta;                ///< 当前角度（弧度）
    double preTheta;             ///< 前一角度
    
} ArcInterpolator_t;

/* ============================================================================
   样条曲线插补器结构体
   ============================================================================ */

/**
 * \brief B样条参数
 * 用于NURBS/B样条曲线的参数
 */
typedef struct {
    double *ctrlPoints;         ///< 控制点数组 (n_ctrl * dim)
    uint32_t ctrlPointCount;    ///< 控制点数量
    
    double *knotVector;         ///< 节点向量
    uint32_t knotSize;          ///< 节点向量大小
    
    uint32_t degree;            ///< B样条次数（最常用3=三次）
    uint32_t dimension;         ///< 空间维数
    
    double totalLength;         ///< 样条曲线总长度（需预计算）
    
} BSplineParam_t;

/**
 * \brief 样条曲线段节点
 * 用于标记样条曲线的分段参数域
 */
typedef struct {
    uint32_t spanIndex;         ///< 样条段索引
    double uStart;              ///< 参数区间起始
    double uEnd;                ///< 参数区间终止
    double sStart;              ///< 弧长起始（累积）
    double sEnd;                ///< 弧长终止（累积）
    double segmentLength;       ///< 该段长度
} SplineSegment_t;

/**
 * \brief 样条曲线插补器参数
 */
typedef struct {
    double startPose[MAX_AXIS_COUNT];  ///< 起点位姿
    double endPose[MAX_AXIS_COUNT];    ///< 终点位姿
    
    BSplineParam_t bspline;            ///< B样条参数
    
    // 用于弧长参数化
    SplineSegment_t *segments;         ///< 分段信息数组
    uint32_t segmentCount;             ///< 段数
    double *arcLengthTable;            ///< 弧长查找表（可选）
    
} SplineInterpolatorParam_t;

/**
 * \brief 样条曲线插补器
 * 参数方程：P(u) = NURBS曲线，u ∈ [u_start, u_end]
 * 其中使用弧长参数 s 参数化：s = cumulative_arc_length(u)
 */
typedef struct {
    InterpolatorBase_t base;         ///< 基础插补器参数
    SplineInterpolatorParam_t param;
    
    double arcLength;                ///< 已插补的弧长
    double preArcLength;             ///< 前一插补的弧长
    
    double u;                        ///< 当前参数值
    double preU;                     ///< 前一参数值
    
} SplineInterpolator_t;

/* ============================================================================
   统一插补器接口（使用联合体支持多类型路径）
   ============================================================================ */

/**
 * \brief 支持多路径类型的通用插补器
 * 在轴组中使用此结构体，根据 base.pathType 选择相应的插补处理函数
 */
typedef union {
    InterpolatorBase_t    base;        ///< 基础参数（所有类型共用）
    LineInterpolator_t    linear;      ///< 直线插补器
    ArcInterpolator_t     arc;         ///< 圆弧插补器
    SplineInterpolator_t  spline;      ///< 样条曲线插补器
} UnifiedInterpolator_t;

/* ============================================================================
   插补器操作回调接口
   ============================================================================ */

/**
 * \brief 插补器初始化函数指针
 */
typedef int (*InterpolatorInit_fp)(struct AxisGroupControl_s *grp, double rate);

/**
 * \brief 插补执行函数指针
 */
typedef int (*InterpolatorExecute_fp)(struct AxisGroupControl_s *grp, double rate);

/**
 * \brief 插补器查询函数指针（用于获取当前位置/速度等）
 */
typedef int (*InterpolatorQuery_fp)(struct AxisGroupControl_s *grp, double *pos, double *vel);

/**
 * \brief 插补器清理函数指针
 */
typedef int (*InterpolatorCleanup_fp)(struct AxisGroupControl_s *grp);

/**
 * \brief 插补器操作接口表
 * 为不同的路径类型定义对应的处理函数集
 */
typedef struct {
    InterpolatorInit_fp     init;       ///< 初始化函数
    InterpolatorExecute_fp  execute;    ///< 执行插补函数
    InterpolatorQuery_fp    query;      ///< 查询函数
    InterpolatorCleanup_fp  cleanup;    ///< 清理函数
} InterpolatorOps_t;

typedef struct AxisGroupControl_t AxisGroupControl_t;

/**
 * \brief 初始化直线路径位姿同步插补器。
 *
 * 该函数将“位姿插补”统一映射到标量路径长度：
 * L_total = max(L_pos, L_rot)
 * 其中：
 *  - L_pos = ||P1 - P0||
 *  - theta = 2 * acos(dot(q0, q1_aligned))
 *  - L_rot = theta / wMax * vMax
 *
 * \param interp     直线插补器对象。
 * \param startPos   起点位置[x,y,z]。
 * \param endPos     终点位置[x,y,z]。
 * \param startQua   起点姿态四元数(w,x,y,z)。
 * \param endQua     终点姿态四元数(w,x,y,z)。
 * \param vMax       线速度上限。
 * \param wMax       角速度上限。
 * \param accMax     线加速度上限。
 * \param decMax     线减速度上限。
 * \param dt         控制周期。
 * \return 0 成功；非0 失败。
 */
int initLineInterpolator(struct AxisGroupControl_t *grp,
                         double rate);

/**
 * \brief 执行一步直线位姿同步插补。
 *
 * 流程：
 *  1) 由梯形速度曲线得到标量路径长度 x(t)
 *  2) 归一化 s(t)=x(t)/L_total
 *  3) 位置 p(t)=p0+s(t)*(p1-p0)
 *  4) 姿态 q(t)=SLERP(q0,q1,s(t))
 *
 * \param interp      直线插补器对象。
 * \param rate        运行倍率（可正向/反向）。
 * \param outPos      输出位置[x,y,z]，允许为NULL。
 * \param outQua      输出姿态四元数(w,x,y,z)，允许为NULL。
 * \param isFinished  完成标志输出，允许为NULL。
 * \return 0 成功；非0 失败。
 */
int executeLineInterpolator(LineInterpolator_t *interp,
                            double rate,
                            double outPos[3],
                            double outQua[4],
                            int *isFinished);

/**
 * \brief 兼容 Interpolation.c 流程：读取当前轨迹并初始化直线位姿插补器。
 */
int initLinePoseInp(AxisGroupControl_t *grp,
                    LineInterpolator_t *interp,
                    double rate,
                    const double startQua[4],
                    const double endQua[4],
                    double wMax);

/**
 * \brief 兼容 Interpolation.c 流程：执行一步直线位姿插补并回写 grp->inp。
 */
int linePoseInterpolation(AxisGroupControl_t *grp,
                          LineInterpolator_t *interp,
                          double rate,
                          double outQua[4]);

/**
 * \brief 兼容 Interpolation.c 流程：状态驱动的直线位姿插补入口。
 */
int trajectoryLinePoseInterpolation(AxisGroupControl_t *grp,
                                    LineInterpolator_t *interp,
                                    double rate,
                                    const double startQua[4],
                                    const double endQua[4],
                                    double wMax,
                                    double outQua[4]);

#ifdef __cplusplus
}
#endif

#endif /* INTERPOLATOR_STRUCTURE_H */
