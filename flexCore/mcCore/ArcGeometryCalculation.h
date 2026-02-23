/**
 * @file ArcGeometryCalculation.h
 * @brief 圆弧几何参数计算详细说明与示例
 * @date 2026-02-19
 */

/*
================================================================================
圆弧插补器的关键：从三个点计算圆弧几何参数

在运动控制中，圆弧通常由以下方式指定：
1. 三点法：P_start, P_mid, P_end
2. 圆心+半径法：center, radius, startAngle, sweepAngle
3. 圆心+两端点法：center, P_start, P_end

本文档重点讲解三点法（最常见）如何转换为圆心、半径、角度。
================================================================================
*/

#ifndef ARC_GEOMETRY_CALCULATION_H
#define ARC_GEOMETRY_CALCULATION_H

#ifdef __cplusplus
extern "C" {
#endif

#include <math.h>
#include <string.h>
#include "MathLib.h"
/* ============================================================================
   数学基础回顾
   ============================================================================

三点确定的圆弧：给定三个不共线的点 P1, P2, P3，存在唯一的圆弧通过这三点。

问题转化为：
1. 求圆心 C
2. 求半径 R
3. 求起始角 θ_start
4. 求终止角 θ_end

============================================================================ */
/**
 * \brief 圆弧几何参数
 * 定义圆弧的几何特性
 */
typedef struct {
    int type;               ///< 圆弧类型：0-圆弧, 1-整圆
    //位置参数
    double center[3];       ///< 圆心（XYZ）
    double length;          ///< 圆弧长度，= radius * |sweepAngle|
    double radius;          ///< 半径
    double theta;           ///< 扫过的圆心角，弧度
    double u1[3];           ///< 弧长向量1，u1=p2-p1方向;
    double u2[3];           ///< 弧长向量2，u2=p3-p2方向;  
    double normal[3];       ///< 平面法向量 n（圆弧所在平面）
    double w[3];            ///< 垂直于法向量n,v1的向量, w = n × v1
    //姿态参数
    double startQua[4];     ///< 起始姿态四元数,形式为(w,x,y,z)
    double startAngle;      ///< 起始姿态旋转角，弧度
    double startAxis[3];    ///< 起始姿态旋转轴，单位向量
    double midQua[4];       ///< 中间姿态四元数
    double midAngle;        ///< 中间姿态旋转角，弧度
    double midAxis[3];      ///< 中间姿态旋转轴，单位向量
    double endQua[4];       ///< 终止姿态四元数
    double endAngle;        ///< 终止姿态旋转角，弧度
    double endAxis[3];      ///< 终止姿态旋转轴，单位向量
} ArcGeometry_t;




#ifdef __cplusplus
}
#endif

#endif /* ARC_GEOMETRY_CALCULATION_H */
