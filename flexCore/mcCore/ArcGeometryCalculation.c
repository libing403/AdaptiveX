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
#include "ArcGeometryCalculation.h"
#include "MathLib.h"
#include "InterpolatorStructure.h"
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
 * @brief 计算圆弧几何参数
 * 
 * @param type  圆弧类型：0-圆弧, 1-整圆;
 * @param p1    圆弧起点坐标;
 * @param p2    圆弧中点坐标;
 * @param p3    圆弧终点坐标;
 * @param geom  输出参数，包含圆心、半径、起始角、扫过角等几何信息
 * @return int  0-成功，非0-失败（如共线、无法计算等）
 */
int calcArcParam(int type, double *p1, double *p2, double *p3, ArcGeometry_t *geom) 
{
    //mathlab代码实现
    /*
    %圆心及半径
    u1=(p2-p1)/norm(p2-p1);
    u2=(p3-p2)/norm(p3-p2);
    oe=(p1+p2)/2;
    of=(p2+p3)/2;
    n1=cross(u1,u2);
    n1=n1/norm(n1);%单位化
    m1=cross(n1,u1);
    m1=m1/norm(m1);
    oc=oe+(dot(u2,of)-dot(u2,oe))/(dot(u2,m1))*m1;
    R=norm(oc-p1);
    %圆心角计算
    v1=(p1-oc)/norm(p1-oc);
    v2=(p3-oc)/norm(p2-oc);
    n2=cross(v1,v2);
    n2=n2/norm(n2);
    dt=dot(v1,v2);
    if(dt>1.0)
        dt=1.0;
    elseif(dt<-1.0)
        dt=-1.0;
    end
    if(type==1)
        alpha=2*3.1415926;
    elseif(norm(n1-n2)<1.0e-5)
        alpha=acos(dt);
    else
        alpha=2*3.1415926-acos(dt);
    end

    arc_t.type=type;
    arc_t.L=alpha*R;
    arc_t.p1=p1;
    arc_t.p2=p2;
    arc_t.p3=p3;
    arc_t.oc=oc;
    arc_t.R=R;
    arc_t.u1=u1;
    arc_t.u2=u2;
    arc_t.n1=n1;
    arc_t.n2=n2;
    arc_t.v1=v1;
    arc_t.v2=v2;
    */
    //C代码实现
    int ret = 0;
    double u1[3], u2[3], m1[3], m2[3], of[3],oe[3],oc[3],normal[3];
    vec3_sub(p2, p1, u1);
    vec3_sub(p3, p2, u2);
     
    // 计算法向量 n = u1 × u2
    vec3_cross(u1, u2, normal); 
    // 检查是否共线
    double norm_n = sqrt(vec3_dot(normal, normal));
    if (norm_n < 1.0e-5) {
        ULOG_ERROR("calculateArcGeometry: points are collinear");
        return 1;
    }
    // 归一化法向量    vec3_mult_value(normal, 1.0 / norm_n, normal);
    vec3_mult_value(normal, 1.0 / norm_n, normal);
    vec3_cpy(geom->normal, normal);
    ret=vec3_unit(u1, u1);
    if (ret != 0) {
        ULOG_ERROR("calculateArcGeometry: cannot normalize u1");
        return 1;
    }
    ret=vec3_unit(u2, u2);
    if (ret != 0) {
        ULOG_ERROR("calculateArcGeometry: cannot normalize u2");
        return 1;
    }

    vec3_add(p1, p2, m1);
    vec3_mult_value(m1, 0.5, oe); // m1 = (p1 + p2) / 2
    vec3_add(p2, p3, m2);
    vec3_mult_value(m2, 0.5, of); // m2 = (p2 + p3) / 2
    // 垂直平分线方向
     
    vec3_cross(normal, u1, m1); // m1 ⊥ n, m1 ⊥ u1
    vec3_cross(normal, u2, m2); // m2 ⊥ n, m2 ⊥ u2
    ret=vec3_unit(m1, m1);
    if (ret != 0) {
        ULOG_ERROR("calculateArcGeometry: cannot normalize m1");
        return 1;
    }
    ret=vec3_unit(m2, m2);
    if (ret != 0) {
        ULOG_ERROR("calculateArcGeometry: cannot normalize m2");
        return 1;
    }
    // 求交点：oe + t1*m1 = of + t2*m2
    double t1, t2;
    t1=ve3_dot(u2, of) - vec3_dot(u2, oe);
    t2=vec3_dot(u2, m1);
    if (fabs(t2) < 1.0e-10) {
        ULOG_ERROR("calculateArcGeometry: cannot find arc center");
        return 1;
    }
    t1 = t1 / t2;
    // 圆心 oc = oe + t1*m1
    vec3_mult_value(m1, t1, m1);
    vec3_add(oe, m1, oc);
    // 半径 R = ||oc - p1||
    double radius;
    vec3_sub(oc, p1, m1);
    radius = sqrt(vec3_dot(m1, m1));
    geom->radius = radius;
    vec3_cpy(oc, geom->center);
    // 计算圆心角
    double v1[3], v2[3];
    vec3_sub(p1, oc, v1);
    vec3_sub(p3, oc, v2);
    ret=vec3_unit(v1, v1);
    if (ret != 0) {
        ULOG_ERROR("calculateArcGeometry: cannot normalize v1");
        return 1;
    }
    ret=vec3_unit(v2, v2);
    if (ret != 0) {
        ULOG_ERROR("calculateArcGeometry: cannot normalize v2");
        return 1;
    }
    double n2[3];
    vec3_cross(v1, v2, n2);
    ret=vec3_unit(n2, n2);
    if (ret != 0) {
        ULOG_ERROR("calculateArcGeometry: cannot normalize n2");
        return 1;
    }
    double dt = vec3_dot(v1, v2);
    if (dt > 1.0) dt = 1.0;
    else if (dt < -1.0)     dt = -1.0;
    double theta;
    if (type == 1) 
    {
        theta = 2.0 * PM_PI;
    }
    else 
    {
        
        if (vec3_dot(normal, n2) > 0) {
            theta = acos(dt);
        }
        else {
            theta = 2.0 * PM_PI - acos(dt);
        }
    }
    double w[3];
    vec3_cross(normal,v1,w);
    ret=vec3_unit(w,w);
    if (ret != 0) {
        ULOG_ERROR("calculateArcGeometry: cannot normalize w");
        return 1;
    }
    geom->theta = theta;
    geom->length = theta * radius;

    vec3_cpy(u1, geom->u1);
    vec3_cpy(u2, geom->u2);
    vec3_cpy(w, geom->w);

    //根据rpy角计算姿态旋转轴和旋转角
    double roll, pitch, yaw;
    roll=p1[3];
    pitch=p1[4];
    yaw=p1[5];
    double rpy[3]={roll, pitch, yaw};
    double R[3][3];
    rpy2r(rpy, R);
    double q[4];
    rot2q(R, q);
    geom->startQua[0] = q[3];
    geom->startQua[1] = q[0];
    geom->startQua[2] = q[1];
    geom->startQua[3] = q[2];
    geom->

    //中间姿态四元数
    roll=p2[3];
    pitch=p2[4];
    yaw=p2[5];
    double rpy2[3]={roll, pitch, yaw};
    double R2[3][3];
    rpy2r(rpy2, R2);
    double q2[4];
    rot2q(R2, q2);
    geom->midQua[0] = q2[3];
    geom->midQua[1] = q2[0];
    geom->midQua[2] = q2[1];
    geom->midQua[3] = q2[2];
    //终止姿态四元数
    roll=p3[3];
    pitch=p3[4];
    yaw=p3[5];
    double rpy3[3]={roll, pitch, yaw};
    double R3[3][3];
    rpy2r(rpy3, R3);
    double q3[4];
    rot2q(R3, q3);
    geom->endQua[0] = q3[3];
    geom->endQua[1] = q3[0];
    geom->endQua[2] = q3[1];
    geom->endQua[3] = q3[2];

    return 0;
}





#ifdef __cplusplus
}
#endif

#endif /* ARC_GEOMETRY_CALCULATION_H */
