#ifndef __INTERPOLATION_H__
#define __INTERPOLATION_H__
 
#ifdef __cplusplus
extern "C"{
#endif
#include "axisControl.h"
#include "groupControl.h"

int initLinearInp(AxisGroupControl_t *grp,double rate) ;
	
/**
 * \brief	线性插补函数.
 *
 * \param	robotNum
 * \return  返回零值执行成功，非零值执行失败。
 * \retval  VelPlanFailErr		轨迹规划失败。
 */
int linearInpterpolation(AxisGroupControl_t *grp,double rate);
int trajectoryLinearInterpolation(AxisGroupControl_t *grp,double rate);
int groupInterpolation(AxisGroupControl_t *grp);
int forwardKinematics(AxisGroupControl_t* grp);
int writeGrpTimeQ(AxisGroupControl_t* grp);
int inverseKinematics(AxisGroupControl_t* grp);

#ifdef __cplusplus
}
#endif

#endif
