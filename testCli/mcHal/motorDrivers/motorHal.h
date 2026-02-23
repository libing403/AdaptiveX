#ifndef MOTORHAL_H
#define MOTORHAL_H
#ifdef __cplusplus
extern "C" {
#endif
#include <stdint.h>
#include <stdbool.h>


int setMotorEnableSim(int axisId, int enable, int timeoutMs);
int setMotorPositionSim(int axisId,double position,int timeOut_us);
int getMotorPositionSim(int axisId,double *position,int timeOut_us);


#ifdef __cplusplus
}
#endif
#endif // MOTOR_HAL_H