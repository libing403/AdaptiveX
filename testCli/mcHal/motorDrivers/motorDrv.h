
#ifndef MOTORDRV_H
#define MOTORDRV_H

// #ifdef __cplusplus
// extern "C" {
// #endif


int setMotorEnableSim(int axisId, int enable, int timeoutMs);
int setMotorPositionSim(int axisId,double position,int timeOut_us);
int getMotorPositionSim(int axisId,double *position,int timeOut_us);
int stopSim();
// #ifdef __cplusplus
// }
// #endif

#endif // !MOTORDRV_H