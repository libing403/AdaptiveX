#ifndef API_C_WRAPPER_H_
#define API_C_WRAPPER_H_
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif





void* sim_createClient();
void sim_destroyClient(void* h);

void* sim_createSim(void* h);
void sim_destroySim(void* h);

// Object 查找
uint64_t sim_getObject(void* h, const char* name);

// 运动控制 API
void sim_setStepping(void* h, int enable);
void sim_startSimulation(void* h);
void sim_stopSimulation(void* h);
void sim_step(void* h);
void sim_setJointTargetPosition(void* h, uint64_t joint, double pos);
double sim_getJointPosition(void* h, uint64_t joint);
double sim_getSimulationTime(void* h);


#ifdef __cplusplus
}
#endif

#endif