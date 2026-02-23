#include "motorHal.h"
#include <string.h>
#include "remote_api_c_wrapper.h"

typedef struct {
    void *cli;
    void *sim;
    char jointName[12][64];
    uint64_t jointHandle[12];
}coppeliaSimMotor_t;
 

coppeliaSimMotor_t simHandle=
{
    .cli = NULL,
    .sim = NULL,
    .jointName = {
        "/toolMotorX",
        "/toolMotorY1",
        "/toolMotorY2",
        "/toolMotorZ",
        "/joint1",
        "/joint2",
        "/joint3",
        "/joint4",
        "/joint5",
        "/joint6",
        "/joint7",
        "/joint8"

    }

};

static int initialized = 0;
int setMotorEnableSim(int axisId, int enable,int timeOut_us)
{
    if(axisId<0||axisId>=12)
        return -1;
    if(initialized == 0)
    {
        simHandle.cli=sim_createClient();
        if(!simHandle.cli)
        {
            printf("sim_createClient error");
            return -1;
        }
        simHandle.sim=sim_createSim(simHandle.cli);
        if(!simHandle.sim)
        {
            printf("sim_createSim error");
            return -1;
        }
        sim_setStepping(simHandle.sim, 1);
        sim_startSimulation(simHandle.sim);

        for(int i=0;i<4;i++)
        {
            simHandle.jointHandle[i]=sim_getObject(simHandle.sim, simHandle.jointName[i]);
            if(simHandle.jointHandle[i]==0)
            {
                printf("sim_getObject %s error",simHandle.jointName[i]);
                return -1;
            }
        }
        initialized = 1;
    }
    return 0;
}

int setMotorPositionSim(int axisId,double position,int timeOut_us)
{
    if(axisId<0||axisId>=12)
        return -1;
    sim_setJointTargetPosition(simHandle.sim, simHandle.jointHandle[axisId], position);
    sim_step(simHandle.sim);
    return 0;
}

int getMotorPositionSim(int axisId,double *position,int timeOut_us)
{
    if(axisId<0||axisId>=12)
        return -1;
    *position=sim_getJointPosition(simHandle.sim, simHandle.jointHandle[axisId]);
    return 0;
}