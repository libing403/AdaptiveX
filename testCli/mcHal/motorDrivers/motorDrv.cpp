
#include "RemoteAPIClient.h"
#include<string.h>
typedef struct {
    char jointName[12][64];
    uint64_t joint[12];
}coppeliaSimMotor_t;
coppeliaSimMotor_t motorHandle=
{
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

static RemoteAPIClient client;
static RemoteAPIObject::sim simHandle=client.getObject().sim();

static int initialized = 0;

int setMotorEnableSim(int axisId, int enable,int timeOut_us)
{
    if(axisId<0||axisId>=12)
        return -1;
    if(initialized == 0)
    {

        //simHandle=client.getObject().sim();
        simHandle.setStepping(true);
        simHandle.startSimulation();

        for(int i=0;i<4;i++)
        {
            motorHandle.joint[i]=simHandle.getObject(motorHandle.jointName[i]);
            if(motorHandle.joint[i]==0)
            {
                printf("sim_getObject %s error",motorHandle.jointName[i]);
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
    simHandle.setJointTargetPosition(motorHandle.joint[axisId], position);    
    simHandle.step();
    return 0;
}

int getMotorPositionSim(int axisId,double *position,int timeOut_us)
{
    if(axisId<0||axisId>=12)
        return -1;
    *position=simHandle.getJointPosition(motorHandle.joint[axisId]);
    
    return 0;
}

int stopSim()
{
    simHandle.stopSimulation();
    return 0;
}