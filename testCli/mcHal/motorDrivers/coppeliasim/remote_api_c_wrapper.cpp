#include "remote_api_c_wrapper.h"
#include "RemoteAPIClient.h"
 
extern "C" {

void *sim_createClient()
{
     
    return new RemoteAPIClient();

}

void *sim_createSim(void *h)
{
    auto w = (RemoteAPIClient*)h;
    return new RemoteAPIObject::sim(w);
}

void sim_destroySim(void *h)
{
    auto w = (RemoteAPIObject::sim*)h;
    delete w;
}

void sim_destroyClient(void* h)
{
    auto w = (RemoteAPIClient*)h;
    delete w;
}

uint64_t sim_getObject(void* h, const char* name)
{
    return ((RemoteAPIObject::sim*)h)->getObject(name);
}


void sim_setStepping(void* h, int enable)
{
    ((RemoteAPIObject::sim*)h)->setStepping(enable != 0);
}    

void sim_startSimulation(void* h)
{
    ((RemoteAPIObject::sim*)h)->startSimulation();
}

void sim_stopSimulation(void* h)
{
    ((RemoteAPIObject::sim*)h)->stopSimulation();
}

void sim_step(void* h)
{
    ((RemoteAPIObject::sim*)h)->step();
}

void sim_setJointTargetPosition(void* h, uint64_t joint, double pos)
{
    ((RemoteAPIObject::sim*)h)->setJointTargetPosition(joint, pos);
}

double sim_getJointPosition(void* h, uint64_t joint)
{
    return ((RemoteAPIObject::sim*)h)->getJointPosition(joint);
}

double sim_getSimulationTime(void* h)
{
    return ((RemoteAPIObject::sim*)h)->getSimulationTime();
}



} // extern "C"
