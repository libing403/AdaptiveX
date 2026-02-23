#include "remote_api_c_wrapper.h"
#include <stdio.h>



int main()
{

    void *cli = sim_createClient();
    void *sim = sim_createSim(cli);
    sim_setStepping(sim, 1);
    sim_startSimulation(sim);
    uint64_t joint_X = sim_getObject(sim, "/toolMotorX");
    uint64_t joint_Y = sim_getObject(sim, "/toolMotorY1");
    uint64_t joint_Z = sim_getObject(sim, "/toolMotorZ");
    double x_target = 0, y_target = 0, z_target = 0;
    while (sim_getSimulationTime(sim) < 4.0)
    {
        sim_setJointTargetPosition(sim, joint_X, x_target);
        sim_setJointTargetPosition(sim, joint_Y, y_target);
        sim_setJointTargetPosition(sim, joint_Z, z_target);
        x_target += 100 * 1e-3 * 1e-3 * 50;
        y_target += 100 * 1e-3 * 1e-3 * 50;
        z_target += 100 * 1e-3 * 1e-3 * 50;
        double x_cur = sim_getJointPosition(sim, joint_X);
        double y_cur = sim_getJointPosition(sim, joint_Y);
        double z_cur = sim_getJointPosition(sim, joint_Z);
        printf("x_current = %.5f  y_current = %.5f  z_current = %.5f\n", x_cur, y_cur, z_cur);  
        sim_step(sim);
    }
    sim_stopSimulation(sim);
    sim_destroySim(sim);
    sim_destroyClient(cli);
    return 0;

}