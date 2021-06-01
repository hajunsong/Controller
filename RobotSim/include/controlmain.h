#include <iostream>
#include <vector>

using namespace std;

#include "robotarm.h"
#include "fileio.h"

#include <stdio.h>
#include <signal.h>
#include <unistd.h>
#include <sys/mman.h>

#include <native/task.h>
#include <native/timer.h>
#include <rtdk.h>

#include <pthread.h>

typedef unsigned int uint;

class ControlMain{
public:
    ControlMain();
    ~ControlMain();
    void start();
    void run_kinematics();
    void run_inverse_kinematics();
    void run_dynamics();
    RT_TASK robot_fk_task, robot_ik_task, robot_dy_task;
private:
    RobotArm *robotArm;

    vector<double> recurdyn_data1, recurdyn_data2, analysis_data_fk, analysis_data_ik, analysis_data_dy;
    uint size1[2], size2[2], row, col;
    FILE *fp_fk, *fp_ik, *fp_dy;
    double t_current, step_size;
    double q[6], q_dot[6], q_ddot[6], cur_q[6];
    double des_pose[6], end_pose[6], end_vel[6];
    double tool_offset[3];
    double fac_mat[36];
    int indx_array[6];
    bool fk_sim_run, ik_sim_run, dy_sim_run;
    unsigned long long period_time1, period_time2, sim_time1, sim_time2;
    unsigned long long fk_time1, fk_time2, ik_time1, ik_time2, dy_time1, dy_time2;
    double fk_time, ik_time, dy_time;
    uint indx;

    static void robot_FK_RT(void *arg);
    static void robot_IK_RT(void *arg);
    static void robot_DY_RT(void *arg);
    void robot_FK_RT_start();
    void robot_IK_RT_start();
    void robot_DY_RT_start();
};
