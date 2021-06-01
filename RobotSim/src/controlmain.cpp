#include "controlmain.h"

static double temp = 0;

ControlMain::ControlMain()
{
    recurdyn_data1.clear();
    recurdyn_data2.clear();
    analysis_data_fk.clear();
    analysis_data_ik.clear();
    analysis_data_dy.clear();

    memset(size1, 0, sizeof(unsigned int)*2);
    memset(size2, 0, sizeof(unsigned int)*2);
    row = 0;
    col = 0;

    t_current = 0;
    step_size = 0.001;

    memset(q, 0, sizeof(double)*6);
    memset(q_dot, 0, sizeof(double)*6);
    memset(des_pose, 0, sizeof(double)*6);
    memset(end_pose, 0, sizeof(double)*6);
    memset(end_vel, 0, sizeof(double)*6);
    memset(cur_q, 0, sizeof(double)*6);
    memset(fac_mat, 0, sizeof(double)*36);
    memset(indx_array, 0, sizeof(int)*6);

    fk_sim_run = false;
    ik_sim_run = false;
    dy_sim_run = false;

    indx = 0;

    fk_time1 = 0;
    fk_time2 = 0;
    ik_time1 = 0;
    ik_time2 = 0;
    dy_time1 = 0;
    dy_time2 = 0;
    fk_time = 0;
    ik_time = 0;
    dy_time = 0;

    robotArm = new RobotArm(6, 6, step_size, FAR_V3);
    tool_offset[0] = -4.77396e-18;
    tool_offset[1] = 0.14008;
    tool_offset[2] = -0.005;
    robotArm->set_tool_offset(tool_offset);

    load_data("data/recurdyn_data2.txt", &recurdyn_data1, "\t", size1);
    load_data("data/recurdyn_data3.txt", &recurdyn_data2, "\t", size2);

    printf("Recurdyn data1 size : %d, %d\n", size1[0], size1[1]);
    printf("Recurdyn data2 size : %d, %d\n", size2[0], size2[1]);

    fp_fk = fopen("data/analysis_data_kinematics.txt", "w+");
    fp_ik = fopen("data/analysis_data_inverse_kinematics.txt", "w+");
    fp_dy = fopen("data/analysis_data_dynamics.txt", "w+");
}

ControlMain::~ControlMain()
{
    printf("finishing...\n");

    recurdyn_data1.clear();
    recurdyn_data2.clear();
    analysis_data_fk.clear();
    analysis_data_ik.clear();
    analysis_data_dy.clear();

    fclose(fp_fk);
    fclose(fp_ik);
    fclose(fp_dy);

    printf("Finished\n");
}

void ControlMain::start()
{
    printf("Start FK\n");
    robot_FK_RT_start();
    usleep(10000);
    while(fk_sim_run){
        usleep(10000);
    }
    rt_task_suspend(&robot_fk_task);
    usleep(100000);
    rt_task_delete(&robot_fk_task);
    usleep(100000);
    printf("Finished FK\n");

    printf("Start IK\n");
    robot_IK_RT_start();
    usleep(10000);
    while(ik_sim_run){
        usleep(10000);
    }
    rt_task_suspend(&robot_ik_task);
    usleep(100000);
    rt_task_delete(&robot_ik_task);
    usleep(100000);
    printf("Finished IK\n");

    printf("Start DY\n");
    robot_DY_RT_start();
    usleep(10000);
    while(dy_sim_run){
        usleep(10000);
    }
    rt_task_suspend(&robot_dy_task);
    usleep(100000);
    rt_task_delete(&robot_dy_task);
    usleep(100000);
    printf("Finished DY\n");
}

void ControlMain::robot_FK_RT_start()
{
    rt_print_auto_init(1);

    /* Avoids memory swapping for this program */
    mlockall(MCL_CURRENT|MCL_FUTURE);

    int ret = rt_task_create(&robot_fk_task, "Robot Controll Task", 0, 99, T_FPU);
    if (ret < 0){
        printf("Failed to create Robot Control Task %s\n", strerror(-ret));
    }
    else{
        printf("Robot Control FK RT Task Start\n");

        rt_task_start(&robot_fk_task, &robot_FK_RT, this);
    }
}

void ControlMain::robot_IK_RT_start()
{
    rt_print_auto_init(1);

    /* Avoids memory swapping for this program */
    mlockall(MCL_CURRENT|MCL_FUTURE);

    int ret = rt_task_create(&robot_ik_task, "Robot Controll Task", 0, 99, T_FPU);
    if (ret < 0){
        printf("Failed to create Robot Control Task %s\n", strerror(-ret));
    }
    else{
        printf("Robot Control IK RT Task Start\n");

        rt_task_start(&robot_ik_task, &robot_IK_RT, this);
    }
}

void ControlMain::robot_DY_RT_start()
{
    rt_print_auto_init(1);

    /* Avoids memory swapping for this program */
    mlockall(MCL_CURRENT|MCL_FUTURE);

    int ret = rt_task_create(&robot_dy_task, "Robot Controll Task", 0, 99, T_FPU);
    if (ret < 0){
        printf("Failed to create Robot Control Task %s\n", strerror(-ret));
    }
    else{
        printf("Robot Control DY RT Task Start\n");

        rt_task_start(&robot_dy_task, &robot_DY_RT, this);
    }
}

void ControlMain::robot_FK_RT(void *arg)
{
    ControlMain *pThis = static_cast<ControlMain*>(arg);
    pThis->period_time1= rt_timer_read();
    pThis->fk_sim_run = false;

    rt_task_set_periodic(&pThis->robot_fk_task, TM_NOW, 1e6);

    pThis->indx = 0;
    pThis->t_current = 0;
    pThis->row = pThis->size1[0];
    pThis->col = pThis->size1[1];

    pThis->fk_sim_run = true;
    while(pThis->indx < pThis->row){
        rt_task_wait_period(nullptr);
        pThis->period_time2 = rt_timer_read();

        for(uint j = 0; j < 6; j++){
            pThis->q[j] = pThis->recurdyn_data1[pThis->indx*pThis->col + j+2];
            pThis->q_dot[j] = pThis->recurdyn_data1[pThis->indx*pThis->col + j+14];
        }

        pThis->fk_time1 = rt_timer_read();
        pThis->robotArm->run_kinematics(pThis->q, pThis->end_pose);
        pThis->fk_time2 = rt_timer_read();

        pThis->robotArm->jacobian_zyx();
        for(uint i = 0; i < 6; i++)
        {
            temp = 0;
            for(uint j = 0; j < 6; j++)
            {
                temp += pThis->robotArm->J[i*6 + j]*pThis->q_dot[j];
            }
            pThis->end_vel[i] = temp;
        }

        pThis->analysis_data_fk.push_back(pThis->t_current);
        for(uint j = 0; j < 6; j++){
            pThis->analysis_data_fk.push_back(pThis->q[j]);
        }
        for(uint j = 0; j < 6; j++){
            pThis->analysis_data_fk.push_back(pThis->end_pose[j]);
        }
        for(uint j = 0; j < 6; j++){
            pThis->analysis_data_fk.push_back(pThis->q_dot[j]);
        }
        for(uint j = 0; j < 6; j++){
            pThis->analysis_data_fk.push_back(pThis->end_vel[j]);
        }

        pThis->t_current += static_cast<double>((pThis->period_time2 - pThis->period_time1)/1000000.0);
        pThis->fk_time = static_cast<double>((pThis->fk_time2 - pThis->fk_time1)/1000000.0);
        pThis->analysis_data_fk.push_back(pThis->fk_time);
        pThis->indx++;

        pThis->period_time1 = pThis->period_time2;

        rt_printf("[FK]T_current : %5.5f\n", pThis->t_current);
    }

    printf("FK data write start\n");

    for(uint i = 0; i < pThis->row; i++){
        fprintf(pThis->fp_fk, "%d\t", i);
        for(uint j = 0; j < 26; j++){
            fprintf(pThis->fp_fk, "%3.7f\t", pThis->analysis_data_fk[i*26+j]);
        }
        fprintf(pThis->fp_fk, "\n");
    }

    printf("FK data write finished\n");
    printf("Finished FK simulation\n");

    pThis->fk_sim_run = false;
}

void ControlMain::robot_IK_RT(void *arg)
{
    ControlMain *pThis = static_cast<ControlMain*>(arg);
    pThis->period_time1= rt_timer_read();
    pThis->ik_sim_run = false;

    rt_task_set_periodic(&pThis->robot_ik_task, TM_NOW, 1e6);

    pThis->indx = 0;
    pThis->t_current = 0;
    pThis->row = pThis->size1[0];
    pThis->col = pThis->size1[1];

    pThis->ik_sim_run = true;

    for(uint j = 0; j < 6; j++){
        pThis->cur_q[j] = pThis->recurdyn_data1[0*pThis->col + j+2];
    }

    pThis->robotArm->run_kinematics(pThis->cur_q, pThis->end_pose);

    while(pThis->indx < pThis->row){
        rt_task_wait_period(nullptr);
        pThis->period_time2 = rt_timer_read();

        for(uint j = 0; j < 6; j++){
            pThis->des_pose[j] = pThis->recurdyn_data1[pThis->indx*pThis->col + j+8];
            pThis->end_vel[j] = pThis->recurdyn_data1[pThis->indx*pThis->col + j+20];
        }

        for(uint j = 0; j < 6; j++){
            pThis->cur_q[j] = pThis->robotArm->body[j+1].qi;
        }

        pThis->ik_time1 = rt_timer_read();
        pThis->robotArm->run_inverse_kinematics(pThis->cur_q, pThis->des_pose, pThis->q, pThis->end_pose);

        memset(pThis->indx_array, 0, sizeof(int)*6);
        pThis->robotArm->numeric->ludcmp(pThis->robotArm->J, 6, pThis->indx_array, 0.0, pThis->fac_mat);
        pThis->robotArm->numeric->lubksb(pThis->fac_mat, 6, pThis->indx_array, pThis->end_vel, pThis->q_dot);
        pThis->ik_time2 = rt_timer_read();

        pThis->analysis_data_ik.push_back(pThis->t_current);
        for(uint j = 0; j < 6; j++){
            pThis->analysis_data_ik.push_back(pThis->q[j]);
        }
        for(uint j = 0; j < 6; j++){
            pThis->analysis_data_ik.push_back(pThis->end_pose[j]);
        }
        for(uint j = 0; j < 6; j++){
            pThis->analysis_data_ik.push_back(pThis->q_dot[j]);
        }
        for(uint j = 0; j < 6; j++){
            pThis->analysis_data_ik.push_back(pThis->end_vel[j]);
        }

        pThis->t_current += static_cast<double>((pThis->period_time2 - pThis->period_time1)/1000000.0);
        pThis->ik_time = static_cast<double>((pThis->ik_time2 - pThis->ik_time1)/1000000.0);
        pThis->analysis_data_ik.push_back(pThis->ik_time);
        pThis->indx++;

        pThis->period_time1 = pThis->period_time2;

        rt_printf("[IK]T_current : %5.5f\n", pThis->t_current);
    }

    printf("IK data write start\n");

    for(uint i = 0; i < pThis->row; i++){
        fprintf(pThis->fp_ik, "%d\t", i);
        for(uint j = 0; j < 26; j++){
            fprintf(pThis->fp_ik, "%3.7f\t", pThis->analysis_data_ik[i*26+j]);
        }
        fprintf(pThis->fp_ik, "\n");
    }

    printf("IK data write finished\n");
    printf("Finished IK simulation\n");

    pThis->ik_sim_run = false;
}

void ControlMain::robot_DY_RT(void *arg)
{
    ControlMain *pThis = static_cast<ControlMain*>(arg);
    pThis->period_time1= rt_timer_read();
    pThis->dy_sim_run = false;

    rt_task_set_periodic(&pThis->robot_dy_task, TM_NOW, 1e6);

    pThis->indx = 0;
    pThis->t_current = 0;
    pThis->row = pThis->size2[0];
    pThis->col = pThis->size2[1];

    pThis->dy_sim_run = true;
    while(pThis->indx < pThis->row){
        rt_task_wait_period(nullptr);
        pThis->period_time2 = rt_timer_read();

        for(uint j = 0; j < 6; j++){
            pThis->q[j] = pThis->recurdyn_data2[pThis->indx*pThis->col + j+2];
            pThis->q_dot[j] = pThis->recurdyn_data2[pThis->indx*pThis->col + j+14];
        }

        pThis->dy_time1 = rt_timer_read();
        pThis->robotArm->run_dynamics(pThis->q, pThis->q_dot, pThis->q_ddot, pThis->end_pose);

        pThis->robotArm->jacobian_zyx();
        for(uint i = 0; i < 6; i++)
        {
            temp = 0;
            for(uint j = 0; j < 6; j++)
            {
                temp += pThis->robotArm->J[i*6 + j]*pThis->q_dot[j];
            }
            pThis->end_vel[i] = temp;
        }
        pThis->dy_time2 = rt_timer_read();

        pThis->analysis_data_dy.push_back(pThis->t_current);
        for(uint j = 0; j < 6; j++){
            pThis->analysis_data_dy.push_back(pThis->q[j]);
        }
        for(uint j = 0; j < 6; j++){
            pThis->analysis_data_dy.push_back(pThis->end_pose[j]);
        }
        for(uint j = 0; j < 6; j++){
            pThis->analysis_data_dy.push_back(pThis->q_dot[j]);
        }
        for(uint j = 0; j < 6; j++){
            pThis->analysis_data_dy.push_back(pThis->end_vel[j]);
        }
        for(uint j = 0; j < 6; j++){
            pThis->analysis_data_dy.push_back(pThis->q_ddot[j]);
        }

        pThis->t_current += static_cast<double>((pThis->period_time2 - pThis->period_time1)/1000000.0);
        pThis->dy_time = static_cast<double>((pThis->dy_time2 - pThis->dy_time1)/1000000.0);
        pThis->analysis_data_dy.push_back(pThis->dy_time);
        pThis->indx++;

        pThis->period_time1 = pThis->period_time2;

        rt_printf("[DY]T_current : %5.5f\n", pThis->t_current);
    }

    printf("DY data write start\n");

    for(uint i = 0; i < pThis->row; i++){
        fprintf(pThis->fp_dy, "%d\t", i);
        for(uint j = 0; j < 32; j++){
            fprintf(pThis->fp_dy, "%3.7f\t", pThis->analysis_data_dy[i*32+j]);
        }
        fprintf(pThis->fp_dy, "\n");
    }

    printf("DY data write finished\n");
    printf("Finished DY simulation\n");

    pThis->dy_sim_run = false;
}

void ControlMain::run_kinematics()
{
//    printf("Start RobotController - Kinematics Simulation\n");

//    load_data("../data/recurdyn_data2.txt", &recurdyn_data, "\t", size);
//    row = size[0];
//    col = size[1];

//    fp = fopen("../data/analysis_data_kinematics.txt", "w+");

//    t_current = 0;
//    for(uint indx = 0; indx < size[0]; indx++){
//        for(uint j = 0; j < 6; j++){
//            q[j] = recurdyn_data[indx*col + j+2];
//            q_dot[j] = recurdyn_data[indx*col + j+14];
//        }

//        robotArm->run_kinematics(q, end_pose);

//        robotArm->jacobian_zyx();
//        for(uint i = 0; i < 6; i++)
//        {
//            temp = 0;
//            for(uint j = 0; j < 6; j++)
//            {
//                temp += robotArm->J[i*6 + j]*q_dot[j];
//            }
//            end_vel[i] = temp;
//        }

//        fprintf(fp, "%d\t%3.3f\t", indx, t_current);
//        for(uint j = 0; j < 6; j++){
//            fprintf(fp, "%3.7f\t", q[j]);
//        }
//        for(uint j = 0; j < 6; j++){
//            fprintf(fp, "%3.7f\t", end_pose[j]);
//        }
//        for(uint j = 0; j < 6; j++){
//            fprintf(fp, "%3.7f\t", q_dot[j]);
//        }
//        for(uint j = 0; j < 6; j++){
//            fprintf(fp, "%3.7f\t", end_vel[j]);
//        }
//        fprintf(fp, "\n");

//        t_current += step_size;
//    }
}

void ControlMain::run_inverse_kinematics()
{
//    printf("Start RobotController - Inverse Kinematics Simulation\n");

//    load_data("../data/recurdyn_data2.txt", &recurdyn_data, "\t", size);
//    row = size[0];
//    col = size[1];

//    fp = fopen("../data/analysis_data_inverse_kinematics.txt", "w+");

//    for(uint j = 0; j < 6; j++){
//        cur_q[j] = recurdyn_data[0*col + j+2];
//    }

//    robotArm->run_kinematics(cur_q, end_pose);

//    t_current = 0;
//    for(uint indx = 0; indx < size[0]; indx++){
//        for(uint j = 0; j < 6; j++){
//            des_pose[j] = recurdyn_data[indx*col + j+8];
//            end_vel[j] = recurdyn_data[indx*col + j+20];
//        }

//        for(uint j = 0; j < 6; j++){
//            cur_q[j] = robotArm->body[j+1].qi;
//        }

//        robotArm->run_inverse_kinematics(cur_q, des_pose, q, end_pose);

//        memset(indx_array, 0, sizeof(int)*6);
//        robotArm->numeric->ludcmp(robotArm->J, 6, indx_array, 0.0, fac_mat);
//        robotArm->numeric->lubksb(fac_mat, 6, indx_array, end_vel, q_dot);

//        fprintf(fp, "%d\t%3.3f\t", indx, t_current);
//        for(uint j = 0; j < 6; j++){
//            fprintf(fp, "%3.7f\t", q[j]);
//        }
//        for(uint j = 0; j < 6; j++){
//            fprintf(fp, "%3.7f\t", end_pose[j]);
//        }
//        for(uint j = 0; j < 6; j++){
//            fprintf(fp, "%3.7f\t", q_dot[j]);
//        }
//        for(uint j = 0; j < 6; j++){
//            fprintf(fp, "%3.7f\t", end_vel[j]);
//        }
//        fprintf(fp, "\n");

////        printf("t_current : %5.5f\n", t_current);
//        t_current += step_size;
//    }
}

void ControlMain::run_dynamics()
{
//    printf("Start RobotController - Dynamics Simulation\n");

//    load_data("../data/recurdyn_data3.txt", &recurdyn_data, "\t", size);
//    row = size[0];
//    col = size[1];

//    fp = fopen("../data/analysis_data_dynamics.txt", "w+");

//    t_current = 0;
//    for(uint indx = 0; indx < size[0]; indx++){
//        for(uint j = 0; j < 6; j++){
//            q[j] = recurdyn_data[indx*col + j+2];
//            q_dot[j] = recurdyn_data[indx*col + j+14];
//        }

//        robotArm->run_dynamics(q, q_dot, q_ddot, end_pose);

//        robotArm->jacobian_zyx();
//        for(uint i = 0; i < 6; i++)
//        {
//            temp = 0;
//            for(uint j = 0; j < 6; j++)
//            {
//                temp += robotArm->J[i*6 + j]*q_dot[j];
//            }
//            end_vel[i] = temp;
//        }

//        fprintf(fp, "%d\t%3.3f\t", indx, t_current);
//        for(uint j = 0; j < 6; j++){
//            fprintf(fp, "%3.7f\t", q[j]);
//        }
//        for(uint j = 0; j < 6; j++){
//            fprintf(fp, "%3.7f\t", end_pose[j]);
//        }
//        for(uint j = 0; j < 6; j++){
//            fprintf(fp, "%3.7f\t", q_dot[j]);
//        }
//        for(uint j = 0; j < 6; j++){
//            fprintf(fp, "%3.7f\t", end_vel[j]);
//        }
//        for(uint j = 0; j < 6; j++){
//            fprintf(fp, "%3.7f\t", q_ddot[j]);
//        }
//        fprintf(fp, "\n");

//        t_current += step_size;
//    }
}
