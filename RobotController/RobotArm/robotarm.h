#pragma once

#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <memory.h>

//#include "FileIO/fileio.h"
#include "Numerical/numerical.h"

using namespace std;

class RobotArm
{
public:
    RobotArm(uint numbody, uint DOF, double step_size);
    ~RobotArm();
#ifdef FILEIO_H_
    void run_kinematics();
    void run_inverse_kinematics();
    void run_dynamics();
#endif
    void run_kinematics(double *q, double *des_pose);
    void run_inverse_kinematics(double* cur_joint, double* des_pose, double* res_joint, double* res_pose);
    void gravity_compensation(double *q, double *q_dot, double *torque);

private:
    inline void tilde(double *a, double *b) {
        *(b++) = 0;	*(b++) = -a[2];	*(b++) = a[1];
        *(b++) = a[2];	*(b++) = 0;	*(b++) = -a[0];
        *(b++) = -a[1];	*(b++) = a[0];	*(b++) = 0;
    }

    class Body
    {
    public:
        Body();
        Body(double psi, double theta, double phi, double sijp_x, double sijp_y, double sijp_z);
        ~Body();

        double qi, qi_dot, qi_ddot;
        double mi, Ixx, Iyy, Izz, Ixy, Iyz, Izx;

        // orientation
        double Ai[9], Aijpp[9], Ai_Cij[9], Cij[9], u_vec[3];
        // position
        double sij[3], sijp[3], ri[3], re[3], Ae[9], roll, pitch, yaw;
        // jacobian
        double Jvi[3], Jwi[3], re_qi[3], Ae_qi[9], r6_qi[3], A6_qi[9], Aijpp_qi[9], Cij_Aijpp[9], Ai_Cij_Aijpp_qi[9];
        double Ae_qi_31, Ae_qi_32, Ae_qi_33, Ae_qi_21, Ae_qi_11, roll_qi, pitch_qi, yaw_qi;
        // velocity state
        double Hi[3], rit[9], Bi[6], Yih[6];
        // cartesian velocity
        double Ti[36], Yib[6], ri_dot[3], wi[3], wit[9], rhoip[3], rhoi[3], ric[3], ric_dot[3];
        // mass & force
        double Cii[9], Ai_Cii[9], Jic[9], Jip[9], rit_dot[9], rict_dot[9], rict[9], Mih[36], fic[3], tic[3], Qih[6], Qih_g[6], Qih_c[6];
        // velocity coupling
        double Hi_dot[3], Di[6], Di_sum[6];
        // system EQM
        double Ki[36], Li[6], Li_g[6], Li_c[6], Ki_Di[6], Ki_Di_sum[6];

        static void ang2mat(double ang_z1, double ang_x, double ang_z2, double* mat, bool deg_flag = true);
    };

    double DH[6*4];

    uint num_body, dof;
    double *PH, *PH_pos, *PH_ori, *delta_q, *J, *JD;
    double *M, *Q, *Q_c, *Q_g;

    // system variable
    double start_time, end_time, h, t_current;
    double g;

    // file
    char file_name[256];
    FILE *fp;

    Body *body;
    Numerical *numeric;

    double lamda;

    void kinematics();
    void inverse_kinematics(double pos_d[3], double ori_d[3]=nullptr);
        void jacobian();
    void dynamics();
    void save_data();

    void mat(double *mat_1, double *mat_2, uint row_1, uint col_1, uint row_2, uint col_2, double *mat_3);
    void mat(double *mat_1, double *vec_2, uint row_1, uint col_1, uint row_2, double *vec_3);

    double Ae_31, Ae_32, Ae_33, Ae_21, Ae_11;
    double roll_q_temp1, roll_q_temp2, roll_q_temp3, roll_q_temp4;
    double pitch_q_temp1, pitch_q_temp2, pitch_q_temp3, pitch_q_temp4;
    double yaw_q_temp1, yaw_q_temp2, yaw_q_temp3, yaw_q_temp4;
};

// Euler Angle to Transformation Matrix(body 3-1-3)
void Euler2Trans(double psi, double theta, double phi, double* Mat);

