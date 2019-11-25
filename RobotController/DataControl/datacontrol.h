#pragma once

#include <string.h>
#include <stdio.h>
#include <iostream>
#include <vector>

#include <NRMKSocketBase.h>
#include <native/timer.h>

#include "FileIO/fileio.h"

#define NUM_JOINT               6
#define NUM_DOF                 6
#define MODULE_TYPE             1 // 1:FAR V1, 2:SEA
#define DATA_INDEX_LEN          1
#define JOINT_POSITION_LEN      8
#define CARTESIAN_POSE_LEN      8
#define JOINT_COMMAND_LEN       8
#define CARTESIAN_COMMAND_LEN   8
#define CARTESIAN_CALCULATE_LEN 8
#define JOINT_VELOCITY_LEN      8
#define JOINT_CURRENT_LEN       8
#define TIME_LEN                8
#define SERVER_TO_CLIENT_LEN    NRMK_SOCKET_TOKEN_SIZE + DATA_INDEX_LEN + \
    JOINT_POSITION_LEN*NUM_JOINT + CARTESIAN_POSE_LEN*NUM_DOF + JOINT_COMMAND_LEN*NUM_JOINT + \
    CARTESIAN_COMMAND_LEN*NUM_DOF + CARTESIAN_CALCULATE_LEN*NUM_DOF + JOINT_VELOCITY_LEN*NUM_JOINT + JOINT_CURRENT_LEN*NUM_JOINT + \
    TIME_LEN + TIME_LEN + TIME_LEN + NRMK_SOCKET_TOKEN_SIZE

#define OP_MODE_LEN             1
#define SUB_MODE_LEN            1
#define DESIRED_JOINT_LEN       8
#define DESIRED_CARTESIAN_LEN   8
#define CLIENT_TO_SERVER_LEM    NRMK_SOCKET_TOKEN_SIZE + OP_MODE_LEN + SUB_MODE_LEN + DESIRED_JOINT_LEN*NUM_JOINT + DESIRED_CARTESIAN_LEN*NUM_DOF + NRMK_SOCKET_TOKEN_SIZE

#define CMD_TYPE_LEN            1
#define CYCLE_COUNT_LEN         1
#define ROW_SIZE_LEN            1
#define COL_SIZE_LEN            1
#define PATH_DATA_LEN           8
#define MASS_LEN                8
#define TORQUE_CONST_LEN        8

class DataControl{
public:
    typedef struct _StructClientToServer{
        char opMode, subMode;
        double desiredJoint[NUM_JOINT], desiredPose[NUM_DOF];
        double move_time, acc_time;
    }StructClientToServer;

    typedef struct _StructServerToClient{
        unsigned char data_index;
        double presentJointPosition[NUM_JOINT], presentCartesianPose[NUM_DOF];
        double desiredJointPosition[NUM_JOINT], desiredCartesianPose[NUM_DOF];
        double calculateCartesianPose[NUM_DOF];
        double presentJointVelocity[NUM_JOINT], presentJointCurrent[NUM_JOINT];
        double time, dxl_time, ik_time;
    }StructServerToClient;

    typedef struct _StructRobotData{
        int32_t present_joint_position[NUM_JOINT];
        int32_t present_joint_velocity[NUM_JOINT];
        int16_t present_joint_current[NUM_JOINT];
        double present_end_pose[NUM_DOF];
        double desired_end_pose[NUM_DOF];
        double old_desired_end_pose[NUM_DOF];
        double present_q[NUM_JOINT];
        double present_q_dot[NUM_JOINT];
        double desired_q[NUM_JOINT];
        int32_t command_joint_position[NUM_JOINT];
        int16_t command_joint_current[NUM_JOINT];
        unsigned long time1, time2, dxl_time1, dxl_time2, ik_time1, ik_time2;
        int32_t offset[6];
        uint8_t joint_op_mode;
        uint8_t run_mode;
    }StructRobotData;

    typedef struct _StructPathGenerateData{
        std::vector<double> path_x, path_y, path_z, path_theta;
        double r[3], R_init[9];
        unsigned int data_size;
    }StructPathGenerateData;

    typedef struct _StructPathData{
        char cmd_type, cycle_count, cycle_count_max;
        uint8_t row, col;
        std::vector<double> total_time, point_x, point_y, point_z, point_theta, acc_time, point_roll, point_pitch, point_yaw;
        StructPathGenerateData movePath[10], readyPath;
        uint path_data_indx;
        uint8_t path_struct_indx;
        std::vector<double> file_data;
    }StructPathData;

    typedef struct _StructTorqueIDEData{
        double mass;
        double torque_constant;
    }StructTorqueIDEData;

    typedef struct _StructControllerPID{
        double Kp, Kd, Ki;
        double err, err_prev, err_accum;
        double des;
        double h;
    }StructControllerPID;

    enum OpMode{ServoOnOff = 0, Initialize, Wait, JointMove, CartesianMove, PathGenerateMode, ReadyMode, RunMode, TorqueIDE};
    enum Motion{JogMotion = 0, JointMotion, CartesianJogMotion, CartesianMotion};
    enum Module{FAR_V1=1, FAR_V2, SEA};
    enum Comm{RS485=1, RS232, EtherCAT};
    enum CmdType{PathCmd=1, ReadyCmd, RunCmd, StopCmd, FileReady, FileRun, CustomRun};

    bool config_check;
    bool cartesian_goal_reach;
    bool joint_goal_reach;

    DataControl();
    ~DataControl();
    void DataReset();

    StructClientToServer ClientToServer;
    StructServerToClient ServerToClient;
    StructRobotData RobotData;
    StructPathData PathData;
    StructTorqueIDEData torqueIdeData;
    StructControllerPID PIDControl;

    void jointPositionENC2DEG(int32_t pos_enc[], double pos_deg[]);
    void jointPositionENC2RAD(int32_t pos_enc[], double pos_rad[]);
    void cartesianPoseScaleUp(double pose_small[], double pose_big[]);
    void cartesianPoseScaleDown(double pose_big[], double pose_small[]);
    void jointPositionRAD2ENC(double pos_rad[], int32_t pos_enc[]);
    void jointPositionDEG2ENC(double pos_deg[], int32_t pos_end[]);
    void jointPositionDEG2ENC(const int32_t pos_deg[], int32_t pos_end[]);
    void jointPositionDEG2ENC(int32_t pos_deg[], int32_t pos_end[]);
    void jointVelocityENC2RPM(int32_t vel_enc[], double vel_rpm[]);
    void jointVelocityENC2RAD(int32_t vel_enc[], double vel_rad[]);
    void jointCurrentRAW2mA(int16_t cur_raw[], double cur_mA[]);
    void jointCurrentmA2RAW(double cur_mA[], int16_t cur_raw[]);

    const double ENC2DEG = 0.088;
    const double DEG2ENC = 11.363636364;
    const double DEG2RAD = 0.017453293;//3.14159265358979323846/180.0;
    const double RAD2DEG = 57.295779513;//180.0/3.14159265358979323846;
    const double ENC2RPM = 	0.229;
    const double RPM2DEG = 6;
    const double RAW2mA = 2.69;
    const double mA2RAW = 0.371747212;

//    const int32_t offset[6] = {2202, 500, 1672, 3200, 901, 1924};
};
