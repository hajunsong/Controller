#ifndef DATACONTROL_H
#define DATACONTROL_H

#include <string.h>
#include <stdio.h>
#include <iostream>
#include <vector>

const uint8_t SOCKET_TOKEN_SIZE = 2;

const uint8_t NUM_JOINT = 6;
const uint8_t NUM_DOF = 6;
const uint8_t MODULE_TYPE = 3; // 1: FAR v1, 2: FAR v2, 3: FAR v3
const uint8_t DATA_INDEX_LEN = 1;
const uint8_t MOTION_DATA_LEN = 8;
const uint8_t TIME_LEN = 8;

const uint8_t OP_MODE_LEN = 1;
const uint8_t SUB_MODE_LEN = 1;
const uint8_t SECTION_LEN = 1;
const uint8_t DESIRED_JOINT_LEN = 8;
const uint8_t DESIRED_CARTESIAN_LEN = 8;

const uint8_t CMD_TYPE_LEN = 1;
const uint8_t CYCLE_COUNT_LEN = 1;
const uint8_t ROW_SIZE_LEN = 1;
const uint8_t COL_SIZE_LEN = 1;
const uint8_t MOVE_TIME_LEN = 8;
const uint8_t ACC_TIME_LEN = 8;
const uint8_t PATH_DATA_LEN = 8;

const uint8_t MASS_LEN = 8;
const uint8_t TORQUE_CONST_LEN = 8;

const char KEY_Q = 113;
const char KEY_Z = 122;
const char KEY_X = 120;
const char KEY_C = 99;
const char KEY_A = 97;
const char KEY_S = 115;
const char KEY_D = 100;
const char KEY_ESC = 27;
const char KEY_SPC = 32;

typedef struct _StructClientToServer{
    char opMode, subMode;
    double desiredJoint[NUM_JOINT], desiredPose[NUM_DOF];
}StructClientToServer;

typedef struct _StructServerToClient{
    int8_t data_index;
    double presentJointPosition[NUM_JOINT], presentCartesianPose[NUM_DOF];
    double desiredJointPosition[NUM_JOINT], desiredCartesianPose[NUM_DOF];
    double calculateCartesianPose[NUM_DOF];
    double presentJointVelocity[NUM_JOINT], presentJointCurrent[NUM_JOINT];
    double presentCartesianVelocity[NUM_DOF];
    double time, dxl_time, ik_time;
    double presentJointTorque[NUM_JOINT], presentJointResidual[NUM_JOINT];
    unsigned long time1, time2, dxl_time1, dxl_time2, ik_time1, ik_time2;
}StructServerToClient;

typedef struct _StructRobotData{
    double t;
    unsigned long time1, time2, dxl_time1, dxl_time2, ik_time1, ik_time2;
    int32_t present_joint_position[NUM_JOINT];
    int32_t present_joint_velocity[NUM_JOINT];
    int16_t present_joint_current[NUM_JOINT];
    int8_t moving[NUM_JOINT];
    int8_t moving_status[NUM_JOINT];
    double present_end_pose[NUM_DOF];
    double desired_end_pose[NUM_DOF];
    bool ik_flag;
    double present_q[NUM_JOINT];
    double present_q_dot[NUM_JOINT];
    double desired_q[NUM_JOINT];
    int32_t command_joint_position[NUM_JOINT];
    int16_t command_joint_current[NUM_JOINT];
    int32_t offset[6];
    bool offset_setting;
    uint8_t joint_op_mode, module_indx;
    double present_end_vel[NUM_DOF];
    double Td[NUM_JOINT], Tg[NUM_JOINT], T[NUM_JOINT], T_limit[NUM_JOINT];
    bool T_err;
    double Kp, Dp, Kr, Dr;
    double F[6];
    bool module_init;
    double present_joint_torque[NUM_JOINT];
    double present_joint_residual[NUM_JOINT];
    double residual_limit_p[NUM_JOINT], residual_limit_n[NUM_JOINT];
    const int32_t initJoint_4 = -141;
    const int32_t initJoint_2 = 2933;
}StructRobotData;

typedef struct _StructKITECHData{
    char food_pixel[20];
    int food_pos[10];
    bool tablet_connect;
    bool camera_request, camera_request_old;
    double sp_food[3];
    double rp[3], sp[3];
    const double r_marker[3] = {-0.2, 0.155, 0};
    const double A_marker[9] = {1, 0, 0, 0, -1, 0, 0, 0, -1};
}StructKITECHData;

typedef struct _StructTrayInfor{
    int section1, section2, section3, section4, section5;
}StructTrayInfor;

class DataControl
{
public:
    DataControl();
    ~DataControl();

    enum OpMode{ServoOnOff = 0, Initialize, Wait, JointMove, CartesianMove, PathGenerateMode, ReadyMode, RunMode, TorqueID, OperateMode, ObiMode};
    enum Motion{JogMotion = 0, JointMotion, CartesianJogMotion, CartesianMotion};
    enum Module{FAR_V1=1, FAR_V2, FAR_V3};
    enum Comm{RS485=1, RS232, EtherCAT};
    enum CmdType{PathCmd=1, ReadyCmd, RunCmd, StopCmd};
    enum Operate{Start=1, Stop, StartTeaching, StopTeaching, StartFeeding, StopFeeding, Feeding, ReadyFeeding};
    enum Section{Side1=1, Side2, Side3, Soup, Rice, Mouse, Home};

    char key_value;
    bool config_check;
    bool cartesian_goal_reach;
    bool gpio_task_run, key_task_run, init_thread_run, robot_task_run;
    double step_size;
    int data_index;
    bool ui_mode;

    StructClientToServer ClientToServer;
    std::vector<StructServerToClient> ServerToClient;
    StructRobotData RobotData;
    StructKITECHData KITECHData;
    StructTrayInfor trayInfor;

    void jointPositionENC2DEG(int32_t pos_enc[], double pos_deg[]);
    void jointPositionENC2RAD(int32_t pos_enc[], double pos_rad[]);
    void cartesianPoseScaleUp(double pose_small[], double pose_big[]);
    void cartesianPoseScaleDown(double pose_big[], double pose_small[]);
    void jointPositionRAD2ENC(double pos_rad[], int32_t pos_enc[]);
    void jointPositionDEG2ENC(double pos_deg[], int32_t pos_end[]);
    void jointPositionDEG2ENC(const double pos_deg[], int32_t pos_end[]);
    void jointPositionDEG2ENC(const int32_t pos_deg[], int32_t pos_end[]);
    void jointPositionDEG2ENC(int32_t pos_deg[], int32_t pos_end[]);
    void jointVelocityENC2RPM(int32_t vel_enc[], double vel_rpm[]);
    void jointVelocityENC2RAD(int32_t vel_enc[], double vel_rad[]);
    void jointCurrentRAW2mA(int16_t cur_raw[], double cur_mA[]);
    void jointCurrentmA2RAW(double cur_mA[], int16_t cur_raw[]);
    void jointPositionRAD2DEG(double pos_rad[], double pos_deg[]);
    void jointCurrentRAW2Torque(int16_t cur_raw[], double cur_torque[]);

    const double ENC2DEG = 0.088;
    const double DEG2ENC = 11.363636364;
    const double DEG2RAD = 0.017453293; // 3.14159265358979323846/180.0;
    const double RAD2DEG = 57.295779513; // 180.0/3.14159265358979323846;
    const double ENC2RPM = 	0.229;
    const double RPM2DEG = 6;
    const double RAW2mA = 2.69;
    const double mA2RAW = 0.371747212;
};

#endif // DATACONTROL_H
