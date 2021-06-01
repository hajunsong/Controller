#ifndef DATACONTROL_H
#define DATACONTROL_H

#include <string.h>
#include <stdio.h>
#include <iostream>
#include <vector>

#include <native/timer.h>

#include "fileio.h"
#include "dynamixel.h"
#include "rapidjson/document.h"
#include "rapidjson/filereadstream.h"
using namespace rapidjson;

#define PRINT_ON 0

const uint8_t SOCKET_TOKEN_SIZE = 2;

const uint8_t NUM_JOINT = 6;
const uint8_t NUM_DOF = 6;
// const uint8_t MODULE_TYPE = 2; // 1: FAR v1, 2: FAR v2, 3: FAR v3
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

class DataControl
{
public:
    typedef struct _StructClientToServer{
        char opMode, subMode;
        double desiredJoint[NUM_JOINT], desiredPose[NUM_DOF];
        double move_time, acc_time;
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
    }StructServerToClient;

    typedef struct _StructRobotData{
        double t;
        int32_t present_joint_position[NUM_JOINT];
        int32_t present_joint_velocity[NUM_JOINT];
        int16_t present_joint_current[NUM_JOINT];
        int8_t moving[NUM_JOINT];
        int8_t moving_status[NUM_JOINT];
        double present_end_pose[NUM_DOF];
        double present_end_pose_zyx[NUM_DOF];
        double previous_end_pose[NUM_DOF];
        double desired_end_pose[NUM_DOF];
        double desired_end_pose_zyx[NUM_DOF];
        double old_desired_end_pose[NUM_DOF];
        bool ik_flag;
        double present_q[NUM_JOINT];
        double present_q_dot[NUM_JOINT];
        double desired_q[NUM_JOINT];
        int32_t command_joint_position[NUM_JOINT];
        int16_t command_joint_current[NUM_JOINT];
        unsigned long time1, time2, dxl_time1, dxl_time2, ik_time1, ik_time2;
        int32_t joint_offset[6];
        bool offset_setting;
        uint8_t joint_op_mode;
        uint8_t run_mode;
        double present_end_vel[NUM_DOF];
        double present_cal_end_pose[NUM_DOF];
        double Td[NUM_JOINT], Tg[NUM_JOINT], T[NUM_JOINT], T_limit[NUM_JOINT];
        bool T_err;
        double Kp, Dp, Kr, Dr;
        double F[6];
        double zeta;
        bool module_init;
        double present_joint_torque[NUM_JOINT];
        double present_joint_residual[NUM_JOINT];
        double residual_limit_p[NUM_JOINT], residual_limit_n[NUM_JOINT];
    }StructRobotData;

    typedef struct _StructPathGenerateData{
        std::vector<double> path_x, path_y, path_z, path_theta;
        double r[3], R_init[9];
        unsigned int data_size;
    }StructPathGenerateData;

    typedef struct _StructPathData{
        char cmd_type, cycle_count, cycle_count_max;
        uint8_t row, col;
        std::vector<double> total_time, point_theta, acc_time;
        std::vector<double> point_px, point_py, point_pz, point_rx, point_ry, point_rz;
        std::vector<double> point_px_home, point_py_home, point_pz_home, point_rx_home, point_ry_home, point_rz_home;
        double teaching_mat[9], teaching_pos[3], teaching_ori[3], teaching_fork_pose[6];
        int32_t fork_joint[6];
        double teaching_pose[NUM_DOF];
        StructPathGenerateData movePath[10], readyPath;
        uint path_data_indx;
        uint8_t path_struct_indx;
        std::vector<double> file_data;
        uint file_size[2];
        uint file_data_indx;
        double fork_offset[3];
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

    typedef struct _StructOperateMode{
        int mode;
        int section;
    }StructOperateMode;

    typedef struct _StructKITECHData{
        int interface_cmd, interface_sub;
        char food_pixel[20];
        int food_pos[10];
        bool tablet_connect, tablet_check, tablet_check_old;
        bool camera_request, camera_request_old;
        double sp_food[3];
        double rp[3], sp[3];
        const double r_marker[3] = {-0.2, 0.155, 0};
        const double A_marker[9] = {1, 0, 0, 0, -1, 0, 0, 0, -1};
    }StructKITECHData;

    typedef struct _StructTrayInfor{
        int section1, section2, section3, section4, section5;
    }StructTrayInfor;

    enum OpMode{ServoOnOff = 0, Initialize, Wait, JointMove, CartesianMove, PathGenerateMode, ReadyMode, RunMode, TorqueID, OperateMode, ObiMode};
    enum Motion{JogMotion = 0, JointMotion, CartesianJogMotion, CartesianMotion};
    enum Module{FAR_V1=1, FAR_V2, FAR_V3};
    enum Comm{RS485=1, RS232, EtherCAT};
    enum CmdType{PathCmd=1, ReadyCmd, RunCmd, StopCmd, FileReady, FileRun, CustomRun};
    enum Operate{Start=1, Stop, StartTeaching, StopTeaching, StartFeeding, StopFeeding, Feeding, ReadyFeeding, ReadyFeeding2};
    enum Section{Side1=1, Side2, Side3, Soup, Rice, Mouse, Home};

    bool config_check;
    bool cartesian_goal_reach;
    bool joint_goal_reach;
    bool feeding;
    bool tablet_mode;
    char section_indx;
    char section_indx2;

    DataControl();
    ~DataControl();
    void DataReset();

    StructClientToServer ClientToServer;
    std::vector<StructServerToClient> ServerToClient;
    StructRobotData RobotData;
    StructPathData PathData;
    StructPathData side1_motion[3], side2_motion[3], side3_motion[3], rice_motion[3], soup_motion;
    StructTorqueIDEData torqueIdeData;
    StructControllerPID PIDControl;
    StructOperateMode operateMode;
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

    const double initJoint2Deg = 30;
    const double initJoint4Deg = -200;

//    const double operateCameraReadyJoint[6] = {1.173420, -0.086010, -0.637394, -0.824773, -1.154989, -1.549713};
    const double operateFeedingReadyPose[6] = {-0.222, 0.058758770483143,  0.0989879236551952, -1.57080736369899,  1.48352986419518, 1.5708665530537};

//    const int32_t initJoint_4 = -141;
//    const int32_t initJoint_2 = 2933;

    const int32_t finishJoint[6] = {2399, 2437, 965, 354, 3405, 1010};

    int32_t joint_offset[6];
    double tool_offset[3];
    double operateCameraReadyJoint[6];
    uint8_t MODULE_TYPE;
};

#endif // DATACONTROL_H
