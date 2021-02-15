#ifndef DATACONTROL_H
#define DATACONTROL_H

#include <string.h>
#include <stdio.h>
#include <iostream>
#include <vector>

#include <native/timer.h>

#include "FileIO/fileio.h"
#include "Dynamixel/dynamixel.h"

#include <QtCore/qglobal.h>

#if defined(DATACONTROLLIB_LIBRARY)
#  define DATACONTROLLIB_EXPORT Q_DECL_EXPORT
#else
#  define DATACONTROLLIB_EXPORT Q_DECL_IMPORT
#endif

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
        int32_t offset[6];
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
        std::vector<double> total_time, /*point_x, point_y, point_z,*/ point_theta, acc_time/*, point_roll, point_pitch, point_yaw*/;
        std::vector<double> point_px, point_py, point_pz, point_rx, point_ry, point_rz;
//        std::vector<double> point_px_fork, point_py_fork, point_pz_fork, point_rx_fork, point_ry_fork, point_rz_fork;
        std::vector<double> point_px_home, point_py_home, point_pz_home, point_rx_home, point_ry_home, point_rz_home;
        double teaching_mat[9], teaching_pos[3], teaching_ori[3], teaching_fork_pose[6];
        int32_t fork_joint[6];
//        double teaching_x, teaching_y, teaching_z, teaching_roll, teaching_pitch, teaching_yaw;
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
    enum Section{Side1=1, Side2, Side3, Soup, Rice, Rice1, Rice2, Rice3, Rice4, Rice5, Rice6, Rice7, Rice8, Rice9, Mouse, Home};

    bool config_check;
    bool cartesian_goal_reach;
    bool joint_goal_reach;
    bool feeding;
    bool obi_mode;
    bool demo_mode;
    unsigned char obi_section_indx;
    char obi_section_indx2;

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

    const double operateReadyJoint[6] = {0.732662200000000, 0.172694300000000, -1.66388430000000, -1.49117350000000, 0.838063900000000, -3.14159265358979/2.0};
    const double operateReadyJoint2[6] = {0.732662200000000, 0.172694300000000, -1.66388430000000, -1.49117350000000, 0.838063900000000, 3.14159265358979/2.0};
    const double operateCameraReadyJoint[6] = {1.173420, -0.086010, -0.637394, -0.824773, -1.154989, -1.549713};
//    const double operateCameraReadyJoint[6] = {0.732662200000000, 0.172694300000000, -1.66388430000000, -1.49117350000000, 0.838063900000000, -3.14159265358979/2.0};
    const double operateForkReadyJoint[6] = {0.732662200000000, 0.172694300000000, -1.66388430000000, -1.49117350000000, 0.838063900000000, -0};
    const double operateReadyPose[6] = {-0.222101784479607, 0.153502631527094, 0.00898782365655648, -1.57080736369898, 1.22652659337934e-05, 1.57072609993231};
    const double operateFeedingReadyPose[6] = {-0.222, 0.058758770483143,  0.0989879236551952, -1.57080736369899,  1.48352986419518, 1.5708665530537};
    const double operateFeedingForkPose[6] = {-0.222105097058184, 0.153506313301957, 0.309147823615697, 1.57080736369898, -1.22652659305623e-05, -1.57086655365748};

    const int32_t initJoint_4 = 2400;
    const int32_t initJoint_2 = 4382;

    const int32_t finishJoint[6] = {2399, 2437, 965, 354, 3405, 1010};

    const double operateRicePoints[5*6] = {
        -0.275, 0.142816624803954, -0.0307361611466054, -1.57080736369899, -0.2617993877991, 1.5708665530537,
        -0.275, 0.054700916162332, -0.036             , -1.57080736369899,  1.0471975511966, 1.5708665530537,
        -0.275, 0.041197425412881, -0.0142898992833717, -1.57080736369899,  1.0471975511966, 1.5708665530537,
        -0.275, 0.052868186579499, -0.0011427286972479, -1.57080736369899,  1.0471975511966, 1.5708665530537,
        -0.222, 0.098758770483143,  0.0389879236551952, -1.57080736369899,  1.48352986419518, 1.5708665530537
    };

    const double operateRicePoints7[7*6] = {
        -0.205112, 0.105523,  0.001374, 1.570796,  0.000000, -1.570796,
        -0.335, 0.127323, -0.051363, 1.570796,  0.698131, -1.570796,
        -0.335, 0.127323, -0.068270, 1.570796,  0.698131, -1.570796,
        -0.335, 0.063537, -0.075270, 1.570796,  0.698131, -1.570796,
        -0.335, 0.127323, -0.068270, 1.570796,  0.698131, -1.570796,
        -0.335, 0.127323, -0.051363, 1.570796,  0.698131, -1.570796,
        -0.205112, 0.105523,  0.001374, 1.570796,  0.000000, -1.570796,
    };
    const double operateRicePoints8[7*6] = {
        -0.205112, 0.105523,  0.001374, 1.570796,  0.000000, -1.570796,
        -0.3, 0.127323, -0.051363, 1.570796,  0.698131, -1.570796,
        -0.3, 0.127323, -0.068270, 1.570796,  0.698131, -1.570796,
        -0.3, 0.063537, -0.075270, 1.570796,  0.698131, -1.570796,
        -0.3, 0.127323, -0.068270, 1.570796,  0.698131, -1.570796,
        -0.3, 0.127323, -0.051363, 1.570796,  0.698131, -1.570796,
        -0.205112, 0.105523,  0.001374, 1.570796,  0.000000, -1.570796,
    };
    const double operateRicePoints9[7*6] = {
        -0.205112, 0.105523,  0.001374, 1.570796,  0.000000, -1.570796,
        -0.25, 0.127323, -0.051363, 1.570796,  0.698131, -1.570796,
        -0.25, 0.127323, -0.072270, 1.570796,  0.698131, -1.570796,
        -0.25, 0.063537, -0.078270, 1.570796,  0.698131, -1.570796,
        -0.25, 0.127323, -0.072270, 1.570796,  0.698131, -1.570796,
        -0.25, 0.127323, -0.051363, 1.570796,  0.698131, -1.570796,
        -0.205112, 0.105523,  0.001374, 1.570796,  0.000000, -1.570796,
    };
    const double operateRicePoints4[7*6] = {
        -0.205112, 0.105523,  0.001374, 1.570796,  0.000000, -1.570796,
        -0.335, 0.083537, -0.051363, 1.570796,  0.698131, -1.570796,
        -0.335, 0.083537, -0.085270, 1.570796,  0.698131, -1.570796,
        -0.335, 0.010167, -0.085270, 1.570796, -0.782312, -1.570796,
        -0.335, 0.018167, -0.080270, 1.570796, -0.782312, -1.570796,
        -0.335, 0.019167,  0.011729, 1.570796, -0.782312, -1.570796,
        -0.205422, 0.026833,  0.058549, 1.564223, -0.782312, -1.566155
    };
    const double operateRicePoints5[7*6] = {
        -0.205112, 0.105523,  0.001374, 1.570796,  0.000000, -1.570796,
        -0.3, 0.083537, -0.051363, 1.570796,  0.698131, -1.570796,
        -0.3, 0.083537, -0.078270, 1.570796,  0.698131, -1.570796,
        -0.3, 0.010167, -0.078270, 1.570796, -0.782312, -1.570796,
        -0.3, 0.018167, -0.080270, 1.570796, -0.782312, -1.570796,
        -0.3, 0.019167,  0.011729, 1.570796, -0.782312, -1.570796,
        -0.205422, 0.026833,  0.058549, 1.564223, -0.782312, -1.566155
    };
    const double operateRicePoints6[7*6] = {
        -0.205112, 0.105523,  0.001374, 1.570796,  0.000000, -1.570796,
        -0.25, 0.083537, -0.051363, 1.570796,  0.698131, -1.570796,
        -0.25, 0.083537, -0.078270, 1.570796,  0.698131, -1.570796,
        -0.25, 0.010167, -0.078270, 1.570796, -0.782312, -1.570796,
        -0.25, 0.018167, -0.080270, 1.570796, -0.782312, -1.570796,
        -0.25, 0.019167,  0.011729, 1.570796, -0.782312, -1.570796,
        -0.205422, 0.026833,  0.058549, 1.564223, -0.782312, -1.566155
    };
    const double operateRicePoints1[7*6] = {
        -0.205112, 0.105523,  0.001374, 1.570796,  0.000000, -1.570796,
        -0.335, 0.053643, -0.051363, 1.570796,  0.698131, -1.570796,
        -0.335, 0.053643, -0.085270, 1.570796,  0.698131, -1.570796,
        -0.335, 0.010167, -0.085270, 1.570796, -0.782312, -1.570796,
        -0.335, 0.018167, -0.080270, 1.570796, -0.782312, -1.570796,
        -0.335, 0.019167,  0.011729, 1.570796, -0.782312, -1.570796,
        -0.205422, 0.026833,  0.058549, 1.564223, -0.782312, -1.566155
    };
    const double operateRicePoints2[7*6] = {
        -0.205112, 0.105523,  0.001374, 1.570796,  0.000000, -1.570796,
        -0.3, 0.053643, -0.051363, 1.570796,  0.698131, -1.570796,
        -0.3, 0.053643, -0.085270, 1.570796,  0.698131, -1.570796,
        -0.3, 0.010167, -0.085270, 1.570796, -0.782312, -1.570796,
        -0.3, 0.018167, -0.080270, 1.570796, -0.782312, -1.570796,
        -0.3, 0.019167,  0.011729, 1.570796, -0.782312, -1.570796,
        -0.205422, 0.026833,  0.058549, 1.564223, -0.782312, -1.566155
    };
    const double operateRicePoints3[7*6] = {
        -0.205112, 0.105523,  0.001374, 1.570796,  0.000000, -1.570796,
        -0.25, 0.053643, -0.051363, 1.570796,  0.698131, -1.570796,
        -0.25, 0.053643, -0.085270, 1.570796,  0.698131, -1.570796,
        -0.25, 0.010167, -0.085270, 1.570796, -0.782312, -1.570796,
        -0.25, 0.018167, -0.080270, 1.570796, -0.782312, -1.570796,
        -0.25, 0.019167,  0.011729, 1.570796, -0.782312, -1.570796,
        -0.205422, 0.026833,  0.058549, 1.564223, -0.782312, -1.566155
    };

//    const int32_t offset[6] = {2047, 2047, 2026, 2044, 2031, 3048}; // 06 robot
//    const int32_t offset[6] = {2060, 2042, 2071, 2073, 2046, 2055}; // 05 robot
//    const int32_t offset[6] = {1031, 3089, 4072, 4071, 3582, 4086}; // V2
//    const int32_t offset[6] = {2047, 2047, 2026, 2044, 2031, 3048}; // V3
//    const int32_t offset[6] = {2769, 4109, 1541, 3600, 488, 500}; // demo
//    const int32_t offset[6] = {762, 4099, 1527, 3595, 3097, 1080}; // demo
//    const int32_t offset_err[6] = {762, 5, 1527, 3595, 3097, 1080}; // demo

    const int32_t offset[6] = {796, 4119, 1544, 3600, 3085, 1043};

//    const int32_t offset[6] = {2311, 2619, 2550, 1035, 3583, 2030};

//    const int32_t offset[6] = {2311, 2619, 2554, 1021, 3583, 2030};

//    const double obi_ready_joint[5*6] = {
//        0.8057250, -0.42357400, -0.8639060, -1.2874644, 0.7650010, 0.00060474689,
//        0.5149660, -0.02152700, -1.3536400, -1.3751454, 1.0557600, 0.00060474689,
//        0.0876270,  0.15779000, -1.5208940, -1.3629784, 1.4830990, 0.00060474689,
//        0.4165570,  0.82357700, -2.2831660, -1.4595624, 1.1543060, -3.4027983,
//        1.0722570,  0.08851600, -1.8553710, -1.7668434, 0.4986060, -3.4027803
//    };
    double obi_ready_joint[5*6];

    const double obi_ready_pose[5*6] = {
        -0.307112, 0.212752, 0.249374, -1.570796, -5.358979E-08, 1.570796
    };

    const double obi_ready_pose_rice[9*6] = {
        -0.335, 0.127323, -0.051363, 1.570796,  0.000000, -1.570796,
        -0.3,   0.127323, -0.051363, 1.570796,  0.000000, -1.570796,
        -0.25,  0.127323, -0.051363, 1.570796,  0.000000, -1.570796,
        -0.335, 0.083537, -0.051363, 1.570796,  0.000000, -1.570796,
        -0.3,   0.083537, -0.051363, 1.570796,  0.000000, -1.570796,
        -0.25,  0.083537, -0.051363, 1.570796,  0.000000, -1.570796,
        -0.335, 0.053643, -0.051363, 1.570796,  0.000000, -1.570796,
        -0.3,   0.053643, -0.051363, 1.570796,  0.000000, -1.570796,
        -0.25,  0.053643, -0.051363, 1.570796,  0.000000, -1.570796
    };

    const double obi_feeding_pose_rice[9*6*6] = {
        -0.335, 0.127323, -0.051363, 1.570796,  0.698131, -1.570796,
        -0.335, 0.127323, -0.068270, 1.570796,  0.698131, -1.570796,
        -0.335, 0.063537, -0.075270, 1.570796,  0.698131, -1.570796,
        -0.335, 0.127323, -0.068270, 1.570796,  0.698131, -1.570796,
        -0.335, 0.127323, -0.051363, 1.570796,  0.698131, -1.570796,
        -0.205112, 0.105523,  0.001374, 1.570796,  0.000000, -1.570796,

        -0.3, 0.127323, -0.051363, 1.570796,  0.698131, -1.570796,
        -0.3, 0.127323, -0.068270, 1.570796,  0.698131, -1.570796,
        -0.3, 0.063537, -0.075270, 1.570796,  0.698131, -1.570796,
        -0.3, 0.127323, -0.068270, 1.570796,  0.698131, -1.570796,
        -0.3, 0.127323, -0.051363, 1.570796,  0.698131, -1.570796,
        -0.205112, 0.105523,  0.001374, 1.570796,  0.000000, -1.570796,

        -0.25, 0.127323, -0.051363, 1.570796,  0.698131, -1.570796,
        -0.25, 0.127323, -0.072270, 1.570796,  0.698131, -1.570796,
        -0.25, 0.063537, -0.078270, 1.570796,  0.698131, -1.570796,
        -0.25, 0.127323, -0.072270, 1.570796,  0.698131, -1.570796,
        -0.25, 0.127323, -0.051363, 1.570796,  0.698131, -1.570796,
        -0.205112, 0.105523,  0.001374, 1.570796,  0.000000, -1.570796,

        -0.335, 0.083537, -0.051363, 1.570796,  0.698131, -1.570796,
        -0.335, 0.083537, -0.085270, 1.570796,  0.698131, -1.570796,
        -0.335, 0.010167, -0.085270, 1.570796, -0.782312, -1.570796,
        -0.335, 0.018167, -0.080270, 1.570796, -0.782312, -1.570796,
        -0.335, 0.019167,  0.011729, 1.570796, -0.782312, -1.570796,
        -0.205422, 0.026833,  0.058549, 1.564223, -0.782312, -1.566155

        -0.3, 0.083537, -0.051363, 1.570796,  0.698131, -1.570796,
        -0.3, 0.083537, -0.078270, 1.570796,  0.698131, -1.570796,
        -0.3, 0.010167, -0.078270, 1.570796, -0.782312, -1.570796,
        -0.3, 0.018167, -0.080270, 1.570796, -0.782312, -1.570796,
        -0.3, 0.019167,  0.011729, 1.570796, -0.782312, -1.570796,
        -0.205422, 0.026833,  0.058549, 1.564223, -0.782312, -1.566155

        -0.25, 0.083537, -0.051363, 1.570796,  0.698131, -1.570796,
        -0.25, 0.083537, -0.078270, 1.570796,  0.698131, -1.570796,
        -0.25, 0.010167, -0.078270, 1.570796, -0.782312, -1.570796,
        -0.25, 0.018167, -0.080270, 1.570796, -0.782312, -1.570796,
        -0.25, 0.019167,  0.011729, 1.570796, -0.782312, -1.570796,
        -0.205422, 0.026833,  0.058549, 1.564223, -0.782312, -1.566155

        -0.335, 0.053643, -0.051363, 1.570796,  0.698131, -1.570796,
        -0.335, 0.053643, -0.085270, 1.570796,  0.698131, -1.570796,
        -0.335, 0.010167, -0.085270, 1.570796, -0.782312, -1.570796,
        -0.335, 0.018167, -0.080270, 1.570796, -0.782312, -1.570796,
        -0.335, 0.019167,  0.011729, 1.570796, -0.782312, -1.570796,
        -0.205422, 0.026833,  0.058549, 1.564223, -0.782312, -1.566155

        -0.3, 0.053643, -0.051363, 1.570796,  0.698131, -1.570796,
        -0.3, 0.053643, -0.085270, 1.570796,  0.698131, -1.570796,
        -0.3, 0.010167, -0.085270, 1.570796, -0.782312, -1.570796,
        -0.3, 0.018167, -0.080270, 1.570796, -0.782312, -1.570796,
        -0.3, 0.019167,  0.011729, 1.570796, -0.782312, -1.570796,
        -0.205422, 0.026833,  0.058549, 1.564223, -0.782312, -1.566155

        -0.25, 0.053643, -0.051363, 1.570796,  0.698131, -1.570796,
        -0.25, 0.053643, -0.085270, 1.570796,  0.698131, -1.570796,
        -0.25, 0.010167, -0.085270, 1.570796, -0.782312, -1.570796,
        -0.25, 0.018167, -0.080270, 1.570796, -0.782312, -1.570796,
        -0.25, 0.019167,  0.011729, 1.570796, -0.782312, -1.570796,
        -0.205422, 0.026833,  0.058549, 1.564223, -0.782312, -1.566155
    };

    const double section1_area[4] = {14.5313 ,  75.5417, 92.8125 ,  11.0833};
    const double section2_area[4] = {117.188 ,  71.4583, 184.922 ,  12.25};
    const double section3_area[4] = {210.469 ,  67.9583, 277.5 ,  16.625};
    const double section4_area[4] = {166.641 ,  -13.125, 276.563 ,  -105.292};
    const double section5_area[4] = {24.1406 ,  -19.25, 130.547 ,  -102.375};
};

#endif // DATACONTROL_H
