#ifndef DATACONTROL_H
#define DATACONTROL_H

#include <string.h>
#include <stdio.h>
#include <iostream>
#include <vector>

#include <native/timer.h>

#include "FileIO/fileio.h"

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
//        double teaching_x, teaching_y, teaching_z, teaching_roll, teaching_pitch, teaching_yaw;
        double teaching_pose[NUM_DOF];
        StructPathGenerateData movePath[10], readyPath;
        uint path_data_indx;
        uint8_t path_struct_indx;
        std::vector<double> file_data;
//        double teaching_pose[NUM_JOINT];
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

    enum OpMode{ServoOnOff = 0, Initialize, Wait, JointMove, CartesianMove, PathGenerateMode, ReadyMode, RunMode, TorqueID, OperateMode, ObiMode};
    enum Motion{JogMotion = 0, JointMotion, CartesianJogMotion, CartesianMotion};
    enum Module{FAR_V1=1, FAR_V2, FAR_V3};
    enum Comm{RS485=1, RS232, EtherCAT};
    enum CmdType{PathCmd=1, ReadyCmd, RunCmd, StopCmd, FileReady, FileRun, CustomRun};
    enum Operate{Start=1, Stop, StartTeaching, StopTeaching, StartFeeding, StopFeeding, Feeding, ReadyFeeding};
    enum Section{Side1=1, Side2, Side3, Soup, Rice, Rice1, Rice2, Rice3, Rice4, Rice5, Rice6, Rice7, Rice8, Rice9, Mouse, Home};

    bool config_check;
    bool cartesian_goal_reach;
    bool joint_goal_reach;
    bool feeding;
    bool obi_mode;
    unsigned char obi_section_indx;

    DataControl();
    ~DataControl();
    void DataReset();

    StructClientToServer ClientToServer;
    std::vector<StructServerToClient> ServerToClient;
    StructRobotData RobotData;
    StructPathData PathData, side1_motion, side2_motion, side3_motion, rice_motion, soup_motion;
    StructTorqueIDEData torqueIdeData;
    StructControllerPID PIDControl;
    StructOperateMode operateMode;

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

    const double ENC2DEG = 0.088;
    const double DEG2ENC = 11.363636364;
    const double DEG2RAD = 0.017453293; // 3.14159265358979323846/180.0;
    const double RAD2DEG = 57.295779513; // 180.0/3.14159265358979323846;
    const double ENC2RPM = 	0.229;
    const double RPM2DEG = 6;
    const double RAW2mA = 2.69;
    const double mA2RAW = 0.371747212;

    const double operateReadyJoint[6] = {0.74944438, 0.40421825, -2.0655972, -1.6613789, 0.82135195, 0};
    const double operateReadyPose[6] = {-0.205112, 0.105523, 0.001374, 1.570796, -7.320674e-08, -1.570796};
    const double operateFeedingReadyPose[6] = {-0.205422, 0.026833,  0.058549, 1.564223, -0.782312, -1.566155};

//    const double operateSection1Point1[6] = {-0.210112, 0.135523, 0.125374, 2.320241,  1.570795, -0.821352};
//    const double operateSection1Point2[6] = {-0.312113, 0.182753, 0.125375, 1.570796, -1.570796, -1.570796};
//    const double operateSection1Point3[6] = {-0.312113, 0.182753, 0.046431, 1.570796, -1.570796, -1.570796};
//    const double operateSection1Point4[6] = {-0.312113, 0.182753, 0.125375, 1.570796, -1.570796, -1.570796};
//    const double operateSection1Point5[6] = {-0.210112, 0.135523, 0.125374, 2.320241,  1.570795, -0.821352};

//    const double operateSection1ReadyPose[3] = {-0.21011225, 0.13552309, 0.12537457};

//    const double operateRicePoint1[6] = {-0.205112, 0.105523,  0.001374, 1.570796,  0.000000, -1.570796};
//    const double operateRicePoint2[6] = {-0.284822, 0.077250, -0.051363, 1.570796,  0.698131, -1.570796};
//    const double operateRicePoint3[6] = {-0.284822, 0.077250, -0.088270, 1.570796,  0.698131, -1.570796};
//    const double operateRicePoint4[6] = {-0.285133, 0.026167, -0.088270, 1.570796,  0.698131, -1.570796};
//    const double operateRicePoint5[6] = {-0.285133, 0.015167, -0.080270, 1.564223, -0.782312, -1.566155};
//    const double operateRicePoint6[6] = {-0.285133, 0.015167,  0.011729, 1.564223, -0.782312, -1.566155};
//    const double operateRicePoint7[6] = {-0.205422, 0.026833,  0.058549, 1.564223, -0.782312, -1.566155};
//    const double operateRicePoints[7*6] = {
//        -0.205112, 0.105523,  0.001374, 1.570796,  0.000000, -1.570796,
//        -0.284822, 0.077250, -0.051363, 1.570796,  0.698131, -1.570796,
//        -0.284822, 0.077250, -0.098270, 1.570796,  0.698131, -1.570796,
//        -0.285133, 0.026167, -0.098270, 1.570796,  0.698131, -1.570796,
//        -0.285133, 0.015167, -0.080270, 1.564223, -0.782312, -1.566155,
//        -0.285133, 0.015167,  0.011729, 1.564223, -0.782312, -1.566155,
//        -0.205422, 0.026833,  0.058549, 1.564223, -0.782312, -1.566155
//    };
    const double operateRicePoints[7*6] = {
        -0.205112, 0.105523,  0.001374, 1.570796,  0.000000, -1.570796,
        -0.284822, 0.077250, -0.051363, 1.570796,  0.698131, -1.570796,
        -0.284822, 0.077250, -0.088270, 1.570796,  0.698131, -1.570796,
        -0.285133, 0.022167, -0.088270, 1.570796,  0.698131, -1.570796,
        -0.285133, 0.017167, -0.085270, 1.564223, -0.782312, -1.566155,
        -0.285133, 0.017167,  0.011729, 1.564223, -0.782312, -1.566155,
        -0.205422, 0.026833,  0.058549, 1.564223, -0.782312, -1.566155
    };

    const double operateRicePoints7[7*6] = {
        -0.205112, 0.105523,  0.001374, 1.570796,  0.000000, -1.570796,

        -0.335, 0.127323, -0.051363, 1.570796,  0.698131, -1.570796,
        -0.335, 0.127323, -0.068270, 1.570796,  0.698131, -1.570796,
        -0.335, 0.063537, -0.075270, 1.570796,  0.698131, -1.570796,
//        -0.335, 0.018167, -0.080270, 1.570796, -0.782312, -1.570796,
//        -0.335, 0.019167,  0.011729, 1.570796, -0.782312, -1.570796,
        -0.335, 0.127323, -0.068270, 1.570796,  0.698131, -1.570796,
        -0.335, 0.127323, -0.051363, 1.570796,  0.698131, -1.570796,

        -0.205112, 0.105523,  0.001374, 1.570796,  0.000000, -1.570796,
    };
    const double operateRicePoints8[7*6] = {
        -0.205112, 0.105523,  0.001374, 1.570796,  0.000000, -1.570796,

        -0.3, 0.127323, -0.051363, 1.570796,  0.698131, -1.570796,
        -0.3, 0.127323, -0.068270, 1.570796,  0.698131, -1.570796,
        -0.3, 0.063537, -0.075270, 1.570796,  0.698131, -1.570796,
//        -0.3, 0.018167, -0.080270, 1.570796, -0.782312, -1.570796,
//        -0.3, 0.019167,  0.011729, 1.570796, -0.782312, -1.570796,
        -0.3, 0.127323, -0.068270, 1.570796,  0.698131, -1.570796,
        -0.3, 0.127323, -0.051363, 1.570796,  0.698131, -1.570796,

        -0.205112, 0.105523,  0.001374, 1.570796,  0.000000, -1.570796,
    };
    const double operateRicePoints9[7*6] = {
        -0.205112, 0.105523,  0.001374, 1.570796,  0.000000, -1.570796,

        -0.25, 0.127323, -0.051363, 1.570796,  0.698131, -1.570796,
        -0.25, 0.127323, -0.072270, 1.570796,  0.698131, -1.570796,
        -0.25, 0.063537, -0.078270, 1.570796,  0.698131, -1.570796,
//        -0.25, 0.018167, -0.080270, 1.570796,  0.698131, -1.570796,
//        -0.25, 0.019167,  0.011729, 1.570796, -0.782312, -1.570796,
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

//    const double operateRicePoints[7*6] = {
//        -0.205112, 0.105523,  0.001374, 1.570796,  0.000000, -1.570796,

        // p1
//        -0.322, 0.127323, -0.051363, 1.570796,  0.698131, -1.570796,
//        -0.322, 0.127323, -0.088270, 1.570796,  0.698131, -1.570796,
//        -0.322, 0.022167, -0.088270, 1.570796,  0.698131, -1.570796,
//        -0.322, 0.017167, -0.085270, 1.564223, -0.782312, -1.566155,
//        -0.322, 0.017167,  0.011729, 1.564223, -0.782312, -1.566155,
        // p2
//        -0.282, 0.127323, -0.051363, 1.570796,  0.698131, -1.570796,
//        -0.282, 0.127323, -0.088270, 1.570796,  0.698131, -1.570796,
//        -0.282, 0.022167, -0.088270, 1.570796,  0.698131, -1.570796,
//        -0.282, 0.017167, -0.085270, 1.564223, -0.782312, -1.566155,
//        -0.282, 0.017167,  0.011729, 1.564223, -0.782312, -1.566155,
        // p3
//        -0.242, 0.127323, -0.051363, 1.570796,  0.698131, -1.570796,
//        -0.242, 0.127323, -0.088270, 1.570796,  0.698131, -1.570796,
//        -0.242, 0.022167, -0.088270, 1.570796,  0.698131, -1.570796,
//        -0.242, 0.017167, -0.085270, 1.564223, -0.782312, -1.566155,
//        -0.242, 0.017167,  0.011729, 1.564223, -0.782312, -1.566155,
        // p4
//        -0.322, 0.073537, -0.051363, 1.570796,  0.698131, -1.570796,
//        -0.322, 0.073537, -0.088270, 1.570796,  0.698131, -1.570796,
//        -0.322, 0.022167, -0.088270, 1.570796,  0.698131, -1.570796,
//        -0.322, 0.017167, -0.085270, 1.564223, -0.782312, -1.566155,
//        -0.322, 0.017167,  0.011729, 1.564223, -0.782312, -1.566155,
        // p5 = center
//        -0.282, 0.073537, -0.051363, 1.570796,  0.698131, -1.570796,
//        -0.282, 0.073537, -0.088270, 1.570796,  0.698131, -1.570796,
//        -0.282, 0.022167, -0.088270, 1.570796,  0.698131, -1.570796,
//        -0.282, 0.017167, -0.085270, 1.564223, -0.782312, -1.566155,
//        -0.282, 0.017167,  0.011729, 1.564223, -0.782312, -1.566155,
        // p6
//        -0.242, 0.073537, -0.051363, 1.570796,  0.698131, -1.570796,
//        -0.242, 0.073537, -0.088270, 1.570796,  0.698131, -1.570796,
//        -0.242, 0.022167, -0.088270, 1.570796,  0.698131, -1.570796,
//        -0.242, 0.017167, -0.085270, 1.564223, -0.782312, -1.566155,
//        -0.242, 0.017167,  0.011729, 1.564223, -0.782312, -1.566155,
        // p7
//        -0.322, 0.046643, -0.051363, 1.570796,  0.698131, -1.570796,
//        -0.322, 0.046643, -0.088270, 1.570796,  0.698131, -1.570796,
//        -0.322, 0.022167, -0.088270, 1.570796,  0.698131, -1.570796,
//        -0.322, 0.017167, -0.085270, 1.564223, -0.782312, -1.566155,
//        -0.322, 0.017167,  0.011729, 1.564223, -0.782312, -1.566155,
        // p8
//        -0.282, 0.046643, -0.051363, 1.570796,  0.698131, -1.570796,
//        -0.282, 0.046643, -0.088270, 1.570796,  0.698131, -1.570796,
//        -0.282, 0.022167, -0.088270, 1.570796,  0.698131, -1.570796,
//        -0.282, 0.017167, -0.085270, 1.564223, -0.782312, -1.566155,
//        -0.282, 0.017167,  0.011729, 1.564223, -0.782312, -1.566155,
        // p9
//        -0.242, 0.046643, -0.051363, 1.570796,  0.698131, -1.570796,
//        -0.242, 0.046643, -0.088270, 1.570796,  0.698131, -1.570796,
//        -0.242, 0.022167, -0.088270, 1.570796,  0.698131, -1.570796,
//        -0.242, 0.017167, -0.085270, 1.564223, -0.782312, -1.566155,
//        -0.242, 0.017167,  0.011729, 1.564223, -0.782312, -1.566155,

//        -0.205422, 0.026833,  0.058549, 1.564223, -0.782312, -1.566155
//    };

//    const int32_t offset[6] = {2047, 2047, 2026, 2044, 2031, 3048}; // 06 robot
//    const int32_t offset[6] = {2060, 2042, 2071, 2073, 2046, 2055}; // 05 robot
//    const int32_t offset[6] = {2040, 1634, 491, 1096, 2052, 3048}; // V2
    const int32_t offset[6] = {2047, 2047, 2026, 2044, 2031, 3048}; // V3

    const double obi_ready_joint[5*6] = {
        0.895353, -0.255723, -1.389321,	-1.645044,  0.675442,  3.141592,
        0.615228,  0.160422, -1.864863,	-1.704441,	0.955567,  3.089797,
        0.146956,  0.401652, -2.063777,	-1.662124,	1.423839,  3.089797,
        0.402996,  0.373559, -2.499058,	-2.125499,	1.167799,  0.000174,
        0.915076, -0.201934, -1.914451, -2.116386,  0.655720,  0.000001
    };

    const double obi_ready_pose[5*6] = {
        -0.307112, 0.212752, 0.249374, -1.570796, -5.358979E-08, 1.570796
    };
};

#endif // DATACONTROL_H
