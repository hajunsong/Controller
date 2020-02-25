#ifndef DATACONTROL_H
#define DATACONTROL_H

#include <QtCore/qglobal.h>

#if defined(DATACONTROLLIB_LIBRARY)
#  define DATACONTROLLIB_EXPORT Q_DECL_EXPORT
#else
#  define DATACONTROLLIB_EXPORT Q_DECL_IMPORT
#endif

#include <string.h>
#include <stdio.h>
#include <iostream>
#include <vector>

#include <NRMKSocketBase.h>
#include <native/timer.h>

#include "FileIO/fileio.h"

#define DATA_MAX_INDX           30

#define NUM_JOINT               6
#define NUM_DOF                 6
#define MODULE_TYPE             2 // 1:FAR V1, 2:VAR V2
#define DATA_INDEX_LEN          1
#define JOINT_POSITION_LEN      8
#define CARTESIAN_POSE_LEN      8
#define JOINT_COMMAND_LEN       8
#define CARTESIAN_COMMAND_LEN   8
#define CARTESIAN_CALCULATE_LEN 8
#define JOINT_VELOCITY_LEN      8
#define JOINT_CURRENT_LEN       8
#define CARTESIAN_VELOCITY_LEN  8
#define TIME_LEN                8
#define SERVER_TO_CLIENT_LEN    (NRMK_SOCKET_TOKEN_SIZE + DATA_INDEX_LEN + DATA_INDEX_LEN + \
    JOINT_POSITION_LEN*NUM_JOINT + CARTESIAN_POSE_LEN*NUM_DOF + JOINT_COMMAND_LEN*NUM_JOINT + \
    CARTESIAN_COMMAND_LEN*NUM_DOF + CARTESIAN_CALCULATE_LEN*NUM_DOF + JOINT_VELOCITY_LEN*NUM_JOINT + JOINT_CURRENT_LEN*NUM_JOINT + \
    TIME_LEN + TIME_LEN + TIME_LEN + NRMK_SOCKET_TOKEN_SIZE + CARTESIAN_VELOCITY_LEN*NUM_DOF)*DATA_MAX_INDX

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

class DATACONTROLLIB_EXPORT DataControl
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
        bool offset_setting;
        uint8_t joint_op_mode;
        uint8_t run_mode;
        double present_end_vel[NUM_DOF];
        double present_cal_end_pose[NUM_DOF];
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
        double teaching_pose[NUM_JOINT];
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

    enum OpMode{ServoOnOff = 0, Initialize, Wait, JointMove, CartesianMove, PathGenerateMode, ReadyMode, RunMode, TorqueID, OperateMode};
    enum Motion{JogMotion = 0, JointMotion, CartesianJogMotion, CartesianMotion};
	enum Module{FAR_V1=1, FAR_V2};
    enum Comm{RS485=1, RS232, EtherCAT};
    enum CmdType{PathCmd=1, ReadyCmd, RunCmd, StopCmd, FileReady, FileRun, CustomRun};
    enum Operate{Start=1, Stop, StartTeaching, StopTeaching, StartFeeding, StopFeeding, Feeding};
    enum Section{Side1=1, Side2, Side3, Soup, Rise, Mouse};

    bool config_check;
    bool cartesian_goal_reach;
    bool joint_goal_reach;

    DataControl();
    ~DataControl();
    void DataReset();

    StructClientToServer ClientToServer;
    std::vector<StructServerToClient> ServerToClient;
    StructRobotData RobotData;
    StructPathData PathData, side1_motion, side2_motion, side3_motion, rise_motion, soup_motion;
    StructTorqueIDEData torqueIdeData;
    StructControllerPID PIDControl;
    StructOperateMode operateMode;

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
	void jointPositionRAD2DEG(double pos_rad[], double pos_deg[]);

    const double ENC2DEG = 0.088;
    const double DEG2ENC = 11.363636364;
    const double DEG2RAD = 0.017453293;//3.14159265358979323846/180.0;
    const double RAD2DEG = 57.295779513;//180.0/3.14159265358979323846;
    const double ENC2RPM = 	0.229;
    const double RPM2DEG = 6;
    const double RAW2mA = 2.69;
    const double mA2RAW = 0.371747212;

    const double operateReadyJoint[6] = {0.74944438, 0.40421825, -2.0655972, -1.6613789, 0.82135195, 0};
    const double operateReadyPose[6] = {-0.21011225, 0.13552308, 0.12537458, 1.5707963, 0, -1.5707963};
    const double operateFeedingReadyPose[6] = {-0.21011225, 0.13552308, 0.12537459, 1.5642233, -0.78231204, -1.5661557};
    bool feeding;

//    const int32_t offset[6] = {2202, 500, 1672, 3200, 901, 1924};
};

#endif // DATACONTROL_H
