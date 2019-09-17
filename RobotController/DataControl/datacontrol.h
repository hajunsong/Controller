#pragma once

#include <string.h>
#include <stdio.h>
#include <iostream>
#include <vector>

#include <NRMKSocketBase.h>

#include "FileIO/fileio.h"

#define NUM_JOINT               6
#define NUM_DOF                 6
#define MODULE_TYPE             1 // 1:FAR, 2:SEA, 3:JS-R8
#define DATA_INDEX_LEN          1
#define JOINT_POSITION_LEN      8
#define CARTESIAN_POSE_LEN      8
#define JOINT_COMMAND_LEN       8
#define CARTESIAN_COMMAND_LEN   8
#define CARTESIAN_CALCULATE_LEN 8
#define TIME_LEN                8
#define SERVER_TO_CLIENT_LEN    NRMK_SOCKET_TOKEN_SIZE + DATA_INDEX_LEN + \
    JOINT_POSITION_LEN*NUM_JOINT + CARTESIAN_POSE_LEN*NUM_DOF + JOINT_COMMAND_LEN*NUM_JOINT + \
    CARTESIAN_COMMAND_LEN*NUM_DOF + CARTESIAN_CALCULATE_LEN*NUM_DOF + TIME_LEN + TIME_LEN + TIME_LEN + NRMK_SOCKET_TOKEN_SIZE

#define OP_MODE_LEN             1
#define SUB_MODE_LEN            1
#define DESIRED_JOINT_LEN       8
#define DESIRED_CARTESIAN_LEN   8
#define CLIENT_TO_SERVER_LEM    NRMK_SOCKET_TOKEN_SIZE + OP_MODE_LEN + SUB_MODE_LEN + DESIRED_JOINT_LEN*NUM_JOINT + DESIRED_CARTESIAN_LEN*NUM_DOF + NRMK_SOCKET_TOKEN_SIZE

#define PATH_TYPE_LEN           1
#define CYCLE_COUNT_LEN         1

class DataControl{
public:
    typedef struct _StructClientToServer{
        char opMode, subMode;
        double desiredJoint[NUM_JOINT], desiredCartesian[NUM_DOF];
    }StructClientToServer;

    typedef struct _StructServerToClient{
        unsigned char data_index;
        double presentJointPosition[NUM_JOINT], presentCartesianPose[NUM_DOF];
        double desiredJointPosition[NUM_JOINT], desiredCartesianPose[NUM_DOF];
        double calculateCartesianPose[NUM_DOF];
        double time, dxl_time, ik_time;
    }StructServerToClient;

    typedef struct _StructRobotData{
        int32_t present_joint_position[NUM_JOINT];
        double present_end_pose[NUM_DOF];
        double desired_end_pose[NUM_DOF];
        double old_desired_end_pose[NUM_DOF];
        double present_q[NUM_JOINT];
        double desired_q[NUM_JOINT];
        int32_t command_joint_position[NUM_JOINT];
        unsigned long time1, time2, dxl_time1, dxl_time2, ik_time1, ik_time2;
    }StructRobotData;

    typedef struct _StructPathData{
        char type, repeat;
        uint16_t row, col;
        std::vector<std::vector<double> > pathDataPick;
        std::vector<std::vector<double> > pathDataRect;
        uint path_data_indx;
    }StructPathData;

    enum OpMode{ServoOnOff = 0, Initialize, Wait, JointMove, CartesianMove, RunMode};
    enum Motion{JogMotion = 0, JointMotion, CartesianJogMotion, CartesianMotion};
    enum Module{FAR=1, SEA, JS_R8};
    enum Comm{RS485=1, RS232, EtherCAT};
    enum PathDataType{Save1=1, Save2, Save3, Save4};

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

    void jointPositionENC2DEG(int32_t pos_enc[], double pos_deg[]);
    void jointPositionENC2RAD(int32_t pos_enc[], double pos_rad[]);
    void cartesianPoseScaleUp(double pose_small[], double pose_big[]);
    void cartesianPoseScaleDown(double pose_big[], double pose_small[]);
    void jointPositionRAD2ENC(double pos_rad[], int32_t pos_enc[]);
    void jointPositionDEG2ENC(double pos_deg[], int32_t pos_end[]);
    void jointPositionDEG2ENC(const int32_t pos_deg[], int32_t pos_end[]);
    void jointPositionDEG2ENC(int32_t pos_deg[], int32_t pos_end[]);

    const double ENC2DEG = 0.088;
    const double DEG2ENC = 11.363636364;
    const double DEG2RAD = 0.017453293;//3.14159265358979323846/180.0;
    const double RAD2DEG = 57.295779513;//180.0/3.14159265358979323846;

    const int32_t offset[6] = {2038, 500, 1672, 3200, 1142, 340};
    const int32_t ready_pose[6] = {30, 30, 90, -120, 30, 0};
};
