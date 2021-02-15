#pragma once

#include <string.h>
#include <stdio.h>
#include <iostream>
#include <vector>

#define NUM_JOINT               6
#define NUM_DOF                 6

#define DATA_MAX_INDX           30

#define NRMK_SOCKET_TOKEN_SIZE  2

//#define MODULE_TYPE             2 // 1:FAR, 2:SEA
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
#define CLIENT_TO_SERVER_LEM    NRMK_SOCKET_TOKEN_SIZE + OP_MODE_LEN + JOINT_OP_MODE_LEN + SUB_MODE_LEN + DESIRED_JOINT_LEN*NUM_JOINT + \
    DESIRED_CARTESIAN_LEN*NUM_DOF + NRMK_SOCKET_TOKEN_SIZE

#define PATH_TYPE_LEN           1
#define CYCLE_COUNT_LEN         1
#define PATH_ROW_LEN            2
#define PATH_COL_LEN            2
#define PATH_DATA_LEN           8

#include <QtCore/qglobal.h>

#if defined(DATACONTROLLIB_LIBRARY)
#  define DATACONTROLLIB_EXPORT Q_DECL_EXPORT
#else
#  define DATACONTROLLIB_EXPORT Q_DECL_IMPORT
#endif

class DataControl{
public:
//    typedef struct _StructClientToServerInitParam{
//        char numJoint, numDof, module;
//    }StructClientToServerInitParam;

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
        double time, dxl_time, ik_time, t;
    }StructServerToClient;

    enum OpMode{ServoOnOff = 0, Initialize, Wait, JointMove, CartesianMove, PathGenerateMode, ReadyMode, RunMode, TorqueID, OperateMode};
    enum Motion{JogMotion = 0, JointMotion, CartesianJogMotion, CartesianMotion};
    enum Module{FAR_V1=1, FAR_V2, SEA};
    enum Comm{RS485=1, RS232, EtherCAT};
    enum CmdType{PathCmd=1, ReadyCmd, RunCmd, StopCmd, FileReady, FileRun, CustomRun};
    enum Operate{Start=1, Stop, StartTeaching, StopTeaching, StartFeeding, StopFeeding, Feeding};
    enum Section{Ready=0, Side1=1, Side2, Side3, Soup, Rise};

    DataControl();
    ~DataControl();
    void DataReset();

    StructClientToServer ClientToServer;
    StructServerToClient ServerToClient;

    const double ENC2DEG = 0.088;
    const double DEG2ENC = 11.363636364;
    const double DEG2RAD = 0.017453293;//3.14159265358979323846/180.0;
    const double RAD2DEG = 57.295779513;//180.0/3.14159265358979323846;
    const double ENC2RPM = 	0.229;
    const double RPM2DEG = 6;
    const double RAW2mA = 2.69;
    const double mA2RAW = 0.371747212;
};

enum { current_mode = 0, velocity_mode, position_mode = 3, extended_position_mode, current_based_position_mode, pwm_mode = 16 };
const int32_t JointOpMode[7] = {0, current_mode, velocity_mode, position_mode, extended_position_mode, current_based_position_mode, pwm_mode};
