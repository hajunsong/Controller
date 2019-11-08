#pragma once

#include <string.h>
#include <stdio.h>
#include <iostream>
#include <vector>

#define NUM_JOINT               6
#define NUM_DOF                 6

#define NRMK_SOCKET_TOKEN_SIZE  2

#define MODULE_TYPE             1 // 1:FAR, 2:SEA
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
#define CLIENT_TO_SERVER_LEM    NRMK_SOCKET_TOKEN_SIZE + OP_MODE_LEN + JOINT_OP_MODE_LEN + SUB_MODE_LEN + DESIRED_JOINT_LEN*NUM_JOINT + \
    DESIRED_CARTESIAN_LEN*NUM_DOF + NRMK_SOCKET_TOKEN_SIZE

#define PATH_TYPE_LEN           1
#define CYCLE_COUNT_LEN         1
#define PATH_ROW_LEN            2
#define PATH_COL_LEN            2
#define PATH_DATA_LEN           8


class DataControl{
public:
//    typedef struct _StructClientToServerInitParam{
//        char numJoint, numDof, module;
//    }StructClientToServerInitParam;

    typedef struct _StructClientToServer{
        char opMode, subMode;
        double desiredJoint[NUM_JOINT], desiredCartesian[NUM_DOF];
    }StructClientToServer;

    typedef struct _StructServerToClient{
        unsigned char data_index;
        double presentJointPosition[NUM_JOINT], presentCartesianPose[NUM_DOF];
        double desiredJointPosition[NUM_JOINT], desiredCartesianPose[NUM_DOF];
        double calculateCartesianPose[NUM_DOF];
        double presentJointVelocity[NUM_JOINT], presentJointCurrent[NUM_JOINT];
        double time, dxl_time, ik_time;
    }StructServerToClient;

    enum OpMode{ServoOnOff = 0, Initialize, Wait, JointMove, CartesianMove, PathGenerateMode, ReadyMode, RunMode, TorqueIDE};
    enum Motion{JogMotion = 0, JointMotion, CartesianJogMotion, CartesianMotion};
    enum Module{FAR_V1=1, FAR_V2, SEA};
    enum Comm{RS485=1, RS232, EtherCAT};
    enum PathDataType{JointPath = 1, CartPath, Save1, Save2, Save3, Save4};
    enum CmdType{PathCmd=1, ReadyCmd, RunCmd, StopCmd};

    DataControl();
    ~DataControl();
    void DataReset();

    StructClientToServer ClientToServer;
    StructServerToClient ServerToClient;
};

enum { current_mode = 0, velocity_mode, position_mode = 3, extended_position_mode, current_based_position_mode, pwm_mode = 16 };
const int32_t JointOpMode[7] = {0, current_mode, velocity_mode, position_mode, extended_position_mode, current_based_position_mode, pwm_mode};
