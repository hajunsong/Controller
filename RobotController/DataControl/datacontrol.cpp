#include "datacontrol.h"


DataControl::DataControl()
{
    memset(&ClientToServer, 0, sizeof(ClientToServer));
    memset(&ServerToClient, 0, sizeof(ServerToClient));
    memset(&RobotData, 0, sizeof(RobotData));
    config_check = false;
    cartesian_goal_reach = true;
    joint_goal_reach = true;
    memset(&PathData, 0, sizeof(PathData));

    load_data("data/inverse_kinematics_result.txt", &PathData.pathDataPick, "\t");
    load_data("data/rect_motion.txt", &PathData.pathDataRect, "\t");
    load_data("data/rect_motion2.txt", &PathData.pathDataRect2, "\t");
    load_data("data/CalPos.csv", &PathData.pathDataCalibration, ",");
    load_data("data/linear_motion_42.txt", &PathData.pathDataLinear42, "\t");
    load_data("data/linear_motion_24.txt", &PathData.pathDataLinear24, "\t");
}

DataControl::~DataControl()
{
    uint row = 0;
    row = PathData.pathDataPick.size();
    for(uint i = 0; i < row; i++){
        PathData.pathDataPick[i].clear();
    }
    row = PathData.pathDataRect.size();
    for(uint i = 0; i < row; i++){
        PathData.pathDataRect[i].clear();
    }
    row = PathData.pathDataRect2.size();
    for(uint i = 0; i < row; i++){
        PathData.pathDataRect2[i].clear();
    }
    row = PathData.pathDataCalibration.size();
    for(uint i = 0; i < row; i++){
        PathData.pathDataCalibration[i].clear();
    }
    row = PathData.pathDataLinear42.size();
    for(uint i = 0; i < row; i++){
        PathData.pathDataLinear42[i].clear();
    }
    row = PathData.pathDataLinear24.size();
    for(uint i = 0; i < row; i++){
        PathData.pathDataLinear24[i].clear();
    }
    PathData.pathDataPick.clear();
    PathData.pathDataRect.clear();
    PathData.pathDataRect2.clear();
    PathData.pathDataCalibration.clear();
    PathData.pathDataLinear42.clear();
    PathData.pathDataLinear24.clear();
}

void DataControl::DataReset()
{
    memset(&ClientToServer, 0, sizeof(ClientToServer));
    memset(&ServerToClient, 0, sizeof(ServerToClient));
    config_check = false;
    cartesian_goal_reach = true;
    joint_goal_reach = true;
//    memset(&PathData, 0, sizeof(PathData));
}

void DataControl::jointPositionENC2DEG(int32_t pos_enc[], double pos_deg[])
{
    for(int i = 0; i < NUM_JOINT; i++){
        pos_deg[i] = (pos_enc[i] - offset[i])*ENC2DEG;
    }
}

void DataControl::jointPositionENC2RAD(int32_t pos_enc[], double pos_rad[])
{
    for(int i = 0; i < NUM_JOINT; i++){
        pos_rad[i] = (pos_enc[i] - offset[i])*ENC2DEG*DEG2RAD;
    }
}

void DataControl::cartesianPoseScaleUp(double pose_small[], double pose_big[])
{
    for(int i = 0; i < NUM_DOF; i++){
        if (i < 3){
            pose_big[i] = pose_small[i]*1000;
        }
        else{
            pose_big[i] = pose_small[i]*RAD2DEG;
        }
    }
}

void DataControl::cartesianPoseScaleDown(double pose_big[], double pose_small[])
{
    for(int i = 0; i < NUM_DOF; i++){
        if (i < 3){
            pose_small[i] = pose_big[i]*0.001;
        }
        else{
            pose_small[i] = pose_big[i]*DEG2RAD;
        }
    }
}

void DataControl::jointPositionRAD2ENC(double pos_rad[], int32_t pos_enc[])
{
    for(int i = 0; i < NUM_JOINT; i++){
        pos_enc[i] = static_cast<int32_t>(pos_rad[i]*RAD2DEG*DEG2ENC) + offset[i];
    }
}

void DataControl::jointPositionDEG2ENC(double pos_deg[], int32_t pos_enc[])
{
    for(int i = 0; i < NUM_JOINT; i++){
        pos_enc[i] = static_cast<int32_t>(pos_deg[i]*DEG2ENC) + offset[i];
    }
}

void DataControl::jointPositionDEG2ENC(const int32_t pos_deg[], int32_t pos_enc[])
{
    for(int i = 0; i < NUM_JOINT; i++){
        pos_enc[i] = static_cast<int32_t>(pos_deg[i]*DEG2ENC) + offset[i];
    }
}

void DataControl::jointPositionDEG2ENC(int32_t pos_deg[], int32_t pos_enc[])
{
    for(int i = 0; i < NUM_JOINT; i++){
        pos_enc[i] = static_cast<int32_t>(pos_deg[i]*DEG2ENC) + offset[i];
    }
}

