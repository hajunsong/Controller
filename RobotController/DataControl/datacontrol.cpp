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
    memset(&torqueIdeData, 0, sizeof(torqueIdeData));
    memset(&PIDControl, 0, sizeof(PIDControl));

//    load_data("data/motion_pick.txt", &PathData.pathDataPick, "\t");
//    load_data("data/rect_motion.txt", &PathData.pathDataRect, "\t");
//    load_data("data/motion7.txt", &PathData.pathDataRect2, "\t");
//    load_data("data/CalPos.csv", &PathData.pathDataCalibration, ",");
//    load_data("data/linear_motion_42.txt", &PathData.pathDataLinear42, "\t");
//    load_data("data/linear_motion_24.txt", &PathData.pathDataLinear24, "\t");
}

DataControl::~DataControl()
{
//    PathData.pathDataPick.clear();
//    PathData.pathDataRect.clear();
//    PathData.pathDataRect2.clear();
//    PathData.pathDataCalibration.clear();
//    PathData.pathDataLinear42.clear();
//    PathData.pathDataLinear24.clear();
    PathData.path_x.clear();
    PathData.path_y.clear();
    PathData.path_z.clear();
}

void DataControl::DataReset()
{
    memset(&ClientToServer, 0, sizeof(ClientToServer));
    memset(&ServerToClient, 0, sizeof(ServerToClient));
    config_check = false;
    cartesian_goal_reach = true;
    joint_goal_reach = true;
    memset(&PIDControl, 0, sizeof(PIDControl));
//    memset(&PathData, 0, sizeof(PathData));
}

void DataControl::jointPositionENC2DEG(int32_t pos_enc[], double pos_deg[])
{
    for(int i = 0; i < NUM_JOINT; i++){
        pos_deg[i] = (pos_enc[i] - RobotData.offset[i])*ENC2DEG;
    }
}

void DataControl::jointPositionENC2RAD(int32_t pos_enc[], double pos_rad[])
{
    for(int i = 0; i < NUM_JOINT; i++){
        pos_rad[i] = (pos_enc[i] - RobotData.offset[i])*ENC2DEG*DEG2RAD;
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
        pos_enc[i] = static_cast<int32_t>(pos_rad[i]*RAD2DEG*DEG2ENC) + RobotData.offset[i];
    }
}

void DataControl::jointPositionDEG2ENC(double pos_deg[], int32_t pos_enc[])
{
    for(int i = 0; i < NUM_JOINT; i++){
        pos_enc[i] = static_cast<int32_t>(pos_deg[i]*DEG2ENC) + RobotData.offset[i];
    }
}

void DataControl::jointPositionDEG2ENC(const int32_t pos_deg[], int32_t pos_enc[])
{
    for(int i = 0; i < NUM_JOINT; i++){
        pos_enc[i] = static_cast<int32_t>(pos_deg[i]*DEG2ENC) + RobotData.offset[i];
    }
}

void DataControl::jointPositionDEG2ENC(int32_t pos_deg[], int32_t pos_enc[])
{
    for(int i = 0; i < NUM_JOINT; i++){
        pos_enc[i] = static_cast<int32_t>(pos_deg[i]*DEG2ENC) + RobotData.offset[i];
    }
}

void DataControl::jointVelocityENC2RPM(int32_t vel_enc[], double vel_rpm[])
{
    for(int i = 0; i < NUM_JOINT; i++){
        vel_rpm[i] = vel_enc[i]*ENC2RPM;
    }
}

void DataControl::jointVelocityENC2RAD(int32_t vel_enc[], double vel_rad[]){
    for(int i = 0; i < NUM_JOINT; i++){
        vel_rad[i] = vel_enc[i]*ENC2RPM*RPM2DEG*DEG2RAD;
    }
}

void DataControl::jointCurrentRAW2mA(int16_t cur_raw[], double cur_mA[])
{
    for(int i = 0; i < NUM_JOINT; i++){
        cur_mA[i] = cur_raw[i]*RAW2mA;
    }
}

void DataControl::jointCurrentmA2RAW(double cur_mA[], int16_t cur_raw[])
{
    for(int i = 0; i < NUM_JOINT; i++){
        cur_raw[i] = static_cast<int16_t>(cur_mA[i]*mA2RAW);
    }
}

void DataControl::PathGenerator(double start_pt, double final_pt, double start_vel, double final_vel, double start_acc, double final_acc, double tf, double step_size, std::vector<double> *path){
    double a0 = 0, a1 = 0, a2 = 0, a3 = 0, a4 = 0, a5 = 0;

    a0 = start_pt;
    a1 = start_vel;
    a2 = start_acc / 2;
    a3 = (20*(final_pt - start_pt) - (8*final_vel + 12*start_vel)*tf - (3*start_acc - final_acc)*tf*tf)/(2*tf*tf*tf);
    a4 = (30*(start_pt - final_pt) + (14*final_vel + 16*start_vel)*tf + (3*start_acc - 2*final_acc)*tf*tf)/(2*tf*tf*tf*tf);
    a5 = (12*(final_pt - start_pt) - (6*final_vel + 6*start_vel)*tf - (start_acc - final_acc)*tf*tf)/(2*tf*tf*tf*tf*tf);

    for(double t = 0; t < tf; t += step_size){
        path->push_back(a0 + a1*t + a2*t*t + a3*t*t*t + a4*t*t*t*t + a5*t*t*t*t*t);
    }
}
