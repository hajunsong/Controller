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
    RobotData.offset_setting = false;
    RobotData.ik_flag = true;
//    memcpy(RobotData.offset, offset, sizeof(int32_t)*NUM_JOINT);
    obi_mode = false;
    obi_section_indx = -1;
    demo_mode = false;
//    memset(&trayInfor, 0, sizeof(trayInfor));
    trayInfor.section1 = 0;
    trayInfor.section2 = 0;
    trayInfor.section3 = 0;
    trayInfor.section4 = 0;
    trayInfor.section5 = 0;

    KITECHData.camera_request = false;
    KITECHData.camera_request_old = KITECHData.camera_request;
    KITECHData.tablet_check = false;
    KITECHData.tablet_check_old = KITECHData.tablet_check;

//    load_data("motion_data/motion.txt", &PathData.file_data, "\t");
//    load_data("motion_data/side1_motion.txt", &side1_motion.file_data, "\t");
//    load_data("motion_data/side2_motion.txt", &side2_motion.file_data, "\t");
//    load_data("motion_data/side3_motion.txt", &side3_motion.file_data, "\t");
//    load_data("motion_data/rice_motion.txt", &rice_motion.file_data, "\t");
//    load_data("motion_data/soup_motion.txt", &soup_motion.file_data, "\t");

    load_data("motion_data3/side1_1_motion.csv", &side1_motion[0].file_data, ",", side1_motion[0].file_size);
    printf("%d, %d\n", side1_motion[0].file_size[0], side1_motion[0].file_size[1]);
    load_data("motion_data3/side1_2_motion.csv", &side1_motion[1].file_data, ",", side1_motion[1].file_size);
    printf("%d, %d\n", side1_motion[1].file_size[0], side1_motion[0].file_size[1]);
//    load_data("motion_data3/side1_3_motion.csv", &side1_motion[2].file_data, ",", side1_motion[2].file_size);
//    printf("%d, %d\n", side1_motion[2].file_size[0], side1_motion[0].file_size[1]);

    load_data("motion_data3/side2_1_motion.csv", &side2_motion[0].file_data, ",", side2_motion[0].file_size);
    printf("%d, %d\n", side2_motion[0].file_size[0], side2_motion[0].file_size[1]);
    load_data("motion_data3/side2_2_motion.csv", &side2_motion[1].file_data, ",", side2_motion[1].file_size);
    printf("%d, %d\n", side2_motion[1].file_size[0], side2_motion[0].file_size[1]);
//    load_data("motion_data3/side2_3_motion.csv", &side2_motion[2].file_data, ",", side2_motion[2].file_size);
//    printf("%d, %d\n", side2_motion[2].file_size[0], side2_motion[0].file_size[1]);

    load_data("motion_data3/side3_1_motion.csv", &side3_motion[0].file_data, ",", side3_motion[0].file_size);
    printf("%d, %d\n", side3_motion[0].file_size[0], side3_motion[0].file_size[1]);
    load_data("motion_data3/side3_2_motion.csv", &side3_motion[1].file_data, ",", side3_motion[1].file_size);
    printf("%d, %d\n", side3_motion[1].file_size[0], side3_motion[0].file_size[1]);
//    load_data("motion_data3/side3_3_motion.csv", &side3_motion[2].file_data, ",", side3_motion[2].file_size);
//    printf("%d, %d\n", side3_motion[2].file_size[0], side3_motion[0].file_size[1]);

    load_data("motion_data3/rice_1_motion.csv", &rice_motion[0].file_data, ",", rice_motion[0].file_size);
    printf("%d, %d\n", rice_motion[0].file_size[0], rice_motion[0].file_size[1]);
    load_data("motion_data3/rice_2_motion.csv", &rice_motion[1].file_data, ",", rice_motion[1].file_size);
    printf("%d, %d\n", rice_motion[1].file_size[0], rice_motion[0].file_size[1]);
    load_data("motion_data3/rice_3_motion.csv", &rice_motion[2].file_data, ",", rice_motion[2].file_size);
    printf("%d, %d\n", rice_motion[2].file_size[0], rice_motion[0].file_size[1]);

    load_data("motion_data3/soup_motion.csv", &soup_motion.file_data, ",", soup_motion.file_size);
    printf("%d, %d\n", soup_motion.file_size[0], soup_motion.file_size[1]);

    for(unsigned int i = 0; i  < 6; i++){
        obi_ready_joint[0*6 + i] = side1_motion[0].file_data[500*side1_motion[0].file_size[1]+i];
        obi_ready_joint[1*6 + i] = side2_motion[0].file_data[500*side2_motion[0].file_size[1]+i];
        obi_ready_joint[2*6 + i] = side3_motion[0].file_data[500*side3_motion[0].file_size[1]+i];
        obi_ready_joint[3*6 + i] = soup_motion.file_data[500*soup_motion.file_size[1]+i];
        obi_ready_joint[4*6 + i] = rice_motion[0].file_data[500*rice_motion[0].file_size[1]+i];
    }
}

DataControl::~DataControl()
{
    PathData.readyPath.path_x.clear();
    PathData.readyPath.path_y.clear();
    PathData.readyPath.path_z.clear();
    for(uint i = 0; i < PathData.row; i++){
        PathData.movePath[i].path_x.clear();
        PathData.movePath[i].path_y.clear();
        PathData.movePath[i].path_z.clear();
        PathData.movePath[i].path_theta.clear();
    }
    PathData.file_data.clear();
//    side1_motion.file_data.clear();
//    side2_motion.file_data.clear();
//    side3_motion.file_data.clear();
//    rice_motion.file_data.clear();
//    soup_motion.file_data.clear();
    for(int i = 0; i < 3; i++){
        side1_motion[i].file_data.clear();
        side2_motion[i].file_data.clear();
        side3_motion[i].file_data.clear();
        rice_motion[i].file_data.clear();
    }
    soup_motion.file_data.clear();
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

void DataControl::jointPositionDEG2ENC(const double pos_deg[], int32_t pos_enc[])
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

void DataControl::jointPositionRAD2DEG(double pos_rad[], double pos_deg[])
{
	for(int i = 0; i < NUM_JOINT; i++){
		pos_deg[i] = pos_rad[i]*RAD2DEG;
    }
}

void DataControl::jointCurrentRAW2Torque(int16_t cur_raw[], double cur_torque[])
{
    if(MODULE_TYPE == 2){

    }
    else if(MODULE_TYPE == 3){
        cur_torque[0] = cur_raw[0]*RAW2mA*0.001*TORQUE_CONSTANT_W270;
        cur_torque[1] = cur_raw[1]*RAW2mA*0.001*TORQUE_CONSTANT_W270;
        cur_torque[2] = cur_raw[2]*RAW2mA*0.001*TORQUE_CONSTANT_W270;
        cur_torque[3] = cur_raw[3]*RAW2mA*0.001*TORQUE_CONSTANT_W350;
        cur_torque[4] = cur_raw[4]*RAW2mA*0.001*TORQUE_CONSTANT_W350;
        cur_torque[5] = cur_raw[5]*RAW2mA*0.001*TORQUE_CONSTANT_W350;
    }
}
