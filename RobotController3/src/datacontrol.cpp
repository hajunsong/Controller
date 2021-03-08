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
    tablet_mode = false;
    section_indx = -1;
    trayInfor.section1 = 0;
    trayInfor.section2 = 0;
    trayInfor.section3 = 0;
    trayInfor.section4 = 0;
    trayInfor.section5 = 0;

    KITECHData.camera_request = false;
    KITECHData.camera_request_old = KITECHData.camera_request;
    KITECHData.tablet_check = false;
    KITECHData.tablet_check_old = KITECHData.tablet_check;
    KITECHData.interface_cmd = 0;
    KITECHData.interface_sub = 0;

    memset(joint_offset, 0, sizeof(int32_t)*6);
    memset(tool_offset, 0, sizeof(double)*3);

    char readBuffer[65536];
    FILE* fp = fopen("config.json", "r");
    FileReadStream frs(fp, readBuffer, sizeof(readBuffer));
    Document document;
    document.ParseStream(frs);

    const Value& joint_offset_json = document["JOINT_OFFSET"];
    assert(joint_offset_json.IsArray());
    for (SizeType i = 0; i < joint_offset_json.Size(); i++){
        printf("joint_offset[%d] = %d\n", i, joint_offset_json[i].GetInt());
        joint_offset[i] = joint_offset_json[i].GetInt();
    }
    const Value& tool_offset_json = document["TOOL_OFFSET"];
    assert(tool_offset_json.IsArray());
    for (SizeType i = 0; i < tool_offset_json.Size(); i++){
        printf("tool_offset[%d] = %f\n", i, tool_offset_json[i].GetDouble());
        tool_offset[i] = tool_offset_json[i].GetDouble();
    }
    const Value& camera_joint_json = document["CAMERA_JOINT"];
    assert(camera_joint_json.IsArray());
    for (SizeType i = 0; i < camera_joint_json.Size(); i++){
        printf("camera_joint[%d] = %f\n", i, camera_joint_json[i].GetDouble());
        operateCameraReadyJoint[i] = camera_joint_json[i].GetDouble();
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
        pos_deg[i] = (pos_enc[i] - RobotData.joint_offset[i])*ENC2DEG;
    }
}

void DataControl::jointPositionENC2RAD(int32_t pos_enc[], double pos_rad[])
{
    for(int i = 0; i < NUM_JOINT; i++){
        pos_rad[i] = (pos_enc[i] - RobotData.joint_offset[i])*ENC2DEG*DEG2RAD;
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
        pos_enc[i] = static_cast<int32_t>(pos_rad[i]*RAD2DEG*DEG2ENC) + RobotData.joint_offset[i];
    }
}

void DataControl::jointPositionDEG2ENC(double pos_deg[], int32_t pos_enc[])
{
    for(int i = 0; i < NUM_JOINT; i++){
        pos_enc[i] = static_cast<int32_t>(pos_deg[i]*DEG2ENC) + RobotData.joint_offset[i];
    }
}

void DataControl::jointPositionDEG2ENC(const double pos_deg[], int32_t pos_enc[])
{
    for(int i = 0; i < NUM_JOINT; i++){
        pos_enc[i] = static_cast<int32_t>(pos_deg[i]*DEG2ENC) + RobotData.joint_offset[i];
    }
}

void DataControl::jointPositionDEG2ENC(const int32_t pos_deg[], int32_t pos_enc[])
{
    for(int i = 0; i < NUM_JOINT; i++){
        pos_enc[i] = static_cast<int32_t>(pos_deg[i]*DEG2ENC) + RobotData.joint_offset[i];
    }
}

void DataControl::jointPositionDEG2ENC(int32_t pos_deg[], int32_t pos_enc[])
{
    for(int i = 0; i < NUM_JOINT; i++){
        pos_enc[i] = static_cast<int32_t>(pos_deg[i]*DEG2ENC) + RobotData.joint_offset[i];
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
