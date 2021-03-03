#include "controlmain_custom.h"
#include "ControlMain/controlmain.h"

ControlMainCustom::ControlMainCustom(){
}

ControlMainCustom::~ControlMainCustom(){
}

int ControlMainCustom::robot_run(void *arg)
{
    ControlMain* pThis = static_cast<ControlMain*>(arg);
    RTIME now, previous;
    previous = rt_timer_read();
    pThis->dataControl->RobotData.time1 = static_cast<unsigned long>(rt_timer_read());

    DataControl::StructServerToClient ServerToClientTemp;
    pThis->dataControl->RobotData.time2 = static_cast<unsigned long>(rt_timer_read());

    switch(pThis->dataControl->ClientToServer.opMode){
        case DataControl::OpMode::ServoOnOff:
//            pThis->robotServoOn(pThis->dataControl->ClientToServer.subMode);
            break;
        case DataControl::OpMode::Initialize:
//            pThis->robotInitialize();
            break;
        case DataControl::OpMode::Wait:
            break;
        case DataControl::OpMode::JointMove:
//            pThis->robotJointMove(pThis->dataControl->ClientToServer.subMode, pThis->dataControl->ClientToServer.desiredJoint);
            break;
        case DataControl::OpMode::CartesianMove:
//            pThis->robotCartesianMove(pThis->dataControl->ClientToServer.subMode, pThis->dataControl->ClientToServer.desiredPose);
            break;
        case DataControl::OpMode::PathGenerateMode:
            break;
        case DataControl::OpMode::ReadyMode:
//            pThis->robotReady();
            break;
        case DataControl::OpMode::RunMode:
//            pThis->robotRun();
            break;
        case DataControl::OpMode::TorqueID:
//            pThis->robotSPGC();
            break;
        case DataControl::OpMode::OperateMode:
//            pThis->robotOperate();
            break;
        case DataControl::OpMode::ObiMode:
            break;
        default : break;
    }

    pThis->dataControl->RobotData.dxl_time1 = static_cast<unsigned long>(rt_timer_read());
    if (NUM_JOINT == 6){
        if (MODULE_TYPE == DataControl::Module::FAR_V1){
//                pThis->module->getGroupSyncReadIndirectAddress(pThis->dataControl->RobotData.present_joint_position, pThis->dataControl->RobotData.present_joint_velocity,
//                                                               pThis->dataControl->RobotData.present_joint_current, NUM_JOINT);
        }
        else if(MODULE_TYPE == DataControl::Module::FAR_V2){
            pThis->module->getGroupSyncReadIndirectAddress(pThis->dataControl->RobotData.present_joint_position, pThis->dataControl->RobotData.present_joint_velocity,
                                                           pThis->dataControl->RobotData.present_joint_current, pThis->dataControl->RobotData.moving,
                                                           pThis->dataControl->RobotData.moving_status, NUM_JOINT);
        }
        else if(MODULE_TYPE == DataControl::Module::FAR_V3){
            pThis->module->getGroupSyncReadIndirectAddress(pThis->dataControl->RobotData.present_joint_position, pThis->dataControl->RobotData.present_joint_velocity,
                                                           pThis->dataControl->RobotData.present_joint_current, pThis->dataControl->RobotData.moving,
                                                           pThis->dataControl->RobotData.moving_status, NUM_JOINT);
        }

    }
    else if(NUM_JOINT == 1){
        pThis->module->getGroupSyncReadIndirectAddress(&pThis->dataControl->RobotData.present_joint_position[0],
                &pThis->dataControl->RobotData.present_joint_velocity[0],
                &pThis->dataControl->RobotData.present_joint_current[0], NUM_JOINT, pThis->module->single_id);
    }
    pThis->dataControl->RobotData.dxl_time2 = static_cast<unsigned long>(rt_timer_read());

    if (NUM_JOINT == 6){
//        pThis->robotKinematics();
//        pThis->robotDynamics();
    }

        rt_printf("[FK] present joint position : %d, %d, %d, %d, %d, %d\n",
                  pThis->dataControl->RobotData.present_joint_position[0], pThis->dataControl->RobotData.present_joint_position[1],
                pThis->dataControl->RobotData.present_joint_position[2], pThis->dataControl->RobotData.present_joint_position[3],
                pThis->dataControl->RobotData.present_joint_position[4], pThis->dataControl->RobotData.present_joint_position[5]);
        rt_printf("[FK] present q : %f, %f, %f, %f, %f, %f\n",
                  pThis->dataControl->RobotData.present_q[0], pThis->dataControl->RobotData.present_q[1],
                pThis->dataControl->RobotData.present_q[2], pThis->dataControl->RobotData.present_q[3],
                pThis->dataControl->RobotData.present_q[4], pThis->dataControl->RobotData.present_q[5]);

    ServerToClientTemp.data_index = pThis->dataControl->data_index;

    pThis->dataControl->jointPositionENC2DEG(pThis->dataControl->RobotData.present_joint_position, ServerToClientTemp.presentJointPosition);
    pThis->dataControl->cartesianPoseScaleUp(pThis->dataControl->RobotData.present_end_pose, ServerToClientTemp.presentCartesianPose);
    pThis->dataControl->jointVelocityENC2RPM(pThis->dataControl->RobotData.present_joint_velocity, ServerToClientTemp.presentJointVelocity);
    pThis->dataControl->jointCurrentRAW2mA(pThis->dataControl->RobotData.present_joint_current, ServerToClientTemp.presentJointCurrent);
    pThis->dataControl->cartesianPoseScaleUp(pThis->dataControl->RobotData.desired_end_pose, ServerToClientTemp.desiredCartesianPose);
    pThis->dataControl->jointPositionRAD2DEG(pThis->dataControl->RobotData.desired_q, ServerToClientTemp.desiredJointPosition);
//    memcpy(ServerToClientTemp.calculateCartesianPose, pThis->dataControl->RobotData.present_cal_end_pose, sizeof(double)*NUM_DOF);
    memcpy(ServerToClientTemp.presentCartesianVelocity, pThis->dataControl->RobotData.present_end_vel, sizeof(double)*NUM_DOF);
    memcpy(ServerToClientTemp.presentJointTorque, pThis->dataControl->RobotData.present_joint_torque, sizeof(double)*NUM_DOF);
    memcpy(ServerToClientTemp.presentJointResidual, pThis->dataControl->RobotData.present_joint_residual, sizeof(double)*NUM_DOF);

    ServerToClientTemp.time = static_cast<double>((pThis->dataControl->RobotData.time2 - pThis->dataControl->RobotData.time1)/1000000.0);
    ServerToClientTemp.dxl_time = static_cast<double>((pThis->dataControl->RobotData.dxl_time2 - pThis->dataControl->RobotData.dxl_time1)/1000000.0);
    ServerToClientTemp.ik_time = static_cast<double>((pThis->dataControl->RobotData.ik_time2 - pThis->dataControl->RobotData.ik_time1)/1000000.0);

    pThis->dataControl->RobotData.time1 = pThis->dataControl->RobotData.time2;
    pThis->dataControl->data_index++;

    if(pThis->tcpServerClient->isConnected()){
        pThis->dataControl->ServerToClient.push_back(ServerToClientTemp);
    }
    else{
        return -1;
    }

    now = rt_timer_read();
}