#include "controlmain_custom.h"
#include "controlmain.h"

ControlMainCustom::ControlMainCustom(){
}

ControlMainCustom::~ControlMainCustom(){
}

void ControlMainCustom::robot_run(void *arg)
{
    ControlMain* pThis = static_cast<ControlMain*>(arg);
    RTIME now, previous;
    previous = rt_timer_read();
    pThis->dataControl->RobotData.time1 = static_cast<unsigned long>(rt_timer_read());

    pThis->robot_task_run = false;

    rt_task_set_periodic(&pThis->robot_task, TM_NOW, 3e6);

    pThis->robot_task_run = true;

    while(pThis->robot_task_run){
        rt_task_wait_period(nullptr); //wait for next cycle

        DataControl::StructServerToClient ServerToClientTemp;
        pThis->dataControl->RobotData.time2 = static_cast<unsigned long>(rt_timer_read());

        switch(pThis->dataControl->ClientToServer.opMode){
            case DataControl::OpMode::ServoOnOff:
                pThis->robotServoOn(pThis->dataControl->ClientToServer.subMode);
                break;
            case DataControl::OpMode::Initialize:
//                pThis->robotInitialize();
                break;
            case DataControl::OpMode::Wait:
                break;
            case DataControl::OpMode::JointMove:
                pThis->robotJointMove(pThis->dataControl->ClientToServer.subMode, pThis->dataControl->ClientToServer.desiredJoint);
                break;
            case DataControl::OpMode::CartesianMove:
                pThis->robotCartesianMove(pThis->dataControl->ClientToServer.subMode, pThis->dataControl->ClientToServer.desiredPose);
                break;
            case DataControl::OpMode::PathGenerateMode:
                break;
            case DataControl::OpMode::ReadyMode:
                pThis->robotReady();
                break;
            case DataControl::OpMode::RunMode:
                pThis->robotRun();
                break;
            case DataControl::OpMode::TorqueID:
                pThis->robotSPGC();
                break;
            case DataControl::OpMode::OperateMode:
                pThis->robotOperate();
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
            pThis->robotKinematics();
            pThis->robotDynamics();
        }

        rt_printf("present joint position : %d, %d, %d, %d, %d, %d\n",
                  pThis->dataControl->RobotData.present_joint_position[0], pThis->dataControl->RobotData.present_joint_position[1],
                pThis->dataControl->RobotData.present_joint_position[2], pThis->dataControl->RobotData.present_joint_position[3],
                pThis->dataControl->RobotData.present_joint_position[4], pThis->dataControl->RobotData.present_joint_position[5]);
//        rt_printf("present q(deg) : %f, %f, %f, %f, %f, %f\n",
//                  pThis->dataControl->RobotData.present_q[0]*pThis->dataControl->RAD2DEG, pThis->dataControl->RobotData.present_q[1]*pThis->dataControl->RAD2DEG,
//                pThis->dataControl->RobotData.present_q[2]*pThis->dataControl->RAD2DEG, pThis->dataControl->RobotData.present_q[3]*pThis->dataControl->RAD2DEG,
//                pThis->dataControl->RobotData.present_q[4]*pThis->dataControl->RAD2DEG, pThis->dataControl->RobotData.present_q[5]*pThis->dataControl->RAD2DEG);
//        rt_printf("Present Pose : %f, %f, %f, %f, %f, %f\n",
//                  pThis->dataControl->RobotData.present_end_pose[0], pThis->dataControl->RobotData.present_end_pose[1],
//                pThis->dataControl->RobotData.present_end_pose[2], pThis->dataControl->RobotData.present_end_pose[3],
//                pThis->dataControl->RobotData.present_end_pose[4], pThis->dataControl->RobotData.present_end_pose[5]);

        ServerToClientTemp.data_index = pThis->data_indx;

        pThis->dataControl->jointPositionENC2DEG(pThis->dataControl->RobotData.present_joint_position, ServerToClientTemp.presentJointPosition);
        pThis->dataControl->cartesianPoseScaleUp(pThis->dataControl->RobotData.present_end_pose, ServerToClientTemp.presentCartesianPose);
        pThis->dataControl->jointVelocityENC2RPM(pThis->dataControl->RobotData.present_joint_velocity, ServerToClientTemp.presentJointVelocity);
        pThis->dataControl->jointCurrentRAW2mA(pThis->dataControl->RobotData.present_joint_current, ServerToClientTemp.presentJointCurrent);
        pThis->dataControl->cartesianPoseScaleUp(pThis->dataControl->RobotData.desired_end_pose, ServerToClientTemp.desiredCartesianPose);
        pThis->dataControl->jointPositionRAD2DEG(pThis->dataControl->RobotData.desired_q, ServerToClientTemp.desiredJointPosition);
        memcpy(ServerToClientTemp.calculateCartesianPose, pThis->dataControl->RobotData.present_cal_end_pose, sizeof(double)*NUM_DOF);
        memcpy(ServerToClientTemp.presentCartesianVelocity, pThis->dataControl->RobotData.present_end_vel, sizeof(double)*NUM_DOF);
        memcpy(ServerToClientTemp.presentJointTorque, pThis->dataControl->RobotData.present_joint_torque, sizeof(double)*NUM_DOF);
        memcpy(ServerToClientTemp.presentJointResidual, pThis->dataControl->RobotData.present_joint_residual, sizeof(double)*NUM_DOF);

        ServerToClientTemp.time = static_cast<double>((pThis->dataControl->RobotData.time2 - pThis->dataControl->RobotData.time1)/1000000.0);
        ServerToClientTemp.dxl_time = static_cast<double>((pThis->dataControl->RobotData.dxl_time2 - pThis->dataControl->RobotData.dxl_time1)/1000000.0);
        ServerToClientTemp.ik_time = static_cast<double>((pThis->dataControl->RobotData.ik_time2 - pThis->dataControl->RobotData.ik_time1)/1000000.0);

        pThis->dataControl->RobotData.time1 = pThis->dataControl->RobotData.time2;
        pThis->data_indx++;

        if(pThis->tcpServer->isConnected()){
            pThis->dataControl->ServerToClient.push_back(ServerToClientTemp);
        }
        else{
            if(!pThis->dataControl->tablet_mode){
                pThis->robot_task_run = false;
            }
        }

        now = rt_timer_read();
    }
}

void ControlMainCustom::robot_stop(){
    rt_task_suspend(&robot_task);
    printf("Robot RT Task Stop\n");
    rt_task_delete(&robot_task);
    printf("Robot RT Task Delete\n");
}
