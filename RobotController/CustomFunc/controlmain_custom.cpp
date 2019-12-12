#include "controlmain_custom.h"
#include "ControlMain/controlmain.h"

ControlMainCustom::ControlMainCustom(){
}

ControlMainCustom::~ControlMainCustom(){
}

void ControlMainCustom::robot_run(void *arg)
{
    ControlMain* pThis = static_cast<ControlMain*>(arg);
    pThis->dataControl->RobotData.time1 = static_cast<unsigned long>(rt_timer_read());

    pThis->robot_thread_run = false;

    rt_task_set_periodic(&pThis->robot_task, TM_NOW, 5e6);

    pThis->robot_thread_run = true;

    while(pThis->robot_thread_run){
        DataControl::StructServerToClient ServerToClientTemp;
        rt_task_wait_period(nullptr); //wait for next cycle
        pThis->dataControl->RobotData.time2 = static_cast<unsigned long>(rt_timer_read());

        switch(pThis->dataControl->ClientToServer.opMode){
            case DataControl::OpMode::ServoOnOff:
                pThis->robotServoOn(pThis->dataControl->ClientToServer.subMode);
                break;
            case DataControl::OpMode::Initialize:
                pThis->robotInitialize();
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
                pThis->robotPathGenerate();
                break;
            case DataControl::OpMode::ReadyMode:
                pThis->robotReady();
                break;
            case DataControl::OpMode::RunMode:
                pThis->robotRun();
                break;
            default : break;
        }

        pThis->dataControl->RobotData.dxl_time1 = static_cast<unsigned long>(rt_timer_read());
        if (MODULE_TYPE == DataControl::Module::FAR_V1){
            pThis->module->getGroupSyncReadIndirectAddress(pThis->dataControl->RobotData.present_joint_position, pThis->dataControl->RobotData.present_joint_velocity,
                                                           pThis->dataControl->RobotData.present_joint_current, NUM_JOINT);
        }
		else if(MODULE_TYPE == DataControl::Module::FAR_V2){
			pThis->module->getGroupSyncReadIndirectAddress(pThis->dataControl->RobotData.present_joint_position, pThis->dataControl->RobotData.present_joint_velocity,
														   pThis->dataControl->RobotData.present_joint_current, NUM_JOINT);
		}
        pThis->dataControl->RobotData.dxl_time2 = static_cast<unsigned long>(rt_timer_read());

        pThis->robotKinematics();
        pThis->robotDynamics();

        ServerToClientTemp.data_index = pThis->data_indx;

//        rt_printf("data index : %d\n", pThis->dataControl->ServerToClient.data_index);

        pThis->dataControl->jointPositionENC2DEG(pThis->dataControl->RobotData.present_joint_position, ServerToClientTemp.presentJointPosition);
        pThis->dataControl->cartesianPoseScaleUp(pThis->dataControl->RobotData.present_end_pose, ServerToClientTemp.presentCartesianPose);
        pThis->dataControl->jointVelocityENC2RPM(pThis->dataControl->RobotData.present_joint_velocity, ServerToClientTemp.presentJointVelocity);
        pThis->dataControl->jointCurrentRAW2mA(pThis->dataControl->RobotData.present_joint_current, ServerToClientTemp.presentJointCurrent);
        memcpy(ServerToClientTemp.calculateCartesianPose, pThis->dataControl->RobotData.present_cal_end_pose, sizeof(double)*NUM_DOF);
        memcpy(ServerToClientTemp.presentCartesianVelocity, pThis->dataControl->RobotData.present_end_vel, sizeof(double)*NUM_DOF);

//        rt_printf("Present Joint Position : %f, %f, %f, %f, %f, %d\n",
//                  ServerToClientTemp.presentJointPosition[0], ServerToClientTemp.presentJointPosition[1],
//                ServerToClientTemp.presentJointPosition[2], ServerToClientTemp.presentJointPosition[3],
//                ServerToClientTemp.presentJointPosition[4], ServerToClientTemp.presentJointPosition[5]);
//        rt_printf("Cartesian Pose : %f, %f, %f, %f, %f, %f\n",
//                  ServerToClientTemp.presentCartesianPose[0], ServerToClientTemp.presentCartesianPose[1],
//                ServerToClientTemp.presentCartesianPose[2], ServerToClientTemp.presentCartesianPose[3],
//                ServerToClientTemp.presentCartesianPose[4], ServerToClientTemp.presentCartesianPose[5]);
//        rt_printf("Present Joint Velocity : %f, %f, %f, %f, %f, %f\n",
//                  ServerToClientTemp.presentJointVelocity[0], ServerToClientTemp.presentJointVelocity[1],
//                ServerToClientTemp.presentJointVelocity[2], ServerToClientTemp.presentJointVelocity[3],
//                ServerToClientTemp.presentJointVelocity[4], ServerToClientTemp.presentJointVelocity[5]);
//        rt_printf("Present Joint Current : %f, %f, %f, %f, %f, %f\n",
//                  ServerToClientTemp.presentJointCurrent[0], ServerToClientTemp.presentJointCurrent[1],
//                ServerToClientTemp.presentJointCurrent[2], ServerToClientTemp.presentJointCurrent[3],
//                ServerToClientTemp.presentJointCurrent[4], ServerToClientTemp.presentJointCurrent[5]);
//        rt_printf("Calculate Cartesian Pose : %f, %f, %f, %f, %f, %f\n",
//                  ServerToClientTemp.calculateCartesianPose[0], ServerToClientTemp.calculateCartesianPose[1],
//                ServerToClientTemp.calculateCartesianPose[2], ServerToClientTemp.calculateCartesianPose[3],
//                ServerToClientTemp.calculateCartesianPose[4], ServerToClientTemp.calculateCartesianPose[5]);
//        rt_printf("Present Cartesian Velocity : %f, %f, %f, %f, %f, %f\n",
//                  ServerToClientTemp.presentCartesianVelocity[0], ServerToClientTemp.presentCartesianVelocity[1],
//                ServerToClientTemp.presentCartesianVelocity[2], ServerToClientTemp.presentCartesianVelocity[3],
//                ServerToClientTemp.presentCartesianVelocity[4], ServerToClientTemp.presentCartesianVelocity[5]);


        ServerToClientTemp.time = static_cast<double>((pThis->dataControl->RobotData.time2 - pThis->dataControl->RobotData.time1)/1000000.0);
        ServerToClientTemp.dxl_time = static_cast<double>((pThis->dataControl->RobotData.dxl_time2 - pThis->dataControl->RobotData.dxl_time1)/1000000.0);
        ServerToClientTemp.ik_time = static_cast<double>((pThis->dataControl->RobotData.ik_time2 - pThis->dataControl->RobotData.ik_time1)/1000000.0);

        pThis->dataControl->RobotData.time1 = pThis->dataControl->RobotData.time2;
        pThis->data_indx++;

        pThis->dataControl->ServerToClient.push_back(ServerToClientTemp);
//        pThis->tcpServer->data_size = static_cast<int8_t>(pThis->dataControl->ServerToClient.size());

        if (!pThis->tcpServer->isConnected()){
            pThis->robot_thread_run = false;
            emit pThis->disconnectClientSignal();
        }

//        if (pThis->tcpServer->data_size >= DATA_MAX_INDX){
//            pThis->data_indx = 0;
//            pThis->dataControl->ServerToClient.clear();
//        }
    }
}

void ControlMainCustom::robot_stop(){
    rt_task_suspend(&robot_task);
    printf("Robot RT Task Stop\n");
    rt_task_delete(&robot_task);
    printf("Robot RT Task Delete\n");
}
