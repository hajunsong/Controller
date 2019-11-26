#include "controlmain_custom.h"
#include "ControlMain/controlmain.h"

ControlMainCustom::ControlMainCustom(){
}

ControlMainCustom::~ControlMainCustom(){
    robot_stop();
}

void ControlMainCustom::robot_run(void *arg)
{
    ControlMain* pThis = static_cast<ControlMain*>(arg);
    pThis->dataControl->RobotData.time1 = static_cast<unsigned long>(rt_timer_read());

    pThis->robot_thread_run = false;

    rt_task_set_periodic(&pThis->robot_task, TM_NOW, 5e6);

    pThis->robot_thread_run = true;

    while(pThis->robot_thread_run){
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
        pThis->dataControl->RobotData.dxl_time2 = static_cast<unsigned long>(rt_timer_read());

        pThis->robotKinematics();
        pThis->robotDynamics();

        pThis->dataControl->ServerToClient.data_index = pThis->data_indx;

        pThis->dataControl->jointPositionENC2DEG(pThis->dataControl->RobotData.present_joint_position, pThis->dataControl->ServerToClient.presentJointPosition[pThis->data_indx]);
        pThis->dataControl->cartesianPoseScaleUp(pThis->dataControl->RobotData.present_end_pose, pThis->dataControl->ServerToClient.presentCartesianPose[pThis->data_indx]);
        pThis->dataControl->jointVelocityENC2RPM(pThis->dataControl->RobotData.present_joint_velocity, pThis->dataControl->ServerToClient.presentJointVelocity[pThis->data_indx]);
        pThis->dataControl->jointCurrentRAW2mA(pThis->dataControl->RobotData.present_joint_current, pThis->dataControl->ServerToClient.presentJointCurrent[pThis->data_indx]);

        pThis->dataControl->ServerToClient.time = static_cast<double>((pThis->dataControl->RobotData.time2 - pThis->dataControl->RobotData.time1)/1000000.0);
        pThis->dataControl->ServerToClient.dxl_time = static_cast<double>((pThis->dataControl->RobotData.dxl_time2 - pThis->dataControl->RobotData.dxl_time1)/1000000.0);
        pThis->dataControl->ServerToClient.ik_time = static_cast<double>((pThis->dataControl->RobotData.ik_time2 - pThis->dataControl->RobotData.ik_time1)/1000000.0);

        pThis->dataControl->RobotData.time1 = pThis->dataControl->RobotData.time2;
        pThis->data_indx++;

        if (!pThis->tcpServer->isConnected()){
            emit pThis->disconnectClientSignal();
        }

        if (pThis->data_indx >= 200){
            pThis->data_indx = 0;
        }
    }
}

void ControlMainCustom::robot_stop(){
    rt_task_suspend(&robot_task);
    printf("Robot RT Task Stop\n");
    rt_task_delete(&robot_task);
    printf("Robot Comm RT Task Delet\n");
}
