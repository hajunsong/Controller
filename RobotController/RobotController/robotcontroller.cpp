#include "robotcontroller.h"

RobotController::RobotController(QObject *parent) : QObject(parent){
    dataControl = new DataControl();
    tcpServer = new NRMKHelper::TcpServer(dataControl);
    module_init = false;
    dxlTimer = new QTimer(this);
    connect(dxlTimer, SIGNAL(timeout()), this, SLOT(dxlTimeout()));
    dxlTimer->setInterval(1000);

    module = new DxlControl();

    data_index = 0;
    module_indx = 0;

    robotArm = new RobotArm(NUM_JOINT, NUM_DOF);

    old_end_pose_update = false;
}

RobotController::~RobotController(){
    qDebug() << "finished program\n";

    delete tcpServer;
    delete dataControl;

    if (robot_thread_run){
        rt_task_suspend(&robot_task);
        QThread::msleep(100);
        rt_task_delete(&robot_task);
        QThread::msleep(100);
        qDebug() << "Robot Control RT Task Stop";
    }
    else{
        qDebug() << "Robot Control RT Thread not running...";
    }

    delete dxlTimer;

    for(uint8_t i = 0; i < NUM_JOINT; i++){
        module->dxl_deinit(i);
    }
    delete module;

    delete robotArm;
}

void RobotController::start()
{
    qDebug() << "Start RobotController";

    tcpServer->setting("192.168.137.100", 5050);
    if (tcpServer->startServer(SOCK_TCP, tcpServer->getPort())){
        qDebug() << "Control server started at IP : " << tcpServer->getIP() << ", Port : " << tcpServer->getPort();
    }

    qDebug() << "Waiting for Control Tool to connect...";
    tcpServer->waitForConnection(0);

//    robot_RT();
//    dxlTimeout();

    module->init();
//    moduleInitFAR();
//    dxlTimer->start();

//    dxl_RT();
}

//void RobotController::dxl_RT(){
//    rt_print_auto_init(1);
//    /* Avoids memory swapping for this program */
//    mlockall(MCL_CURRENT|MCL_FUTURE);

//    int ret = rt_task_create(&dxl_task, "Dynamixel Task", 0, 20, 0);
//    if (ret < 0){
//        printf("Failed to create Dynamixel Task : %s\n", strerror(-ret));
//    }
//    else{
//        printf("Dynamixel RT Task Start\n");

//        rt_task_start(&dxl_task, &dxl_run, this);
//    }
//}

//void RobotController::dxl_run(void *arg){
//    RTIME now, previous;
//    previous = rt_timer_read();
//    RobotController* pThis = static_cast<RobotController*>(arg);

//    rt_task_set_periodic(&pThis->dxl_task, TM_NOW, 1000e6);

//    pThis->module->init();
//    pThis->module_indx = 0;

//    while(true) {
//        rt_task_wait_period(nullptr); //wait for next cycle
//        now = rt_timer_read();
//        if (pThis->dataControl->config_check){
//            switch(MODULE_TYPE){
//            case DataControl::Module::FAR:
//                if (!pThis->module_init){
////                    rt_printf("Start FAR Module Initilization\n");
//                    pThis->moduleInitFAR();
//                }
//                else{
//                    pThis->tcpServer->sendKey('S');
//                }
//                break;
//            case DataControl::Module::SEA:
//                if (!pThis->module_init){
//                    pThis->moduleInitSEA();
//                }
//                else{
//                    pThis->tcpServer->sendKey('S');
//                }
//                break;
//            case DataControl::Module::JS_R8:
//                break;
//            default:
//                break;
//            }
//        }
//        rt_printf("Comm : Time since last turn: %ld.%06ld ms\n",
//                  static_cast<unsigned long>(now - previous) / 1000000,
//                  static_cast<unsigned long>(now - previous) % 1000000);
//        previous = now;
//    }
//}

void RobotController::dxlTimeout(){
//    dxlTimer->stop();
    if (dataControl->config_check){
        switch(MODULE_TYPE){
        case DataControl::Module::FAR:
            if (!module_init){
                moduleInitFAR();
            }
            else{
                tcpServer->sendKey('S');
            }
            break;
        case DataControl::Module::SEA:
            if (!module_init){
                module->init();
                moduleInitSEA();
            }
            else{
                tcpServer->sendKey('S');
            }
            break;
        case DataControl::Module::JS_R8:
            break;
        default:
            break;
        }
    }
//    dxlTimer->start();
}

void RobotController::robot_RT(){

    rt_print_auto_init(1);
    /* Avoids memory swapping for this program */
    mlockall(MCL_CURRENT|MCL_FUTURE);

    int ret = rt_task_create(&robot_task, "Robot Controll Task", 0, 99, 0);
    if (ret < 0){
        qDebug() << "Failed to create Robot Control Task : " << strerror(-ret);
    }
    else{
        qDebug() << "Robot Control RT Task Start";

        dataControl->ClientToServer.opMode = DataControl::OpMode::Wait;

        rt_task_start(&robot_task, &robot_run, this);
    }
}

void RobotController::robot_run(void *arg){
    RTIME now, previous, dxl_now, dxl_previous;
    previous = rt_timer_read();
    RobotController* pThis = static_cast<RobotController*>(arg);

    pThis->robot_thread_run = false;

    rt_task_set_periodic(&pThis->robot_task, TM_NOW, 5e6);

    while(1){
        rt_task_wait_period(nullptr); //wait for next cycle
        if (pThis->tcpServer->isConnected())
        {
            pThis->robot_thread_run = true;
            now = rt_timer_read();

            switch(pThis->dataControl->ClientToServer.opMode){
            case DataControl::OpMode::ServoOnOff:
                pThis->robotServoOn(pThis->dataControl->ClientToServer.subMode);
                break;
            case DataControl::OpMode::Initialize:
//                pThis->robotInitialize();
                break;
            case DataControl::OpMode::Wait:
                dxl_previous = rt_timer_read();
                if (MODULE_TYPE == DataControl::Module::FAR){
                    pThis->module->getGroupSyncReadPresentPosition(pThis->dataControl->RobotData.present_joint_position);
                }
                else if(MODULE_TYPE == DataControl::Module::SEA){
                    pThis->dataControl->RobotData.present_joint_position[0] = pThis->module->getPresentPosition(1);
                }
                dxl_now = rt_timer_read();

                pThis->robotKinematics();
                pThis->robotDynamics();

                if (!pThis->dataControl->cartesian_goal_reach && pThis->dataControl->joint_goal_reach){
                    pThis->dataControl->ClientToServer.opMode = DataControl::OpMode::CartesianMove;
                    goto OpMode_JointMove;
                }
                else if (pThis->dataControl->cartesian_goal_reach && !pThis->dataControl->joint_goal_reach){
                    pThis->dataControl->ClientToServer.opMode = DataControl::OpMode::JointMove;
                    goto OpMode_CartesianMove;
                }

                break;
            case DataControl::OpMode::JointMove:
                OpMode_JointMove:
                if (pThis->dataControl->PathData.type == DataControl::PathDataType::JointPath){
                    memcpy(&pThis->dataControl->PathData.data[0], pThis->dataControl->ClientToServer.desiredJoint, DESIRED_JOINT_LEN*NUM_JOINT);
                }
                pThis->robotJointMove(pThis->dataControl->ClientToServer.subMode, pThis->dataControl->ClientToServer.desiredJoint);
                break;
            case DataControl::OpMode::CartesianMove:
                if (pThis->dataControl->PathData.type == DataControl::PathDataType::CartPath){

                }
                OpMode_CartesianMove:
                pThis->robotCartesianMove(pThis->dataControl->ClientToServer.subMode, pThis->dataControl->ClientToServer.desiredCartesian);
                break;
            default : break;
            }

//            rt_printf("Present Position : %d, %d, %d, %d, %d, %d\n",
//                      pThis->dataControl->RobotData.present_joint_position[0], pThis->dataControl->RobotData.present_joint_position[1],
//                    pThis->dataControl->RobotData.present_joint_position[2], pThis->dataControl->RobotData.present_joint_position[3],
//                    pThis->dataControl->RobotData.present_joint_position[4], pThis->dataControl->RobotData.present_joint_position[5]);
//            rt_printf("Present Pose : %f, %f, %f, %f, %f, %f\n",
//                      pThis->dataControl->ServerToClient.presentCartesianPose[0], pThis->dataControl->ServerToClient.presentCartesianPose[1],
//                    pThis->dataControl->ServerToClient.presentCartesianPose[2], pThis->dataControl->ServerToClient.presentCartesianPose[3],
//                    pThis->dataControl->ServerToClient.presentCartesianPose[4], pThis->dataControl->ServerToClient.presentCartesianPose[5]);
//            rt_printf("Comm : Time since last turn: %ld.%06ld ms\n",
//                      static_cast<unsigned long>(now - previous) / 1000000,
//                      static_cast<unsigned long>(now - previous) % 1000000);
//            rt_printf("Comm : Time since last turn: %ld.%06ld ms\n",
//                      static_cast<unsigned long>(dxl_now - dxl_previous) / 1000000,
//                      static_cast<unsigned long>(dxl_now - dxl_previous) % 1000000);

            pThis->dataControl->ServerToClient.data_index = pThis->data_index;

            pThis->dataControl->jointPositionENC2DEG(pThis->dataControl->RobotData.present_joint_position, pThis->dataControl->ServerToClient.presentJointPosition);
            pThis->dataControl->cartesianPoseScaleUp(pThis->dataControl->RobotData.present_end_pose, pThis->dataControl->ServerToClient.presentCartesianPose);

            previous = now;
            pThis->data_index++;
        }
    }
}

void RobotController::moduleInitSEA(){
    qDebug() << "Start SEA Module Initilization";
//    for(uint8_t i = 0; i < static_cast<uint8_t>(dataControl->ClientToServerInitParam.numJoint); i++){
//        Module_SEA->dxl_init(i+1);
//    }
    while(!module_init){
        int init_result = module->dxl_init(1, false);
        if (init_result){
            module_init = true;
        }
    }
//    dxlTimer->start();
    robot_RT();
}

void RobotController::moduleInitFAR(){
//    qDebug() << "Start FAR Module Initilization";

//    uint8_t module_indx = 0;
    while(!module_init)
    {
        printf("Start module initialize %d\n", module_indx);
        int init_result = module->dxl_init(module_indx, false);
        if (init_result){
            int32_t pos = module->getPresentPosition(module_indx);
            printf("%d axis present position : %d\n", module_indx, pos);
            if (pos != 0)
            {
                module_indx++;
            }
        }
        if (module_indx >= NUM_JOINT){
            module_init = true;
            tcpServer->sendKey('S');
            robot_RT();
        }
//        usleep(10e3);
    }
//    dxlTimer->start();
//    tcpServer->sendKey('S');
//    robot_RT();
}

void RobotController::robotInitialize()
{
    dataControl->DataReset();
    memset(&dataControl->RobotData, 0, sizeof(dataControl->RobotData));
}

void RobotController::robotServoOn(char enable){
    module->setGroupSyncWriteTorqueEnable(static_cast<uint8_t>(enable));

    dataControl->DataReset();
    dataControl->ClientToServer.opMode = DataControl::OpMode::Wait;
}

void RobotController::robotKinematics(){
//    double q[NUM_JOINT] = {0,}, pose[NUM_DOF] = {0,};
    dataControl->jointPositionENC2RAD(dataControl->RobotData.present_joint_position, dataControl->RobotData.present_q);

    robotArm->run_kinematics(dataControl->RobotData.present_q, dataControl->RobotData.present_end_pose);
}

void RobotController::robotDynamics(){

}

void RobotController::robotJointMove(char mode, double desJoint[NUM_JOINT])
{
    switch(mode){
    case DataControl::Motion::JogMotion:
        if (dataControl->joint_goal_reach){
            for(uchar i = 0; i < NUM_JOINT; i++){
                if (desJoint[i] < 0 || desJoint[i] > 0){
                    dataControl->RobotData.command_joint_position[i] = dataControl->RobotData.present_joint_position[i] + static_cast<int32_t>(desJoint[i]/POSITION_UNIT);
                    module->setPosition(dataControl->RobotData.command_joint_position[i], i);
                }
                else{
                    dataControl->RobotData.command_joint_position[i] = dataControl->RobotData.present_joint_position[i];
                }
            }
            dataControl->joint_goal_reach = false;
        }
        else{
            goalReach(dataControl->RobotData.command_joint_position, dataControl->RobotData.present_joint_position, &dataControl->joint_goal_reach);
        }
        break;
    case DataControl::Motion::JointMotion:
        if (dataControl->joint_goal_reach){
            for(uchar i = 0; i < NUM_JOINT; i++){
                if (desJoint[i] < 0 || desJoint[i] > 0){
                    dataControl->RobotData.command_joint_position[i] = static_cast<int32_t>(desJoint[i]/POSITION_UNIT);
                    module->setPosition(dataControl->RobotData.command_joint_position[i], i);
                }
                else{
                    dataControl->RobotData.command_joint_position[i] = dataControl->RobotData.present_joint_position[i];
                }
            }
            dataControl->joint_goal_reach = false;
        }
        else{
            goalReach(dataControl->RobotData.command_joint_position, dataControl->RobotData.present_joint_position, &dataControl->joint_goal_reach);
        }
        break;
    default :
        break;
    }

    dataControl->ClientToServer.opMode = DataControl::OpMode::Wait;
}

void RobotController::robotCartesianMove(char mode, double desCartesian[NUM_DOF])
{
    if (!old_end_pose_update){
        memcpy(dataControl->RobotData.old_desired_end_pose, dataControl->RobotData.present_end_pose, CARTESIAN_POSE_LEN*NUM_JOINT);
        old_end_pose_update = true;
    }

    double des_pose[NUM_DOF];
    dataControl->cartesianPoseScaleDown(desCartesian, des_pose);
    switch(mode){
    case DataControl::Motion::CartesianJogMotion:
        if (dataControl->cartesian_goal_reach){
            for(uchar i = 0; i < NUM_DOF; i++){
                if (desCartesian[i] < 0 || desCartesian[i] > 0){
                    dataControl->RobotData.desired_end_pose[i] = dataControl->RobotData.old_desired_end_pose[i] + des_pose[i];
                }
                else{
                    dataControl->RobotData.desired_end_pose[i] = dataControl->RobotData.old_desired_end_pose[i];
                }
            }

            robotArm->run_inverse_kinematics(dataControl->RobotData.present_q, dataControl->RobotData.desired_end_pose,
                                             dataControl->RobotData.desired_q, dataControl->RobotData.present_end_pose);

            memcpy(dataControl->RobotData.old_desired_end_pose, dataControl->RobotData.desired_end_pose, CARTESIAN_POSE_LEN*NUM_JOINT);

            dataControl->jointPositionRAD2ENC(dataControl->RobotData.desired_q, dataControl->RobotData.command_joint_position);
            dataControl->cartesian_goal_reach = false;
        }
        else{
            goalReach(dataControl->RobotData.command_joint_position, dataControl->RobotData.present_joint_position, &dataControl->cartesian_goal_reach);
        }

        module->setGroupSyncWriteGoalPosition(dataControl->RobotData.command_joint_position);

        break;
    case DataControl::Motion::CartesianMotion:
        if (dataControl->cartesian_goal_reach){
            for(uchar i = 0; i < NUM_DOF; i++){
                if (desCartesian[i] < 0 || desCartesian[i] > 0){
                    dataControl->RobotData.desired_end_pose[i] = des_pose[i];
                }
                else{
                    dataControl->RobotData.desired_end_pose[i] = dataControl->RobotData.old_desired_end_pose[i];
                }
            }

            robotArm->run_inverse_kinematics(dataControl->RobotData.present_q, dataControl->RobotData.desired_end_pose,
                                             dataControl->RobotData.desired_q, dataControl->RobotData.present_end_pose);

            memcpy(dataControl->RobotData.old_desired_end_pose, dataControl->RobotData.desired_end_pose, CARTESIAN_POSE_LEN*NUM_JOINT);

            dataControl->jointPositionRAD2ENC(dataControl->RobotData.desired_q, dataControl->RobotData.command_joint_position);
            dataControl->cartesian_goal_reach = false;
        }
        else{
            goalReach(dataControl->RobotData.command_joint_position, dataControl->RobotData.present_joint_position, &dataControl->cartesian_goal_reach);
        }

        module->setGroupSyncWriteGoalPosition(dataControl->RobotData.command_joint_position);
        break;
    default :
        break;
    }

    dataControl->ClientToServer.opMode = DataControl::OpMode::Wait;
}

void RobotController::goalReach(int32_t desired_position[], int32_t present_position[], bool *goal_reach)
{
    int32_t epsilon = 50;
    int32_t value = 0;
    int indx = 0;
    for(int i = 0; i < NUM_JOINT; i++){
        value = abs(desired_position[i] - present_position[i]);
        if (value <= epsilon){
            indx++;
        }
    }

    if (indx == NUM_JOINT){
        *goal_reach = true;
    }
    else{
        *goal_reach = false;
    }
}
