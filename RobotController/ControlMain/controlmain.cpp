#include "controlmain.h"

ControlMain::ControlMain(QObject *parent) : QObject(parent)
{
    dataControl = new DataControl();
    tcpServer = new NRMKHelper::TcpServer(dataControl);

    module_init = false;
    module = new DxlControl();

    data_indx = 0;
    module_indx = 0;

    robotArm = new RobotArm(NUM_JOINT, NUM_DOF);

    old_end_pose_update = false;

    dxlTimer = new QTimer();
    QObject::connect(dxlTimer, SIGNAL(timeout()), this, SLOT(dxlTimeout()));
    dxlTimer->setInterval(100);

    ready_pose = false;
}

ControlMain::~ControlMain()
{
    printf("finished program\n");

    if (dxlTimer->isActive()){
        dxlTimer->stop();
    }
    delete dxlTimer;
    delete tcpServer;
    delete dataControl;
    delete robotArm;

    if (robot_thread_run){
        robot_thread_run = false;
        usleep(1000e3);
        rt_task_suspend(&robot_task);
        usleep(1000e3);
        rt_task_delete(&robot_task);
        usleep(1000e3);
        printf("Robot Control RT Task Stop\n");
    }
    else{
        printf("Robot Control RT Thread not running...\n");
    }

    if (module_init){
        for(uint8_t i = 0; i < NUM_JOINT; i++){
            module->dxl_deinit(i);
        }
    }
    delete module;
    printf("Complete destructure ControlMain class\n");
}

void ControlMain::start()
{
    printf("Start RobotController\n");

    tcpServer->setting("192.168.137.100", 5050);
    if (tcpServer->startServer(SOCK_TCP, tcpServer->getPort())){
        printf("Control server started at IP : %s, Port : %d\n", tcpServer->getIP().toStdString().c_str(), tcpServer->getPort());
    }

    printf("Waiting for Control Tool to connect...\n");
    tcpServer->waitForConnection(0);

    dxlTimer->start();
}

void ControlMain::dxlTimeout(){
    dxlTimer->stop();
    if (dataControl->config_check){
        switch(MODULE_TYPE){
        case DataControl::Module::FAR:
            if (!module_init){
                printf("Start FAR Module Initilization\n");
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
    dxlTimer->start();
}

void ControlMain::robot_RT()
{
    rt_print_auto_init(1);
    /* Avoids memory swapping for this program */
    mlockall(MCL_CURRENT|MCL_FUTURE);

    int ret = rt_task_create(&robot_task, "Robot Controll Task", 0, 99, 0);
    if (ret < 0){
        printf("Failed to create Robot Control Task %s\n", strerror(-ret));
    }
    else{
        printf("Robot Control RT Task Start\n");

        dataControl->ClientToServer.opMode = DataControl::OpMode::Wait;

        rt_task_start(&robot_task, &robot_run, this);
    }
}

void ControlMain::robot_run(void *arg)
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
            if (!pThis->dataControl->cartesian_goal_reach){
                pThis->dataControl->ClientToServer.opMode = DataControl::OpMode::CartesianMove;
            }
            break;
        case DataControl::OpMode::JointMove:
            pThis->robotJointMove(pThis->dataControl->ClientToServer.subMode, pThis->dataControl->ClientToServer.desiredJoint);
            break;
        case DataControl::OpMode::CartesianMove:
            pThis->robotCartesianMove(pThis->dataControl->ClientToServer.subMode, pThis->dataControl->ClientToServer.desiredCartesian);
            break;
        case DataControl::OpMode::RunMode:
            switch(pThis->dataControl->PathData.type){
            case DataControl::PathDataType::Save1:
            {
                printf("path_data_indx : %d\n", pThis->dataControl->PathData.path_data_indx);
                double path[6];
                for(uint i = 0; i < 6; i++){
                    path[i] = pThis->dataControl->PathData.pathDataPick[pThis->dataControl->PathData.path_data_indx][i+1];
                }

                pThis->dataControl->jointPositionRAD2ENC(path, pThis->dataControl->RobotData.command_joint_position);

                printf("%d, %d, %d, %d, %d, %d\n", pThis->dataControl->RobotData.command_joint_position[0], pThis->dataControl->RobotData.command_joint_position[1],
                        pThis->dataControl->RobotData.command_joint_position[2], pThis->dataControl->RobotData.command_joint_position[3],
                        pThis->dataControl->RobotData.command_joint_position[4], pThis->dataControl->RobotData.command_joint_position[5]);

                pThis->module->setGroupSyncWriteGoalPosition(pThis->dataControl->RobotData.command_joint_position);

                pThis->dataControl->PathData.path_data_indx += 1;

                if (pThis->dataControl->PathData.path_data_indx >= pThis->dataControl->PathData.row){
                    if(pThis->dataControl->PathData.repeat == -1){
                        pThis->dataControl->PathData.path_data_indx = 0;
                    }
                    else{
                        pThis->dataControl->ClientToServer.opMode = DataControl::OpMode::Wait;
                    }
                }
                break;
            }
            case DataControl::PathDataType::Save2:
            {
                if (!pThis->ready_pose){
                    pThis->dataControl->jointPositionDEG2ENC(pThis->dataControl->ready_pose, pThis->dataControl->RobotData.command_joint_position);

                    printf("ready command joint : %d, %d, %d, %d, %d, %d\n", pThis->dataControl->RobotData.command_joint_position[0], pThis->dataControl->RobotData.command_joint_position[1],
                            pThis->dataControl->RobotData.command_joint_position[2], pThis->dataControl->RobotData.command_joint_position[3],
                            pThis->dataControl->RobotData.command_joint_position[4], pThis->dataControl->RobotData.command_joint_position[5]);

                    pThis->module->setGroupSyncWriteGoalPosition(pThis->dataControl->RobotData.command_joint_position);
                    pThis->ready_pose = true;
                }
                else{
                    printf("path_data_indx : %d\n", pThis->dataControl->PathData.path_data_indx);
                    for(uint i = 0; i < 6; i++){
                        pThis->dataControl->RobotData.desired_end_pose[i] = pThis->dataControl->PathData.pathDataPick[pThis->dataControl->PathData.path_data_indx][i+7];
                    }

                    pThis->dataControl->RobotData.ik_time1 = static_cast<unsigned long>(rt_timer_read());
                    pThis->robotArm->run_inverse_kinematics(pThis->dataControl->RobotData.present_q, pThis->dataControl->RobotData.desired_end_pose,
                                                            pThis->dataControl->RobotData.desired_q, pThis->dataControl->RobotData.present_end_pose);
                    pThis->dataControl->RobotData.ik_time2 = static_cast<unsigned long>(rt_timer_read());

                    pThis->dataControl->cartesianPoseScaleUp(pThis->dataControl->RobotData.present_end_pose, pThis->dataControl->ServerToClient.calculateCartesianPose);

                    pThis->dataControl->jointPositionRAD2ENC(pThis->dataControl->RobotData.desired_q, pThis->dataControl->RobotData.command_joint_position);

                    printf("command joint : %d, %d, %d, %d, %d, %d\n", pThis->dataControl->RobotData.command_joint_position[0], pThis->dataControl->RobotData.command_joint_position[1],
                            pThis->dataControl->RobotData.command_joint_position[2], pThis->dataControl->RobotData.command_joint_position[3],
                            pThis->dataControl->RobotData.command_joint_position[4], pThis->dataControl->RobotData.command_joint_position[5]);

                    pThis->module->setGroupSyncWriteGoalPosition(pThis->dataControl->RobotData.command_joint_position);

                    pThis->goalReach(pThis->dataControl->RobotData.desired_end_pose, pThis->dataControl->RobotData.present_end_pose, &pThis->dataControl->cartesian_goal_reach);

                    if (pThis->dataControl->cartesian_goal_reach){
                        pThis->dataControl->PathData.path_data_indx += 1;
                        if (pThis->dataControl->PathData.path_data_indx == 697){
                            pThis->dataControl->PathData.path_data_indx = 1500;
                        }
                    }

                    if (pThis->dataControl->PathData.path_data_indx >= pThis->dataControl->PathData.row){
                        if(pThis->dataControl->PathData.repeat == -1){
                            pThis->dataControl->PathData.path_data_indx = 0;
                        }
                        else{
                            pThis->dataControl->ClientToServer.opMode = DataControl::OpMode::Wait;
                            pThis->ready_pose = false;
                        }
                    }
                }
                break;
            }
            case DataControl::PathDataType::Save3:
            {
                printf("path_data_indx : %d\n", pThis->dataControl->PathData.path_data_indx);
                double path[6];
                for(uint i = 0; i < 6; i++){
                    path[i] = pThis->dataControl->PathData.pathDataRect[pThis->dataControl->PathData.path_data_indx][i+2];
                }

                pThis->dataControl->jointPositionRAD2ENC(path, pThis->dataControl->RobotData.command_joint_position);

                printf("%d, %d, %d, %d, %d, %d\n", pThis->dataControl->RobotData.command_joint_position[0], pThis->dataControl->RobotData.command_joint_position[1],
                        pThis->dataControl->RobotData.command_joint_position[2], pThis->dataControl->RobotData.command_joint_position[3],
                        pThis->dataControl->RobotData.command_joint_position[4], pThis->dataControl->RobotData.command_joint_position[5]);

                pThis->module->setGroupSyncWriteGoalPosition(pThis->dataControl->RobotData.command_joint_position);

                pThis->dataControl->PathData.path_data_indx += 1;

                if (pThis->dataControl->PathData.path_data_indx >= pThis->dataControl->PathData.row){
                    if(pThis->dataControl->PathData.repeat == -1){
                        pThis->dataControl->PathData.path_data_indx = 0;
                    }
                    else{
                        pThis->dataControl->ClientToServer.opMode = DataControl::OpMode::Wait;
                    }
                }
                break;
            }
            case DataControl::PathDataType::Save4:
            {
                if (!pThis->ready_pose){
                    double path[6];
                    for(uint i = 0; i < 6; i++){
                        path[i] = pThis->dataControl->PathData.pathDataRect[pThis->dataControl->PathData.path_data_indx][i+2];
                    }
                    pThis->dataControl->jointPositionRAD2ENC(path, pThis->dataControl->RobotData.command_joint_position);

                    printf("ready command joint : %d, %d, %d, %d, %d, %d\n", pThis->dataControl->RobotData.command_joint_position[0], pThis->dataControl->RobotData.command_joint_position[1],
                            pThis->dataControl->RobotData.command_joint_position[2], pThis->dataControl->RobotData.command_joint_position[3],
                            pThis->dataControl->RobotData.command_joint_position[4], pThis->dataControl->RobotData.command_joint_position[5]);

                    pThis->module->setGroupSyncWriteGoalPosition(pThis->dataControl->RobotData.command_joint_position);
                    pThis->ready_pose = true;
                }
                else{
                    printf("path_data_indx : %d\n", pThis->dataControl->PathData.path_data_indx);
                    for(uint i = 0; i < 6; i++){
                        pThis->dataControl->RobotData.desired_end_pose[i] = pThis->dataControl->PathData.pathDataRect[pThis->dataControl->PathData.path_data_indx][i+8];
                    }

                    pThis->dataControl->RobotData.ik_time1 = static_cast<unsigned long>(rt_timer_read());
                    pThis->robotArm->run_inverse_kinematics(pThis->dataControl->RobotData.present_q, pThis->dataControl->RobotData.desired_end_pose,
                                                            pThis->dataControl->RobotData.desired_q, pThis->dataControl->RobotData.present_end_pose);
                    pThis->dataControl->RobotData.ik_time2 = static_cast<unsigned long>(rt_timer_read());

                    pThis->dataControl->cartesianPoseScaleUp(pThis->dataControl->RobotData.present_end_pose, pThis->dataControl->ServerToClient.calculateCartesianPose);

                    pThis->dataControl->jointPositionRAD2ENC(pThis->dataControl->RobotData.desired_q, pThis->dataControl->RobotData.command_joint_position);

                    printf("command joint : %d, %d, %d, %d, %d, %d\n", pThis->dataControl->RobotData.command_joint_position[0], pThis->dataControl->RobotData.command_joint_position[1],
                            pThis->dataControl->RobotData.command_joint_position[2], pThis->dataControl->RobotData.command_joint_position[3],
                            pThis->dataControl->RobotData.command_joint_position[4], pThis->dataControl->RobotData.command_joint_position[5]);

                    pThis->module->setGroupSyncWriteGoalPosition(pThis->dataControl->RobotData.command_joint_position);

                    pThis->goalReach(pThis->dataControl->RobotData.desired_end_pose, pThis->dataControl->RobotData.present_end_pose, &pThis->dataControl->cartesian_goal_reach);

                    if (pThis->dataControl->cartesian_goal_reach){
                        pThis->dataControl->PathData.path_data_indx += 1;
                    }

                    if (pThis->dataControl->PathData.path_data_indx >= pThis->dataControl->PathData.row){

                        if(pThis->dataControl->PathData.repeat == -1){
                            pThis->dataControl->PathData.path_data_indx = 0;
                        }
                        else{
                            pThis->dataControl->ClientToServer.opMode = DataControl::OpMode::Wait;
                            pThis->ready_pose = false;
                        }
                    }
                }
                break;
            }

            }
            break;
        default : break;
        }

        pThis->dataControl->RobotData.dxl_time1 = static_cast<unsigned long>(rt_timer_read());
        if (MODULE_TYPE == DataControl::Module::FAR){
            pThis->module->getGroupSyncReadPresentPosition(pThis->dataControl->RobotData.present_joint_position);
        }
        else if(MODULE_TYPE == DataControl::Module::SEA){
            pThis->module->getPresentPosition(1, &pThis->dataControl->RobotData.present_joint_position[0]);
        }
        pThis->dataControl->RobotData.dxl_time2 = static_cast<unsigned long>(rt_timer_read());

        pThis->robotKinematics();
        pThis->robotDynamics();

//        rt_printf("Present Position : %d, %d, %d, %d, %d, %d\n",
//                  pThis->dataControl->RobotData.present_joint_position[0], pThis->dataControl->RobotData.present_joint_position[1],
//                pThis->dataControl->RobotData.present_joint_position[2], pThis->dataControl->RobotData.present_joint_position[3],
//                pThis->dataControl->RobotData.present_joint_position[4], pThis->dataControl->RobotData.present_joint_position[5]);
//        rt_printf("Present Pose : %f, %f, %f, %f, %f, %f\n",
//                  pThis->dataControl->ServerToClient.presentCartesianPose[0], pThis->dataControl->ServerToClient.presentCartesianPose[1],
//                pThis->dataControl->ServerToClient.presentCartesianPose[2], pThis->dataControl->ServerToClient.presentCartesianPose[3],
//                pThis->dataControl->ServerToClient.presentCartesianPose[4], pThis->dataControl->ServerToClient.presentCartesianPose[5]);
//        rt_printf("Robot : Time since last turn: %ld.%06ld ms\n",
//                  (pThis->dataControl->RobotData.time2 - pThis->dataControl->RobotData.time1) / 1000000,
//                  (pThis->dataControl->RobotData.time2 - pThis->dataControl->RobotData.time1) % 1000000);
//        rt_printf("Dxl : Time since last turn: %ld.%06ld ms\n",
//                  (pThis->dataControl->RobotData.dxl_time2 - pThis->dataControl->RobotData.dxl_time1) / 1000000,
//                  (pThis->dataControl->RobotData.dxl_time2 - pThis->dataControl->RobotData.dxl_time1) % 1000000);

        pThis->dataControl->ServerToClient.data_index = pThis->data_indx;

        pThis->dataControl->jointPositionENC2DEG(pThis->dataControl->RobotData.present_joint_position, pThis->dataControl->ServerToClient.presentJointPosition);
        pThis->dataControl->cartesianPoseScaleUp(pThis->dataControl->RobotData.present_end_pose, pThis->dataControl->ServerToClient.presentCartesianPose);

        pThis->dataControl->ServerToClient.time = static_cast<double>((pThis->dataControl->RobotData.time2 - pThis->dataControl->RobotData.time1)/1000000.0);
        pThis->dataControl->RobotData.time1 = pThis->dataControl->RobotData.time2;
        pThis->data_indx++;
    }
}

void ControlMain::moduleInitSEA()
{
    printf("Start SEA Module Initilization\n");
    //    while(!module_init){
    //        int init_result = module->dxl_init(1, false);
    //        if (init_result){
    //            module_init = true;
    //        }
    //    }
    tcpServer->sendKey('S');
    robot_RT();
}

void ControlMain::moduleInitFAR()
{
    module->init();

    while(!module_init)
    {
        printf("Start module initialize %d\n", module_indx);
        int init_result = module->dxl_init(module_indx, false);
        if (init_result){
            int32_t pos = 0;
            module->getPresentPosition(module_indx, &pos);
            printf("%d axis present position : %d\n", module_indx, pos);
            if (pos != 0)
            {
                module_indx++;
            }
        }
        if (module_indx >= NUM_JOINT){
            module_init = true;
        }
    }
    dxlTimer->stop();
    tcpServer->sendKey('S');
    robot_RT();
}


void ControlMain::robotInitialize()
{
    dataControl->DataReset();
    memset(&dataControl->RobotData, 0, sizeof(dataControl->RobotData));
}

void ControlMain::robotServoOn(char enable){
    module->setGroupSyncWriteTorqueEnable(static_cast<uint8_t>(enable));

    dataControl->DataReset();
    dataControl->ClientToServer.opMode = DataControl::OpMode::Wait;
}

void ControlMain::robotKinematics(){
    dataControl->jointPositionENC2RAD(dataControl->RobotData.present_joint_position, dataControl->RobotData.present_q);

    robotArm->run_kinematics(dataControl->RobotData.present_q, dataControl->RobotData.present_end_pose);
}

void ControlMain::robotDynamics(){

}

void ControlMain::robotJointMove(char mode, double desJoint[NUM_JOINT])
{
    switch(mode){
    case DataControl::Motion::JogMotion:
        for(unsigned char i = 0; i < NUM_JOINT; i++){
            if (desJoint[i] < 0 || desJoint[i] > 0){
                dataControl->RobotData.command_joint_position[i] = dataControl->RobotData.present_joint_position[i] + static_cast<int32_t>(desJoint[i]/POSITION_UNIT);
//                module->setGoalPosition(dataControl->RobotData.command_joint_position[i], i);
            }
            else{
                dataControl->RobotData.command_joint_position[i] = dataControl->RobotData.present_joint_position[i];
            }
        }
        module->setGroupSyncWriteGoalPosition(dataControl->RobotData.command_joint_position);
        break;
    case DataControl::Motion::JointMotion:
        for(unsigned char i = 0; i < NUM_JOINT; i++){
            if (desJoint[i] < 0 || desJoint[i] > 0){
                dataControl->RobotData.command_joint_position[i] = static_cast<int32_t>(desJoint[i]/POSITION_UNIT);
//                module->setGoalPosition(dataControl->RobotData.command_joint_position[i], i);
            }
            else{
                dataControl->RobotData.command_joint_position[i] = dataControl->RobotData.present_joint_position[i];
            }
        }
        module->setGroupSyncWriteGoalPosition(dataControl->RobotData.command_joint_position);
        break;
    default :
        break;
    }

    dataControl->ClientToServer.opMode = DataControl::OpMode::Wait;
}

void ControlMain::robotCartesianMove(char mode, double desCartesian[NUM_DOF])
{
    if (!old_end_pose_update){
        memcpy(dataControl->RobotData.old_desired_end_pose, dataControl->RobotData.present_end_pose, CARTESIAN_POSE_LEN*NUM_JOINT);
        old_end_pose_update = true;
    }

    double des_pose[NUM_DOF];
    dataControl->cartesianPoseScaleDown(desCartesian, des_pose);
    switch(mode){
    case DataControl::Motion::CartesianJogMotion:
        if (!dataControl->cartesian_goal_reach){
            for(unsigned char i = 0; i < NUM_DOF; i++){
                if (desCartesian[i] < 0 || desCartesian[i] > 0){
                    dataControl->RobotData.desired_end_pose[i] = dataControl->RobotData.old_desired_end_pose[i] + des_pose[i];
                }
                else{
                    dataControl->RobotData.desired_end_pose[i] = dataControl->RobotData.old_desired_end_pose[i];
                }
            }

            dataControl->RobotData.ik_time1 = static_cast<unsigned long>(rt_timer_read());
            robotArm->run_inverse_kinematics(dataControl->RobotData.present_q, dataControl->RobotData.desired_end_pose,
                                             dataControl->RobotData.desired_q, dataControl->RobotData.present_end_pose);
            dataControl->RobotData.ik_time2 = static_cast<unsigned long>(rt_timer_read());

            memcpy(dataControl->RobotData.old_desired_end_pose, dataControl->RobotData.desired_end_pose, CARTESIAN_POSE_LEN*NUM_DOF);
            dataControl->cartesianPoseScaleUp(dataControl->RobotData.present_end_pose, dataControl->ServerToClient.calculateCartesianPose);

            dataControl->jointPositionRAD2ENC(dataControl->RobotData.desired_q, dataControl->RobotData.command_joint_position);

            module->setGroupSyncWriteGoalPosition(dataControl->RobotData.command_joint_position);

            goalReach(dataControl->RobotData.desired_end_pose, dataControl->RobotData.present_end_pose, &dataControl->cartesian_goal_reach);
        }
        else{
            module->setGroupSyncWriteGoalPosition(dataControl->RobotData.command_joint_position);

            goalReach(dataControl->RobotData.desired_end_pose, dataControl->RobotData.present_end_pose, &dataControl->cartesian_goal_reach);
        }

        break;
    case DataControl::Motion::CartesianMotion:
        if (!dataControl->cartesian_goal_reach){
            for(unsigned char i = 0; i < NUM_DOF; i++){
                if (desCartesian[i] < 0 || desCartesian[i] > 0){
                    dataControl->RobotData.desired_end_pose[i] = des_pose[i];
                }
                else{
                    dataControl->RobotData.desired_end_pose[i] = dataControl->RobotData.old_desired_end_pose[i];
                }
            }

            dataControl->RobotData.ik_time1 = static_cast<unsigned long>(rt_timer_read());
            robotArm->run_inverse_kinematics(dataControl->RobotData.present_q, dataControl->RobotData.desired_end_pose,
                                             dataControl->RobotData.desired_q, dataControl->RobotData.present_end_pose);
            dataControl->RobotData.ik_time2 = static_cast<unsigned long>(rt_timer_read());

            memcpy(dataControl->RobotData.old_desired_end_pose, dataControl->RobotData.desired_end_pose, CARTESIAN_POSE_LEN*NUM_DOF);
            dataControl->cartesianPoseScaleUp(dataControl->RobotData.present_end_pose, dataControl->ServerToClient.calculateCartesianPose);

            dataControl->jointPositionRAD2ENC(dataControl->RobotData.desired_q, dataControl->RobotData.command_joint_position);

            module->setGroupSyncWriteGoalPosition(dataControl->RobotData.command_joint_position);

            goalReach(dataControl->RobotData.desired_end_pose, dataControl->RobotData.present_end_pose, &dataControl->cartesian_goal_reach);
        }
        else{
            module->setGroupSyncWriteGoalPosition(dataControl->RobotData.command_joint_position);

            goalReach(dataControl->RobotData.desired_end_pose, dataControl->RobotData.present_end_pose, &dataControl->cartesian_goal_reach);
        }
        break;
    default :
        break;
    }

    dataControl->ClientToServer.opMode = DataControl::OpMode::Wait;
}

void ControlMain::goalReach(double desired_pose[], double present_pose[], bool *goal_reach)
{
    double epsilon_pos = 0.05;
    double epsilon_ang = 1;

    double pos = sqrt(pow(desired_pose[0] - present_pose[0], 2) + pow(desired_pose[1] - present_pose[1], 2) + pow(desired_pose[2] - present_pose[2], 2));
    double ang_r = abs(desired_pose[3] - present_pose[3]);
    double ang_p = abs(desired_pose[4] - present_pose[4]);
    double ang_y = abs(desired_pose[5] - present_pose[5]);

    printf("pos : %f\n", pos);
    printf("ang_r : %f\t ang_p : %f\t ang_y : %f\n", ang_r, ang_p, ang_y);

    if (pos < epsilon_pos && ang_r < epsilon_ang && ang_p < epsilon_ang && ang_y < epsilon_ang){
        *goal_reach = true;
    }
    else{
        *goal_reach = false;
    }
}
