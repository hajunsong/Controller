#include "controlmain.h"

ControlMain::ControlMain(QObject *parent) : QObject(parent)
{
    dataControl = new DataControl();
    tcpServer = new NRMKHelper::TcpServer(dataControl);

    module_init = false;
    module = new DxlControl();

    data_indx = 0;
    module_indx = 0;

    robotArm = new RobotArm(NUM_JOINT, NUM_DOF, 0.005);

    old_end_pose_update = false;

    dxlTimer = new QTimer();
    QObject::connect(dxlTimer, SIGNAL(timeout()), this, SLOT(dxlTimeout()));
    dxlTimer->setInterval(100);

    ready_pose = false;
    cartesian_move_flag = false;

    delay = 0;
}

ControlMain::~ControlMain()
{
    printf("finished program\n");

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
            module->dxl_deinit(NUM_JOINT == 1 ? module->single_id : i);
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
            case DataControl::Module::FAR_V1:
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
            case DataControl::OpMode::PathGenerateMode:
                pThis->robotPathGenerate();
                break;
            case DataControl::OpMode::ReadyMode:
                pThis->robotReady();
                break;
            case DataControl::OpMode::RunMode:
                pThis->robotRun();
                break;
            case DataControl::OpMode::TorqueIDE:
                pThis->robotPositionControl();
                break;
            default : break;
        }

        pThis->dataControl->RobotData.dxl_time1 = static_cast<unsigned long>(rt_timer_read());
        if (MODULE_TYPE == DataControl::Module::FAR_V1){
            for(uint8_t i = 0; i < NUM_JOINT; i++){
                pThis->module->getGroupSyncReadIndirectAddress(NUM_JOINT == 1 ? pThis->module->single_id : i,
                                                               &pThis->dataControl->RobotData.present_joint_position[i],
                                                               &pThis->dataControl->RobotData.present_joint_velocity[i],
                                                               &pThis->dataControl->RobotData.present_joint_current[i]);
            }
        }
        else if(MODULE_TYPE == DataControl::Module::SEA){
            pThis->module->getPresentPosition(1, &pThis->dataControl->RobotData.present_joint_position[0]);
        }
        pThis->dataControl->RobotData.dxl_time2 = static_cast<unsigned long>(rt_timer_read());

        pThis->robotKinematics();
//        pThis->robotDynamics();

//        if (NUM_JOINT == 1){
//            rt_printf("[ID:%03d] Pos=%d, Vel=%d, Cur=%d\n", pThis->module->single_id,
//                      pThis->dataControl->RobotData.present_joint_position[0],
//                      pThis->dataControl->RobotData.present_joint_velocity[0],
//                      pThis->dataControl->RobotData.present_joint_current[0]);
//        }

//        rt_printf("Joint Offset : %d, %d, %d, %d, %d, %d\n",
//                  pThis->dataControl->RobotData.offset[0], pThis->dataControl->RobotData.offset[1],
//                pThis->dataControl->RobotData.offset[2], pThis->dataControl->RobotData.offset[3],
//                pThis->dataControl->RobotData.offset[4], pThis->dataControl->RobotData.offset[5]);

//        rt_printf("Present Position : %d, %d, %d, %d, %d, %d\n",
//                  pThis->dataControl->RobotData.present_joint_position[0], pThis->dataControl->RobotData.present_joint_position[1],
//                pThis->dataControl->RobotData.present_joint_position[2], pThis->dataControl->RobotData.present_joint_position[3],
//                pThis->dataControl->RobotData.present_joint_position[4], pThis->dataControl->RobotData.present_joint_position[5]);
//        rt_printf("Present Velocity : %d, %d, %d, %d, %d, %d\n",
//                  pThis->dataControl->RobotData.present_joint_velocity[0], pThis->dataControl->RobotData.present_joint_velocity[1],
//                pThis->dataControl->RobotData.present_joint_velocity[2], pThis->dataControl->RobotData.present_joint_velocity[3],
//                pThis->dataControl->RobotData.present_joint_velocity[4], pThis->dataControl->RobotData.present_joint_velocity[5]);
//        rt_printf("Present Current : %d, %d, %d, %d, %d, %d\n\n",
//                  pThis->dataControl->RobotData.present_joint_current[0], pThis->dataControl->RobotData.present_joint_current[1],
//                pThis->dataControl->RobotData.present_joint_current[2], pThis->dataControl->RobotData.present_joint_current[3],
//                pThis->dataControl->RobotData.present_joint_current[4], pThis->dataControl->RobotData.present_joint_current[5]);

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
        pThis->dataControl->jointVelocityENC2RPM(pThis->dataControl->RobotData.present_joint_velocity, pThis->dataControl->ServerToClient.presentJointVelocity);
        pThis->dataControl->jointCurrentRAW2mA(pThis->dataControl->RobotData.present_joint_current, pThis->dataControl->ServerToClient.presentJointCurrent);

        pThis->dataControl->ServerToClient.time = static_cast<double>((pThis->dataControl->RobotData.time2 - pThis->dataControl->RobotData.time1)/1000000.0);
        pThis->dataControl->RobotData.time1 = pThis->dataControl->RobotData.time2;
        pThis->data_indx++;
    }
}

void ControlMain::moduleInitSEA()
{

}

void ControlMain::moduleInitFAR()
{
    module->init();

    while(!module_init)
    {
        printf("Start module initialize %d\n", module_indx);
        if (NUM_JOINT ==  1){
            module->dxl_searching();
            module->dxl_init(module->single_id, dataControl->RobotData.joint_op_mode);
            int32_t pos = 0;
            module->getPresentPosition(module->single_id, &pos);
            printf("%d axis present position : %d\n", module->single_id, pos);
            if (pos != 0)
            {
                module->initGroupSyncReadIndirectAddress(module->single_id);
                module_init = true;

                module->getGroupSyncReadPresentPosition(dataControl->RobotData.offset, NUM_JOINT);
            }
        }
        else if (NUM_JOINT == 6){
            int init_result = module->dxl_init(module_indx, dataControl->RobotData.joint_op_mode);
            if (init_result){
                int32_t pos = 0;
                module->getPresentPosition(module_indx, &pos);
                printf("%d axis present position : %d\n", module_indx, pos);
                if (pos != 0)
                {
                    module->initGroupSyncReadIndirectAddress(module_indx);
                    module_indx++;
                }
            }
            if (module_indx >= NUM_JOINT){
                module_init = true;

                module->getGroupSyncReadPresentPosition(dataControl->RobotData.offset, NUM_JOINT);
            }
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

    module->setGroupSyncWriteTorqueEnable(static_cast<uint8_t>(enable), NUM_JOINT);

    dataControl->DataReset();
    dataControl->ClientToServer.opMode = DataControl::OpMode::Wait;
}

void ControlMain::robotKinematics(){
    dataControl->jointPositionENC2RAD(dataControl->RobotData.present_joint_position, dataControl->RobotData.present_q);

    robotArm->run_kinematics(dataControl->RobotData.present_q, dataControl->RobotData.present_end_pose);
}

void ControlMain::robotDynamics(){
    dataControl->jointPositionENC2RAD(dataControl->RobotData.present_joint_position, dataControl->RobotData.present_q);
    dataControl->jointVelocityENC2RAD(dataControl->RobotData.present_joint_velocity, dataControl->RobotData.present_q_dot);

    double goal_torque[NUM_JOINT] = {0,};
    robotArm->gravity_compensation(dataControl->RobotData.present_q, dataControl->RobotData.present_q_dot, goal_torque);

    double goal_current[NUM_JOINT] = {0,};
    double alpha = 1.0;
    for(uint i = 0; i < NUM_JOINT; i++){
        if (i < 3){
            goal_current[i] = 1000*(goal_torque[i] / TORQUE_CONSTANT_W270)*alpha;
        }
        else{
            goal_current[i] = 1000*(goal_torque[i] / TORQUE_CONSTANT_W150)*alpha;
        }
    }

    dataControl->jointCurrentmA2RAW(goal_current, dataControl->RobotData.command_joint_current);
    if (dataControl->RobotData.joint_op_mode == JointOpMode::current_mode){
        module->setGroupSyncWriteGoalCurrent(dataControl->RobotData.command_joint_current, NUM_JOINT);
    }
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
            module->setGroupSyncWriteGoalPosition(dataControl->RobotData.command_joint_position, NUM_JOINT);
            break;
        case DataControl::Motion::JointMotion:
            for(unsigned char i = 0; i < NUM_JOINT; i++){
                if (desJoint[i] < 0 || desJoint[i] > 0){
//                    dataControl->RobotData.command_joint_position[i] = static_cast<int32_t>(desJoint[i]/POSITION_UNIT);
                    //                module->setGoalPosition(dataControl->RobotData.command_joint_position[i], i);
                    dataControl->jointPositionDEG2ENC(desJoint, dataControl->RobotData.command_joint_position);
                }
                else{
                    dataControl->RobotData.command_joint_position[i] = dataControl->RobotData.present_joint_position[i];
                }
            }
            module->setGroupSyncWriteGoalPosition(dataControl->RobotData.command_joint_position, NUM_JOINT);
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

                module->setGroupSyncWriteGoalPosition(dataControl->RobotData.command_joint_position, NUM_JOINT);

                goalReach(dataControl->RobotData.desired_end_pose, dataControl->RobotData.present_end_pose, &dataControl->cartesian_goal_reach);
            }
            else{
                module->setGroupSyncWriteGoalPosition(dataControl->RobotData.command_joint_position, NUM_JOINT);

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

                module->setGroupSyncWriteGoalPosition(dataControl->RobotData.command_joint_position, NUM_JOINT);

                goalReach(dataControl->RobotData.desired_end_pose, dataControl->RobotData.present_end_pose, &dataControl->cartesian_goal_reach);
            }
            else{
                module->setGroupSyncWriteGoalPosition(dataControl->RobotData.command_joint_position, NUM_JOINT);

                goalReach(dataControl->RobotData.desired_end_pose, dataControl->RobotData.present_end_pose, &dataControl->cartesian_goal_reach);
            }
            break;
        default :
            break;
    }

    dataControl->ClientToServer.opMode = DataControl::OpMode::Wait;
}

void ControlMain::robotPathGenerate()
{
    uint8_t row = static_cast<uint8_t>(dataControl->PathData.row);
    for(uint i = 0; i < row - 1; i++){
        path_generator(dataControl->PathData.point_x[i], dataControl->PathData.point_x[i + 1],
                dataControl->PathData.total_time[i + 1] - dataControl->PathData.total_time[i],
                dataControl->PathData.acc_time[i], 0.005, &dataControl->PathData.path_x);

        path_generator(dataControl->PathData.point_y[i], dataControl->PathData.point_y[i + 1],
                dataControl->PathData.total_time[i + 1] - dataControl->PathData.total_time[i],
                dataControl->PathData.acc_time[i], 0.005, &dataControl->PathData.path_y);

        path_generator(dataControl->PathData.point_z[i], dataControl->PathData.point_z[i + 1],
                dataControl->PathData.total_time[i + 1] - dataControl->PathData.total_time[i],
                dataControl->PathData.acc_time[i], 0.005, &dataControl->PathData.path_z);
    }

    rt_printf("path size : %d\n", dataControl->PathData.path_x.size());
    dataControl->ClientToServer.opMode = DataControl::OpMode::Wait;
}

void ControlMain::robotRun()
{
    switch(dataControl->RobotData.run_mode){
        case 1:
//            for(uint i = 0; i < NUM_DOF; i++){
//                dataControl->RobotData.desired_end_pose[i] = dataControl->PathData.pathDataPick[dataControl->PathData.path_data_indx*dataControl->PathData.col + i+8];
//            }

//            dataControl->RobotData.ik_time1 = static_cast<unsigned long>(rt_timer_read());
//            robotArm->run_inverse_kinematics(dataControl->RobotData.present_q, dataControl->RobotData.desired_end_pose,
//                                             dataControl->RobotData.desired_q, dataControl->RobotData.present_end_pose);
//            dataControl->RobotData.ik_time2 = static_cast<unsigned long>(rt_timer_read());

//            dataControl->cartesianPoseScaleUp(dataControl->RobotData.present_end_pose, dataControl->ServerToClient.calculateCartesianPose);

//            dataControl->jointPositionRAD2ENC(dataControl->RobotData.desired_q, dataControl->RobotData.command_joint_position);

            dataControl->RobotData.desired_q[0] = 0.7665113;
            dataControl->RobotData.desired_q[1] = -0.3214468;
            dataControl->RobotData.desired_q[2] = 2.2424687;
            dataControl->RobotData.desired_q[3] = -1.9210219;
            dataControl->RobotData.desired_q[4] = 0.2806862;
            dataControl->RobotData.desired_q[5] = 0;

            dataControl->jointPositionRAD2ENC(dataControl->RobotData.desired_q, dataControl->RobotData.command_joint_position);

            module->setGroupSyncWriteGoalPosition(dataControl->RobotData.command_joint_position, NUM_JOINT);
            break;
        case 2:
            dataControl->RobotData.desired_end_pose[0] = dataControl->PathData.path_x[dataControl->PathData.path_data_indx];
            dataControl->RobotData.desired_end_pose[1] = dataControl->PathData.path_y[dataControl->PathData.path_data_indx];
            dataControl->RobotData.desired_end_pose[2] = dataControl->PathData.path_z[dataControl->PathData.path_data_indx];
            dataControl->RobotData.desired_end_pose[3] = 1.5707963;
            dataControl->RobotData.desired_end_pose[4] = 0;
            dataControl->RobotData.desired_end_pose[5] = -2.094399;

            rt_printf("desired_end_pose : %f, %f, %f, %f, %f, %f\n",
                      dataControl->RobotData.desired_end_pose[0], dataControl->RobotData.desired_end_pose[1],
                    dataControl->RobotData.desired_end_pose[2], dataControl->RobotData.desired_end_pose[3],
                    dataControl->RobotData.desired_end_pose[4], dataControl->RobotData.desired_end_pose[5]);

            dataControl->RobotData.ik_time1 = static_cast<unsigned long>(rt_timer_read());
            robotArm->run_inverse_kinematics(dataControl->RobotData.present_q, dataControl->RobotData.desired_end_pose,
                                             dataControl->RobotData.desired_q, dataControl->RobotData.present_end_pose);
            dataControl->RobotData.ik_time2 = static_cast<unsigned long>(rt_timer_read());

            dataControl->cartesianPoseScaleUp(dataControl->RobotData.present_end_pose, dataControl->ServerToClient.calculateCartesianPose);

            dataControl->jointPositionRAD2ENC(dataControl->RobotData.desired_q, dataControl->RobotData.command_joint_position);

            rt_printf("path_data_indx : %d\n", dataControl->PathData.path_data_indx);

            module->setGroupSyncWriteGoalPosition(dataControl->RobotData.command_joint_position, NUM_JOINT);

            goalReach(dataControl->RobotData.desired_end_pose, dataControl->RobotData.present_end_pose, &dataControl->cartesian_goal_reach);

            if (dataControl->cartesian_goal_reach){
                ready_pose = true;
                dataControl->PathData.path_data_indx += 1;
            }

            if (dataControl->PathData.path_data_indx >= dataControl->PathData.path_x.size()){
                if(dataControl->PathData.cycle_count == -1)
                {
                    dataControl->PathData.path_data_indx = 0;
                }
                else
                {
                    dataControl->ClientToServer.opMode = DataControl::OpMode::Wait;
                }
            }

            break;
        default:
            break;
    }

//    switch(type){
//        case DataControl::PathDataType::Save1: // pick-up motion(joint)
//        {
//            double path[6];
//            for(uint i = 0; i < 6; i++){
//                path[i] = dataControl->PathData.pathDataPick[dataControl->PathData.path_data_indx*dataControl->PathData.col + i+2];
//            }

//            dataControl->jointPositionRAD2ENC(path, dataControl->RobotData.command_joint_position);
//            break;
//        }
//        case DataControl::PathDataType::Save2: // pick-up motion(cartesian)
//        {
//            for(uint i = 0; i < NUM_DOF; i++){
//                dataControl->RobotData.desired_end_pose[i] = dataControl->PathData.pathDataPick[dataControl->PathData.path_data_indx*dataControl->PathData.col + i+8];
//            }

//            dataControl->RobotData.ik_time1 = static_cast<unsigned long>(rt_timer_read());
//            robotArm->run_inverse_kinematics(dataControl->RobotData.present_q, dataControl->RobotData.desired_end_pose,
//                                             dataControl->RobotData.desired_q, dataControl->RobotData.present_end_pose);
//            dataControl->RobotData.ik_time2 = static_cast<unsigned long>(rt_timer_read());

//            dataControl->cartesianPoseScaleUp(dataControl->RobotData.present_end_pose, dataControl->ServerToClient.calculateCartesianPose);

//            dataControl->jointPositionRAD2ENC(dataControl->RobotData.desired_q, dataControl->RobotData.command_joint_position);
//            break;
//        }
//        case DataControl::PathDataType::Save3: // rect motion(joint)
//        {
//            double path[6];
//            for(uint i = 0; i < 6; i++){
//                path[i] = dataControl->PathData.pathDataRect[dataControl->PathData.path_data_indx*dataControl->PathData.col + i+2];
//            }

//            dataControl->jointPositionRAD2ENC(path, dataControl->RobotData.command_joint_position);
//            break;
//        }
//        case DataControl::PathDataType::Save4: // rect motion(cartesian)
//        {
//            for(uint i = 0; i < NUM_DOF; i++){
//                dataControl->RobotData.desired_end_pose[i] = dataControl->PathData.pathDataRect[dataControl->PathData.path_data_indx*dataControl->PathData.col + i+8];
//            }

//            dataControl->RobotData.ik_time1 = static_cast<unsigned long>(rt_timer_read());
//            robotArm->run_inverse_kinematics(dataControl->RobotData.present_q, dataControl->RobotData.desired_end_pose,
//                                             dataControl->RobotData.desired_q, dataControl->RobotData.present_end_pose);
//            dataControl->RobotData.ik_time2 = static_cast<unsigned long>(rt_timer_read());

//            dataControl->cartesianPoseScaleUp(dataControl->RobotData.present_end_pose, dataControl->ServerToClient.calculateCartesianPose);

//            dataControl->jointPositionRAD2ENC(dataControl->RobotData.desired_q, dataControl->RobotData.command_joint_position);
//            break;
//        }
//        case DataControl::PathDataType::Save5: // rect motion for evaluation(joint)
//        {
//            double path[6];
//            for(uint i = 0; i < 6; i++){
//                path[i] = dataControl->PathData.pathDataRect2[dataControl->PathData.path_data_indx*dataControl->PathData.col + i+2];
//            }

//            dataControl->jointPositionRAD2ENC(path, dataControl->RobotData.command_joint_position);
//            break;
//        }
//        case DataControl::PathDataType::Save6: // rect motion for evaluation(cartesian)
//        {
//            for(uint i = 0; i < NUM_DOF; i++){
//                dataControl->RobotData.desired_end_pose[i] = dataControl->PathData.pathDataRect2[dataControl->PathData.path_data_indx*dataControl->PathData.col + i+8];
//            }

//            dataControl->RobotData.ik_time1 = static_cast<unsigned long>(rt_timer_read());
//            robotArm->run_inverse_kinematics(dataControl->RobotData.present_q, dataControl->RobotData.desired_end_pose,
//                                             dataControl->RobotData.desired_q, dataControl->RobotData.present_end_pose);
//            dataControl->RobotData.ik_time2 = static_cast<unsigned long>(rt_timer_read());

//            dataControl->cartesianPoseScaleUp(dataControl->RobotData.present_end_pose, dataControl->ServerToClient.calculateCartesianPose);

//            dataControl->jointPositionRAD2ENC(dataControl->RobotData.desired_q, dataControl->RobotData.command_joint_position);
//            break;
//        }
//        case DataControl::PathDataType::Save7:  // calibration motion(joint)
//        {
//            double path[6];
//            for(uint i = 0; i < 6; i++){
//                path[i] = dataControl->PathData.pathDataCalibration[dataControl->PathData.path_data_indx*dataControl->PathData.col + i+1];
//            }

//            dataControl->jointPositionDEG2ENC(path, dataControl->RobotData.command_joint_position);
//            break;
//        }
//        case DataControl::PathDataType::Save8: // linear motion 42(joint)
//        {
//            double path[6];
//            for(uint i = 0; i < 6; i++){
//                path[i] = dataControl->PathData.pathDataLinear42[dataControl->PathData.path_data_indx*dataControl->PathData.col + i+2];
//            }

//            dataControl->jointPositionRAD2ENC(path, dataControl->RobotData.command_joint_position);
//            break;
//        }
//        case DataControl::PathDataType::Save9: // linear motion 42(cartesian)
//        {
//            for(uint i = 0; i < NUM_DOF; i++){
//                dataControl->RobotData.desired_end_pose[i] = dataControl->PathData.pathDataLinear42[dataControl->PathData.path_data_indx*dataControl->PathData.col + i+8];
//            }

//            dataControl->RobotData.ik_time1 = static_cast<unsigned long>(rt_timer_read());
//            robotArm->run_inverse_kinematics(dataControl->RobotData.present_q, dataControl->RobotData.desired_end_pose,
//                                             dataControl->RobotData.desired_q, dataControl->RobotData.present_end_pose);
//            dataControl->RobotData.ik_time2 = static_cast<unsigned long>(rt_timer_read());

//            dataControl->cartesianPoseScaleUp(dataControl->RobotData.present_end_pose, dataControl->ServerToClient.calculateCartesianPose);

//            dataControl->jointPositionRAD2ENC(dataControl->RobotData.desired_q, dataControl->RobotData.command_joint_position);
//            break;
//        }
//        case DataControl::PathDataType::Save10: // linear motion 24(joint)
//        {
//            double path[6];
//            for(uint i = 0; i < 6; i++){
//                path[i] = dataControl->PathData.pathDataLinear24[dataControl->PathData.path_data_indx*dataControl->PathData.col + i+2];
//            }

//            dataControl->jointPositionRAD2ENC(path, dataControl->RobotData.command_joint_position);
//            break;
//        }
//        case DataControl::PathDataType::Save11: // linear motion 24(cartesian)
//        {
//            for(uint i = 0; i < NUM_DOF; i++){
//                dataControl->RobotData.desired_end_pose[i] = dataControl->PathData.pathDataLinear24[dataControl->PathData.path_data_indx*dataControl->PathData.col + i+8];
//            }

//            dataControl->RobotData.ik_time1 = static_cast<unsigned long>(rt_timer_read());
//            robotArm->run_inverse_kinematics(dataControl->RobotData.present_q, dataControl->RobotData.desired_end_pose,
//                                             dataControl->RobotData.desired_q, dataControl->RobotData.present_end_pose);
//            dataControl->RobotData.ik_time2 = static_cast<unsigned long>(rt_timer_read());

//            dataControl->cartesianPoseScaleUp(dataControl->RobotData.present_end_pose, dataControl->ServerToClient.calculateCartesianPose);

//            dataControl->jointPositionRAD2ENC(dataControl->RobotData.desired_q, dataControl->RobotData.command_joint_position);
//            break;
//        }
//    }

//    rt_printf("path_data_indx : %d\n", dataControl->PathData.path_data_indx);

//    for(uint i = 0; i < NUM_JOINT; i++){
//        rt_printf("%d\t", dataControl->RobotData.command_joint_position[i]);
//    }
//    rt_printf("\n");

//    module->setGroupSyncWriteGoalPosition(dataControl->RobotData.command_joint_position, NUM_JOINT);

//    if (type == DataControl::PathDataType::Save5 || type == DataControl::PathDataType::Save6 ||
//            type == DataControl::PathDataType::Save8 || type == DataControl::PathDataType::Save9 ||
//            type == DataControl::PathDataType::Save10 || type == DataControl::PathDataType::Save11){
//        if ((dataControl->PathData.path_data_indx - 1) % 500 == 0){
//            delay++;
//            if (delay >= 600){
//                delay = 0;
//            }
//        }
//    }
//    else if (type == DataControl::PathDataType::Save7){
//        delay++;
//        if (delay >= 1000){
//            delay = 0;
//        }
//    }

//    if (delay == 0){
//        if (cartesian_move_flag){
//            goalReach(dataControl->RobotData.desired_end_pose, dataControl->RobotData.present_end_pose, &dataControl->cartesian_goal_reach);

//            if (dataControl->cartesian_goal_reach){
//                ready_pose = true;
//                dataControl->PathData.path_data_indx += 1;
//            }
//        }
//        else{
//            dataControl->PathData.path_data_indx += 1;
//        }
//    }

//    if (dataControl->PathData.path_data_indx >= dataControl->PathData.row){
//        if(dataControl->PathData.repeat == -1)
//        {
//            dataControl->PathData.path_data_indx = 0;
//            ready_pose = false;
//            cartesian_move_flag = false;
//        }
//        else
//        {
//            dataControl->ClientToServer.opMode = DataControl::OpMode::Wait;
//            ready_pose = false;
//            cartesian_move_flag = false;
//        }
//    }
}

void ControlMain::robotReady()
{
//    path_generator(dataControl->RobotData.present_end_pose[0], dataControl->PathData.point_x[0], 0.5, 0.1, 0.005, &dataControl->PathData.ready_path_x);
//    path_generator(dataControl->RobotData.present_end_pose[1], dataControl->PathData.point_y[0], 0.5, 0.1, 0.005, &dataControl->PathData.ready_path_y);
//    path_generator(dataControl->RobotData.present_end_pose[2], dataControl->PathData.point_z[0], 0.5, 0.1, 0.005, &dataControl->PathData.ready_path_z);

    dataControl->RobotData.run_mode = 1;

    dataControl->ClientToServer.opMode = DataControl::OpMode::RunMode;

    dataControl->PathData.path_data_indx = 0;
}

void ControlMain::robotPositionControl()
{
    dataControl->PIDControl.Kp = 2700;
    dataControl->PIDControl.Kd = 50;
    dataControl->PIDControl.Ki = 200;

    dataControl->PIDControl.h = 0.005;
    dataControl->PIDControl.des = 90*M_PI/180.0;

    double present_position = 0;
    dataControl->jointPositionENC2RAD(&dataControl->RobotData.present_joint_position[0], &present_position);
    dataControl->PIDControl.err = dataControl->PIDControl.des - present_position;

    dataControl->PIDControl.err_accum += dataControl->PIDControl.err*dataControl->PIDControl.h;

    double T = dataControl->PIDControl.err*dataControl->PIDControl.Kp
            + (dataControl->PIDControl.err - dataControl->PIDControl.err_prev)/dataControl->PIDControl.h*dataControl->PIDControl.Kd
            + dataControl->PIDControl.err_accum*dataControl->PIDControl.Ki;

    T = dataControl->torqueIdeData.mass*(9.80665)*0.2*sin(present_position);

    double current = 0;
//    double Kt = 1.65;
    double Kt = dataControl->torqueIdeData.torque_constant;
    current = 1000 * T / Kt;

    int16_t goal_current = 0;
    dataControl->jointCurrentmA2RAW(&current, &goal_current);

    module->setGroupSyncWriteGoalCurrent(&goal_current, NUM_JOINT);

    printf("Present Pos : %f, Goal Toruqe : %f, Dxl Current : %d\n", present_position*180/M_PI, T, goal_current);

    dataControl->PIDControl.err_prev = dataControl->PIDControl.err;
}

void ControlMain::goalReach(double desired_pose[], double present_pose[], bool *goal_reach)
{
    double epsilon_pos = 0.05;
    double epsilon_ang = 1;

    double pos = sqrt(pow(desired_pose[0] - present_pose[0], 2) + pow(desired_pose[1] - present_pose[1], 2) + pow(desired_pose[2] - present_pose[2], 2));
    double ang_r = abs(desired_pose[3] - present_pose[3]);
    double ang_p = abs(desired_pose[4] - present_pose[4]);
    double ang_y = abs(desired_pose[5] - present_pose[5]);

//    printf("pos : %f\n", pos);
//    printf("ang_r : %f\t ang_p : %f\t ang_y : %f\n", ang_r, ang_p, ang_y);

    if (pos < epsilon_pos && ang_r < epsilon_ang && ang_p < epsilon_ang && ang_y < epsilon_ang){
        *goal_reach = true;
    }
    else{
        *goal_reach = false;
    }
}

void ControlMain::path_generator(double x0, double xf, double tf, double ta, double h, std::vector<double> *path)
{
//    rt_printf("Start path generator, x0 : %f, xf : %f, tf : %f, ta : %f, h : %f\n", x0, xf, tf, ta, h);
    double td = tf - ta;
    double vd = (xf - x0)/td;
    double xa = x0 + 0.5*ta*vd;
    double xd = xf - 0.5*ta*vd;

    double pos0, posf, vel0, velf, acc0, accf, ts;
    double a0, a1, a2, a3, a4, a5;

    // section of acceleration
    pos0 = x0; posf = xa; vel0 = 0; velf = vd; acc0 = 0; accf = 0; ts = ta;
    a0 = pos0;
    a1 = vel0;
    a2 = acc0/2;
    a3 = (20*(posf - pos0) - (8*velf + 12*vel0)*ts - (3*acc0 - accf)*pow(ts,2))/(2*pow(ts,3));
    a4 = (30*(pos0 - posf) + (14*velf + 16*vel0)*ts + (3*acc0 - 2*accf)*pow(ts,2))/(2*pow(ts, 4));
    a5 = (12*(posf - pos0) - (6*velf + 6*vel0)*ts - (acc0 - accf)*pow(ts,2))/(2*pow(ts,5));

    for(double t = 0; t < ts; t += h){
        path->push_back(a0 + a1*t + a2*pow(t,2) + a3*pow(t,3) + a4*pow(t,4)+ a5*pow(t,5));
    }

    // section of constant velocity
    pos0 = xa; posf = xd; vel0 = vd; velf = vd; acc0 = 0; accf = 0; ts = td - ta;
    a0 = pos0;
    a1 = vel0;
    a2 = acc0/2;
    a3 = (20*(posf - pos0) - (8*velf + 12*vel0)*ts - (3*acc0 - accf)*pow(ts,2))/(2*pow(ts,3));
    a4 = (30*(pos0 - posf) + (14*velf + 16*vel0)*ts + (3*acc0 - 2*accf)*pow(ts,2))/(2*pow(ts, 4));
    a5 = (12*(posf - pos0) - (6*velf + 6*vel0)*ts - (acc0 - accf)*pow(ts,2))/(2*pow(ts,5));

    for(double t = 0; t < ts; t += h){
        path->push_back(a0 + a1*t + a2*pow(t,2) + a3*pow(t,3) + a4*pow(t,4)+ a5*pow(t,5));
    }

    // section of deceleration
    pos0 = xd; posf = xf; vel0 = vd; velf = 0; acc0 = 0; accf = 0; ts = tf - td;
    a0 = pos0;
    a1 = vel0;
    a2 = acc0/2;
    a3 = (20*(posf - pos0) - (8*velf + 12*vel0)*ts - (3*acc0 - accf)*pow(ts,2))/(2*pow(ts,3));
    a4 = (30*(pos0 - posf) + (14*velf + 16*vel0)*ts + (3*acc0 - 2*accf)*pow(ts,2))/(2*pow(ts, 4));
    a5 = (12*(posf - pos0) - (6*velf + 6*vel0)*ts - (acc0 - accf)*pow(ts,2))/(2*pow(ts,5));

    for(double t = 0; t < ts; t += h){
        path->push_back(a0 + a1*t + a2*pow(t,2) + a3*pow(t,3) + a4*pow(t,4)+ a5*pow(t,5));
    }
}
