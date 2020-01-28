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
    connect(dxlTimer, SIGNAL(timeout()), this, SLOT(dxlTimeout()));
    dxlTimer->setInterval(100);

    ready_pose = false;
    cartesian_move_flag = false;

    delay_cnt_max = 1000;
    delay_cnt = 0;
    fork_cnt_max = 0;
    fork_cnt = 0;
    dataControl->feeding = false;

    connect(this, SIGNAL(disconnectClientSignal()), this, SLOT(disconnectClient()));

    controlMainCustom = new ControlMainCustom();
}

ControlMain::~ControlMain()
{
    printf("finishing...\n");

    robot_RT_stop();
    usleep(10000);
    delete controlMainCustom;
    printf("Complete destructure ControlMainCustom\n");
    usleep(10000);
    delete tcpServer;
    printf("Complete destructure TcpServer\n");
    delete dataControl;
    printf("Complete destructure DataControl\n");
    delete robotArm;
    printf("Complete destructure RobotArm\n");

    usleep(10000);
    if (module_init){
        for(uint8_t i = 0; i < NUM_JOINT; i++){
            module->dxl_deinit(NUM_JOINT == 1 ? module->single_id : i);
        }
    }
    delete module;
    printf("Complete destructure Dynamixel\n");

    printf("Finished\n");
}

void ControlMain::disconnectClient(){
    robot_RT_stop();

    printf("Restart\n");
    start();
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
					printf("Start FAR V1 Module Initilization\n");
                    moduleInitFAR();
                }
                else{
                    tcpServer->sendKey('S');
                }
                break;
			case DataControl::Module::FAR_V2:
				if (!module_init){
					printf("Start FAR V2 Module Initilization\n");
					moduleInitFAR();
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

        rt_task_start(&robot_task, &controlMainCustom->robot_run, this);
    }
}

void ControlMain::robot_RT_stop(){
    if (robot_thread_run){
        controlMainCustom->robot_stop();
        robot_thread_run = false;
    }
    else{
        printf("Robot Control RT Thread not running...\n");
    }
}

//void ControlMain::robot_run(void *arg)
//{
//    ControlMain* pThis = static_cast<ControlMain*>(arg);
//    pThis->dataControl->RobotData.time1 = static_cast<unsigned long>(rt_timer_read());

//    pThis->robot_thread_run = false;

//    rt_task_set_periodic(&pThis->robot_task, TM_NOW, 5e6);

//    pThis->robot_thread_run = true;

//    while(pThis->robot_thread_run){
//        rt_task_wait_period(nullptr); //wait for next cycle
//        pThis->dataControl->RobotData.time2 = static_cast<unsigned long>(rt_timer_read());

//        switch(pThis->dataControl->ClientToServer.opMode){
//            case DataControl::OpMode::ServoOnOff:
//                pThis->robotServoOn(pThis->dataControl->ClientToServer.subMode);
//                break;
//            case DataControl::OpMode::Initialize:
//                pThis->robotInitialize();
//                break;
//            case DataControl::OpMode::Wait:
//                break;
//            case DataControl::OpMode::JointMove:
//                pThis->robotJointMove(pThis->dataControl->ClientToServer.subMode, pThis->dataControl->ClientToServer.desiredJoint);
//                break;
//            case DataControl::OpMode::CartesianMove:
//                pThis->robotCartesianMove(pThis->dataControl->ClientToServer.subMode, pThis->dataControl->ClientToServer.desiredPose);
//                break;
//            case DataControl::OpMode::PathGenerateMode:
//                pThis->robotPathGenerate();
//                break;
//            case DataControl::OpMode::ReadyMode:
//                pThis->robotReady();
//                break;
//            case DataControl::OpMode::RunMode:
//                pThis->robotRun();
//                break;
//            case DataControl::OpMode::TorqueIDE:
//                pThis->robotPositionControl();
//                break;
//            default : break;
//        }

//        pThis->dataControl->RobotData.dxl_time1 = static_cast<unsigned long>(rt_timer_read());
//        if (MODULE_TYPE == DataControl::Module::FAR_V1){
//            pThis->module->getGroupSyncReadIndirectAddress(pThis->dataControl->RobotData.present_joint_position,
//                                                           pThis->dataControl->RobotData.present_joint_velocity,
//                                                           pThis->dataControl->RobotData.present_joint_current, NUM_JOINT);
//        }
//        pThis->dataControl->RobotData.dxl_time2 = static_cast<unsigned long>(rt_timer_read());

//        pThis->robotKinematics();
//        pThis->robotDynamics();

////        if (NUM_JOINT == 1){
////            rt_printf("[ID:%03d] Pos=%d, Vel=%d, Cur=%d\n", pThis->module->single_id,
////                      pThis->dataControl->RobotData.present_joint_position[0],
////                      pThis->dataControl->RobotData.present_joint_velocity[0],
////                      pThis->dataControl->RobotData.present_joint_current[0]);
////        }

////        rt_printf("Joint Offset : %d, %d, %d, %d, %d, %d\n",
////                  pThis->dataControl->RobotData.offset[0], pThis->dataControl->RobotData.offset[1],
////                pThis->dataControl->RobotData.offset[2], pThis->dataControl->RobotData.offset[3],
////                pThis->dataControl->RobotData.offset[4], pThis->dataControl->RobotData.offset[5]);

////        rt_printf("Present Position : %d, %d, %d, %d, %d, %d\n",
////                  pThis->dataControl->RobotData.present_joint_position[0], pThis->dataControl->RobotData.present_joint_position[1],
////                pThis->dataControl->RobotData.present_joint_position[2], pThis->dataControl->RobotData.present_joint_position[3],
////                pThis->dataControl->RobotData.present_joint_position[4], pThis->dataControl->RobotData.present_joint_position[5]);
////        rt_printf("Present Velocity : %d, %d, %d, %d, %d, %d\n",
////                  pThis->dataControl->RobotData.present_joint_velocity[0], pThis->dataControl->RobotData.present_joint_velocity[1],
////                pThis->dataControl->RobotData.present_joint_velocity[2], pThis->dataControl->RobotData.present_joint_velocity[3],
////                pThis->dataControl->RobotData.present_joint_velocity[4], pThis->dataControl->RobotData.present_joint_velocity[5]);
////        rt_printf("Present Current : %d, %d, %d, %d, %d, %d\n\n",
////                  pThis->dataControl->RobotData.present_joint_current[0], pThis->dataControl->RobotData.present_joint_current[1],
////                pThis->dataControl->RobotData.present_joint_current[2], pThis->dataControl->RobotData.present_joint_current[3],
////                pThis->dataControl->RobotData.present_joint_current[4], pThis->dataControl->RobotData.present_joint_current[5]);

////        rt_printf("Present Pose : %f, %f, %f, %f, %f, %f\n",
////                  pThis->dataControl->ServerToClient.presentCartesianPose[0], pThis->dataControl->ServerToClient.presentCartesianPose[1],
////                pThis->dataControl->ServerToClient.presentCartesianPose[2], pThis->dataControl->ServerToClient.presentCartesianPose[3],
////                pThis->dataControl->ServerToClient.presentCartesianPose[4], pThis->dataControl->ServerToClient.presentCartesianPose[5]);
////        rt_printf("Robot : Time since last turn: %ld.%06ld ms\n",
////                  (pThis->dataControl->RobotData.time2 - pThis->dataControl->RobotData.time1) / 1000000,
////                  (pThis->dataControl->RobotData.time2 - pThis->dataControl->RobotData.time1) % 1000000);
////        rt_printf("Dxl : Time since last turn: %ld.%06ld ms\n",
////                  (pThis->dataControl->RobotData.dxl_time2 - pThis->dataControl->RobotData.dxl_time1) / 1000000,
////                  (pThis->dataControl->RobotData.dxl_time2 - pThis->dataControl->RobotData.dxl_time1) % 1000000);

//        pThis->dataControl->ServerToClient.data_index = pThis->data_indx;

//        pThis->dataControl->jointPositionENC2DEG(pThis->dataControl->RobotData.present_joint_position, pThis->dataControl->ServerToClient.presentJointPosition);
//        pThis->dataControl->cartesianPoseScaleUp(pThis->dataControl->RobotData.present_end_pose, pThis->dataControl->ServerToClient.presentCartesianPose);
//        pThis->dataControl->jointVelocityENC2RPM(pThis->dataControl->RobotData.present_joint_velocity, pThis->dataControl->ServerToClient.presentJointVelocity);
//        pThis->dataControl->jointCurrentRAW2mA(pThis->dataControl->RobotData.present_joint_current, pThis->dataControl->ServerToClient.presentJointCurrent);

//        pThis->dataControl->ServerToClient.time = static_cast<double>((pThis->dataControl->RobotData.time2 - pThis->dataControl->RobotData.time1)/1000000.0);
//        pThis->dataControl->ServerToClient.dxl_time = static_cast<double>((pThis->dataControl->RobotData.dxl_time2 - pThis->dataControl->RobotData.dxl_time1)/1000000.0);
//        pThis->dataControl->ServerToClient.ik_time = static_cast<double>((pThis->dataControl->RobotData.ik_time2 - pThis->dataControl->RobotData.ik_time1)/1000000.0);

//        pThis->dataControl->RobotData.time1 = pThis->dataControl->RobotData.time2;
//        pThis->data_indx++;

//        if (!pThis->tcpServer->isConnected()){
//            emit pThis->disconnectClientSignal();
//        }
//    }
//}

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
                    module->initGroupSyncWriteIndirectAddress(module_indx);
                    module_indx++;
                }
            }
            if (module_indx >= NUM_JOINT){
                module_init = true;

                if (!dataControl->RobotData.offset_setting){
                    module->getGroupSyncReadPresentPosition(dataControl->RobotData.offset, NUM_JOINT);
                }
                dataControl->RobotData.offset[3] -= static_cast<int32_t>(-90*dataControl->DEG2ENC);
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

//    rt_printf("[FK] present joint position : %d, %d, %d, %d, %d, %d\n",
//           dataControl->RobotData.present_joint_position[0], dataControl->RobotData.present_joint_position[1],
//            dataControl->RobotData.present_joint_position[2], dataControl->RobotData.present_joint_position[3],
//            dataControl->RobotData.present_joint_position[4], dataControl->RobotData.present_joint_position[5]);
//    rt_printf("[FK] present q : %f, %f, %f, %f, %f, %f\n",
//           dataControl->RobotData.present_q[0], dataControl->RobotData.present_q[1],
//            dataControl->RobotData.present_q[2], dataControl->RobotData.present_q[3],
//            dataControl->RobotData.present_q[4], dataControl->RobotData.present_q[5]);
//    rt_printf("[FK] present pose : %f, %f, %f, %f, %f, %f\n",
//           dataControl->RobotData.present_end_pose[0], dataControl->RobotData.present_end_pose[1],
//            dataControl->RobotData.present_end_pose[2], dataControl->RobotData.present_end_pose[3],
//            dataControl->RobotData.present_end_pose[4], dataControl->RobotData.present_end_pose[5]);

    robotArm->jacobian();

//    for(uint i = 0; i < 6; i++){
//        for(uint j = 0; j < 6; j++){
//            printf("%f\t", robotArm->J[i*6+j]);
//        }
//        printf("\n");
//    }
//    printf("\n");

    double velocity;
    for(uint i = 0; i < 6; i++)
    {
        velocity = 0;
        for(uint j = 0; j < 6; j++)
        {
            velocity += robotArm->J[i*6 + j]*dataControl->RobotData.present_q_dot[j];
        }
        dataControl->RobotData.present_end_vel[i] = velocity;
    }

}

void ControlMain::robotDynamics(){
    dataControl->jointPositionENC2RAD(dataControl->RobotData.present_joint_position, dataControl->RobotData.present_q);
    dataControl->jointVelocityENC2RAD(dataControl->RobotData.present_joint_velocity, dataControl->RobotData.present_q_dot);

    double goal_torque[NUM_JOINT] = {0,};
    robotArm->gravity_compensation(dataControl->RobotData.present_q, dataControl->RobotData.present_q_dot, goal_torque);

    double goal_current[NUM_JOINT] = {0,};
    double alpha = 1.0;
    for(uint i = 0; i < NUM_JOINT; i++){
#if MODULE_TYPE == 1
        if (i < 3){
            goal_current[i] = 1000*(goal_torque[i] / TORQUE_CONSTANT_W270)*alpha;
        }
        else{
            goal_current[i] = 1000*(goal_torque[i] / TORQUE_CONSTANT_W150)*alpha;
        }
#elif MODULE_TYPE == 2
		if (i < 3){
            goal_current[i] = 1000*(goal_torque[i] / TORQUE_CONSTANT_V270)*alpha;
		}
		else{
            goal_current[i] = 1000*(goal_torque[i] / TORQUE_CONSTANT_V350)*alpha;
		}
#endif
	}

//    rt_printf("Desired Torque : %f, %f, %f, %f, %f, %f\n", goal_torque[0], goal_torque[1], goal_torque[2], goal_torque[3], goal_torque[4], goal_torque[5]);

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
                dataControl->RobotData.command_joint_position[i] = dataControl->RobotData.present_joint_position[i] + static_cast<int32_t>(desJoint[i]*dataControl->DEG2ENC);
            }
            module->setGroupSyncWriteGoalPosition(dataControl->RobotData.command_joint_position, NUM_JOINT);
            break;
        case DataControl::Motion::JointMotion:
            dataControl->jointPositionDEG2ENC(desJoint, dataControl->RobotData.command_joint_position);
            module->setGroupSyncWriteGoalPosition(dataControl->RobotData.command_joint_position, NUM_JOINT);
            break;
        default :
            break;
    }

    dataControl->ClientToServer.opMode = DataControl::OpMode::Wait;
}

void ControlMain::robotCartesianMove(char mode, double desCartesian[NUM_DOF])
{
    dataControl->cartesianPoseScaleDown(desCartesian, dataControl->RobotData.desired_end_pose);
    switch(mode){
        case DataControl::Motion::CartesianJogMotion:
            for(unsigned char i = 0; i < NUM_DOF; i++){
                dataControl->RobotData.desired_end_pose[i] += dataControl->RobotData.present_end_pose[i];
            }
            break;
        case DataControl::Motion::CartesianMotion:

            break;
        default:
            break;
    }

    dataControl->PathData.readyPath.path_x.clear();
    dataControl->PathData.readyPath.path_y.clear();
    dataControl->PathData.readyPath.path_z.clear();
    dataControl->PathData.readyPath.path_theta.clear();

    path_generator(dataControl->RobotData.present_end_pose[0], dataControl->RobotData.desired_end_pose[0],
            dataControl->ClientToServer.move_time, dataControl->ClientToServer.acc_time, 0.005, &dataControl->PathData.readyPath.path_x);
    path_generator(dataControl->RobotData.present_end_pose[1], dataControl->RobotData.desired_end_pose[1],
            dataControl->ClientToServer.move_time, dataControl->ClientToServer.acc_time, 0.005, &dataControl->PathData.readyPath.path_y);
    path_generator(dataControl->RobotData.present_end_pose[2], dataControl->RobotData.desired_end_pose[2],
            dataControl->ClientToServer.move_time, dataControl->ClientToServer.acc_time, 0.005, &dataControl->PathData.readyPath.path_z);

    double R_init[9], R_final[9], r[3], theta;
    RobotArm::rpy2mat(dataControl->RobotData.present_end_pose[5], dataControl->RobotData.present_end_pose[4], dataControl->RobotData.present_end_pose[3], R_init);
    RobotArm::rpy2mat(dataControl->RobotData.desired_end_pose[5], dataControl->RobotData.desired_end_pose[4], dataControl->RobotData.desired_end_pose[3], R_final);
    RobotArm::mat_to_axis_angle(R_init, R_final, r, &theta);
    memcpy(dataControl->PathData.readyPath.r, r, sizeof(double)*3);

    path_generator(0, theta, dataControl->ClientToServer.move_time, dataControl->ClientToServer.acc_time, 0.005, &dataControl->PathData.readyPath.path_theta);
    memcpy(dataControl->PathData.readyPath.R_init, robotArm->body[robotArm->num_body].Ae, sizeof(double)*9);

    dataControl->RobotData.run_mode = DataControl::CmdType::ReadyCmd;
    dataControl->PathData.path_data_indx = 0;

    dataControl->PathData.readyPath.data_size = dataControl->PathData.readyPath.path_x.size();
    rt_printf("Cartesian move path size : %d\n", dataControl->PathData.readyPath.data_size);
    dataControl->ClientToServer.opMode = DataControl::OpMode::RunMode;
}

void ControlMain::robotPathGenerate()
{
    for(uint8_t i = 0; i < dataControl->PathData.row - 1; i++){
        path_generator(dataControl->PathData.point_x[i], dataControl->PathData.point_x[i + 1],
                dataControl->PathData.total_time[i + 1] - dataControl->PathData.total_time[i], dataControl->PathData.acc_time[i], 0.005, &dataControl->PathData.movePath[i].path_x);

        path_generator(dataControl->PathData.point_y[i], dataControl->PathData.point_y[i + 1],
                dataControl->PathData.total_time[i + 1] - dataControl->PathData.total_time[i], dataControl->PathData.acc_time[i], 0.005, &dataControl->PathData.movePath[i].path_y);

        path_generator(dataControl->PathData.point_z[i], dataControl->PathData.point_z[i + 1],
                dataControl->PathData.total_time[i + 1] - dataControl->PathData.total_time[i], dataControl->PathData.acc_time[i], 0.005, &dataControl->PathData.movePath[i].path_z);
    }

    dataControl->PathData.point_theta.push_back(0);
    for(uint8_t i = 0; i < dataControl->PathData.row - 1; i++){
        double R_init[9], R_final[9], r[3], theta;
        RobotArm::rpy2mat(dataControl->PathData.point_yaw[i], dataControl->PathData.point_pitch[i], dataControl->PathData.point_roll[i], R_init);
        RobotArm::rpy2mat(dataControl->PathData.point_yaw[i + 1], dataControl->PathData.point_pitch[i + 1], dataControl->PathData.point_roll[i + 1], R_final);
        RobotArm::mat_to_axis_angle(R_init, R_final, r, &theta);
        memcpy(dataControl->PathData.movePath[i].r, r, sizeof(double)*3);
        dataControl->PathData.point_theta.push_back(theta);

//        rt_printf("r : %f, %f, %f, %f\n", r[0], r[1], r[2], theta);
    }

    for(uint8_t i = 0; i < dataControl->PathData.row - 1; i++){
        path_generator(0, dataControl->PathData.point_theta[i + 1],
                dataControl->PathData.total_time[i + 1] - dataControl->PathData.total_time[i], dataControl->PathData.acc_time[i], 0.005, &dataControl->PathData.movePath[i].path_theta);

        dataControl->PathData.movePath[i].data_size = dataControl->PathData.movePath[i].path_x.size();
        rt_printf("section %d path size : %d\n", i, dataControl->PathData.movePath[i].data_size);
    }

    dataControl->ClientToServer.opMode = DataControl::OpMode::Wait;
}

void ControlMain::robotReady()
{
    path_generator(dataControl->RobotData.present_end_pose[0], dataControl->PathData.point_x[0],
            dataControl->PathData.total_time[1] - dataControl->PathData.total_time[0], dataControl->PathData.acc_time[0], 0.005, &dataControl->PathData.readyPath.path_x);
    path_generator(dataControl->RobotData.present_end_pose[1], dataControl->PathData.point_y[0],
            dataControl->PathData.total_time[1] - dataControl->PathData.total_time[0], dataControl->PathData.acc_time[0], 0.005, &dataControl->PathData.readyPath.path_y);
    path_generator(dataControl->RobotData.present_end_pose[2], dataControl->PathData.point_z[0],
            dataControl->PathData.total_time[1] - dataControl->PathData.total_time[0], dataControl->PathData.acc_time[0], 0.005, &dataControl->PathData.readyPath.path_z);

    double R_init[9], R_final[9], r[3], theta;
    RobotArm::rpy2mat(dataControl->RobotData.present_end_pose[5], dataControl->RobotData.present_end_pose[4], dataControl->RobotData.present_end_pose[3], R_init);
    RobotArm::rpy2mat(dataControl->PathData.point_yaw[0], dataControl->PathData.point_pitch[0], dataControl->PathData.point_roll[0], R_final);
    RobotArm::mat_to_axis_angle(R_init, R_final, r, &theta);
    memcpy(dataControl->PathData.readyPath.r, r, sizeof(double)*3);

    path_generator(0, theta, dataControl->PathData.total_time[1] - dataControl->PathData.total_time[0], dataControl->PathData.acc_time[0], 0.005, &dataControl->PathData.readyPath.path_theta);
    memcpy(dataControl->PathData.readyPath.R_init, robotArm->body[robotArm->num_body].Ae, sizeof(double)*9);

    dataControl->RobotData.run_mode = DataControl::CmdType::ReadyCmd;

    dataControl->PathData.readyPath.data_size = dataControl->PathData.readyPath.path_x.size();
    rt_printf("ready path size : %d\n", dataControl->PathData.readyPath.data_size);
    dataControl->ClientToServer.opMode = DataControl::OpMode::RunMode;

    dataControl->PathData.path_data_indx = 0;
}

void ControlMain::robotRun()
{
    switch(dataControl->RobotData.run_mode){
        case DataControl::CmdType::ReadyCmd:
        {
            rt_printf("path_data_indx : %d\n", dataControl->PathData.path_data_indx);

            dataControl->RobotData.desired_end_pose[0] = dataControl->PathData.readyPath.path_x[dataControl->PathData.path_data_indx];
            dataControl->RobotData.desired_end_pose[1] = dataControl->PathData.readyPath.path_y[dataControl->PathData.path_data_indx];
            dataControl->RobotData.desired_end_pose[2] = dataControl->PathData.readyPath.path_z[dataControl->PathData.path_data_indx];

            double Ri[9], Rd[9];
            RobotArm::axis_angle_to_mat(dataControl->PathData.readyPath.r, dataControl->PathData.readyPath.path_theta[dataControl->PathData.path_data_indx], Ri);
            RobotArm::mat(dataControl->PathData.readyPath.R_init, Ri, 3, 3, 3, 3, Rd);
            RobotArm::mat2rpy(Rd, dataControl->RobotData.desired_end_pose + 3);

//            rt_printf("Desired Pose : %f, %f, %f, %f, %f, %f\n",
//                      dataControl->RobotData.desired_end_pose[0], dataControl->RobotData.desired_end_pose[1],
//                    dataControl->RobotData.desired_end_pose[2], dataControl->RobotData.desired_end_pose[3],
//                    dataControl->RobotData.desired_end_pose[4], dataControl->RobotData.desired_end_pose[5]);

            dataControl->jointPositionENC2RAD(dataControl->RobotData.present_joint_position, dataControl->RobotData.present_q);

            if (dataControl->RobotData.joint_op_mode == JointOpMode::current_mode){
                // Gravity compensation torque
                dataControl->jointVelocityENC2RAD(dataControl->RobotData.present_joint_velocity, dataControl->RobotData.present_q_dot);

                double gravity_compensation_torque[NUM_JOINT] = {0,};
                robotArm->gravity_compensation(dataControl->RobotData.present_q, dataControl->RobotData.present_q_dot, gravity_compensation_torque);

                robotArm->run_kinematics(dataControl->RobotData.present_q, dataControl->RobotData.present_end_pose);
                robotArm->jacobian();

                double end_velocity[6] = {0,};
                for(uint i = 0; i < 6; i++)
                {
                    for(uint j = 0; j < 6; j++)
                    {
                        end_velocity[i] += robotArm->J[i*6 + j]*dataControl->RobotData.present_q_dot[j];
                    }
                }

                double des_A[9], pre_A[9];
                RobotArm::rpy2mat(dataControl->RobotData.present_end_pose[5], dataControl->RobotData.present_end_pose[4], dataControl->RobotData.present_end_pose[3], pre_A);
                RobotArm::rpy2mat(dataControl->RobotData.desired_end_pose[5], dataControl->RobotData.desired_end_pose[4], dataControl->RobotData.desired_end_pose[3], des_A);
                double diff_A[9] = {0,};
                for(uint i = 0; i < 9; i++){
                    diff_A[i] = des_A[i] - pre_A[i];
                }
                double diff_ori[3];
                RobotArm::mat2rpy(diff_A, diff_ori);

                double Kp = 800*0;
                double Dp = 15*0;
                double Kr = 1000;
                double Dr = 0;

                double F[NUM_DOF] = {0,};

                rt_printf("Err Pose : %f, %f, %f, %f, %f, %f\n",
                        dataControl->RobotData.desired_end_pose[0] - dataControl->RobotData.present_end_pose[0],
                        dataControl->RobotData.desired_end_pose[1] - dataControl->RobotData.present_end_pose[1],
                        dataControl->RobotData.desired_end_pose[2] - dataControl->RobotData.present_end_pose[2],
                        dataControl->RobotData.desired_end_pose[3] - dataControl->RobotData.present_end_pose[3],
                        dataControl->RobotData.desired_end_pose[4] - dataControl->RobotData.present_end_pose[4],
                        dataControl->RobotData.desired_end_pose[5] - dataControl->RobotData.present_end_pose[5]);

                rt_printf("Err Pose : %f, %f, %f, %f, %f, %f\n",
                          dataControl->RobotData.desired_end_pose[0] - dataControl->RobotData.present_end_pose[0],
                          dataControl->RobotData.desired_end_pose[1] - dataControl->RobotData.present_end_pose[1],
                          dataControl->RobotData.desired_end_pose[2] - dataControl->RobotData.present_end_pose[2],
                        diff_ori[0],
                        diff_ori[1],
                        diff_ori[2]);

                rt_printf("Present Pose : %f, %f, %f, %f, %f, %f\n",
                        dataControl->RobotData.present_end_pose[0],
                        dataControl->RobotData.present_end_pose[1],
                        dataControl->RobotData.present_end_pose[2],
                        dataControl->RobotData.present_end_pose[3],
                        dataControl->RobotData.present_end_pose[4],
                        dataControl->RobotData.present_end_pose[5]);

                rt_printf("Desired Pose : %f, %f, %f, %f, %f, %f\n",
                        dataControl->RobotData.desired_end_pose[0],
                        dataControl->RobotData.desired_end_pose[1],
                        dataControl->RobotData.desired_end_pose[2],
                        dataControl->RobotData.desired_end_pose[3],
                        dataControl->RobotData.desired_end_pose[4],
                        dataControl->RobotData.desired_end_pose[5]);

                F[0] = Kp*(dataControl->RobotData.desired_end_pose[0] - dataControl->RobotData.present_end_pose[0]) - Dp*(end_velocity[0]);
                F[1] = Kp*(dataControl->RobotData.desired_end_pose[1] - dataControl->RobotData.present_end_pose[1]) - Dp*(end_velocity[1]);
                F[2] = Kp*(dataControl->RobotData.desired_end_pose[2] - dataControl->RobotData.present_end_pose[2]) - Dp*(end_velocity[2]);
                F[3] = Kr*(diff_ori[0]) - Dr*(end_velocity[3]);
                F[4] = 0*Kr*(diff_ori[1]) - Dr*(end_velocity[4]);
                F[5] = 0*Kr*(diff_ori[2]) - Dr*(end_velocity[5]);

                double T[NUM_JOINT] = {0,};
                double alpha = 1.0;
                for(uint i = 0; i < 6; i++){
                    for(uint j = 0; j < 6; j++){
                        T[i] += robotArm->J[j*6 + i]*F[j];
                    }
                    T[i] += gravity_compensation_torque[i]*alpha;
                }

                double goal_current[NUM_JOINT] = {0,};
                for(uint i = 0; i < NUM_JOINT; i++){
                    if (i < 3){
                        goal_current[i] = 1000*(T[i] / TORQUE_CONSTANT_W270);
                    }
                    else{
                        goal_current[i] = 1000*(T[i] / TORQUE_CONSTANT_W150);
                    }
                }

                rt_printf("Desired Force : %f, %f, %f, %f, %f, %f\n", F[0], F[1], F[2], F[3], F[4], F[5]);
                rt_printf("Desired Torque : %f, %f, %f, %f, %f, %f\n", T[0], T[1], T[2], T[3], T[4], T[5]);

                dataControl->jointCurrentmA2RAW(goal_current, dataControl->RobotData.command_joint_current);

//                module->setGroupSyncWriteGoalCurrent(dataControl->RobotData.command_joint_current, NUM_JOINT);
            }
            else if (dataControl->RobotData.joint_op_mode == JointOpMode::extended_position_mode){
                dataControl->RobotData.ik_time1 = static_cast<unsigned long>(rt_timer_read());
                robotArm->run_inverse_kinematics(dataControl->RobotData.present_q, dataControl->RobotData.desired_end_pose,
                                                 dataControl->RobotData.desired_q, dataControl->RobotData.present_end_pose);
                dataControl->RobotData.ik_time2 = static_cast<unsigned long>(rt_timer_read());

                dataControl->cartesianPoseScaleUp(dataControl->RobotData.present_end_pose, dataControl->RobotData.present_cal_end_pose);

                dataControl->jointPositionRAD2ENC(dataControl->RobotData.desired_q, dataControl->RobotData.command_joint_position);

                module->setGroupSyncWriteGoalPosition(dataControl->RobotData.command_joint_position, NUM_JOINT);
            }

            goalReach(dataControl->RobotData.desired_end_pose, dataControl->RobotData.present_end_pose, &dataControl->cartesian_goal_reach);

            if (dataControl->cartesian_goal_reach){
                dataControl->PathData.path_data_indx += 1;
            }

            if (dataControl->PathData.path_data_indx >= dataControl->PathData.readyPath.data_size - 1){
                dataControl->PathData.path_data_indx = 0;
                dataControl->ClientToServer.opMode = DataControl::OpMode::Wait;

                memcpy(dataControl->PathData.movePath[0].R_init, Rd, sizeof(double)*9);
            }

            break;
        }
        case DataControl::CmdType::RunCmd:
        {
            dataControl->RobotData.desired_end_pose[0] = dataControl->PathData.movePath[dataControl->PathData.path_struct_indx].path_x[dataControl->PathData.path_data_indx];
            dataControl->RobotData.desired_end_pose[1] = dataControl->PathData.movePath[dataControl->PathData.path_struct_indx].path_y[dataControl->PathData.path_data_indx];
            dataControl->RobotData.desired_end_pose[2] = dataControl->PathData.movePath[dataControl->PathData.path_struct_indx].path_z[dataControl->PathData.path_data_indx];

            double Ri[9], Rd[9];
            RobotArm::axis_angle_to_mat(dataControl->PathData.movePath[dataControl->PathData.path_struct_indx].r,
                    dataControl->PathData.movePath[dataControl->PathData.path_struct_indx].path_theta[dataControl->PathData.path_data_indx], Ri);
            RobotArm::mat(dataControl->PathData.movePath[dataControl->PathData.path_struct_indx].R_init, Ri, 3, 3, 3, 3, Rd);
            RobotArm::mat2rpy(Rd, dataControl->RobotData.desired_end_pose + 3);

            dataControl->jointPositionENC2RAD(dataControl->RobotData.present_joint_position, dataControl->RobotData.present_q);

//            rt_printf("Present Position : %d, %d, %d, %d, %d, %d\n",
//                      dataControl->RobotData.present_joint_position[0], dataControl->RobotData.present_joint_position[1],
//                    dataControl->RobotData.present_joint_position[2], dataControl->RobotData.present_joint_position[3],
//                    dataControl->RobotData.present_joint_position[4], dataControl->RobotData.present_joint_position[5]);

//            rt_printf("Desired Pose : %f, %f, %f, %f, %f, %f\n",
//                      dataControl->RobotData.desired_end_pose[0], dataControl->RobotData.desired_end_pose[1],
//                    dataControl->RobotData.desired_end_pose[2], dataControl->RobotData.desired_end_pose[3],
//                    dataControl->RobotData.desired_end_pose[4], dataControl->RobotData.desired_end_pose[5]);

//            rt_printf("Present Pose : %f, %f, %f, %f, %f, %f\n",
//                      dataControl->RobotData.present_end_pose[0], dataControl->RobotData.present_end_pose[1],
//                    dataControl->RobotData.present_end_pose[2], dataControl->RobotData.present_end_pose[3],
//                    dataControl->RobotData.present_end_pose[4], dataControl->RobotData.present_end_pose[5]);

            if (dataControl->RobotData.joint_op_mode == JointOpMode::current_mode){
                // Gravity compensation torque
                dataControl->jointVelocityENC2RAD(dataControl->RobotData.present_joint_velocity, dataControl->RobotData.present_q_dot);

                double gravity_compensation_torque[NUM_JOINT] = {0,};
                robotArm->gravity_compensation(dataControl->RobotData.present_q, dataControl->RobotData.present_q_dot, gravity_compensation_torque);

                robotArm->run_kinematics(dataControl->RobotData.present_q, dataControl->RobotData.present_end_pose);
                robotArm->jacobian();

                double end_velocity[6] = {0,};
                for(uint i = 0; i < 6; i++)
                {
                    for(uint j = 0; j < 6; j++)
                    {
                        end_velocity[i] += robotArm->J[i*6 + j]*dataControl->RobotData.present_q_dot[j];
                    }
                }

                double Kp = 800;
                double Dp = 15;
                double Kr = 0;
                double Dr = 0;

                double F[NUM_DOF] = {0,};

                rt_printf("Err Pose : %f, %f, %f, %f, %f, %f\n",
                        dataControl->RobotData.desired_end_pose[0] - dataControl->RobotData.present_end_pose[0],
                        dataControl->RobotData.desired_end_pose[1] - dataControl->RobotData.present_end_pose[1],
                        dataControl->RobotData.desired_end_pose[2] - dataControl->RobotData.present_end_pose[2],
                        dataControl->RobotData.desired_end_pose[3] - dataControl->RobotData.present_end_pose[3],
                        dataControl->RobotData.desired_end_pose[4] - dataControl->RobotData.present_end_pose[4],
                        dataControl->RobotData.desired_end_pose[5] - dataControl->RobotData.present_end_pose[5]);

                rt_printf("Present Pose : %f, %f, %f, %f, %f, %f\n",
                        dataControl->RobotData.present_end_pose[0],
                        dataControl->RobotData.present_end_pose[1],
                        dataControl->RobotData.present_end_pose[2],
                        dataControl->RobotData.present_end_pose[3],
                        dataControl->RobotData.present_end_pose[4],
                        dataControl->RobotData.present_end_pose[5]);

                rt_printf("Desired Pose : %f, %f, %f, %f, %f, %f\n",
                        dataControl->RobotData.desired_end_pose[0],
                        dataControl->RobotData.desired_end_pose[1],
                        dataControl->RobotData.desired_end_pose[2],
                        dataControl->RobotData.desired_end_pose[3],
                        dataControl->RobotData.desired_end_pose[4],
                        dataControl->RobotData.desired_end_pose[5]);

                F[0] = Kp*(dataControl->RobotData.desired_end_pose[0] - dataControl->RobotData.present_end_pose[0]) - Dp*(end_velocity[0]);
                F[1] = Kp*(dataControl->RobotData.desired_end_pose[1] - dataControl->RobotData.present_end_pose[1]) - Dp*(end_velocity[1]);
                F[2] = Kp*(dataControl->RobotData.desired_end_pose[2] - dataControl->RobotData.present_end_pose[2]) - Dp*(end_velocity[2]);
                F[3] = Kr*(dataControl->RobotData.desired_end_pose[3] - dataControl->RobotData.present_end_pose[3]) - Dr*(end_velocity[3]);
                F[4] = Kr*(dataControl->RobotData.desired_end_pose[4] - dataControl->RobotData.present_end_pose[4]) - Dr*(end_velocity[4]);
                F[5] = Kr*(dataControl->RobotData.desired_end_pose[5] - dataControl->RobotData.present_end_pose[5]) - Dr*(end_velocity[5]);

                double T[NUM_JOINT] = {0,};
                double alpha = 1.0;
                for(uint i = 0; i < 6; i++){
                    for(uint j = 0; j < 6; j++){
                        T[i] += robotArm->J[j*6 + i]*F[j];
                    }
                    T[i] += gravity_compensation_torque[i]*alpha;
                }

                double goal_current[NUM_JOINT] = {0,};
                for(uint i = 0; i < NUM_JOINT; i++){
                    if (i < 3){
                        goal_current[i] = 1000*(T[i] / TORQUE_CONSTANT_W270);
                    }
                    else{
                        goal_current[i] = 1000*(T[i] / TORQUE_CONSTANT_W150);
                    }
                }

                rt_printf("Desired Force : %f, %f, %f, %f, %f, %f\n", F[0], F[1], F[2], F[3], F[4], F[5]);
                rt_printf("Desired Torque : %f, %f, %f, %f, %f, %f\n", T[0], T[1], T[2], T[3], T[4], T[5]);

                dataControl->jointCurrentmA2RAW(goal_current, dataControl->RobotData.command_joint_current);

//                module->setGroupSyncWriteGoalCurrent(dataControl->RobotData.command_joint_current, NUM_JOINT);
            }
            else if(dataControl->RobotData.joint_op_mode == JointOpMode::extended_position_mode){
                dataControl->RobotData.ik_time1 = static_cast<unsigned long>(rt_timer_read());
                robotArm->run_inverse_kinematics(dataControl->RobotData.present_q, dataControl->RobotData.desired_end_pose,
                                                 dataControl->RobotData.desired_q, dataControl->RobotData.present_end_pose);
                dataControl->RobotData.ik_time2 = static_cast<unsigned long>(rt_timer_read());

                dataControl->cartesianPoseScaleUp(dataControl->RobotData.present_end_pose, dataControl->RobotData.present_cal_end_pose);

                dataControl->jointPositionRAD2ENC(dataControl->RobotData.desired_q, dataControl->RobotData.command_joint_position);

//                rt_printf("Desired Joint : ");
//                for(int i = 0; i < 6; i++){
//                    rt_printf("%f\t", dataControl->RobotData.desired_q[i]);
//                }
//                rt_printf("\n");

                if (delay_cnt == 0){
                    module->setGroupSyncWriteGoalPosition(dataControl->RobotData.command_joint_position, NUM_JOINT);
                }
            }

            if (delay_cnt == 0){
                rt_printf("path_data_indx : %d\n", dataControl->PathData.path_data_indx);
                rt_printf("path_indx : %d\n", dataControl->PathData.path_struct_indx);

                goalReach(dataControl->RobotData.desired_end_pose, dataControl->RobotData.present_end_pose, &dataControl->cartesian_goal_reach);

                if (dataControl->cartesian_goal_reach){
                    dataControl->PathData.path_data_indx += 1;
                }
            }

            if (dataControl->PathData.path_data_indx >= dataControl->PathData.movePath[dataControl->PathData.path_struct_indx].data_size - 1){
                if (delay_cnt >= delay_cnt_max){
                    fork_cnt++;
                    if (fork_cnt <= 1000){
                        module->setGoalPosition(5, dataControl->RobotData.present_joint_position[5] - 26);
                    }
                    else if (fork_cnt >= 2000 && fork_cnt <= 3000){
                        module->setGoalPosition(5, dataControl->RobotData.present_joint_position[5] + 26);
                    }
                    if (fork_cnt >= fork_cnt_max){
                        dataControl->PathData.path_data_indx = 0;
                        dataControl->PathData.path_struct_indx++;
                        memcpy(dataControl->PathData.movePath[dataControl->PathData.path_struct_indx].R_init, Rd, sizeof(double)*9);
                        delay_cnt = 0;
                    }
                }
                else{
                    delay_cnt++;
                }

                if (dataControl->PathData.path_struct_indx >= dataControl->PathData.row - 1){
                    if(dataControl->PathData.cycle_count_max == -1)
                    {
                        dataControl->PathData.path_data_indx = 0;
                        dataControl->PathData.path_struct_indx = 0;
                        memcpy(dataControl->PathData.movePath[dataControl->PathData.path_struct_indx].R_init, Rd, sizeof(double)*9);
                    }
                    else
                    {
                        dataControl->feeding = false;
                        rt_printf("%d\n", dataControl->feeding);
                        dataControl->PathData.path_data_indx = 0;
                        dataControl->PathData.path_struct_indx = 0;
                        dataControl->ClientToServer.opMode = DataControl::OpMode::Wait;
                    }
                }
            }

            break;
        }
        case DataControl::CmdType::FileReady:
        {
            dataControl->PathData.readyPath.path_x.clear();
            dataControl->PathData.readyPath.path_y.clear();
            dataControl->PathData.readyPath.path_z.clear();
            dataControl->PathData.readyPath.path_theta.clear();

            path_generator(dataControl->RobotData.present_end_pose[0], dataControl->PathData.file_data[1],
                    1.0, 0.3, 0.005, &dataControl->PathData.readyPath.path_x);
            path_generator(dataControl->RobotData.present_end_pose[1], dataControl->PathData.file_data[2],
                    1.0, 0.3, 0.005, &dataControl->PathData.readyPath.path_y);
            path_generator(dataControl->RobotData.present_end_pose[2], dataControl->PathData.file_data[3],
                    1.0, 0.3, 0.005, &dataControl->PathData.readyPath.path_z);

            double R_init[9], R_final[9], r[3], theta;
            RobotArm::rpy2mat(dataControl->RobotData.present_end_pose[5], dataControl->RobotData.present_end_pose[4], dataControl->RobotData.present_end_pose[3], R_init);
            RobotArm::rpy2mat(dataControl->PathData.file_data[6], dataControl->PathData.file_data[5], dataControl->PathData.file_data[4], R_final);
            RobotArm::mat_to_axis_angle(R_init, R_final, r, &theta);
            memcpy(dataControl->PathData.readyPath.r, r, sizeof(double)*3);

            path_generator(0, theta, 1.0, 0.3, 0.005, &dataControl->PathData.readyPath.path_theta);
            memcpy(dataControl->PathData.readyPath.R_init, robotArm->body[robotArm->num_body].Ae, sizeof(double)*9);

            dataControl->RobotData.run_mode = DataControl::CmdType::ReadyCmd;
            dataControl->PathData.path_data_indx = 0;

            dataControl->PathData.readyPath.data_size = dataControl->PathData.readyPath.path_x.size();

            dataControl->ClientToServer.opMode = DataControl::OpMode::RunMode;
            break;
        }
        case DataControl::CmdType::FileRun:
        {
            rt_printf("path_data_indx : %d\n", dataControl->PathData.path_data_indx);

            dataControl->RobotData.desired_end_pose[0] = dataControl->PathData.file_data[dataControl->PathData.path_data_indx*7 + 1];
            dataControl->RobotData.desired_end_pose[1] = dataControl->PathData.file_data[dataControl->PathData.path_data_indx*7 + 2];
            dataControl->RobotData.desired_end_pose[2] = dataControl->PathData.file_data[dataControl->PathData.path_data_indx*7 + 3];
            dataControl->RobotData.desired_end_pose[3] = dataControl->PathData.file_data[dataControl->PathData.path_data_indx*7 + 4];
            dataControl->RobotData.desired_end_pose[4] = dataControl->PathData.file_data[dataControl->PathData.path_data_indx*7 + 5];
            dataControl->RobotData.desired_end_pose[5] = dataControl->PathData.file_data[dataControl->PathData.path_data_indx*7 + 6];

            dataControl->jointPositionENC2RAD(dataControl->RobotData.present_joint_position, dataControl->RobotData.present_q);

            dataControl->RobotData.ik_time1 = static_cast<unsigned long>(rt_timer_read());
            robotArm->run_inverse_kinematics(dataControl->RobotData.present_q, dataControl->RobotData.desired_end_pose,
                                             dataControl->RobotData.desired_q, dataControl->RobotData.present_end_pose);
            dataControl->RobotData.ik_time2 = static_cast<unsigned long>(rt_timer_read());

            dataControl->cartesianPoseScaleUp(dataControl->RobotData.present_end_pose, dataControl->RobotData.present_cal_end_pose);

            dataControl->jointPositionRAD2ENC(dataControl->RobotData.desired_q, dataControl->RobotData.command_joint_position);

            module->setGroupSyncWriteGoalPosition(dataControl->RobotData.command_joint_position, NUM_JOINT);

            goalReach(dataControl->RobotData.desired_end_pose, dataControl->RobotData.present_end_pose, &dataControl->cartesian_goal_reach);

            if (dataControl->cartesian_goal_reach){
                dataControl->PathData.path_data_indx += 1;
            }

            if (dataControl->PathData.path_data_indx >= dataControl->PathData.file_data.size()/7 - 1){
                if(dataControl->PathData.cycle_count_max == -1)
                {
                    dataControl->PathData.path_data_indx = 0;
                    dataControl->PathData.path_struct_indx = 0;
                }
                else
                {
                    dataControl->PathData.path_data_indx = 0;
                    dataControl->PathData.path_struct_indx = 0;
                    dataControl->ClientToServer.opMode = DataControl::OpMode::Wait;
                }
            }
            break;
        }
        case DataControl::CmdType::CustomRun:
        {
            dataControl->RobotData.desired_end_pose[0] = dataControl->PathData.movePath[dataControl->PathData.path_struct_indx].path_x[dataControl->PathData.path_data_indx];
            dataControl->RobotData.desired_end_pose[1] = dataControl->PathData.movePath[dataControl->PathData.path_struct_indx].path_y[dataControl->PathData.path_data_indx];
            dataControl->RobotData.desired_end_pose[2] = dataControl->PathData.movePath[dataControl->PathData.path_struct_indx].path_z[dataControl->PathData.path_data_indx];

            double Ri[9], Rd[9];
            RobotArm::axis_angle_to_mat(dataControl->PathData.movePath[dataControl->PathData.path_struct_indx].r,
                    dataControl->PathData.movePath[dataControl->PathData.path_struct_indx].path_theta[dataControl->PathData.path_data_indx], Ri);
            RobotArm::mat(dataControl->PathData.movePath[dataControl->PathData.path_struct_indx].R_init, Ri, 3, 3, 3, 3, Rd);
            RobotArm::mat2rpy(Rd, dataControl->RobotData.desired_end_pose + 3);

//            1.570796	0.000000	-2.094399
            dataControl->RobotData.desired_end_pose[3] = 1.570796;
            dataControl->RobotData.desired_end_pose[4] = 0.000000;
            dataControl->RobotData.desired_end_pose[5] = -2.094399;

//            rt_printf("Desired Pose : %f, %f, %f, %f, %f, %f\n",
//                      dataControl->RobotData.desired_end_pose[0], dataControl->RobotData.desired_end_pose[1],
//                    dataControl->RobotData.desired_end_pose[2], dataControl->RobotData.desired_end_pose[3],
//                    dataControl->RobotData.desired_end_pose[4], dataControl->RobotData.desired_end_pose[5]);

//            rt_printf("Present Pose : %f, %f, %f, %f, %f, %f\n",
//                      dataControl->RobotData.present_end_pose[0], dataControl->RobotData.present_end_pose[1],
//                    dataControl->RobotData.present_end_pose[2], dataControl->RobotData.present_end_pose[3],
//                    dataControl->RobotData.present_end_pose[4], dataControl->RobotData.present_end_pose[5]);

            dataControl->jointPositionENC2RAD(dataControl->RobotData.present_joint_position, dataControl->RobotData.present_q);

            dataControl->RobotData.ik_time1 = static_cast<unsigned long>(rt_timer_read());
            robotArm->run_inverse_kinematics(dataControl->RobotData.present_q, dataControl->RobotData.desired_end_pose,
                                             dataControl->RobotData.desired_q, dataControl->RobotData.present_end_pose);
            dataControl->RobotData.ik_time2 = static_cast<unsigned long>(rt_timer_read());

            dataControl->cartesianPoseScaleUp(dataControl->RobotData.present_end_pose, dataControl->RobotData.present_cal_end_pose);

            dataControl->jointPositionRAD2ENC(dataControl->RobotData.desired_q, dataControl->RobotData.command_joint_position);

            if (delay_cnt == 0){
//                if (dataControl->PathData.path_struct_indx <= 3){
                    module->setGroupSyncWriteGoalPosition(dataControl->RobotData.command_joint_position, NUM_JOINT);
//                }
//                else{
//                    double angle[6] = {0.77811285, -0.31936499, 2.2571916, -1.9378266, 0.26908082, -1.9854194e-014};
//                    dataControl->jointPositionRAD2ENC(angle, dataControl->RobotData.command_joint_position);
//                    module->setGroupSyncWriteGoalPosition(dataControl->RobotData.command_joint_position, NUM_JOINT);
//                }

                rt_printf("path_data_indx : %d\n", dataControl->PathData.path_data_indx);
                rt_printf("path_indx : %d\n", dataControl->PathData.path_struct_indx);
            }

            if (delay_cnt == 0){
                goalReach(dataControl->RobotData.desired_end_pose, dataControl->RobotData.present_end_pose, &dataControl->cartesian_goal_reach);

                if (dataControl->cartesian_goal_reach){
                    dataControl->PathData.path_data_indx += 1;
                }
            }

            if (dataControl->PathData.path_data_indx >= dataControl->PathData.movePath[dataControl->PathData.path_struct_indx].data_size-1){
                if (delay_cnt >= delay_cnt_max){
                    dataControl->PathData.path_data_indx = 0;
                    dataControl->PathData.path_struct_indx++;
                    delay_cnt = 0;
                }
                else{
                    delay_cnt++;
                }

                if (dataControl->PathData.path_struct_indx >= dataControl->PathData.row - 1){
                    if(dataControl->PathData.cycle_count_max == -1)
                    {
                        dataControl->PathData.path_data_indx = 0;
                        dataControl->PathData.path_struct_indx = 0;
                    }
                    else
                    {
                        dataControl->PathData.path_data_indx = 0;
                        dataControl->PathData.path_struct_indx = 0;
                        dataControl->ClientToServer.opMode = DataControl::OpMode::Wait;
                    }
                }
                else{
                    memcpy(dataControl->PathData.movePath[dataControl->PathData.path_struct_indx].R_init, robotArm->body[robotArm->num_body].Ae, sizeof(double)*9);
                }
            }

            break;
        }
        default:
            break;
    }
}

void ControlMain::robotSPGC()
{
	double present_position = 0;
	dataControl->jointPositionENC2RAD(&dataControl->RobotData.present_joint_position[0], &present_position);

	double T = dataControl->torqueIdeData.mass*(9.80665)*0.2*sin(present_position);

    double current = 0;
    double Kt = dataControl->torqueIdeData.torque_constant;
    current = 1000 * T / Kt;

    int16_t goal_current = 0;
    dataControl->jointCurrentmA2RAW(&current, &goal_current);

    module->setGroupSyncWriteGoalCurrent(&goal_current, NUM_JOINT);

	printf("Present Pos : %f, Goal Toruqe : %f, Dxl Current : %d\n", present_position*180/M_PI,T, goal_current);
}

void ControlMain::robotOperate()
{
    switch(dataControl->operateMode.mode){
        case DataControl::Operate::Start:
        {
            memcpy(dataControl->RobotData.desired_q, dataControl->operateReadyJoint, sizeof(double)*NUM_JOINT);
            dataControl->jointPositionRAD2ENC(dataControl->RobotData.desired_q, dataControl->RobotData.command_joint_position);

            module->setGroupSyncWriteGoalPosition(dataControl->RobotData.command_joint_position, NUM_JOINT);
            break;
        }
        case DataControl::Operate::Stop:
        {
            break;
        }
        case DataControl::Operate::StartTeaching:
        {
            dataControl->PathData.total_time.clear();
            dataControl->PathData.point_x.clear();
            dataControl->PathData.point_y.clear();
            dataControl->PathData.point_z.clear();
            dataControl->PathData.point_roll.clear();
            dataControl->PathData.point_pitch.clear();
            dataControl->PathData.point_yaw.clear();
            dataControl->PathData.point_theta.clear();
            dataControl->PathData.acc_time.clear();

            dataControl->PathData.row = 3;
            for(uint i = 0; i < dataControl->PathData.row; i++){
                dataControl->PathData.movePath[i].path_x.clear();
                dataControl->PathData.movePath[i].path_y.clear();
                dataControl->PathData.movePath[i].path_z.clear();
                dataControl->PathData.movePath[i].path_theta.clear();
            }

            dataControl->PathData.total_time.push_back(0);
            dataControl->PathData.point_x.push_back(dataControl->operateFeedingReadyPose[0]);
            dataControl->PathData.point_y.push_back(dataControl->operateFeedingReadyPose[1]);
            dataControl->PathData.point_z.push_back(dataControl->operateFeedingReadyPose[2]);
            dataControl->PathData.point_roll.push_back(dataControl->operateFeedingReadyPose[3]);
            dataControl->PathData.point_pitch.push_back(dataControl->operateFeedingReadyPose[4]);
            dataControl->PathData.point_yaw.push_back(dataControl->operateFeedingReadyPose[5]);
            dataControl->PathData.acc_time.push_back(0.5);

            module->setGroupSyncWriteTorqueEnable(0, NUM_JOINT);
            module->setGroupSyncWriteOperatingMode(JointOpMode::current_mode, NUM_JOINT);
            module->setGroupSyncWriteTorqueEnable(1, NUM_JOINT);
            dataControl->RobotData.joint_op_mode = JointOpMode::current_mode;
            break;
        }
        case DataControl::Operate::StopTeaching:
        {
            dataControl->PathData.total_time.push_back(3);
            dataControl->PathData.point_x.push_back(dataControl->RobotData.present_end_pose[0]);
            dataControl->PathData.point_y.push_back(dataControl->RobotData.present_end_pose[1]);
            dataControl->PathData.point_z.push_back(dataControl->RobotData.present_end_pose[2]);
            dataControl->PathData.point_roll.push_back(dataControl->RobotData.present_end_pose[3]);
            dataControl->PathData.point_pitch.push_back(dataControl->RobotData.present_end_pose[4]);
            dataControl->PathData.point_yaw.push_back(dataControl->RobotData.present_end_pose[5]);
            dataControl->PathData.acc_time.push_back(0.5);

            dataControl->PathData.total_time.push_back(6);
            dataControl->PathData.point_x.push_back(dataControl->operateReadyPose[0]);
            dataControl->PathData.point_y.push_back(dataControl->operateReadyPose[1]);
            dataControl->PathData.point_z.push_back(dataControl->operateReadyPose[2]);
            dataControl->PathData.point_roll.push_back(dataControl->operateReadyPose[3]);
            dataControl->PathData.point_pitch.push_back(dataControl->operateReadyPose[4]);
            dataControl->PathData.point_yaw.push_back(dataControl->operateReadyPose[5]);
            dataControl->PathData.acc_time.push_back(0.5);

            module->setGroupSyncWriteTorqueEnable(0, NUM_JOINT);
            module->setGroupSyncWriteOperatingMode(JointOpMode::extended_position_mode, NUM_JOINT);
            module->setGroupSyncWriteIndirectAddress(init_profile_acc, init_profile_vel, init_vel_limit, init_pos_p_gain, NUM_JOINT);
            module->setGroupSyncWriteTorqueEnable(1, NUM_JOINT);
            dataControl->RobotData.joint_op_mode = JointOpMode::extended_position_mode;

            for(int8_t i = 0; i < dataControl->PathData.row; i ++){
                rt_printf("%f, %f, %f, %f, %f, %f, %f, %f\n",
                          dataControl->PathData.total_time[i], dataControl->PathData.point_x[i], dataControl->PathData.point_y[i], dataControl->PathData.point_z[i], dataControl->PathData.point_roll[i], dataControl->PathData.point_pitch[i], dataControl->PathData.point_yaw[i], dataControl->PathData.acc_time[i]);
            }

            robotPathGenerate();
            break;
        }
        case DataControl::Operate::StartFeeding:
        {
            memcpy(dataControl->RobotData.desired_q, dataControl->operateReadyJoint, sizeof(double)*NUM_JOINT);
            dataControl->jointPositionRAD2ENC(dataControl->RobotData.desired_q, dataControl->RobotData.command_joint_position);

            module->setGroupSyncWriteGoalPosition(dataControl->RobotData.command_joint_position, NUM_JOINT);

//            module->setGroupSyncWriteIndirectAddress(default_profile_acc, default_profile_vel, default_vel_limit, default_pos_p_gain, NUM_JOINT);
            break;
        }
        case DataControl::Operate::StopFeeding:
        {
            break;
        }
        case DataControl::Operate::Feeding:
        {
            module->setGroupSyncWriteIndirectAddress(default_profile_acc, default_profile_vel, default_vel_limit, default_pos_p_gain, NUM_JOINT);
            int interval = 2;
            dataControl->feeding = true;
            switch(dataControl->operateMode.section){
                case DataControl::Section::Side1:
                {
                    rt_printf("path_data_indx : %d\n", dataControl->side1_motion.path_data_indx);

                    for(uint i = 0; i < NUM_JOINT; i++){
                        dataControl->RobotData.desired_q[i] = dataControl->side1_motion.file_data[dataControl->side1_motion.path_data_indx*32 + i + 2];
                    }

                    dataControl->side1_motion.path_data_indx += interval;
                    if (dataControl->side1_motion.path_data_indx >= 2000){
                        dataControl->side1_motion.path_data_indx = 0;
                        dataControl->ClientToServer.opMode = DataControl::OpMode::Wait;
                        dataControl->feeding = false;
                    }
                    fork_cnt_max = 3000;
                    fork_cnt = 0;

                    break;
                }
                case DataControl::Section::Side2:
                {
                    rt_printf("path_data_indx : %d\n", dataControl->side2_motion.path_data_indx);

                    for(uint i = 0; i < NUM_JOINT; i++){
                        dataControl->RobotData.desired_q[i] = dataControl->side2_motion.file_data[dataControl->side2_motion.path_data_indx*32 + i + 2];
                    }

                    dataControl->side2_motion.path_data_indx += interval;
                    if (dataControl->side2_motion.path_data_indx >= 2000){
                        dataControl->side2_motion.path_data_indx = 0;
                        dataControl->ClientToServer.opMode = DataControl::OpMode::Wait;
                        dataControl->feeding = false;
                    }
                    fork_cnt_max = 3000;
                    fork_cnt = 0;

                    break;
                }
                case DataControl::Section::Side3:
                {
                    rt_printf("path_data_indx : %d\n", dataControl->side3_motion.path_data_indx);

                    for(uint i = 0; i < NUM_JOINT; i++){
                        dataControl->RobotData.desired_q[i] = dataControl->side3_motion.file_data[dataControl->side3_motion.path_data_indx*32 + i + 2];
                    }

                    dataControl->side3_motion.path_data_indx += interval;
                    if (dataControl->side3_motion.path_data_indx >= 2000){
                        dataControl->side3_motion.path_data_indx = 0;
                        dataControl->ClientToServer.opMode = DataControl::OpMode::Wait;
                        dataControl->feeding = false;
                    }
                    fork_cnt_max = 3000;
                    fork_cnt = 0;

                    break;
                }
                case DataControl::Section::Rise:
                {
                    rt_printf("path_data_indx : %d\n", dataControl->rise_motion.path_data_indx);

                    for(uint i = 0; i < NUM_JOINT; i++){
                        dataControl->RobotData.desired_q[i] = dataControl->rise_motion.file_data[dataControl->rise_motion.path_data_indx*32 + i + 2];
                    }

                    dataControl->rise_motion.path_data_indx += interval;
                    if (dataControl->rise_motion.path_data_indx >= 2000){
                        dataControl->rise_motion.path_data_indx = 0;

//                        dataControl->ClientToServer.opMode = DataControl::OpMode::Wait;
                        dataControl->operateMode.section = DataControl::Section::Mouse;
                    }

                    break;
                }
                case DataControl::Section::Soup:
                {
                    rt_printf("path_data_indx : %d\n", dataControl->soup_motion.path_data_indx);

                    for(uint i = 0; i < NUM_JOINT; i++){
                        dataControl->RobotData.desired_q[i] = dataControl->soup_motion.file_data[dataControl->soup_motion.path_data_indx*32 + i + 2];
                    }

                    dataControl->soup_motion.path_data_indx += interval;
                    if (dataControl->soup_motion.path_data_indx >= 2000){
                        dataControl->soup_motion.path_data_indx = 0;

//                        dataControl->ClientToServer.opMode = DataControl::OpMode::Wait;
                        dataControl->operateMode.section = DataControl::Section::Mouse;
                    }

                    break;
                }
                case DataControl::Section::Mouse:
                {
                    RobotArm::rpy2mat(dataControl->operateFeedingReadyPose[5], dataControl->operateFeedingReadyPose[4], dataControl->operateFeedingReadyPose[3], dataControl->PathData.movePath[0].R_init);

                    dataControl->ClientToServer.opMode = DataControl::OpMode::RunMode;

                    dataControl->RobotData.run_mode = DataControl::CmdType::RunCmd;
                    dataControl->PathData.path_data_indx = 0;
                    dataControl->PathData.path_struct_indx = 0;
                    dataControl->PathData.cycle_count_max = 1;
                    delay_cnt = 0;

                    break;
                }
            }
//            for(int i = 0; i < 6; i++){
//                rt_printf("%f\t", dataControl->RobotData.desired_q[i]);
//            }
//            rt_printf("\n");
            dataControl->jointPositionRAD2ENC(dataControl->RobotData.desired_q, dataControl->RobotData.command_joint_position);

            module->setGroupSyncWriteGoalPosition(dataControl->RobotData.command_joint_position, NUM_JOINT);
            break;
        }
    }
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

void ControlMain::getPresentEnc(int32_t enc[])
{
    memcpy(enc, dataControl->RobotData.present_joint_position, sizeof(int32_t)*NUM_JOINT);
}

void ControlMain::setOffsetEnc(int32_t enc[]){
    memcpy(dataControl->RobotData.offset, enc, sizeof(int32_t)*NUM_JOINT);
    dataControl->RobotData.offset_setting = true;
    printf("Setting offset enc pulse\n");
}
