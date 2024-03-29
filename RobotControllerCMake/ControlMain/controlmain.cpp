#include "controlmain.h"

ControlMain::ControlMain()
{
    dataControl = new DataControl();
    tcpServer = new TcpServer(dataControl);

    dataControl->RobotData.module_init = false;
    module = new DxlControl();

    data_indx = 0;
    module_indx = 0;
    step_size = 0.003;

    robotArm = new RobotArm(NUM_JOINT, NUM_DOF, step_size);

    old_end_pose_update = false;

    ready_pose = false;
    cartesian_move_flag = false;

    delay_cnt_max = 500;
    delay_cnt = 0;
    fork_cnt_max = 0;
    fork_cnt = 0;
    dataControl->feeding = false;

    controlMainCustom = new ControlMainCustom();

    init_thread_run = false;
}

ControlMain::~ControlMain()
{
    printf("finishing...\n");

    if(init_thread_run){
        init_thread_run = false;
        pthread_cancel(init_thread);
        printf("Finished Module Init Thread\n");
        usleep(10000);
    }

    robot_RT_stop();
    usleep(10000);
    delete controlMainCustom;
    printf("Complete destructure ControlMainCustom\n");
    usleep(10000);
    delete tcpServer;
    usleep(10000);
    printf("Complete destructure TcpServer\n");
    delete dataControl;
    printf("Complete destructure DataControl\n");
    delete robotArm;
    printf("Complete destructure RobotArm\n");

    usleep(10000);
    if (dataControl->RobotData.module_init){
//        if(dataControl->obi_mode){
//            double joint[6] = {0, 0, 0, -90, 0, 0};
//            dataControl->jointPositionDEG2ENC(joint, dataControl->RobotData.command_joint_position);
//            module->setGroupSyncWriteGoalPosition(dataControl->RobotData.command_joint_position, NUM_JOINT);
//            usleep(1000000);
//        }
        for(uint8_t i = 0; i < NUM_JOINT; i++){
            module->dxl_deinit(NUM_JOINT == 1 ? module->single_id : i);
        }
    }
    delete module;
    printf("Complete destructure Dynamixel\n");

    printf("Finished\n");
}

void ControlMain::start()
{
    printf("Start RobotController\n");

    tcpServer->setting(5050);
    tcpServer->start();

    init_thread_run = true;
    pthread_create(&init_thread, nullptr, init_func, this);
}

void ControlMain::setObiMode()
{
    dataControl->obi_mode = true;
    usleep(10000);
    dataControl->RobotData.joint_op_mode = JointOpMode::extended_position_mode;
    dataControl->config_check = true;

    if(!init_thread_run){
        init_thread_run = true;
        pthread_create(&init_thread, nullptr, init_func, this);
    }

    while(!dataControl->RobotData.module_init){
        usleep(1000);
    }
    printf("Init complete\n");

//    while(!this->robot_thread_run){
//        usleep(1000);
//    }
//    rt_printf("Robot RT thread run\n");

//    dataControl->ClientToServer.opMode = DataControl::OpMode::ServoOnOff;
//    dataControl->ClientToServer.subMode = 1;
//    while(!(dataControl->ClientToServer.opMode == DataControl::OpMode::Wait)){
//        usleep(1000);
//    }
//    rt_printf("Servo on\n");

//    dataControl->ClientToServer.opMode = DataControl::OpMode::ServoOnOff;
//    dataControl->ClientToServer.subMode = 1;
//    while(!(dataControl->ClientToServer.opMode == DataControl::OpMode::Wait)){
//        usleep(1000);
//    }
//    rt_printf("Servo on\n");

//    dataControl->ClientToServer.opMode = DataControl::OpMode::ServoOnOff;
//    dataControl->ClientToServer.subMode = 1;
//    while(!(dataControl->ClientToServer.opMode == DataControl::OpMode::Wait)){
//        usleep(1000);
//    }
//    rt_printf("Servo on\n");

////    dataControl->ClientToServer.opMode = DataControl::OpMode::Initialize;
////    while(!(dataControl->ClientToServer.opMode == DataControl::OpMode::Wait)){
////        usleep(1000);
////    }
////    rt_printf("Initialized\n");

//    dataControl->ClientToServer.opMode = DataControl::OpMode::OperateMode;
//    dataControl->operateMode.mode = DataControl::Operate::Start;

//    usleep(10000);
//    rt_printf("Feeding start\n");

//    dataControl->obi_section_indx = -1;
}

void ControlMain::unsetObiMode()
{
    dataControl->obi_mode = false;
    dataControl->config_check = false;

    if(init_thread_run){
        init_thread_run = false;
        pthread_cancel(init_thread);
        printf("Finished Module Init Thread\n");
        usleep(10000);
    }

    robot_RT_stop();
    usleep(10000);

    if (dataControl->RobotData.module_init){
        for(uint8_t i = 0; i < NUM_JOINT; i++){
            module->dxl_deinit(NUM_JOINT == 1 ? module->single_id : i);
        }
    }
    dataControl->RobotData.module_init = false;

    printf("Finished\n");
}

void ControlMain::putObiMode(char key)
{
    if(key == 38 || key == 'a'){
        if(dataControl->ClientToServer.opMode == DataControl::OpMode::Wait){
            dataControl->ClientToServer.opMode = DataControl::OpMode::OperateMode;
            dataControl->operateMode.mode = DataControl::Operate::ReadyFeeding;

            dataControl->obi_section_indx++;
            if(dataControl->obi_section_indx >= 5){
                dataControl->obi_section_indx = 0;
            }
        }
    }
    else if(key == 39 || key == 's'){
        if(dataControl->ClientToServer.opMode == DataControl::OpMode::Wait){
            dataControl->ClientToServer.opMode = DataControl::OpMode::OperateMode;
            dataControl->operateMode.mode = DataControl::Operate::Feeding;
            switch(dataControl->obi_section_indx){
                case 0: // side 1
                {
                    dataControl->side1_motion.path_data_indx = 500;
                    dataControl->operateMode.section = DataControl::Section::Side1;
                    break;
                }
                case 1: // side 2
                {
                    dataControl->side2_motion.path_data_indx = 500;
                    dataControl->operateMode.section = DataControl::Section::Side2;
                    break;
                }
                case 2: // side 3
                {
                    dataControl->side3_motion.path_data_indx = 500;
                    dataControl->operateMode.section = DataControl::Section::Side3;
                    break;
                }
                case 3: // soup
                {
                    dataControl->soup_motion.path_data_indx = 500;
                    dataControl->operateMode.section = DataControl::Section::Soup;
                    break;
                }
                case 4: // rice
                {
                    dataControl->rice_motion.path_data_indx = 500;
                    dataControl->operateMode.section = DataControl::Section::Rice;
                    break;
                }
            }
        }

//        dataControl->ClientToServer.opMode = DataControl::OpMode::OperateMode;
//        dataControl->operateMode.mode = DataControl::Operate::Feeding;
//        dataControl->operateMode.section = DataControl::Section::Mouse;
    }
    else if(key == 53 || key == 'x'){
        if(dataControl->operateMode.mode == DataControl::Operate::Start){
            dataControl->ClientToServer.opMode = DataControl::OpMode::OperateMode;
            dataControl->operateMode.mode = DataControl::Operate::StartTeaching;
            usleep(10000);
        }
        else if(dataControl->operateMode.mode == DataControl::Operate::StartTeaching){
            dataControl->ClientToServer.opMode = DataControl::OpMode::OperateMode;
            dataControl->operateMode.mode = DataControl::Operate::StopTeaching;
            usleep(10000);
            while(!(dataControl->ClientToServer.opMode == DataControl::OpMode::Wait)){
                usleep(1000);
            }
        }
        else if(dataControl->operateMode.mode == DataControl::Operate::StopTeaching){
            dataControl->ClientToServer.opMode = DataControl::OpMode::OperateMode;
            dataControl->operateMode.mode = DataControl::Operate::StartFeeding;
        }
    }
//    switch(key){
//        case 38:   // switching section
//        {
//            dataControl->ClientToServer.opMode = DataControl::OpMode::OperateMode;
//            dataControl->operateMode.mode = DataControl::Operate::ReadyFeeding;

//            dataControl->obi_section_indx++;
//            if(dataControl->obi_section_indx >= 5){
//                dataControl->obi_section_indx = 0;
//            }
//            break;
//        }
//        case 39:   // feeding
//        {
//            dataControl->ClientToServer.opMode = DataControl::OpMode::OperateMode;
//            dataControl->operateMode.mode = DataControl::Operate::Feeding;
//            switch(dataControl->obi_section_indx){
//                case 0: // side 1
//                {
//                    dataControl->side1_motion.path_data_indx = 500;
//                    dataControl->operateMode.section = DataControl::Section::Side1;
//                    break;
//                }
//                case 1: // side 2
//                {
//                    dataControl->side2_motion.path_data_indx = 500;
//                    dataControl->operateMode.section = DataControl::Section::Side2;
//                    break;
//                }
//                case 2: // side 3
//                {
//                    dataControl->side3_motion.path_data_indx = 500;
//                    dataControl->operateMode.section = DataControl::Section::Side3;
//                    break;
//                }
//                case 3: // soup
//                {
//                    dataControl->soup_motion.path_data_indx = 500;
//                    dataControl->operateMode.section = DataControl::Section::Soup;
//                    break;
//                }
//                case 4: // rice
//                {
//                    dataControl->rice_motion.path_data_indx = 500;
//                    dataControl->operateMode.section = DataControl::Section::Rice;
//                    break;
//                }
//            }

//            dataControl->ClientToServer.opMode = DataControl::OpMode::OperateMode;
//            dataControl->operateMode.mode = DataControl::Operate::Feeding;
//            dataControl->operateMode.section = DataControl::Section::Mouse;
//        }
//        case 53:   // teaching mode on/off
//        {
//            if(dataControl->operateMode.mode == DataControl::Operate::Start){
//                dataControl->ClientToServer.opMode = DataControl::OpMode::OperateMode;
//                dataControl->operateMode.mode = DataControl::Operate::StartTeaching;
//                usleep(10000);
//            }
//            else if(dataControl->operateMode.mode == DataControl::Operate::StartTeaching){
//                dataControl->ClientToServer.opMode = DataControl::OpMode::OperateMode;
//                dataControl->operateMode.mode = DataControl::Operate::StopTeaching;
//                usleep(10000);
//                while(!(dataControl->ClientToServer.opMode == DataControl::OpMode::Wait)){
//                    usleep(1000);
//                }
//            }
//            else if(dataControl->operateMode.mode == DataControl::Operate::StopTeaching){
//                dataControl->ClientToServer.opMode = DataControl::OpMode::OperateMode;
//                dataControl->operateMode.mode = DataControl::Operate::StartFeeding;
//            }
//            break;
//        }
//    }
}

void* ControlMain::init_func(void* arg){
    ControlMain *pThis = static_cast<ControlMain*>(arg);

    while(!pThis->dataControl->config_check){
        usleep(10000);
    }
    pThis->module->init();

    while(pThis->init_thread_run){
        switch(MODULE_TYPE){
            case DataControl::Module::FAR_V1:
                if (!pThis->dataControl->RobotData.module_init){
                    printf("Start FAR V1 Module Initilization\n");
                    pThis->module_indx = 0;
                    pThis->moduleInitFAR();
                }
                break;
            case DataControl::Module::FAR_V2:
                if (!pThis->dataControl->RobotData.module_init){
                    printf("Start FAR V2 Module Initilization\n");
                    pThis->module_indx = 0;
                    pThis->moduleInitFAR();
                }
            case DataControl::Module::FAR_V3:
                if (!pThis->dataControl->RobotData.module_init){
                    printf("Start FAR V3 Module Initilization\n");
                    pThis->module_indx = 0;
                    pThis->moduleInitFAR();
                }
                break;
            default:
                break;
        }
        usleep(1000);
    }
    printf("finished init thread\n");

    return nullptr;
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

void ControlMain::moduleInitSEA()
{

}

void ControlMain::moduleInitFAR()
{
    while(!dataControl->RobotData.module_init)
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
                dataControl->RobotData.module_init = true;

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
                dataControl->RobotData.module_init = true;

//                if (!dataControl->RobotData.offset_setting){
//                    module->getGroupSyncReadPresentPosition(dataControl->RobotData.offset, NUM_JOINT);
//                }
                dataControl->RobotData.offset[3] = dataControl->offset[3] - static_cast<int32_t>(-90*dataControl->DEG2ENC);
//                dataControl->RobotData.offset[5] -= static_cast<int32_t>(-90*dataControl->DEG2ENC);
            }
        }
    }

    tcpServer->sendKey("S");
    robot_RT();
}


void ControlMain::robotInitialize()
{
//    double joint[6] = {0, 0, 0, -90, 0, 90};
//    dataControl->jointPositionDEG2ENC(joint, dataControl->RobotData.command_joint_position);
//    module->setGroupSyncWriteGoalPosition(dataControl->RobotData.command_joint_position, NUM_JOINT);

    module->setGoalPosition(1, 2000);
    usleep(1000);

    dataControl->ClientToServer.opMode = DataControl::OpMode::Wait;
}

void ControlMain::robotServoOn(char enable){

    module->setGroupSyncWriteTorqueEnable(static_cast<uint8_t>(enable), NUM_JOINT);

    dataControl->DataReset();
    dataControl->ClientToServer.opMode = DataControl::OpMode::Wait;
}

void ControlMain::robotKinematics(){
    dataControl->jointPositionENC2RAD(dataControl->RobotData.present_joint_position, dataControl->RobotData.present_q);
    dataControl->jointVelocityENC2RAD(dataControl->RobotData.present_joint_velocity, dataControl->RobotData.present_q_dot);

//    rt_printf("[FK] present q : %f, %f, %f, %f, %f, %f\n",
//              dataControl->RobotData.present_q[0], dataControl->RobotData.present_q[1],
//            dataControl->RobotData.present_q[2], dataControl->RobotData.present_q[3],
//            dataControl->RobotData.present_q[4], dataControl->RobotData.present_q[5]);

//    for(int i = 0; i < 6; i++){
//        dataControl->RobotData.present_q[i] = dataControl->operateRicePoint5[i];
//    }
    robotArm->run_kinematics(dataControl->RobotData.present_q, dataControl->RobotData.present_end_pose);

//    memcpy(dataControl->RobotData.previous_end_pose, dataControl->RobotData.present_end_pose, sizeof(double)*6);

//    rt_printf("[FK] present joint position : %d, %d, %d, %d, %d, %d\n",
//           dataControl->RobotData.present_joint_position[0], dataControl->RobotData.present_joint_position[1],
//            dataControl->RobotData.present_joint_position[2], dataControl->RobotData.present_joint_position[3],
//            dataControl->RobotData.present_joint_position[4], dataControl->RobotData.present_joint_position[5]);
//    rt_printf("[FK] present q : %f, %f, %f, %f, %f, %f\n",
//           dataControl->RobotData.present_q[0], dataControl->RobotData.present_q[1],
//            dataControl->RobotData.present_q[2], dataControl->RobotData.present_q[3],
//            dataControl->RobotData.present_q[4], dataControl->RobotData.present_q[5]);
//    rt_printf("[FK] present pose : %f, %f, %f, %f, %f, %f\n",
//           dataControl->RobotData.present_end_pose_zyx[0], dataControl->RobotData.present_end_pose_zyx[1],
//            dataControl->RobotData.present_end_pose_zyx[2], dataControl->RobotData.present_end_pose_zyx[3],
//            dataControl->RobotData.present_end_pose_zyx[4], dataControl->RobotData.present_end_pose_zyx[5]);

    robotArm->jacobian_zyx();

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
        if(MODULE_TYPE == DataControl::Module::FAR_V1){
            if (i < 3){
                goal_current[i] = 1000*(goal_torque[i] / TORQUE_CONSTANT_W270)*alpha;
            }
            else{
                goal_current[i] = 1000*(goal_torque[i] / TORQUE_CONSTANT_W150)*alpha;
            }
        }
        else if(MODULE_TYPE == DataControl::Module::FAR_V2){
            if (i < 3){
                goal_current[i] = 1000*(goal_torque[i] / TORQUE_CONSTANT_V270)*alpha;
            }
            else{
                goal_current[i] = 1000*(goal_torque[i] / TORQUE_CONSTANT_V350)*alpha;
            }
        }
        else if(MODULE_TYPE == DataControl::Module::FAR_V3){
            if (i < 3){
                goal_current[i] = 1000*(goal_torque[i] / TORQUE_CONSTANT_W270)*alpha;
            }
            else{
                goal_current[i] = 1000*(goal_torque[i] / TORQUE_CONSTANT_W350)*alpha;
            }
        }
    }

//    rt_printf("Desired Torque : %f, %f, %f, %f, %f, %f\n", goal_torque[0], goal_torque[1], goal_torque[2], goal_torque[3], goal_torque[4], goal_torque[5]);
//    rt_printf("Desired Current : %f, %f, %f, %f, %f, %f\n", goal_current[0], goal_current[1], goal_current[2], goal_current[3], goal_current[4], goal_current[5]);

    dataControl->jointCurrentmA2RAW(goal_current, dataControl->RobotData.command_joint_current);
//    rt_printf("Desired Current : %d, %d, %d, %d, %d, %d\n",
//              dataControl->RobotData.command_joint_current[0], dataControl->RobotData.command_joint_current[1],
//            dataControl->RobotData.command_joint_current[2], dataControl->RobotData.command_joint_current[3],
//            dataControl->RobotData.command_joint_current[4], dataControl->RobotData.command_joint_current[5]);
    if (dataControl->RobotData.joint_op_mode == JointOpMode::current_mode){
        module->setGroupSyncWriteGoalCurrent(dataControl->RobotData.command_joint_current, NUM_JOINT);
    }
}

void ControlMain::robotJointMove(char mode, double desJoint[NUM_JOINT])
{
    for(uint i = 0; i < NUM_JOINT; i++){
        module->feeding_profile_acc[i] = 1;//50;
        module->feeding_profile_vel[i] = 3;//static_cast<uint32_t>((500/64.0)*module->feeding_profile_acc[i]);
        module->feeding_pos_p_gain[i] = default_pos_p_gain[i];
    }
    module->setGroupSyncWriteIndirectAddress(module->feeding_profile_acc, module->feeding_profile_vel, module->feeding_pos_p_gain, NUM_JOINT);

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

    if(dataControl->ClientToServer.move_time > 0 && dataControl->ClientToServer.acc_time > 0
            && dataControl->ClientToServer.move_time >= dataControl->ClientToServer.acc_time){

        dataControl->PathData.readyPath.path_x.clear();
        dataControl->PathData.readyPath.path_y.clear();
        dataControl->PathData.readyPath.path_z.clear();
        dataControl->PathData.readyPath.path_theta.clear();

        path_generator(dataControl->RobotData.present_end_pose[0], dataControl->RobotData.desired_end_pose[0],
                dataControl->ClientToServer.move_time, dataControl->ClientToServer.acc_time, step_size, &dataControl->PathData.readyPath.path_x);
        path_generator(dataControl->RobotData.present_end_pose[1], dataControl->RobotData.desired_end_pose[1],
                dataControl->ClientToServer.move_time, dataControl->ClientToServer.acc_time, step_size, &dataControl->PathData.readyPath.path_y);
        path_generator(dataControl->RobotData.present_end_pose[2], dataControl->RobotData.desired_end_pose[2],
                dataControl->ClientToServer.move_time, dataControl->ClientToServer.acc_time, step_size, &dataControl->PathData.readyPath.path_z);

        double R_init[9], R_final[9], r[3], theta;
        RobotArm::zyx2mat(dataControl->RobotData.present_end_pose[5], dataControl->RobotData.present_end_pose[4], dataControl->RobotData.present_end_pose[3], R_init);
        RobotArm::zyx2mat(dataControl->RobotData.desired_end_pose[5], dataControl->RobotData.desired_end_pose[4], dataControl->RobotData.desired_end_pose[3], R_final);
        RobotArm::mat_to_axis_angle(R_init, R_final, r, &theta);
        memcpy(dataControl->PathData.readyPath.r, r, sizeof(double)*3);

        path_generator(0, theta, dataControl->ClientToServer.move_time, dataControl->ClientToServer.acc_time, step_size, &dataControl->PathData.readyPath.path_theta);
        memcpy(dataControl->PathData.readyPath.R_init, robotArm->body[robotArm->num_body].Ae, sizeof(double)*9);

        dataControl->RobotData.run_mode = DataControl::CmdType::ReadyCmd;
        dataControl->PathData.path_data_indx = 0;

        dataControl->PathData.readyPath.data_size = dataControl->PathData.readyPath.path_x.size();
        rt_printf("Cartesian move path size : %d\n", dataControl->PathData.readyPath.data_size);
        dataControl->ClientToServer.opMode = DataControl::OpMode::RunMode;
    }
}

void ControlMain::robotPathGenerate()
{
    for(uint8_t i = 0; i < dataControl->PathData.row - 1; i++){
        path_generator(dataControl->PathData.point_px[i], dataControl->PathData.point_px[i + 1],
                dataControl->PathData.total_time[i + 1] - dataControl->PathData.total_time[i], dataControl->PathData.acc_time[i], step_size, &dataControl->PathData.movePath[i].path_x);

        path_generator(dataControl->PathData.point_py[i], dataControl->PathData.point_py[i + 1],
                dataControl->PathData.total_time[i + 1] - dataControl->PathData.total_time[i], dataControl->PathData.acc_time[i], step_size, &dataControl->PathData.movePath[i].path_y);

        path_generator(dataControl->PathData.point_pz[i], dataControl->PathData.point_pz[i + 1],
                dataControl->PathData.total_time[i + 1] - dataControl->PathData.total_time[i], dataControl->PathData.acc_time[i], step_size, &dataControl->PathData.movePath[i].path_z);
    }

    dataControl->PathData.point_theta.push_back(0);
    for(uint8_t i = 0; i < dataControl->PathData.row - 1; i++){
        double R_init[9], R_final[9], r[3], theta;
        RobotArm::zyx2mat(dataControl->PathData.point_rz[i], dataControl->PathData.point_ry[i], dataControl->PathData.point_rx[i], R_init);
        RobotArm::zyx2mat(dataControl->PathData.point_rz[i + 1], dataControl->PathData.point_ry[i + 1], dataControl->PathData.point_rx[i + 1], R_final);
        RobotArm::mat_to_axis_angle(R_init, R_final, r, &theta);
        memcpy(dataControl->PathData.movePath[i].r, r, sizeof(double)*3);
        dataControl->PathData.point_theta.push_back(theta);

//        rt_printf("r : %f, %f, %f, %f\n", r[0], r[1], r[2], theta);
    }

    for(uint8_t i = 0; i < dataControl->PathData.row - 1; i++){
        path_generator(0, dataControl->PathData.point_theta[i + 1],
                dataControl->PathData.total_time[i + 1] - dataControl->PathData.total_time[i], dataControl->PathData.acc_time[i], step_size, &dataControl->PathData.movePath[i].path_theta);

        dataControl->PathData.movePath[i].data_size = dataControl->PathData.movePath[i].path_x.size();
        rt_printf("section %d path size : %d\n", i, dataControl->PathData.movePath[i].data_size);
    }

//    int indx = 1;
////    for(int i = 0; i < dataControl->PathData.row - 1; i++){
//    int i = 0;
//        for(uint j = 0; j < dataControl->PathData.movePath[i].path_x.size(); j++){
//            printf("%d, %f, %f, %f\n",
//                      indx++, dataControl->PathData.movePath[i].path_x[j], dataControl->PathData.movePath[i].path_y[j], dataControl->PathData.movePath[i].path_z[j]);
//        }
////    }

    dataControl->ClientToServer.opMode = DataControl::OpMode::Wait;
}

void ControlMain::robotReady()
{
    path_generator(dataControl->RobotData.present_end_pose[0], dataControl->PathData.point_px[0],
            dataControl->PathData.total_time[1] - dataControl->PathData.total_time[0], dataControl->PathData.acc_time[0], step_size, &dataControl->PathData.readyPath.path_x);
    path_generator(dataControl->RobotData.present_end_pose[1], dataControl->PathData.point_py[0],
            dataControl->PathData.total_time[1] - dataControl->PathData.total_time[0], dataControl->PathData.acc_time[0], step_size, &dataControl->PathData.readyPath.path_y);
    path_generator(dataControl->RobotData.present_end_pose[2], dataControl->PathData.point_pz[0],
            dataControl->PathData.total_time[1] - dataControl->PathData.total_time[0], dataControl->PathData.acc_time[0], step_size, &dataControl->PathData.readyPath.path_z);

    double R_init[9], R_final[9], r[3], theta;
    RobotArm::zyx2mat(dataControl->RobotData.present_end_pose[5], dataControl->RobotData.present_end_pose[4], dataControl->RobotData.present_end_pose[3], R_init);
    RobotArm::zyx2mat(dataControl->PathData.point_rz[0], dataControl->PathData.point_ry[0], dataControl->PathData.point_rx[0], R_final);
    RobotArm::mat_to_axis_angle(R_init, R_final, r, &theta);
    memcpy(dataControl->PathData.readyPath.r, r, sizeof(double)*3);

    path_generator(0, theta, dataControl->PathData.total_time[1] - dataControl->PathData.total_time[0], dataControl->PathData.acc_time[0], step_size, &dataControl->PathData.readyPath.path_theta);
    memcpy(dataControl->PathData.readyPath.R_init, robotArm->body[robotArm->num_body].Ae, sizeof(double)*9);

    dataControl->RobotData.run_mode = DataControl::CmdType::ReadyCmd;

    dataControl->PathData.readyPath.data_size = dataControl->PathData.readyPath.path_x.size();
    rt_printf("ready path size : %d\n", dataControl->PathData.readyPath.data_size);
    dataControl->ClientToServer.opMode = DataControl::OpMode::RunMode;

    dataControl->PathData.path_data_indx = 0;
    delay_cnt = 0;
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
            memcpy(dataControl->RobotData.desired_end_pose_zyx, dataControl->RobotData.desired_end_pose, sizeof(double)*3);

            double Ri[9], Rd[9];
            RobotArm::axis_angle_to_mat(dataControl->PathData.readyPath.r, dataControl->PathData.readyPath.path_theta[dataControl->PathData.path_data_indx], Ri);
            RobotArm::mat(dataControl->PathData.readyPath.R_init, Ri, 3, 3, 3, 3, Rd);
            RobotArm::mat2rpy(Rd, dataControl->RobotData.desired_end_pose + 3);
            RobotArm::mat2zyx(Rd, dataControl->RobotData.desired_end_pose_zyx + 3);

            dataControl->jointPositionENC2RAD(dataControl->RobotData.present_joint_position, dataControl->RobotData.present_q);
            dataControl->jointVelocityENC2RAD(dataControl->RobotData.present_joint_velocity, dataControl->RobotData.present_q_dot);

            if (dataControl->RobotData.joint_op_mode == JointOpMode::current_mode){

                robotVSD();

                for(uint i = 0; i < NUM_JOINT; i++){
                    if (i < 3){
                        goal_current[i] = 1000*(dataControl->RobotData.T[i] / TORQUE_CONSTANT_W270);
                    }
                    else{
                        goal_current[i] = 1000*(dataControl->RobotData.T[i] / TORQUE_CONSTANT_W350);
                    }
                }

                dataControl->jointCurrentmA2RAW(goal_current, dataControl->RobotData.command_joint_current);

                module->setGroupSyncWriteGoalCurrent(dataControl->RobotData.command_joint_current, NUM_JOINT);
            }
            else if (dataControl->RobotData.joint_op_mode == JointOpMode::extended_position_mode){
                dataControl->RobotData.ik_time1 = static_cast<unsigned long>(rt_timer_read());
                robotArm->run_inverse_kinematics(dataControl->RobotData.present_q, dataControl->RobotData.desired_end_pose,
                                                 dataControl->RobotData.desired_q, dataControl->RobotData.present_end_pose);
                dataControl->RobotData.ik_time2 = static_cast<unsigned long>(rt_timer_read());

//                rt_printf("Present q : %f, %f, %f, %f, %f, %f\n",
//                          dataControl->RobotData.present_q[0], dataControl->RobotData.present_q[1], dataControl->RobotData.present_q[2],
//                        dataControl->RobotData.present_q[3], dataControl->RobotData.present_q[4], dataControl->RobotData.present_q[5]);
//                rt_printf("Desired q : %f, %f, %f, %f, %f, %f\n",
//                          dataControl->RobotData.desired_q[0], dataControl->RobotData.desired_q[1], dataControl->RobotData.desired_q[2],
//                        dataControl->RobotData.desired_q[3], dataControl->RobotData.desired_q[4], dataControl->RobotData.desired_q[5]);

                dataControl->cartesianPoseScaleUp(dataControl->RobotData.present_end_pose, dataControl->RobotData.present_cal_end_pose);

                dataControl->jointPositionRAD2ENC(dataControl->RobotData.desired_q, dataControl->RobotData.command_joint_position);

//                rt_printf("Present Position : %d, %d, %d, %d, %d, %d\n",
//                          dataControl->RobotData.present_joint_position[0], dataControl->RobotData.present_joint_position[1],
//                        dataControl->RobotData.present_joint_position[2], dataControl->RobotData.present_joint_position[3],
//                        dataControl->RobotData.present_joint_position[4], dataControl->RobotData.present_joint_position[5]);

//                rt_printf("Command Position : %d, %d, %d, %d, %d, %d\n",
//                          dataControl->RobotData.command_joint_position[0], dataControl->RobotData.command_joint_position[1],
//                        dataControl->RobotData.command_joint_position[2], dataControl->RobotData.command_joint_position[3],
//                        dataControl->RobotData.command_joint_position[4], dataControl->RobotData.command_joint_position[5]);

//                rt_printf("Desired Pose : %f, %f, %f, %f, %f, %f\n",
//                          dataControl->RobotData.desired_end_pose[0], dataControl->RobotData.desired_end_pose[1],
//                        dataControl->RobotData.desired_end_pose[2], dataControl->RobotData.desired_end_pose[3],
//                        dataControl->RobotData.desired_end_pose[4], dataControl->RobotData.desired_end_pose[5]);

//                rt_printf("Present Pose : %f, %f, %f, %f, %f, %f\n",
//                          dataControl->RobotData.present_end_pose[0], dataControl->RobotData.present_end_pose[1],
//                        dataControl->RobotData.present_end_pose[2], dataControl->RobotData.present_end_pose[3],
//                        dataControl->RobotData.present_end_pose[4], dataControl->RobotData.present_end_pose[5]);

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
            memcpy(dataControl->RobotData.desired_end_pose_zyx, dataControl->RobotData.desired_end_pose, sizeof(double)*3);

            double Ri[9], Rd[9];
            RobotArm::axis_angle_to_mat(dataControl->PathData.movePath[dataControl->PathData.path_struct_indx].r,
                    dataControl->PathData.movePath[dataControl->PathData.path_struct_indx].path_theta[dataControl->PathData.path_data_indx], Ri);
            RobotArm::mat(dataControl->PathData.movePath[dataControl->PathData.path_struct_indx].R_init, Ri, 3, 3, 3, 3, Rd);
            RobotArm::mat2rpy(Rd, dataControl->RobotData.desired_end_pose + 3);
            RobotArm::mat2zyx(Rd, dataControl->RobotData.desired_end_pose_zyx + 3);

            dataControl->jointPositionENC2RAD(dataControl->RobotData.present_joint_position, dataControl->RobotData.present_q);

            if (dataControl->RobotData.joint_op_mode == JointOpMode::current_mode){

                robotVSD();

                for(uint i = 0; i < NUM_JOINT; i++){
                    if (i < 3){
                        goal_current[i] = 1000*(dataControl->RobotData.T[i] / TORQUE_CONSTANT_W270);
                    }
                    else{
                        goal_current[i] = 1000*(dataControl->RobotData.T[i] / TORQUE_CONSTANT_W350);
                    }
                }

                dataControl->jointCurrentmA2RAW(goal_current, dataControl->RobotData.command_joint_current);

                module->setGroupSyncWriteGoalCurrent(dataControl->RobotData.command_joint_current, NUM_JOINT);
            }
            else if(dataControl->RobotData.joint_op_mode == JointOpMode::extended_position_mode){

                dataControl->RobotData.ik_time1 = static_cast<unsigned long>(rt_timer_read());

                robotArm->run_inverse_kinematics(dataControl->RobotData.present_q, dataControl->RobotData.desired_end_pose,
                                                           dataControl->RobotData.desired_q, dataControl->RobotData.present_end_pose);

                dataControl->RobotData.ik_time2 = static_cast<unsigned long>(rt_timer_read());

                dataControl->cartesianPoseScaleUp(dataControl->RobotData.present_end_pose, dataControl->RobotData.present_cal_end_pose);

                if (delay_cnt == 0){
//                    if(err == 0){
                        dataControl->jointPositionRAD2ENC(dataControl->RobotData.desired_q, dataControl->RobotData.command_joint_position);
                        module->setGroupSyncWriteGoalPosition(dataControl->RobotData.command_joint_position, NUM_JOINT);
//                    }
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
                    if (fork_cnt_max > 0){
                        fork_cnt++;
                        if (fork_cnt <= 1000){
                            module->setProfileVelocity(5, init_profile_vel[5] - 3);
                            module->setProfileAcceleration(5, init_profile_acc[5] - 3);
                            module->setGoalPosition(5, static_cast<int32_t>((-210)*dataControl->DEG2ENC) + dataControl->RobotData.offset[5]);
                        }
                        else if (fork_cnt >= 2000 && fork_cnt <= 3000){
                            module->setGoalPosition(5, dataControl->RobotData.command_joint_position[5]);
                        }
                    }
//                    rt_printf("fork_cnt : %d\t, fork_cnt_max : %d\n", fork_cnt, fork_cnt_max);
                    if (fork_cnt >= fork_cnt_max){
                        module->setProfileVelocity(5, init_profile_vel[5]);
                        module->setProfileAcceleration(5, init_profile_acc[5]);

                        dataControl->PathData.path_data_indx = 0;
                        dataControl->PathData.path_struct_indx++;
                        memcpy(dataControl->PathData.movePath[dataControl->PathData.path_struct_indx].R_init, Rd, sizeof(double)*9);
                        delay_cnt = 0;
                        dataControl->RobotData.ik_flag = true;
                    }
                }
                else{
                    delay_cnt++;
//                    rt_printf("delay_cnt : %d\n", delay_cnt);
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
                        dataControl->RobotData.ik_flag = true;

                        if(dataControl->feeding && dataControl->operateMode.section == DataControl::Section::Mouse){
                            dataControl->PathData.total_time.clear();
                            dataControl->PathData.point_px.clear();
                            dataControl->PathData.point_py.clear();
                            dataControl->PathData.point_pz.clear();
                            dataControl->PathData.point_rx.clear();
                            dataControl->PathData.point_ry.clear();
                            dataControl->PathData.point_rz.clear();
                            dataControl->PathData.point_theta.clear();
                            dataControl->PathData.acc_time.clear();

                            dataControl->PathData.row = 2;
                            for(uint i = 0; i < dataControl->PathData.row; i++){
                                dataControl->PathData.movePath[i].path_x.clear();
                                dataControl->PathData.movePath[i].path_y.clear();
                                dataControl->PathData.movePath[i].path_z.clear();
                                dataControl->PathData.movePath[i].path_theta.clear();
                            }

                            dataControl->PathData.total_time.push_back(0);
                            dataControl->PathData.point_px.push_back(dataControl->RobotData.desired_end_pose[0]);
                            dataControl->PathData.point_py.push_back(dataControl->RobotData.desired_end_pose[1]);
                            dataControl->PathData.point_pz.push_back(dataControl->RobotData.desired_end_pose[2]);
                            dataControl->PathData.point_rx.push_back(dataControl->RobotData.desired_end_pose[3]);
                            dataControl->PathData.point_ry.push_back(dataControl->RobotData.desired_end_pose[4]);
                            dataControl->PathData.point_rz.push_back(dataControl->RobotData.desired_end_pose[5]);
                            dataControl->PathData.acc_time.push_back(0.5);

                            dataControl->PathData.total_time.push_back(3);
                            dataControl->PathData.point_px.push_back(dataControl->operateReadyPose[0]);
                            dataControl->PathData.point_py.push_back(dataControl->operateReadyPose[1]);
                            dataControl->PathData.point_pz.push_back(dataControl->operateReadyPose[2]);
                            dataControl->PathData.point_rx.push_back(dataControl->operateReadyPose[3]);
                            dataControl->PathData.point_ry.push_back(dataControl->operateReadyPose[4]);
                            dataControl->PathData.point_rz.push_back(dataControl->operateReadyPose[5]);
                            dataControl->PathData.acc_time.push_back(0.5);

                            for(int8_t i = 0; i < dataControl->PathData.row; i ++){
                                rt_printf("%f, %f, %f, %f, %f, %f, %f, %f\n",
                                          dataControl->PathData.total_time[i],
                                          dataControl->PathData.point_px[i], dataControl->PathData.point_py[i], dataControl->PathData.point_pz[i],
                                          dataControl->PathData.point_rx[i], dataControl->PathData.point_ry[i], dataControl->PathData.point_rz[i],
                                          dataControl->PathData.acc_time[i]);
                            }

                            robotPathGenerate();

                            dataControl->operateMode.section = DataControl::Section::Home;
                            dataControl->operateMode.mode = DataControl::Operate::Feeding;
                            dataControl->ClientToServer.opMode = DataControl::OpMode::OperateMode;
                        }
                        else{
                            dataControl->feeding = false;
                            rt_printf("%d\n", dataControl->feeding);
                            dataControl->PathData.path_data_indx = 0;
                            dataControl->PathData.path_struct_indx = 0;

                            if(dataControl->operateMode.section == DataControl::Section::Home){
                                dataControl->ClientToServer.opMode = DataControl::OpMode::OperateMode;
                                dataControl->operateMode.mode = DataControl::Operate::ReadyFeeding;
                            }
                            else{
                                dataControl->ClientToServer.opMode = DataControl::OpMode::Wait;
                            }
                        }
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
                    1.0, 0.3, step_size, &dataControl->PathData.readyPath.path_x);
            path_generator(dataControl->RobotData.present_end_pose[1], dataControl->PathData.file_data[2],
                    1.0, 0.3, step_size, &dataControl->PathData.readyPath.path_y);
            path_generator(dataControl->RobotData.present_end_pose[2], dataControl->PathData.file_data[3],
                    1.0, 0.3, step_size, &dataControl->PathData.readyPath.path_z);

            double R_init[9], R_final[9], r[3], theta;
            RobotArm::zyx2mat(dataControl->RobotData.present_end_pose[5], dataControl->RobotData.present_end_pose[4], dataControl->RobotData.present_end_pose[3], R_init);
            RobotArm::zyx2mat(dataControl->PathData.file_data[6], dataControl->PathData.file_data[5], dataControl->PathData.file_data[4], R_final);
            RobotArm::mat_to_axis_angle(R_init, R_final, r, &theta);
            memcpy(dataControl->PathData.readyPath.r, r, sizeof(double)*3);

            path_generator(0, theta, 1.0, 0.3, step_size, &dataControl->PathData.readyPath.path_theta);
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
            memcpy(dataControl->RobotData.desired_end_pose_zyx, dataControl->RobotData.desired_end_pose, sizeof(double)*3);

            double Ri[9], Rd[9];
            RobotArm::axis_angle_to_mat(dataControl->PathData.movePath[dataControl->PathData.path_struct_indx].r,
                    dataControl->PathData.movePath[dataControl->PathData.path_struct_indx].path_theta[dataControl->PathData.path_data_indx], Ri);
            RobotArm::mat(dataControl->PathData.movePath[dataControl->PathData.path_struct_indx].R_init, Ri, 3, 3, 3, 3, Rd);
            RobotArm::mat2rpy(Rd, dataControl->RobotData.desired_end_pose + 3);
            RobotArm::mat2zyx(Rd, dataControl->RobotData.desired_end_pose_zyx + 3);

            dataControl->jointPositionENC2RAD(dataControl->RobotData.present_joint_position, dataControl->RobotData.present_q);

            if (dataControl->RobotData.joint_op_mode == JointOpMode::current_mode){

                robotVSD();

                for(uint i = 0; i < NUM_JOINT; i++){
                    if (i < 3){
                        goal_current[i] = 1000*(dataControl->RobotData.T[i] / TORQUE_CONSTANT_W270);
                    }
                    else{
                        goal_current[i] = 1000*(dataControl->RobotData.T[i] / TORQUE_CONSTANT_W350);
                    }
                }

                dataControl->jointCurrentmA2RAW(goal_current, dataControl->RobotData.command_joint_current);

                module->setGroupSyncWriteGoalCurrent(dataControl->RobotData.command_joint_current, NUM_JOINT);
            }
            else if(dataControl->RobotData.joint_op_mode == JointOpMode::extended_position_mode){

                dataControl->RobotData.ik_time1 = static_cast<unsigned long>(rt_timer_read());

                robotArm->run_inverse_kinematics(dataControl->RobotData.present_q, dataControl->RobotData.desired_end_pose,
                                                           dataControl->RobotData.desired_q, dataControl->RobotData.present_end_pose);

                dataControl->RobotData.ik_time2 = static_cast<unsigned long>(rt_timer_read());

                dataControl->cartesianPoseScaleUp(dataControl->RobotData.present_end_pose, dataControl->RobotData.present_cal_end_pose);

                if (delay_cnt == 0){
//                    if(err == 0){
                        dataControl->jointPositionRAD2ENC(dataControl->RobotData.desired_q, dataControl->RobotData.command_joint_position);
                        module->setGroupSyncWriteGoalPosition(dataControl->RobotData.command_joint_position, NUM_JOINT);
//                    }
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
                    if (fork_cnt_max > 0){
                        fork_cnt++;
                        if (fork_cnt <= 1000){
                            module->setProfileVelocity(5, init_profile_vel[5] - 3);
                            module->setProfileAcceleration(5, init_profile_acc[5] - 3);
                            module->setGoalPosition(5, static_cast<int32_t>((-210)*dataControl->DEG2ENC) + dataControl->RobotData.offset[5]);
                        }
                        else if (fork_cnt >= 2000 && fork_cnt <= 3000){
                            module->setGoalPosition(5, dataControl->RobotData.command_joint_position[5]);
                        }
                    }
//                    rt_printf("fork_cnt : %d\t, fork_cnt_max : %d\n", fork_cnt, fork_cnt_max);
                    if (fork_cnt >= fork_cnt_max){
                        module->setProfileVelocity(5, init_profile_vel[5]);
                        module->setProfileAcceleration(5, init_profile_acc[5]);

                        dataControl->PathData.path_data_indx = 0;
                        dataControl->PathData.path_struct_indx++;
                        memcpy(dataControl->PathData.movePath[dataControl->PathData.path_struct_indx].R_init, Rd, sizeof(double)*9);
                        delay_cnt = 0;
                        dataControl->RobotData.ik_flag = true;
                    }
                }
                else{
                    delay_cnt++;
//                    rt_printf("delay_cnt : %d\n", delay_cnt);
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
                        dataControl->RobotData.ik_flag = true;

                        if(dataControl->feeding && dataControl->operateMode.section == DataControl::Section::Mouse){
                            dataControl->PathData.total_time.clear();
                            dataControl->PathData.point_px.clear();
                            dataControl->PathData.point_py.clear();
                            dataControl->PathData.point_pz.clear();
                            dataControl->PathData.point_rx.clear();
                            dataControl->PathData.point_ry.clear();
                            dataControl->PathData.point_rz.clear();
                            dataControl->PathData.point_theta.clear();
                            dataControl->PathData.acc_time.clear();

                            dataControl->PathData.row = 2;
                            for(uint i = 0; i < dataControl->PathData.row; i++){
                                dataControl->PathData.movePath[i].path_x.clear();
                                dataControl->PathData.movePath[i].path_y.clear();
                                dataControl->PathData.movePath[i].path_z.clear();
                                dataControl->PathData.movePath[i].path_theta.clear();
                            }

                            dataControl->PathData.total_time.push_back(0);
                            dataControl->PathData.point_px.push_back(dataControl->RobotData.desired_end_pose[0]);
                            dataControl->PathData.point_py.push_back(dataControl->RobotData.desired_end_pose[1]);
                            dataControl->PathData.point_pz.push_back(dataControl->RobotData.desired_end_pose[2]);
                            dataControl->PathData.point_rx.push_back(dataControl->RobotData.desired_end_pose[3]);
                            dataControl->PathData.point_ry.push_back(dataControl->RobotData.desired_end_pose[4]);
                            dataControl->PathData.point_rz.push_back(dataControl->RobotData.desired_end_pose[5]);
                            dataControl->PathData.acc_time.push_back(0.5);

                            dataControl->PathData.total_time.push_back(3);
                            dataControl->PathData.point_px.push_back(dataControl->operateReadyPose[0]);
                            dataControl->PathData.point_py.push_back(dataControl->operateReadyPose[1]);
                            dataControl->PathData.point_pz.push_back(dataControl->operateReadyPose[2]);
                            dataControl->PathData.point_rx.push_back(dataControl->operateReadyPose[3]);
                            dataControl->PathData.point_ry.push_back(dataControl->operateReadyPose[4]);
                            dataControl->PathData.point_rz.push_back(dataControl->operateReadyPose[5]);
                            dataControl->PathData.acc_time.push_back(0.5);

                            for(int8_t i = 0; i < dataControl->PathData.row; i ++){
                                rt_printf("%f, %f, %f, %f, %f, %f, %f, %f\n",
                                          dataControl->PathData.total_time[i],
                                          dataControl->PathData.point_px[i], dataControl->PathData.point_py[i], dataControl->PathData.point_pz[i],
                                          dataControl->PathData.point_rx[i], dataControl->PathData.point_ry[i], dataControl->PathData.point_rz[i],
                                          dataControl->PathData.acc_time[i]);
                            }

                            robotPathGenerate();

                            dataControl->operateMode.section = DataControl::Section::Home;
                            dataControl->operateMode.mode = DataControl::Operate::Feeding;
                            dataControl->ClientToServer.opMode = DataControl::OpMode::OperateMode;
                        }
                        else{
                            dataControl->feeding = false;
                            rt_printf("%d\n", dataControl->feeding);
                            dataControl->PathData.path_data_indx = 0;
                            dataControl->PathData.path_struct_indx = 0;
                            dataControl->ClientToServer.opMode = DataControl::OpMode::Wait;
                        }
                    }
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
            for(uint i = 0; i < NUM_JOINT; i++){
                module->feeding_profile_acc[i] = 100;//50;
                module->feeding_profile_vel[i] = 1000;//static_cast<uint32_t>((500/64.0)*module->feeding_profile_acc[i]);
                module->feeding_pos_p_gain[i] = default_pos_p_gain[i];
            }
            module->setGroupSyncWriteIndirectAddress(module->feeding_profile_acc, module->feeding_profile_vel, module->feeding_pos_p_gain, NUM_JOINT);

            memcpy(dataControl->RobotData.desired_q, dataControl->operateReadyJoint, sizeof(double)*NUM_JOINT);
            dataControl->jointPositionRAD2ENC(dataControl->RobotData.desired_q, dataControl->RobotData.command_joint_position);

            module->setGroupSyncWriteGoalPosition(dataControl->RobotData.command_joint_position, NUM_JOINT);
            usleep(module->feeding_profile_vel[0]*1000);
//            int i = 0;
//            while(1){
//                for(i = 0; i < NUM_JOINT; i++){
//                    if((dataControl->RobotData.moving_status[i] & 0x01) != 0x01){
//                        break;
//                    }
//                }
//                rt_printf("%d\n", i);
//                if(i == NUM_JOINT){
//                    break;
//                }
//                i = 0;
////                rt_printf("moving : %d, %d, %d, %d, %d, %d\n", dataControl->RobotData.moving[0], dataControl->RobotData.moving[1], dataControl->RobotData.moving[2],
////                        dataControl->RobotData.moving[3], dataControl->RobotData.moving[4], dataControl->RobotData.moving[5]);
////                rt_printf("moving status : %d, %d, %d, %d, %d, %d\n", dataControl->RobotData.moving_status[0], dataControl->RobotData.moving_status[1],
////                        dataControl->RobotData.moving_status[2], dataControl->RobotData.moving_status[3],
////                        dataControl->RobotData.moving_status[4], dataControl->RobotData.moving_status[5]);
//            }

            break;
        }
        case DataControl::Operate::Stop:
        {
            break;
        }
        case DataControl::Operate::StartTeaching:
        {
            dataControl->PathData.total_time.clear();
            dataControl->PathData.point_px.clear();
            dataControl->PathData.point_py.clear();
            dataControl->PathData.point_pz.clear();
            dataControl->PathData.point_rx.clear();
            dataControl->PathData.point_ry.clear();
            dataControl->PathData.point_rz.clear();
            dataControl->PathData.point_theta.clear();
            dataControl->PathData.acc_time.clear();

            dataControl->PathData.row = 2;
            for(uint i = 0; i < dataControl->PathData.row; i++){
                dataControl->PathData.movePath[i].path_x.clear();
                dataControl->PathData.movePath[i].path_y.clear();
                dataControl->PathData.movePath[i].path_z.clear();
                dataControl->PathData.movePath[i].path_theta.clear();
            }

            dataControl->PathData.total_time.push_back(0);
            dataControl->PathData.point_px.push_back(dataControl->operateFeedingReadyPose[0]);
            dataControl->PathData.point_py.push_back(dataControl->operateFeedingReadyPose[1]);
            dataControl->PathData.point_pz.push_back(dataControl->operateFeedingReadyPose[2]);
            dataControl->PathData.point_rx.push_back(dataControl->operateFeedingReadyPose[3]);
            dataControl->PathData.point_ry.push_back(dataControl->operateFeedingReadyPose[4]);
            dataControl->PathData.point_rz.push_back(dataControl->operateFeedingReadyPose[5]);
            dataControl->PathData.acc_time.push_back(0.5);

            module->setGroupSyncWriteTorqueEnable(0, NUM_JOINT);
            module->setGroupSyncWriteOperatingMode(JointOpMode::current_mode, NUM_JOINT);
            module->setGroupSyncWriteTorqueEnable(1, NUM_JOINT);
            dataControl->RobotData.joint_op_mode = JointOpMode::current_mode;
            break;
        }
        case DataControl::Operate::StopTeaching:
        {
            memcpy(dataControl->PathData.teaching_pose, dataControl->RobotData.present_end_pose, sizeof(double)*NUM_DOF);

            dataControl->PathData.total_time.push_back(3);
            dataControl->PathData.point_px.push_back(dataControl->PathData.teaching_pose[0]);
            dataControl->PathData.point_py.push_back(dataControl->PathData.teaching_pose[1]);
            dataControl->PathData.point_pz.push_back(dataControl->PathData.teaching_pose[2]);
            dataControl->PathData.point_rx.push_back(dataControl->PathData.teaching_pose[3]);
            dataControl->PathData.point_ry.push_back(dataControl->PathData.teaching_pose[4]);
            dataControl->PathData.point_rz.push_back(dataControl->PathData.teaching_pose[5]);
            dataControl->PathData.acc_time.push_back(0.5);

            module->setGroupSyncWriteTorqueEnable(0, NUM_JOINT);
            module->setGroupSyncWriteOperatingMode(JointOpMode::extended_position_mode, NUM_JOINT);
//            module->setGroupSyncWriteIndirectAddress(init_profile_acc, init_profile_vel, init_pos_p_gain, NUM_JOINT);
            module->setGroupSyncWriteIndirectAddress(module->feeding_profile_acc, module->feeding_profile_vel, module->feeding_pos_p_gain, NUM_JOINT);
            module->setGroupSyncWriteTorqueEnable(1, NUM_JOINT);
            dataControl->RobotData.joint_op_mode = JointOpMode::extended_position_mode;

            for(int8_t i = 0; i < dataControl->PathData.row; i ++){
                rt_printf("%f, %f, %f, %f, %f, %f, %f, %f\n",
                          dataControl->PathData.total_time[i],
                          dataControl->PathData.point_px[i], dataControl->PathData.point_py[i], dataControl->PathData.point_pz[i],
                          dataControl->PathData.point_rx[i], dataControl->PathData.point_ry[i], dataControl->PathData.point_rz[i],
                          dataControl->PathData.acc_time[i]);
            }

            robotPathGenerate();
            break;
        }
        case DataControl::Operate::StartFeeding:
        {
            memcpy(dataControl->RobotData.desired_q, dataControl->operateReadyJoint, sizeof(double)*NUM_JOINT);
            dataControl->jointPositionRAD2ENC(dataControl->RobotData.desired_q, dataControl->RobotData.command_joint_position);

            module->setGroupSyncWriteGoalPosition(dataControl->RobotData.command_joint_position, NUM_JOINT);
            usleep(module->feeding_profile_vel[0]*1000);

            dataControl->ClientToServer.opMode = DataControl::OpMode::Wait;

//            for(uint i = 0; i < NUM_JOINT; i++){
//                module->feeding_profile_acc[i] = 0;
//                module->feeding_profile_vel[i] = 0;
//                module->feeding_pos_p_gain[i] = default_pos_p_gain[i];
//            }
//            module->setGroupSyncWriteIndirectAddress(module->feeding_profile_acc, module->feeding_profile_vel, module->feeding_pos_p_gain, NUM_JOINT);

//            module->setGroupSyncWriteIndirectAddress(default_profile_acc, default_profile_vel, default_vel_limit, default_pos_p_gain, NUM_JOINT);
            break;
        }
        case DataControl::Operate::StopFeeding:
        {
            break;
        }
        case DataControl::Operate::Feeding:
        {
            for(uint i = 0; i < NUM_JOINT; i++){
                module->feeding_profile_acc[i] = 1;//50;
                module->feeding_profile_vel[i] = 3;//static_cast<uint32_t>((500/64.0)*module->feeding_profile_acc[i]);
                module->feeding_pos_p_gain[i] = default_pos_p_gain[i];
            }
            module->setGroupSyncWriteIndirectAddress(module->feeding_profile_acc, module->feeding_profile_vel, module->feeding_pos_p_gain, NUM_JOINT);

            int interval = 1;
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
//                        dataControl->ClientToServer.opMode = DataControl::OpMode::OperateMode;
//                        dataControl->operateMode.mode = DataControl::Operate::ReadyFeeding;
//                        usleep(1000000);
                    }
                    fork_cnt_max = 4000;
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
//                        dataControl->ClientToServer.opMode = DataControl::OpMode::OperateMode;
//                        dataControl->operateMode.mode = DataControl::Operate::ReadyFeeding;
//                        usleep(1000000);
                    }
                    fork_cnt_max = 4000;
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
//                        dataControl->ClientToServer.opMode = DataControl::OpMode::OperateMode;
//                        dataControl->operateMode.mode = DataControl::Operate::ReadyFeeding;
//                        usleep(1000000);
                    }
                    fork_cnt_max = 4000;
                    fork_cnt = 0;

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
                case DataControl::Section::Rice:
                {                    
                    rt_printf("path_data_indx : %d\n", dataControl->rice_motion.path_data_indx);

                    for(uint i = 0; i < NUM_JOINT; i++){
                        dataControl->RobotData.desired_q[i] = dataControl->rice_motion.file_data[dataControl->rice_motion.path_data_indx*32 + i + 2];
                    }
                    dataControl->rice_motion.path_data_indx += interval;
                    if (dataControl->rice_motion.path_data_indx >= 2000){
                        dataControl->rice_motion.path_data_indx = 0;

//                        dataControl->ClientToServer.opMode = DataControl::OpMode::Wait;
                        dataControl->operateMode.section = DataControl::Section::Mouse;
                    }

//                    dataControl->PathData.total_time.clear();
//                    dataControl->PathData.point_px.clear();
//                    dataControl->PathData.point_py.clear();
//                    dataControl->PathData.point_pz.clear();
//                    dataControl->PathData.point_rx.clear();
//                    dataControl->PathData.point_ry.clear();
//                    dataControl->PathData.point_rz.clear();
//                    dataControl->PathData.point_theta.clear();
//                    dataControl->PathData.acc_time.clear();

//                    dataControl->PathData.row = 7;
//                    double time = 0;
//                    for(uint i = 0; i < dataControl->PathData.row; i++){
//                        dataControl->PathData.movePath[i].path_x.clear();
//                        dataControl->PathData.movePath[i].path_y.clear();
//                        dataControl->PathData.movePath[i].path_z.clear();
//                        dataControl->PathData.movePath[i].path_theta.clear();

//                        dataControl->PathData.total_time.push_back(time);
//                        dataControl->PathData.point_px.push_back(dataControl->operateRicePoints[i*NUM_DOF + 0]);
//                        dataControl->PathData.point_py.push_back(dataControl->operateRicePoints[i*NUM_DOF + 1]);
//                        dataControl->PathData.point_pz.push_back(dataControl->operateRicePoints[i*NUM_DOF + 2]);
//                        dataControl->PathData.point_rx.push_back(dataControl->operateRicePoints[i*NUM_DOF + 3]);
//                        dataControl->PathData.point_ry.push_back(dataControl->operateRicePoints[i*NUM_DOF + 4]);
//                        dataControl->PathData.point_rz.push_back(dataControl->operateRicePoints[i*NUM_DOF + 5]);
//                        dataControl->PathData.acc_time.push_back(0.2);
//                        time += 1.0;
//                    }

//                    rt_printf("path data : \n");
//                    for(int8_t i = 0; i < dataControl->PathData.row; i ++){
//                        rt_printf("%f, %f, %f, %f, %f, %f, %f, %f\n",
//                                  dataControl->PathData.total_time[i],
//                                  dataControl->PathData.point_px[i], dataControl->PathData.point_py[i], dataControl->PathData.point_pz[i],
//                                  dataControl->PathData.point_rx[i], dataControl->PathData.point_ry[i], dataControl->PathData.point_rz[i],
//                                  dataControl->PathData.acc_time[i]);
//                    }

//                    robotPathGenerate();

//                    RobotArm::zyx2mat(dataControl->operateRicePoints[5], dataControl->operateRicePoints[4], dataControl->operateRicePoints[3], dataControl->PathData.movePath[0].R_init);

//                    dataControl->ClientToServer.opMode = DataControl::OpMode::RunMode;

//                    dataControl->RobotData.run_mode = DataControl::CmdType::RunCmd;
//                    dataControl->PathData.path_data_indx = 0;
//                    dataControl->PathData.path_struct_indx = 0;
//                    dataControl->PathData.cycle_count_max = 1;
//                    delay_cnt = 0;
//                    delay_cnt_max = 100;

                    break;
                }
                case DataControl::Section::Rice1:{
                    dataControl->PathData.total_time.clear();
                    dataControl->PathData.point_px.clear();
                    dataControl->PathData.point_py.clear();
                    dataControl->PathData.point_pz.clear();
                    dataControl->PathData.point_rx.clear();
                    dataControl->PathData.point_ry.clear();
                    dataControl->PathData.point_rz.clear();
                    dataControl->PathData.point_theta.clear();
                    dataControl->PathData.acc_time.clear();

                    dataControl->PathData.row = 7;
                    double time = 0;
                    for(uint i = 0; i < dataControl->PathData.row; i++){
                        dataControl->PathData.movePath[i].path_x.clear();
                        dataControl->PathData.movePath[i].path_y.clear();
                        dataControl->PathData.movePath[i].path_z.clear();
                        dataControl->PathData.movePath[i].path_theta.clear();

                        dataControl->PathData.total_time.push_back(time);
                        dataControl->PathData.point_px.push_back(dataControl->operateRicePoints1[i*NUM_DOF + 0]);
                        dataControl->PathData.point_py.push_back(dataControl->operateRicePoints1[i*NUM_DOF + 1]);
                        dataControl->PathData.point_pz.push_back(dataControl->operateRicePoints1[i*NUM_DOF + 2]);
                        dataControl->PathData.point_rx.push_back(dataControl->operateRicePoints1[i*NUM_DOF + 3]);
                        dataControl->PathData.point_ry.push_back(dataControl->operateRicePoints1[i*NUM_DOF + 4]);
                        dataControl->PathData.point_rz.push_back(dataControl->operateRicePoints1[i*NUM_DOF + 5]);
                        dataControl->PathData.acc_time.push_back(0.2);
                        time += 1.0;
                    }

                    rt_printf("path data : \n");
                    for(int8_t i = 0; i < dataControl->PathData.row; i ++){
                        rt_printf("%f, %f, %f, %f, %f, %f, %f, %f\n",
                                  dataControl->PathData.total_time[i],
                                  dataControl->PathData.point_px[i], dataControl->PathData.point_py[i], dataControl->PathData.point_pz[i],
                                  dataControl->PathData.point_rx[i], dataControl->PathData.point_ry[i], dataControl->PathData.point_rz[i],
                                  dataControl->PathData.acc_time[i]);
                    }

                    robotPathGenerate();

                    RobotArm::zyx2mat(dataControl->operateRicePoints[5], dataControl->operateRicePoints[4], dataControl->operateRicePoints[3], dataControl->PathData.movePath[0].R_init);

                    dataControl->ClientToServer.opMode = DataControl::OpMode::RunMode;

                    dataControl->RobotData.run_mode = DataControl::CmdType::RunCmd;
                    dataControl->PathData.path_data_indx = 0;
                    dataControl->PathData.path_struct_indx = 0;
                    dataControl->PathData.cycle_count_max = 1;
                    delay_cnt = 0;
                    delay_cnt_max = 100;

                    break;
                }
                case DataControl::Section::Rice2:{
                    dataControl->PathData.total_time.clear();
                    dataControl->PathData.point_px.clear();
                    dataControl->PathData.point_py.clear();
                    dataControl->PathData.point_pz.clear();
                    dataControl->PathData.point_rx.clear();
                    dataControl->PathData.point_ry.clear();
                    dataControl->PathData.point_rz.clear();
                    dataControl->PathData.point_theta.clear();
                    dataControl->PathData.acc_time.clear();

                    dataControl->PathData.row = 7;
                    double time = 0;
                    for(uint i = 0; i < dataControl->PathData.row; i++){
                        dataControl->PathData.movePath[i].path_x.clear();
                        dataControl->PathData.movePath[i].path_y.clear();
                        dataControl->PathData.movePath[i].path_z.clear();
                        dataControl->PathData.movePath[i].path_theta.clear();

                        dataControl->PathData.total_time.push_back(time);
                        dataControl->PathData.point_px.push_back(dataControl->operateRicePoints2[i*NUM_DOF + 0]);
                        dataControl->PathData.point_py.push_back(dataControl->operateRicePoints2[i*NUM_DOF + 1]);
                        dataControl->PathData.point_pz.push_back(dataControl->operateRicePoints2[i*NUM_DOF + 2]);
                        dataControl->PathData.point_rx.push_back(dataControl->operateRicePoints2[i*NUM_DOF + 3]);
                        dataControl->PathData.point_ry.push_back(dataControl->operateRicePoints2[i*NUM_DOF + 4]);
                        dataControl->PathData.point_rz.push_back(dataControl->operateRicePoints2[i*NUM_DOF + 5]);
                        dataControl->PathData.acc_time.push_back(0.2);
                        time += 1.0;
                    }

                    rt_printf("path data : \n");
                    for(int8_t i = 0; i < dataControl->PathData.row; i ++){
                        rt_printf("%f, %f, %f, %f, %f, %f, %f, %f\n",
                                  dataControl->PathData.total_time[i],
                                  dataControl->PathData.point_px[i], dataControl->PathData.point_py[i], dataControl->PathData.point_pz[i],
                                  dataControl->PathData.point_rx[i], dataControl->PathData.point_ry[i], dataControl->PathData.point_rz[i],
                                  dataControl->PathData.acc_time[i]);
                    }

                    robotPathGenerate();

                    RobotArm::zyx2mat(dataControl->operateRicePoints[5], dataControl->operateRicePoints[4], dataControl->operateRicePoints[3], dataControl->PathData.movePath[0].R_init);

                    dataControl->ClientToServer.opMode = DataControl::OpMode::RunMode;

                    dataControl->RobotData.run_mode = DataControl::CmdType::RunCmd;
                    dataControl->PathData.path_data_indx = 0;
                    dataControl->PathData.path_struct_indx = 0;
                    dataControl->PathData.cycle_count_max = 1;
                    delay_cnt = 0;
                    delay_cnt_max = 100;

                    break;
                }
                case DataControl::Section::Rice3:{
                    dataControl->PathData.total_time.clear();
                    dataControl->PathData.point_px.clear();
                    dataControl->PathData.point_py.clear();
                    dataControl->PathData.point_pz.clear();
                    dataControl->PathData.point_rx.clear();
                    dataControl->PathData.point_ry.clear();
                    dataControl->PathData.point_rz.clear();
                    dataControl->PathData.point_theta.clear();
                    dataControl->PathData.acc_time.clear();

                    dataControl->PathData.row = 7;
                    double time = 0;
                    for(uint i = 0; i < dataControl->PathData.row; i++){
                        dataControl->PathData.movePath[i].path_x.clear();
                        dataControl->PathData.movePath[i].path_y.clear();
                        dataControl->PathData.movePath[i].path_z.clear();
                        dataControl->PathData.movePath[i].path_theta.clear();

                        dataControl->PathData.total_time.push_back(time);
                        dataControl->PathData.point_px.push_back(dataControl->operateRicePoints3[i*NUM_DOF + 0]);
                        dataControl->PathData.point_py.push_back(dataControl->operateRicePoints3[i*NUM_DOF + 1]);
                        dataControl->PathData.point_pz.push_back(dataControl->operateRicePoints3[i*NUM_DOF + 2]);
                        dataControl->PathData.point_rx.push_back(dataControl->operateRicePoints3[i*NUM_DOF + 3]);
                        dataControl->PathData.point_ry.push_back(dataControl->operateRicePoints3[i*NUM_DOF + 4]);
                        dataControl->PathData.point_rz.push_back(dataControl->operateRicePoints3[i*NUM_DOF + 5]);
                        dataControl->PathData.acc_time.push_back(0.2);
                        time += 1.0;
                    }

                    rt_printf("path data : \n");
                    for(int8_t i = 0; i < dataControl->PathData.row; i ++){
                        rt_printf("%f, %f, %f, %f, %f, %f, %f, %f\n",
                                  dataControl->PathData.total_time[i],
                                  dataControl->PathData.point_px[i], dataControl->PathData.point_py[i], dataControl->PathData.point_pz[i],
                                  dataControl->PathData.point_rx[i], dataControl->PathData.point_ry[i], dataControl->PathData.point_rz[i],
                                  dataControl->PathData.acc_time[i]);
                    }

                    robotPathGenerate();

                    RobotArm::zyx2mat(dataControl->operateRicePoints[5], dataControl->operateRicePoints[4], dataControl->operateRicePoints[3], dataControl->PathData.movePath[0].R_init);

                    dataControl->ClientToServer.opMode = DataControl::OpMode::RunMode;

                    dataControl->RobotData.run_mode = DataControl::CmdType::RunCmd;
                    dataControl->PathData.path_data_indx = 0;
                    dataControl->PathData.path_struct_indx = 0;
                    dataControl->PathData.cycle_count_max = 1;
                    delay_cnt = 0;
                    delay_cnt_max = 100;

                    break;
                }
                case DataControl::Section::Rice4:{
                    dataControl->PathData.total_time.clear();
                    dataControl->PathData.point_px.clear();
                    dataControl->PathData.point_py.clear();
                    dataControl->PathData.point_pz.clear();
                    dataControl->PathData.point_rx.clear();
                    dataControl->PathData.point_ry.clear();
                    dataControl->PathData.point_rz.clear();
                    dataControl->PathData.point_theta.clear();
                    dataControl->PathData.acc_time.clear();

                    dataControl->PathData.row = 7;
                    double time = 0;
                    for(uint i = 0; i < dataControl->PathData.row; i++){
                        dataControl->PathData.movePath[i].path_x.clear();
                        dataControl->PathData.movePath[i].path_y.clear();
                        dataControl->PathData.movePath[i].path_z.clear();
                        dataControl->PathData.movePath[i].path_theta.clear();

                        dataControl->PathData.total_time.push_back(time);
                        dataControl->PathData.point_px.push_back(dataControl->operateRicePoints4[i*NUM_DOF + 0]);
                        dataControl->PathData.point_py.push_back(dataControl->operateRicePoints4[i*NUM_DOF + 1]);
                        dataControl->PathData.point_pz.push_back(dataControl->operateRicePoints4[i*NUM_DOF + 2]);
                        dataControl->PathData.point_rx.push_back(dataControl->operateRicePoints4[i*NUM_DOF + 3]);
                        dataControl->PathData.point_ry.push_back(dataControl->operateRicePoints4[i*NUM_DOF + 4]);
                        dataControl->PathData.point_rz.push_back(dataControl->operateRicePoints4[i*NUM_DOF + 5]);
                        dataControl->PathData.acc_time.push_back(0.2);
                        time += 1.0;
                    }

                    rt_printf("path data : \n");
                    for(int8_t i = 0; i < dataControl->PathData.row; i ++){
                        rt_printf("%f, %f, %f, %f, %f, %f, %f, %f\n",
                                  dataControl->PathData.total_time[i],
                                  dataControl->PathData.point_px[i], dataControl->PathData.point_py[i], dataControl->PathData.point_pz[i],
                                  dataControl->PathData.point_rx[i], dataControl->PathData.point_ry[i], dataControl->PathData.point_rz[i],
                                  dataControl->PathData.acc_time[i]);
                    }

                    robotPathGenerate();

                    RobotArm::zyx2mat(dataControl->operateRicePoints[5], dataControl->operateRicePoints[4], dataControl->operateRicePoints[3], dataControl->PathData.movePath[0].R_init);

                    dataControl->ClientToServer.opMode = DataControl::OpMode::RunMode;

                    dataControl->RobotData.run_mode = DataControl::CmdType::RunCmd;
                    dataControl->PathData.path_data_indx = 0;
                    dataControl->PathData.path_struct_indx = 0;
                    dataControl->PathData.cycle_count_max = 1;
                    delay_cnt = 0;
                    delay_cnt_max = 100;

                    break;
                }
                case DataControl::Section::Rice5:{
                    dataControl->PathData.total_time.clear();
                    dataControl->PathData.point_px.clear();
                    dataControl->PathData.point_py.clear();
                    dataControl->PathData.point_pz.clear();
                    dataControl->PathData.point_rx.clear();
                    dataControl->PathData.point_ry.clear();
                    dataControl->PathData.point_rz.clear();
                    dataControl->PathData.point_theta.clear();
                    dataControl->PathData.acc_time.clear();

                    dataControl->PathData.row = 7;
                    double time = 0;
                    for(uint i = 0; i < dataControl->PathData.row; i++){
                        dataControl->PathData.movePath[i].path_x.clear();
                        dataControl->PathData.movePath[i].path_y.clear();
                        dataControl->PathData.movePath[i].path_z.clear();
                        dataControl->PathData.movePath[i].path_theta.clear();

                        dataControl->PathData.total_time.push_back(time);
                        dataControl->PathData.point_px.push_back(dataControl->operateRicePoints5[i*NUM_DOF + 0]);
                        dataControl->PathData.point_py.push_back(dataControl->operateRicePoints5[i*NUM_DOF + 1]);
                        dataControl->PathData.point_pz.push_back(dataControl->operateRicePoints5[i*NUM_DOF + 2]);
                        dataControl->PathData.point_rx.push_back(dataControl->operateRicePoints5[i*NUM_DOF + 3]);
                        dataControl->PathData.point_ry.push_back(dataControl->operateRicePoints5[i*NUM_DOF + 4]);
                        dataControl->PathData.point_rz.push_back(dataControl->operateRicePoints5[i*NUM_DOF + 5]);
                        dataControl->PathData.acc_time.push_back(0.2);
                        time += 1.0;
                    }

                    rt_printf("path data : \n");
                    for(int8_t i = 0; i < dataControl->PathData.row; i ++){
                        rt_printf("%f, %f, %f, %f, %f, %f, %f, %f\n",
                                  dataControl->PathData.total_time[i],
                                  dataControl->PathData.point_px[i], dataControl->PathData.point_py[i], dataControl->PathData.point_pz[i],
                                  dataControl->PathData.point_rx[i], dataControl->PathData.point_ry[i], dataControl->PathData.point_rz[i],
                                  dataControl->PathData.acc_time[i]);
                    }

                    robotPathGenerate();

                    RobotArm::zyx2mat(dataControl->operateRicePoints[5], dataControl->operateRicePoints[4], dataControl->operateRicePoints[3], dataControl->PathData.movePath[0].R_init);

                    dataControl->ClientToServer.opMode = DataControl::OpMode::RunMode;

                    dataControl->RobotData.run_mode = DataControl::CmdType::RunCmd;
                    dataControl->PathData.path_data_indx = 0;
                    dataControl->PathData.path_struct_indx = 0;
                    dataControl->PathData.cycle_count_max = 1;
                    delay_cnt = 0;
                    delay_cnt_max = 100;

                    break;
                }
                case DataControl::Section::Rice6:{
                    dataControl->PathData.total_time.clear();
                    dataControl->PathData.point_px.clear();
                    dataControl->PathData.point_py.clear();
                    dataControl->PathData.point_pz.clear();
                    dataControl->PathData.point_rx.clear();
                    dataControl->PathData.point_ry.clear();
                    dataControl->PathData.point_rz.clear();
                    dataControl->PathData.point_theta.clear();
                    dataControl->PathData.acc_time.clear();

                    dataControl->PathData.row = 7;
                    double time = 0;
                    for(uint i = 0; i < dataControl->PathData.row; i++){
                        dataControl->PathData.movePath[i].path_x.clear();
                        dataControl->PathData.movePath[i].path_y.clear();
                        dataControl->PathData.movePath[i].path_z.clear();
                        dataControl->PathData.movePath[i].path_theta.clear();

                        dataControl->PathData.total_time.push_back(time);
                        dataControl->PathData.point_px.push_back(dataControl->operateRicePoints6[i*NUM_DOF + 0]);
                        dataControl->PathData.point_py.push_back(dataControl->operateRicePoints6[i*NUM_DOF + 1]);
                        dataControl->PathData.point_pz.push_back(dataControl->operateRicePoints6[i*NUM_DOF + 2]);
                        dataControl->PathData.point_rx.push_back(dataControl->operateRicePoints6[i*NUM_DOF + 3]);
                        dataControl->PathData.point_ry.push_back(dataControl->operateRicePoints6[i*NUM_DOF + 4]);
                        dataControl->PathData.point_rz.push_back(dataControl->operateRicePoints6[i*NUM_DOF + 5]);
                        dataControl->PathData.acc_time.push_back(0.2);
                        time += 1.0;
                    }

                    rt_printf("path data : \n");
                    for(int8_t i = 0; i < dataControl->PathData.row; i ++){
                        rt_printf("%f, %f, %f, %f, %f, %f, %f, %f\n",
                                  dataControl->PathData.total_time[i],
                                  dataControl->PathData.point_px[i], dataControl->PathData.point_py[i], dataControl->PathData.point_pz[i],
                                  dataControl->PathData.point_rx[i], dataControl->PathData.point_ry[i], dataControl->PathData.point_rz[i],
                                  dataControl->PathData.acc_time[i]);
                    }

                    robotPathGenerate();

                    RobotArm::zyx2mat(dataControl->operateRicePoints[5], dataControl->operateRicePoints[4], dataControl->operateRicePoints[3], dataControl->PathData.movePath[0].R_init);

                    dataControl->ClientToServer.opMode = DataControl::OpMode::RunMode;

                    dataControl->RobotData.run_mode = DataControl::CmdType::RunCmd;
                    dataControl->PathData.path_data_indx = 0;
                    dataControl->PathData.path_struct_indx = 0;
                    dataControl->PathData.cycle_count_max = 1;
                    delay_cnt = 0;
                    delay_cnt_max = 100;

                    break;
                }
                case DataControl::Section::Rice7:{
                    dataControl->PathData.total_time.clear();
                    dataControl->PathData.point_px.clear();
                    dataControl->PathData.point_py.clear();
                    dataControl->PathData.point_pz.clear();
                    dataControl->PathData.point_rx.clear();
                    dataControl->PathData.point_ry.clear();
                    dataControl->PathData.point_rz.clear();
                    dataControl->PathData.point_theta.clear();
                    dataControl->PathData.acc_time.clear();

                    dataControl->PathData.row = 7;
                    double time = 0;
                    for(uint i = 0; i < dataControl->PathData.row; i++){
                        dataControl->PathData.movePath[i].path_x.clear();
                        dataControl->PathData.movePath[i].path_y.clear();
                        dataControl->PathData.movePath[i].path_z.clear();
                        dataControl->PathData.movePath[i].path_theta.clear();

                        dataControl->PathData.total_time.push_back(time);
                        dataControl->PathData.point_px.push_back(dataControl->operateRicePoints7[i*NUM_DOF + 0]);
                        dataControl->PathData.point_py.push_back(dataControl->operateRicePoints7[i*NUM_DOF + 1]);
                        dataControl->PathData.point_pz.push_back(dataControl->operateRicePoints7[i*NUM_DOF + 2]);
                        dataControl->PathData.point_rx.push_back(dataControl->operateRicePoints7[i*NUM_DOF + 3]);
                        dataControl->PathData.point_ry.push_back(dataControl->operateRicePoints7[i*NUM_DOF + 4]);
                        dataControl->PathData.point_rz.push_back(dataControl->operateRicePoints7[i*NUM_DOF + 5]);
                        dataControl->PathData.acc_time.push_back(0.2);
                        time += 1.0;
                    }

                    rt_printf("path data : \n");
                    for(int8_t i = 0; i < dataControl->PathData.row; i ++){
                        rt_printf("%f, %f, %f, %f, %f, %f, %f, %f\n",
                                  dataControl->PathData.total_time[i],
                                  dataControl->PathData.point_px[i], dataControl->PathData.point_py[i], dataControl->PathData.point_pz[i],
                                  dataControl->PathData.point_rx[i], dataControl->PathData.point_ry[i], dataControl->PathData.point_rz[i],
                                  dataControl->PathData.acc_time[i]);
                    }

                    robotPathGenerate();

                    RobotArm::zyx2mat(dataControl->operateRicePoints[5], dataControl->operateRicePoints[4], dataControl->operateRicePoints[3], dataControl->PathData.movePath[0].R_init);

                    dataControl->ClientToServer.opMode = DataControl::OpMode::RunMode;

                    dataControl->RobotData.run_mode = DataControl::CmdType::RunCmd;
                    dataControl->PathData.path_data_indx = 0;
                    dataControl->PathData.path_struct_indx = 0;
                    dataControl->PathData.cycle_count_max = 1;
                    delay_cnt = 0;
                    delay_cnt_max = 100;

                    break;
                }
                case DataControl::Section::Rice8:{
                    dataControl->PathData.total_time.clear();
                    dataControl->PathData.point_px.clear();
                    dataControl->PathData.point_py.clear();
                    dataControl->PathData.point_pz.clear();
                    dataControl->PathData.point_rx.clear();
                    dataControl->PathData.point_ry.clear();
                    dataControl->PathData.point_rz.clear();
                    dataControl->PathData.point_theta.clear();
                    dataControl->PathData.acc_time.clear();

                    dataControl->PathData.row = 7;
                    double time = 0;
                    for(uint i = 0; i < dataControl->PathData.row; i++){
                        dataControl->PathData.movePath[i].path_x.clear();
                        dataControl->PathData.movePath[i].path_y.clear();
                        dataControl->PathData.movePath[i].path_z.clear();
                        dataControl->PathData.movePath[i].path_theta.clear();

                        dataControl->PathData.total_time.push_back(time);
                        dataControl->PathData.point_px.push_back(dataControl->operateRicePoints8[i*NUM_DOF + 0]);
                        dataControl->PathData.point_py.push_back(dataControl->operateRicePoints8[i*NUM_DOF + 1]);
                        dataControl->PathData.point_pz.push_back(dataControl->operateRicePoints8[i*NUM_DOF + 2]);
                        dataControl->PathData.point_rx.push_back(dataControl->operateRicePoints8[i*NUM_DOF + 3]);
                        dataControl->PathData.point_ry.push_back(dataControl->operateRicePoints8[i*NUM_DOF + 4]);
                        dataControl->PathData.point_rz.push_back(dataControl->operateRicePoints8[i*NUM_DOF + 5]);
                        dataControl->PathData.acc_time.push_back(0.2);
                        time += 1.0;
                    }

                    rt_printf("path data : \n");
                    for(int8_t i = 0; i < dataControl->PathData.row; i ++){
                        rt_printf("%f, %f, %f, %f, %f, %f, %f, %f\n",
                                  dataControl->PathData.total_time[i],
                                  dataControl->PathData.point_px[i], dataControl->PathData.point_py[i], dataControl->PathData.point_pz[i],
                                  dataControl->PathData.point_rx[i], dataControl->PathData.point_ry[i], dataControl->PathData.point_rz[i],
                                  dataControl->PathData.acc_time[i]);
                    }

                    robotPathGenerate();

                    RobotArm::zyx2mat(dataControl->operateRicePoints[5], dataControl->operateRicePoints[4], dataControl->operateRicePoints[3], dataControl->PathData.movePath[0].R_init);

                    dataControl->ClientToServer.opMode = DataControl::OpMode::RunMode;

                    dataControl->RobotData.run_mode = DataControl::CmdType::RunCmd;
                    dataControl->PathData.path_data_indx = 0;
                    dataControl->PathData.path_struct_indx = 0;
                    dataControl->PathData.cycle_count_max = 1;
                    delay_cnt = 0;
                    delay_cnt_max = 100;

                    break;
                }
                case DataControl::Section::Rice9:{
                    dataControl->PathData.total_time.clear();
                    dataControl->PathData.point_px.clear();
                    dataControl->PathData.point_py.clear();
                    dataControl->PathData.point_pz.clear();
                    dataControl->PathData.point_rx.clear();
                    dataControl->PathData.point_ry.clear();
                    dataControl->PathData.point_rz.clear();
                    dataControl->PathData.point_theta.clear();
                    dataControl->PathData.acc_time.clear();

                    dataControl->PathData.row = 7;
                    double time = 0;
                    for(uint i = 0; i < dataControl->PathData.row; i++){
                        dataControl->PathData.movePath[i].path_x.clear();
                        dataControl->PathData.movePath[i].path_y.clear();
                        dataControl->PathData.movePath[i].path_z.clear();
                        dataControl->PathData.movePath[i].path_theta.clear();

                        dataControl->PathData.total_time.push_back(time);
                        dataControl->PathData.point_px.push_back(dataControl->operateRicePoints9[i*NUM_DOF + 0]);
                        dataControl->PathData.point_py.push_back(dataControl->operateRicePoints9[i*NUM_DOF + 1]);
                        dataControl->PathData.point_pz.push_back(dataControl->operateRicePoints9[i*NUM_DOF + 2]);
                        dataControl->PathData.point_rx.push_back(dataControl->operateRicePoints9[i*NUM_DOF + 3]);
                        dataControl->PathData.point_ry.push_back(dataControl->operateRicePoints9[i*NUM_DOF + 4]);
                        dataControl->PathData.point_rz.push_back(dataControl->operateRicePoints9[i*NUM_DOF + 5]);
                        dataControl->PathData.acc_time.push_back(0.2);
                        time += 1.0;
                    }

                    rt_printf("path data : \n");
                    for(int8_t i = 0; i < dataControl->PathData.row; i ++){
                        rt_printf("%f, %f, %f, %f, %f, %f, %f, %f\n",
                                  dataControl->PathData.total_time[i],
                                  dataControl->PathData.point_px[i], dataControl->PathData.point_py[i], dataControl->PathData.point_pz[i],
                                  dataControl->PathData.point_rx[i], dataControl->PathData.point_ry[i], dataControl->PathData.point_rz[i],
                                  dataControl->PathData.acc_time[i]);
                    }

                    robotPathGenerate();

                    RobotArm::zyx2mat(dataControl->operateRicePoints[5], dataControl->operateRicePoints[4], dataControl->operateRicePoints[3], dataControl->PathData.movePath[0].R_init);

                    dataControl->ClientToServer.opMode = DataControl::OpMode::RunMode;

                    dataControl->RobotData.run_mode = DataControl::CmdType::RunCmd;
                    dataControl->PathData.path_data_indx = 0;
                    dataControl->PathData.path_struct_indx = 0;
                    dataControl->PathData.cycle_count_max = 1;
                    delay_cnt = 0;
                    delay_cnt_max = 100;

                    break;
                }
                case DataControl::Section::Mouse:
                {
                    dataControl->PathData.total_time.clear();
                    dataControl->PathData.point_px.clear();
                    dataControl->PathData.point_py.clear();
                    dataControl->PathData.point_pz.clear();
                    dataControl->PathData.point_rx.clear();
                    dataControl->PathData.point_ry.clear();
                    dataControl->PathData.point_rz.clear();
                    dataControl->PathData.point_theta.clear();
                    dataControl->PathData.acc_time.clear();

                    dataControl->PathData.row = 2;
                    for(uint i = 0; i < dataControl->PathData.row; i++){
                        dataControl->PathData.movePath[i].path_x.clear();
                        dataControl->PathData.movePath[i].path_y.clear();
                        dataControl->PathData.movePath[i].path_z.clear();
                        dataControl->PathData.movePath[i].path_theta.clear();
                    }

                    dataControl->PathData.total_time.push_back(0);
                    dataControl->PathData.point_px.push_back(dataControl->operateFeedingReadyPose[0]);
                    dataControl->PathData.point_py.push_back(dataControl->operateFeedingReadyPose[1]);
                    dataControl->PathData.point_pz.push_back(dataControl->operateFeedingReadyPose[2]);
                    dataControl->PathData.point_rx.push_back(dataControl->operateFeedingReadyPose[3]);
                    dataControl->PathData.point_ry.push_back(dataControl->operateFeedingReadyPose[4]);
                    dataControl->PathData.point_rz.push_back(dataControl->operateFeedingReadyPose[5]);
                    dataControl->PathData.acc_time.push_back(0.5);

                    dataControl->PathData.total_time.push_back(3);
                    dataControl->PathData.point_px.push_back(dataControl->PathData.teaching_pose[0]);
                    dataControl->PathData.point_py.push_back(dataControl->PathData.teaching_pose[1]);
                    dataControl->PathData.point_pz.push_back(dataControl->PathData.teaching_pose[2]);
                    dataControl->PathData.point_rx.push_back(dataControl->PathData.teaching_pose[3]);
                    dataControl->PathData.point_ry.push_back(dataControl->PathData.teaching_pose[4]);
                    dataControl->PathData.point_rz.push_back(dataControl->PathData.teaching_pose[5]);
                    dataControl->PathData.acc_time.push_back(0.5);

                    for(int8_t i = 0; i < dataControl->PathData.row; i ++){
                        rt_printf("%f, %f, %f, %f, %f, %f, %f, %f\n",
                                  dataControl->PathData.total_time[i],
                                  dataControl->PathData.point_px[i], dataControl->PathData.point_py[i], dataControl->PathData.point_pz[i],
                                  dataControl->PathData.point_rx[i], dataControl->PathData.point_ry[i], dataControl->PathData.point_rz[i],
                                  dataControl->PathData.acc_time[i]);
                    }

                    robotPathGenerate();

                    RobotArm::zyx2mat(dataControl->operateFeedingReadyPose[5], dataControl->operateFeedingReadyPose[4], dataControl->operateFeedingReadyPose[3], dataControl->PathData.movePath[0].R_init);

                    dataControl->ClientToServer.opMode = DataControl::OpMode::RunMode;

                    dataControl->RobotData.run_mode = DataControl::CmdType::RunCmd;
                    dataControl->PathData.path_data_indx = 0;
                    dataControl->PathData.path_struct_indx = 0;
                    dataControl->PathData.cycle_count_max = 1;
                    delay_cnt = 0;

                    break;
                }
                case DataControl::Section::Home:
                {
                    RobotArm::zyx2mat(dataControl->RobotData.desired_end_pose[5], dataControl->RobotData.desired_end_pose[4], dataControl->RobotData.desired_end_pose[3],
                            dataControl->PathData.movePath[0].R_init);
////                    rt_printf("case DataControl::Section::Home\n");

                    dataControl->ClientToServer.opMode = DataControl::OpMode::RunMode;

                    dataControl->RobotData.run_mode = DataControl::CmdType::RunCmd;
                    dataControl->PathData.path_data_indx = 0;
                    dataControl->PathData.path_struct_indx = 0;
                    dataControl->PathData.cycle_count_max = 1;
                    delay_cnt = 0;
////                    uint32_t profile_acc[6] = {0,};
////                    uint32_t profile_vel[6] = {0,};
////                    for(unsigned int i = 0; i < NUM_JOINT; i++){
////                        profile_acc[i] = init_profile_acc[i] - 3;
////                        profile_vel[i] = init_profile_vel[i] - 3;
////                    }
////                    module->setGroupSyncWriteIndirectAddress(profile_acc, profile_vel, init_pos_p_gain, NUM_JOINT);
//                    memcpy(dataControl->RobotData.desired_q, dataControl->operateReadyJoint, sizeof(double)*NUM_JOINT);
//                    dataControl->jointPositionRAD2ENC(dataControl->RobotData.desired_q, dataControl->RobotData.command_joint_position);

//                    module->setGroupSyncWriteGoalPosition(dataControl->RobotData.command_joint_position, NUM_JOINT);
//                    dataControl->feeding = false;

                    break;
                }
            }
            dataControl->jointPositionRAD2ENC(dataControl->RobotData.desired_q, dataControl->RobotData.command_joint_position);

            module->setGroupSyncWriteGoalPosition(dataControl->RobotData.command_joint_position, NUM_JOINT);
//            usleep(module->feeding_profile_vel[0]*1000);
            break;
        }
        case DataControl::Operate::ReadyFeeding:{
            for(uint i = 0; i < NUM_JOINT; i++){
                module->feeding_profile_acc[i] = 1;//50;
                module->feeding_profile_vel[i] = 3;//static_cast<uint32_t>((500/64.0)*module->feeding_profile_acc[i]);
                module->feeding_pos_p_gain[i] = default_pos_p_gain[i];
            }
            module->setGroupSyncWriteIndirectAddress(module->feeding_profile_acc, module->feeding_profile_vel, module->feeding_pos_p_gain, NUM_JOINT);
            for(int i = 0; i < 6; i++){
                dataControl->RobotData.desired_q[i] = dataControl->obi_ready_joint[dataControl->obi_section_indx*6 + i];
            }
            dataControl->jointPositionRAD2ENC(dataControl->RobotData.desired_q, dataControl->RobotData.command_joint_position);
            module->setGroupSyncWriteGoalPosition(dataControl->RobotData.command_joint_position, NUM_JOINT);
            usleep(module->feeding_profile_vel[0]*1000);
            dataControl->ClientToServer.opMode = DataControl::OpMode::Wait;


//            dataControl->PathData.total_time.clear();
//            dataControl->PathData.point_px.clear();
//            dataControl->PathData.point_py.clear();
//            dataControl->PathData.point_pz.clear();
//            dataControl->PathData.point_rx.clear();
//            dataControl->PathData.point_ry.clear();
//            dataControl->PathData.point_rz.clear();
//            dataControl->PathData.point_theta.clear();
//            dataControl->PathData.acc_time.clear();

//            dataControl->PathData.row = 2;
//            for(uint i = 0; i < dataControl->PathData.row; i++){
//                dataControl->PathData.movePath[i].path_x.clear();
//                dataControl->PathData.movePath[i].path_y.clear();
//                dataControl->PathData.movePath[i].path_z.clear();
//                dataControl->PathData.movePath[i].path_theta.clear();
//            }

//            dataControl->PathData.total_time.push_back(0);
//            dataControl->PathData.point_px.push_back(dataControl->operateReadyPose[0]);
//            dataControl->PathData.point_py.push_back(dataControl->operateReadyPose[1]);
//            dataControl->PathData.point_pz.push_back(dataControl->operateReadyPose[2]);
//            dataControl->PathData.point_rx.push_back(dataControl->operateReadyPose[3]);
//            dataControl->PathData.point_ry.push_back(dataControl->operateReadyPose[4]);
//            dataControl->PathData.point_rz.push_back(dataControl->operateReadyPose[5]);
//            dataControl->PathData.acc_time.push_back(0.5);

//            dataControl->PathData.total_time.push_back(3);
//            dataControl->PathData.point_px.push_back(dataControl->obi_ready_pose[dataControl->obi_section_indx*6 + 0]);
//            dataControl->PathData.point_py.push_back(dataControl->obi_ready_pose[dataControl->obi_section_indx*6 + 1]);
//            dataControl->PathData.point_pz.push_back(dataControl->obi_ready_pose[dataControl->obi_section_indx*6 + 2]);
//            dataControl->PathData.point_rx.push_back(dataControl->operateReadyPose[3]);
//            dataControl->PathData.point_ry.push_back(dataControl->operateReadyPose[4]);
//            dataControl->PathData.point_rz.push_back(dataControl->operateReadyPose[5]);
//            dataControl->PathData.acc_time.push_back(0.5);

//            for(int8_t i = 0; i < dataControl->PathData.row; i ++){
//                rt_printf("%f, %f, %f, %f, %f, %f, %f, %f\n",
//                          dataControl->PathData.total_time[i],
//                          dataControl->PathData.point_px[i], dataControl->PathData.point_py[i], dataControl->PathData.point_pz[i],
//                          dataControl->PathData.point_rx[i], dataControl->PathData.point_ry[i], dataControl->PathData.point_rz[i],
//                          dataControl->PathData.acc_time[i]);
//            }

//            robotPathGenerate();

//            RobotArm::zyx2mat(dataControl->operateReadyPose[5], dataControl->operateReadyPose[4], dataControl->operateReadyPose[3], dataControl->PathData.movePath[0].R_init);

//            dataControl->ClientToServer.opMode = DataControl::OpMode::RunMode;

//            dataControl->RobotData.run_mode = DataControl::CmdType::CustomRun;
//            dataControl->PathData.path_data_indx = 0;
//            dataControl->PathData.path_struct_indx = 0;
//            dataControl->PathData.cycle_count_max = 1;
//            delay_cnt = 0;

            break;
        }
    }
}

void ControlMain::robotVSD()
{
    robotArm->run_kinematics(dataControl->RobotData.present_q, dataControl->RobotData.present_end_pose);

    robotArm->gravity_compensation(dataControl->RobotData.present_q, dataControl->RobotData.present_q_dot, dataControl->RobotData.Tg);

    robotArm->jacobian_zyx();

    for(uint i = 0; i < 6; i++)
    {
        dataControl->RobotData.present_end_vel[i] = 0;
        for(uint j = 0; j < 6; j++)
        {
            dataControl->RobotData.present_end_vel[i] += robotArm->J[i*6 + j]*dataControl->RobotData.present_q_dot[j];
        }
    }

    dataControl->RobotData.Kp = 260;
    dataControl->RobotData.Dp = 0.25;
    dataControl->RobotData.Kr = 10;
    dataControl->RobotData.Dr = 0.05;

    for(int i = 0; i < 3; i++){
        dataControl->RobotData.F[i] =
                dataControl->RobotData.Kp*(dataControl->RobotData.desired_end_pose_zyx[i] - dataControl->RobotData.present_end_pose_zyx[i]) -
                dataControl->RobotData.Dp*(dataControl->RobotData.present_end_vel[i]);
        dataControl->RobotData.F[i+3] =
                dataControl->RobotData.Kr*(dataControl->RobotData.desired_end_pose_zyx[i + 3] - dataControl->RobotData.present_end_pose_zyx[i + 3]) -
                dataControl->RobotData.Dr*(dataControl->RobotData.present_end_vel[i + 3]);
    }

    for(uint i = 0; i < 6; i++){
        dataControl->RobotData.Td[i] = 0;
        for(uint j = 0; j < 6; j++){
            dataControl->RobotData.Td[i] += robotArm->J[j*6 + i]*dataControl->RobotData.F[j];
        }
    }

    dataControl->RobotData.T_limit[0] = 20.0;
    dataControl->RobotData.T_limit[1] = 20.0;
    dataControl->RobotData.T_limit[2] = 20.0;
    dataControl->RobotData.T_limit[3] = 20.0;
    dataControl->RobotData.T_limit[4] = 20.0;
    dataControl->RobotData.T_limit[5] = 20.0;

    dataControl->RobotData.T_err = false;
    for(int i = 0; i < 6; i++){
        if(abs(dataControl->RobotData.Td[i]) > dataControl->RobotData.T_limit[i]){
            dataControl->RobotData.T_err = true;
            rt_printf("\n\tVSD Torque exceed torque limit\n");
            rt_printf("\tTorque vsd : %f\t %f\t %f\t %f\t %f\t %f\n\n",
                      dataControl->RobotData.Td[0], dataControl->RobotData.Td[1], dataControl->RobotData.Td[2],
                    dataControl->RobotData.Td[3], dataControl->RobotData.Td[4], dataControl->RobotData.Td[5]);
            break;
        }
    }
    if(dataControl->RobotData.T_err)
        memset(dataControl->RobotData.Td, 0, sizeof(double)*6);

    for(int i = 0; i < 6; i++){
        dataControl->RobotData.T[i] = dataControl->RobotData.Td[i] + dataControl->RobotData.Tg[i];
    }

//    rt_printf("Desired Pose : %f, %f, %f, %f, %f, %f\n",
//              dataControl->RobotData.desired_end_pose_zyx[0], dataControl->RobotData.desired_end_pose_zyx[1],
//            dataControl->RobotData.desired_end_pose_zyx[2], dataControl->RobotData.desired_end_pose_zyx[3],
//            dataControl->RobotData.desired_end_pose_zyx[4], dataControl->RobotData.desired_end_pose_zyx[5]);

//    rt_printf("Present Pose : %f, %f, %f, %f, %f, %f\n",
//              dataControl->RobotData.present_end_pose_zyx[0], dataControl->RobotData.present_end_pose_zyx[1],
//            dataControl->RobotData.present_end_pose_zyx[2], dataControl->RobotData.present_end_pose_zyx[3],
//            dataControl->RobotData.present_end_pose_zyx[4], dataControl->RobotData.present_end_pose_zyx[5]);

//    rt_printf("Present Velocity : %f, %f, %f, %f, %f, %f\n",
//              dataControl->RobotData.present_end_vel[0], dataControl->RobotData.present_end_vel[1],
//            dataControl->RobotData.present_end_vel[2], dataControl->RobotData.present_end_vel[3],
//            dataControl->RobotData.present_end_vel[4], dataControl->RobotData.present_end_vel[5]);
//    rt_printf("Present Joint Velocity : %f, %f, %f, %f, %f, %f\n",
//              dataControl->RobotData.present_q_dot[0], dataControl->RobotData.present_q_dot[1],
//            dataControl->RobotData.present_q_dot[2], dataControl->RobotData.present_q_dot[3],
//            dataControl->RobotData.present_q_dot[4], dataControl->RobotData.present_q_dot[5]);
//    rt_printf("Error : %f, %f, %f, %f, %f, %f\n\n",
//              dataControl->RobotData.desired_end_pose_zyx[0] - dataControl->RobotData.present_end_pose_zyx[0],
//            dataControl->RobotData.desired_end_pose_zyx[1] - dataControl->RobotData.present_end_pose_zyx[1],
//            dataControl->RobotData.desired_end_pose_zyx[2] - dataControl->RobotData.present_end_pose_zyx[2],
//            dataControl->RobotData.desired_end_pose_zyx[3] - dataControl->RobotData.present_end_pose_zyx[3],
//            dataControl->RobotData.desired_end_pose_zyx[4] - dataControl->RobotData.present_end_pose_zyx[4],
//            dataControl->RobotData.desired_end_pose_zyx[5] - dataControl->RobotData.present_end_pose_zyx[5]);

//    rt_printf("Torque g : %f, %f, %f, %f, %f, %f\n",
//              dataControl->RobotData.Tg[0], dataControl->RobotData.Tg[1], dataControl->RobotData.Tg[2],
//            dataControl->RobotData.Tg[3], dataControl->RobotData.Tg[4], dataControl->RobotData.Tg[5]);
//    rt_printf("Torque vsd : %f, %f, %f, %f, %f, %f\n",
//              dataControl->RobotData.Td[0], dataControl->RobotData.Td[1], dataControl->RobotData.Td[2],
//            dataControl->RobotData.Td[3], dataControl->RobotData.Td[4], dataControl->RobotData.Td[5]);
//    rt_printf("Force : %f, %f, %f, %f, %f, %f\n",
//              dataControl->RobotData.F[0], dataControl->RobotData.F[1], dataControl->RobotData.F[2],
//            dataControl->RobotData.F[3], dataControl->RobotData.F[4], dataControl->RobotData.F[5]);
//    rt_printf("Torque : %f, %f, %f, %f, %f, %f\n",
//              dataControl->RobotData.T[0], dataControl->RobotData.T[1], dataControl->RobotData.T[2],
//            dataControl->RobotData.T[3], dataControl->RobotData.T[4], dataControl->RobotData.T[5]);
}

void ControlMain::goalReach(double desired_pose[], double present_pose[], bool *goal_reach)
{
    double epsilon_pos = 0.05;
    double epsilon_ang = 1;

    double pos = sqrt(pow(desired_pose[0] - present_pose[0], 2) + pow(desired_pose[1] - present_pose[1], 2) + pow(desired_pose[2] - present_pose[2], 2));
    double ang_r = abs(desired_pose[3] - present_pose[3]);
    double ang_p = abs(desired_pose[4] - present_pose[4]);
    double ang_y = abs(desired_pose[5] - present_pose[5]);

//    rt_printf("pos : %f\n", pos);
//    rt_printf("ang_r : %f\t ang_p : %f\t ang_y : %f\n", ang_r, ang_p, ang_y);

    if (pos < epsilon_pos/* && ang_r < epsilon_ang && ang_p < epsilon_ang && ang_y < epsilon_ang*/){
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
