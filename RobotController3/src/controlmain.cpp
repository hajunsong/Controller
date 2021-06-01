#include "controlmain.h"

ControlMain::ControlMain()
{
    dataControl = new DataControl();
    tcpServer = new TcpServer(dataControl);
    tcpServerLatte = new TcpServer(dataControl);

    dataControl->RobotData.module_init = false;
    module = new DxlControl();

    data_indx = 0;
    module_indx = 0;
    step_size = 0.003;

    robotArm = new RobotArm(NUM_JOINT, NUM_DOF, step_size, dataControl->MODULE_TYPE);
    robotArm->set_tool_offset(dataControl->tool_offset);

    ready_pose = false;
    cartesian_move_flag = false;

    delay_cnt_max = 100;
    delay_cnt = 0;
    fork_cnt_max = 0;
    fork_cnt = 0;
    dataControl->feeding = false;

    controlMainCustom = new ControlMainCustom();

    init_thread_run = false;

    robot_task_run = false;
    gpio_task_run = false;
    key_input_task_run = false;

    gpio0_state = 0;
    gpio1_state = 0;
    gpio0_state_old = 0;
    gpio1_state_old = 0;
    gpio0 = TP_PORT::port0;
    gpio1 = TP_PORT::port1;

    key_value = 0;
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

    robot_rt_stop();
    usleep(10000);
    delete controlMainCustom;
    printf("Complete destructure ControlMainCustom\n");
    usleep(10000);

    if(gpio_task_run){
        gpio_task_run = false;
        rt_task_join(&gpio_task);
        usleep(10000);
        rt_task_delete(&gpio_task);
        usleep(10000);
        printf("Finished GPIO Task\n");
    }

    if(key_input_task_run){
        key_input_task_run = false;
        rt_task_join(&key_input_task);
        usleep(10000);
        rt_task_delete(&key_input_task);
        usleep(10000);
        printf("Finished Key Input Task\n");
    }
    delete tcpServer;
    usleep(10000);
    printf("Complete destructure TcpServer\n");
    usleep(10000);
    delete tcpServerLatte;
    usleep(10000);
    printf("Complete destructure TcpServerLatte\n");
    delete dataControl;
    printf("Complete destructure DataControl\n");
    delete robotArm;
    printf("Complete destructure RobotArm\n");

    usleep(10000);
    if (dataControl->RobotData.module_init){
//        if(dataControl->tablet_mode){
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

    gpio_rt_start();
    usleep(10000);

//    key_input_rt_start();
//    usleep(10000);

    tcpServer->setting(5050);
    tcpServer->start();
    usleep(10000);

    tcpServerLatte->setting(5053);
    tcpServerLatte->start();
    usleep(10000);

    init_thread_run = true;
    pthread_create(&init_thread, nullptr, init_func, this);
}

void ControlMain::gpio_rt_start()
{
    mlockall(MCL_CURRENT|MCL_FUTURE);

    /* Perform auto-init of rt_print buffers if the task doesn't do so */
    rt_print_auto_init(1);

    if (tp_gpio_init()!=0)	//initialize port, must be called before used
    {
        printf("GPIO Init. Error!\n");
        return;
    }

    tp_gpio_set_dir(gpio0, 0x38);
    tp_gpio_set_dir(gpio1, 0x0F);

    printf("Running, check the GPIO state....\n");

    //create and start a realtime periodic task for writing gpio
    rt_task_create(&gpio_task, "rtGpioTask", 0, 99, 0);
    rt_task_start(&gpio_task, &gpio_RT, this);
}

void ControlMain::key_input_rt_start()
{
    mlockall(MCL_CURRENT|MCL_FUTURE);

    /* Perform auto-init of rt_print buffers if the task doesn't do so */
    rt_print_auto_init(1);

    printf("Running, Keyboard Input Manager....\n");

    //create and start a realtime periodic task for writing gpio
    rt_task_create(&key_input_task, "rtKeyTask", 0, 99, 0);
    rt_task_start(&key_input_task, &key_input_RT, this);
}

void ControlMain::gpio_RT(void *arg)
{
    ControlMain *pControlMain = static_cast<ControlMain*>(arg);
    int step = 0;
    unsigned int count = 0;
    bool led = true;
    const unsigned int led_onoff_count = 5;

    rt_task_set_periodic(NULL, TM_NOW, 100e6);

    pControlMain->key_value = 0;

    tp_writeport(pControlMain->gpio0, 0x00);
    tp_writeport(pControlMain->gpio1, 0x01);

    pControlMain->gpio_task_run = true;
    pControlMain->dataControl->KITECHData.tablet_connect = false;

    while(pControlMain->gpio_task_run){
        rt_task_wait_period(NULL);

        pControlMain->gpio0_state = tp_readport(pControlMain->gpio0);
        pControlMain->gpio1_state = tp_readport(pControlMain->gpio1);

        count++;
        if(count < led_onoff_count){
            led = true;
        }
        else if(count >= led_onoff_count && count < led_onoff_count*2){
            led = false;
        }
        else{
            count = 0;
        }

        if((pControlMain->gpio0_state) != (pControlMain->gpio0_state_old)){
#if PRINT_ON
            rt_printf("GPIO 0 : \t");
            rt_printf((pControlMain->gpio0_state & pin07) == pin07 ? "1\t" : "0\t");
            rt_printf((pControlMain->gpio0_state & pin06) == pin06 ? "1\t" : "0\t");
            rt_printf((pControlMain->gpio0_state & pin05) == pin05 ? "1\t" : "0\t");
            rt_printf((pControlMain->gpio0_state & pin04) == pin04 ? "1\t" : "0\t");
            rt_printf((pControlMain->gpio0_state & pin03) == pin03 ? "1\t" : "0\t");
            rt_printf((pControlMain->gpio0_state & pin02) == pin02 ? "1\t" : "0\t");
            rt_printf((pControlMain->gpio0_state & pin01) == pin01 ? "1\t" : "0\t");
            rt_printf((pControlMain->gpio0_state & pin00) == pin00 ? "1\t" : "0\t");
            rt_printf("\n");
            rt_printf("GPIO 1 : \t");
            rt_printf((pControlMain->gpio1_state & pin07) == pin07 ? "1\t" : "0\t");
            rt_printf((pControlMain->gpio1_state & pin06) == pin06 ? "1\t" : "0\t");
            rt_printf((pControlMain->gpio1_state & pin05) == pin05 ? "1\t" : "0\t");
            rt_printf((pControlMain->gpio1_state & pin04) == pin04 ? "1\t" : "0\t");
            rt_printf((pControlMain->gpio1_state & pin03) == pin03 ? "1\t" : "0\t");
            rt_printf((pControlMain->gpio1_state & pin02) == pin02 ? "1\t" : "0\t");
            rt_printf((pControlMain->gpio1_state & pin01) == pin01 ? "1\t" : "0\t");
            rt_printf((pControlMain->gpio1_state & pin00) == pin00 ? "1\t" : "0\t");
            rt_printf("\n");
            rt_printf("\n");
#endif

            if((~pControlMain->gpio0_state & pin00) == pin00)
            {
                pControlMain->key_value = KEY_Z;
                pControlMain->gpio1_state |= pin01;
            }
            else if((~pControlMain->gpio0_state & pin01) == pin01){
                pControlMain->key_value = KEY_X;
                pControlMain->gpio1_state |= pin02;
            }
            else if((~pControlMain->gpio0_state & pin02) == pin02){
                pControlMain->key_value = KEY_C;
                pControlMain->gpio1_state |= pin03;
            }
            tp_writeport(pControlMain->gpio1, pControlMain->gpio1_state);
            usleep(5000);

            rt_printf("key value : %d\n", pControlMain->key_value);

            if(pControlMain->key_value == KEY_Z){
                pControlMain->gpio0_state &= ~(pin03|pin04|pin05);
                tp_writeport(pControlMain->gpio0, pControlMain->gpio0_state);
                usleep(5000);

                if(pControlMain->dataControl->tablet_mode){
                    pControlMain->unsetTabletMode();
                    step = 0;
                }
                else{
                    pControlMain->setTabletMode();
                    pControlMain->gpio0_state |= pin03;
                    tp_writeport(pControlMain->gpio0, pControlMain->gpio0_state);
                    step = 1;
                }
            }

            else if(pControlMain->key_value == KEY_X){
                if(pControlMain->dataControl->operateMode.mode == DataControl::Operate::Start){
                    pControlMain->dataControl->ClientToServer.opMode = DataControl::OpMode::OperateMode;
                    pControlMain->dataControl->operateMode.mode = DataControl::Operate::StartTeaching;
                    pControlMain->gpio0_state |= pin04;
                    tp_writeport(pControlMain->gpio0, pControlMain->gpio0_state);
                    usleep(5000);
                    step = 2;
                }
                else if(pControlMain->dataControl->operateMode.mode == DataControl::Operate::StartTeaching){
                    pControlMain->dataControl->ClientToServer.opMode = DataControl::OpMode::OperateMode;
                    pControlMain->dataControl->operateMode.mode = DataControl::Operate::StopTeaching;
                    usleep(10000);
                    while(!(pControlMain->dataControl->ClientToServer.opMode == DataControl::OpMode::Wait)){
                        usleep(10000);
                    }
                    pControlMain->gpio0_state &= ~pin04;
                    tp_writeport(pControlMain->gpio0, pControlMain->gpio0_state);
                    step = 3;
                }
            }
            else if(pControlMain->key_value == KEY_C){
                if(pControlMain->dataControl->operateMode.mode == DataControl::Operate::StopTeaching){
                    pControlMain->dataControl->ClientToServer.opMode = DataControl::OpMode::OperateMode;
                    pControlMain->dataControl->operateMode.mode = DataControl::Operate::StartFeeding;

                    pControlMain->gpio0_state &= ~(pin03|pin04|pin05);
                    tp_writeport(pControlMain->gpio0, pControlMain->gpio0_state);
                    usleep(5000);

                    pControlMain->gpio0_state |= (pin04|pin05);
                    tp_writeport(pControlMain->gpio0, pControlMain->gpio0_state);
                    usleep(5000);
                    step = 4;
                }
            }
//            else{
//                pControlMain->putTabletMode(pControlMain->key_value);
//            }
        }

        pControlMain->gpio1_state &= ~(pin01|pin02|pin03);
        tp_writeport(pControlMain->gpio1, pControlMain->gpio1_state);
        switch(step){
            case 0:
                pControlMain->gpio1_state = led ? pControlMain->gpio1_state|pin01 : pControlMain->gpio1_state & ~(pin01|pin02|pin03);
                tp_writeport(pControlMain->gpio1, pControlMain->gpio1_state);
                break;
            case 1:
                pControlMain->gpio1_state = led ? pControlMain->gpio1_state|pin02 : pControlMain->gpio1_state & ~(pin01|pin02|pin03);
                tp_writeport(pControlMain->gpio1, pControlMain->gpio1_state);
                break;
            case 2:
                pControlMain->gpio1_state |= pin02;
                tp_writeport(pControlMain->gpio1, pControlMain->gpio1_state);
                break;
            case 3:
                pControlMain->gpio1_state = led ? pControlMain->gpio1_state|pin03 : pControlMain->gpio1_state & ~(pin01|pin02|pin03);
                tp_writeport(pControlMain->gpio1, pControlMain->gpio1_state);
                break;
            case 4:
                if(!pControlMain->dataControl->KITECHData.tablet_connect){
                    pControlMain->dataControl->KITECHData.tablet_check = true;
                    pControlMain->gpio0_state = led ? pControlMain->gpio0_state|0x30 : pControlMain->gpio0_state&(0xCF);
                    tp_writeport(pControlMain->gpio0, pControlMain->gpio0_state);
                }
                else{
                    pControlMain->gpio0_state |= 0x30;
                    tp_writeport(pControlMain->gpio0, pControlMain->gpio0_state);
                    pControlMain->dataControl->KITECHData.tablet_check = false;
                }
                break;
            default:
                break;
        }

        pControlMain->key_value = 0;
        pControlMain->gpio0_state_old = pControlMain->gpio0_state;
    }
}

void ControlMain::key_input_RT(void *arg)
{
    ControlMain *pControlMain = static_cast<ControlMain*>(arg);

    rt_task_set_periodic(NULL, TM_NOW, 10e6);

    pControlMain->key_input_task_run = true;
    pControlMain->dataControl->KITECHData.tablet_connect = false;

    while(pControlMain->key_input_task_run){
        rt_task_wait_period(NULL);
        pControlMain->key_value = getch();
        rt_printf("key value : %d\n", pControlMain->key_value);

//        if(pControlMain->key_value == KEY_A){
//            pControlMain->dataControl->KITECHData.camera_request = true;
//        }
//        else if(pControlMain->key_value == KEY_S){
//            pControlMain->dataControl->KITECHData.camera_request = false;
//        }
    }
    rt_printf("Finished Key Input Thread\n");
}

void ControlMain::setTabletMode()
{
    dataControl->tablet_mode = true;
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

    while(!this->robot_task_run){
        usleep(1000);
    }
    rt_printf("Robot RT thread run\n");

    dataControl->ClientToServer.opMode = DataControl::OpMode::ServoOnOff;
    dataControl->ClientToServer.subMode = 1;
    while(!(dataControl->ClientToServer.opMode == DataControl::OpMode::Wait)){
        usleep(1000);
    }
    rt_printf("Servo on\n");

    dataControl->ClientToServer.opMode = DataControl::OpMode::ServoOnOff;
    dataControl->ClientToServer.subMode = 1;
    while(!(dataControl->ClientToServer.opMode == DataControl::OpMode::Wait)){
        usleep(1000);
    }
    rt_printf("Servo on\n");

    dataControl->ClientToServer.opMode = DataControl::OpMode::ServoOnOff;
    dataControl->ClientToServer.subMode = 1;
    while(!(dataControl->ClientToServer.opMode == DataControl::OpMode::Wait)){
        usleep(1000);
    }
    rt_printf("Servo on\n");

    dataControl->ClientToServer.opMode = DataControl::OpMode::OperateMode;
    dataControl->operateMode.mode = DataControl::Operate::Start;

    usleep(10000);
    rt_printf("Feeding start\n");

    dataControl->section_indx = -1;
    dataControl->section_indx2 = 0;
}

void ControlMain::unsetTabletMode()
{
    dataControl->ClientToServer.opMode = DataControl::OpMode::ServoOnOff;
    dataControl->ClientToServer.subMode = 0;
    while(!(dataControl->ClientToServer.opMode == DataControl::OpMode::Wait)){
        usleep(1000);
    }
    rt_printf("Servo off\n");

    dataControl->tablet_mode = false;
    dataControl->config_check = false;

    if(init_thread_run){
        init_thread_run = false;
        pthread_cancel(init_thread);
        printf("Finished Module Init Thread\n");
        usleep(10000);
    }

    robot_rt_stop();
    usleep(10000);

//    if (dataControl->RobotData.module_init){
//        for(uint8_t i = 0; i < NUM_JOINT; i++){
//            module->dxl_deinit(NUM_JOINT == 1 ? module->single_id : i);
//        }
//    }
    dataControl->RobotData.module_init = false;

    printf("Finished\n");
}
#if 0
char ControlMain::putTabletMode(char key)
{
    if(key == 38 || key == 'a'){
        for(uint i = 0; i < NUM_JOINT; i++){
            module->feeding_profile_acc[i] = 100;//50;
            module->feeding_profile_vel[i] = 1000;//static_cast<uint32_t>((500/64.0)*module->feeding_profile_acc[i]);
            module->feeding_pos_p_gain[i] = default_pos_p_gain[i];
        }
        module->setGroupSyncWriteIndirectAddress(module->feeding_profile_acc, module->feeding_profile_vel, module->feeding_pos_p_gain, NUM_JOINT);
        if(dataControl->ClientToServer.opMode == DataControl::OpMode::Wait){
            dataControl->ClientToServer.opMode = DataControl::OpMode::OperateMode;
            dataControl->operateMode.mode = DataControl::Operate::ReadyFeeding;

            dataControl->section_indx++;
            if(dataControl->section_indx >= 5){
                dataControl->section_indx = 0;
            }
        }
    }
    else if(key == 39 || key == 's'){
        if(dataControl->ClientToServer.opMode == DataControl::OpMode::Wait/* && dataControl->section_indx2 >= 0*/){
//            for(uint i = 0; i < NUM_JOINT; i++){
//                module->feeding_profile_acc[i] = 1;//50;
//                module->feeding_profile_vel[i] = 3;//static_cast<uint32_t>((500/64.0)*module->feeding_profile_acc[i]);
//                module->feeding_pos_p_gain[i] = default_pos_p_gain[i];
//            }
//            module->setGroupSyncWriteIndirectAddress(module->feeding_profile_acc, module->feeding_profile_vel, module->feeding_pos_p_gain, NUM_JOINT);
//            usleep(100000);

            dataControl->ClientToServer.opMode = DataControl::OpMode::OperateMode;
            dataControl->operateMode.mode = DataControl::Operate::Feeding;

            switch(dataControl->section_indx){
                case 0: // side 1
                {
                    rt_printf("section1 count : %d, %d\n", dataControl->trayInfor.section1, dataControl->trayInfor.section1%2);
                    dataControl->side1_motion[dataControl->trayInfor.section1%2].file_data_indx = 500;
                    dataControl->operateMode.section = DataControl::Section::Side1;
                    break;
                }
                case 1: // side 2
                {
                    rt_printf("section2 count : %d, %d\n", dataControl->trayInfor.section2, dataControl->trayInfor.section2%2);
                    dataControl->side2_motion[dataControl->trayInfor.section2%2].file_data_indx = 500;
                    dataControl->operateMode.section = DataControl::Section::Side2;
                    break;
                }
                case 2: // side 3
                {
                    rt_printf("section3 count : %d, %d\n", dataControl->trayInfor.section3, dataControl->trayInfor.section3%2);
                    dataControl->side3_motion[dataControl->trayInfor.section3%2].file_data_indx = 500;
                    dataControl->operateMode.section = DataControl::Section::Side3;
                    break;
                }
                case 3: // soup
                {
                    rt_printf("section4 count : %d\n", dataControl->trayInfor.section4);
                    dataControl->soup_motion.file_data_indx = 500;
                    dataControl->operateMode.section = DataControl::Section::Soup;
                    break;
                }
                case 4: // rice
                {
                    rt_printf("section5 count : %d, %d\n", dataControl->trayInfor.section5, dataControl->trayInfor.section5%3);
//                    dataControl->rice_motion[dataControl->trayInfor.section5%9].file_data_indx = 500;
                    dataControl->operateMode.section = DataControl::Section::Rice;
                    break;
                }
            }
        }
    }
    else if(key == 40 || key == 'd'){
        for(uint i = 0; i < NUM_JOINT; i++){
            module->feeding_profile_acc[i] = 100;//50;
            module->feeding_profile_vel[i] = 1000;//static_cast<uint32_t>((500/64.0)*module->feeding_profile_acc[i]);
            module->feeding_pos_p_gain[i] = default_pos_p_gain[i];
        }
        module->setGroupSyncWriteIndirectAddress(module->feeding_profile_acc, module->feeding_profile_vel, module->feeding_pos_p_gain, NUM_JOINT);
        if(dataControl->ClientToServer.opMode == DataControl::OpMode::Wait){
            dataControl->ClientToServer.opMode = DataControl::OpMode::OperateMode;
            dataControl->operateMode.mode = DataControl::Operate::ReadyFeeding2;

            dataControl->section_indx2++;
            if(dataControl->section_indx == 0 || dataControl->section_indx == 1 || dataControl->section_indx == 2){
                if(dataControl->section_indx2 >= 4){
                    dataControl->section_indx2 = 0;
                }
            }
            else if(dataControl->section_indx == 4){
                if(dataControl->section_indx2 >= 9){
                    dataControl->section_indx2 = 0;
                }
            }
        }
    }
    else if(key == 53 || key == 'x' || key == 120){
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
//        else if(dataControl->operateMode.mode == DataControl::Operate::StopTeaching){
//            dataControl->ClientToServer.opMode = DataControl::OpMode::OperateMode;
//            dataControl->operateMode.mode = DataControl::Operate::StartFeeding;
//        }
    }
    else if(key == 54 || key == 'c' || key == 99){
        if(dataControl->operateMode.mode == DataControl::Operate::StopTeaching){
            dataControl->ClientToServer.opMode = DataControl::OpMode::OperateMode;
            dataControl->operateMode.mode = DataControl::Operate::StartFeeding;
            while(!(dataControl->ClientToServer.opMode == DataControl::OpMode::Wait)){
                usleep(1000);
            }
            dataControl->operateMode.mode = DataControl::Operate::Feeding;
            dataControl->operateMode.section = DataControl::Section::Rice;
        }
    }

    return key;
}
#endif

void* ControlMain::init_func(void* arg){
    ControlMain *pThis = static_cast<ControlMain*>(arg);

    while(!pThis->dataControl->config_check){
        usleep(10000);
    }
    pThis->module->init();

    while(pThis->init_thread_run){
        switch(pThis->dataControl->MODULE_TYPE){
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
                break;
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

void ControlMain::robot_rt_start()
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

void ControlMain::robot_rt_stop(){
    if (robot_task_run){
        controlMainCustom->robot_stop();
        robot_task_run = false;
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

                module->getGroupSyncReadPresentPosition(dataControl->RobotData.joint_offset, NUM_JOINT);
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

                    if(module_indx == 1){
                        if(pos < 3000){

                        }
                    }
                }
            }
            if (module_indx >= NUM_JOINT){
                dataControl->RobotData.module_init = true;

//                if (!dataControl->RobotData.offset_setting){
//                    module->getGroupSyncReadPresentPosition(dataControl->RobotData.offset, NUM_JOINT);
//                }
                memcpy(dataControl->RobotData.joint_offset, dataControl->joint_offset, sizeof(int32_t)*NUM_JOINT);

                dataControl->RobotData.joint_offset[3] = dataControl->joint_offset[3] - static_cast<int32_t>(-90*dataControl->DEG2ENC);
//                dataControl->RobotData.offset[5] -= static_cast<int32_t>(-90*dataControl->DEG2ENC);



            }
        }
    }

    if(dataControl->tablet_mode){
        for(uint i = 0; i < NUM_JOINT; i++){
            module->feeding_profile_acc[i] = 100;//50;
            module->feeding_profile_vel[i] = 2000;//static_cast<uint32_t>((500/64.0)*module->feeding_profile_acc[i]);
            module->feeding_pos_p_gain[i] = default_pos_p_gain[i];
        }
        module->setGroupSyncWriteIndirectAddress(module->feeding_profile_acc, module->feeding_profile_vel, module->feeding_pos_p_gain, NUM_JOINT);

        int err = 0;
        do{
            err = module->setTorqueEnable(1, 1);
        }while(err == 1);

        do{
            err = module->setTorqueEnable(2, 1);
        }while(err == 1);

        do{
            err = module->setTorqueEnable(3, 1);
        }while(err == 1);

        do{
            err = module->setTorqueEnable(4, 1);
        }while(err == 1);

        do{
            err = module->setTorqueEnable(5, 1);
        }while(err == 1);

        do{
            err = module->setTorqueEnable(0, 1);
        }while(err == 1);

        module->setGoalPosition(3, dataControl->initJoint4Deg*dataControl->DEG2ENC + dataControl->RobotData.joint_offset[3]);
        usleep(1500000);

        module->setGoalPosition(1, dataControl->initJoint2Deg*dataControl->DEG2ENC + dataControl->RobotData.joint_offset[1]);
        usleep(1500000);
    }

    if(!dataControl->tablet_mode){
        if(gpio_task_run){
            gpio_task_run = false;
            rt_task_join(&gpio_task);
            usleep(10000);
            rt_task_delete(&gpio_task);
            usleep(10000);
            printf("Finished GPIO Task\n");
        }

        if(key_input_task_run){
            key_input_task_run = false;
            rt_task_join(&key_input_task);
            usleep(10000);
            rt_task_delete(&key_input_task);
            usleep(10000);
            printf("Finished Key Input Task\n");
        }

        usleep(1000000);
        rt_printf("Operating start\n");

        tcpServer->sendKey("S");
    }
    robot_rt_start();
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
    double alpha = 1.5;
    for(uint i = 0; i < NUM_JOINT; i++){
        if(dataControl->MODULE_TYPE == DataControl::Module::FAR_V1){
            if (i < 3){
                goal_current[i] = 1000*(goal_torque[i] / TORQUE_CONSTANT_W270)*alpha;
            }
            else{
                goal_current[i] = 1000*(goal_torque[i] / TORQUE_CONSTANT_W150)*alpha;
            }
        }
        else if(dataControl->MODULE_TYPE == DataControl::Module::FAR_V2){
            if (i < 3){
                goal_current[i] = 1000*(goal_torque[i] / TORQUE_CONSTANT_W270)*alpha;
            }
            else{
                goal_current[i] = 1000*(goal_torque[i] / TORQUE_CONSTANT_W350)*alpha;
            }
        }
        else if(dataControl->MODULE_TYPE == DataControl::Module::FAR_V3){
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

    dataControl->jointCurrentRAW2Torque(dataControl->RobotData.present_joint_current, dataControl->RobotData.present_joint_torque);

    robotArm->disturbance_observer(dataControl->RobotData.present_q, dataControl->RobotData.present_q_dot, dataControl->RobotData.present_joint_torque, dataControl->RobotData.present_joint_residual);

    dataControl->RobotData.residual_limit_p[0] = 0.018963*1.1;
    dataControl->RobotData.residual_limit_p[1] = 0.103507*1.1;
    dataControl->RobotData.residual_limit_p[2] = 0.272331*1.1;
    dataControl->RobotData.residual_limit_p[3] = 0.03391*1.1;
    dataControl->RobotData.residual_limit_p[4] = 0.01152*1.1;
    dataControl->RobotData.residual_limit_p[5] = 0.0012*1.1;

    dataControl->RobotData.residual_limit_n[0] = -0.01746*1.1;
    dataControl->RobotData.residual_limit_n[1] = -0.04885*1.1;
    dataControl->RobotData.residual_limit_n[2] = -0.185205*1.1;
    dataControl->RobotData.residual_limit_n[3] = -0.01583*1.1;
    dataControl->RobotData.residual_limit_n[4] = -0.01386*1.1;
    dataControl->RobotData.residual_limit_n[5] = -0.00093*1.1;

//    for(int i = 0; i < 1; i++){
//        if(dataControl->RobotData.present_joint_residual[i] > dataControl->RobotData.residual_limit_p[i]){
//            dataControl->PathData.path_data_indx = 0;
//            dataControl->PathData.path_struct_indx = 0;
//            dataControl->ClientToServer.opMode = DataControl::OpMode::Wait;

//            rt_printf("Present Joint Residual : %f, %f, %f, %f, %f, %f\n",
//                      dataControl->RobotData.present_joint_residual[0], dataControl->RobotData.present_joint_residual[1], dataControl->RobotData.present_joint_residual[2],
//                    dataControl->RobotData.present_joint_residual[3], dataControl->RobotData.present_joint_residual[4], dataControl->RobotData.present_joint_residual[5]);
//        }
//        else if(dataControl->RobotData.present_joint_residual[i] < dataControl->RobotData.residual_limit_n[i]){
//            dataControl->PathData.path_data_indx = 0;
//            dataControl->PathData.path_struct_indx = 0;
//            dataControl->ClientToServer.opMode = DataControl::OpMode::Wait;

//            rt_printf("Present Joint Residual : %f, %f, %f, %f, %f, %f\n",
//                      dataControl->RobotData.present_joint_residual[0], dataControl->RobotData.present_joint_residual[1], dataControl->RobotData.present_joint_residual[2],
//                    dataControl->RobotData.present_joint_residual[3], dataControl->RobotData.present_joint_residual[4], dataControl->RobotData.present_joint_residual[5]);
//        }
//    }

//    rt_printf("Present Joint Torque : %f, %f, %f, %f, %f, %f\n",
//              dataControl->RobotData.present_joint_torque[0], dataControl->RobotData.present_joint_torque[1], dataControl->RobotData.present_joint_torque[2],
//            dataControl->RobotData.present_joint_torque[3], dataControl->RobotData.present_joint_torque[4], dataControl->RobotData.present_joint_torque[5]);
//    rt_printf("Present Joint Residual : %f, %f, %f, %f, %f, %f\n",
//              dataControl->RobotData.present_joint_residual[0], dataControl->RobotData.present_joint_residual[1], dataControl->RobotData.present_joint_residual[2],
//            dataControl->RobotData.present_joint_residual[3], dataControl->RobotData.present_joint_residual[4], dataControl->RobotData.present_joint_residual[5]);
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
    rt_printf("Desired pose : %3.5f, %3.5f, %3.5f, %3.5f, %3.5f, %3.5f\n",
        desCartesian[0], desCartesian[1], desCartesian[2], 
        desCartesian[3], desCartesian[4], desCartesian[5]);
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
        for(uint i = 0; i < NUM_JOINT; i++){
            module->feeding_profile_acc[i] = 1;//50;
            module->feeding_profile_vel[i] = 3;//static_cast<uint32_t>((500/64.0)*module->feeding_profile_acc[i]);
            module->feeding_pos_p_gain[i] = default_pos_p_gain[i];
        }
        module->setGroupSyncWriteIndirectAddress(module->feeding_profile_acc, module->feeding_profile_vel, module->feeding_pos_p_gain, NUM_JOINT);


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
        double time = 0;
        for(uint i = 0; i < dataControl->PathData.row; i++){
            dataControl->PathData.movePath[i].path_x.clear();
            dataControl->PathData.movePath[i].path_y.clear();
            dataControl->PathData.movePath[i].path_z.clear();
            dataControl->PathData.movePath[i].path_theta.clear();

            dataControl->PathData.total_time.push_back(time);
            dataControl->PathData.acc_time.push_back(dataControl->ClientToServer.acc_time);
            time += dataControl->ClientToServer.move_time;
        }

        dataControl->PathData.point_px.push_back(dataControl->RobotData.present_end_pose[0]);
        dataControl->PathData.point_py.push_back(dataControl->RobotData.present_end_pose[1]);
        dataControl->PathData.point_pz.push_back(dataControl->RobotData.present_end_pose[2]);
        dataControl->PathData.point_rx.push_back(dataControl->RobotData.present_end_pose[3]);
        dataControl->PathData.point_ry.push_back(dataControl->RobotData.present_end_pose[4]);
        dataControl->PathData.point_rz.push_back(dataControl->RobotData.present_end_pose[5]);

        dataControl->PathData.point_px.push_back(dataControl->RobotData.desired_end_pose[0]);
        dataControl->PathData.point_py.push_back(dataControl->RobotData.desired_end_pose[1]);
        dataControl->PathData.point_pz.push_back(dataControl->RobotData.desired_end_pose[2]);
        dataControl->PathData.point_rx.push_back(dataControl->RobotData.desired_end_pose[3]);
        dataControl->PathData.point_ry.push_back(dataControl->RobotData.desired_end_pose[4]);
        dataControl->PathData.point_rz.push_back(dataControl->RobotData.desired_end_pose[5]);

        rt_printf("path data : \n");
        for(int8_t i = 0; i < dataControl->PathData.row; i ++){
            rt_printf("%f, %f, %f, %f, %f, %f, %f, %f\n",
                      dataControl->PathData.total_time[i],
                      dataControl->PathData.point_px[i], dataControl->PathData.point_py[i], dataControl->PathData.point_pz[i],
                      dataControl->PathData.point_rx[i], dataControl->PathData.point_ry[i], dataControl->PathData.point_rz[i],
                      dataControl->PathData.acc_time[i]);
        }

        robotPathGenerate(dataControl->PathData.point_px, dataControl->PathData.point_py, dataControl->PathData.point_pz,
                          dataControl->PathData.point_rx, dataControl->PathData.point_ry, dataControl->PathData.point_rz);

        RobotArm::zyx2mat(dataControl->RobotData.present_end_pose[5], dataControl->RobotData.present_end_pose[4], dataControl->RobotData.present_end_pose[3], dataControl->PathData.movePath[0].R_init);

        dataControl->ClientToServer.opMode = DataControl::OpMode::RunMode;

        dataControl->RobotData.run_mode = DataControl::CmdType::RunCmd;
        dataControl->PathData.path_data_indx = 0;
        dataControl->PathData.path_struct_indx = 0;
        dataControl->PathData.cycle_count_max = 1;
        delay_cnt = 0;
        delay_cnt_max = 200;
    }
}

void ControlMain::robotPathGenerate(std::vector<double> px, std::vector<double> py, std::vector<double> pz, std::vector<double> rx, std::vector<double> ry, std::vector<double> rz)
{
    dataControl->PathData.point_theta.clear();

    for(uint8_t i = 0; i < dataControl->PathData.row - 1; i++){
        dataControl->PathData.movePath[i].path_x.clear();
        dataControl->PathData.movePath[i].path_y.clear();
        dataControl->PathData.movePath[i].path_z.clear();
        path_generator(px[i], px[i + 1], dataControl->PathData.total_time[i + 1] - dataControl->PathData.total_time[i], dataControl->PathData.acc_time[i], step_size, &dataControl->PathData.movePath[i].path_x, i);
        path_generator(py[i], py[i + 1], dataControl->PathData.total_time[i + 1] - dataControl->PathData.total_time[i], dataControl->PathData.acc_time[i], step_size, &dataControl->PathData.movePath[i].path_y, i);
        path_generator(pz[i], pz[i + 1], dataControl->PathData.total_time[i + 1] - dataControl->PathData.total_time[i], dataControl->PathData.acc_time[i], step_size, &dataControl->PathData.movePath[i].path_z, i);
    }

    dataControl->PathData.point_theta.push_back(0);
    for(uint8_t i = 0; i < dataControl->PathData.row - 1; i++){
        double R_init[9], R_final[9], r[3], theta;
        RobotArm::zyx2mat(rz[i], ry[i], rx[i], R_init);
        RobotArm::zyx2mat(rz[i + 1], ry[i + 1], rx[i + 1], R_final);
        RobotArm::mat_to_axis_angle(R_init, R_final, r, &theta);
        memcpy(dataControl->PathData.movePath[i].r, r, sizeof(double)*3);
        dataControl->PathData.point_theta.push_back(theta);

//        rt_printf("r : %f, %f, %f, %f\n", r[0], r[1], r[2], theta);
    }

    for(uint8_t i = 0; i < dataControl->PathData.row - 1; i++){
        path_generator(0, dataControl->PathData.point_theta[i + 1],
                dataControl->PathData.total_time[i + 1] - dataControl->PathData.total_time[i], dataControl->PathData.acc_time[i], step_size, &dataControl->PathData.movePath[i].path_theta, i);

        dataControl->PathData.movePath[i].data_size = dataControl->PathData.movePath[i].path_x.size();
        rt_printf("section %d path size : %d\n", i, dataControl->PathData.movePath[i].data_size);
    }

    // int indx = 1;
    // for (int i = 0; i < dataControl->PathData.row - 1; i++){
    //     for (uint j = 0; j < dataControl->PathData.movePath[i].path_x.size(); j++){
    //         printf("%d, %f, %f, %f\n",
    //                indx++, dataControl->PathData.movePath[i].path_x[j], dataControl->PathData.movePath[i].path_y[j], dataControl->PathData.movePath[i].path_z[j]);
    //     }
    // }

    // dataControl->ClientToServer.opMode = DataControl::OpMode::Wait;
}

void ControlMain::robotReady()
{
    for (uint i = 0; i < NUM_JOINT; i++)
    {
        module->feeding_profile_acc[i] = 1;  //50;
        module->feeding_profile_vel[i] = 3; //static_cast<uint32_t>((500/64.0)*module->feeding_profile_acc[i]);
        module->feeding_pos_p_gain[i] = default_pos_p_gain[i];
    }
    module->setGroupSyncWriteIndirectAddress(module->feeding_profile_acc, module->feeding_profile_vel, module->feeding_pos_p_gain, NUM_JOINT);

    path_generator(dataControl->RobotData.present_end_pose[0], dataControl->PathData.point_px[0], 2.0, 0.4, step_size, &dataControl->PathData.readyPath.path_x, 0);
    path_generator(dataControl->RobotData.present_end_pose[1], dataControl->PathData.point_py[0], 2.0, 0.4, step_size, &dataControl->PathData.readyPath.path_y, 0);
    path_generator(dataControl->RobotData.present_end_pose[2], dataControl->PathData.point_pz[0], 2.0, 0.4, step_size, &dataControl->PathData.readyPath.path_z, 0);

    double R_init[9], R_final[9], r[3], theta;
    RobotArm::zyx2mat(dataControl->RobotData.present_end_pose[5], dataControl->RobotData.present_end_pose[4], dataControl->RobotData.present_end_pose[3], R_init);
    RobotArm::zyx2mat(dataControl->PathData.point_rz[0], dataControl->PathData.point_ry[0], dataControl->PathData.point_rx[0], R_final);
    RobotArm::mat_to_axis_angle(R_init, R_final, r, &theta);
    memcpy(dataControl->PathData.readyPath.r, r, sizeof(double)*3);

    path_generator(0, theta, 2.0, 0.4, step_size, &dataControl->PathData.readyPath.path_theta, 0);
    memcpy(dataControl->PathData.readyPath.R_init, robotArm->body[robotArm->num_body].Ae, sizeof(double)*9);

    dataControl->RobotData.run_mode = DataControl::CmdType::ReadyCmd;

    dataControl->PathData.readyPath.data_size = dataControl->PathData.readyPath.path_x.size();
    rt_printf("ready path size : %d\n", dataControl->PathData.readyPath.data_size);
    dataControl->ClientToServer.opMode = DataControl::OpMode::RunMode;

    dataControl->PathData.path_data_indx = 0;
    delay_cnt = 0;

    // double desired_pose[6] = {0,};
    // desired_pose[0] = dataControl->PathData.point_px[0];
    // desired_pose[1] = dataControl->PathData.point_py[0];
    // desired_pose[2] = dataControl->PathData.point_pz[0];
    // desired_pose[3] = dataControl->PathData.point_rx[0];
    // desired_pose[4] = dataControl->PathData.point_ry[0];
    // desired_pose[5] = dataControl->PathData.point_rz[0];
    // dataControl->ClientToServer.move_time = 1;
    // dataControl->ClientToServer.acc_time = 0.3;

    // dataControl->ClientToServer.opMode = DataControl::Motion::CartesianMotion;

    // robotCartesianMove(dataControl->ClientToServer.opMode, desired_pose);
}

void ControlMain::robotRun()
{
    switch(dataControl->RobotData.run_mode){
        case DataControl::CmdType::ReadyCmd:
        {
#if PRINT_ON
            rt_printf("path_data_indx : %d\n", dataControl->PathData.path_data_indx);
#endif

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

                robotPathGenerate(dataControl->PathData.point_px, dataControl->PathData.point_py, dataControl->PathData.point_pz,
                    dataControl->PathData.point_rx, dataControl->PathData.point_ry, dataControl->PathData.point_rz);
            }

            break;
        }
        case DataControl::CmdType::RunCmd:
        {
            // rt_printf("%d\n", dataControl->PathData.path_data_indx);
            // rt_printf("%d\n", dataControl->PathData.path_struct_indx);
            // rt_printf("%d\n", dataControl->PathData.movePath[dataControl->PathData.path_struct_indx].path_x.size());
            // rt_printf("%d\n", dataControl->PathData.movePath[dataControl->PathData.path_struct_indx].path_y.size());
            // rt_printf("%d\n", dataControl->PathData.movePath[dataControl->PathData.path_struct_indx].path_z.size());
            dataControl->RobotData.desired_end_pose[0] = dataControl->PathData.movePath[dataControl->PathData.path_struct_indx].path_x[dataControl->PathData.path_data_indx];
            dataControl->RobotData.desired_end_pose[1] = dataControl->PathData.movePath[dataControl->PathData.path_struct_indx].path_y[dataControl->PathData.path_data_indx];
            dataControl->RobotData.desired_end_pose[2] = dataControl->PathData.movePath[dataControl->PathData.path_struct_indx].path_z[dataControl->PathData.path_data_indx];

            double Ri[9], Rd[9];
            RobotArm::axis_angle_to_mat(dataControl->PathData.movePath[dataControl->PathData.path_struct_indx].r,
                    dataControl->PathData.movePath[dataControl->PathData.path_struct_indx].path_theta[dataControl->PathData.path_data_indx], Ri);
            RobotArm::mat(dataControl->PathData.movePath[dataControl->PathData.path_struct_indx].R_init, Ri, 3, 3, 3, 3, Rd);
            RobotArm::mat2rpy(Rd, dataControl->RobotData.desired_end_pose + 3);

//            rt_printf("Present Position : %d, %d, %d, %d, %d, %d\n",
//                      dataControl->RobotData.present_joint_position[0], dataControl->RobotData.present_joint_position[1],
//                    dataControl->RobotData.present_joint_position[2], dataControl->RobotData.present_joint_position[3],
//                    dataControl->RobotData.present_joint_position[4], dataControl->RobotData.present_joint_position[5]);

//            rt_printf("Command Position : %d, %d, %d, %d, %d, %d\n",
//                      dataControl->RobotData.command_joint_position[0], dataControl->RobotData.command_joint_position[1],
//                    dataControl->RobotData.command_joint_position[2], dataControl->RobotData.command_joint_position[3],
//                    dataControl->RobotData.command_joint_position[4], dataControl->RobotData.command_joint_position[5]);

//            rt_printf("Desired Pose : %f, %f, %f, %f, %f, %f\n",
//                      dataControl->RobotData.desired_end_pose[0], dataControl->RobotData.desired_end_pose[1],
//                    dataControl->RobotData.desired_end_pose[2], dataControl->RobotData.desired_end_pose[3],
//                    dataControl->RobotData.desired_end_pose[4], dataControl->RobotData.desired_end_pose[5]);

//            rt_printf("Present Pose : %f, %f, %f, %f, %f, %f\n",
//                      dataControl->RobotData.present_end_pose[0], dataControl->RobotData.present_end_pose[1],
//                    dataControl->RobotData.present_end_pose[2], dataControl->RobotData.present_end_pose[3],
//                    dataControl->RobotData.present_end_pose[4], dataControl->RobotData.present_end_pose[5]);

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
#if PRINT_ON
                rt_printf("path_data_indx : %d\n", dataControl->PathData.path_data_indx);
                rt_printf("path_indx : %d\n", dataControl->PathData.path_struct_indx);
#endif

                goalReach(dataControl->RobotData.desired_end_pose, dataControl->RobotData.present_end_pose, &dataControl->cartesian_goal_reach);

                if (dataControl->cartesian_goal_reach){
                    dataControl->PathData.path_data_indx += 1;
                }
            }

            if (dataControl->PathData.path_data_indx >= dataControl->PathData.movePath[dataControl->PathData.path_struct_indx].data_size - 1){
                if(dataControl->operateMode.section != DataControl::Section::Home){
                    if (delay_cnt >= delay_cnt_max){
                        dataControl->PathData.path_data_indx = 0;
                        dataControl->PathData.path_struct_indx++;
                        memcpy(dataControl->PathData.movePath[dataControl->PathData.path_struct_indx].R_init, Rd, sizeof(double)*9);
                        delay_cnt = 0;
                    }
                    else{
                        delay_cnt++;
#if PRINT_ON
                        rt_printf("delay_cnt : %d\n", delay_cnt);
#endif
                    }
                }
                else{
                    dataControl->PathData.path_data_indx = 0;
                    dataControl->PathData.path_struct_indx++;
                    memcpy(dataControl->PathData.movePath[dataControl->PathData.path_struct_indx].R_init, Rd, sizeof(double)*9);
                    delay_cnt = 0;
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
                        if(dataControl->feeding){
                            if(dataControl->KITECHData.interface_cmd == CMD_SECTION && dataControl->KITECHData.interface_sub == 1){
                                dataControl->ClientToServer.opMode = DataControl::OpMode::Wait;
                            }
                            else{
                                switch(dataControl->operateMode.section){
                                    case DataControl::Section::Side1:
                                    case DataControl::Section::Side2:
                                    case DataControl::Section::Side3:
                                    {
                                        for(uint i = 0; i < NUM_JOINT; i++){
                                            module->feeding_profile_acc[i] = 100;//50;
                                            module->feeding_profile_vel[i] = 2000;//static_cast<uint32_t>((500/64.0)*module->feeding_profile_acc[i]);
                                            module->feeding_pos_p_gain[i] = default_pos_p_gain[i];
                                        }
                                        module->setGroupSyncWriteIndirectAddress(module->feeding_profile_acc, module->feeding_profile_vel, module->feeding_pos_p_gain, NUM_JOINT);

//                                        memcpy(dataControl->RobotData.desired_q, dataControl->operateCameraReadyJoint, sizeof(double)*NUM_JOINT);
                                        for(int i = 0; i < NUM_JOINT; i++){
                                            dataControl->RobotData.desired_q[i] = dataControl->operateFeedingForkEndJoint[i]*dataControl->DEG2RAD;
                                        }
                                        dataControl->jointPositionRAD2ENC(dataControl->RobotData.desired_q, dataControl->RobotData.command_joint_position);

                                        module->setGroupSyncWriteGoalPosition(dataControl->RobotData.command_joint_position, NUM_JOINT);
                                        usleep(module->feeding_profile_vel[0]*1000);

                                        dataControl->fork_flag = true;

                                        dataControl->operateMode.section = DataControl::Section::Mouse;
                                        dataControl->operateMode.mode = DataControl::Operate::Feeding;
                                        dataControl->ClientToServer.opMode = DataControl::OpMode::OperateMode;
                                        break;
                                    }
                                    case DataControl::Section::Rice:
                                    case DataControl::Section::Soup:
                                        dataControl->operateMode.section = DataControl::Section::Mouse;
                                        dataControl->operateMode.mode = DataControl::Operate::Feeding;
                                        dataControl->ClientToServer.opMode = DataControl::OpMode::OperateMode;
                                        break;
                                    case DataControl::Section::Mouse:
                                        if(dataControl->fork_flag)
                                        {
                                            for(uint i = 0; i < NUM_JOINT; i++){
                                                module->feeding_profile_acc[i] = 100;//50;
                                                module->feeding_profile_vel[i] = 2000;//static_cast<uint32_t>((500/64.0)*module->feeding_profile_acc[i]);
                                                module->feeding_pos_p_gain[i] = default_pos_p_gain[i];
                                            }
                                            module->setGroupSyncWriteIndirectAddress(module->feeding_profile_acc, module->feeding_profile_vel, module->feeding_pos_p_gain, NUM_JOINT);

                                            memcpy(dataControl->RobotData.command_joint_position, dataControl->PathData.fork_joint, sizeof(int32_t)*NUM_JOINT);
                                            module->setGroupSyncWriteGoalPosition(dataControl->RobotData.command_joint_position, NUM_JOINT);
                                            usleep(module->feeding_profile_vel[0]*1000);

                                            usleep(3000000);

                                            memcpy(dataControl->RobotData.command_joint_position, dataControl->PathData.teaching_joint, sizeof(int32_t)*NUM_JOINT);
                                            module->setGroupSyncWriteGoalPosition(dataControl->RobotData.command_joint_position, NUM_JOINT);
                                            usleep(module->feeding_profile_vel[0]*1000);

                                            dataControl->fork_flag = false;
                                        }
                                        else{
                                            usleep(3000000);
                                        }

                                        dataControl->operateMode.section = DataControl::Section::Home;
                                        dataControl->operateMode.mode = DataControl::Operate::Feeding;
                                        dataControl->ClientToServer.opMode = DataControl::OpMode::OperateMode;
                                        break;
                                    case DataControl::Section::Home:
                                        dataControl->ClientToServer.opMode = DataControl::OpMode::OperateMode;
                                        dataControl->operateMode.mode = DataControl::Operate::ReadyFeeding;
                                        break;
                                }
                            }
                        }
                        else{
                            dataControl->ClientToServer.opMode = DataControl::OpMode::Wait;
                        }

                        dataControl->PathData.path_data_indx = 0;
                        dataControl->PathData.path_struct_indx = 0;

//                        if(dataControl->feeding && dataControl->operateMode.section == DataControl::Section::Mouse){

//                            dataControl->operateMode.section = DataControl::Section::Home;
//                            dataControl->operateMode.mode = DataControl::Operate::Feeding;
//                            dataControl->ClientToServer.opMode = DataControl::OpMode::OperateMode;
//                        }
//                        else{
//                            rt_printf("%d\n", dataControl->feeding);
//                            dataControl->PathData.path_data_indx = 0;
//                            dataControl->PathData.path_struct_indx = 0;

//                            if(dataControl->operateMode.section == DataControl::Section::Home && dataControl->tablet_mode){
//                                dataControl->ClientToServer.opMode = DataControl::OpMode::OperateMode;
//                                dataControl->operateMode.mode = DataControl::Operate::ReadyFeeding;
//                            }
//                            else{
//                                dataControl->feeding = false;
//                                dataControl->ClientToServer.opMode = DataControl::OpMode::Wait;
//                            }
//                        }
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
                module->feeding_profile_vel[i] = 2000;//static_cast<uint32_t>((500/64.0)*module->feeding_profile_acc[i]);
                module->feeding_pos_p_gain[i] = default_pos_p_gain[i];
            }
            module->setGroupSyncWriteIndirectAddress(module->feeding_profile_acc, module->feeding_profile_vel, module->feeding_pos_p_gain, NUM_JOINT);

            memcpy(dataControl->RobotData.desired_q, dataControl->operateCameraReadyJoint, sizeof(double)*NUM_JOINT);
            dataControl->jointPositionRAD2ENC(dataControl->RobotData.desired_q, dataControl->RobotData.command_joint_position);

            module->setGroupSyncWriteGoalPosition(dataControl->RobotData.command_joint_position, NUM_JOINT);
            usleep(module->feeding_profile_vel[0]*1000);
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
            dataControl->ClientToServer.opMode = DataControl::OpMode::Wait;
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

            memcpy(dataControl->PathData.teaching_joint, dataControl->RobotData.present_joint_position, sizeof(int32_t)*NUM_JOINT);
            memcpy(dataControl->PathData.fork_joint, dataControl->RobotData.present_joint_position, sizeof(int32_t)*NUM_JOINT);
            dataControl->PathData.fork_joint[5] += static_cast<int32_t>(180*dataControl->DEG2ENC);

//            rt_printf("%d, %d, %d, %d, %d, %d\n",
//                      dataControl->PathData.fork_joint[0], dataControl->PathData.fork_joint[1], dataControl->PathData.fork_joint[2],
//                    dataControl->PathData.fork_joint[3], dataControl->PathData.fork_joint[4], dataControl->PathData.fork_joint[5]);

//            robotPathGenerate(dataControl->PathData.point_px, dataControl->PathData.point_py, dataControl->PathData.point_pz,
//                              dataControl->PathData.point_rx, dataControl->PathData.point_ry, dataControl->PathData.point_rz);
            dataControl->ClientToServer.opMode = DataControl::OpMode::Wait;
            break;
        }
        case DataControl::Operate::StartFeeding:
        {
            for(uint i = 0; i < NUM_JOINT; i++){
                module->feeding_profile_acc[i] = 100;//50;
                module->feeding_profile_vel[i] = 2000;//static_cast<uint32_t>((500/64.0)*module->feeding_profile_acc[i]);
                module->feeding_pos_p_gain[i] = default_pos_p_gain[i];
            }
            module->setGroupSyncWriteIndirectAddress(module->feeding_profile_acc, module->feeding_profile_vel, module->feeding_pos_p_gain, NUM_JOINT);

            memcpy(dataControl->RobotData.desired_q, dataControl->operateCameraReadyJoint, sizeof(double)*NUM_JOINT);
            dataControl->jointPositionRAD2ENC(dataControl->RobotData.desired_q, dataControl->RobotData.command_joint_position);

            module->setGroupSyncWriteGoalPosition(dataControl->RobotData.command_joint_position, NUM_JOINT);
            usleep(module->feeding_profile_vel[0]*1000);

//            dataControl->KITECHData.camera_request = true;
//            usleep(3000000);
//            dataControl->KITECHData.camera_request = false;

            dataControl->ClientToServer.opMode = DataControl::OpMode::Wait;

//            dataControl->operateMode.mode = DataControl::Operate::Feeding;
//            dataControl->operateMode.section = DataControl::Section::Rice;

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
            dataControl->feeding = true;

            dataControl->PathData.total_time.clear();
            dataControl->PathData.point_px.clear();
            dataControl->PathData.point_py.clear();
            dataControl->PathData.point_pz.clear();
            dataControl->PathData.point_rx.clear();
            dataControl->PathData.point_ry.clear();
            dataControl->PathData.point_rz.clear();
            dataControl->PathData.point_theta.clear();
            dataControl->PathData.acc_time.clear();

            switch(dataControl->operateMode.section){
                case DataControl::Section::Side1:
                {
                    unsigned int s_count = 0, wp_row = 0, wp_col = 0;

                    for(uint i = 0; i < NUM_JOINT; i++){
                        module->feeding_profile_acc[i] = 100;//50;
                        module->feeding_profile_vel[i] = 2000;//static_cast<uint32_t>((500/64.0)*module->feeding_profile_acc[i]);
                        module->feeding_pos_p_gain[i] = default_pos_p_gain[i];
                    }
                    module->setGroupSyncWriteIndirectAddress(module->feeding_profile_acc, module->feeding_profile_vel, module->feeding_pos_p_gain, NUM_JOINT);
                    for(unsigned int i = 0; i < NUM_JOINT; i++){
                        dataControl->RobotData.desired_q[i] = dataControl->readyJoints.wp[(dataControl->operateMode.section-1)*6 + i]*dataControl->DEG2RAD;
                    }

                    dataControl->jointPositionRAD2ENC(dataControl->RobotData.desired_q, dataControl->RobotData.command_joint_position);
                    module->setGroupSyncWriteGoalPosition(dataControl->RobotData.command_joint_position, NUM_JOINT);
                    usleep(module->feeding_profile_vel[0]*1000);

                    for(uint i = 0; i < NUM_JOINT; i++){
                        module->feeding_profile_acc[i] = 1;//50;
                        module->feeding_profile_vel[i] = 3;//static_cast<uint32_t>((500/64.0)*module->feeding_profile_acc[i]);
                        module->feeding_pos_p_gain[i] = default_pos_p_gain[i];
                    }
                    module->setGroupSyncWriteIndirectAddress(module->feeding_profile_acc, module->feeding_profile_vel, module->feeding_pos_p_gain, NUM_JOINT);

                    memcpy(dataControl->RobotData.present_joint_position, dataControl->RobotData.command_joint_position, sizeof(int32_t)*NUM_JOINT);
                    robotKinematics();

                    s_count = dataControl->trayInfor.section1%2;
                    wp_row = dataControl->wp_side1[s_count].size[0];
                    wp_col = dataControl->wp_side1[s_count].size[1];
                    rt_printf("s_count : %d\n", s_count);
                    rt_printf("wp_row : %d\n", wp_row);
                    rt_printf("wp_col : %d\n", wp_col);

                    dataControl->PathData.row = static_cast<uint8_t>(wp_row + 1);
                    for(uint i = 0; i < dataControl->PathData.row; i++){
                        dataControl->PathData.movePath[i].path_x.clear();
                        dataControl->PathData.movePath[i].path_y.clear();
                        dataControl->PathData.movePath[i].path_z.clear();
                        dataControl->PathData.movePath[i].path_theta.clear();
                    }

                    dataControl->PathData.point_px.push_back(dataControl->RobotData.present_end_pose[0]);
                    dataControl->PathData.point_py.push_back(dataControl->RobotData.present_end_pose[1]);
                    dataControl->PathData.point_pz.push_back(dataControl->RobotData.present_end_pose[2]);
                    dataControl->PathData.point_rx.push_back(dataControl->RobotData.present_end_pose[3]);
                    dataControl->PathData.point_ry.push_back(dataControl->RobotData.present_end_pose[4]);
                    dataControl->PathData.point_rz.push_back(dataControl->RobotData.present_end_pose[5]);
                    dataControl->PathData.total_time.push_back(0);
                    dataControl->PathData.acc_time.push_back(0.3);

                    for(uint i = 1; i < dataControl->PathData.row; i++){
                        dataControl->PathData.total_time.push_back(dataControl->wp_side1[s_count].wp[(i-1)*wp_col + 0] + 1.0);
                        dataControl->PathData.point_px.push_back(dataControl->wp_side1[s_count].wp[(i-1)*wp_col + 1]*0.001);
                        dataControl->PathData.point_py.push_back(dataControl->wp_side1[s_count].wp[(i-1)*wp_col + 2]*0.001);
                        dataControl->PathData.point_pz.push_back(dataControl->wp_side1[s_count].wp[(i-1)*wp_col + 3]*0.001);
                        dataControl->PathData.point_rx.push_back(dataControl->wp_side1[s_count].wp[(i-1)*wp_col + 4]*dataControl->DEG2RAD);
                        dataControl->PathData.point_ry.push_back(dataControl->wp_side1[s_count].wp[(i-1)*wp_col + 5]*dataControl->DEG2RAD);
                        dataControl->PathData.point_rz.push_back(dataControl->wp_side1[s_count].wp[(i-1)*wp_col + 6]*dataControl->DEG2RAD);
                        dataControl->PathData.acc_time.push_back(dataControl->wp_side1[s_count].wp[(i-1)*wp_col + 7]);
                    }

                    rt_printf("side1 path data : \n");
                    for(int8_t i = 0; i < dataControl->PathData.row; i ++){
                        rt_printf("%f, %f, %f, %f, %f, %f, %f, %f\n",
                                  dataControl->PathData.total_time[i],
                                  dataControl->PathData.point_px[i], dataControl->PathData.point_py[i], dataControl->PathData.point_pz[i],
                                  dataControl->PathData.point_rx[i], dataControl->PathData.point_ry[i], dataControl->PathData.point_rz[i],
                                  dataControl->PathData.acc_time[i]);
                    }

                    robotPathGenerate(dataControl->PathData.point_px, dataControl->PathData.point_py, dataControl->PathData.point_pz,
                                      dataControl->PathData.point_rx, dataControl->PathData.point_ry, dataControl->PathData.point_rz);

                    RobotArm::zyx2mat(dataControl->RobotData.present_end_pose[5], dataControl->RobotData.present_end_pose[4], dataControl->RobotData.present_end_pose[3], dataControl->PathData.movePath[0].R_init);

                    dataControl->ClientToServer.opMode = DataControl::OpMode::RunMode;

                    dataControl->RobotData.run_mode = DataControl::CmdType::RunCmd;
                    dataControl->PathData.path_data_indx = 0;
                    dataControl->PathData.path_struct_indx = 0;
                    dataControl->PathData.cycle_count_max = 1;
                    delay_cnt = 0;
                    delay_cnt_max = 100;
                    dataControl->trayInfor.section1++;

                    break;
                }
                case DataControl::Section::Side2:
                {
                    unsigned int s_count = 0, wp_row = 0, wp_col = 0;

                    for(uint i = 0; i < NUM_JOINT; i++){
                        module->feeding_profile_acc[i] = 100;//50;
                        module->feeding_profile_vel[i] = 2000;//static_cast<uint32_t>((500/64.0)*module->feeding_profile_acc[i]);
                        module->feeding_pos_p_gain[i] = default_pos_p_gain[i];
                    }
                    module->setGroupSyncWriteIndirectAddress(module->feeding_profile_acc, module->feeding_profile_vel, module->feeding_pos_p_gain, NUM_JOINT);
                    for(unsigned int i = 0; i < NUM_JOINT; i++){
                        dataControl->RobotData.desired_q[i] = dataControl->readyJoints.wp[(dataControl->operateMode.section-1)*6 + i]*dataControl->DEG2RAD;
                    }

                    dataControl->jointPositionRAD2ENC(dataControl->RobotData.desired_q, dataControl->RobotData.command_joint_position);
                    module->setGroupSyncWriteGoalPosition(dataControl->RobotData.command_joint_position, NUM_JOINT);
                    usleep(module->feeding_profile_vel[0]*1000);

                    for(uint i = 0; i < NUM_JOINT; i++){
                        module->feeding_profile_acc[i] = 1;//50;
                        module->feeding_profile_vel[i] = 3;//static_cast<uint32_t>((500/64.0)*module->feeding_profile_acc[i]);
                        module->feeding_pos_p_gain[i] = default_pos_p_gain[i];
                    }
                    module->setGroupSyncWriteIndirectAddress(module->feeding_profile_acc, module->feeding_profile_vel, module->feeding_pos_p_gain, NUM_JOINT);

                    memcpy(dataControl->RobotData.present_joint_position, dataControl->RobotData.command_joint_position, sizeof(int32_t)*NUM_JOINT);
                    robotKinematics();

                    s_count = dataControl->trayInfor.section2%2;
                    wp_row = dataControl->wp_side2[s_count].size[0];
                    wp_col = dataControl->wp_side2[s_count].size[1];
                    rt_printf("s_count : %d\n", s_count);
                    rt_printf("wp_row : %d\n", wp_row);
                    rt_printf("wp_col : %d\n", wp_col);

                    dataControl->PathData.row = static_cast<uint8_t>(wp_row + 1);
                    for(uint i = 0; i < dataControl->PathData.row; i++){
                        dataControl->PathData.movePath[i].path_x.clear();
                        dataControl->PathData.movePath[i].path_y.clear();
                        dataControl->PathData.movePath[i].path_z.clear();
                        dataControl->PathData.movePath[i].path_theta.clear();
                    }

                    dataControl->PathData.point_px.push_back(dataControl->RobotData.present_end_pose[0]);
                    dataControl->PathData.point_py.push_back(dataControl->RobotData.present_end_pose[1]);
                    dataControl->PathData.point_pz.push_back(dataControl->RobotData.present_end_pose[2]);
                    dataControl->PathData.point_rx.push_back(dataControl->RobotData.present_end_pose[3]);
                    dataControl->PathData.point_ry.push_back(dataControl->RobotData.present_end_pose[4]);
                    dataControl->PathData.point_rz.push_back(dataControl->RobotData.present_end_pose[5]);
                    dataControl->PathData.total_time.push_back(0);
                    dataControl->PathData.acc_time.push_back(0.3);

                    for(uint i = 1; i < dataControl->PathData.row; i++){
                        dataControl->PathData.total_time.push_back(dataControl->wp_side2[s_count].wp[(i-1)*wp_col + 0] + 1.0);
                        dataControl->PathData.point_px.push_back(dataControl->wp_side2[s_count].wp[(i-1)*wp_col + 1]*0.001);
                        dataControl->PathData.point_py.push_back(dataControl->wp_side2[s_count].wp[(i-1)*wp_col + 2]*0.001);
                        dataControl->PathData.point_pz.push_back(dataControl->wp_side2[s_count].wp[(i-1)*wp_col + 3]*0.001);
                        dataControl->PathData.point_rx.push_back(dataControl->wp_side2[s_count].wp[(i-1)*wp_col + 4]*dataControl->DEG2RAD);
                        dataControl->PathData.point_ry.push_back(dataControl->wp_side2[s_count].wp[(i-1)*wp_col + 5]*dataControl->DEG2RAD);
                        dataControl->PathData.point_rz.push_back(dataControl->wp_side2[s_count].wp[(i-1)*wp_col + 6]*dataControl->DEG2RAD);
                        dataControl->PathData.acc_time.push_back(dataControl->wp_side2[s_count].wp[(i-1)*wp_col + 7]);
                    }

                    rt_printf("side2 path data : \n");
                    for(int8_t i = 0; i < dataControl->PathData.row; i ++){
                        rt_printf("%f, %f, %f, %f, %f, %f, %f, %f\n",
                                  dataControl->PathData.total_time[i],
                                  dataControl->PathData.point_px[i], dataControl->PathData.point_py[i], dataControl->PathData.point_pz[i],
                                  dataControl->PathData.point_rx[i], dataControl->PathData.point_ry[i], dataControl->PathData.point_rz[i],
                                  dataControl->PathData.acc_time[i]);
                    }

                    robotPathGenerate(dataControl->PathData.point_px, dataControl->PathData.point_py, dataControl->PathData.point_pz,
                                      dataControl->PathData.point_rx, dataControl->PathData.point_ry, dataControl->PathData.point_rz);

                    RobotArm::zyx2mat(dataControl->RobotData.present_end_pose[5], dataControl->RobotData.present_end_pose[4], dataControl->RobotData.present_end_pose[3], dataControl->PathData.movePath[0].R_init);

                    dataControl->ClientToServer.opMode = DataControl::OpMode::RunMode;

                    dataControl->RobotData.run_mode = DataControl::CmdType::RunCmd;
                    dataControl->PathData.path_data_indx = 0;
                    dataControl->PathData.path_struct_indx = 0;
                    dataControl->PathData.cycle_count_max = 1;
                    delay_cnt = 0;
                    delay_cnt_max = 100;
                    dataControl->trayInfor.section2++;
                    break;
                }
                case DataControl::Section::Side3:
                {
                    unsigned int s_count = 0, wp_row = 0, wp_col = 0;

                    for(uint i = 0; i < NUM_JOINT; i++){
                        module->feeding_profile_acc[i] = 100;//50;
                        module->feeding_profile_vel[i] = 2000;//static_cast<uint32_t>((500/64.0)*module->feeding_profile_acc[i]);
                        module->feeding_pos_p_gain[i] = default_pos_p_gain[i];
                    }
                    module->setGroupSyncWriteIndirectAddress(module->feeding_profile_acc, module->feeding_profile_vel, module->feeding_pos_p_gain, NUM_JOINT);
                    for(unsigned int i = 0; i < NUM_JOINT; i++){
                        dataControl->RobotData.desired_q[i] = dataControl->readyJoints.wp[(dataControl->operateMode.section-1)*6 + i]*dataControl->DEG2RAD;
                    }

                    dataControl->jointPositionRAD2ENC(dataControl->RobotData.desired_q, dataControl->RobotData.command_joint_position);
                    module->setGroupSyncWriteGoalPosition(dataControl->RobotData.command_joint_position, NUM_JOINT);
                    usleep(module->feeding_profile_vel[0]*1000);

                    for(uint i = 0; i < NUM_JOINT; i++){
                        module->feeding_profile_acc[i] = 1;//50;
                        module->feeding_profile_vel[i] = 3;//static_cast<uint32_t>((500/64.0)*module->feeding_profile_acc[i]);
                        module->feeding_pos_p_gain[i] = default_pos_p_gain[i];
                    }
                    module->setGroupSyncWriteIndirectAddress(module->feeding_profile_acc, module->feeding_profile_vel, module->feeding_pos_p_gain, NUM_JOINT);

                    memcpy(dataControl->RobotData.present_joint_position, dataControl->RobotData.command_joint_position, sizeof(int32_t)*NUM_JOINT);
                    robotKinematics();

                    s_count = dataControl->trayInfor.section3%2;
                    wp_row = dataControl->wp_side3[s_count].size[0];
                    wp_col = dataControl->wp_side3[s_count].size[1];
                    rt_printf("s_count : %d\n", s_count);
                    rt_printf("wp_row : %d\n", wp_row);
                    rt_printf("wp_col : %d\n", wp_col);

                    dataControl->PathData.row = static_cast<uint8_t>(wp_row + 1);
                    for(uint i = 0; i < dataControl->PathData.row; i++){
                        dataControl->PathData.movePath[i].path_x.clear();
                        dataControl->PathData.movePath[i].path_y.clear();
                        dataControl->PathData.movePath[i].path_z.clear();
                        dataControl->PathData.movePath[i].path_theta.clear();
                    }

                    dataControl->PathData.point_px.push_back(dataControl->RobotData.present_end_pose[0]);
                    dataControl->PathData.point_py.push_back(dataControl->RobotData.present_end_pose[1]);
                    dataControl->PathData.point_pz.push_back(dataControl->RobotData.present_end_pose[2]);
                    dataControl->PathData.point_rx.push_back(dataControl->RobotData.present_end_pose[3]);
                    dataControl->PathData.point_ry.push_back(dataControl->RobotData.present_end_pose[4]);
                    dataControl->PathData.point_rz.push_back(dataControl->RobotData.present_end_pose[5]);
                    dataControl->PathData.total_time.push_back(0);
                    dataControl->PathData.acc_time.push_back(0.3);

                    for(uint i = 1; i < dataControl->PathData.row; i++){
                        dataControl->PathData.total_time.push_back(dataControl->wp_side3[s_count].wp[(i-1)*wp_col + 0] + 1.0);
                        dataControl->PathData.point_px.push_back(dataControl->wp_side3[s_count].wp[(i-1)*wp_col + 1]*0.001);
                        dataControl->PathData.point_py.push_back(dataControl->wp_side3[s_count].wp[(i-1)*wp_col + 2]*0.001);
                        dataControl->PathData.point_pz.push_back(dataControl->wp_side3[s_count].wp[(i-1)*wp_col + 3]*0.001);
                        dataControl->PathData.point_rx.push_back(dataControl->wp_side3[s_count].wp[(i-1)*wp_col + 4]*dataControl->DEG2RAD);
                        dataControl->PathData.point_ry.push_back(dataControl->wp_side3[s_count].wp[(i-1)*wp_col + 5]*dataControl->DEG2RAD);
                        dataControl->PathData.point_rz.push_back(dataControl->wp_side3[s_count].wp[(i-1)*wp_col + 6]*dataControl->DEG2RAD);
                        dataControl->PathData.acc_time.push_back(dataControl->wp_side3[s_count].wp[(i-1)*wp_col + 7]);
                    }

                    rt_printf("side3 path data : \n");
                    for(int8_t i = 0; i < dataControl->PathData.row; i ++){
                        rt_printf("%f, %f, %f, %f, %f, %f, %f, %f\n",
                                  dataControl->PathData.total_time[i],
                                  dataControl->PathData.point_px[i], dataControl->PathData.point_py[i], dataControl->PathData.point_pz[i],
                                  dataControl->PathData.point_rx[i], dataControl->PathData.point_ry[i], dataControl->PathData.point_rz[i],
                                  dataControl->PathData.acc_time[i]);
                    }

                    robotPathGenerate(dataControl->PathData.point_px, dataControl->PathData.point_py, dataControl->PathData.point_pz,
                                      dataControl->PathData.point_rx, dataControl->PathData.point_ry, dataControl->PathData.point_rz);

                    RobotArm::zyx2mat(dataControl->RobotData.present_end_pose[5], dataControl->RobotData.present_end_pose[4], dataControl->RobotData.present_end_pose[3], dataControl->PathData.movePath[0].R_init);

                    dataControl->ClientToServer.opMode = DataControl::OpMode::RunMode;

                    dataControl->RobotData.run_mode = DataControl::CmdType::RunCmd;
                    dataControl->PathData.path_data_indx = 0;
                    dataControl->PathData.path_struct_indx = 0;
                    dataControl->PathData.cycle_count_max = 1;
                    delay_cnt = 0;
                    delay_cnt_max = 100;
                    dataControl->trayInfor.section3++;
                    break;
                }
                case DataControl::Section::Soup:
                {
                    unsigned int s_count = 0, wp_row = 0, wp_col = 0;

                    for(uint i = 0; i < NUM_JOINT; i++){
                        module->feeding_profile_acc[i] = 100;//50;
                        module->feeding_profile_vel[i] = 2000;//static_cast<uint32_t>((500/64.0)*module->feeding_profile_acc[i]);
                        module->feeding_pos_p_gain[i] = default_pos_p_gain[i];
                    }
                    module->setGroupSyncWriteIndirectAddress(module->feeding_profile_acc, module->feeding_profile_vel, module->feeding_pos_p_gain, NUM_JOINT);
                    for(unsigned int i = 0; i < NUM_JOINT; i++){
                        dataControl->RobotData.desired_q[i] = dataControl->readyJoints.wp[(dataControl->operateMode.section-1)*6 + i]*dataControl->DEG2RAD;
                    }

                    dataControl->jointPositionRAD2ENC(dataControl->RobotData.desired_q, dataControl->RobotData.command_joint_position);
                    module->setGroupSyncWriteGoalPosition(dataControl->RobotData.command_joint_position, NUM_JOINT);
                    usleep(module->feeding_profile_vel[0]*1000);

                    for(uint i = 0; i < NUM_JOINT; i++){
                        module->feeding_profile_acc[i] = 1;//50;
                        module->feeding_profile_vel[i] = 3;//static_cast<uint32_t>((500/64.0)*module->feeding_profile_acc[i]);
                        module->feeding_pos_p_gain[i] = default_pos_p_gain[i];
                    }
                    module->setGroupSyncWriteIndirectAddress(module->feeding_profile_acc, module->feeding_profile_vel, module->feeding_pos_p_gain, NUM_JOINT);

                    memcpy(dataControl->RobotData.present_joint_position, dataControl->RobotData.command_joint_position, sizeof(int32_t)*NUM_JOINT);
                    robotKinematics();

                    s_count = dataControl->trayInfor.section4%1;
                    wp_row = dataControl->wp_soup[s_count].size[0];
                    wp_col = dataControl->wp_soup[s_count].size[1];
                    rt_printf("s_count : %d\n", s_count);
                    rt_printf("wp_row : %d\n", wp_row);
                    rt_printf("wp_col : %d\n", wp_col);

                    dataControl->PathData.row = static_cast<uint8_t>(wp_row + 1);
                    for(uint i = 0; i < dataControl->PathData.row; i++){
                        dataControl->PathData.movePath[i].path_x.clear();
                        dataControl->PathData.movePath[i].path_y.clear();
                        dataControl->PathData.movePath[i].path_z.clear();
                        dataControl->PathData.movePath[i].path_theta.clear();
                    }

                    dataControl->PathData.point_px.push_back(dataControl->RobotData.present_end_pose[0]);
                    dataControl->PathData.point_py.push_back(dataControl->RobotData.present_end_pose[1]);
                    dataControl->PathData.point_pz.push_back(dataControl->RobotData.present_end_pose[2]);
                    dataControl->PathData.point_rx.push_back(dataControl->RobotData.present_end_pose[3]);
                    dataControl->PathData.point_ry.push_back(dataControl->RobotData.present_end_pose[4]);
                    dataControl->PathData.point_rz.push_back(dataControl->RobotData.present_end_pose[5]);
                    dataControl->PathData.total_time.push_back(0);
                    dataControl->PathData.acc_time.push_back(0.3);

                    for(uint i = 1; i < dataControl->PathData.row; i++){
                        dataControl->PathData.total_time.push_back(dataControl->wp_soup[s_count].wp[(i-1)*wp_col + 0] + 1.0);
                        dataControl->PathData.point_px.push_back(dataControl->wp_soup[s_count].wp[(i-1)*wp_col + 1]*0.001);
                        dataControl->PathData.point_py.push_back(dataControl->wp_soup[s_count].wp[(i-1)*wp_col + 2]*0.001);
                        dataControl->PathData.point_pz.push_back(dataControl->wp_soup[s_count].wp[(i-1)*wp_col + 3]*0.001);
                        dataControl->PathData.point_rx.push_back(dataControl->wp_soup[s_count].wp[(i-1)*wp_col + 4]*dataControl->DEG2RAD);
                        dataControl->PathData.point_ry.push_back(dataControl->wp_soup[s_count].wp[(i-1)*wp_col + 5]*dataControl->DEG2RAD);
                        dataControl->PathData.point_rz.push_back(dataControl->wp_soup[s_count].wp[(i-1)*wp_col + 6]*dataControl->DEG2RAD);
                        dataControl->PathData.acc_time.push_back(dataControl->wp_soup[s_count].wp[(i-1)*wp_col + 7]);
                    }

                    rt_printf("soup path data : \n");
                    for(int8_t i = 0; i < dataControl->PathData.row; i ++){
                        rt_printf("%f, %f, %f, %f, %f, %f, %f, %f\n",
                                  dataControl->PathData.total_time[i],
                                  dataControl->PathData.point_px[i], dataControl->PathData.point_py[i], dataControl->PathData.point_pz[i],
                                  dataControl->PathData.point_rx[i], dataControl->PathData.point_ry[i], dataControl->PathData.point_rz[i],
                                  dataControl->PathData.acc_time[i]);
                    }

                    robotPathGenerate(dataControl->PathData.point_px, dataControl->PathData.point_py, dataControl->PathData.point_pz,
                                      dataControl->PathData.point_rx, dataControl->PathData.point_ry, dataControl->PathData.point_rz);

                    RobotArm::zyx2mat(dataControl->RobotData.present_end_pose[5], dataControl->RobotData.present_end_pose[4], dataControl->RobotData.present_end_pose[3], dataControl->PathData.movePath[0].R_init);

                    dataControl->ClientToServer.opMode = DataControl::OpMode::RunMode;

                    dataControl->RobotData.run_mode = DataControl::CmdType::RunCmd;
                    dataControl->PathData.path_data_indx = 0;
                    dataControl->PathData.path_struct_indx = 0;
                    dataControl->PathData.cycle_count_max = 1;
                    delay_cnt = 0;
                    delay_cnt_max = 100;
                    dataControl->trayInfor.section4++;
                    dataControl->fork_flag = false;
                    break;
                }
                case DataControl::Section::Rice:
                {
                    unsigned int s_count = 0, wp_row = 0, wp_col = 0;

                    for(uint i = 0; i < NUM_JOINT; i++){
                        module->feeding_profile_acc[i] = 100;//50;
                        module->feeding_profile_vel[i] = 2000;//static_cast<uint32_t>((500/64.0)*module->feeding_profile_acc[i]);
                        module->feeding_pos_p_gain[i] = default_pos_p_gain[i];
                    }
                    module->setGroupSyncWriteIndirectAddress(module->feeding_profile_acc, module->feeding_profile_vel, module->feeding_pos_p_gain, NUM_JOINT);
                    for(unsigned int i = 0; i < NUM_JOINT; i++){
                        dataControl->RobotData.desired_q[i] = dataControl->readyJoints.wp[(dataControl->operateMode.section-1)*6 + i]*dataControl->DEG2RAD;
                    }

                    dataControl->jointPositionRAD2ENC(dataControl->RobotData.desired_q, dataControl->RobotData.command_joint_position);
                    module->setGroupSyncWriteGoalPosition(dataControl->RobotData.command_joint_position, NUM_JOINT);
                    usleep(module->feeding_profile_vel[0]*1000);

                    for(uint i = 0; i < NUM_JOINT; i++){
                        module->feeding_profile_acc[i] = 1;//50;
                        module->feeding_profile_vel[i] = 3;//static_cast<uint32_t>((500/64.0)*module->feeding_profile_acc[i]);
                        module->feeding_pos_p_gain[i] = default_pos_p_gain[i];
                    }
                    module->setGroupSyncWriteIndirectAddress(module->feeding_profile_acc, module->feeding_profile_vel, module->feeding_pos_p_gain, NUM_JOINT);

                    memcpy(dataControl->RobotData.present_joint_position, dataControl->RobotData.command_joint_position, sizeof(int32_t)*NUM_JOINT);
                    robotKinematics();

                    s_count = dataControl->trayInfor.section5%9;
                    wp_row = dataControl->wp_rice[s_count].size[0];
                    wp_col = dataControl->wp_rice[s_count].size[1];
                    rt_printf("s_count : %d\n", s_count);
                    rt_printf("wp_row : %d\n", wp_row);
                    rt_printf("wp_col : %d\n", wp_col);

                    dataControl->PathData.row = static_cast<uint8_t>(wp_row + 1);
                    for(uint i = 0; i < dataControl->PathData.row; i++){
                        dataControl->PathData.movePath[i].path_x.clear();
                        dataControl->PathData.movePath[i].path_y.clear();
                        dataControl->PathData.movePath[i].path_z.clear();
                        dataControl->PathData.movePath[i].path_theta.clear();
                    }

                    dataControl->PathData.point_px.push_back(dataControl->RobotData.present_end_pose[0]);
                    dataControl->PathData.point_py.push_back(dataControl->RobotData.present_end_pose[1]);
                    dataControl->PathData.point_pz.push_back(dataControl->RobotData.present_end_pose[2]);
                    dataControl->PathData.point_rx.push_back(dataControl->RobotData.present_end_pose[3]);
                    dataControl->PathData.point_ry.push_back(dataControl->RobotData.present_end_pose[4]);
                    dataControl->PathData.point_rz.push_back(dataControl->RobotData.present_end_pose[5]);
                    dataControl->PathData.total_time.push_back(0);
                    dataControl->PathData.acc_time.push_back(0.3);

                    for(uint i = 1; i < dataControl->PathData.row; i++){
                        dataControl->PathData.total_time.push_back(dataControl->wp_rice[s_count].wp[(i-1)*wp_col + 0] + 1.0);
                        dataControl->PathData.point_px.push_back(dataControl->wp_rice[s_count].wp[(i-1)*wp_col + 1]*0.001);
                        dataControl->PathData.point_py.push_back(dataControl->wp_rice[s_count].wp[(i-1)*wp_col + 2]*0.001);
                        dataControl->PathData.point_pz.push_back(dataControl->wp_rice[s_count].wp[(i-1)*wp_col + 3]*0.001);
                        dataControl->PathData.point_rx.push_back(dataControl->wp_rice[s_count].wp[(i-1)*wp_col + 4]*dataControl->DEG2RAD);
                        dataControl->PathData.point_ry.push_back(dataControl->wp_rice[s_count].wp[(i-1)*wp_col + 5]*dataControl->DEG2RAD);
                        dataControl->PathData.point_rz.push_back(dataControl->wp_rice[s_count].wp[(i-1)*wp_col + 6]*dataControl->DEG2RAD);
                        dataControl->PathData.acc_time.push_back(dataControl->wp_rice[s_count].wp[(i-1)*wp_col + 7]);
                    }

//                    dataControl->KITECHData.sp_food[0] = ((double)dataControl->KITECHData.food_pos[2*(dataControl->operateMode.section - 1)])*0.001;
//                    dataControl->KITECHData.sp_food[1] = ((double)dataControl->KITECHData.food_pos[2*(dataControl->operateMode.section - 1) + 1])*0.001;
//                    if(dataControl->KITECHData.interface_cmd == CMD_SECTION && dataControl->KITECHData.interface_sub == 1){
//                        dataControl->KITECHData.sp_food[2] = -200;
//                    }
//                    else{
//                        dataControl->KITECHData.sp_food[2] = 0;
//                    }

//                    RobotArm::mat(dataControl->KITECHData.A_marker, dataControl->KITECHData.sp_food, 3,3,3, dataControl->KITECHData.sp);
//                    for(uint i = 0; i < 3; i++){
//                        dataControl->KITECHData.rp[i] = dataControl->KITECHData.r_marker[i] + dataControl->KITECHData.sp[i];
//                    }

//                    dataControl->PathData.point_px.push_back(dataControl->KITECHData.rp[0]);
//                    dataControl->PathData.point_py.push_back(dataControl->KITECHData.rp[1]);
//                    dataControl->PathData.point_pz.push_back(0);
//                    dataControl->PathData.point_rx.push_back(-1.530946);
//                    dataControl->PathData.point_ry.push_back(-0.009205);
//                    dataControl->PathData.point_rz.push_back(1.594819);

                    rt_printf("rice path data : \n");
                    for(int8_t i = 0; i < dataControl->PathData.row; i ++){
                        rt_printf("%f, %f, %f, %f, %f, %f, %f, %f\n",
                                  dataControl->PathData.total_time[i],
                                  dataControl->PathData.point_px[i], dataControl->PathData.point_py[i], dataControl->PathData.point_pz[i],
                                  dataControl->PathData.point_rx[i], dataControl->PathData.point_ry[i], dataControl->PathData.point_rz[i],
                                  dataControl->PathData.acc_time[i]);
                    }

                    robotPathGenerate(dataControl->PathData.point_px, dataControl->PathData.point_py, dataControl->PathData.point_pz, dataControl->PathData.point_rx, dataControl->PathData.point_ry, dataControl->PathData.point_rz);

                    RobotArm::zyx2mat(dataControl->RobotData.present_end_pose[5], dataControl->RobotData.present_end_pose[4], dataControl->RobotData.present_end_pose[3], dataControl->PathData.movePath[0].R_init);

                    dataControl->ClientToServer.opMode = DataControl::OpMode::RunMode;

                    dataControl->RobotData.run_mode = DataControl::CmdType::RunCmd;
                    dataControl->PathData.path_data_indx = 0;
                    dataControl->PathData.path_struct_indx = 0;
                    dataControl->PathData.cycle_count_max = 1;
                    delay_cnt = 0;
                    delay_cnt_max = 100;
                    dataControl->trayInfor.section5++;

                    break;
                }
                case DataControl::Section::Mouse:
                {
                    for(uint i = 0; i < NUM_JOINT; i++){
                        module->feeding_profile_acc[i] = 0;//50;
                        module->feeding_profile_vel[i] = 1;//static_cast<uint32_t>((500/64.0)*module->feeding_profile_acc[i]);
                        module->feeding_pos_p_gain[i] = default_pos_p_gain[i];
                    }
                    module->setGroupSyncWriteIndirectAddress(module->feeding_profile_acc, module->feeding_profile_vel, module->feeding_pos_p_gain, NUM_JOINT);

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
                    dataControl->PathData.total_time.push_back(3);

                    dataControl->PathData.acc_time.push_back(0.5);
                    dataControl->PathData.acc_time.push_back(0.5);

                    dataControl->PathData.point_px.push_back(dataControl->RobotData.present_end_pose[0]);
                    dataControl->PathData.point_py.push_back(dataControl->RobotData.present_end_pose[1]);
                    dataControl->PathData.point_pz.push_back(dataControl->RobotData.present_end_pose[2]);
                    dataControl->PathData.point_rx.push_back(dataControl->RobotData.present_end_pose[3]);
                    dataControl->PathData.point_ry.push_back(dataControl->RobotData.present_end_pose[4]);
                    dataControl->PathData.point_rz.push_back(dataControl->RobotData.present_end_pose[5]);

                    dataControl->PathData.point_px.push_back(dataControl->PathData.teaching_pose[0]);
                    dataControl->PathData.point_py.push_back(dataControl->PathData.teaching_pose[1]);
                    dataControl->PathData.point_pz.push_back(dataControl->PathData.teaching_pose[2]);
                    dataControl->PathData.point_rx.push_back(dataControl->PathData.teaching_pose[3]);
                    dataControl->PathData.point_ry.push_back(dataControl->PathData.teaching_pose[4]);
                    dataControl->PathData.point_rz.push_back(dataControl->PathData.teaching_pose[5]);

                    rt_printf("go mouse\n");
                    rt_printf("start pose : %f, %f, %f, %f, %f, %f\n",
                              dataControl->RobotData.present_end_pose[0], dataControl->RobotData.present_end_pose[1], dataControl->RobotData.present_end_pose[2],
                              dataControl->RobotData.present_end_pose[3], dataControl->RobotData.present_end_pose[4], dataControl->RobotData.present_end_pose[5]);

                    rt_printf("end pose : %f, %f, %f, %f, %f, %f\n",
                              dataControl->PathData.teaching_pose[0], dataControl->PathData.teaching_pose[1], dataControl->PathData.teaching_pose[2],
                              dataControl->PathData.teaching_pose[3], dataControl->PathData.teaching_pose[4], dataControl->PathData.teaching_pose[5]);

                    robotPathGenerate(dataControl->PathData.point_px, dataControl->PathData.point_py, dataControl->PathData.point_pz,
                                      dataControl->PathData.point_rx, dataControl->PathData.point_ry, dataControl->PathData.point_rz);

                    RobotArm::zyx2mat(dataControl->RobotData.present_end_pose[5], dataControl->RobotData.present_end_pose[4], dataControl->RobotData.present_end_pose[3], dataControl->PathData.movePath[0].R_init);

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
//                    for(uint i = 0; i < NUM_JOINT; i++){
//                        module->feeding_profile_acc[i] = 100;//50;
//                        module->feeding_profile_vel[i] = 1000;//static_cast<uint32_t>((500/64.0)*module->feeding_profile_acc[i]);
//                        module->feeding_pos_p_gain[i] = default_pos_p_gain[i];
//                    }
//                    module->setGroupSyncWriteIndirectAddress(module->feeding_profile_acc, module->feeding_profile_vel, module->feeding_pos_p_gain, NUM_JOINT);

                    for(uint i = 0; i < NUM_JOINT; i++){
                        module->feeding_profile_acc[i] = 1;//50;
                        module->feeding_profile_vel[i] = 3;//static_cast<uint32_t>((500/64.0)*module->feeding_profile_acc[i]);
                        module->feeding_pos_p_gain[i] = default_pos_p_gain[i];
                    }
                    module->setGroupSyncWriteIndirectAddress(module->feeding_profile_acc, module->feeding_profile_vel, module->feeding_pos_p_gain, NUM_JOINT);

                    dataControl->PathData.total_time.clear();
                    dataControl->PathData.point_px_home.clear();
                    dataControl->PathData.point_py_home.clear();
                    dataControl->PathData.point_pz_home.clear();
                    dataControl->PathData.point_rx_home.clear();
                    dataControl->PathData.point_ry_home.clear();
                    dataControl->PathData.point_rz_home.clear();
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
                    dataControl->PathData.total_time.push_back(3);

                    dataControl->PathData.acc_time.push_back(0.5);
                    dataControl->PathData.acc_time.push_back(0.5);

                    dataControl->PathData.point_px_home.push_back(dataControl->RobotData.present_end_pose[0]);
                    dataControl->PathData.point_py_home.push_back(dataControl->RobotData.present_end_pose[1]);
                    dataControl->PathData.point_pz_home.push_back(dataControl->RobotData.present_end_pose[2]);
                    dataControl->PathData.point_rx_home.push_back(dataControl->RobotData.present_end_pose[3]);
                    dataControl->PathData.point_ry_home.push_back(dataControl->RobotData.present_end_pose[4]);
                    dataControl->PathData.point_rz_home.push_back(dataControl->RobotData.present_end_pose[5]);

                    dataControl->PathData.point_px_home.push_back(dataControl->operateFeedingReadyPose[0]);
                    dataControl->PathData.point_py_home.push_back(dataControl->operateFeedingReadyPose[1]);
                    dataControl->PathData.point_pz_home.push_back(dataControl->operateFeedingReadyPose[2]);
                    dataControl->PathData.point_rx_home.push_back(dataControl->operateFeedingReadyPose[3]);
                    dataControl->PathData.point_ry_home.push_back(dataControl->operateFeedingReadyPose[4]);
                    dataControl->PathData.point_rz_home.push_back(dataControl->operateFeedingReadyPose[5]);

                    RobotArm::zyx2mat(dataControl->RobotData.present_end_pose[5], dataControl->RobotData.present_end_pose[4], dataControl->RobotData.present_end_pose[3], dataControl->PathData.movePath[0].R_init);

                    rt_printf("go home\n");
                    rt_printf("start pose : %f, %f, %f, %f, %f, %f\n",
                              dataControl->RobotData.present_end_pose[0], dataControl->RobotData.present_end_pose[1], dataControl->RobotData.present_end_pose[2],
                              dataControl->RobotData.present_end_pose[3], dataControl->RobotData.present_end_pose[4], dataControl->RobotData.present_end_pose[5]);

                    rt_printf("end pose : %f, %f, %f, %f, %f, %f\n",
                              dataControl->operateFeedingReadyPose[0], dataControl->operateFeedingReadyPose[1], dataControl->operateFeedingReadyPose[2],
                              dataControl->operateFeedingReadyPose[3], dataControl->operateFeedingReadyPose[4], dataControl->operateFeedingReadyPose[5]);

                    robotPathGenerate(dataControl->PathData.point_px_home, dataControl->PathData.point_py_home, dataControl->PathData.point_pz_home,
                                      dataControl->PathData.point_rx_home, dataControl->PathData.point_ry_home, dataControl->PathData.point_rz_home);

                    dataControl->ClientToServer.opMode = DataControl::OpMode::RunMode;

                    dataControl->RobotData.run_mode = DataControl::CmdType::RunCmd;
                    dataControl->PathData.path_data_indx = 0;
                    dataControl->PathData.path_struct_indx = 0;
                    dataControl->PathData.cycle_count_max = 1;
                    delay_cnt = 0;

                    break;
                }
            }
            break;
        }
        case DataControl::Operate::FeedingSwitch:
        {
//            for(uint i = 0; i < NUM_JOINT; i++){
//                module->feeding_profile_acc[i] = 1;//50;
//                module->feeding_profile_vel[i] = 3;//static_cast<uint32_t>((500/64.0)*module->feeding_profile_acc[i]);
//                module->feeding_pos_p_gain[i] = default_pos_p_gain[i];
//            }
//            module->setGroupSyncWriteIndirectAddress(module->feeding_profile_acc, module->feeding_profile_vel, module->feeding_pos_p_gain, NUM_JOINT);

//            dataControl->PathData.total_time.clear();
//            dataControl->PathData.point_px.clear();
//            dataControl->PathData.point_py.clear();
//            dataControl->PathData.point_pz.clear();
//            dataControl->PathData.point_rx.clear();
//            dataControl->PathData.point_ry.clear();
//            dataControl->PathData.point_rz.clear();
//            dataControl->PathData.point_theta.clear();
//            dataControl->PathData.acc_time.clear();

            unsigned int s_count = 0;

            switch(dataControl->KITECHData.interface_sub)
            {
                case 1: // choose section
                {
                    for(uint i = 0; i < NUM_JOINT; i++){
                        module->feeding_profile_acc[i] = 100;//50;
                        module->feeding_profile_vel[i] = 2000;//static_cast<uint32_t>((500/64.0)*module->feeding_profile_acc[i]);
                        module->feeding_pos_p_gain[i] = default_pos_p_gain[i];
                    }
                    module->setGroupSyncWriteIndirectAddress(module->feeding_profile_acc, module->feeding_profile_vel, module->feeding_pos_p_gain, NUM_JOINT);
                    for(unsigned int i = 0; i < NUM_JOINT; i++){
                        dataControl->RobotData.desired_q[i] = dataControl->readyJoints.wp[dataControl->section_indx*6 + i]*dataControl->DEG2RAD;
                    }
                    dataControl->jointPositionRAD2ENC(dataControl->RobotData.desired_q, dataControl->RobotData.command_joint_position);
                    module->setGroupSyncWriteGoalPosition(dataControl->RobotData.command_joint_position, NUM_JOINT);
                    usleep(module->feeding_profile_vel[0]*1000);

                    dataControl->ClientToServer.opMode = DataControl::OpMode::Wait;

                    dataControl->feeding = false;

//                    dataControl->section_indx++;
//                    if(dataControl->section_indx >= 5) {
//                        dataControl->section_indx = 0;
//                    }
//                    rt_printf("Section Index : %d\n", dataControl->section_indx);

                    break;
                }
                case 2: // feeding
                {

                    break;
                }
                default:
                {
                    break;
                }
            }
            break;
        }
        case DataControl::Operate::ReadyFeeding:
        {
            for(uint i = 0; i < NUM_JOINT; i++){
                module->feeding_profile_acc[i] = 100;//50;
                module->feeding_profile_vel[i] = 3000;//static_cast<uint32_t>((500/64.0)*module->feeding_profile_acc[i]);
                module->feeding_pos_p_gain[i] = default_pos_p_gain[i];
            }
            module->setGroupSyncWriteIndirectAddress(module->feeding_profile_acc, module->feeding_profile_vel, module->feeding_pos_p_gain, NUM_JOINT);
            usleep(100000);

            for(int i = 0; i < 6; i++){
                dataControl->RobotData.desired_q[i] = dataControl->operateCameraReadyJoint[i];
            }
            dataControl->jointPositionRAD2ENC(dataControl->RobotData.desired_q, dataControl->RobotData.command_joint_position);
            module->setGroupSyncWriteGoalPosition(dataControl->RobotData.command_joint_position, NUM_JOINT);
            usleep(module->feeding_profile_vel[0]*1000);
//            rt_printf("Wait....\n");
//            usleep(3000000);
//            dataControl->KITECHData.camera_request = true;
//            usleep(3000000);
//            dataControl->KITECHData.camera_request = false;

//            if(dataControl->section_indx == 4 || dataControl->section_indx == 3){
//                for(int i = 0; i < 6; i++){
//                    dataControl->RobotData.desired_q[i] = dataControl->operateCameraReadyJoint[i];
//                }
//                dataControl->jointPositionRAD2ENC(dataControl->RobotData.desired_q, dataControl->RobotData.command_joint_position);
//                module->setGroupSyncWriteGoalPosition(dataControl->RobotData.command_joint_position, NUM_JOINT);
//                usleep(module->feeding_profile_vel[0]*1000);
//            }

//            for(int i = 0; i < 6; i++){
//                dataControl->RobotData.desired_q[i] = dataControl->obi_ready_joint[dataControl->section_indx*6 + i];
//            }
//            dataControl->jointPositionRAD2ENC(dataControl->RobotData.desired_q, dataControl->RobotData.command_joint_position);
//            module->setGroupSyncWriteGoalPosition(dataControl->RobotData.command_joint_position, NUM_JOINT);
//            usleep(module->feeding_profile_vel[0]*1000);

            dataControl->ClientToServer.opMode = DataControl::OpMode::Wait;

            dataControl->feeding = false;

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

    rt_printf("desired end pose : \n%f, %f, %f, %f, %f, %f\n", dataControl->RobotData.desired_end_pose[0], dataControl->RobotData.desired_end_pose[1],
            dataControl->RobotData.desired_end_pose[2], dataControl->RobotData.desired_end_pose[3],
            dataControl->RobotData.desired_end_pose[4], dataControl->RobotData.desired_end_pose[5]);
    rt_printf("present end pose : \n%f, %f, %f, %f, %f, %f\n\n", dataControl->RobotData.present_end_pose[0], dataControl->RobotData.present_end_pose[1],
            dataControl->RobotData.present_end_pose[2], dataControl->RobotData.present_end_pose[3],
            dataControl->RobotData.present_end_pose[4], dataControl->RobotData.present_end_pose[5]);

    dataControl->RobotData.Kp = 300;
    dataControl->RobotData.Dp = 30;
    dataControl->RobotData.Kr = 45;
    dataControl->RobotData.Dr = 0;

    for(int i = 0; i < 3; i++){
        dataControl->RobotData.F[i] =
                dataControl->RobotData.Kp*(dataControl->RobotData.desired_end_pose[i] - dataControl->RobotData.present_end_pose[i]) -
                dataControl->RobotData.Dp*(dataControl->RobotData.present_end_vel[i]);
        dataControl->RobotData.F[i+3] =
                dataControl->RobotData.Kr*(dataControl->RobotData.desired_end_pose[i + 3] - dataControl->RobotData.present_end_pose[i + 3]) -
                dataControl->RobotData.Dr*(dataControl->RobotData.present_end_vel[i + 3]);
    }


    for(uint i = 0; i < 6; i++){
        dataControl->RobotData.Td[i] = 0;
        for(uint j = 0; j < 6; j++){
            dataControl->RobotData.Td[i] += robotArm->J[j*6 + i]*dataControl->RobotData.F[j];
        }
    }

    dataControl->RobotData.T_limit[0] = 30.0;
    dataControl->RobotData.T_limit[1] = 30.0;
    dataControl->RobotData.T_limit[2] = 30.0;
    dataControl->RobotData.T_limit[3] = 30.0;
    dataControl->RobotData.T_limit[4] = 30.0;
    dataControl->RobotData.T_limit[5] = 30.0;

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
//    double epsilon_ang = 0.5;

    double pos = sqrt(pow(desired_pose[0] - present_pose[0], 2) + pow(desired_pose[1] - present_pose[1], 2) + pow(desired_pose[2] - present_pose[2], 2));
//    double ang_x = abs(desired_pose[3] - present_pose[3]);
//    double ang_y = abs(desired_pose[4] - present_pose[4]);
//    double ang_z = abs(desired_pose[5] - present_pose[5]);

//    rt_printf("pos : %f\n", pos);
//    rt_printf("ang_r : %f\t ang_p : %f\t ang_y : %f\n", ang_r, ang_p, ang_y);

    if (pos < epsilon_pos/* && ang_x < epsilon_ang && ang_y < epsilon_ang && ang_z < epsilon_ang*/){
        *goal_reach = true;
    }
    else{
        *goal_reach = false;
    }
}

void ControlMain::path_generator(double x0, double xf, double tf, double ta, double h, std::vector<double> *path, int path_index)
{
    // if(path_index == 1){
    //     double x_step = (xf - x0)/(tf/h);
    //     double x = x0;
    //     for(double t = 0; t < tf; t += h){
    //         path->push_back(x);
    //         x += x_step;
    //     }
    // }
    // else{
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
    // }
}

// void ControlMain::path_generator(double x0, double xf, double tf, double ta, double h, std::vector<double> *path, int path_index)
// {
// //    rt_printf("Start path generator, x0 : %f, xf : %f, tf : %f, ta : %f, h : %f\n", x0, xf, tf, ta, h);
//     double td = tf - ta;
//     double vd = (xf - x0)/td;
//     double xa = x0 + 0.5*ta*vd;
//     double xd = xf - 0.5*ta*vd;

//     double pos0, posf, vel0, velf, acc0, accf, ts;
//     double a0, a1, a2, a3, a4, a5;

//     // section of acceleration
//     pos0 = x0; posf = xa; vel0 = 0; velf = vd; acc0 = 0; accf = 0; ts = ta;
//     a0 = pos0;
//     a1 = vel0;
//     a2 = acc0/2;
//     a3 = (20*(posf - pos0) - (8*velf + 12*vel0)*ts - (3*acc0 - accf)*pow(ts,2))/(2*pow(ts,3));
//     a4 = (30*(pos0 - posf) + (14*velf + 16*vel0)*ts + (3*acc0 - 2*accf)*pow(ts,2))/(2*pow(ts, 4));
//     a5 = (12*(posf - pos0) - (6*velf + 6*vel0)*ts - (acc0 - accf)*pow(ts,2))/(2*pow(ts,5));

//     for(double t = 0; t < ts; t += h){
//         path->push_back(a0 + a1*t + a2*pow(t,2) + a3*pow(t,3) + a4*pow(t,4)+ a5*pow(t,5));
//     }

//     // section of constant velocity
//     pos0 = xa; posf = xd; vel0 = vd; velf = vd; acc0 = 0; accf = 0; ts = td - ta;
//     a0 = pos0;
//     a1 = vel0;
//     a2 = acc0/2;
//     a3 = (20*(posf - pos0) - (8*velf + 12*vel0)*ts - (3*acc0 - accf)*pow(ts,2))/(2*pow(ts,3));
//     a4 = (30*(pos0 - posf) + (14*velf + 16*vel0)*ts + (3*acc0 - 2*accf)*pow(ts,2))/(2*pow(ts, 4));
//     a5 = (12*(posf - pos0) - (6*velf + 6*vel0)*ts - (acc0 - accf)*pow(ts,2))/(2*pow(ts,5));

//     for(double t = 0; t < ts; t += h){
//         path->push_back(a0 + a1*t + a2*pow(t,2) + a3*pow(t,3) + a4*pow(t,4)+ a5*pow(t,5));
//     }

//     // section of deceleration
//     pos0 = xd; posf = xf; vel0 = vd; velf = 0; acc0 = 0; accf = 0; ts = tf - td;
//     a0 = pos0;
//     a1 = vel0;
//     a2 = acc0/2;
//     a3 = (20*(posf - pos0) - (8*velf + 12*vel0)*ts - (3*acc0 - accf)*pow(ts,2))/(2*pow(ts,3));
//     a4 = (30*(pos0 - posf) + (14*velf + 16*vel0)*ts + (3*acc0 - 2*accf)*pow(ts,2))/(2*pow(ts, 4));
//     a5 = (12*(posf - pos0) - (6*velf + 6*vel0)*ts - (acc0 - accf)*pow(ts,2))/(2*pow(ts,5));

//     for(double t = 0; t < ts; t += h){
//         path->push_back(a0 + a1*t + a2*pow(t,2) + a3*pow(t,3) + a4*pow(t,4)+ a5*pow(t,5));
//     }
// }

void ControlMain::getPresentEnc(int32_t enc[])
{
    memcpy(enc, dataControl->RobotData.present_joint_position, sizeof(int32_t)*NUM_JOINT);
}

void ControlMain::setOffsetEnc(int32_t enc[]){
    memcpy(dataControl->RobotData.joint_offset, enc, sizeof(int32_t)*NUM_JOINT);
    dataControl->RobotData.offset_setting = true;
    printf("Setting offset enc pulse\n");
}
