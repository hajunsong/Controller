#include "controlmain.h"

ControlMain::ControlMain()
{
//    printf("ControlMain Class Constructor\n");

    gpio0 = TP_PORT::port0;
    gpio1 = TP_PORT::port1;

    dataControl = new DataControl();

    tcpServerClient = new TcpServer(dataControl);
    tcpServerLatte = new TcpServer(dataControl);

    dataControl->RobotData.module_init = false;
    module = new DxlControl();
}

ControlMain::~ControlMain()
{
//    printf("\nControlMain Class Destructor\n");
    if(dataControl->gpio_task_run){
        dataControl->gpio_task_run = false;
        rt_task_join(&rtGpioTask);
        usleep(10000);
        rt_task_delete(&rtGpioTask);
        usleep(10000);
        printf("Finished GPIO Task\n");
    }
    if(dataControl->key_task_run){
        dataControl->key_task_run = false;
        rt_task_join(&rtKeyTask);
        usleep(10000);
        rt_task_delete(&rtKeyTask);
        usleep(10000);
        printf("Finished Key Input Task\n");
    }
    delete module;
    usleep(10000);
    delete tcpServerClient;
    usleep(10000);
    delete tcpServerLatte;
    usleep(10000);
    delete dataControl;
    usleep(10000);
}

void ControlMain::start()
{
    startGpioTask();
    usleep(10000);

    startKeyTask();
    usleep(10000);

    tcpServerClient->setting(5050);
    tcpServerClient->start();
    usleep(10000);

    tcpServerLatte->setting(5053);
    tcpServerLatte->start();
    usleep(10000);

    dataControl->init_thread_run = true;
    pthread_create(&init_thread, nullptr, init_func, this);
}

void ControlMain::startGpioTask()
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
    rt_task_create(&rtGpioTask, "rtGpioTask", 0, 99, 0);
    rt_task_start(&rtGpioTask, &rtGpioFunc, this);
}

void ControlMain::startKeyTask()
{
    mlockall(MCL_CURRENT|MCL_FUTURE);

    /* Perform auto-init of rt_print buffers if the task doesn't do so */
    rt_print_auto_init(1);

    printf("Running, Keyboard Input Manager....\n");

    //create and start a realtime periodic task for writing gpio
    rt_task_create(&rtKeyTask, "rtKeyTask", 0, 99, 0);
    rt_task_start(&rtKeyTask, &rtKeyFunc, this);
}

void ControlMain::rtGpioFunc(void *arg)
{
    ControlMain *pControlMain = static_cast<ControlMain*>(arg);
    uint8_t gpio0_state_old;
    int step = 0;
    unsigned int count = 0;
    bool led = true;
    const unsigned int led_onoff_count = 5;

    rt_task_set_periodic(NULL, TM_NOW, 100e6);

    pControlMain->dataControl->key_value = 0;
    gpio0_state_old = 0;

    tp_writeport(pControlMain->gpio0, 0x00);
    tp_writeport(pControlMain->gpio1, 0x01);

    pControlMain->dataControl->gpio_task_run = true;
    pControlMain->dataControl->KITECHData.tablet_connect = false;

    while(pControlMain->dataControl->gpio_task_run){
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

        if((pControlMain->gpio0_state) != (gpio0_state_old)){
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

            if((~pControlMain->gpio0_state & pin00) == pin00)
            {
                pControlMain->dataControl->key_value = KEY_Z;
                pControlMain->gpio1_state |= pin01;
            }
            else if((~pControlMain->gpio0_state & pin01) == pin01){
                pControlMain->dataControl->key_value = KEY_X;
                pControlMain->gpio1_state |= pin02;
            }
            else if((~pControlMain->gpio0_state & pin02) == pin02){
                pControlMain->dataControl->key_value = KEY_C;
                pControlMain->gpio1_state |= pin03;
            }
            tp_writeport(pControlMain->gpio1, pControlMain->gpio1_state);
            usleep(5000);

            rt_printf("key value : %d\n", pControlMain->dataControl->key_value);

            if(pControlMain->dataControl->key_value == KEY_Z){
                pControlMain->gpio0_state &= ~(pin03|pin04|pin05);
                tp_writeport(pControlMain->gpio0, pControlMain->gpio0_state);
                usleep(5000);
            }

            else if(pControlMain->dataControl->key_value == KEY_X){

            }
            else if(pControlMain->dataControl->key_value == KEY_C){

            }
            else{
            }
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
                    pControlMain->gpio0_state = led ? pControlMain->gpio0_state|0x30 : pControlMain->gpio0_state&(0xCF);
                    tp_writeport(pControlMain->gpio0, pControlMain->gpio0_state);
                }
                else{
                    pControlMain->gpio0_state |= 0x30;
                    tp_writeport(pControlMain->gpio0, pControlMain->gpio0_state);
                }
                break;
            default:
                break;
        }

        pControlMain->dataControl->key_value = 0;
        gpio0_state_old = pControlMain->gpio0_state;
    }
}

void ControlMain::rtKeyFunc(void *arg)
{
    ControlMain *pControlMain = static_cast<ControlMain*>(arg);

    rt_task_set_periodic(NULL, TM_NOW, 10e6);

    pControlMain->dataControl->key_task_run = true;
    pControlMain->dataControl->KITECHData.tablet_connect = false;

    while(pControlMain->dataControl->key_task_run){
        rt_task_wait_period(NULL);
        pControlMain->dataControl->key_value = getch();
        rt_printf("key value : %d\n", pControlMain->dataControl->key_value);

        if(pControlMain->dataControl->key_value == KEY_A){
            pControlMain->dataControl->KITECHData.camera_request = true;
        }
        else if(pControlMain->dataControl->key_value == KEY_S){
            pControlMain->dataControl->KITECHData.camera_request = false;
        }
    }
    rt_printf("Finished Key Input Thread\n");
}

void *ControlMain::init_func(void *arg)
{
    ControlMain *pThis = static_cast<ControlMain*>(arg);

    while(!pThis->dataControl->config_check){
        usleep(10000);
    }
    pThis->module->init();

    while(pThis->dataControl->init_thread_run){
        switch(MODULE_TYPE){
            case DataControl::Module::FAR_V1:
                if (!pThis->dataControl->RobotData.module_init){
                    printf("Start FAR V1 Module Initilization\n");
                    pThis->dataControl->RobotData.module_indx = 0;
                    pThis->moduleInitFAR();
                }
                break;
            case DataControl::Module::FAR_V2:
                if (!pThis->dataControl->RobotData.module_init){
                    printf("Start FAR V2 Module Initilization\n");
                    pThis->dataControl->RobotData.module_indx = 0;
                    pThis->moduleInitFAR();
                }
            case DataControl::Module::FAR_V3:
                if (!pThis->dataControl->RobotData.module_init){
                    printf("Start FAR V3 Module Initilization\n");
                    pThis->dataControl->RobotData.module_indx = 0;
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

void ControlMain::moduleInitFAR()
{
    while(!dataControl->RobotData.module_init)
    {
        printf("Start module initialize %d\n", dataControl->RobotData.module_indx);
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
            int init_result = module->dxl_init(dataControl->RobotData.module_indx, dataControl->RobotData.joint_op_mode);
            if (init_result){
                int32_t pos = 0;
                module->getPresentPosition(dataControl->RobotData.module_indx, &pos);
                printf("%d axis present position : %d\n", dataControl->RobotData.module_indx, pos);
                if (pos != 0)
                {
                    module->initGroupSyncReadIndirectAddress(dataControl->RobotData.module_indx);
                    module->initGroupSyncWriteIndirectAddress(dataControl->RobotData.module_indx);
                    dataControl->RobotData.module_indx++;

                    if(dataControl->RobotData.module_indx == 1){
                        if(pos < 3000){

                        }
                    }
                }
            }
            if (dataControl->RobotData.module_indx >= NUM_JOINT){
                dataControl->RobotData.module_init = true;

//                if (!dataControl->RobotData.offset_setting){
//                    module->getGroupSyncReadPresentPosition(dataControl->RobotData.offset, NUM_JOINT);
//                }
//                dataControl->RobotData.offset[3] -= static_cast<int32_t>(-90*dataControl->DEG2ENC);
//                dataControl->RobotData.offset[5] -= static_cast<int32_t>(-90*dataControl->DEG2ENC);
            }
        }
    }

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

    module->setGoalPosition(3, dataControl->RobotData.initJoint_4);
    usleep(1500000);

    module->setGoalPosition(1, dataControl->RobotData.initJoint_2);
    usleep(1500000);

    tcpServerClient->sendKey("S");
    startRobotTask();
}

void ControlMain::startRobotTask(){
    mlockall(MCL_CURRENT|MCL_FUTURE);

    /* Perform auto-init of rt_print buffers if the task doesn't do so */
    rt_print_auto_init(1);

    int ret = rt_task_create(&rtRobotTask, "Robot Controll Task", 0, 99, 0);
    if (ret < 0){
        printf("Failed to create Robot Control Task %s\n", strerror(-ret));
    }
    else{
        printf("Robot Control RT Task Start\n");

        dataControl->ClientToServer.opMode = DataControl::OpMode::Wait;

        rt_task_start(&rtRobotTask, &rtRobotFunc, this);
    }
}

void ControlMain::rtRobotFunc(void *arg){
    ControlMain* pThis = static_cast<ControlMain*>(arg);

    pThis->dataControl->robot_task_run = false;

    rt_task_set_periodic(&pThis->rtRobotTask, TM_NOW, 3e6);

    pThis->dataControl->robot_task_run = true;

    while(pThis->dataControl->robot_task_run){
        rt_task_wait_period(nullptr); //wait for next cycle

        pThis->controlMainCustom->robot_run(arg);
    }
}
