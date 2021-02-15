#include "datacontrol.h"

DataControl::DataControl()
{
//    printf("DataControl Class Constructor\n");

    gpio_task_run = false;
    key_task_run = false;
    init_thread_run = false;
    robot_task_run = false;

    memset(&ClientToServer, 0, sizeof(ClientToServer));
    memset(&ServerToClient, 0, sizeof(ServerToClient));
    memset(&RobotData, 0, sizeof(RobotData));
    config_check = false;
    cartesian_goal_reach = true;
    ui_mode = true;

    RobotData.module_indx = 0;

    key_value = 0;
    data_index = 0;

    step_size = 0.003;

    trayInfor.section1 = 0;
    trayInfor.section2 = 0;
    trayInfor.section3 = 0;
    trayInfor.section4 = 0;
    trayInfor.section5 = 0;

    KITECHData.camera_request = false;
    KITECHData.camera_request_old = KITECHData.camera_request;
}

DataControl::~DataControl()
{
//    printf("DataControl Class Destructor\n");
}
