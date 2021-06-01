#include "tcpserver.h"

TcpServer::TcpServer(DataControl *_dataControl)
{
    dataControl = _dataControl;

    connected = false;

    temp = 0;

    comm_period = 100e6;
    recv_task_run = false;
    send_task_run = false;
    comm_manager_run = false;
}

TcpServer::~TcpServer(){
    stop();
}

void TcpServer::start(){
    pthread_create(&comm_manager_thread, nullptr, comm_manager_func, this);
}

void TcpServer::stop(){
    rt_task_suspend(&recv_task);
    usleep(10000);
    rt_task_delete(&recv_task);
    usleep(10000);
    printf("Finished Comm Rx Task\n");
    rt_task_suspend(&send_task);
    usleep(10000);
    rt_task_delete(&send_task);
    usleep(10000);
    printf("Finished Comm Tx Task\n");

    pthread_cancel(comm_manager_thread);
    usleep(10000);
    printf("Finished Comm Manager Thread\n");
}

void TcpServer::setting(uint16_t _port)
{
    port = _port;
}


void *TcpServer::comm_manager_func(void *arg)
{
    TcpServer *pTcpServer = static_cast<TcpServer*>(arg);

    pTcpServer->comm_manager_run = true;

    rt_print_auto_init(1);

    /* Avoids memory swapping for this program */
    mlockall(MCL_CURRENT|MCL_FUTURE);

    pTcpServer->initSocket();

    while(pTcpServer->comm_manager_run){
        pTcpServer->connectSocket();

        int ret = rt_task_create(&pTcpServer->recv_task, "Tcp Comm RX Task", 0, 96, T_JOINABLE);
        if (ret < 0){
            rt_printf("Failed to create Tcp Comm RX Task : %s\n", strerror(-ret));
        }
        else{
            rt_printf("Tcp Comm RX RT Task Start\n");

            rt_task_start(&pTcpServer->recv_task, &pTcpServer->recv_RT, arg);
        }

        ret = rt_task_create(&pTcpServer->send_task, "Tcp Comm TX Task", 0, 97, T_JOINABLE);
        if (ret < 0){
            rt_printf("Failed to create Tcp Comm TX Task : %s\n", strerror(-ret));
        }
        else{
            rt_printf("Tcp Comm TX RT Task Start\n");

            rt_task_start(&pTcpServer->send_task, &pTcpServer->send_RT, arg);
        }

        rt_task_join(&pTcpServer->recv_task);
        usleep(10000);
        rt_task_join(&pTcpServer->send_task);
        usleep(10000);
        rt_task_delete(&pTcpServer->recv_task);
        usleep(10000);
        rt_task_delete(&pTcpServer->send_task);
        usleep(10000);
    }
    return nullptr;
}

void TcpServer::recv_RT(void *arg)
{
    TcpServer *pTcpServer = static_cast<TcpServer*>(arg);

    rt_task_set_periodic(NULL, TM_NOW, pTcpServer->comm_period);

    pTcpServer->recv_task_run = true;

    while(pTcpServer->comm_manager_run && pTcpServer->recv_task_run){
        rt_task_wait_period(NULL);

        if(pTcpServer->isConnected()){
            memset(pTcpServer->bufRecv, 0, MAXRECEIVEBUFSIZE);
            pTcpServer->byteLen = recv(pTcpServer->clientSockFD, pTcpServer->bufRecv, MAXRECEIVEBUFSIZE, 0);

            if(pTcpServer->byteLen > 0){
                if(pTcpServer->port ==  5053){
                    rt_printf("Received byteLen : %ld\n", pTcpServer->byteLen);
                    rt_printf("received data : ");
                    for(uint8_t i = 0; i < 23; i++){
                        rt_printf("%d ", pTcpServer->bufRecv[i]);
                    }
                    rt_printf("\n");
                }
            }

            if(pTcpServer->byteLen == 0){
                rt_printf("Recv Disconnected\n");
                pTcpServer->connected = false;
            }
            else if(pTcpServer->byteLen > 0){
                pTcpServer->recvData();
            }
        }
        else{
            pTcpServer->recv_task_run = false;
            break;
        }
    }

    rt_printf("Finished Comm RX Thread\n");
}

void TcpServer::send_RT(void *arg){
    TcpServer *pTcpServer = static_cast<TcpServer*>(arg);

    rt_task_set_periodic(NULL, TM_NOW, pTcpServer->comm_period);

    pTcpServer->sendByteLen = 0;
    pTcpServer->send_task_run = true;

    while(pTcpServer->comm_manager_run && pTcpServer->send_task_run){
        rt_task_wait_period(NULL);

        if(pTcpServer->connected){
            pTcpServer->sendData();

            if(pTcpServer->sendByteLen < 0){
                rt_printf("Send Disconnected(%d)\n", pTcpServer->port);
                pTcpServer->connected = false;
            }
        }
        else{
            pTcpServer->send_task_run = false;
            break;
        }
    }

    rt_printf("Finished Comm TX Thread(%d)\n", pTcpServer->port);
}

void TcpServer::recvData(){
    if(/*(byteLen == 23 || byteLen == 4) && */bufRecv[0] == SOP_RX/* && !dataControl->feeding*//* && (buf[3] == EOP || buf[22] == EOP)*/){
        dataControl->KITECHData.camera_request = false;
        dataControl->KITECHData.interface_cmd = bufRecv[1];
        dataControl->KITECHData.interface_sub = bufRecv[2];

        switch(dataControl->KITECHData.interface_cmd){
            case CMD_DATA:
            {
                rt_printf("Received Data Packet\n");

                for(int i = 0; i < 20; i++){
                    dataControl->KITECHData.food_pixel[i] = (char)bufRecv[i + 2];
                }

                for(int i = 0, j = 0; i < 10; i++, j+=2){
                    dataControl->KITECHData.food_pos[i] = (int)dataControl->KITECHData.food_pixel[j]*100 + (int)dataControl->KITECHData.food_pixel[j+1];
                }

                rt_printf("%d, %d\n", dataControl->KITECHData.food_pos[0], dataControl->KITECHData.food_pos[1]);
                rt_printf("%d, %d\n", dataControl->KITECHData.food_pos[2], dataControl->KITECHData.food_pos[3]);
                rt_printf("%d, %d\n", dataControl->KITECHData.food_pos[4], dataControl->KITECHData.food_pos[5]);
                rt_printf("%d, %d\n", dataControl->KITECHData.food_pos[6], dataControl->KITECHData.food_pos[7]);
                rt_printf("%d, %d\n", dataControl->KITECHData.food_pos[8], dataControl->KITECHData.food_pos[9]);

                for(int i = 0; i < 10; i++){
                    if(abs(dataControl->KITECHData.food_pos[i]) < 10 || abs(dataControl->KITECHData.food_pos[i]) > 200){
                        rt_printf("Re-Send to KITECH request camera\n");
                        dataControl->KITECHData.camera_request = true;
                        usleep(3000000);
                        dataControl->KITECHData.camera_request = false;
                        break;
                    }
                }
                break;
            }
            case CMD_SECTION:
            {
                rt_printf("Received Section Packet\n");
                // choose section
                if(dataControl->KITECHData.interface_sub == 1){
                    dataControl->section_indx++;
                    if(dataControl->section_indx >= 5) {
                        dataControl->section_indx = 0;
                    }
                    rt_printf("Section Index : %d\n", dataControl->section_indx);
                }
                else if(dataControl->KITECHData.interface_sub == 2){
                    rt_printf("Received Feeding Packet\n");
                    rt_printf("Section : %d\n", dataControl->section_indx);
                }

                dataControl->ClientToServer.opMode = DataControl::OpMode::OperateMode;
                dataControl->operateMode.mode = DataControl::Operate::Feeding;
                switch(dataControl->section_indx){
                    case 0: // side 1
                    {
                        rt_printf("section1 count : %d, %d\n", dataControl->trayInfor.section1, dataControl->trayInfor.section1%2);
                        dataControl->operateMode.section = DataControl::Section::Side1;
                        break;
                    }
                    case 1: // side 2
                    {
                        rt_printf("section2 count : %d, %d\n", dataControl->trayInfor.section2, dataControl->trayInfor.section2%2);
                        dataControl->operateMode.section = DataControl::Section::Side2;
                        break;
                    }
                    // case 2: // side 3
                    // {
                    //     rt_printf("section3 count : %d, %d\n", dataControl->trayInfor.section3, dataControl->trayInfor.section3%2);
                    //     dataControl->operateMode.section = DataControl::Section::Side3;
                    //     break;
                    // }
                    // case 3: // soup
                    // {
                    //     rt_printf("section4 count : %d\n", dataControl->trayInfor.section4);
                    //     dataControl->operateMode.section = DataControl::Section::Soup;
                    //     break;
                    // }
                    case 4: // rice
                    {
                        rt_printf("section5 count : %d, %d\n", dataControl->trayInfor.section5, dataControl->trayInfor.section5%3);
                        dataControl->operateMode.section = DataControl::Section::Rice;
                        break;
                    }
                }
                break;
            }
            case CMD_FEEDING:
            {
                rt_printf("Received Feeding Packet\n");
                rt_printf("Section : %d\n", dataControl->KITECHData.interface_sub);
                dataControl->section_indx = dataControl->KITECHData.interface_sub - 1;
                dataControl->ClientToServer.opMode = DataControl::OpMode::OperateMode;
                dataControl->operateMode.mode = DataControl::Operate::Feeding;
                switch(dataControl->section_indx){
                    case 0: // side 1
                    {
                        rt_printf("section1 count : %d, %d\n", dataControl->trayInfor.section1, dataControl->trayInfor.section1%2);
                        dataControl->operateMode.section = DataControl::Section::Side1;
                        break;
                    }
                    case 1: // side 2
                    {
                        rt_printf("section2 count : %d, %d\n", dataControl->trayInfor.section2, dataControl->trayInfor.section2%2);
                        dataControl->operateMode.section = DataControl::Section::Side2;
                        break;
                    }
                    case 2: // side 3
                    {
                        rt_printf("section3 count : %d, %d\n", dataControl->trayInfor.section3, dataControl->trayInfor.section3%2);
                        dataControl->operateMode.section = DataControl::Section::Side3;
                        break;
                    }
                    case 3: // soup
                    {
                        rt_printf("section4 count : %d\n", dataControl->trayInfor.section4);
                        dataControl->operateMode.section = DataControl::Section::Soup;
                        break;
                    }
                    case 4: // rice
                    {
                        rt_printf("section5 count : %d, %d\n", dataControl->trayInfor.section5, dataControl->trayInfor.section5%3);
                        dataControl->operateMode.section = DataControl::Section::Rice;
                        break;
                    }
                }
                break;
            }
            case CMD_TABLET_CHECK:
            {
                if(dataControl->KITECHData.interface_sub == 1){
                    dataControl->KITECHData.tablet_connect = true;
                }
                else{
                    dataControl->KITECHData.tablet_connect = false;
                }
            }
            default:
            {
                break;
            }
        }
    }

    if(bufRecv[0] == 'N' && bufRecv[1] == 'D'){
        dataControl->config_check = false;
        dataControl->RobotData.module_init = false;

        if(bufRecv[2] == NUM_JOINT && bufRecv[3] == NUM_DOF && bufRecv[4] == dataControl->MODULE_TYPE){
            rt_printf("Client & Server configuration check complete\n");
            dataControl->RobotData.joint_op_mode = bufRecv[5];
            dataControl->config_check = true;
            dataControl->tablet_mode = false;
        }
        else{
            if(bufRecv[2] != NUM_JOINT){
                rt_printf("Client & Server does not matched number of joints\n");
            }
            if(bufRecv[3] != NUM_DOF){
                rt_printf("Client & Server does not matched degree of freedom\n");
            }
            if(bufRecv[4] != dataControl->MODULE_TYPE){
                rt_printf("Client & Server does not matched module type\n");
            }
        }

        if(!dataControl->config_check){
            sendKey("X");
        }
    }
    else if(bufRecv[0] == 'N' && bufRecv[1] == 'S'){
        uint16_t indx = SOCKET_TOKEN_SIZE;

        dataControl->ClientToServer.opMode = bufRecv[indx];
        indx += OP_MODE_LEN;
        dataControl->ClientToServer.subMode = bufRecv[indx];
        indx += SUB_MODE_LEN;

        if(dataControl->ClientToServer.opMode == DataControl::OpMode::JointMove){
            char buf[8];
            for(int i = 0; i < NUM_JOINT; i++, indx += DESIRED_JOINT_LEN){
                memcpy(buf, bufRecv + indx, DESIRED_JOINT_LEN);
                dataControl->ClientToServer.desiredJoint[i] = atof(buf);
            }
            for(int i = 0; i < NUM_DOF; i++){
                dataControl->ClientToServer.desiredPose[i] = 0;
            }
        }
        else if(dataControl->ClientToServer.opMode == DataControl::OpMode::CartesianMove){
            char buf[8];
            for(int i = 0; i < NUM_JOINT; i++){
                dataControl->ClientToServer.desiredJoint[i] = 0;
            }
            for(int i = 0; i < NUM_DOF; i++, indx += DESIRED_CARTESIAN_LEN){
                memcpy(buf, bufRecv + indx, DESIRED_CARTESIAN_LEN);
                dataControl->ClientToServer.desiredPose[i] = atof(buf);
            }
            memcpy(buf, bufRecv + indx, MOVE_TIME_LEN);
            dataControl->ClientToServer.move_time = atof(buf);
            indx += MOVE_TIME_LEN;
            memcpy(buf, bufRecv + indx, ACC_TIME_LEN);
            dataControl->ClientToServer.acc_time = atof(buf);
            indx += ACC_TIME_LEN;
        }

//        rt_printf("op mode : %d\n", dataControl->ClientToServer.opMode);
//        rt_printf("sub mode : %d\n", dataControl->ClientToServer.subMode);
//        for(int i = 0; i < 6; i++){
//            rt_printf("des joint %d : %f\n", i, dataControl->ClientToServer.desiredJoint[i]);
//        }
//        for(int i = 0; i < 6; i++){
//            rt_printf("des pose %d : %f\n", i, dataControl->ClientToServer.desiredPose[i]);
//        }
//        rt_printf("move_time : %f\n", dataControl->ClientToServer.move_time);
//        rt_printf("acc_time  : %f\n", dataControl->ClientToServer.acc_time);
    }
    else if(bufRecv[0] == 'N' && bufRecv[1] == 'U'){
        uint16_t indx = SOCKET_TOKEN_SIZE;

        dataControl->ClientToServer.opMode = static_cast<char>(bufRecv[indx]);
        indx += OP_MODE_LEN;
        dataControl->PathData.cmd_type = static_cast<char>(bufRecv[indx]);
        indx += CMD_TYPE_LEN;

        switch(dataControl->PathData.cmd_type){
            case DataControl::CmdType::PathCmd:
            {
                dataControl->PathData.row = static_cast<char>(bufRecv[indx]);
                indx += ROW_SIZE_LEN;
                dataControl->PathData.col = static_cast<char>(bufRecv[indx]);
                indx += COL_SIZE_LEN;
                dataControl->PathData.total_time.clear();
                dataControl->PathData.point_px.clear();
                dataControl->PathData.point_py.clear();
                dataControl->PathData.point_pz.clear();
                dataControl->PathData.acc_time.clear();
                dataControl->PathData.point_rx.clear();
                dataControl->PathData.point_ry.clear();
                dataControl->PathData.point_rz.clear();
                dataControl->PathData.point_theta.clear();

                char buf[8];
                double bufData = 0;
                for(int i = 0; i < dataControl->PathData.row; i++){
                    memcpy(buf, bufRecv + indx, MOVE_TIME_LEN);
                    bufData = atof(buf);
                    dataControl->PathData.total_time.push_back(bufData);
                    indx += MOVE_TIME_LEN;

                    memcpy(buf, bufRecv + indx, PATH_DATA_LEN);
                    bufData = atof(buf);
                    dataControl->PathData.point_px.push_back(bufData);
                    indx += PATH_DATA_LEN;

                    memcpy(buf, bufRecv + indx, PATH_DATA_LEN);
                    bufData = atof(buf);
                    dataControl->PathData.point_py.push_back(bufData);
                    indx += PATH_DATA_LEN;

                    memcpy(buf, bufRecv + indx, PATH_DATA_LEN);
                    bufData = atof(buf);
                    dataControl->PathData.point_pz.push_back(bufData);
                    indx += PATH_DATA_LEN;

                    memcpy(buf, bufRecv + indx, PATH_DATA_LEN);
                    bufData = atof(buf);
                    dataControl->PathData.point_rx.push_back(bufData);
                    indx += PATH_DATA_LEN;

                    memcpy(buf, bufRecv + indx, PATH_DATA_LEN);
                    bufData = atof(buf);
                    dataControl->PathData.point_ry.push_back(bufData);
                    indx += PATH_DATA_LEN;

                    memcpy(buf, bufRecv + indx, PATH_DATA_LEN);
                    bufData = atof(buf);
                    dataControl->PathData.point_rz.push_back(bufData);
                    indx += PATH_DATA_LEN;

                    memcpy(buf, bufRecv + indx, ACC_TIME_LEN);
                    bufData = atof(buf);
                    dataControl->PathData.acc_time.push_back(bufData);
                    indx += ACC_TIME_LEN;
                }
                rt_printf("received path data : \n");
                for(int i = 0; i < dataControl->PathData.row; i++){
                    rt_printf("%f %f %f %f %f %f %f %f\n",
                              dataControl->PathData.total_time[i], dataControl->PathData.point_px[i],
                              dataControl->PathData.point_py[i], dataControl->PathData.point_pz[i],
                              dataControl->PathData.point_rx[i], dataControl->PathData.point_ry[i],
                              dataControl->PathData.point_rz[i], dataControl->PathData.acc_time[i]);
                }

                break;
            }
            case DataControl::CmdType::ReadyCmd:
            {
                dataControl->RobotData.run_mode = DataControl::CmdType::ReadyCmd;
                dataControl->PathData.path_data_indx = 0;
                dataControl->PathData.path_struct_indx = 0;
                for(uint i = 0; i < dataControl->PathData.row; i++){
                    dataControl->PathData.movePath[i].path_x.clear();
                    dataControl->PathData.movePath[i].path_y.clear();
                    dataControl->PathData.movePath[i].path_z.clear();
                    dataControl->PathData.movePath[i].path_theta.clear();
                }
                dataControl->PathData.readyPath.path_x.clear();
                dataControl->PathData.readyPath.path_y.clear();
                dataControl->PathData.readyPath.path_z.clear();
                dataControl->PathData.readyPath.path_theta.clear();
                rt_printf("Ready Feeding Assitant Robot\n");
                break;
            }
            case DataControl::CmdType::RunCmd:
            {
                dataControl->PathData.cycle_count_max = static_cast<char>(bufRecv[indx]);
                indx += CYCLE_COUNT_LEN;
                dataControl->RobotData.run_mode = DataControl::CmdType::RunCmd;
                dataControl->PathData.path_data_indx = 0;
                dataControl->PathData.path_struct_indx = 0;
                dataControl->PathData.cycle_count = 1;
                rt_printf("Start Feeding Assitant Robot\n");
                break;
            }
            case DataControl::CmdType::StopCmd:
            {
                dataControl->PathData.path_data_indx = 0;
                dataControl->PathData.path_struct_indx = 0;
                dataControl->ClientToServer.opMode = DataControl::OpMode::Wait;
                break;
            }
            case DataControl::CmdType::FileReady:
            {
                dataControl->PathData.cycle_count_max = static_cast<char>(bufRecv[indx]);
                dataControl->RobotData.run_mode = DataControl::CmdType::FileReady;
                dataControl->PathData.path_data_indx = 0;
                break;
            }
            case DataControl::CmdType::FileRun:
            {
                dataControl->PathData.cycle_count_max = static_cast<char>(bufRecv[indx]);
                dataControl->RobotData.run_mode = DataControl::CmdType::FileRun;
                dataControl->PathData.path_data_indx = 0;
                break;
            }
            case DataControl::CmdType::CustomRun:
            {
                dataControl->PathData.cycle_count_max = static_cast<char>(bufRecv[indx]);
                dataControl->RobotData.run_mode = DataControl::CmdType::CustomRun;
                dataControl->PathData.path_data_indx = 0;
                break;
            }
        }
    }
    else if(bufRecv[0] == 'N' && bufRecv[1] == 'O'){
        rt_printf("%d\n", dataControl->feeding);
        if (!dataControl->feeding){
            int indx = SOCKET_TOKEN_SIZE;
            dataControl->ClientToServer.opMode = bufRecv[indx];
            indx += OP_MODE_LEN;
            dataControl->operateMode.mode = bufRecv[indx];
            indx += SUB_MODE_LEN;
            dataControl->operateMode.section = bufRecv[indx];
            indx += SECTION_LEN;

            printf("Mode : %d, Sub Mode : %d, Section : %d\n", dataControl->ClientToServer.opMode,
                   dataControl->operateMode.mode, dataControl->operateMode.section);
        }
    }
}

void TcpServer::sendData(){
    if(port == 5050){
//        rt_printf("server to client size : %d\n", dataControl->ServerToClient.size());
        if(dataControl->ServerToClient.size() > 0){
            int size = dataControl->ServerToClient.size();
            for(int i = 0; i < size; i++){
                uint16_t indx = 0;
                memset(bufSend, 0, SENDBUFSIZE);
                strcpy(bufSend, "NC");
                indx = SOCKET_TOKEN_SIZE;

                bufSend[indx] = dataControl->ServerToClient.front().data_index;
                indx += DATA_INDEX_LEN;
//                rt_printf("Indx : %d\n", dataControl->ServerToClient.front().data_index);

                for(int j = 0; j < NUM_JOINT; j++){
                    memcpy(bufSend + indx, to_string(dataControl->ServerToClient.front().presentJointPosition[j]).c_str(), MOTION_DATA_LEN);
                    indx += MOTION_DATA_LEN;
                }
                for(int j = 0; j < NUM_DOF; j++){
                    memcpy(bufSend + indx, to_string(dataControl->ServerToClient.front().presentCartesianPose[j]).c_str(), MOTION_DATA_LEN);
                    indx += MOTION_DATA_LEN;
                }
                for(int j = 0; j < NUM_JOINT; j++){
                    memcpy(bufSend + indx, to_string(dataControl->ServerToClient.front().desiredJointPosition[j]).c_str(), MOTION_DATA_LEN);
                    indx += MOTION_DATA_LEN;
                }
                for(int j = 0; j < NUM_DOF; j++){
                    memcpy(bufSend + indx, to_string(dataControl->ServerToClient.front().desiredCartesianPose[j]).c_str(), MOTION_DATA_LEN);
                    indx += MOTION_DATA_LEN;
                }
                for(int j = 0; j < NUM_DOF; j++){
                    memcpy(bufSend + indx, to_string(dataControl->ServerToClient.front().calculateCartesianPose[j]).c_str(), MOTION_DATA_LEN);
                    indx += MOTION_DATA_LEN;
                }
                for(int j = 0; j < NUM_JOINT; j++){
                    memcpy(bufSend + indx, to_string(dataControl->ServerToClient.front().presentJointVelocity[j]).c_str(), MOTION_DATA_LEN);
                    indx += MOTION_DATA_LEN;
                }
                for(int j = 0; j < NUM_JOINT; j++){
                    memcpy(bufSend + indx, to_string(dataControl->ServerToClient.front().presentJointCurrent[j]).c_str(), MOTION_DATA_LEN);
                    indx += MOTION_DATA_LEN;
                }
                for(int j = 0; j < NUM_DOF; j++){
                    memcpy(bufSend + indx, to_string(dataControl->ServerToClient.front().presentCartesianVelocity[j]).c_str(), MOTION_DATA_LEN);
                    indx += MOTION_DATA_LEN;
                }
                memcpy(bufSend + indx, to_string(dataControl->ServerToClient.front().time).c_str(), TIME_LEN);
                indx += TIME_LEN;
                memcpy(bufSend + indx, to_string(dataControl->ServerToClient.front().dxl_time).c_str(), TIME_LEN);
                indx += TIME_LEN;
                memcpy(bufSend + indx, to_string(dataControl->ServerToClient.front().ik_time).c_str(), TIME_LEN);
                indx += TIME_LEN;

                for(int j = 0; j < NUM_JOINT; j++){
                    memcpy(bufSend + indx, to_string(dataControl->ServerToClient.front().presentJointTorque[j]).c_str(), MOTION_DATA_LEN);
                    indx += MOTION_DATA_LEN;
                }
                for(int j = 0; j < NUM_JOINT; j++){
                    memcpy(bufSend + indx, to_string(dataControl->ServerToClient.front().presentJointResidual[j]).c_str(), MOTION_DATA_LEN);
                    indx += MOTION_DATA_LEN;
                }


                strcpy(bufSend + indx, "NE");
                indx += SOCKET_TOKEN_SIZE;

                sendByteLen = send(clientSockFD, bufSend, static_cast<size_t>(indx), 0);
//                rt_printf("sendByteLen : %d\n", sendByteLen);
//                rt_printf("Send data : ");
//                for(int i = 0; i < sendByteLen; i++){
//                    rt_printf("%d ", bufSend[i]);
//                }
//                rt_printf("\n");
                dataControl->ServerToClient.erase(dataControl->ServerToClient.begin());

                if(sendByteLen < 0){
                    rt_printf("Send Error\n");
                    comm_manager_run = false;
                    connected = false;
                    dataControl->config_check = false;
                    dataControl->ServerToClient.clear();

                    return;
                }
            }
        }
    }
    else if(port == 5053){
        if(dataControl->KITECHData.camera_request && (dataControl->KITECHData.camera_request_old != dataControl->KITECHData.camera_request)){
            memset(bufSend, 0, SENDBUFSIZE);

            bufSend[0] = SOP_TX;
            bufSend[1] = 1;
            bufSend[2] = EOP;
            sendByteLen = send(clientSockFD, bufSend, 3, 0);
            rt_printf("Send to KITECH request camera\n");

            if(sendByteLen < 0){
                rt_printf("Send Error\n");
                comm_manager_run = false;
                connected = false;

                return;
            }
        }
        dataControl->KITECHData.camera_request_old = dataControl->KITECHData.camera_request;

        if(dataControl->KITECHData.tablet_check && (dataControl->KITECHData.tablet_check_old != dataControl->KITECHData.tablet_check)){
            memset(bufSend, 0, SENDBUFSIZE);

            bufSend[0] = SOP_TX;
            bufSend[1] = 2;
            bufSend[2] = EOP;
            sendByteLen = send(clientSockFD, bufSend, 3, 0);
            rt_printf("Send to KITECH tablet connect check\n");

            if(sendByteLen < 0){
                rt_printf("Send Error\n");
                comm_manager_run = false;
                connected = false;

                return;
            }
        }
        dataControl->KITECHData.tablet_check_old = dataControl->KITECHData.tablet_check;
    }
}

void TcpServer::initSocket()
{
    listenSockFD = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if(listenSockFD < 0){
        cout << endl << "socket create error" << endl;
        return;
    }

    int on = 1;
    if(setsockopt(listenSockFD, SOL_SOCKET, SO_REUSEADDR, reinterpret_cast<const char*>(&on), sizeof(on)) < 0){
        cout << endl << "set option curLen = 0; error!!" << endl;
        return;
    }
    if(setsockopt(listenSockFD, SOL_SOCKET, SO_KEEPALIVE, reinterpret_cast<const char*>(&on), sizeof(on)) < 0){
        cout << endl << "set option curLen = 0; error!!" << endl;
        return;
    }

    // server_addr.sin_addr.s_addr = inet_addr("192.168.0.100");
    server_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(port);

    cout << "server binded" << endl;
    cout << endl << "address : " << inet_ntoa(server_addr.sin_addr) << endl;
    cout << "port : " << ntohs(server_addr.sin_port) << endl << endl;

    if(bind(listenSockFD, reinterpret_cast<struct sockaddr*>(&server_addr), sizeof(server_addr)) < 0){
        cout << endl << "bind error" << endl;
        return;
    }
}

void TcpServer::connectSocket()
{
    cout << "server running waiting. waiting client..." << endl;

    if(listen(listenSockFD, MAXCONNECTIONS) < 0){
        cout << endl << "listen error" << endl;
    }

    int clientAddrSize = sizeof(client_addr);

    curLen = 0;
    memset(bufWait, 0, MAXWAITBUFSIZE);
    ptrRecvBufIndx = bufWait;
    clientSockFD = accept(listenSockFD, reinterpret_cast<struct sockaddr*>(&client_addr), reinterpret_cast<socklen_t*>(&clientAddrSize));

    if(clientSockFD < 0){
        cout << endl << "accept error" << endl;
    }

    cout << "client accepted" << endl;
    cout << "address : " << inet_ntoa(client_addr.sin_addr) << endl;
    cout << "port : " << ntohs(client_addr.sin_port) << endl;

    connected = true;
}

void TcpServer::sendKey(const char* key){
    memset(bufSend, 0, MAXSENDBUFSIZE);
    strcpy(bufSend, key);
    len = 1;
    sendByteLen = send(clientSockFD, bufSend, static_cast<size_t>(len), 0);
//    rt_printf("sendByteLen : %d\n", sendByteLen);
    if(sendByteLen < 0){
        rt_printf("Send Error\n");
        comm_manager_run = false;
        connected = false;
    }
}
