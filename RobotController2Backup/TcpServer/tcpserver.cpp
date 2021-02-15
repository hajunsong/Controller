#include "tcpserver.h"

TcpServer::TcpServer(DataControl *_dataControl)
{
    //    printf("TcpSocket Class Constructor\n");

    dataControl = _dataControl;

    connected = false;

    comm_period = 100e6;
}

TcpServer::~TcpServer()
{
    //    printf("TcpSocket Class Destructor\n");
    stop();
}

void TcpServer::start(){
    startCommManagerTask();
    usleep(10000);
    printf("start comm\n");
}

void TcpServer::startCommManagerTask()
{
    printf("Running, TCP Comm Manager....\n");

    pthread_create(&commManagerThread, nullptr, commManagerFunc, this);
}

void *TcpServer::commManagerFunc(void *arg)
{
    TcpServer *pTcpServer = static_cast<TcpServer*>(arg);

    pTcpServer->comm_manager_run = true;

    rt_print_auto_init(1);

    /* Avoids memory swapping for this program */
    mlockall(MCL_CURRENT|MCL_FUTURE);

    pTcpServer->initSocket();

    while(pTcpServer->comm_manager_run){
        pTcpServer->connectSocket();

        int ret = rt_task_create(&pTcpServer->rtTcpCommRxTask, "Tcp Comm RX Task", 0, 96, T_JOINABLE);
        if (ret < 0){
            rt_printf("Failed to create Tcp Comm RX Task : %s\n", strerror(-ret));
        }
        else{
            rt_printf("Tcp Comm RX RT Task Start\n");
            rt_task_start(&pTcpServer->rtTcpCommRxTask, &pTcpServer->rtCommRxFunc, arg);
        }

        ret = rt_task_create(&pTcpServer->rtTcpCommTxTask, "Tcp Comm TX Task", 0, 97, T_JOINABLE);
        if (ret < 0){
            rt_printf("Failed to create Tcp Comm TX Task : %s\n", strerror(-ret));
        }
        else{
            rt_printf("Tcp Comm TX RT Task Start\n");
            rt_task_start(&pTcpServer->rtTcpCommTxTask, &pTcpServer->rtCommTxFunc, arg);
        }

        rt_task_join(&pTcpServer->rtTcpCommRxTask);
        usleep(10000);
        rt_task_join(&pTcpServer->rtTcpCommTxTask);
        usleep(10000);
        rt_task_delete(&pTcpServer->rtTcpCommRxTask);
        usleep(10000);
        rt_task_delete(&pTcpServer->rtTcpCommTxTask);
        usleep(10000);
    }
    return nullptr;
}

void TcpServer::rtCommRxFunc(void *arg)
{
    TcpServer *pTcpServer = static_cast<TcpServer*>(arg);

    rt_task_set_periodic(NULL, TM_NOW, pTcpServer->comm_period);

    pTcpServer->comm_rx_run = true;

    while(pTcpServer->comm_rx_run){
        rt_task_wait_period(NULL);

        if(pTcpServer->connected){
            memset(pTcpServer->buf, 0, MAXRECEIVEBUFSIZE);
            pTcpServer->byteLen = recv(pTcpServer->clientSockFD, pTcpServer->buf, MAXRECEIVEBUFSIZE, 0);

            rt_printf("Received byteLen : %ld\n", pTcpServer->byteLen);
            rt_printf("received data : ");
            for(uint8_t i = 0; i < 23; i++){
                rt_printf("%d ", pTcpServer->buf[i]);
            }
            rt_printf("\n");

            if(pTcpServer->byteLen == 0){
                rt_printf("Recv Disconnected\n");
                pTcpServer->connected = false;
            }
            else if(pTcpServer->byteLen > 0){
                pTcpServer->recvData();
            }
        }
        else{
            pTcpServer->comm_rx_run = false;
            break;
        }
    }

    rt_printf("Finished Comm RX Thread\n");
}

void TcpServer::rtCommTxFunc(void *arg)
{
    TcpServer *pTcpServer = static_cast<TcpServer*>(arg);

    rt_task_set_periodic(NULL, TM_NOW, pTcpServer->comm_period);

    pTcpServer->comm_tx_run = true;

    while(pTcpServer->comm_tx_run){
        rt_task_wait_period(NULL);

        if(pTcpServer->connected){
            pTcpServer->sendData();

            if(pTcpServer->sendByteLen < 0){
                rt_printf("Send Disconnected\n");
                pTcpServer->connected = false;
            }
        }
        else{
            pTcpServer->comm_tx_run = false;
            break;
        }
    }

    rt_printf("Finished Comm TX Thread\n");
}

void TcpServer::stop()
{
    rt_task_suspend(&rtTcpCommRxTask);
    usleep(10000);
    rt_task_delete(&rtTcpCommRxTask);
    usleep(10000);
    printf("Finished Comm Rx Task\n");
    rt_task_suspend(&rtTcpCommTxTask);
    usleep(10000);
    rt_task_delete(&rtTcpCommTxTask);
    usleep(10000);
    printf("Finished Comm Tx Task\n");

    pthread_cancel(commManagerThread);
    usleep(10000);
    printf("Finished Comm Manager Thread\n");
}

void TcpServer::setting(uint16_t _port){
    port = _port;
}

void TcpServer::initSocket()
{
    listenSockFD = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if(listenSockFD < 0){
        rt_printf("\n socket create error\n");
        return;
    }

    int on = 1;
    if(setsockopt(listenSockFD, SOL_SOCKET, SO_REUSEADDR, reinterpret_cast<const char*>(&on), sizeof(on)) < 0){
        rt_printf("\n set option curLen = 0; error!!\n");
        return;
    }

    struct timeval tv_recv, tv_send;
    tv_recv.tv_sec = 1;
    tv_recv.tv_usec = 0;
    tv_send.tv_sec = 0;
    tv_send.tv_usec = 500;

    //    if(setsockopt(listenSockFD, SOL_SOCKET, SO_RCVTIMEO, reinterpret_cast<struct timeval*>(&tv_recv), sizeof(struct timeval)) < 0){
    //        rt_printf("\n set option curLen = 0; error!!\n");
    //        return;
    //    }
    if(setsockopt(listenSockFD, SOL_SOCKET, SO_SNDTIMEO, reinterpret_cast<struct timeval*>(&tv_send), sizeof(struct timeval)) < 0){
        rt_printf("\n set option curLen = 0; error!!\n");
        return;
    }

    //    server_addr.sin_addr.s_addr = inet_addr("192.168.0.100");
    server_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(port);

    rt_printf("server binded\n");
    rt_printf("address : %s\n", inet_ntoa(server_addr.sin_addr));
    rt_printf("port : %d\n", ntohs(server_addr.sin_port));

    if(bind(listenSockFD, reinterpret_cast<struct sockaddr*>(&server_addr), sizeof(server_addr)) < 0){
        rt_printf("\n bind error\n");
        return;
    }
}

void TcpServer::connectSocket()
{
    rt_printf("server running waiting. waitting client...\n");

    if(listen(listenSockFD, MAXCONNECTIONS) < 0){
        rt_printf("\n listen error\n");
    }

    int clientAddrSize = sizeof(client_addr);

    curLen = 0;
    memset(bufWait, 0, MAXWAITBUFSIZE);
    ptrRecvBufIndx = bufWait;
    do{
        clientSockFD = accept(listenSockFD, reinterpret_cast<struct sockaddr*>(&client_addr), reinterpret_cast<socklen_t*>(&clientAddrSize));
        usleep(10000);

        //    if(clientSockFD < 0){
        //        rt_printf("\n accept error\n");
        //    }
    }while(clientSockFD < 0);

    rt_printf("client accepted\n");
    rt_printf("address : %s\n", inet_ntoa(client_addr.sin_addr));
    rt_printf("port : %d\n", ntohs(client_addr.sin_port));

    connected = true;
}

void TcpServer::sendKey(const char* key){
    if(port == 5050){
        memset(bufSend, 0, MAXSENDBUFSIZE);
        strcpy(bufSend, key);
        len = 1;
        sendByteLen = send(clientSockFD, bufSend, static_cast<size_t>(len), 0);

        if(sendByteLen < 0){
            rt_printf("Send Error\n");
            connected = false;
        }
    }
}

void TcpServer::sendData(){
    if(port == 5050){

    }
    else if(port == 5053){
        if(dataControl->KITECHData.camera_request && (dataControl->KITECHData.camera_request_old != dataControl->KITECHData.camera_request)){
            memset(bufSend, 0, SENDBUFSIZE);

            bufSend[0] = SOP_TX;
            bufSend[1] = 1;
            bufSend[2] = EOP;
            sendByteLen = send(clientSockFD, bufSend, 3, 0);
            rt_printf("Send to KITECH request camera\n");
        }
        dataControl->KITECHData.camera_request_old = dataControl->KITECHData.camera_request;
    }
}

void TcpServer::recvData()
{
    if(port == 5050){
        if(buf[0] == 'N' && buf[1] == 'D'){
            dataControl->config_check = false;
            dataControl->RobotData.module_init = false;

            if(buf[2] == NUM_JOINT && buf[3] == NUM_DOF && buf[4] == MODULE_TYPE){
                rt_printf("Client & Server configuration check complete\n");
                dataControl->RobotData.joint_op_mode = buf[5];
                dataControl->config_check = true;
            }
            else{
                if(buf[2] != NUM_JOINT){
                    rt_printf("Client & Server does not matched number of joints\n");
                }
                if(buf[3] != NUM_DOF){
                    rt_printf("Client & Server does not matched degree of freedom\n");
                }
                if(buf[4] != MODULE_TYPE){
                    rt_printf("Client & Server does not matched module type\n");
                }
            }

            if(!dataControl->config_check){
                sendKey("X");
            }
        }
        else if(buf[0] == 'N' && buf[1] == 'S'){
//            uint16_t indx = SOCKET_TOKEN_SIZE;

//            dataControl->ClientToServer.opMode = buf[indx];
//            indx += OP_MODE_LEN;
//            dataControl->ClientToServer.subMode = buf[indx];
//            indx += SUB_MODE_LEN;

//            if(dataControl->ClientToServer.opMode == DataControl::OpMode::JointMove){
//                char buf[8];
//                for(int i = 0; i < NUM_JOINT; i++, indx += DESIRED_JOINT_LEN){
//                    memcpy(buf, buf + indx, DESIRED_JOINT_LEN);
//                    dataControl->ClientToServer.desiredJoint[i] = atof(buf);
//                }
//                for(int i = 0; i < NUM_DOF; i++){
//                    dataControl->ClientToServer.desiredPose[i] = 0;
//                }
//            }
//            else if(dataControl->ClientToServer.opMode == DataControl::OpMode::CartesianMove){
//                char buf[8];
//                for(int i = 0; i < NUM_JOINT; i++){
//                    dataControl->ClientToServer.desiredJoint[i] = 0;
//                }
//                for(int i = 0; i < NUM_DOF; i++, indx += DESIRED_CARTESIAN_LEN){
//                    memcpy(buf, buf + indx, DESIRED_CARTESIAN_LEN);
//                    dataControl->ClientToServer.desiredPose[i] = atof(buf);
//                }
//                memcpy(buf, buf + indx, MOVE_TIME_LEN);
//                dataControl->ClientToServer.move_time = atof(buf);
//                indx += MOVE_TIME_LEN;
//                memcpy(buf, pThis->buf + indx, ACC_TIME_LEN);
//                dataControl->ClientToServer.acc_time = atof(buf);
//                indx += ACC_TIME_LEN;
//            }
        }
        else if(buf[0] == 'N' && buf[1] == 'U'){
//            uint16_t indx = SOCKET_TOKEN_SIZE;

//            dataControl->ClientToServer.opMode = static_cast<char>(buf[indx]);
//            indx += OP_MODE_LEN;
//            dataControl->PathData.cmd_type = static_cast<char>(buf[indx]);
//            indx += CMD_TYPE_LEN;

//            switch(pThis->dataControl->PathData.cmd_type){
//                case DataControl::CmdType::PathCmd:
//                {
//                    pThis->dataControl->PathData.row = static_cast<char>(pThis->buf[indx]);
//                    indx += ROW_SIZE_LEN;
//                    pThis->dataControl->PathData.col = static_cast<char>(pThis->buf[indx]);
//                    indx += COL_SIZE_LEN;
//                    pThis->dataControl->PathData.total_time.clear();
//                    pThis->dataControl->PathData.point_px.clear();
//                    pThis->dataControl->PathData.point_py.clear();
//                    pThis->dataControl->PathData.point_pz.clear();
//                    pThis->dataControl->PathData.acc_time.clear();
//                    pThis->dataControl->PathData.point_rx.clear();
//                    pThis->dataControl->PathData.point_ry.clear();
//                    pThis->dataControl->PathData.point_rz.clear();
//                    pThis->dataControl->PathData.point_theta.clear();

//                    char buf[8];
//                    double bufData = 0;
//                    for(int i = 0; i < pThis->dataControl->PathData.row; i++){
//                        memcpy(buf, pThis->buf + indx, MOVE_TIME_LEN);
//                        bufData = atof(buf);
//                        pThis->dataControl->PathData.total_time.push_back(bufData);
//                        indx += MOVE_TIME_LEN;

//                        memcpy(buf, pThis->buf + indx, PATH_DATA_LEN);
//                        bufData = atof(buf);
//                        pThis->dataControl->PathData.point_px.push_back(bufData);
//                        indx += PATH_DATA_LEN;

//                        memcpy(buf, pThis->buf + indx, PATH_DATA_LEN);
//                        bufData = atof(buf);
//                        pThis->dataControl->PathData.point_py.push_back(bufData);
//                        indx += PATH_DATA_LEN;

//                        memcpy(buf, pThis->buf + indx, PATH_DATA_LEN);
//                        bufData = atof(buf);
//                        pThis->dataControl->PathData.point_pz.push_back(bufData);
//                        indx += PATH_DATA_LEN;

//                        memcpy(buf, pThis->buf + indx, PATH_DATA_LEN);
//                        bufData = atof(buf);
//                        pThis->dataControl->PathData.point_rx.push_back(bufData);
//                        indx += PATH_DATA_LEN;

//                        memcpy(buf, pThis->buf + indx, PATH_DATA_LEN);
//                        bufData = atof(buf);
//                        pThis->dataControl->PathData.point_ry.push_back(bufData);
//                        indx += PATH_DATA_LEN;

//                        memcpy(buf, pThis->buf + indx, PATH_DATA_LEN);
//                        bufData = atof(buf);
//                        pThis->dataControl->PathData.point_rz.push_back(bufData);
//                        indx += PATH_DATA_LEN;

//                        memcpy(buf, pThis->buf + indx, ACC_TIME_LEN);
//                        bufData = atof(buf);
//                        pThis->dataControl->PathData.acc_time.push_back(bufData);
//                        indx += ACC_TIME_LEN;
//                    }
//                    rt_printf("received path data : \n");
//                    for(int i = 0; i < pThis->dataControl->PathData.row; i++){
//                        rt_printf("%f %f %f %f %f %f %f %f\n",
//                                  pThis->dataControl->PathData.total_time[i], pThis->dataControl->PathData.point_px[i],
//                                  pThis->dataControl->PathData.point_py[i], pThis->dataControl->PathData.point_pz[i],
//                                  pThis->dataControl->PathData.point_rx[i], pThis->dataControl->PathData.point_ry[i],
//                                  pThis->dataControl->PathData.point_rz[i], pThis->dataControl->PathData.acc_time[i]);
//                    }

//                    break;
//                }
//                case DataControl::CmdType::ReadyCmd:
//                {
//                    pThis->dataControl->RobotData.run_mode = DataControl::CmdType::ReadyCmd;
//                    pThis->dataControl->PathData.path_data_indx = 0;
//                    pThis->dataControl->PathData.path_struct_indx = 0;
//                    for(uint i = 0; i < pThis->dataControl->PathData.row; i++){
//                        pThis->dataControl->PathData.movePath[i].path_x.clear();
//                        pThis->dataControl->PathData.movePath[i].path_y.clear();
//                        pThis->dataControl->PathData.movePath[i].path_z.clear();
//                        pThis->dataControl->PathData.movePath[i].path_theta.clear();
//                    }
//                    pThis->dataControl->PathData.readyPath.path_x.clear();
//                    pThis->dataControl->PathData.readyPath.path_y.clear();
//                    pThis->dataControl->PathData.readyPath.path_z.clear();
//                    pThis->dataControl->PathData.readyPath.path_theta.clear();
//                    rt_printf("Ready Feeding Assitant Robot\n");
//                    break;
//                }
//                case DataControl::CmdType::RunCmd:
//                {
//                    pThis->dataControl->PathData.cycle_count_max = static_cast<char>(pThis->buf[indx]);
//                    indx += CYCLE_COUNT_LEN;
//                    pThis->dataControl->RobotData.run_mode = DataControl::CmdType::RunCmd;
//                    pThis->dataControl->PathData.path_data_indx = 0;
//                    pThis->dataControl->PathData.path_struct_indx = 0;
//                    pThis->dataControl->PathData.cycle_count = 1;
//                    rt_printf("Start Feeding Assitant Robot\n");
//                    break;
//                }
//                case DataControl::CmdType::StopCmd:
//                {
//                    pThis->dataControl->PathData.path_data_indx = 0;
//                    pThis->dataControl->PathData.path_struct_indx = 0;
//                    pThis->dataControl->ClientToServer.opMode = DataControl::OpMode::Wait;
//                    break;
//                }
//                case DataControl::CmdType::FileReady:
//                {
//                    pThis->dataControl->PathData.cycle_count_max = static_cast<char>(pThis->buf[indx]);
//                    pThis->dataControl->RobotData.run_mode = DataControl::CmdType::FileReady;
//                    pThis->dataControl->PathData.path_data_indx = 0;
//                    break;
//                }
//                case DataControl::CmdType::FileRun:
//                {
//                    pThis->dataControl->PathData.cycle_count_max = static_cast<char>(pThis->buf[indx]);
//                    pThis->dataControl->RobotData.run_mode = DataControl::CmdType::FileRun;
//                    pThis->dataControl->PathData.path_data_indx = 0;
//                    break;
//                }
//                case DataControl::CmdType::CustomRun:
//                {
//                    pThis->dataControl->PathData.cycle_count_max = static_cast<char>(pThis->buf[indx]);
//                    pThis->dataControl->RobotData.run_mode = DataControl::CmdType::CustomRun;
//                    pThis->dataControl->PathData.path_data_indx = 0;
//                    break;
//                }
//            }
        }
        else if(buf[0] == 'N' && buf[1] == 'O'){
//            rt_printf("%d\n", pThis->dataControl->feeding);
//            if (!pThis->dataControl->feeding){
//                int indx = SOCKET_TOKEN_SIZE;
//                pThis->dataControl->ClientToServer.opMode = pThis->buf[indx];
//                indx += OP_MODE_LEN;
//                pThis->dataControl->operateMode.mode = pThis->buf[indx];
//                indx += SUB_MODE_LEN;
//                pThis->dataControl->operateMode.section = pThis->buf[indx];
//                indx += SECTION_LEN;

//                printf("Mode : %d, Sub Mode : %d, Section : %d\n", pThis->dataControl->ClientToServer.opMode,
//                       pThis->dataControl->operateMode.mode, pThis->dataControl->operateMode.section);
//            }
        }
    }
    else if(port ==  5053){
        rt_printf("Received byteLen : %ld\n", byteLen);
        rt_printf("received data : ");
        for(uint8_t i = 0; i < 23; i++){
            rt_printf("%d ", buf[i]);
        }
        rt_printf("\n");

        if(buf[0] == SOP_RX){
            dataControl->KITECHData.camera_request = false;

            switch(buf[1]){
                case CMD_DATA:
                {
                    rt_printf("Received Data Packet\n");

                    for(int i = 0; i < 20; i++){
                        dataControl->KITECHData.food_pixel[i] = (char)buf[i + 2];
                    }

                    for(int i = 0, j = 0; i < 10; i++, j+=2){
                        dataControl->KITECHData.food_pos[i] = (int)dataControl->KITECHData.food_pixel[j]*100 + (int)dataControl->KITECHData.food_pixel[j+1];
                    }

                    rt_printf("%d, %d\n", dataControl->KITECHData.food_pos[0], dataControl->KITECHData.food_pos[1]);
                    rt_printf("%d, %d\n", dataControl->KITECHData.food_pos[2], dataControl->KITECHData.food_pos[3]);
                    rt_printf("%d, %d\n", dataControl->KITECHData.food_pos[4], dataControl->KITECHData.food_pos[5]);
                    rt_printf("%d, %d\n", dataControl->KITECHData.food_pos[6], dataControl->KITECHData.food_pos[7]);
                    rt_printf("%d, %d\n", dataControl->KITECHData.food_pos[8], dataControl->KITECHData.food_pos[9]);
                    break;
                }
                case CMD_SECTION:
                {
                    rt_printf("Received Section Packet\n");
                    break;
                }
                case CMD_FEEDING:
                {
                    rt_printf("Received Feeding Packet\n");
                    break;
                }
                case CMD_TABLET_CHECK:
                {
                    rt_printf("Tablet Connect : %s\n", buf[2] == 1 ? "true" : "false");
                    if(buf[2] == 1){
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
    }
}

int TcpServer::in_area(const double section[4], double *p, int div_count)
{
    int indx = -1;
    if(div_count == 4){
        double x_c = (section[0] + section[2])/2.0;
        double y_c = (section[1] + section[3])/2.0;

        if(p[0] >= section[0] && p[0] <= x_c && p[1] <= section[1] && p[1] >= y_c)
        {
            indx = 0;
            rt_printf("top left, indx : %d\n", indx);
        }
        else if(p[0] <= section[2] && p[0] >= x_c && p[1] <= section[1] && p[1] >= y_c)
        {
            indx = 1;
            rt_printf("top right, indx : %d\n", indx);
        }

        else if(p[0] >= section[0] && p[0] <= x_c && p[1] >= section[3] && p[1] <= y_c)
        {
            indx = 2;
            rt_printf("down left, indx : %d\n", indx);
        }
        else if(p[0] <= section[2] && p[0] >= x_c && p[1] >= section[3] && p[1] <= y_c)
        {
            indx = 3;
            rt_printf("down right, indx : %d\n", indx);
        }
        return indx;
    }
    else if(div_count == 9){
        double x_1 = section[0] + (section[2] - section[0]) / 3.0;
        double x_2 = section[0] + (section[2] - section[0]) / 3.0*2.0;
        double y_1 = section[1] - (section[1] - section[3]) / 3.0;
        double y_2 = section[1] - (section[1] - section[3]) / 3.0*2.0;

        //        rt_printf("p1 : %f, p2 : %f\n", p[0], p[1]);

        //        rt_printf("section : %f, %f, %f, %f\n", section[0], section[1], section[2], section[3]);

        //        rt_printf("x_1 : %f\n", x_1);
        //        rt_printf("x_2 : %f\n", x_2);
        //        rt_printf("y_1 : %f\n", y_1);
        //        rt_printf("y_2 : %f\n", y_2);

        if(p[0] >= section[0] && p[0] <= x_1 && p[1] <= section[1] && p[1] >= y_1){
            indx = 0;
            rt_printf("top lef, indx : %d\n", indx);
        }
        else if(p[0] >= x_1 && p[0] <= x_2 && p[1] <= section[1] && p[1] >= y_1){
            indx = 1;
            rt_printf("top center, indx : %d\n", indx);
        }
        else if(p[0] >= x_2 && p[0] <= section[2] && p[1] <= section[1] && p[1] >= y_1){
            indx = 2;
            rt_printf("top right, indx : %d\n", indx);
        }

        else if(p[0] >= section[0] && p[0] <= x_1 && p[1] <= y_1 && p[1] >= y_2){
            indx = 3;
            rt_printf("middle lef, indx : %d\n", indx);
        }
        else if(p[0] >= x_1 && p[0] <= x_2 && p[1] <= y_1 && p[1] >= y_2){
            indx = 4;
            rt_printf("middle center, indx : %d\n", indx);
        }
        else if(p[0] >= x_2 && p[0] <= section[2] && p[1] <= y_1 && p[1] >= y_2){
            indx = 5;
            rt_printf("middle right, indx : %d\n", indx);
        }

        else if(p[0] >= section[0] && p[0] <= x_1 && p[1] <= y_2 && p[1] >= section[3]){
            indx = 6;
            rt_printf("bottom lef, indx : %d\n", indx);
        }
        else if(p[0] >= x_1 && p[0] <= x_2 && p[1] <= y_2 && p[1] >= section[3]){
            indx = 7;
            rt_printf("bottom center, indx : %d\n", indx);
        }
        else if(p[0] >= x_2 && p[0] <= section[2] && p[1] <= y_2 && p[1] >= section[3]){
            indx = 8;
            rt_printf("bottom right, indx : %d\n", indx);
        }
        return indx;
    }
    else{
        return indx;
    }
}
