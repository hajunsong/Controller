#include "tcpserver.h"

TcpServer::TcpServer(DataControl *_dataControl)
{
    dataControl = _dataControl;

    connected = false;
    comm_thread_run = false;
    comm_run = false;
    tcpServerCustom = new TcpServerCustom();

    temp = 0;
}

TcpServer::~TcpServer(){
    comm_run = false;

    close(clientSockFD);
    usleep(10000);
    printf("Shutdown socket\n");

    pthread_cancel(comm_thread);
    usleep(10000);
    printf("finished tcp comm pthread\n");

    if (comm_thread_run){
        delete tcpServerCustom;
        printf("Complete destructure ControlMainCustom\n");
        comm_thread_run = false;
    }
}

void TcpServer::start(){
    pthread_create(&comm_thread, nullptr, comm_func, this);
}

void TcpServer::stop(){
    if (comm_thread_run){
        delete tcpServerCustom;
        printf("Complete destructure ControlMainCustom\n");
        comm_thread_run = false;
    }

    comm_thread_run = false;
    pthread_join(comm_thread, nullptr);
    usleep(10000);
}

void TcpServer::setting(uint16_t _port)
{
    port = _port;
}


void *TcpServer::comm_func(void *arg)
{
    TcpServer *pThis = static_cast<TcpServer*>(arg);
    pthread_setcanceltype(PTHREAD_CANCEL_ENABLE, nullptr);
    pThis->comm_run = true;

    pThis->initSocket();

    while(pThis->comm_run)
    {
        pThis->connectSocket();

        rt_print_auto_init(1);

        /* Avoids memory swapping for this program */
        mlockall(MCL_CURRENT|MCL_FUTURE);

        int ret = rt_task_create(&pThis->tcpServerCustom->comm_task, "Tcp Comm Task", 0, 50, T_JOINABLE);
        if (ret < 0){
            printf("Failed to create Tcp Comm Task : %s\n", strerror(-ret));
        }
        else{
            printf("Tcp Comm RT Task Start\n");
            rt_task_start(&pThis->tcpServerCustom->comm_task, &pThis->tcpServerCustom->comm_run, pThis);

            pthread_create(&pThis->comm_rx_thread, nullptr, comm_rx_func, pThis);
        }


        rt_task_join(&pThis->tcpServerCustom->comm_task);
        usleep(10000);
        rt_task_suspend(&pThis->tcpServerCustom->comm_task);
        usleep(10000);
        rt_task_delete(&pThis->tcpServerCustom->comm_task);
        usleep(10000);

        pthread_join(pThis->comm_rx_thread, nullptr);
        usleep(10000);
    }

    printf("finish comm func\n");

    pthread_cancel(pThis->comm_rx_thread);

    return nullptr;
}

void *TcpServer::comm_rx_func(void *arg)
{
    TcpServer *pThis = static_cast<TcpServer*>(arg);

    pthread_setcanceltype(PTHREAD_CANCEL_ENABLE, nullptr);

    while(pThis->comm_thread_run){
        if(pThis->isConnected()){
            memset(pThis->buf, 0, MAXRECEIVEBUFSIZE);
            pThis->byteLen = recv(pThis->clientSockFD, pThis->buf, MAXRECEIVEBUFSIZE, 0);

//            rt_printf("%d\n", pThis->byteLen);
//            if(pThis->byteLen == 0){
//                pThis->connected = false;
//                pThis->comm_thread_run = false;
//                break;
//            }

            if(pThis->byteLen > 0){
                if(pThis->port ==  5053){
                    rt_printf("Received byteLen : %ld\n", pThis->byteLen);
                    rt_printf("received data : ");
                    for(uint8_t i = 0; i < 23; i++){
                        rt_printf("%d ", pThis->buf[i]);
                    }
                    rt_printf("\n");
                }
            }
//            if(pThis->buf[0] != 0){
//                rt_printf("Received byteLen : %ld\n", pThis->byteLen);
//                rt_printf("received data : ");
//                for(uint8_t i = 0; i < 23; i++){
//                    rt_printf("%d ", pThis->buf[i]);
//                }
//                rt_printf("\n");
//            }


            if(/*(pThis->byteLen == 23 || pThis->byteLen == 4) && */pThis->buf[0] == SOP_RX/* && !pThis->dataControl->feeding*//* && (pThis->buf[3] == EOP || pThis->buf[22] == EOP)*/){
                pThis->dataControl->KITECHData.camera_request = false;

                switch(pThis->buf[1]){
                    case CMD_DATA:
                    {
                        rt_printf("Received Data Packet\n");

                        for(int i = 0; i < 20; i++){
                            pThis->dataControl->KITECHData.food_pixel[i] = (char)pThis->buf[i + 2];
                        }

                        for(int i = 0, j = 0; i < 10; i++, j+=2){
                            pThis->dataControl->KITECHData.food_pos[i] = (int)pThis->dataControl->KITECHData.food_pixel[j]*100 + (int)pThis->dataControl->KITECHData.food_pixel[j+1];
                        }

                        rt_printf("%d, %d\n", pThis->dataControl->KITECHData.food_pos[0], pThis->dataControl->KITECHData.food_pos[1]);
                        rt_printf("%d, %d\n", pThis->dataControl->KITECHData.food_pos[2], pThis->dataControl->KITECHData.food_pos[3]);
                        rt_printf("%d, %d\n", pThis->dataControl->KITECHData.food_pos[4], pThis->dataControl->KITECHData.food_pos[5]);
                        rt_printf("%d, %d\n", pThis->dataControl->KITECHData.food_pos[6], pThis->dataControl->KITECHData.food_pos[7]);
                        rt_printf("%d, %d\n", pThis->dataControl->KITECHData.food_pos[8], pThis->dataControl->KITECHData.food_pos[9]);
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
//                        break;
                        rt_printf("Section : %d\n", pThis->buf[2]);
//                        pThis->dataControl->ClientToServer.opMode = DataControl::OpMode::OperateMode;
                        pThis->dataControl->obi_section_indx = pThis->buf[2] - 1;
//                        pThis->dataControl->operateMode.mode = DataControl::Operate::ReadyFeeding;
//                        while(pThis->dataControl->ClientToServer.opMode != DataControl::OpMode::Wait){
//                            usleep(1000);
//                        }
                        pThis->dataControl->ClientToServer.opMode = DataControl::OpMode::OperateMode;
                        pThis->dataControl->operateMode.mode = DataControl::Operate::Feeding;
                        switch(pThis->dataControl->obi_section_indx){
                            case 0: // side 1
                            {
                                rt_printf("section1 count : %d, %d\n", pThis->dataControl->trayInfor.section1, pThis->dataControl->trayInfor.section1%2);
                                pThis->dataControl->side1_motion[pThis->dataControl->trayInfor.section1%2].file_data_indx = 500;
                                pThis->dataControl->operateMode.section = DataControl::Section::Side1;
                                break;
                            }
                            case 1: // side 2
                            {
                                rt_printf("section2 count : %d, %d\n", pThis->dataControl->trayInfor.section2, pThis->dataControl->trayInfor.section2%2);
                                pThis->dataControl->side2_motion[pThis->dataControl->trayInfor.section2%2].file_data_indx = 500;
                                pThis->dataControl->operateMode.section = DataControl::Section::Side2;
                                break;
                            }
                            case 2: // side 3
                            {
                                rt_printf("section3 count : %d, %d\n", pThis->dataControl->trayInfor.section3, pThis->dataControl->trayInfor.section3%2);
                                pThis->dataControl->side3_motion[pThis->dataControl->trayInfor.section3%2].file_data_indx = 500;
                                pThis->dataControl->operateMode.section = DataControl::Section::Side3;
                                break;
                            }
                            case 3: // soup
                            {
                                rt_printf("section4 count : %d\n", pThis->dataControl->trayInfor.section4);
                                pThis->dataControl->soup_motion.file_data_indx = 500;
                                pThis->dataControl->operateMode.section = DataControl::Section::Soup;
                                break;
                            }
                            case 4: // rice
                            {
                                rt_printf("section5 count : %d, %d\n", pThis->dataControl->trayInfor.section5, pThis->dataControl->trayInfor.section5%3);
                                pThis->dataControl->rice_motion[pThis->dataControl->trayInfor.section5%3].file_data_indx = 500;
                                pThis->dataControl->operateMode.section = DataControl::Section::Rice;
                                break;
                            }
                        }
//                        int sub_pos = -1;
//                        switch(pThis->buf[2]){
//                            case 1:{
////                                sub_pos = pThis->in_area(pThis->dataControl->section1_area, (double*)pThis->dataControl->foodPosData.pos, 4);
//                                break;
//                            }
//                            case 2:{
////                                sub_pos = pThis->in_area(pThis->dataControl->section2_area, (double*)pThis->dataControl->foodPosData.pos + 2, 4);
//                                break;
//                            }
//                            case 3:{
////                                sub_pos = pThis->in_area(pThis->dataControl->section3_area, (double*)pThis->dataControl->foodPosData.pos + 4, 4);
//                                break;
//                            }
//                            case 4:{
////                                sub_pos = pThis->in_area(pThis->dataControl->section4_area, (double*)pThis->dataControl->foodPosData.pos + 6, 9);
//                                break;
//                            }
//                            case 5:{
////                                sub_pos = pThis->in_area(pThis->dataControl->section5_area, (double*)pThis->dataControl->foodPosData.pos + 8, 9);
//                                break;
//                            }
//                            default:{
//                                break;
//                            }
//                        }
                        break;
                    }
                    case CMD_TABLET_CHECK:
                    {
//                        rt_printf("Tablet Connect : %s\n", pThis->buf[2] == 1 ? "true" : "false");
                        if(pThis->buf[2] == 1){
                            pThis->dataControl->KITECHData.tablet_connect = true;
                        }
                        else{
                            pThis->dataControl->KITECHData.tablet_connect = false;
                        }
                    }
                    default:
                    {
                        break;
                    }
                }
            }

            if(pThis->buf[0] == 'N' && pThis->buf[1] == 'D'){
                pThis->dataControl->config_check = false;
                pThis->dataControl->RobotData.module_init = false;

                if(pThis->buf[2] == NUM_JOINT && pThis->buf[3] == NUM_DOF && pThis->buf[4] == MODULE_TYPE){
                    rt_printf("Client & Server configuration check complete\n");
                    pThis->dataControl->RobotData.joint_op_mode = pThis->buf[5];
                    pThis->dataControl->config_check = true;
                }
                else{
                    if(pThis->buf[2] != NUM_JOINT){
                        rt_printf("Client & Server does not matched number of joints\n");
                    }
                    if(pThis->buf[3] != NUM_DOF){
                        rt_printf("Client & Server does not matched degree of freedom\n");
                    }
                    if(pThis->buf[4] != MODULE_TYPE){
                        rt_printf("Client & Server does not matched module type\n");
                    }
                }

                if(!pThis->dataControl->config_check){
//                    pThis->sendKey("X");
                }
            }
            else if(pThis->buf[0] == 'N' && pThis->buf[1] == 'S'){
                uint16_t indx = SOCKET_TOKEN_SIZE;

                pThis->dataControl->ClientToServer.opMode = pThis->buf[indx];
                indx += OP_MODE_LEN;
                pThis->dataControl->ClientToServer.subMode = pThis->buf[indx];
                indx += SUB_MODE_LEN;

                if(pThis->dataControl->ClientToServer.opMode == DataControl::OpMode::JointMove){
                    char buf[8];
                    for(int i = 0; i < NUM_JOINT; i++, indx += DESIRED_JOINT_LEN){
                        memcpy(buf, pThis->buf + indx, DESIRED_JOINT_LEN);
                        pThis->dataControl->ClientToServer.desiredJoint[i] = atof(buf);
                    }
                    for(int i = 0; i < NUM_DOF; i++){
                        pThis->dataControl->ClientToServer.desiredPose[i] = 0;
                    }
                }
                else if(pThis->dataControl->ClientToServer.opMode == DataControl::OpMode::CartesianMove){
                    char buf[8];
                    for(int i = 0; i < NUM_JOINT; i++){
                        pThis->dataControl->ClientToServer.desiredJoint[i] = 0;
                    }
                    for(int i = 0; i < NUM_DOF; i++, indx += DESIRED_CARTESIAN_LEN){
                        memcpy(buf, pThis->buf + indx, DESIRED_CARTESIAN_LEN);
                        pThis->dataControl->ClientToServer.desiredPose[i] = atof(buf);
                    }
                    memcpy(buf, pThis->buf + indx, MOVE_TIME_LEN);
                    pThis->dataControl->ClientToServer.move_time = atof(buf);
                    indx += MOVE_TIME_LEN;
                    memcpy(buf, pThis->buf + indx, ACC_TIME_LEN);
                    pThis->dataControl->ClientToServer.acc_time = atof(buf);
                    indx += ACC_TIME_LEN;
                }
            }
            else if(pThis->buf[0] == 'N' && pThis->buf[1] == 'U'){
                uint16_t indx = SOCKET_TOKEN_SIZE;

                pThis->dataControl->ClientToServer.opMode = static_cast<char>(pThis->buf[indx]);
                indx += OP_MODE_LEN;
                pThis->dataControl->PathData.cmd_type = static_cast<char>(pThis->buf[indx]);
                indx += CMD_TYPE_LEN;

                switch(pThis->dataControl->PathData.cmd_type){
                    case DataControl::CmdType::PathCmd:
                    {
                        pThis->dataControl->PathData.row = static_cast<char>(pThis->buf[indx]);
                        indx += ROW_SIZE_LEN;
                        pThis->dataControl->PathData.col = static_cast<char>(pThis->buf[indx]);
                        indx += COL_SIZE_LEN;
                        pThis->dataControl->PathData.total_time.clear();
                        pThis->dataControl->PathData.point_px.clear();
                        pThis->dataControl->PathData.point_py.clear();
                        pThis->dataControl->PathData.point_pz.clear();
                        pThis->dataControl->PathData.acc_time.clear();
                        pThis->dataControl->PathData.point_rx.clear();
                        pThis->dataControl->PathData.point_ry.clear();
                        pThis->dataControl->PathData.point_rz.clear();
                        pThis->dataControl->PathData.point_theta.clear();

                        char buf[8];
                        double bufData = 0;
                        for(int i = 0; i < pThis->dataControl->PathData.row; i++){
                            memcpy(buf, pThis->buf + indx, MOVE_TIME_LEN);
                            bufData = atof(buf);
                            pThis->dataControl->PathData.total_time.push_back(bufData);
                            indx += MOVE_TIME_LEN;

                            memcpy(buf, pThis->buf + indx, PATH_DATA_LEN);
                            bufData = atof(buf);
                            pThis->dataControl->PathData.point_px.push_back(bufData);
                            indx += PATH_DATA_LEN;

                            memcpy(buf, pThis->buf + indx, PATH_DATA_LEN);
                            bufData = atof(buf);
                            pThis->dataControl->PathData.point_py.push_back(bufData);
                            indx += PATH_DATA_LEN;

                            memcpy(buf, pThis->buf + indx, PATH_DATA_LEN);
                            bufData = atof(buf);
                            pThis->dataControl->PathData.point_pz.push_back(bufData);
                            indx += PATH_DATA_LEN;

                            memcpy(buf, pThis->buf + indx, PATH_DATA_LEN);
                            bufData = atof(buf);
                            pThis->dataControl->PathData.point_rx.push_back(bufData);
                            indx += PATH_DATA_LEN;

                            memcpy(buf, pThis->buf + indx, PATH_DATA_LEN);
                            bufData = atof(buf);
                            pThis->dataControl->PathData.point_ry.push_back(bufData);
                            indx += PATH_DATA_LEN;

                            memcpy(buf, pThis->buf + indx, PATH_DATA_LEN);
                            bufData = atof(buf);
                            pThis->dataControl->PathData.point_rz.push_back(bufData);
                            indx += PATH_DATA_LEN;

                            memcpy(buf, pThis->buf + indx, ACC_TIME_LEN);
                            bufData = atof(buf);
                            pThis->dataControl->PathData.acc_time.push_back(bufData);
                            indx += ACC_TIME_LEN;
                        }
                        rt_printf("received path data : \n");
                        for(int i = 0; i < pThis->dataControl->PathData.row; i++){
                            rt_printf("%f %f %f %f %f %f %f %f\n",
                                   pThis->dataControl->PathData.total_time[i], pThis->dataControl->PathData.point_px[i],
                                   pThis->dataControl->PathData.point_py[i], pThis->dataControl->PathData.point_pz[i],
                                   pThis->dataControl->PathData.point_rx[i], pThis->dataControl->PathData.point_ry[i],
                                   pThis->dataControl->PathData.point_rz[i], pThis->dataControl->PathData.acc_time[i]);
                        }

                        break;
                    }
                    case DataControl::CmdType::ReadyCmd:
                    {
                        pThis->dataControl->RobotData.run_mode = DataControl::CmdType::ReadyCmd;
                        pThis->dataControl->PathData.path_data_indx = 0;
                        pThis->dataControl->PathData.path_struct_indx = 0;
                        for(uint i = 0; i < pThis->dataControl->PathData.row; i++){
                            pThis->dataControl->PathData.movePath[i].path_x.clear();
                            pThis->dataControl->PathData.movePath[i].path_y.clear();
                            pThis->dataControl->PathData.movePath[i].path_z.clear();
                            pThis->dataControl->PathData.movePath[i].path_theta.clear();
                        }
                        pThis->dataControl->PathData.readyPath.path_x.clear();
                        pThis->dataControl->PathData.readyPath.path_y.clear();
                        pThis->dataControl->PathData.readyPath.path_z.clear();
                        pThis->dataControl->PathData.readyPath.path_theta.clear();
                        rt_printf("Ready Feeding Assitant Robot\n");
                        break;
                    }
                    case DataControl::CmdType::RunCmd:
                    {
                        pThis->dataControl->PathData.cycle_count_max = static_cast<char>(pThis->buf[indx]);
                        indx += CYCLE_COUNT_LEN;
                        pThis->dataControl->RobotData.run_mode = DataControl::CmdType::RunCmd;
                        pThis->dataControl->PathData.path_data_indx = 0;
                        pThis->dataControl->PathData.path_struct_indx = 0;
                        pThis->dataControl->PathData.cycle_count = 1;
                        rt_printf("Start Feeding Assitant Robot\n");
                        break;
                    }
                    case DataControl::CmdType::StopCmd:
                    {
                        pThis->dataControl->PathData.path_data_indx = 0;
                        pThis->dataControl->PathData.path_struct_indx = 0;
                        pThis->dataControl->ClientToServer.opMode = DataControl::OpMode::Wait;
                        break;
                    }
                    case DataControl::CmdType::FileReady:
                    {
                        pThis->dataControl->PathData.cycle_count_max = static_cast<char>(pThis->buf[indx]);
                        pThis->dataControl->RobotData.run_mode = DataControl::CmdType::FileReady;
                        pThis->dataControl->PathData.path_data_indx = 0;
                        break;
                    }
                    case DataControl::CmdType::FileRun:
                    {
                        pThis->dataControl->PathData.cycle_count_max = static_cast<char>(pThis->buf[indx]);
                        pThis->dataControl->RobotData.run_mode = DataControl::CmdType::FileRun;
                        pThis->dataControl->PathData.path_data_indx = 0;
                        break;
                    }
                    case DataControl::CmdType::CustomRun:
                    {
                        pThis->dataControl->PathData.cycle_count_max = static_cast<char>(pThis->buf[indx]);
                        pThis->dataControl->RobotData.run_mode = DataControl::CmdType::CustomRun;
                        pThis->dataControl->PathData.path_data_indx = 0;
                        break;
                    }
                }
            }
            else if(pThis->buf[0] == 'N' && pThis->buf[1] == 'O'){
                rt_printf("%d\n", pThis->dataControl->feeding);
                if (!pThis->dataControl->feeding){
                    int indx = SOCKET_TOKEN_SIZE;
                    pThis->dataControl->ClientToServer.opMode = pThis->buf[indx];
                    indx += OP_MODE_LEN;
                    pThis->dataControl->operateMode.mode = pThis->buf[indx];
                    indx += SUB_MODE_LEN;
                    pThis->dataControl->operateMode.section = pThis->buf[indx];
                    indx += SECTION_LEN;

                    printf("Mode : %d, Sub Mode : %d, Section : %d\n", pThis->dataControl->ClientToServer.opMode,
                           pThis->dataControl->operateMode.mode, pThis->dataControl->operateMode.section);
                }
            }
        }
    }
    return nullptr;
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

    server_addr.sin_addr.s_addr = inet_addr("192.168.0.100");
//    server_addr.sin_addr.s_addr = htonl(INADDR_ANY);
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

    if(sendByteLen < 0){
        rt_printf("Send Error\n");
        comm_thread_run = false;
        connected = false;
    }
}

int TcpServer::sendData(){
    if(comm_thread_run){
//        rt_printf("vector size : %d\n", dataControl->ServerToClient.size());
        if(port == 5050){
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
                        comm_thread_run = false;
                        connected = false;
                        dataControl->config_check = false;
                        dataControl->ServerToClient.clear();

                        return -1;
                    }
                }
            }
        }
        else if(port == 5053){
//            if(dataControl->KITECHData.camera_request && (dataControl->KITECHData.camera_request_old != dataControl->KITECHData.camera_request)){
//                memset(bufSend, 0, SENDBUFSIZE);

//                bufSend[0] = SOP_TX;
//                bufSend[1] = 1;
//                bufSend[2] = EOP;
//                sendByteLen = send(clientSockFD, bufSend, 3, 0);
//                rt_printf("Send to KITECH request camera\n");

//                if(sendByteLen < 0){
//                    rt_printf("Send Error\n");
//                    comm_thread_run = false;
//                    connected = false;
////                    dataControl->config_check = false;
////                    dataControl->ServerToClient.clear();

//                    return -1;
//                }
//            }
//            dataControl->KITECHData.camera_request_old = dataControl->KITECHData.camera_request;

            if(dataControl->KITECHData.tablet_check && (dataControl->KITECHData.tablet_check_old != dataControl->KITECHData.tablet_check)){
                memset(bufSend, 0, SENDBUFSIZE);

                bufSend[0] = SOP_TX;
                bufSend[1] = 2;
                bufSend[2] = EOP;
                sendByteLen = send(clientSockFD, bufSend, 3, 0);
                rt_printf("Send to KITECH tablet connect check\n");

                if(sendByteLen < 0){
                    rt_printf("Send Error\n");
                    comm_thread_run = false;
                    connected = false;

                    return -1;
                }
            }
            dataControl->KITECHData.tablet_check_old = dataControl->KITECHData.tablet_check;
        }
    }

    return 0;
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
