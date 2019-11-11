#include "tcpserver.h"

NRMKHelper::TcpServer::TcpServer(DataControl *dataControl_) : NRMKSocketBase (), dataReceiveEvent(true)
{
    commandkey = 0;
    connected = false;
    comm_thread_run = false;
    data_corrected = false;
    dataControl = dataControl_;
}

NRMKHelper::TcpServer::~TcpServer()
{
    if (comm_thread_run){
        rt_task_suspend(&comm_task);
        rt_task_delete(&comm_task);
    }
    if (IsOpen()){
        StopComm();
        CloseComm();
    }
}

void NRMKHelper::TcpServer::setting(QString _ip, int _port)
{
    IP = _ip;
    PORT = _port;
}

void NRMKHelper::TcpServer::OnEvent(UINT uEvent, LPVOID lpvData)
{
    switch( uEvent )
    {
        case EVT_CONSUCCESS:
            printf("One client has connected to the Command Server...\n");

            rt_print_auto_init(1);
            /* Avoids memory swapping for this program */
            mlockall(MCL_CURRENT|MCL_FUTURE);

            if (!comm_thread_run){
                int ret = rt_task_create(&comm_task, "Tcp Comm Task", 0, 50, 0);
                if (ret < 0){
                    printf("Failed to create Tcp Comm Task : %s\n", strerror(-ret));
                }
                else{
                    printf("Tcp Comm RT Task Start\n");
                    rt_task_start(&comm_task, &comm_run, this);
                }
            }
            else{
                printf("Tcp Comm RT Task Resume\n");
                rt_task_resume(&comm_task);
            }

            break;

        case EVT_CONDROP:
            printf("Command Server Connection abandoned\n");

            if (comm_thread_run){
                rt_task_suspend(&comm_task);
                printf("Tcp Comm RT Task Stop\n");
                rt_task_delete(&comm_task);
                printf("Tcp Comm RT Task Delet\n");
                comm_thread_run = false;
            }

            connected = false;
            commandkey = 0;

            StopComm();

            //        dataControl->ClientToServer.opMode = DataControl::OpMode::Initialize;

            if (IsServer())
                waitForConnection(0);

            break;

        default:
            NRMKSocketBase::OnEvent(uEvent, lpvData);
            break;
    }
}

void NRMKHelper::TcpServer::sendKey(char key)
{
    //_key = key;
    //    WriteComm((LPBYTE)&key, 1, INFINITE);
    WriteComm(reinterpret_cast<LPBYTE>(&key), 1, INFINITE);
}

void NRMKHelper::TcpServer::getKey(char &key)
{
    if (connected) {
        dataReceiveEvent.wait();
        key = commandkey;
    }
    commandkey = 0;
}

bool NRMKHelper::TcpServer::isConnected()
{
    return connected;
}

void NRMKHelper::TcpServer::OnDataReceived(const LPBYTE lpBuffer, DWORD dwCount)
{
//    printf("dwCount : %ld\n", dwCount);

//    printf("Receive data : ");
//    for(unsigned long i = 0; i < dwCount; i++){
//        printf("%d\t", lpBuffer[i]);
//    }
//    printf("\n");

    if (!connected)
    {
        connected = true;
        commandkey = static_cast<char>(lpBuffer[0]); //first byte is Key
        dataReceiveEvent.set();
        dataControl->DataReset();
        dataControl->ClientToServer.opMode = -1;

        printf("Server initialization complete\n");
        return;
    }

    if (dwCount >= 3)
    {
        if (connected)
        {
            if (lpBuffer[0] == 'N' && lpBuffer[1] == 'D'){
                dataControl->config_check = false;

                if (lpBuffer[2] == NUM_JOINT && lpBuffer[3] == NUM_DOF && lpBuffer[4] == MODULE_TYPE){
                    printf("Client & Server configuration check complete\n");
                    dataControl->RobotData.joint_op_mode = lpBuffer[5];
                    dataControl->config_check = true;
//                    if (dataControl->ClientToServer.opMode == DataControl::OpMode::Initialize){
//                        sendKey('S');
//                    }
                }
                else{
                    if (lpBuffer[2] != NUM_JOINT){
                        printf("Client & Server does not mathced number of joints\n");
                    }
                    else if (lpBuffer[3] != NUM_DOF){
                        printf("Client & Server does not matched degree of freedom\n");
                    }
                    else if (lpBuffer[4] != MODULE_TYPE){
                        printf("Client & Server does not matched module type\n");
                    }
                }

                if (!dataControl->config_check){
                    sendKey('X');
                }
            }
            else if (lpBuffer[0] == 'N' && lpBuffer[1] == 'S'){
                int indx = NRMK_SOCKET_TOKEN_SIZE;
                dataControl->ClientToServer.opMode = static_cast<char>(lpBuffer[indx]);
                indx += OP_MODE_LEN;
                dataControl->ClientToServer.subMode = static_cast<char>(lpBuffer[indx]);
                indx += SUB_MODE_LEN;
                char buf[8];
                for(int i = 0; i < NUM_JOINT; i++){
                    memcpy(buf, lpBuffer + indx, DESIRED_JOINT_LEN);
                    dataControl->ClientToServer.desiredJoint[i] = atof(buf);
                    indx += DESIRED_JOINT_LEN;
                }
                for(int i = 0; i < NUM_DOF; i++){
                    memcpy(buf, lpBuffer + indx, DESIRED_CARTESIAN_LEN);
                    dataControl->ClientToServer.desiredCartesian[i] = atof(buf);
                    indx += DESIRED_CARTESIAN_LEN;
                }
                dataControl->cartesianPoseScaleDown(dataControl->ClientToServer.desiredCartesian, dataControl->RobotData.desired_end_pose);

//                printf("OpMode : %d\n", dataControl->ClientToServer.opMode);
//                printf("SubMode : %d\n", dataControl->ClientToServer.subMode);
//                printf("Desired Joint : %f, %f, %f, %f, %f, %f\n", dataControl->ClientToServer.desiredJoint[0], dataControl->ClientToServer.desiredJoint[1], dataControl->ClientToServer.desiredJoint[2],
//                        dataControl->ClientToServer.desiredJoint[3], dataControl->ClientToServer.desiredJoint[4], dataControl->ClientToServer.desiredJoint[5]);
//                printf("Desired Cartesian : %f, %f, %f, %f, %f, %f\n", dataControl->ClientToServer.desiredCartesian[0], dataControl->ClientToServer.desiredCartesian[1], dataControl->ClientToServer.desiredCartesian[2],
//                        dataControl->ClientToServer.desiredCartesian[3], dataControl->ClientToServer.desiredCartesian[4], dataControl->ClientToServer.desiredCartesian[5]);
            }
            else if(lpBuffer[0] == 'N' && lpBuffer[1] == 'U'){
                QByteArray rxData;
                rxData = QByteArray::fromRawData(reinterpret_cast<const char*>(lpBuffer), static_cast<int>(dwCount));

                int indx = NRMK_SOCKET_TOKEN_SIZE;
                dataControl->PathData.cmd_type = static_cast<char>(lpBuffer[indx]);
                indx += CMD_TYPE_LEN;
                dataControl->ClientToServer.opMode = static_cast<char>(lpBuffer[indx]);
                indx += OP_MODE_LEN;

                QByteArrayList data;
                switch(dataControl->PathData.cmd_type){
                    case DataControl::CmdType::PathCmd:
                        dataControl->PathData.row = static_cast<uchar>(lpBuffer[indx]);
                        indx += ROW_SIZE_LEN;
                        dataControl->PathData.col = static_cast<uchar>(lpBuffer[indx]);
                        indx += COL_SIZE_LEN;
                        dataControl->PathData.total_time.clear();
                        dataControl->PathData.point_x.clear();
                        dataControl->PathData.point_y.clear();
                        dataControl->PathData.point_z.clear();
                        dataControl->PathData.path_x.clear();
                        dataControl->PathData.path_y.clear();
                        dataControl->PathData.path_z.clear();
                        dataControl->PathData.acc_time.clear();

                        data = rxData.split(',');
                        for(int8_t i = 1; i <= dataControl->PathData.row*dataControl->PathData.col; i += dataControl->PathData.col){
                            dataControl->PathData.total_time.push_back(data[i].toDouble());
                            dataControl->PathData.point_x.push_back(data[i + 1].toDouble());
                            dataControl->PathData.point_y.push_back(data[i + 2].toDouble());
                            dataControl->PathData.point_z.push_back(data[i + 3].toDouble());
                            dataControl->PathData.acc_time.push_back(data[i + 4].toDouble());
                        }

                        printf("row : %d, col : %d\n", dataControl->PathData.row, dataControl->PathData.col);
                        printf("path : \n");
                        for(uint8_t i = 0; i < dataControl->PathData.row; i++){
                            printf("time : %f, x : %f, y : %f, z : %f, acc_time : %f\n",
                                   dataControl->PathData.total_time[i],
                                   dataControl->PathData.point_x[i],
                                   dataControl->PathData.point_y[i],
                                   dataControl->PathData.point_z[i],
                                   dataControl->PathData.acc_time[i]);
                        }
                        break;
                    case DataControl::CmdType::ReadyCmd:
                        dataControl->RobotData.run_mode = 1;
                        dataControl->PathData.path_data_indx = 0;
                        printf("Ready Feeding Assitant Robot\n");
                        break;
                    case DataControl::CmdType::RunCmd:
                        dataControl->PathData.cycle_count = static_cast<char>(lpBuffer[indx]);
                        indx += CYCLE_COUNT_LEN;
                        dataControl->RobotData.run_mode = 2;
                        dataControl->PathData.path_data_indx = 0;
                        printf("Start Feeding Assitant Robot\n");
                        break;
                    case DataControl::CmdType::StopCmd:
                        dataControl->RobotData.run_mode = 0;
                        dataControl->PathData.path_data_indx = 0;
                        break;
                }
            }
            else if(lpBuffer[0] == 'N' && lpBuffer[1] == 'T'){
                int indx = NRMK_SOCKET_TOKEN_SIZE;
                dataControl->ClientToServer.opMode = static_cast<char>(lpBuffer[indx]);
                indx += OP_MODE_LEN;
                char buf[8];
                memcpy(buf, lpBuffer + indx, MASS_LEN);
                dataControl->torqueIdeData.mass = atof(buf);
                indx += MASS_LEN;
                memcpy(buf, lpBuffer + indx, TORQUE_CONST_LEN);
                dataControl->torqueIdeData.torque_constant =atof(buf);
                indx += TORQUE_CONST_LEN;

                printf("Mode : %d, Mass : %f, Torque Const : %f\n",
                       dataControl->ClientToServer.opMode, dataControl->torqueIdeData.mass, dataControl->torqueIdeData.torque_constant);
            }
            else {
                data_corrected = false;
                return;
            }
        }
    }
    else
    {
        // wrong format packet
    }
}

void NRMKHelper::TcpServer::comm_run(void *arg){
    RTIME now, previous;
    previous = rt_timer_read();
    NRMKHelper::TcpServer* pTcpServer = static_cast<NRMKHelper::TcpServer*>(arg);
    pTcpServer->comm_thread_run = false;

    rt_task_set_periodic(&pTcpServer->comm_task, TM_NOW, 100e6);

    //    printf("connected : %d\n", pTcpServer->connected);
    while(1){
        if (pTcpServer->connected){
            pTcpServer->comm_thread_run = true;
            rt_task_wait_period(nullptr); //wait for next cycle

            //        if (pTcpServer->dataControl->ClientToServer.opMode == DataControl::OpMode::Initialize){
            //            pTcpServer->sendKey('S');
            //        }
            if (pTcpServer->dataControl->ClientToServer.opMode >= 2){
                pTcpServer->sendData();
            }

            now = rt_timer_read();

            //            rt_printf("Comm : Time since last turn: %ld.%06ld ms\n",
            //                      static_cast<unsigned long>(now - previous) / 1000000,
            //                      static_cast<unsigned long>(now - previous) % 1000000);
            previous = now;
        }
    }
}

void NRMKHelper::TcpServer::sendData()
{
    unsigned char _buf[SERVER_TO_CLIENT_LEN];
    uint indx = 0;

    dataControl->jointPositionENC2DEG(dataControl->RobotData.command_joint_position, dataControl->ServerToClient.desiredJointPosition);
    dataControl->cartesianPoseScaleUp(dataControl->RobotData.desired_end_pose, dataControl->ServerToClient.desiredCartesianPose);

    memcpy(_buf, NRMK_SOCKET_START_TOKEN, NRMK_SOCKET_TOKEN_SIZE); // 2 byte, START token
    indx += NRMK_SOCKET_TOKEN_SIZE;
    memcpy(_buf + indx, &dataControl->ServerToClient.data_index, DATA_INDEX_LEN); // 1 byte, data index
    indx += DATA_INDEX_LEN;
    memcpy(_buf + indx, dataControl->ServerToClient.presentJointPosition, JOINT_POSITION_LEN*NUM_JOINT); // 8 byte x 6, present joint position data
    indx += JOINT_POSITION_LEN*NUM_JOINT;
    memcpy(_buf + indx, dataControl->ServerToClient.presentCartesianPose, CARTESIAN_POSE_LEN*NUM_JOINT); // 8 byte x 6, present Cartesian position data
    indx += CARTESIAN_POSE_LEN*NUM_DOF;
    memcpy(_buf + indx, dataControl->ServerToClient.desiredJointPosition, JOINT_COMMAND_LEN*NUM_JOINT); // 8 byte x 6, command joint position data
    indx += JOINT_COMMAND_LEN*NUM_JOINT;
    memcpy(_buf + indx, dataControl->ServerToClient.desiredCartesianPose, CARTESIAN_COMMAND_LEN*NUM_DOF); // 8 byte x 6, command joint Cartesian data
    indx += CARTESIAN_COMMAND_LEN*NUM_DOF;
    memcpy(_buf + indx, dataControl->ServerToClient.calculateCartesianPose, CARTESIAN_CALCULATE_LEN*NUM_DOF); // 8 byte x 6, calculate cartesian position data
    indx += CARTESIAN_CALCULATE_LEN*NUM_DOF;
    memcpy(_buf + indx, dataControl->ServerToClient.presentJointVelocity, JOINT_VELOCITY_LEN*NUM_JOINT); // 8 byte x 6, present joint velocity data
    indx += JOINT_VELOCITY_LEN*NUM_JOINT;
    memcpy(_buf + indx, dataControl->ServerToClient.presentJointCurrent, JOINT_CURRENT_LEN*NUM_JOINT); // 8 byte x 6, present joint current data
    indx += JOINT_CURRENT_LEN*NUM_JOINT;
    memcpy(_buf + indx, &dataControl->ServerToClient.time, TIME_LEN); // 8 byte, Thread time
    indx += TIME_LEN;
    memcpy(_buf + indx, &dataControl->ServerToClient.dxl_time, TIME_LEN); // 8 byte, Dynamixel time
    indx += TIME_LEN;
    memcpy(_buf + indx, &dataControl->ServerToClient.ik_time, TIME_LEN); // 8 byte, Inverse Kinematics time
    indx += TIME_LEN;
    memcpy(_buf + indx, NRMK_SOCKET_END_TOKEN, NRMK_SOCKET_TOKEN_SIZE); // 2 byte, END token
    indx += NRMK_SOCKET_TOKEN_SIZE;

    WriteComm(_buf, indx, INFINITE);
}
