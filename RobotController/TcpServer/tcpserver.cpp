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
                if (lpBuffer[2] != NUM_JOINT){
                    printf("Client & Server does not mathced number of joints\n");
                }
                else if (lpBuffer[3] != NUM_DOF){
                    printf("Client & Server does not matched degree of freedom\n");
                }
                else if (lpBuffer[4] != MODULE_TYPE){
                    printf("Client & Server does not matched module type\n");
                }
                else{
                    printf("Client & Server configuration check complete\n");
                    dataControl->config_check = true;
                    //                    if (dataControl->ClientToServer.opMode == DataControl::OpMode::Initialize){
                    //                        sendKey('S');
                    //                    }
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
                int indx = NRMK_SOCKET_TOKEN_SIZE;
                dataControl->PathData.type = static_cast<char>(lpBuffer[indx]);
                indx += PATH_TYPE_LEN;
                dataControl->ClientToServer.opMode = static_cast<char>(lpBuffer[indx]);
                indx += OP_MODE_LEN;
                dataControl->PathData.repeat = static_cast<char>(lpBuffer[indx]);
                indx += CYCLE_COUNT_LEN;

                //                if (dataControl->PathData.type == DataControl::PathDataType::CartPath || dataControl->PathData.type == DataControl::PathDataType::JointPath){
                //                    dataControl->PathData.data.clear();
                //                    for(int i = 0; i < dataControl->PathData.row; i++){
                //                        std::vector<double> vec_temp;
                //                        for(int j = 0; j < dataControl->PathData.col; j++){
                //                            char buf[8];
                //                            memcpy(buf, lpBuffer + indx, PATH_DATA_LEN);
                //                            vec_temp.push_back(atof(buf));
                //                            indx += PATH_DATA_LEN;
                //                        }
                //                        dataControl->PathData.data.push_back(vec_temp);
                //                    }
                //                }

                switch(dataControl->PathData.type){
                    case DataControl::PathDataType::Save1:
                    case DataControl::PathDataType::Save2:
                        dataControl->PathData.row = static_cast<uint16_t>(dataControl->PathData.pathDataPick.size());
                        dataControl->PathData.col = static_cast<uint16_t>(dataControl->PathData.pathDataPick[0].size());
                        dataControl->PathData.path_data_indx = 0;
                        break;

                    case DataControl::PathDataType::Save3:
                    case DataControl::PathDataType::Save4:
                        dataControl->PathData.row = static_cast<uint16_t>(dataControl->PathData.pathDataRect.size());
                        dataControl->PathData.col = static_cast<uint16_t>(dataControl->PathData.pathDataRect[0].size());
                        dataControl->PathData.path_data_indx = 0;
                        break;
                    case DataControl::PathDataType::Save5:
                    case DataControl::PathDataType::Save6:
                        dataControl->PathData.row = static_cast<uint16_t>(dataControl->PathData.pathDataRect2.size());
                        dataControl->PathData.col = static_cast<uint16_t>(dataControl->PathData.pathDataRect2[0].size());
                        dataControl->PathData.path_data_indx = 0;
                        break;
                    case DataControl::PathDataType::Save7:
                        dataControl->PathData.row = static_cast<uint16_t>(dataControl->PathData.pathDataCalibration.size());
                        dataControl->PathData.col = static_cast<uint16_t>(dataControl->PathData.pathDataCalibration[0].size());
                        dataControl->PathData.path_data_indx = 0;
                        break;
                    case DataControl::PathDataType::Save8:
                    case DataControl::PathDataType::Save9:
                        dataControl->PathData.row = static_cast<uint16_t>(dataControl->PathData.pathDataLinear42.size());
                        dataControl->PathData.col = static_cast<uint16_t>(dataControl->PathData.pathDataLinear42[0].size());
                        dataControl->PathData.path_data_indx = 0;
                        break;
                    case DataControl::PathDataType::Save10:
                    case DataControl::PathDataType::Save11:
                        dataControl->PathData.row = static_cast<uint16_t>(dataControl->PathData.pathDataLinear24.size());
                        dataControl->PathData.col = static_cast<uint16_t>(dataControl->PathData.pathDataLinear24[0].size());
                        dataControl->PathData.path_data_indx = 0;
                        break;
                }
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

    rt_task_set_periodic(&pTcpServer->comm_task, TM_NOW, 10e6);

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

    dataControl->ServerToClient.dxl_time = static_cast<double>((dataControl->RobotData.dxl_time2 - dataControl->RobotData.dxl_time1)/1000000.0);
    dataControl->ServerToClient.ik_time = static_cast<double>((dataControl->RobotData.ik_time2 - dataControl->RobotData.ik_time1)/1000000.0);

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
