#include "tcpserver.h"

NRMKHelper::TcpServer::TcpServer(DataControl *dataControl_) : NRMKSocketBase(), dataReceiveEvent(true)
{
    commandkey = 0;
    connected = false;
    comm_thread_run = false;
    data_corrected = false;
    dataControl = dataControl_;
    tcpServerCustom = new TcpServerCustom();
}

NRMKHelper::TcpServer::~TcpServer()
{
    if (comm_thread_run){
        delete tcpServerCustom;
        comm_thread_run = false;
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
                int ret = rt_task_create(&tcpServerCustom->comm_task, "Tcp Comm Task", 0, 50, 0);
                if (ret < 0){
                    printf("Failed to create Tcp Comm Task : %s\n", strerror(-ret));
                }
                else{
                    printf("Tcp Comm RT Task Start\n");
                    rt_task_start(&tcpServerCustom->comm_task, &tcpServerCustom->comm_run, this);
                }
            }
            else{
                printf("Tcp Comm RT Task Resume\n");
                rt_task_resume(&tcpServerCustom->comm_task);
            }

            break;

        case EVT_CONDROP:
            printf("Command Server Connection abandoned\n");

            if (comm_thread_run){
                tcpServerCustom->comm_stop();
                comm_thread_run = false;
            }

            if (dataControl->config_check){
                dataControl->config_check = false;
            }

            connected = false;
            commandkey = 0;

            StopComm();

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
                QByteArray rxData;
                rxData = QByteArray::fromRawData(reinterpret_cast<const char*>(lpBuffer), static_cast<int>(dwCount));

                int indx = NRMK_SOCKET_TOKEN_SIZE;
                dataControl->ClientToServer.opMode = static_cast<char>(lpBuffer[indx]);
                indx += OP_MODE_LEN;
                dataControl->ClientToServer.subMode = static_cast<char>(lpBuffer[indx]);
                indx += SUB_MODE_LEN;
                QByteArrayList data;
                data = rxData.split(',');
                if (dataControl->ClientToServer.opMode == DataControl::OpMode::JointMove){
                    for(int i = 1; i <= NUM_JOINT; i++){
                        dataControl->ClientToServer.desiredJoint[i - 1] = data[i].toDouble();
                    }
                    for(int i = 1; i <= NUM_DOF; i++){
                        dataControl->ClientToServer.desiredPose[i - 1] = 0;
                    }
                }
                else if(dataControl->ClientToServer.opMode == DataControl::OpMode::CartesianMove){
                    int i = 0;
                    for(i = 1; i <= NUM_DOF; i++){
                        dataControl->ClientToServer.desiredPose[i - 1] = data[i].toDouble();
                    }
                    dataControl->ClientToServer.move_time = data[i++].toDouble();
                    dataControl->ClientToServer.acc_time = data[i].toDouble();
                    for(int i = 1; i <= NUM_JOINT; i++){
                        dataControl->ClientToServer.desiredJoint[i - 1] = 0;
                    }
                }

//                printf("OpMode : %d\n", dataControl->ClientToServer.opMode);
//                printf("SubMode : %d\n", dataControl->ClientToServer.subMode);
//                printf("Desired Joint : %f, %f, %f, %f, %f, %f\n",
//                       dataControl->ClientToServer.desiredJoint[0], dataControl->ClientToServer.desiredJoint[1], dataControl->ClientToServer.desiredJoint[2],
//                        dataControl->ClientToServer.desiredJoint[3], dataControl->ClientToServer.desiredJoint[4], dataControl->ClientToServer.desiredJoint[5]);
//                printf("Desired Cartesian : %f, %f, %f, %f, %f, %f\n",
//                       dataControl->ClientToServer.desiredPose[0], dataControl->ClientToServer.desiredPose[1], dataControl->ClientToServer.desiredPose[2],
//                        dataControl->ClientToServer.desiredPose[3], dataControl->ClientToServer.desiredPose[4], dataControl->ClientToServer.desiredPose[5]);
//                printf("Move time : %f, Acc Time : %f\n", dataControl->ClientToServer.move_time, dataControl->ClientToServer.acc_time);
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
                        dataControl->PathData.acc_time.clear();
                        dataControl->PathData.point_roll.clear();
                        dataControl->PathData.point_pitch.clear();
                        dataControl->PathData.point_yaw.clear();
                        dataControl->PathData.point_theta.clear();

                        data = rxData.split(',');
                        for(int8_t i = 1; i <= dataControl->PathData.row*dataControl->PathData.col; i += dataControl->PathData.col){
                            dataControl->PathData.total_time.push_back(data[i].toDouble());
                            dataControl->PathData.point_x.push_back(data[i + 1].toDouble());
                            dataControl->PathData.point_y.push_back(data[i + 2].toDouble());
                            dataControl->PathData.point_z.push_back(data[i + 3].toDouble());
                            dataControl->PathData.point_roll.push_back(data[i + 4].toDouble());
                            dataControl->PathData.point_pitch.push_back(data[i + 5].toDouble());
                            dataControl->PathData.point_yaw.push_back(data[i + 6].toDouble());
                            dataControl->PathData.acc_time.push_back(data[i + 7].toDouble());
//                            printf("%f, %f, %f, %f, %f, %f, %f, %f\n",
//                                   data[i].toDouble(), data[i + 1].toDouble(), data[i + 2].toDouble(),
//                                    data[i + 3].toDouble(), data[i + 4].toDouble(), data[i + 5].toDouble(),
//                                    data[i + 6].toDouble(), data[i + 7].toDouble());
                        }
                        break;
                    case DataControl::CmdType::ReadyCmd:
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
                        printf("Ready Feeding Assitant Robot\n");
                        break;
                    case DataControl::CmdType::RunCmd:
                        dataControl->PathData.cycle_count_max = static_cast<char>(lpBuffer[indx]);
                        indx += CYCLE_COUNT_LEN;
                        dataControl->RobotData.run_mode = DataControl::CmdType::RunCmd;
                        dataControl->PathData.path_data_indx = 0;
                        dataControl->PathData.path_struct_indx = 0;
                        dataControl->PathData.cycle_count = 1;
                        printf("Start Feeding Assitant Robot\n");
                        break;
                    case DataControl::CmdType::StopCmd:
                        dataControl->PathData.path_data_indx = 0;
                        dataControl->PathData.path_struct_indx = 0;
                        dataControl->ClientToServer.opMode = DataControl::OpMode::Wait;
                        break;
                    case DataControl::CmdType::FileReady:
                        dataControl->PathData.cycle_count_max = static_cast<char>(lpBuffer[indx]);
                        dataControl->RobotData.run_mode = DataControl::CmdType::FileReady;
                        dataControl->PathData.path_data_indx = 0;
                        break;
                    case DataControl::CmdType::FileRun:
                        dataControl->PathData.cycle_count_max = static_cast<char>(lpBuffer[indx]);
                        dataControl->RobotData.run_mode = DataControl::CmdType::FileRun;
                        dataControl->PathData.path_data_indx = 0;
                        break;
                    case DataControl::CmdType::CustomRun:
                        dataControl->PathData.cycle_count_max = static_cast<char>(lpBuffer[indx]);
                        dataControl->RobotData.run_mode = DataControl::CmdType::CustomRun;
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
                dataControl->torqueIdeData.torque_constant = atof(buf);
                indx += TORQUE_CONST_LEN;

                printf("Mode : %d, Mass : %f, Torque Const : %f\n",
                       dataControl->ClientToServer.opMode, dataControl->torqueIdeData.mass, dataControl->torqueIdeData.torque_constant);
            }
            else if(lpBuffer[0] == 'N' && lpBuffer[1] == 'O'){
                rt_printf("%d\n", dataControl->feeding);
                if (!dataControl->feeding){
                    int indx = NRMK_SOCKET_TOKEN_SIZE;
                    dataControl->ClientToServer.opMode = static_cast<char>(lpBuffer[indx]);
                    indx += OP_MODE_LEN;
                    dataControl->operateMode.mode = static_cast<char>(lpBuffer[indx]);
                    indx += SUB_MODE_LEN;
                    dataControl->operateMode.section = static_cast<char>(lpBuffer[indx]);

                    printf("Mode : %d, Sub Mode : %d, Section : %d\n", dataControl->ClientToServer.opMode, dataControl->operateMode.mode, dataControl->operateMode.section);
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

void NRMKHelper::TcpServer::sendData()
{
    uint data_size = static_cast<uint>(dataControl->ServerToClient.size());
//    rt_printf("data size : %d\n", data_size);
    QByteArray txData;
    txData.append(NRMK_SOCKET_START_TOKEN);
    txData.append(static_cast<char>(data_size));

    txData.append('=');
    for(uint i = 0; i < data_size; i++){
//        rt_printf("time : %f\n", dataControl->ServerToClient[i].time);
        dataControl->RobotData.t += dataControl->ServerToClient[i].time;
        txData.append(QString::number(dataControl->RobotData.t, 'f', 6));
        txData.append(',');
        for(uint j = 0; j < NUM_JOINT; j++){
            txData.append(QString::number(dataControl->ServerToClient[i].presentJointPosition[j], 'f', 6));
            txData.append(',');
        }
        for(uint j = 0; j < NUM_DOF; j++){
            txData.append(QString::number(dataControl->ServerToClient[i].presentCartesianPose[j], 'f', 6));
            txData.append(',');
        }
        for(uint j = 0; j < NUM_JOINT; j++){
            txData.append(QString::number(dataControl->ServerToClient[i].desiredJointPosition[j], 'f', 6));
            txData.append(',');
        }
        for(uint j = 0; j < NUM_DOF; j++){
            txData.append(QString::number(dataControl->ServerToClient[i].desiredCartesianPose[j], 'f', 6));
            txData.append(',');
        }
        for(uint j = 0; j < NUM_DOF; j++){
            txData.append(QString::number(dataControl->ServerToClient[i].calculateCartesianPose[j], 'f', 6));
            txData.append(',');
        }
        for(uint j = 0; j < NUM_JOINT; j++){
            txData.append(QString::number(dataControl->ServerToClient[i].presentJointVelocity[j], 'f', 6));
            txData.append(',');
        }
        for(uint j = 0; j < NUM_JOINT; j++){
            txData.append(QString::number(dataControl->ServerToClient[i].presentJointCurrent[j], 'f', 6));
            txData.append(',');
        }
        for(uint j = 0; j < NUM_DOF; j++){
            txData.append(QString::number(dataControl->ServerToClient[i].presentCartesianVelocity[j], 'f', 6));
            txData.append(',');
        }
        txData.append(QString::number(dataControl->ServerToClient[i].time, 'f', 6));
        txData.append(',');
        txData.append(QString::number(dataControl->ServerToClient[i].dxl_time, 'f', 6));
        txData.append(',');
        txData.append(QString::number(dataControl->ServerToClient[i].ik_time, 'f', 6));
        txData.append('=');
    }
    txData.append(NRMK_SOCKET_END_TOKEN);

//    memcpy(_buf, NRMK_SOCKET_START_TOKEN, NRMK_SOCKET_TOKEN_SIZE); // 2 byte, START token
//    indx += NRMK_SOCKET_TOKEN_SIZE;

//    memcpy(_buf + indx, &data_size, DATA_INDEX_LEN); // 1 byte, data index
//    indx += DATA_INDEX_LEN;
//    rt_printf("total indx : %d\n", data_size);

//    for(int i = 0; i < data_size; i++){
//        memcpy(_buf + indx, &dataControl->ServerToClient[i].data_index, DATA_INDEX_LEN); // 1 byte, data index
//        indx += DATA_INDEX_LEN;
//        memcpy(_buf + indx, dataControl->ServerToClient[i].presentJointPosition, JOINT_POSITION_LEN*NUM_JOINT); // 8 byte x 6, present joint position data
//        indx += JOINT_POSITION_LEN*NUM_JOINT;
//        memcpy(_buf + indx, dataControl->ServerToClient[i].presentCartesianPose, CARTESIAN_POSE_LEN*NUM_JOINT); // 8 byte x 6, present Cartesian position data
//        indx += CARTESIAN_POSE_LEN*NUM_DOF;
//        memcpy(_buf + indx, dataControl->ServerToClient[i].desiredJointPosition, JOINT_COMMAND_LEN*NUM_JOINT); // 8 byte x 6, command joint position data
//        indx += JOINT_COMMAND_LEN*NUM_JOINT;
//        memcpy(_buf + indx, dataControl->ServerToClient[i].desiredCartesianPose, CARTESIAN_COMMAND_LEN*NUM_DOF); // 8 byte x 6, command joint Cartesian data
//        indx += CARTESIAN_COMMAND_LEN*NUM_DOF;
//        memcpy(_buf + indx, dataControl->ServerToClient[i].calculateCartesianPose, CARTESIAN_CALCULATE_LEN*NUM_DOF); // 8 byte x 6, calculate cartesian position data
//        indx += CARTESIAN_CALCULATE_LEN*NUM_DOF;
//        memcpy(_buf + indx, dataControl->ServerToClient[i].presentJointVelocity, JOINT_VELOCITY_LEN*NUM_JOINT); // 8 byte x 6, present joint velocity data
//        indx += JOINT_VELOCITY_LEN*NUM_JOINT;
//        memcpy(_buf + indx, dataControl->ServerToClient[i].presentJointCurrent, JOINT_CURRENT_LEN*NUM_JOINT); // 8 byte x 6, present joint current data
//        indx += JOINT_CURRENT_LEN*NUM_JOINT;
//        memcpy(_buf + indx, dataControl->ServerToClient[i].presentCartesianVelocity, CARTESIAN_VELOCITY_LEN*NUM_DOF); // 8 byte x 6, present joint current data
//        indx += CARTESIAN_VELOCITY_LEN*NUM_DOF;
//        memcpy(_buf + indx, &dataControl->ServerToClient[i].time, TIME_LEN); // 8 byte, Thread time
//        indx += TIME_LEN;
//        memcpy(_buf + indx, &dataControl->ServerToClient[i].dxl_time, TIME_LEN); // 8 byte, Dynamixel time
//        indx += TIME_LEN;
//        memcpy(_buf + indx, &dataControl->ServerToClient[i].ik_time, TIME_LEN); // 8 byte, Inverse Kinematics time
//        indx += TIME_LEN;
//    }
//    memcpy(_buf + indx, NRMK_SOCKET_END_TOKEN, NRMK_SOCKET_TOKEN_SIZE); // 2 byte, END token
//    indx += NRMK_SOCKET_TOKEN_SIZE;

    WriteComm(reinterpret_cast<unsigned char*>(txData.data()), static_cast<unsigned long>(txData.size()), INFINITE);

    dataControl->ServerToClient.erase(dataControl->ServerToClient.begin(), dataControl->ServerToClient.begin() + static_cast<int>(data_size));
}
