#include "tcpclient.h"
#include "MainWindow/mainwindow.h"

TcpClient::TcpClient(DataControl *_dataControl, void* _mainWindow)
{
    dataControl = _dataControl;

    server_connected = false;
    comm_thread_run = false;

    mainWindow = static_cast<MainWindow*>(_mainWindow);
}

TcpClient::~TcpClient(){
    stop();
    usleep(10000);
}

void TcpClient::start(){
    initSocket();
    connectSocket();

    pthread_create(&comm_rx_thread, nullptr, comm_rx_func, this);
}

void TcpClient::initSocket()
{
    serverSockFD = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if(serverSockFD < 0){
        cout << endl << "socket create error" << endl;
        return;
    }

    int on = 1;
    if(setsockopt(serverSockFD, SOL_SOCKET, SO_REUSEADDR, reinterpret_cast<const char*>(&on), sizeof(on)) < 0){
        cout << endl << "set option curLen = 0; error!!" << endl;
        return;
    }
    if(setsockopt(serverSockFD, SOL_SOCKET, SO_KEEPALIVE, reinterpret_cast<const char*>(&on), sizeof(on)) < 0){
        cout << endl << "set option curLen = 0; error!!" << endl;
        return;
    }

    memset(&server_addr, 0x00, sizeof(sockaddr_in)) ;

    server_addr.sin_addr.s_addr = inet_addr(ip.c_str());
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(port);
}

void TcpClient::connectSocket()
{
    curLen = 0;
    memset(bufWait, 0, MAXWAITBUFSIZE);
    ptrRecvBufIndx = bufWait;
    int ret = connect(serverSockFD, (struct sockaddr*)&server_addr, sizeof(server_addr));

    if(ret < 0){
        cout << endl << "connect error" << endl;
        server_connected = false;
    }
    else{
        server_connected = true;
        static_cast<MainWindow*>(mainWindow)->onConnectServer();
    }
}

void *TcpClient::comm_rx_func(void *arg)
{
    TcpClient *pThis = static_cast<TcpClient*>(arg);
    pThis->comm_thread_run = true;

    while(pThis->comm_thread_run){
        if(pThis->isConnected()){
            memset(pThis->buf, 0, MAXRECEIVEBUFSIZE);
            pThis->byteLen = recv(pThis->serverSockFD, pThis->buf, RECVBUFSIZE, 0);

//            printf("Received byte length : %ld\n", pThis->byteLen);

            if(pThis->byteLen > 0){
                if(pThis->byteLen == 1){
//                    cout << pThis->buf[0] << endl;
                    if(pThis->buf[0] == 'X'){
                        cout << "Client & Server configuration is difference" << endl;
                        static_cast<MainWindow*>(pThis->mainWindow)->disConnectServer();
                    }
                    else if(pThis->buf[0] == 'S' || pThis->buf[0] == 'E'){
                        static_cast<MainWindow*>(pThis->mainWindow)->componentEnable(true);
                    }
                }
                else{
                    if(pThis->buf[0] == 'N' && pThis->buf[1] == 'C'){
//                        printf("Received byteLen : %ld\n", pThis->byteLen);
//                        printf("received data : ");
//                        for(uint8_t i = 0; i < pThis->byteLen; i++){
//                            printf("%d ", pThis->buf[i]);
//                        }
//                        printf("\n");
                        char buf[8];
                        uint16_t indx = SOCKET_TOKEN_SIZE;
                        pThis->dataControl->ServerToClient.data_index = pThis->buf[indx];
                        indx += DATA_INDEX_LEN;

                        for(int i = 0; i < NUM_JOINT; i++){
                            memcpy(buf, pThis->buf + indx, MOTION_DATA_LEN);
                            pThis->dataControl->ServerToClient.presentJointPosition[i] = atof(buf);
                            indx += MOTION_DATA_LEN;
                        }

                        for(int i = 0; i < NUM_DOF; i++){
                            memcpy(buf, pThis->buf + indx, MOTION_DATA_LEN);
                            pThis->dataControl->ServerToClient.presentCartesianPose[i] = atof(buf);
                            indx += MOTION_DATA_LEN;
                        }

                        for(int i = 0; i < NUM_JOINT; i++){
                            memcpy(buf, pThis->buf + indx, MOTION_DATA_LEN);
                            pThis->dataControl->ServerToClient.desiredJointPosition[i] = atof(buf);
                            indx += MOTION_DATA_LEN;
                        }

                        for(int i = 0; i < NUM_DOF; i++){
                            memcpy(buf, pThis->buf + indx, MOTION_DATA_LEN);
                            pThis->dataControl->ServerToClient.desiredCartesianPose[i] = atof(buf);
                            indx += MOTION_DATA_LEN;
                        }

                        for(int i = 0; i < NUM_DOF; i++){
                            memcpy(buf, pThis->buf + indx, MOTION_DATA_LEN);
                            pThis->dataControl->ServerToClient.calculateCartesianPose[i] = atof(buf);
                            indx += MOTION_DATA_LEN;
                        }

                        for(int i = 0; i < NUM_JOINT; i++){
                            memcpy(buf, pThis->buf + indx, MOTION_DATA_LEN);
                            pThis->dataControl->ServerToClient.presentJointVelocity[i] = atof(buf);
                            indx += MOTION_DATA_LEN;
                        }

                        for(int i = 0; i < NUM_JOINT; i++){
                            memcpy(buf, pThis->buf + indx, MOTION_DATA_LEN);
                            pThis->dataControl->ServerToClient.presentJointCurrent[i] = atof(buf);
                            indx += MOTION_DATA_LEN;
                        }

                        for(int i = 0; i < NUM_DOF; i++){
                            memcpy(buf, pThis->buf + indx, MOTION_DATA_LEN);
                            pThis->dataControl->ServerToClient.presentCartesianVelocity[i] = atof(buf);
                            indx += MOTION_DATA_LEN;
                        }
                        memcpy(buf, pThis->buf + indx, TIME_LEN);
                        pThis->dataControl->ServerToClient.time = atof(buf);
                        indx += TIME_LEN;
                        memcpy(buf, pThis->buf + indx, TIME_LEN);
                        pThis->dataControl->ServerToClient.dxl_time = atof(buf);
                        indx += TIME_LEN;
                        memcpy(buf, pThis->buf + indx, TIME_LEN);
                        pThis->dataControl->ServerToClient.ik_time = atof(buf);
                        indx += TIME_LEN;
                        for(int i = 0; i < NUM_JOINT; i++){
                            memcpy(buf, pThis->buf + indx, MOTION_DATA_LEN);
                            pThis->dataControl->ServerToClient.presentJointTorque[i] = atof(buf);
                            indx += MOTION_DATA_LEN;
                        }

                        for(int i = 0; i < NUM_DOF; i++){
                            memcpy(buf, pThis->buf + indx, MOTION_DATA_LEN);
                            pThis->dataControl->ServerToClient.presentJointResidual[i] = atof(buf);
                            indx += MOTION_DATA_LEN;
                        }

                        pThis->dataControl->ServerToClient.opMode = pThis->buf[indx];
                        indx++;
//                        printf("Op Mode : %d\n", pThis->dataControl->ServerToClient.opMode);

                        if(pThis->dataControl->logging_start){
                            QString logStr;
                            logStr.push_back(QString::number(pThis->dataControl->ServerToClient.data_index, 'f', 6));
                            logStr.push_back(",");
                            for(int i = 0; i < NUM_JOINT; i++){
                                logStr.push_back(QString::number(pThis->dataControl->ServerToClient.presentJointPosition[i], 'f', 6));
                                logStr.push_back(",");
                            }
                            for(int i = 0; i < NUM_DOF; i++){
                                logStr.push_back(QString::number(pThis->dataControl->ServerToClient.presentCartesianPose[i], 'f', 6));
                                logStr.push_back(",");
                            }
                            for(int i = 0; i < NUM_JOINT; i++){
                                logStr.push_back(QString::number(pThis->dataControl->ServerToClient.presentJointVelocity[i], 'f', 6));
                                logStr.push_back(",");
                            }
                            for(int i = 0; i < NUM_DOF; i++){
                                logStr.push_back(QString::number(pThis->dataControl->ServerToClient.presentCartesianVelocity[i], 'f', 6));
                                logStr.push_back(",");
                            }
                            for(int i = 0; i < NUM_JOINT; i++){
                                logStr.push_back(QString::number(pThis->dataControl->ServerToClient.presentJointCurrent[i], 'f', 6));
                                logStr.push_back(",");
                            }
                            for(int i = 0; i < NUM_DOF; i++){
                                logStr.push_back(QString::number(pThis->dataControl->ServerToClient.desiredCartesianPose[i], 'f', 6));
                                logStr.push_back(",");
                            }
                            for(int i = 0; i < NUM_JOINT; i++){
                                logStr.push_back(QString::number(pThis->dataControl->ServerToClient.desiredJointPosition[i], 'f', 6));
                                logStr.push_back(",");
                            }
                            for(int i = 0; i < NUM_JOINT; i++){
                                logStr.push_back(QString::number(pThis->dataControl->ServerToClient.presentJointTorque[i], 'f', 6));
                                logStr.push_back(",");
                            }
                            for(int i = 0; i < NUM_JOINT; i++){
                                logStr.push_back(QString::number(pThis->dataControl->ServerToClient.presentJointResidual[i], 'f', 6));
                                logStr.push_back(",");
                            }
                            logStr.push_back("\n");
                            pThis->dataControl->logger->write(logStr);
                        }
                    }
                }
            }
            else{
                pThis->server_connected = false;
                break;
            }
        }
        usleep(1000);
    }
    cout << "recv thread finished" << endl;
    pThis->comm_thread_run = false;

    return nullptr;
}

void TcpClient::sendData(char* buf, int len){
    if(comm_thread_run && isConnected()){
        memcpy(bufSend, buf, len);
        sendByteLen = send(serverSockFD, bufSend, len, 0);

        if(sendByteLen < 0){
            printf("Send Error\n");
            stop();
        }
    }
}

void TcpClient::setting(const string& _ip, uint16_t _port)
{
    ip = _ip;
    port = _port;
}
void TcpClient::stop(){
    comm_thread_run = false;
//    pthread_join(comm_rx_thread, nullptr);
    pthread_cancel(comm_rx_thread);
    usleep(10000);
    server_connected = false;
}
