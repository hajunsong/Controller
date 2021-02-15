#ifndef TCPSOCKET_H
#define TCPSOCKET_H

#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>
#include <cstring>
#include <arpa/inet.h>
#include <iostream>
#include <math.h>
#include <time.h>
#include <sstream>
#include <stdio.h>

#include <stdio.h>
#include <signal.h>
#include <unistd.h>
#include <sys/mman.h>

#include <native/task.h>
#include <native/timer.h>
#include <rtdk.h>

using namespace std;

const int MAXCONNECTIONS = 5;
const int MAXWAITBUFSIZE = 4096;
const int MAXSENDBUFSIZE = 1024;
const int MAXRECEIVEBUFSIZE = 1024;
const int SENDBUFSIZE = 156;

const uint8_t SOP_RX = 0xCA;
const uint8_t SOP_TX = 0xCB;
const uint8_t EOP = 0xCE;
const uint8_t CMD_DATA = 0xD0;
const uint8_t CMD_SECTION = 0xD1;
const uint8_t CMD_FEEDING = 0xD2;
const uint8_t CMD_TABLET_CHECK = 0xD3;

#include "DataControl/datacontrol.h"

class TcpServer
{
public:
    TcpServer(DataControl *_dataControl);
    ~TcpServer();

    DataControl *dataControl;

    bool comm_rx_run, comm_tx_run, comm_manager_run;
    int clientSockFD;
    RTIME comm_period;

    void start();
    void initSocket();
    void connectSocket();
    void stop();
    void sendData();
    void recvData();
    void sendKey(const char* key);
    bool isConnected(){return connected;}
    uint16_t getPort(){return port;}
    void setting(uint16_t port_);
    int in_area(const double section[4], double *p, int div_count);


private:
    RT_TASK rtTcpCommRxTask, rtTcpCommTxTask;

    long sendByteLen;
    long byteLen, len;
    unsigned int curLen;
    int listenSockFD;
    char *ptrRecvBufIndx = nullptr;
    unsigned char buf[MAXRECEIVEBUFSIZE] = {0,};
    char bufWait[MAXWAITBUFSIZE] = {0,};
    char bufSend[MAXSENDBUFSIZE] = {0,};

    int dataLen = 0;

    sockaddr_in server_addr, client_addr;

    uint16_t port;
    string msg;

    bool connected;

    pthread_t commManagerThread;

    void startCommManagerTask();
    static void *commManagerFunc(void *arg);
    static void rtCommRxFunc(void *arg);
    static void rtCommTxFunc(void *arg);
};

#endif // TCPSOCKET_H
