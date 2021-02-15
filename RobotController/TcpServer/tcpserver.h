#ifndef TCPSERVER_H
#define TCPSERVER_H

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
#include "CustomFunc/tcpserver_custom.h"

#include <QtCore/qglobal.h>

#if defined(TCPSERVERLIB_LIBRARY)
#  define TCPSERVERLIB_EXPORT Q_DECL_EXPORT
#else
#  define TCPSERVERLIB_EXPORT Q_DECL_IMPORT
#endif

class TcpServer
{
public:
    TcpServer(DataControl *_dataControl);
    ~TcpServer();

    DataControl* dataControl;
    TcpServerCustom *tcpServerCustom;

    void start();
    void initSocket();
    void connectSocket();
    void stop();
    int sendData();
    void sendKey(const char* key);
    bool isConnected(){return connected;}
    void setting(uint16_t port_);
    uint16_t getPort(){return port;}
    static void* comm_func(void* arg);
    static void* comm_rx_func(void* arg);
    int in_area(const double section[4], double *p, int div_count);

    bool comm_thread_run, comm_run;
    int clientSockFD;

private:
    long sendByteLen;
    long byteLen, len;
    unsigned int curLen;
    int listenSockFD;
    char *ptrRecvBufIndx = nullptr;
    unsigned char buf[MAXRECEIVEBUFSIZE] = {0,};
    char bufWait[MAXWAITBUFSIZE] = {0,};
    char bufSend[MAXSENDBUFSIZE] = {0,};

    int dataLen = 0;

    pthread_t comm_thread, comm_rx_thread;

    sockaddr_in server_addr, client_addr;
    bool comm_thread_rx_run, comm_thread_tx_run;

    uint16_t port;
    string msg;

    bool connected;
    uint8_t temp;
};

#endif // TCPSERVER_H
