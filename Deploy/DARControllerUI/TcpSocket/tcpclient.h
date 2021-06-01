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

const int MAXCONNECTIONS = 5;
const int MAXWAITBUFSIZE = 4096;
const int MAXSENDBUFSIZE = 1024;
const int MAXRECEIVEBUFSIZE = 1024;
const int RECVBUFSIZE = 509;

#include "DataControl/datacontrol.h"

using namespace std;

class TcpClient
{
public:
    TcpClient(DataControl *_dataControl, void *_mainWindow);
    ~TcpClient();

    DataControl* dataControl;

    void start();
    void initSocket();
    void connectSocket();
    void stop();
    bool isConnected(){return server_connected;}
    void setting(const string& _ip, uint16_t _port);
    void sendData(char *buf, int len);
    static void* comm_func(void* arg);
    static void* comm_rx_func(void* arg);

    bool comm_thread_run;
    char bufSend[MAXSENDBUFSIZE] = {0,};
    long len;

private:
    long sendByteLen;
    long recvByteLen;
    long byteLen;
    unsigned int curLen;
    int serverSockFD;
    char *ptrRecvBufIndx = nullptr;
    char buf[MAXRECEIVEBUFSIZE] = {0,};
    char bufWait[MAXWAITBUFSIZE] = {0,};

    int dataLen = 0;

    pthread_t comm_rx_thread;

    sockaddr_in server_addr;

    uint16_t port;
    string ip;

    bool server_connected;

    void *mainWindow;
};

#endif // TCPSERVER_H
