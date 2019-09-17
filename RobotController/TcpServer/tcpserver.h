#pragma once

#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <unistd.h>
#include <sys/mman.h>
#include <string.h>		// string function definitions
#include <fcntl.h>		// File control definitions
#include <errno.h>		// Error number definitions
#include <termios.h>	// POSIX terminal control definitions
#include <time.h>		// time calls
#include <sys/ioctl.h>
#include <math.h>

//-xenomai-///////////////////////////////////////////////////////////////
#include <native/task.h>
#include <native/timer.h>
#include <native/mutex.h>
#include <rtdk.h>		//The rdtk real-time printing library
/****************************************************************************/

#include <Poco/Event.h>
#include <NRMKSocketBase.h>

#include "DataControl/datacontrol.h"

#include <QString>

namespace NRMKHelper{
    class TcpServer : public NRMKSocketBase
    {
    public:
        TcpServer(DataControl *dataControl_);
    public:
        TcpServer();
        ~TcpServer();
        void setting(QString _ip, int _port);
        void OnEvent(UINT uEvent, LPVOID lpvData);
        // sendKey is used to send key input only
        void sendKey(char key);
        void getKey(char & key);
        bool isConnected();
        void OnDataReceived(const LPBYTE lpBuffer, DWORD dwCount);
        int getPort(){return PORT;}
        QString getIP(){return IP;}
        bool getDataCorrected(){return data_corrected;}
        void setConnected(bool flag);
//        void getCmdBuf(unsigned char* CmdBuf);

    private:
        QString IP;
        int PORT;
        volatile char commandkey;
        Poco::Event dataReceiveEvent;
        volatile bool connected;

        RT_TASK comm_task;
        static void comm_run[ [noreturn] ](void *arg);
        bool comm_thread_run;
        unsigned char *cmdbuf;
        bool data_corrected;
        DataControl *dataControl;

        void sendData();
    };
}
