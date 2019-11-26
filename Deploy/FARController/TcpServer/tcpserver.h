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

#include <Poco/Event.h>
#include <NRMKSocketBase.h>

#include "DataControl/datacontrol.h"
#include "CustomFunc/tcpserver_custom.h"

#include <QString>
#include <QByteArray>
#include <QtDebug>

#include <QtCore/qglobal.h>

#if defined(TCPSERVERLIB_LIBRARY)
#  define TCPSERVERLIB_EXPORT Q_DECL_EXPORT
#else
#  define TCPSERVERLIB_EXPORT Q_DECL_IMPORT
#endif

namespace NRMKHelper{
    class TcpServer : public NRMKSocketBase
    {
    public:
        TcpServer(DataControl *dataControl_);
        ~TcpServer();

        DataControl *dataControl;
        void sendData();
        bool comm_thread_run;
        TcpServerCustom *tcpServerCustom;
        bool isConnected();

        // sendKey is used to send key input only
        void sendKey(char key);
        void getKey(char & key);
        void setting(QString _ip, int _port);
        bool getDataCorrected(){return data_corrected;}
        int getPort(){return PORT;}
        QString getIP(){return IP;}

    private:
        QString IP;
        int PORT;
        volatile char commandkey;
        Poco::Event dataReceiveEvent;
        unsigned char *cmdbuf;
        bool data_corrected;
        void OnEvent(UINT uEvent, LPVOID lpvData);
        void OnDataReceived(const LPBYTE lpBuffer, DWORD dwCount);
        void setConnected(bool flag);
        volatile bool connected;

    signals:
        void disconnectClient();
    };
}
