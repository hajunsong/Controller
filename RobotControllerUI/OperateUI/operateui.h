#ifndef OPERATEUI_H
#define OPERATEUI_H

#include <QDebug>
#include <QWidget>
#include <QDateTime>

#include "TcpSocket/tcpclient.h"
#include "TcpSocket/tcpserver.h"
#include "DataControl/datacontrol.h"

namespace Ui{
class OperateUI;
}

class OperateUI : public QWidget
{
    Q_OBJECT
public:
    explicit OperateUI(void *_tcpClient, QWidget *parent = nullptr);
    ~OperateUI();
    Ui::OperateUI *ui;

public slots:
    // button event
    void btnStartClciked();
    void btnTeachingClicked();
    void btnFeedingClicked();
    void btnSide1Clicked();
    void btnSide2Clicked();
    void btnSide3Clicked();
    void btnRiseClicked();
    void btnSoupClicked();
    void btnListenClicked();

    // TcpSocket event
    void readMessage();
    void connected();
    void disconnected();

private:
    void *tcpClient;
    QByteArray txData;
    TcpServer *tcpServer;

    void componentEnable(bool enable);
    void sendDataToServer(char *data);
};

#endif // OPERATEUI_H
