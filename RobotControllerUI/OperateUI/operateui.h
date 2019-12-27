#ifndef OPERATEUI_H
#define OPERATEUI_H

#include <QMainWindow>
#include <QDebug>
#include <QWidget>

#include "TcpSocket/tcpclient.h"
#include "DataControl/datacontrol.h"

namespace Ui{
class OperateUI;
}

class OperateUI : public QMainWindow
{
    Q_OBJECT
public:
    explicit OperateUI(void *_tcp, QWidget *parent = nullptr);

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

private:
    Ui::OperateUI *ui;
    void *tcp;
    QByteArray txData;

    void componentEnable(bool enable);
    void sendDataToServer(char *data);
};

#endif // OPERATEUI_H
