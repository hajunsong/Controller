#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QDebug>
#include <QSettings>
#include <QFile>
#include <QDesktopServices>
#include <QDir>
#include <QWidget>
#include <QTableView>
#include <QStandardItemModel>
#include <QStandardItem>
#include <QFileDialog>
#include <QListWidgetItem>
#include <QTimer>

#include "TcpSocket/tcpclient.h"
#include "Settings/customsettings.h"
#include "DataControl/datacontrol.h"
#include "FileIO/fileio.h"
#include "Input/keyinputclass.h"
#include "OperateUI/operateui.h"
#include "TorqueID/torqueid.h"

#include <QtCore/qglobal.h>

#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>
#include <cstring>
#include <arpa/inet.h>
#include <errno.h>
#include <iostream>
#include <math.h>

//const int MAXCONNECTIONS = 5;
//const int MAXWAITBUFSIZE = 4096;
//const int MAXRECEIVEBUFSIZE = 1024;
//const int RECVBUFSIZE = 156;

#if defined(MAINWINDOWLIB_LIBRARY)
#  define MAINWINDOWLIB_EXPORT Q_DECL_EXPORT
#else
#  define MAINWINDOWLIB_EXPORT Q_DECL_IMPORT
#endif

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

    enum OpMode{ServoOnOff = 0, Initialize, Wait, JointMove, CartesianMove, PathTransfer};
    enum Servo{On=1,Off=0};
    enum Motion{JogMotion = 0, JointMotion, CartesianJogMotion, CartesianMotion};

    void onConnectServer();
    void disConnectServer();
    void componentEnable(bool enable);

private:
    Ui::MainWindow *ui;
    TcpClient *tcpClient;
    CustomSettings *customSettings;
    DataControl *dataControl;
    KeyInputClass *keyInputClass;

    QStandardItemModel *model, *pathModel;
    QByteArray txData;
    int rowClickedIndex, colClickedIndex, rowPressedIndex, colPressedIndex;

    QVector<QLineEdit*> txtJCmd, txtCCmd;
    void setTxtCommandClear();

    bool cmdJointRel, cmdJointAbs, cmdCartRel, cmdCartAbs;

    OperateUI *operateUI;
    TorqueID *torqueID;

    QTimer *mainTimer;

    char bufSend[MAXSENDBUFSIZE];

public slots:
    // button event
    void btnConnectClicked();
    void btnDisconnectClicked();
    void btnSetInitClicked();
    void btnServOnClicked();
    void btnServoOffClicked();
    void btnSetJCommandClicked();
    void btnSetCCommandClicked();
    void btnPathApplyClicked();
    void btnPathClearClicked();
    void btnPathInsertClicked();
    void btnPathDeleteClicked();
    void btnPathAppendClicked();
    void btnRunClicked();
    void btnStopClicked();
    void btnReadyClicked();
    void btnFileReadyClicked();
    void btnFileRunClicked();
    void btnCustomRunClicked();
    void btnLoggingStartClicked();
    void btnLoggingStopClicked();

    // checkbox event
    void cbJointRelChanged(int arg);
    void cbJointAbsChanged(int arg);
    void cbCartRelChanged(int arg);
    void cbCartAbsChanged(int arg);

    // event
     void keyPressEvent(QKeyEvent *event);
     void closeEvent(QCloseEvent* event);

    // tableview event
    void tvCellClicked(const QModelIndex &index);
    void verticalSectionClicked(int index);
    void horizontalSectionClicked(int index);

    // timer event
    void mainTimeOut();
};

#endif // MAINWINDOW_H
