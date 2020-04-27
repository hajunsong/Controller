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

#include "TcpSocket/tcpclient.h"

enum { current_mode = 0, velocity_mode, position_mode = 3, extended_position_mode, current_based_position_mode, pwm_mode = 16 };
const int8_t JointOpMode[7] = {0, current_mode, velocity_mode, position_mode, extended_position_mode, current_based_position_mode, pwm_mode};
const int8_t NUM_JOINT = 6;
const int8_t NUM_DOF = 6;

class DataControl{
public:
//    typedef struct _StructClientToServerInitParam{
//        char numJoint, numDof, module;
//    }StructClientToServerInitParam;

    typedef struct _StructClientToServer{
        char opMode, subMode;
        double desiredJoint[NUM_JOINT], desiredPose[NUM_DOF];
    }StructClientToServer;

    typedef struct _StructServerToClient{
        int8_t data_index;
        double presentJointPosition[NUM_JOINT], presentCartesianPose[NUM_DOF];
        double desiredJointPosition[NUM_JOINT], desiredCartesianPose[NUM_DOF];
        double calculateCartesianPose[NUM_DOF];
        double presentJointVelocity[NUM_JOINT], presentJointCurrent[NUM_JOINT];
        double presentCartesianVelocity[NUM_DOF];
        double time, dxl_time, ik_time, t;
    }StructServerToClient;

    enum OpMode{ServoOnOff = 0, Initialize, Wait, JointMove, CartesianMove, PathGenerateMode, ReadyMode, RunMode, TorqueID, OperateMode};
    enum Motion{JogMotion = 0, JointMotion, CartesianJogMotion, CartesianMotion};
    enum Module{FAR_V1=1, FAR_V2, SEA};
    enum Comm{RS485=1, RS232, EtherCAT};
    enum CmdType{PathCmd=1, ReadyCmd, RunCmd, StopCmd, FileReady, FileRun, CustomRun};
    enum Operate{Start=1, Stop, StartTeaching, StopTeaching, StartFeeding, StopFeeding, Feeding};
    enum Section{Ready=0, Side1=1, Side2, Side3, Soup, Rise};

    DataControl();
    ~DataControl();
    void DataReset();

    StructClientToServer ClientToServer;
    StructServerToClient ServerToClient;

    const double ENC2DEG = 0.088;
    const double DEG2ENC = 11.363636364;
    const double DEG2RAD = 0.017453293;//3.14159265358979323846/180.0;
    const double RAD2DEG = 57.295779513;//180.0/3.14159265358979323846;
    const double ENC2RPM = 	0.229;
    const double RPM2DEG = 6;
    const double RAW2mA = 2.69;
    const double mA2RAW = 0.371747212;
};


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

private:
    Ui::MainWindow *ui;
    TcpClient *tcpClient;
    DataControl *dataControl;

    QStandardItemModel *model;
    QByteArray txData;
    int rowClickedIndex, colClickedIndex, rowPressedIndex, colPressedIndex;

    void componentEnable(bool enable);

    QVector<QLineEdit*> txtJCmd;

    bool cmdJointRel, cmdJointAbs;

public slots:
    // button event
    void btnConnectClicked();
    void btnDisconnectClicked();
    void btnSetInitClicked();
    void btnServOnClicked();
    void btnServoOffClicked();
    void btnSetJCommandClicked();

    // server event
    void onConnectServer();
    void disConnectServer();
    void readMessage();

    // event
     void closeEvent(QCloseEvent* event);
};

#endif // MAINWINDOW_H
