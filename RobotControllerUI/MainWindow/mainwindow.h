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
#include "Settings/customsettings.h"
#include "DataControl/datacontrol.h"
#include "FileIO/fileio.h"
#include "Input/keyinputclass.h"

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
    CustomSettings *customSettings;
    DataControl *dataControl;
    KeyInputClass *keyInputClass;

    QStandardItemModel *model, *torqueIdeModel;
//    QStandardItemModel *jointPathModel, *cartPathModel;
    QByteArray txData;
//    int rowClickedIndex, colClickedIndex, rowPressedIndex, colPressedIndex;

    void componentEnable(bool enable);

    QVector<QLineEdit*> txtJCmd, txtCCmd;
    void setTxtCommandClear();

    bool cmdJointRel, cmdJointAbs, cmdCartRel, cmdCartAbs;

public slots:
    // button event
    void btnConnectClicked();
    void btnDisconnectClicked();
    void btnSetInitClicked();
    void btnServOnClicked();
    void btnServoOffClicked();
    void btnJogMoveClicked();
    void btnSetJCommandClicked();
    void btnSetCCommandClicked();
    void btnFileLoadClicked();
//    void btnPathApplyClicked();
//    void btnPathClearClicked();
//    void btnPathInsertClicked();
//    void btnPathDeleteClicked();
//    void btnPathAppendClicked();
    void btnRunClicked();
    void btnStopClicked();
    void btnReadyClicked();
//    void btnStartClicked();
//    void btnSaveClicked();

    // checkbox event
//    void cbJointPathClicked();
//    void cbCartesianPathClicked();
    void cbJointRelChanged(int arg);
    void cbJointAbsChanged(int arg);
    void cbCartRelChanged(int arg);
    void cbCartAbsChanged(int arg);

    // server event
    void onConnectServer();
    void disConnectServer();
    void readMessage();

    // event
     void keyPressEvent(QKeyEvent *event);
     void closeEvent(QCloseEvent* event);

    // tableview event
//    void tvCellClicked(const QModelIndex &index);
//    void verticalSectionClicked(int index);
//    void horizontalSectionClicked(int index);
//    void verticalSectionPressed(int index);
//    void horizontalSectionPressed(int index);

     // spin box event
//     void sbMassEditingFinished();
//     void sbTorqueConstEditingFinished();
};

#endif // MAINWINDOW_H
