#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent) : QMainWindow(parent), ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    connect(ui->btnConnect, SIGNAL(clicked()), this, SLOT(btnConnectClicked()));
    connect(ui->btnDisconnect, SIGNAL(clicked()), this, SLOT(btnDisconnectClicked()));

    ui->btnDisconnect->setEnabled(false);

    connect(ui->btnSetInit, SIGNAL(clicked()), this, SLOT(btnSetInitClicked()));

    dataControl = new DataControl();
    tcpClient = new TcpClient(dataControl, this);

    mainTimer = new QTimer(this);
    connect(mainTimer, SIGNAL(timeout()), this, SLOT(mainTimeOut()));
    mainTimer->setInterval(10);

    connect(ui->btnServoOn, SIGNAL(clicked()), this, SLOT(btnServOnClicked()));
    connect(ui->btnServoOff, SIGNAL(clicked()), this, SLOT(btnServoOffClicked()));

    connect(ui->btnJSet, SIGNAL(clicked()), this, SLOT(btnSetJCommandClicked()));
    connect(ui->btnCSet, SIGNAL(clicked()), this, SLOT(btnSetCCommandClicked()));

    connect(ui->btnJ1N, SIGNAL(clicked()), this, SLOT(btnSetJCommandClicked()));
    connect(ui->btnJ2N, SIGNAL(clicked()), this, SLOT(btnSetJCommandClicked()));
    connect(ui->btnJ3N, SIGNAL(clicked()), this, SLOT(btnSetJCommandClicked()));
    connect(ui->btnJ4N, SIGNAL(clicked()), this, SLOT(btnSetJCommandClicked()));
    connect(ui->btnJ5N, SIGNAL(clicked()), this, SLOT(btnSetJCommandClicked()));
    connect(ui->btnJ6N, SIGNAL(clicked()), this, SLOT(btnSetJCommandClicked()));
    connect(ui->btnJ1P, SIGNAL(clicked()), this, SLOT(btnSetJCommandClicked()));
    connect(ui->btnJ2P, SIGNAL(clicked()), this, SLOT(btnSetJCommandClicked()));
    connect(ui->btnJ3P, SIGNAL(clicked()), this, SLOT(btnSetJCommandClicked()));
    connect(ui->btnJ4P, SIGNAL(clicked()), this, SLOT(btnSetJCommandClicked()));
    connect(ui->btnJ5P, SIGNAL(clicked()), this, SLOT(btnSetJCommandClicked()));
    connect(ui->btnJ6P, SIGNAL(clicked()), this, SLOT(btnSetJCommandClicked()));

    connect(ui->btnC1N, SIGNAL(clicked()), this, SLOT(btnSetCCommandClicked()));
    connect(ui->btnC2N, SIGNAL(clicked()), this, SLOT(btnSetCCommandClicked()));
    connect(ui->btnC3N, SIGNAL(clicked()), this, SLOT(btnSetCCommandClicked()));
    connect(ui->btnC4N, SIGNAL(clicked()), this, SLOT(btnSetCCommandClicked()));
    connect(ui->btnC5N, SIGNAL(clicked()), this, SLOT(btnSetCCommandClicked()));
    connect(ui->btnC6N, SIGNAL(clicked()), this, SLOT(btnSetCCommandClicked()));
    connect(ui->btnC1P, SIGNAL(clicked()), this, SLOT(btnSetCCommandClicked()));
    connect(ui->btnC2P, SIGNAL(clicked()), this, SLOT(btnSetCCommandClicked()));
    connect(ui->btnC3P, SIGNAL(clicked()), this, SLOT(btnSetCCommandClicked()));
    connect(ui->btnC4P, SIGNAL(clicked()), this, SLOT(btnSetCCommandClicked()));
    connect(ui->btnC5P, SIGNAL(clicked()), this, SLOT(btnSetCCommandClicked()));
    connect(ui->btnC6P, SIGNAL(clicked()), this, SLOT(btnSetCCommandClicked()));

    txtJCmd.push_back(ui->txtJCmd1);
    txtJCmd.push_back(ui->txtJCmd2);
    txtJCmd.push_back(ui->txtJCmd3);
    txtJCmd.push_back(ui->txtJCmd4);
    txtJCmd.push_back(ui->txtJCmd5);
    txtJCmd.push_back(ui->txtJCmd6);
    txtCCmd.push_back(ui->txtCCmd1);
    txtCCmd.push_back(ui->txtCCmd2);
    txtCCmd.push_back(ui->txtCCmd3);
    txtCCmd.push_back(ui->txtCCmd4);
    txtCCmd.push_back(ui->txtCCmd5);
    txtCCmd.push_back(ui->txtCCmd6);

    for(int i = 0; i < txtJCmd.length(); i++)
    {
        txtJCmd[i]->setText("0");
    }
    for(int i = 0; i < txtCCmd.length(); i++)
    {
        txtCCmd[i]->setText("0");
    }

    ui->txtCartesianMoveTime->setText("0");
    ui->txtCartesianMoveAccTime->setText("0");

    connect(ui->btnPathClear, SIGNAL(clicked()), this, SLOT(btnPathClearClicked()));
    connect(ui->btnPathApply, SIGNAL(clicked()), this, SLOT(btnPathApplyClicked()));
    connect(ui->btnPathInsert, SIGNAL(clicked()), this, SLOT(btnPathInsertClicked()));
    connect(ui->btnPathDelete, SIGNAL(clicked()), this, SLOT(btnPathDeleteClicked()));
    connect(ui->btnPathAppend, SIGNAL(clicked()), this, SLOT(btnPathAppendClicked()));

    connect(ui->tvPathData, SIGNAL(clicked(QModelIndex)), this, SLOT(tvCellClicked(QModelIndex)));
    connect(ui->tvPathData->horizontalHeader(), SIGNAL(sectionClicked(int)), this, SLOT(horizontalSectionClicked(int)));
    connect(ui->tvPathData->verticalHeader(), SIGNAL(sectionClicked(int)), this, SLOT(verticalSectionClicked(int)));
    connect(ui->tvPathData->horizontalHeader(), SIGNAL(sectionPressed(int)), this, SLOT(horizontalSectionPressed(int)));
    connect(ui->tvPathData->verticalHeader(), SIGNAL(sectionPressed(int)), this, SLOT(verticalSectionPressed(int)));

    connect(ui->btnRun, SIGNAL(clicked()), this, SLOT(btnRunClicked()));
    connect(ui->btnStop, SIGNAL(clicked()), this, SLOT(btnStopClicked()));
    connect(ui->btnReady, SIGNAL(clicked()), this, SLOT(btnReadyClicked()));

    connect(ui->cbJointRel, SIGNAL(stateChanged(int)), this, SLOT(cbJointRelChanged(int)));
    connect(ui->cbJointAbs, SIGNAL(stateChanged(int)), this, SLOT(cbJointAbsChanged(int)));
    connect(ui->cbCartRel, SIGNAL(stateChanged(int)), this, SLOT(cbCartRelChanged(int)));
    connect(ui->cbCartAbs, SIGNAL(stateChanged(int)), this, SLOT(cbCartAbsChanged(int)));

    cmdJointRel = false;
    cmdJointAbs = false;
    cmdCartRel = false;
    cmdCartAbs = false;

    ui->btnRun->setEnabled(false);
    ui->btnReady->setEnabled(false);

    operateUI = new OperateUI(tcpClient);
    torqueID = new TorqueID(tcpClient);

    keyInputClass = new KeyInputClass(ui, operateUI, torqueID, tcpClient);

    ui->tabWidget->setCurrentIndex(0);

    connect(ui->btnFileReady, SIGNAL(clicked()), this, SLOT(btnFileReadyClicked()));
    connect(ui->btnFileRun, SIGNAL(clicked()), this, SLOT(btnFileRunClicked()));

//    ui->gbRobotConfig->setEnabled(true);
//    ui->btnSetInit->setEnabled(true);

//    connect(ui->btnCustomRun, SIGNAL(clicked()), this, SLOT(btnCustomRunClicked()));
    ui->btnCustomRun->hide();

    connect(ui->btnLoggingStart, SIGNAL(clicked()), this, SLOT(btnLoggingStartClicked()));
    connect(ui->btnLoggingStop, SIGNAL(clicked()), this, SLOT(btnLoggingStopClicked()));

    ui->tabWidget->addTab(operateUI, "Operate");

    customSettings = new CustomSettings(ui, operateUI->ui);
    customSettings->loadConfigFile();
}

MainWindow::~MainWindow()
{
    customSettings->saveConfigFile();
    txtJCmd.clear();
    txtCCmd.clear();
    delete ui;
    delete tcpClient;
    delete dataControl;
    delete customSettings;
    delete keyInputClass;
    delete operateUI;
}

void MainWindow::btnConnectClicked(){
    tcpClient->setting(ui->txtIP->text().toStdString(), ui->txtPORT->text().toUShort());
    tcpClient->start();
}

void MainWindow::btnDisconnectClicked(){
    mainTimer->stop();
    ui->btnConnect->setEnabled(true);
    ui->btnDisconnect->setEnabled(false);
    ui->gbRobotConfig->setEnabled(false);
    componentEnable(false);
    tcpClient->stop();
    qDebug() << "Disconnected Server";

    ui->btnRun->setEnabled(false);

    operateUI->stop();
    usleep(10000);

    ui->tabWidget->setCurrentIndex(0);
}

void MainWindow::mainTimeOut()
{
    if(tcpClient->isConnected() && ui->gbRobotInfor->isEnabled()){
        for(int i = 0; i < NUM_JOINT; i++){
            QModelIndex index = model->index(0, i);
            model->setData(index, dataControl->ServerToClient.presentJointPosition[i]);
            ui->tvRobotInfor->update(index);
        }

        for(int i = 0; i < NUM_DOF; i++){
            QModelIndex index = model->index(1, i);
            model->setData(index, dataControl->ServerToClient.presentCartesianPose[i]);
            ui->tvRobotInfor->update(index);
        }

        for(int i = 0; i < NUM_JOINT; i++){
            QModelIndex index = model->index(2, i);
            model->setData(index, dataControl->ServerToClient.presentJointVelocity[i]);
            ui->tvRobotInfor->update(index);
        }

        for(int i = 0; i < NUM_JOINT; i++){
            QModelIndex index = model->index(3, i);
            model->setData(index, dataControl->ServerToClient.presentJointCurrent[i]);
            ui->tvRobotInfor->update(index);
        }

        for(int i = 0; i < NUM_DOF; i++){
            QModelIndex index = model->index(4, i);
            model->setData(index, dataControl->ServerToClient.presentCartesianVelocity[i]*dataControl->RAD2DEG);
            ui->tvRobotInfor->update(index);
        }

        ui->txtTime->setText(QString::number(dataControl->ServerToClient.time, 'f', 6));
        ui->txtDxlTime->setText(QString::number(dataControl->ServerToClient.dxl_time, 'f', 6));
        ui->txtIKTime->setText(QString::number(dataControl->ServerToClient.ik_time, 'f', 6));
    }
    if(!tcpClient->isConnected()){
        mainTimer->stop();
        ui->btnDisconnect->animateClick(100);
    }
}

void MainWindow::onConnectServer(){
    qDebug() << "Connected Server";
    ui->btnConnect->setEnabled(false);
    ui->btnDisconnect->setEnabled(true);
    ui->gbRobotConfig->setEnabled(true);
}

void MainWindow::disConnectServer(){
    ui->btnDisconnect->animateClick(100);
}

void MainWindow::btnSetInitClicked()
{
    int16_t indx = 0;
    memset(bufSend, 0, MAXSENDBUFSIZE);
    bufSend[indx++] = Qt::Key_N;
    bufSend[indx++] = Qt::Key_D;
    bufSend[indx++] = static_cast<char>(ui->cbNumJoint->currentText().toInt());
    bufSend[indx++] = static_cast<char>(ui->cbNumDOF->currentText().toInt());
    bufSend[indx++] = static_cast<char>(ui->cbModuleType->currentIndex());
    bufSend[indx++] = static_cast<char>(JointOpMode[(ui->cbJointMode->currentIndex())]);
    bufSend[indx++] = Qt::Key_N;
    bufSend[indx++] = Qt::Key_E;

    qDebug() << "txData : " << bufSend;

    tcpClient->sendData(bufSend, indx);
    mainTimer->start();

    int row = 5;
    model = new QStandardItemModel(row, NUM_JOINT, this);
    ui->tvRobotInfor->setModel(model);

    for(int i = 0; i < row; i++){
        for(int j = 0; j < NUM_JOINT; j++){
            QModelIndex index = model->index(i, j, QModelIndex());
            model->setData(index, 0);
        }
    }

    QStringList vHeader;
    vHeader.append("Present Joint Pos [deg]");
    vHeader.append("Present End Pose [mm, deg]");
//    vHeader.append("Command Pos [deg]");
//    vHeader.append("Desired Pose [mm, deg]");
//    vHeader.append("Calculate Pose [mm, deg]");
    vHeader.append("Present Joint Vel [RPM]");
    vHeader.append("Present Joint Cur [mA]");
    vHeader.append("Present End Vel [deg/s]");
    model->setVerticalHeaderLabels(vHeader);

    pathModel = new QStandardItemModel(1, 5, this);
    ui->tvPathData->setModel(pathModel);

    for(int i = 0; i < 1; i++){
        for(int j = 0; j < 5; j++){
            QModelIndex indx = pathModel->index(i, j, QModelIndex());
            pathModel->setData(indx, "0");
        }
    }

    QStringList hHeader;
    hHeader.append("Time");
    hHeader.append("PX");
    hHeader.append("PY");
    hHeader.append("PZ");
    hHeader.append("PX");
    hHeader.append("PY");
    hHeader.append("PZ");
    hHeader.append("Acc time");
    pathModel->setHorizontalHeaderLabels(hHeader);
}

void MainWindow::btnServOnClicked()
{
    dataControl->ClientToServer.opMode = OpMode::ServoOnOff;
    dataControl->ClientToServer.subMode = Servo::On;

    int16_t indx = 0;
    memset(bufSend, 0, MAXSENDBUFSIZE);
    bufSend[indx++] = Qt::Key_N;
    bufSend[indx++] = Qt::Key_S;
    bufSend[indx++] = dataControl->ClientToServer.opMode;
    bufSend[indx++] = dataControl->ClientToServer.subMode;
    bufSend[indx++] = Qt::Key_N;
    bufSend[indx++] = Qt::Key_E;

    qDebug() << "txData : " << bufSend;

    tcpClient->sendData(bufSend, indx);
}

void MainWindow::btnServoOffClicked()
{
    dataControl->ClientToServer.opMode = OpMode::ServoOnOff;
    dataControl->ClientToServer.subMode = Servo::Off;

    int16_t indx = 0;
    memset(bufSend, 0, MAXSENDBUFSIZE);
    bufSend[indx++] = Qt::Key_N;
    bufSend[indx++] = Qt::Key_S;
    bufSend[indx++] = dataControl->ClientToServer.opMode;
    bufSend[indx++] = dataControl->ClientToServer.subMode;
    bufSend[indx++] = Qt::Key_N;
    bufSend[indx++] = Qt::Key_E;

    qDebug() << "txData : " << bufSend;

    tcpClient->sendData(bufSend, indx);
}

void MainWindow::btnSetJCommandClicked() {
    dataControl->ClientToServer.opMode = DataControl::OpMode::JointMove;
    QString objName = sender()->objectName();
    if (cmdJointRel || cmdJointAbs) {
        if (cmdJointRel){
            dataControl->ClientToServer.subMode = Motion::JogMotion;
        }

        if (cmdJointAbs){
            dataControl->ClientToServer.subMode = Motion::JointMotion;
        }

        memset(dataControl->ClientToServer.desiredJoint, 0, sizeof(double)*NUM_JOINT);

        for(int i = 0; i < NUM_JOINT; i++)
        {
            dataControl->ClientToServer.desiredJoint[i] = txtJCmd[i]->text().toDouble();
        }
    }
    else if(objName.contains("P") || objName.contains("N")){
        dataControl->ClientToServer.subMode = Motion::JogMotion;
        memset(dataControl->ClientToServer.desiredJoint, 0, sizeof(double)*NUM_JOINT);
        int indx = objName.at(4).digitValue() - 1;
        if (objName.contains("P")){
            dataControl->ClientToServer.desiredJoint[indx] = 5;
        }
        else{
            dataControl->ClientToServer.desiredJoint[indx] = -5;
        }
    }
    else{
        qDebug() << "Select Relative or Absolute";
        return;
    }

    int16_t indx = 0;
    memset(bufSend, 0, MAXSENDBUFSIZE);
    bufSend[indx++] = Qt::Key_N;
    bufSend[indx++] = Qt::Key_S;
    bufSend[indx++] = dataControl->ClientToServer.opMode;
    bufSend[indx++] = dataControl->ClientToServer.subMode;
    for(int i = 0; i < NUM_JOINT; i++)
    {
        memcpy(bufSend + indx, to_string(dataControl->ClientToServer.desiredJoint[i]).c_str(), MOTION_DATA_LEN);
        indx += MOTION_DATA_LEN;
    }
    bufSend[indx++] = Qt::Key_N;
    bufSend[indx++] = Qt::Key_E;

    qDebug() << "txData : " << bufSend;

    tcpClient->sendData(bufSend, indx);

    setTxtCommandClear();
}

void MainWindow::btnSetCCommandClicked() {
    dataControl->ClientToServer.opMode = DataControl::OpMode::CartesianMove;
    QString objName = sender()->objectName();
    double move_time = 0, acc_time = 0;
    if(cmdCartRel || cmdCartAbs) {
        if (cmdCartRel){
            dataControl->ClientToServer.subMode = Motion::CartesianJogMotion;
        }

        if (cmdCartAbs){
            dataControl->ClientToServer.subMode = Motion::CartesianMotion;
        }

        memset(dataControl->ClientToServer.desiredPose, 0, sizeof(double)*NUM_DOF);

        for(int i = 0; i < NUM_DOF; i++){
            dataControl->ClientToServer.desiredPose[i] = txtCCmd[i]->text().toDouble();
        }
        move_time = ui->txtCartesianMoveTime->text().toDouble();
        acc_time = ui->txtCartesianMoveAccTime->text().toDouble();
    }
    else if(objName.contains("P") || objName.contains("N")){
        dataControl->ClientToServer.subMode = Motion::CartesianJogMotion;
        memset(dataControl->ClientToServer.desiredPose, 0, sizeof(double)*NUM_DOF);
        int indx = objName.at(4).digitValue() - 1;
        if (objName.contains("P")){
            dataControl->ClientToServer.desiredPose[indx] = indx < 3 ? 10 : 5;
        }
        else{
            dataControl->ClientToServer.desiredPose[indx] = indx < 3 ? -10 : -5;
        }
    }
    else{
        qDebug() << "Select Relative or Absolute";
        return;
    }

    int16_t indx = 0;
    memset(bufSend, 0, MAXSENDBUFSIZE);
    bufSend[indx++] = Qt::Key_N;
    bufSend[indx++] = Qt::Key_S;
    bufSend[indx++] = dataControl->ClientToServer.opMode;
    bufSend[indx++] = dataControl->ClientToServer.subMode;
    for(int i = 0; i < NUM_DOF; i++)
    {
        memcpy(bufSend + indx, to_string(dataControl->ClientToServer.desiredPose[i]).c_str(), MOTION_DATA_LEN);
        indx += MOTION_DATA_LEN;
    }
//    bufSend[indx++] = move_time;
//    bufSend[indx++] = acc_time;
    memcpy(bufSend + indx, to_string(move_time).c_str(), MOTION_DATA_LEN);
    indx += MOTION_DATA_LEN;
    memcpy(bufSend + indx, to_string(acc_time).c_str(), MOTION_DATA_LEN);
    indx += MOTION_DATA_LEN;
    bufSend[indx++] = Qt::Key_N;
    bufSend[indx++] = Qt::Key_E;

    qDebug() << "txData : " << bufSend;

    tcpClient->sendData(bufSend, indx);

//    setTxtCommandClear();
}

void MainWindow::componentEnable(bool enable){
    ui->gbRobotInfor->setEnabled(enable);
    ui->gbServoControl->setEnabled(enable);
    ui->gbCartMoveCommand->setEnabled(enable);
    ui->gbJointMoveCommand->setEnabled(enable);
    ui->gbTrajectory->setEnabled(enable);
    ui->gbLoggingControl->setEnabled(enable);

    if (ui->cbNumJoint->currentText().toInt() == 1){
        ui->lbJoint2->setEnabled(false);
        ui->lbJoint3->setEnabled(false);
        ui->lbJoint4->setEnabled(false);
        ui->lbJoint5->setEnabled(false);
        ui->lbJoint6->setEnabled(false);
        ui->btnJ2N->setEnabled(false);
        ui->btnJ3N->setEnabled(false);
        ui->btnJ4N->setEnabled(false);
        ui->btnJ5N->setEnabled(false);
        ui->btnJ6N->setEnabled(false);
        ui->btnJ2P->setEnabled(false);
        ui->btnJ3P->setEnabled(false);
        ui->btnJ4P->setEnabled(false);
        ui->btnJ5P->setEnabled(false);
        ui->btnJ6P->setEnabled(false);

        for(int i = 1; i < 6; i++){
            txtJCmd[i]->setEnabled(false);
        }
    }

    if (ui->cbNumDOF->currentText().toInt() == 1){
        ui->gbCartMoveCommand->setEnabled(false);
    }

    ///////////////////////////////////////////////////////////////////////////
//    if (ui->gbTrajectory->isEnabled()){
//        ui->tvPathData->model()->removeRows(0, ui->tvPathData->model()->rowCount());
//        ui->tvPathData->model()->insertRows(0,7);

//        double path[7*8] = {
//            0.0, -0.205112, 0.105523,  0.001374, 1.570796,  0.000000, -1.570796, 0.3,
//            1.0, -0.284822, 0.077250, -0.041363, 1.570796,  0.698131, -1.570796, 0.3,
//            2.0, -0.284822, 0.077250, -0.088270, 1.570796,  0.698131, -1.570796, 0.3,
//            3.0, -0.285133, 0.026167, -0.088270, 1.570796,  0.698131, -1.570796, 0.3,
//            4.0, -0.285133, 0.015167, -0.080270, 1.564223, -0.782312, -1.566155, 0.3,
//            5.0, -0.285133, 0.015167,  0.011729, 1.564223, -0.782312, -1.566155, 0.3,
//            6.0, -0.205422, 0.026833,  0.058549, 1.564223, -0.782312, -1.566155, 0.3
////            0.0, -0.208, 0.1750735,   0.07, 1.5707963, 0.0, -2.094399, 0.3,
////            1.0, -0.124, 0.2590735, -0.014, 1.5707963, 0.0, -2.094399, 0.3,
////            2.0, -0.292, 0.2590735, -0.014, 1.5707963, 0.0, -2.094399, 0.3,
////            3.0, -0.292, 0.0910735,  0.154, 1.5707963, 0.0, -2.094399, 0.3,
////            4.0, -0.124, 0.0910735,  0.154, 1.5707963, 0.0, -2.094399, 0.3,
////            5.0, -0.208, 0.1750735,   0.07, 1.5707963, 0.0, -2.094399, 0.3
//        };

//        for(int i = 0; i < 7; i++){
//            for(int j = 0; j < 8; j++){
//                QModelIndex indx = ui->tvPathData->model()->index(i, j, QModelIndex());
//                ui->tvPathData->model()->setData(indx, QString::number(path[i*8 + j]));
//            }
//        }
//    }
}

void MainWindow::setTxtCommandClear()
{
    for(int i = 0; i < txtJCmd.length(); i++)
    {
        txtJCmd[i]->setText("0");
    }
    for(int i = 0; i < txtCCmd.length(); i++)
    {
        txtCCmd[i]->setText("0");
    }
}

void MainWindow::cbJointRelChanged(int checked)
{
    if (checked){
        ui->cbJointAbs->setChecked(false);
    }
    cmdJointRel = checked;
}

void MainWindow::cbJointAbsChanged(int checked)
{
    if (checked){
        ui->cbJointRel->setChecked(false);
    }
    cmdJointAbs = checked;
}

void MainWindow::cbCartRelChanged(int checked)
{
    if (checked){
        ui->cbCartAbs->setChecked(false);
    }
    cmdCartRel = checked;
}

void MainWindow::cbCartAbsChanged(int checked)
{
    if (checked){
        ui->cbCartRel->setChecked(false);
    }
    cmdCartAbs = checked;
}

void MainWindow::btnRunClicked()
{
    int16_t indx = 0;
    memset(bufSend, 0, MAXSENDBUFSIZE);
    bufSend[indx++] = Qt::Key_N;
    bufSend[indx++] = Qt::Key_U;
    bufSend[indx++] = DataControl::OpMode::RunMode;
    bufSend[indx++] = DataControl::CmdType::RunCmd;
    if(ui->cbRepeat->isChecked()){
        bufSend[indx++] = -1;
    }
    else{
        bufSend[indx++] = 1;
    }
    bufSend[indx++] = Qt::Key_N;
    bufSend[indx++] = Qt::Key_E;

    qDebug() << "txData : " << bufSend;

    tcpClient->sendData(bufSend, indx);
}

void MainWindow::btnStopClicked()
{
    int16_t indx = 0;
    memset(bufSend, 0, MAXSENDBUFSIZE);
    bufSend[indx++] = Qt::Key_N;
    bufSend[indx++] = Qt::Key_U;
    bufSend[indx++] = DataControl::OpMode::Wait;
    bufSend[indx++] = DataControl::CmdType::StopCmd;
    bufSend[indx++] = 0;
    bufSend[indx++] = Qt::Key_N;
    bufSend[indx++] = Qt::Key_E;

    qDebug() << "txData : " << bufSend;

    tcpClient->sendData(bufSend, indx);

    ui->btnRun->setEnabled(false);
    ui->btnReady->setEnabled(false);
}

void MainWindow::btnReadyClicked()
{
    int16_t indx = 0;
    memset(bufSend, 0, MAXSENDBUFSIZE);
    bufSend[indx++] = Qt::Key_N;
    bufSend[indx++] = Qt::Key_U;
    bufSend[indx++] = DataControl::OpMode::ReadyMode;
    bufSend[indx++] = DataControl::CmdType::ReadyCmd;
    if(ui->cbRepeat->isChecked()){
        bufSend[indx++] = -1;
    }
    else{
        bufSend[indx++] = 1;
    }
    bufSend[indx++] = Qt::Key_N;
    bufSend[indx++] = Qt::Key_E;

    qDebug() << "txData : " << bufSend;

    tcpClient->sendData(bufSend, indx);

    ui->btnRun->setEnabled(true);
}

void MainWindow::btnFileReadyClicked()
{
    int16_t indx = 0;
    memset(bufSend, 0, MAXSENDBUFSIZE);
    bufSend[indx++] = Qt::Key_N;
    bufSend[indx++] = Qt::Key_U;
    bufSend[indx++] = DataControl::OpMode::RunMode;
    bufSend[indx++] = DataControl::CmdType::FileReady;
    if(ui->cbRepeat->isChecked()){
        bufSend[indx++] = -1;
    }
    else{
        bufSend[indx++] = 1;
    }
    bufSend[indx++] = Qt::Key_N;
    bufSend[indx++] = Qt::Key_E;

    qDebug() << "txData : " << bufSend;

    tcpClient->sendData(bufSend, indx);
}

void MainWindow::btnFileRunClicked()
{
    int16_t indx = 0;
    memset(bufSend, 0, MAXSENDBUFSIZE);
    bufSend[indx++] = Qt::Key_N;
    bufSend[indx++] = Qt::Key_U;
    bufSend[indx++] = DataControl::OpMode::RunMode;
    bufSend[indx++] = DataControl::CmdType::FileRun;
    if(ui->cbRepeat->isChecked()){
        bufSend[indx++] = -1;
    }
    else{
        bufSend[indx++] = 1;
    }
    bufSend[indx++] = Qt::Key_N;
    bufSend[indx++] = Qt::Key_E;

    qDebug() << "txData : " << bufSend;

    tcpClient->sendData(bufSend, indx);
}

void MainWindow::btnCustomRunClicked()
{
    int16_t indx = 0;
    memset(bufSend, 0, MAXSENDBUFSIZE);
    bufSend[indx++] = Qt::Key_N;
    bufSend[indx++] = Qt::Key_U;
    bufSend[indx++] = DataControl::OpMode::RunMode;
    bufSend[indx++] = DataControl::CmdType::CustomRun;
    if(ui->cbRepeat->isChecked()){
        bufSend[indx++] = -1;
    }
    else{
        bufSend[indx++] = 1;
    }
    bufSend[indx++] = Qt::Key_N;
    bufSend[indx++] = Qt::Key_E;

    qDebug() << "txData : " << bufSend;

    tcpClient->sendData(bufSend, indx);
}

void MainWindow::btnPathApplyClicked()
{
    int8_t tvRow = static_cast<int8_t>(pathModel->rowCount());
    int8_t tvCol = static_cast<int8_t>(pathModel->columnCount());

    int16_t indx = 0;
    memset(bufSend, 0, MAXSENDBUFSIZE);
    bufSend[indx++] = Qt::Key_N;
    bufSend[indx++] = Qt::Key_U;
    bufSend[indx++] = DataControl::OpMode::PathGenerateMode;
    bufSend[indx++] = DataControl::CmdType::PathCmd;
    bufSend[indx++] = tvRow;
    bufSend[indx++] = tvCol;
    for(int i = 0; i < tvRow; i++){
        for(int j = 0; j < tvCol; j++){
            memcpy(bufSend + indx, pathModel->data(pathModel->index(i, j)).toString().toStdString().c_str(), PATH_DATA_LEN);
            indx += PATH_DATA_LEN;
//            qDebug() << pathModel->data(pathModel->index(i, j)).toString();
        }
    }
    bufSend[indx++] = Qt::Key_N;
    bufSend[indx++] = Qt::Key_E;

    qDebug() << "txData : " << bufSend;

    tcpClient->sendData(bufSend, indx);

    ui->btnReady->setEnabled(true);
}

void MainWindow::btnPathClearClicked()
{
    pathModel->removeRows(0, pathModel->rowCount());
}

void MainWindow::btnPathInsertClicked()
{
    if (rowClickedIndex >= 0){
        pathModel->insertRow(rowClickedIndex);
    }
    if (colClickedIndex >= 0){
        pathModel->insertColumn(colClickedIndex);
    }
}

void MainWindow::btnPathDeleteClicked()
{
    if (rowClickedIndex >= 0 && rowPressedIndex >= 0){
        pathModel->removeRows(rowPressedIndex, abs(rowClickedIndex - rowPressedIndex) + 1);
        rowClickedIndex = -1;
        rowPressedIndex = -1;
    }
    if (colClickedIndex >= 0 && colPressedIndex >= 0){
        pathModel->removeColumns(colPressedIndex, abs(colClickedIndex - colPressedIndex) + 1);
        colClickedIndex = -1;
        colPressedIndex = -1;
    }
}

void MainWindow::btnPathAppendClicked()
{
    if (rowClickedIndex >= 0){
        pathModel->insertRow(pathModel->rowCount());
    }
    if (colClickedIndex >= 0){
        pathModel->insertColumn(pathModel->columnCount());
    }
}

void MainWindow::tvCellClicked(const QModelIndex &index)
{
    rowClickedIndex = index.row();
    colClickedIndex = index.column();
    qDebug() << "Clicked row : " << rowClickedIndex << ", col : " << colClickedIndex;
}

void MainWindow::horizontalSectionClicked(int index)
{
    colClickedIndex = index;
    qDebug() << "Clicked horizontal : " << index;
    rowClickedIndex = -1;
}

void MainWindow::verticalSectionClicked(int index)
{
    rowClickedIndex = index;
    qDebug() << "Clicked vertical : " << index;
    colClickedIndex = -1;
}

void MainWindow::horizontalSectionPressed(int index)
{
    colPressedIndex = index;
    qDebug() << "Pressed horizontal : " << index;
    rowClickedIndex = -1;
    rowPressedIndex = -1;
}

void MainWindow::verticalSectionPressed(int index)
{
    rowPressedIndex = index;
    qDebug() << "Pressed vertical : " << index;
    colClickedIndex = -1;
    colPressedIndex = -1;
}

void MainWindow::keyPressEvent(QKeyEvent *event)
{
    keyInputClass->InputKeyboard(event);
}

void MainWindow::closeEvent(QCloseEvent*){
    qDebug() << "Closed MainWindow";
}

void MainWindow::btnLoggingStartClicked()
{
    qDebug() << "Logging Start";
    QDateTime date;
    QString fileName = "../logging/" + date.currentDateTime().toString("yyyy-MM-dd-hh-mm-ss") + ".csv";
    dataControl->logger = new Logger(this, fileName);

    QString data = "Indx";
    data += ",";
    for (uint i = 0; i < NUM_JOINT; i++){
        data += "Joint Position " + QString::number(i+1) + " [deg]";
        data += ",";
    }
    data += "End X [mm],";
    data += "End Y [mm],";
    data += "End Z [mm],";
    data += "End Roll [deg],";
    data += "End Pitch [deg],";
    data += "End Yaw [deg],";
    for (uint i = 0; i < NUM_JOINT; i++){
        data += "Joint Velocity " + QString::number(i+1) + " [RPM]";
        data += ",";
    }
    data += "End VX [mm/s],";
    data += "End VY [mm/s],";
    data += "End VZ [mm/s],";
    data += "End WX [deg/s],";
    data += "End WY [deg/s],";
    data += "End WZ [deg/s],";
    for (uint i = 0; i < NUM_JOINT; i++){
        data += "Joint Current" + QString::number(i+1) + " [mA]";
        data += ",";
    }
    data += "End X Cmd [mm],";
    data += "End Y Cmd [mm],";
    data += "End Z Cmd [mm],";
    data += "End Roll Cmd [deg],";
    data += "End Pitch Cmd [deg],";
    data += "End Yaw Cmd [deg],";
    for (uint i = 0; i < NUM_JOINT; i++){
        data += "Joint Cmd" + QString::number(i+1) + " [deg]";
        data += ",";
    }
    for (uint i = 0; i < NUM_JOINT; i++){
        data += "Joint Tor" + QString::number(i+1) + " [Nm]";
        data += ",";
    }
    for (uint i = 0; i < NUM_JOINT; i++){
        data += "Joint Dist" + QString::number(i+1) + " [Nm]";
        data += ",";
    }
    data += "\n";
    dataControl->logger->write(data);
    dataControl->logging_start = true;
    ui->txtLoggingState->setText("Logging...");
}

void MainWindow::btnLoggingStopClicked()
{
    qDebug() << "Logging Stop";
    delete dataControl->logger;
    dataControl->logging_start = false;
    ui->txtLoggingState->setText("Wait...");
}
