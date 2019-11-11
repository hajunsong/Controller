#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent) : QMainWindow(parent), ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    connect(ui->btnConnect, SIGNAL(clicked()), this, SLOT(btnConnectClicked()));
    connect(ui->btnDisconnect, SIGNAL(clicked()), this, SLOT(btnDisconnectClicked()));

    tcpClient = new TcpClient();
    connect(tcpClient->socket, SIGNAL(connected()), this, SLOT(onConnectServer()));
    connect(tcpClient->socket, SIGNAL(readyRead()), this, SLOT(readMessage()));
    connect(tcpClient->socket, SIGNAL(disconnected()), this, SLOT(disConnectServer()));

    ui->btnDisconnect->setEnabled(false);

    connect(ui->btnSetInit, SIGNAL(clicked()), this, SLOT(btnSetInitClicked()));

    dataControl = new DataControl();

    connect(ui->btnServoOn, SIGNAL(clicked()), this, SLOT(btnServOnClicked()));
    connect(ui->btnServoOff, SIGNAL(clicked()), this, SLOT(btnServoOffClicked()));

    connect(ui->btnJ1N, SIGNAL(clicked()), this, SLOT(btnJogMoveClicked()));
    connect(ui->btnJ1P, SIGNAL(clicked()), this, SLOT(btnJogMoveClicked()));
    connect(ui->btnJ2N, SIGNAL(clicked()), this, SLOT(btnJogMoveClicked()));
    connect(ui->btnJ2P, SIGNAL(clicked()), this, SLOT(btnJogMoveClicked()));
    connect(ui->btnJ3N, SIGNAL(clicked()), this, SLOT(btnJogMoveClicked()));
    connect(ui->btnJ3P, SIGNAL(clicked()), this, SLOT(btnJogMoveClicked()));
    connect(ui->btnJ4N, SIGNAL(clicked()), this, SLOT(btnJogMoveClicked()));
    connect(ui->btnJ4P, SIGNAL(clicked()), this, SLOT(btnJogMoveClicked()));
    connect(ui->btnJ5N, SIGNAL(clicked()), this, SLOT(btnJogMoveClicked()));
    connect(ui->btnJ5P, SIGNAL(clicked()), this, SLOT(btnJogMoveClicked()));
    connect(ui->btnJ6N, SIGNAL(clicked()), this, SLOT(btnJogMoveClicked()));
    connect(ui->btnJ6P, SIGNAL(clicked()), this, SLOT(btnJogMoveClicked()));

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

    connect(ui->btnJSet, SIGNAL(clicked()), this, SLOT(btnSetJCommandClicked()));
    connect(ui->btnCSet, SIGNAL(clicked()), this, SLOT(btnSetCCommandClicked()));

    connect(ui->btnC1N, SIGNAL(clicked()), this, SLOT(btnJogMoveClicked()));
    connect(ui->btnC2N, SIGNAL(clicked()), this, SLOT(btnJogMoveClicked()));
    connect(ui->btnC3N, SIGNAL(clicked()), this, SLOT(btnJogMoveClicked()));
    connect(ui->btnC4N, SIGNAL(clicked()), this, SLOT(btnJogMoveClicked()));
    connect(ui->btnC5N, SIGNAL(clicked()), this, SLOT(btnJogMoveClicked()));
    connect(ui->btnC6N, SIGNAL(clicked()), this, SLOT(btnJogMoveClicked()));
    connect(ui->btnC1P, SIGNAL(clicked()), this, SLOT(btnJogMoveClicked()));
    connect(ui->btnC2P, SIGNAL(clicked()), this, SLOT(btnJogMoveClicked()));
    connect(ui->btnC3P, SIGNAL(clicked()), this, SLOT(btnJogMoveClicked()));
    connect(ui->btnC4P, SIGNAL(clicked()), this, SLOT(btnJogMoveClicked()));
    connect(ui->btnC5P, SIGNAL(clicked()), this, SLOT(btnJogMoveClicked()));
    connect(ui->btnC6P, SIGNAL(clicked()), this, SLOT(btnJogMoveClicked()));

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

    customSettings = new CustomSettings(ui);
    customSettings->loadConfigFile();

    ui->btnRun->setEnabled(false);
    ui->btnReady->setEnabled(false);

    keyInputClass = new KeyInputClass(ui);

    ui->tabWidget->setCurrentIndex(0);

//    ui->gbRobotConfig->setEnabled(true);
//    ui->btnSetInit->setEnabled(true);
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
}

void MainWindow::btnConnectClicked(){
    tcpClient->setIpAddress(ui->txtIP->text());
    tcpClient->setPort(ui->txtPORT->text().toUShort());
    emit tcpClient->connectToServer();
}

void MainWindow::btnDisconnectClicked(){
    ui->btnConnect->setEnabled(true);
    ui->btnDisconnect->setEnabled(false);
    ui->gbRobotConfig->setEnabled(false);
    componentEnable(false);
    tcpClient->socket->close();
    qDebug() << "Disconnected Server";

    ui->btnRun->setEnabled(false);
}

void MainWindow::btnSetInitClicked()
{
    txData.clear();
    txData.append(Qt::Key_N);
    txData.append(Qt::Key_D);
    txData.append(static_cast<char>(ui->cbNumJoint->currentText().toInt()));
    txData.append(static_cast<char>(ui->cbNumDOF->currentText().toInt()));
    txData.append(static_cast<char>(ui->cbModuleType->currentIndex()));
    txData.append(static_cast<char>(JointOpMode[(ui->cbJointMode->currentIndex())]));
    txData.append(Qt::Key_N);
    txData.append(Qt::Key_E);

    qDebug() << "txData : " << txData;

    tcpClient->socket->write(txData);

    int row = 6;
    model = new QStandardItemModel(row, NUM_JOINT, this);
    ui->tvRobotInfor->setModel(model);

    for(int i = 0; i < row; i++){
        for(int j = 0; j < NUM_JOINT; j++){
            QModelIndex index = model->index(i, j, QModelIndex());
            model->setData(index, 0);
        }
    }

    QStringList vHeader;
    vHeader.append("Actual Pos [deg]");
    vHeader.append("Actual Pose [mm, deg]");
    vHeader.append("Command Pos [deg]");
    vHeader.append("Desired Pose [mm, deg]");
    vHeader.append("Calculate Pose [mm, deg]");
    vHeader.append("Present Vel [RPM]");
    vHeader.append("Present Cur [mA]");
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
    hHeader.append("X");
    hHeader.append("Y");
    hHeader.append("Z");
    hHeader.append("Acc time");
    pathModel->setHorizontalHeaderLabels(hHeader);
}

void MainWindow::btnServOnClicked()
{
    dataControl->ClientToServer.opMode = OpMode::ServoOnOff;
    dataControl->ClientToServer.subMode = Servo::On;
    memset(dataControl->ClientToServer.desiredJoint, 0, sizeof(double)*NUM_JOINT);
    memset(dataControl->ClientToServer.desiredCartesian, 0, sizeof(double)*NUM_DOF);

    txData.clear();
    txData.append(Qt::Key_N);
    txData.append(Qt::Key_S);
    txData.append(dataControl->ClientToServer.opMode);
    txData.append(dataControl->ClientToServer.subMode);
    for(int i = 0; i < NUM_JOINT; i++){
        txData.append(QByteArray::number(dataControl->ClientToServer.desiredJoint[i], 'f', 6));
    }
    for(int i = 0; i < NUM_DOF; i++){
        txData.append(QByteArray::number(dataControl->ClientToServer.desiredCartesian[i], 'f', 6));
    }
    txData.append(Qt::Key_N);
    txData.append(Qt::Key_E);


    qDebug() << "txData : " << txData;

    tcpClient->socket->write(txData);
}

void MainWindow::btnServoOffClicked()
{
    dataControl->ClientToServer.opMode = OpMode::ServoOnOff;
    dataControl->ClientToServer.subMode = Servo::Off;
    memset(dataControl->ClientToServer.desiredJoint, 0, sizeof(double)*NUM_JOINT);
    memset(dataControl->ClientToServer.desiredCartesian, 0, sizeof(double)*NUM_DOF);

    txData.clear();
    txData.append(Qt::Key_N);
    txData.append(Qt::Key_S);
    txData.append(dataControl->ClientToServer.opMode);
    txData.append(dataControl->ClientToServer.subMode);
    for(int i = 0; i < NUM_JOINT; i++){
        txData.append(QByteArray::number(dataControl->ClientToServer.desiredJoint[i], 'f', 6));
    }
    for(int i = 0; i < NUM_DOF; i++){
        txData.append(QByteArray::number(dataControl->ClientToServer.desiredCartesian[i], 'f', 6));
    }
    txData.append(Qt::Key_N);
    txData.append(Qt::Key_E);

    qDebug() << "txData : " << txData;

    tcpClient->socket->write(txData);
}

void MainWindow::btnJogMoveClicked()
{
    QString objName = sender()->objectName();
    QString moduleStr = objName.at(objName.length() - 2);
    int moduleIndex = moduleStr.toInt();
    QString PN = objName.at(objName.length() - 1);
    QString CJ = objName.at(3);

    dataControl->ClientToServer.opMode = CJ == "J" ? OpMode::JointMove : OpMode::CartesianMove;
    dataControl->ClientToServer.subMode = CJ == "J" ? Motion::JogMotion : Motion::CartesianJogMotion;
    memset(dataControl->ClientToServer.desiredJoint, 0, sizeof(double)*NUM_JOINT);
    memset(dataControl->ClientToServer.desiredCartesian, 0, sizeof(double)*NUM_DOF);

    txData.clear();
    txData.append(Qt::Key_N);
    txData.append(Qt::Key_S);
    txData.append(dataControl->ClientToServer.opMode);
    txData.append(dataControl->ClientToServer.subMode);
    for(int i = 0; i < NUM_JOINT; i++){
        if (CJ == "J"){
            if (i + 1 == moduleIndex){
                dataControl->ClientToServer.desiredJoint[i] = PN == "N" ? -5 : 5;
            }
        }
        txData.append(QByteArray::number(dataControl->ClientToServer.desiredJoint[i], 'f', 6));
    }
    for(int i = 0; i < NUM_DOF; i++){
        if (CJ == "C"){
            if (i + 1 == moduleIndex){
                if (i < 3){
                    dataControl->ClientToServer.desiredCartesian[i] = PN == "N" ? -10 : 10;
                }
                else{
                    dataControl->ClientToServer.desiredCartesian[i] = PN == "N" ? -5 : 5;
                }
            }
        }
        txData.append(QByteArray::number(dataControl->ClientToServer.desiredCartesian[i], 'f', 6));
    }
    txData.append(Qt::Key_N);
    txData.append(Qt::Key_E);

    qDebug() << "txData : " << txData;

    tcpClient->socket->write(txData);
}

void MainWindow::btnSetJCommandClicked()
{
    if (cmdJointRel || cmdJointAbs)
    {
        dataControl->ClientToServer.opMode = DataControl::OpMode::JointMove;

        if (cmdJointRel){
            dataControl->ClientToServer.subMode = Motion::JogMotion;
        }

        if (cmdJointAbs){
            dataControl->ClientToServer.subMode = Motion::JointMotion;
        }
    }

    memset(dataControl->ClientToServer.desiredJoint, 0, sizeof(double)*NUM_JOINT);
    memset(dataControl->ClientToServer.desiredCartesian, 0, sizeof(double)*NUM_DOF);

    txData.clear();
    txData.append(Qt::Key_N);
    txData.append(Qt::Key_S);
    txData.append(dataControl->ClientToServer.opMode);
    txData.append(dataControl->ClientToServer.subMode);
    for(int i = 0; i < NUM_JOINT; i++)
    {
        dataControl->ClientToServer.desiredJoint[i] = txtJCmd[i]->text().toDouble();
        txData.append(QByteArray::number(dataControl->ClientToServer.desiredJoint[i], 'f', 6));
    }
    for(int i = 0; i < NUM_DOF; i++)
    {
        txData.append(QByteArray::number(dataControl->ClientToServer.desiredCartesian[i], 'f', 6));
    }
    txData.append(Qt::Key_N);
    txData.append(Qt::Key_E);

    qDebug() << "txData : " << txData;

    tcpClient->socket->write(txData);

    setTxtCommandClear();
}

void MainWindow::btnSetCCommandClicked()
{
    if(cmdCartRel || cmdCartAbs)
    {
        dataControl->ClientToServer.opMode = DataControl::OpMode::CartesianMove;

        if (cmdCartRel){
            dataControl->ClientToServer.subMode = Motion::CartesianJogMotion;
        }

        if (cmdCartAbs){
            dataControl->ClientToServer.subMode = Motion::CartesianMotion;
        }
    }

    memset(dataControl->ClientToServer.desiredJoint, 0, sizeof(double)*NUM_JOINT);
    memset(dataControl->ClientToServer.desiredCartesian, 0, sizeof(double)*NUM_DOF);

    txData.clear();
    txData.append(Qt::Key_N);
    txData.append(Qt::Key_S);
    txData.append(dataControl->ClientToServer.opMode);
    txData.append(dataControl->ClientToServer.subMode);
    for(int i = 0; i < NUM_DOF; i++)
    {
        dataControl->ClientToServer.desiredCartesian[i] = txtCCmd[i]->text().toDouble();
        txData.append(QByteArray::number(dataControl->ClientToServer.desiredCartesian[i], 'f', 6));
    }
    txData.append(Qt::Key_N);
    txData.append(Qt::Key_E);

    qDebug() << "txData : " << txData;

    tcpClient->socket->write(txData);

    setTxtCommandClear();
}

void MainWindow::onConnectServer(){
    qDebug() << "Connected Server";
    ui->btnConnect->setEnabled(false);
    ui->btnDisconnect->setEnabled(true);
    ui->gbRobotConfig->setEnabled(true);
    tcpClient->socket->write("O");
}

void MainWindow::readMessage(){
    QByteArray rxData = tcpClient->socket->readAll();
//    qDebug() << "Data size : " << rxData.size();
//    qDebug() << rxData;

    size_t size = static_cast<size_t>(rxData.size());
    if (size == 1){
        if(rxData.at(0) == 'X'){
            qDebug() << "Client & Server configuration is difference";
            disConnectServer();
        }
        else if(rxData.at(0) == 'S'){
//            qDebug() << "Client & Server configuration check complete";
            componentEnable(true);
        }
    }
    else if (size == SERVER_TO_CLIENT_LEN)
    {
        char *pData = rxData.data();
        int indx = NRMK_SOCKET_TOKEN_SIZE;
        memcpy(&dataControl->ServerToClient, pData + indx, DATA_INDEX_LEN);
        indx += DATA_INDEX_LEN;
        memcpy(dataControl->ServerToClient.presentJointPosition, pData + indx, JOINT_POSITION_LEN*NUM_JOINT);
        indx += JOINT_POSITION_LEN*NUM_JOINT;
        memcpy(dataControl->ServerToClient.presentCartesianPose, pData + indx, CARTESIAN_POSE_LEN*NUM_DOF);
        indx += CARTESIAN_POSE_LEN*NUM_DOF;
        memcpy(dataControl->ServerToClient.desiredJointPosition, pData + indx, JOINT_COMMAND_LEN*NUM_JOINT);
        indx += JOINT_COMMAND_LEN*NUM_JOINT;
        memcpy(dataControl->ServerToClient.desiredCartesianPose, pData + indx, CARTESIAN_COMMAND_LEN*NUM_DOF);
        indx += CARTESIAN_COMMAND_LEN*NUM_DOF;
        memcpy(dataControl->ServerToClient.calculateCartesianPose, pData + indx, CARTESIAN_CALCULATE_LEN*NUM_DOF);
        indx += CARTESIAN_CALCULATE_LEN*NUM_DOF;
        memcpy(dataControl->ServerToClient.presentJointVelocity, pData + indx, JOINT_VELOCITY_LEN*NUM_JOINT);
        indx += JOINT_VELOCITY_LEN*NUM_JOINT;
        memcpy(dataControl->ServerToClient.presentJointCurrent, pData + indx, JOINT_CURRENT_LEN*NUM_JOINT);
        indx += JOINT_CURRENT_LEN*NUM_JOINT;

        memcpy(&dataControl->ServerToClient.time, pData + indx, TIME_LEN);
        indx += TIME_LEN;
        memcpy(&dataControl->ServerToClient.dxl_time, pData + indx, TIME_LEN);
        indx += TIME_LEN;
        memcpy(&dataControl->ServerToClient.ik_time, pData + indx, TIME_LEN);
        indx += TIME_LEN;

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
            model->setData(index, dataControl->ServerToClient.desiredJointPosition[i]);
            ui->tvRobotInfor->update(index);
        }

        for(int i = 0; i < NUM_DOF; i++){
            QModelIndex index = model->index(3, i);
            model->setData(index, dataControl->ServerToClient.desiredCartesianPose[i]);
            ui->tvRobotInfor->update(index);
        }

        for(int i = 0; i < NUM_DOF; i++){
            QModelIndex index = model->index(4, i);
            model->setData(index, dataControl->ServerToClient.calculateCartesianPose[i]);
            ui->tvRobotInfor->update(index);
        }

        for(int i = 0; i < NUM_JOINT; i++){
            QModelIndex index = model->index(5, i);
            model->setData(index, dataControl->ServerToClient.presentJointVelocity[i]);
            ui->tvRobotInfor->update(index);
        }

        for(int i = 0; i < NUM_JOINT; i++){
            QModelIndex index = model->index(6, i);
            model->setData(index, dataControl->ServerToClient.presentJointCurrent[i]);
            ui->tvRobotInfor->update(index);
        }

        ui->txtTime->setText(QString::number(dataControl->ServerToClient.time, 'f', 6));
        ui->txtDxlTime->setText(QString::number(dataControl->ServerToClient.dxl_time, 'f', 6));
        ui->txtIKTime->setText(QString::number(dataControl->ServerToClient.ik_time, 'f', 6));
    }
}

void MainWindow::disConnectServer(){
    ui->btnDisconnect->animateClick();
}

void MainWindow::componentEnable(bool enable){
    ui->gbRobotInfor->setEnabled(enable);
    ui->gbServoControl->setEnabled(enable);
    ui->gbCartMoveCommand->setEnabled(enable);
    ui->gbJointMoveCommand->setEnabled(enable);
    ui->gbTrajectory->setEnabled(enable);

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
    txData.clear();
    txData.append(Qt::Key_N);
    txData.append(Qt::Key_U);

    txData.append(DataControl::CmdType::RunCmd);
    txData.append(DataControl::OpMode::RunMode);

    if (ui->cbRepeat->isChecked()){
        txData.append(-1);
    }
    else{
        txData.append(1);
    }

    txData.append(Qt::Key_N);
    txData.append(Qt::Key_E);

    qDebug() << "txData : " << txData;

    tcpClient->socket->write(txData);
}

void MainWindow::btnStopClicked()
{
    txData.clear();
    txData.append(Qt::Key_N);
    txData.append(Qt::Key_U);

    txData.append(DataControl::CmdType::StopCmd);
    txData.append(DataControl::OpMode::Wait);

    if (ui->cbRepeat->isChecked()){
        txData.append(-1);
    }
    else{
        txData.append(1);
    }

    txData.append(Qt::Key_N);
    txData.append(Qt::Key_E);

    qDebug() << "txData : " << txData;

    tcpClient->socket->write(txData);

    ui->btnRun->setEnabled(false);
    ui->btnReady->setEnabled(false);
}

void MainWindow::btnReadyClicked()
{
    txData.clear();
    txData.append(Qt::Key_N);
    txData.append(Qt::Key_U);

    txData.append(DataControl::CmdType::ReadyCmd);
    txData.append(DataControl::OpMode::ReadyMode);

    if (ui->cbRepeat->isChecked()){
        txData.append(-1);
    }
    else{
        txData.append(1);
    }

    txData.append(Qt::Key_N);
    txData.append(Qt::Key_E);

    qDebug() << "txData : " << txData;

    tcpClient->socket->write(txData);

    ui->btnRun->setEnabled(true);
}

void MainWindow::keyPressEvent(QKeyEvent *event)
{
    keyInputClass->InputKeyboard(event);
}

void MainWindow::closeEvent(QCloseEvent*){
    qDebug() << "Closed MainWindow";
}

void MainWindow::btnPathApplyClicked()
{
    txData.clear();
    txData.append(Qt::Key_N);
    txData.append(Qt::Key_U);

    txData.append(DataControl::CmdType::PathCmd);
    txData.append(DataControl::OpMode::PathGenerateMode);

    int8_t tvRow = static_cast<int8_t>(pathModel->rowCount());
    int8_t tvCol = static_cast<int8_t>(pathModel->columnCount());

    txData.append(tvRow);
    txData.append(tvCol);

    for(int i = 0; i < tvRow; i++){
        for(int j = 0; j < tvCol; j++){
            txData.append(',');
            txData.append(QByteArray::number(pathModel->data(pathModel->index(i, j)).toDouble() ,'f', 6));
        }
    }
    txData.append(',');

    txData.append(Qt::Key_N);
    txData.append(Qt::Key_E);

    qDebug() << "txData : " << txData;

    tcpClient->socket->write(txData);

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
