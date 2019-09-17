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

//    connect(ui->cbJointPath, SIGNAL(clicked()), this, SLOT(cbJointPathClicked()));
//    connect(ui->cbCartesianPath, SIGNAL(clicked()), this, SLOT(cbCartesianPathClicked()));

//    connect(ui->btnFileLoad, SIGNAL(clicked()), this, SLOT(btnFileLoadClicked()));
//    ui->btnFileLoad->hide();
//    connect(ui->btnPathClear, SIGNAL(clicked()), this, SLOT(btnPathClearClicked()));
//    connect(ui->btnPathApply, SIGNAL(clicked()), this, SLOT(btnPathApplyClicked()));
//    connect(ui->btnPathInsert, SIGNAL(clicked()), this, SLOT(btnPathInsertClicked()));
//    connect(ui->btnPathDelete, SIGNAL(clicked()), this, SLOT(btnPathDeleteClicked()));
//    connect(ui->btnPathAppend, SIGNAL(clicked()), this, SLOT(btnPathAppendClicked()));

//    jointPathModel = new QStandardItemModel(0, NUM_JOINT + 1, this);
//    cartPathModel = new QStandardItemModel(0, NUM_DOF + 1, this);
//    connect(ui->tvPathData, SIGNAL(clicked(QModelIndex)), this, SLOT(tvCellClicked(QModelIndex)));
//    connect(ui->tvPathData->horizontalHeader(), SIGNAL(sectionClicked(int)), this, SLOT(horizontalSectionClicked(int)));
//    connect(ui->tvPathData->verticalHeader(), SIGNAL(sectionClicked(int)), this, SLOT(verticalSectionClicked(int)));
//    connect(ui->tvPathData->horizontalHeader(), SIGNAL(sectionPressed(int)), this, SLOT(horizontalSectionPressed(int)));
//    connect(ui->tvPathData->verticalHeader(), SIGNAL(sectionPressed(int)), this, SLOT(verticalSectionPressed(int)));

    ui->txtNumJoint->setText(QString::number(NUM_JOINT));
    ui->txtNumDof->setText(QString::number(NUM_DOF));
    ui->txtModuleType->setText(MODULE_TYPE == 1 ? "FAR" : MODULE_TYPE == 2 ? "SEA" : "JS-R8");
    ui->txtCommType->setText(COMM_TYPE == 1 ? "RS485" : "RS232");

    QStringList items;
    items.append("");
    items.append("pick up motion (joint path command)");
    items.append("pick up motion (cartesian path command)");
    items.append("rectangle motion (joint path command)");
    items.append("rectangle motion (cartesian path command)");
    ui->cbPlaylist->addItems(items);

    connect(ui->btnRun, SIGNAL(clicked()), this, SLOT(btnRunClicked()));
    connect(ui->btnStop, SIGNAL(clicked()), this, SLOT(btnStopClicked()));

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
}

MainWindow::~MainWindow()
{
    customSettings->saveConfigFile();
    txtJCmd.clear();
    txtCCmd.clear();
    delete ui;
    delete tcpClient;
    delete dataControl;
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
}
void MainWindow::btnSetInitClicked()
{
    txData.clear();
    txData.append(Qt::Key_N);
    txData.append(Qt::Key_D);
    txData.append(NUM_JOINT);
    txData.append(NUM_DOF);
    txData.append(MODULE_TYPE);
    txData.append(Qt::Key_N);
    txData.append(Qt::Key_E);

    qDebug() << "txData : " << txData;

    tcpClient->socket->write(txData);

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
    vHeader.append("Actual Pos [deg]");
    vHeader.append("Actual Pose [mm, deg]");
    vHeader.append("Command Pos [deg]");
    vHeader.append("Desired Pose [mm, deg]");
    vHeader.append("Calculate Pose [mm, deg]");
    model->setVerticalHeaderLabels(vHeader);
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
            qDebug() << "Client & Server configuration check complete";
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
    ui->gbPathData->setEnabled(enable);
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

//void MainWindow::cbJointPathClicked()
//{
//    ui->cbCartesianPath->setChecked(!ui->cbJointPath->isChecked());
//    ui->tvPathData->setModel(jointPathModel);

//    if (jointPathModel->rowCount() == 0){
//        btnPathAppendClicked();
//    }
//}

//void MainWindow::cbCartesianPathClicked()
//{
//    ui->cbJointPath->setChecked(!ui->cbCartesianPath->isChecked());
//    ui->tvPathData->setModel(cartPathModel);

//    if (cartPathModel->rowCount() == 0){
//        btnPathAppendClicked();
//    }
//}

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

void MainWindow::btnFileLoadClicked()
{
//    QString fileName = QFileDialog::getOpenFileName(this, QString::fromLocal8Bit("텍스트 파일"),"../RobotControllerUI/data","txt (*.txt)");
//    qDebug() << fileName;
//    if (fileName.size() > 1){
//        if (ui->cbJointPath->isChecked()){
//            load_data(fileName.toStdString(), &jointPathTxtData);
//            int row = static_cast<int>(jointPathTxtData.size());
//            int col = static_cast<int>(jointPathTxtData[0].size());
//            jointPathModel->removeRows(0, jointPathModel->rowCount());

//            while(1){
//                if (col == jointPathModel->columnCount()){
//                    break;
//                }
//                else{
//                    jointPathModel->insertColumn(jointPathModel->columnCount());
//                }
//            }

//            jointPathModel->insertRows(0, row);

//            for(int i = 0; i < row; i++){
//                for(int j = 0; j < col; j++){
//                    QModelIndex index = jointPathModel->index(i, j);
//                    jointPathModel->setData(index, QString::number(jointPathTxtData[static_cast<size_t>(i)][static_cast<size_t>(j)]));
//                }
//            }
//        }
//        else{
//            load_data(fileName.toStdString(), &cartPathTxtData);
//            int row = static_cast<int>(cartPathTxtData.size());
//            int col = static_cast<int>(cartPathTxtData[0].size());
//            cartPathModel->removeRows(0, cartPathModel->rowCount());

//            while(1){
//                if (col == cartPathModel->columnCount()){
//                    break;
//                }
//                else{
//                    cartPathModel->insertColumn(cartPathModel->columnCount());
//                }
//            }

//            cartPathModel->insertRows(0, row);

//            for(int i = 0; i < row; i++){
//                for(int j = 0; j < col; j++){
//                    QModelIndex index = cartPathModel->index(i, j);
//                    cartPathModel->setData(index, QString::number(cartPathTxtData[static_cast<size_t>(i)][static_cast<size_t>(j)]));
//                }
//            }
//        }
    //    }
}

void MainWindow::btnRunClicked()
{
    char indx = static_cast<char>(ui->cbPlaylist->currentIndex());

    txData.clear();
    txData.append(Qt::Key_N);
    txData.append(Qt::Key_U);

    txData.append(indx);

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

    txData.append(-1);

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

//void MainWindow::btnPathApplyClicked()
//{
//    txData.clear();
//    txData.append(Qt::Key_N);
//    txData.append(Qt::Key_U);

//    if (ui->cbJointPath->isChecked()){
//        txData.append(DataControl::PathDataType::JointPath);   // path type
//        txData.append(-1);  // cycle count
//        uint16_t tvRow = static_cast<uint16_t>(jointPathModel->rowCount());
//        uint16_t tvCol = static_cast<uint16_t>(jointPathModel->columnCount());
//        char buf[4];
//        memcpy(buf, &tvRow, 2);
//        memcpy(buf + 2, &tvCol, 2);
//        txData.append(buf, 4);
//        for(int i = 0; i < tvRow; i++){
//            for(int j = 0; j < tvCol; j++){
//                txData.append(QByteArray::number(
//                                  jointPathModel->data(jointPathModel->index(i, j)).toDouble(),
//                                  'f', 6));
//            }
//        }
//    }
//    else{
//        txData.append(DataControl::PathDataType::CartPath);   // path type
//        txData.append(-1);  // cycle count
//        uint16_t tvRow = static_cast<uint16_t>(cartPathModel->rowCount());
//        uint16_t tvCol = static_cast<uint16_t>(cartPathModel->columnCount());
//        char tvRowColChar[4];
//        memcpy(tvRowColChar, &tvRow, 2);
//        memcpy(tvRowColChar + 2, &tvCol, 2);
//        txData.append(tvRowColChar);
//        for(int i = 0; i < tvRow; i++){
//            for(int j = 0; j < tvCol; j++){
//                txData.append(QByteArray::number(
//                                  cartPathModel->data(cartPathModel->index(i, j)).toDouble(),
//                                  'f', 6));
//            }
//        }
//    }

//    txData.append(Qt::Key_N);
//    txData.append(Qt::Key_E);

////    qDebug() << "txData : " << txData;

//    tcpClient->socket->write(txData);
//}

//void MainWindow::btnPathClearClicked()
//{
//    if (ui->cbJointPath->isChecked()){
//        jointPathModel->removeRows(0, jointPathModel->rowCount());
//    }
//    else{
//        cartPathModel->removeRows(0, jointPathModel->rowCount());
//    }
//}

//void MainWindow::btnPathInsertClicked()
//{
//    if (ui->cbJointPath->isChecked()){
//        if (rowClickedIndex >= 0){
//            jointPathModel->insertRow(rowClickedIndex);
//        }
//        if (colClickedIndex >= 0){
//            jointPathModel->insertColumn(colClickedIndex);
//        }
//    }
//    else{
//        if (rowClickedIndex >= 0){
//            cartPathModel->insertRow(rowClickedIndex);
//        }
//        if (colClickedIndex >= 0){
//            cartPathModel->insertColumn(colClickedIndex);
//        }
//    }
//}

//void MainWindow::btnPathDeleteClicked()
//{
//    if (rowClickedIndex >= 0 && rowPressedIndex >= 0){
//        if (ui->cbJointPath->isChecked()){
//            jointPathModel->removeRows(rowPressedIndex, abs(rowClickedIndex - rowPressedIndex) + 1);
//        }
//        else{
//            cartPathModel->removeRows(rowPressedIndex, abs(rowClickedIndex - rowPressedIndex) + 1);
//        }
//        rowClickedIndex = -1;
//        rowPressedIndex = -1;
//    }
//    if (colClickedIndex >= 0 && colPressedIndex >= 0){
//        if (ui->cbJointPath->isChecked()){
//            jointPathModel->removeColumns(colPressedIndex, abs(colClickedIndex - colPressedIndex) + 1);
//        }
//        else{
//            cartPathModel->removeColumns(colPressedIndex, abs(colClickedIndex - colPressedIndex) + 1);
//        }
//        colClickedIndex = -1;
//        colPressedIndex = -1;
//    }
//}

//void MainWindow::btnPathAppendClicked()
//{
//    if (ui->cbJointPath->isChecked()){
//        if (rowClickedIndex >= 0){
//            jointPathModel->insertRow(jointPathModel->rowCount());
//        }
//        if (colClickedIndex >= 0){
//            jointPathModel->insertColumn(jointPathModel->columnCount());
//        }
//    }
//    else{
//        if (rowClickedIndex >= 0){
//            cartPathModel->insertRow(cartPathModel->rowCount());
//        }
//        if (colClickedIndex >= 0){
//            cartPathModel->insertColumn(cartPathModel->columnCount());
//        }
//    }
//}

//void MainWindow::tvCellClicked(const QModelIndex &index)
//{
//    rowClickedIndex = index.row();
//    colClickedIndex = index.column();
////    qDebug() << "Clicked row : " << rowClickedIndex << ", col : " << colClickedIndex;
//}

//void MainWindow::horizontalSectionClicked(int index)
//{
//    colClickedIndex = index;
////    qDebug() << "Clicked horizontal : " << index;
//    rowClickedIndex = -1;
//}

//void MainWindow::verticalSectionClicked(int index)
//{
//    rowClickedIndex = index;
////    qDebug() << "Clicked vertical : " << index;
//    colClickedIndex = -1;
//}

//void MainWindow::horizontalSectionPressed(int index)
//{
//    colPressedIndex = index;
////    qDebug() << "Pressed horizontal : " << index;
//    rowClickedIndex = -1;
//    rowPressedIndex = -1;
//}

//void MainWindow::verticalSectionPressed(int index)
//{
//    rowPressedIndex = index;
////    qDebug() << "Pressed vertical : " << index;
//    colClickedIndex = -1;
//    colPressedIndex = -1;
//}
