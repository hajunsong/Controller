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

    connect(ui->btnServoOn, SIGNAL(clicked()), this, SLOT(btnServOnClicked()));
    connect(ui->btnServoOff, SIGNAL(clicked()), this, SLOT(btnServoOffClicked()));

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

    txtJCmd.push_back(ui->txtJCmd1);
    txtJCmd.push_back(ui->txtJCmd2);
    txtJCmd.push_back(ui->txtJCmd3);
    txtJCmd.push_back(ui->txtJCmd4);
    txtJCmd.push_back(ui->txtJCmd5);
    txtJCmd.push_back(ui->txtJCmd6);

    for(int i = 0; i < txtJCmd.length(); i++)
    {
        txtJCmd[i]->setText("0");
    }
}

MainWindow::~MainWindow()
{
    txtJCmd.clear();
    delete ui;
    delete tcpClient;
}

void MainWindow::btnConnectClicked(){
    tcpClient->setIpAddress(ui->txtIP->text());
    tcpClient->setPort(ui->txtPORT->text().toUShort());
    emit tcpClient->connectToServer();
}

void MainWindow::btnDisconnectClicked(){
    ui->btnConnect->setEnabled(false);
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
    txData.append(static_cast<char>(ui->cbNumJoint->currentText().toInt()));
    txData.append(static_cast<char>(ui->cbNumDOF->currentText().toInt()));
    txData.append(static_cast<char>(ui->cbModuleType->currentIndex()));
    txData.append(static_cast<char>(JointOpMode[(ui->cbJointMode->currentIndex())]));
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
    vHeader.append("Present Joint Pos [deg]");
    vHeader.append("Present End Pose [mm, deg]");
    vHeader.append("Present Joint Vel [RPM]");
    vHeader.append("Present Joint Cur [mA]");
    vHeader.append("Present End Vel [deg/s]");
    model->setVerticalHeaderLabels(vHeader);
}

void MainWindow::btnServOnClicked()
{
    dataControl->ClientToServer.opMode = OpMode::ServoOnOff;
    dataControl->ClientToServer.subMode = Servo::On;
    memset(dataControl->ClientToServer.desiredJoint, 0, sizeof(double)*NUM_JOINT);
    memset(dataControl->ClientToServer.desiredPose, 0, sizeof(double)*NUM_DOF);

    txData.clear();
    txData.append(Qt::Key_N);
    txData.append(Qt::Key_S);
    txData.append(dataControl->ClientToServer.opMode);
    txData.append(dataControl->ClientToServer.subMode);
    for(int i = 0; i < NUM_JOINT; i++){
        txData.append(QByteArray::number(dataControl->ClientToServer.desiredJoint[i], 'f', 6));
    }
    for(int i = 0; i < NUM_DOF; i++){
        txData.append(QByteArray::number(dataControl->ClientToServer.desiredPose[i], 'f', 6));
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
    memset(dataControl->ClientToServer.desiredPose, 0, sizeof(double)*NUM_DOF);

    txData.clear();
    txData.append(Qt::Key_N);
    txData.append(Qt::Key_S);
    txData.append(dataControl->ClientToServer.opMode);
    txData.append(dataControl->ClientToServer.subMode);
    for(int i = 0; i < NUM_JOINT; i++){
        txData.append(QByteArray::number(dataControl->ClientToServer.desiredJoint[i], 'f', 6));
    }
    for(int i = 0; i < NUM_DOF; i++){
        txData.append(QByteArray::number(dataControl->ClientToServer.desiredPose[i], 'f', 6));
    }
    txData.append(Qt::Key_N);
    txData.append(Qt::Key_E);

    qDebug() << "txData : " << txData;

    tcpClient->socket->write(txData);
}

void MainWindow::btnSetJCommandClicked() {
    dataControl->ClientToServer.opMode = DataControl::OpMode::JointMove;
    QString objName = sender()->objectName();
    if(objName.contains("P") || objName.contains("N")){
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

    txData.clear();
    txData.append(Qt::Key_N);
    txData.append(Qt::Key_S);
    txData.append(dataControl->ClientToServer.opMode);
    txData.append(dataControl->ClientToServer.subMode);
    for(int i = 0; i < NUM_JOINT; i++)
    {
        txData.append(',');
        txData.append(QByteArray::number(dataControl->ClientToServer.desiredJoint[i], 'f', 6));
    }
    txData.append(',');
    txData.append(Qt::Key_N);
    txData.append(Qt::Key_E);

    qDebug() << "txData : " << txData;

    tcpClient->socket->write(txData);
}


void MainWindow::onConnectServer(){
    qDebug() << "Connected Server";
    ui->btnConnect->setDisabled(false);
    ui->btnDisconnect->setEnabled(true);
    ui->gbRobotConfig->setEnabled(true);
    tcpClient->socket->write("O");

    ui->cbNumJoint->setCurrentIndex(2);
    ui->cbNumDOF->setCurrentIndex(2);
    ui->cbModuleType->setCurrentIndex(2);
    ui->cbJointMode->setCurrentIndex(4);
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
    else
    {
        if (rxData.at(0) == 'N' && rxData.at(1) == 'S'){
            QByteArrayList rxDataSplit = rxData.split('=');
            for(int i = 1; i < rxDataSplit.size() - 1; i++){
                QByteArrayList csvString = rxDataSplit[i].split(',');
//                qDebug() << (int)csvString.size();
                if (csvString.size() == 52){
                    int indx = 0;
                    dataControl->ServerToClient.t = csvString[indx++].toDouble();
                    for(int j = 0; j < NUM_JOINT; j++){
                        dataControl->ServerToClient.presentJointPosition[j] = csvString[indx++].toDouble();
                    }
                    for(int j = 0; j < NUM_DOF; j++){
                        dataControl->ServerToClient.presentCartesianPose[j] = csvString[indx++].toDouble();
                    }
                    for(int j = 0; j < NUM_JOINT; j++){
                        dataControl->ServerToClient.desiredJointPosition[j] = csvString[indx++].toDouble();
                    }
                    for(int j = 0; j < NUM_DOF; j++){
                        dataControl->ServerToClient.desiredCartesianPose[j] = csvString[indx++].toDouble();
                    }
                    for(int j = 0; j < NUM_DOF; j++){
                        dataControl->ServerToClient.calculateCartesianPose[j] = csvString[indx++].toDouble();
                    }
                    for(int j = 0; j < NUM_JOINT; j++){
                        dataControl->ServerToClient.presentJointVelocity[j] = csvString[indx++].toDouble();
                    }
                    for(int j = 0; j < NUM_JOINT; j++){
                        dataControl->ServerToClient.presentJointCurrent[j] = csvString[indx++].toDouble();
                    }
                    for(int j = 0; j < NUM_DOF; j++){
                        dataControl->ServerToClient.presentCartesianVelocity[j] = csvString[indx++].toDouble();
                    }
                    dataControl->ServerToClient.time = csvString[indx++].toDouble();
                    dataControl->ServerToClient.dxl_time = csvString[indx++].toDouble();
                    dataControl->ServerToClient.ik_time = csvString[indx++].toDouble();
                }
            }
        }

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
}

void MainWindow::disConnectServer(){
    ui->btnDisconnect->animateClick();
}

void MainWindow::componentEnable(bool enable){
    ui->gbRobotInfor->setEnabled(enable);
    ui->gbServoControl->setEnabled(enable);
    ui->gbJointMoveCommand->setEnabled(enable);

    ui->txtJCmd1->setEnabled(false);
    ui->txtJCmd2->setEnabled(false);
    ui->txtJCmd3->setEnabled(false);
    ui->txtJCmd4->setEnabled(false);
    ui->txtJCmd5->setEnabled(false);
    ui->txtJCmd6->setEnabled(false);

    ui->cbJointRel->setEnabled(false);
    ui->cbJointAbs->setEnabled(false);
    ui->btnJSet->setEnabled(false);
}

void MainWindow::closeEvent(QCloseEvent*){
    qDebug() << "Closed MainWindow";
}
