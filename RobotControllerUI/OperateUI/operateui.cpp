#include "operateui.h"
#include "ui_operateui.h"

OperateUI::OperateUI(void* _tcpClient, QWidget *parent) : QWidget(parent), ui(new Ui::OperateUI)
{
    ui->setupUi(this);
    tcpClient = static_cast<TcpClient*>(_tcpClient);

    connect(ui->btnTeaching, SIGNAL(clicked()), this, SLOT(btnTeachingClicked()));
    connect(ui->btnFeeding, SIGNAL(clicked()), this, SLOT(btnFeedingClicked()));
    connect(ui->btnSide1, SIGNAL(clicked()), this, SLOT(btnSide1Clicked()));
    connect(ui->btnSide2, SIGNAL(clicked()), this, SLOT(btnSide2Clicked()));
    connect(ui->btnSide3, SIGNAL(clicked()), this, SLOT(btnSide3Clicked()));
    connect(ui->btnRise, SIGNAL(clicked()), this, SLOT(btnRiseClicked()));
    connect(ui->btnSoup, SIGNAL(clicked()), this, SLOT(btnSoupClicked()));
    connect(ui->btnStart, SIGNAL(clicked()), this, SLOT(btnStartClciked()));

    connect(ui->btnListen, SIGNAL(clicked()), this, SLOT(btnListenClicked()));

    tcpServer = new TcpServer();

    client_connected = false;
}

OperateUI::~OperateUI(){
    tcpServer->deleteLater();
    delete tcpServer;
}

void OperateUI::init()
{
    if (ui->btnStart->text().compare("Stop") == 0){
        ui->btnStart->animateClick();
    }
    if (ui->btnFeeding->text().contains("Stop")){
        ui->btnFeeding->animateClick();
    }
    componentEnable(false);
    tcpServer->socket->disconnectFromHost();
}

void OperateUI::stop()
{
    ui->btnStart->setText("Start");
    ui->btnFeeding->setText("Feeding\nMode");
    ui->btnTeaching->setText("Teaching\nMode");
    componentEnable(false);
    if(client_connected){
        tcpServer->socket->close();
    }
}

void OperateUI::btnStartClciked()
{
    char data[2] = {0,0};

    if (ui->btnStart->text().compare("Start") == 0){
        ui->btnStart->setText("Stop");
        data[0] = DataControl::Operate::Start;
    }
    else{
        ui->btnStart->setText("Start");
        data[0] = DataControl::Operate::Stop;
    }

    sendDataToServer(data);
}

void OperateUI::btnTeachingClicked(){
    char data[2] = {0,0};

    if (ui->btnTeaching->text().contains("Stop")){
        qDebug() << "Stop teaching mode";
        ui->txtLogMessage->append("Stop teaching mode");
        ui->btnTeaching->setText("Teaching\nMode");

        data[0] = DataControl::Operate::StopTeaching;
    }
    else {
        qDebug() << "Start teaching mode";
        ui->btnTeaching->setText("Stop\nTeaching");
        ui->txtLogMessage->append("Start teaching mode");

        data[0] = DataControl::Operate::StartTeaching;
    }

    sendDataToServer(data);
}

void OperateUI::btnFeedingClicked(){
    char data[2] = {0, 0};

    if (ui->btnFeeding->text().contains("Stop")){
        qDebug() << "Stop feeding mode";
        ui->btnFeeding->setText("Feeding\nMode");
        ui->txtLogMessage->append("Start feeding mode");
        componentEnable(false);

        data[0] = DataControl::Operate::StopFeeding;
    }
    else {
        qDebug() << "Start feeding mode";
        ui->btnFeeding->setText("Stop\nFeeding");
        ui->txtLogMessage->append("Start feeding mode");
        componentEnable(true);

        data[0] = DataControl::Operate::StartFeeding;
    }

    sendDataToServer(data);
}

void OperateUI::btnSide1Clicked(){
    char data[2] = {DataControl::Operate::Feeding, DataControl::Section::Side1};

    sendDataToServer(data);
}

void OperateUI::btnSide2Clicked(){
    char data[2] = {DataControl::Operate::Feeding, DataControl::Section::Side2};

    sendDataToServer(data);
}

void OperateUI::btnSide3Clicked(){
    char data[2] = {DataControl::Operate::Feeding, DataControl::Section::Side3};

    sendDataToServer(data);
}

void OperateUI::btnRiseClicked(){
    char data[2] = {DataControl::Operate::Feeding, DataControl::Section::Rice};

    sendDataToServer(data);
}

void OperateUI::btnSoupClicked(){
    char data[2] = {DataControl::Operate::Feeding, DataControl::Section::Soup};

    sendDataToServer(data);
}

void OperateUI::btnListenClicked()
{
    if (!tcpServer->isListening()){
        QString ipAddress = ui->txtIP->text();
        unsigned short portNum = ui->txtPORT->text().toUShort();
        tcpServer->setIpAddress(ipAddress);
        tcpServer->setPort(portNum);
        tcpServer->startServer();

        connect(tcpServer, SIGNAL(connectedClient()), this, SLOT(connected()));
        if (tcpServer->isListening()){
            ui->txtLogMessage->append("Listening...");
        }
        else{
            ui->txtLogMessage->append("Could not start server");
        }
    }
}

void OperateUI::connected()
{
    ui->rbConnectState->setChecked(true);
    connect(tcpServer->socket, SIGNAL(readyRead()), this, SLOT(readMessage()), Qt::DirectConnection);
    connect(tcpServer->socket, SIGNAL(disconnected()), this, SLOT(disconnected()), Qt::DirectConnection);
    ui->txtLogMessage->append("Connected tablet");
    client_connected = true;
}

void OperateUI::readMessage()
{
    QString date = QDateTime::currentDateTime().toString("dd.MM.yyyy hh:mm:ss ");
    QByteArray rxData = tcpServer->socket->readAll();
    QString logMessage = date + " Msg : " + rxData;
    ui->txtLogMessage->append(logMessage);

    if (rxData.toInt() == 0){
        ui->rbConnectState->setChecked(true);
        ui->txtLogMessage->append("Ready tablet");
    }

    if (rxData.length() == 1 && ui->rbConnectState->isChecked()){
        switch(rxData.toInt()){
            case DataControl::Section::Side1:
                ui->btnSide1->animateClick();
                qDebug() << "Selected Side 1";
                break;
            case DataControl::Section::Side2:
                ui->btnSide2->animateClick();
                qDebug() << "Selected Side 2";
                break;
            case DataControl::Section::Side3:
                ui->btnSide3->animateClick();
                qDebug() << "Selected Side 3";
                break;
            case DataControl::Section::Soup:
                ui->btnSoup->animateClick();
                qDebug() << "Selected Soup";
                break;
            case DataControl::Section::Rice:
                ui->btnRise->animateClick();
                qDebug() << "Selected Rise";
                break;
        }
    }
}

void OperateUI::disconnected()
{
    ui->rbConnectState->setChecked(false);
    disconnect(tcpServer->socket, SIGNAL(readyRead()), this, SLOT(readMessage()));
    disconnect(tcpServer->socket, SIGNAL(disconnected()), this, SLOT(disconnected()));
    ui->txtLogMessage->append("Disconnected tablet");
    client_connected = false;
}

void OperateUI::componentEnable(bool enable)
{
    ui->btnSide1->setEnabled(enable);
    ui->btnSide2->setEnabled(enable);
    ui->btnSide3->setEnabled(enable);
    ui->btnRise->setEnabled(enable);
    ui->btnSoup->setEnabled(enable);
}

void OperateUI::sendDataToServer(char *data)
{
    int16_t indx = 0;
    memset(bufSend, 0, MAXSENDBUFSIZE);
    bufSend[indx++] = Qt::Key_N;
    bufSend[indx++] = Qt::Key_O;

    bufSend[indx++] = DataControl::OpMode::OperateMode;
    bufSend[indx++] = data[0];
    bufSend[indx++] = data[1];

    bufSend[indx++] = Qt::Key_N;
    bufSend[indx++] = Qt::Key_E;

    qDebug() << "txData : " << QByteArray::fromRawData(bufSend, indx);

    static_cast<TcpClient*>(tcpClient)->sendData(bufSend, indx);
}
