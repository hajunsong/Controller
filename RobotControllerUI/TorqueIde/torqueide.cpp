#include "torqueide.h"
#include "ui_torqueide.h"

TorqueIde::TorqueIde(TcpClient* client, QWidget *parent) : QMainWindow(parent), ui(new Ui::TorqueIde)
{
    ui->setupUi(this);
    tcpClient = client;

    keyInputClass = new KeyInputClass(nullptr, this);

    connect(ui->btnInit, SIGNAL(clicked()), this, SLOT(btnInitClicked()));
    connect(ui->btnDeinit, SIGNAL(clicked()), this, SLOT(btnDeinitClicked()));
    connect(ui->btnUp, SIGNAL(clicked()), this, SLOT(btnUpClicked()));
    connect(ui->btnDown, SIGNAL(clicked()), this, SLOT(btnDownClicked()));
    connect(ui->btnSetOffset, SIGNAL(clicked()), this, SLOT(btnSetOffsetClicked()));
    connect(ui->btnSave, SIGNAL(clicked()), this, SLOT(btnSaveClicked()));
    connect(ui->btnSetMass, SIGNAL(clicked()), this, SLOT(btnSetMassClicked()));

    connect(tcpClient->socket, SIGNAL(readyRead()), this, SLOT(readMessage()));
}

TorqueIde::~TorqueIde(){
}

void TorqueIde::keyPressEvent(QKeyEvent *event)
{
    keyInputClass->InputKeyboard(event);
}

void TorqueIde::closeEvent(QCloseEvent*){
    delete ui;
    delete tcpClient;
}

void TorqueIde::btnInitClicked()
{
    txData.clear();
    txData.append(Qt::Key_N);
    txData.append(Qt::Key_T);
    txData.append(Init);
    txData.append(Qt::Key_N);
    txData.append(Qt::Key_E);
}

void TorqueIde::btnDeinitClicked()
{
    txData.clear();
    txData.append(Qt::Key_N);
    txData.append(Qt::Key_T);
    txData.append(Deinit);
    txData.append(Qt::Key_N);
    txData.append(Qt::Key_E);
}

void TorqueIde::btnSetMassClicked(){
    txData.clear();
    txData.append(Qt::Key_N);
    txData.append(Qt::Key_M);
    txData.append(QString::number(ui->txtMass->text().toDouble()));
    txData.append(Qt::Key_N);
    txData.append(Qt::Key_E);
}

void TorqueIde::btnUpClicked()
{
    txData.clear();
    txData.append(Qt::Key_N);
    txData.append(Qt::Key_T);
    txData.append(Up);
    txData.append(Qt::Key_N);
    txData.append(Qt::Key_E);
}

void TorqueIde::btnDownClicked()
{
    txData.clear();
    txData.append(Qt::Key_N);
    txData.append(Qt::Key_T);
    txData.append(Down);
    txData.append(Qt::Key_N);
    txData.append(Qt::Key_E);
}

void TorqueIde::btnSetOffsetClicked()
{
    txData.clear();
    txData.append(Qt::Key_N);
    txData.append(Qt::Key_T);
    txData.append(Offset);
    txData.append(Qt::Key_N);
    txData.append(Qt::Key_E);
}

void TorqueIde::btnSaveClicked()
{

}

void TorqueIde::readMessage()
{
    QByteArray rxData;
    rxData = tcpClient->socket->readAll();
    qDebug() << rxData;
}
