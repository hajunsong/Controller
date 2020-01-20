#include "operateui.h"
#include "ui_operateui.h"

OperateUI::OperateUI(void* _tcpClient, QWidget *parent) : QWidget(parent), ui(new Ui::OperateUI)
{
    ui->setupUi(this);
//    this->setWindowTitle("Operate UI");
    tcpClient = static_cast<TcpClient*>(_tcpClient);

//    connect(ui->btnTeaching, SIGNAL(clicked()), this, SLOT(btnTeachingClicked()));
//    connect(ui->btnFeeding, SIGNAL(clicked()), this, SLOT(btnFeedingClicked()));
//    connect(ui->btnSide1, SIGNAL(clicked()), this, SLOT(btnSide1Clicked()));
//    connect(ui->btnSide2, SIGNAL(clicked()), this, SLOT(btnSide2Clicked()));
//    connect(ui->btnSide3, SIGNAL(clicked()), this, SLOT(btnSide3Clicked()));
//    connect(ui->btnRise, SIGNAL(clicked()), this, SLOT(btnRiseClicked()));
//    connect(ui->btnSoup, SIGNAL(clicked()), this, SLOT(btnSoupClicked()));
//    connect(ui->btnStart, SIGNAL(clicked()), this, SLOT(btnStartClciked()));
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
        ui->btnTeaching->setText("Teaching\nMode");

        data[0] = DataControl::Operate::StopTeaching;
    }
    else {
        qDebug() << "Start feeding mode";
        ui->btnTeaching->setText("Stop\nTeaching");

        data[0] = DataControl::Operate::StartTeaching;
    }

    sendDataToServer(data);
}

void OperateUI::btnFeedingClicked(){
    char data[2] = {0, 0};

    if (ui->btnFeeding->text().contains("Stop")){
        qDebug() << "Stop feeding mode";
        ui->btnFeeding->setText("Feeding\nMode");
        componentEnable(false);

        data[0] = DataControl::Operate::StopFeeding;
    }
    else {
        qDebug() << "Start feeding mode";
        ui->btnFeeding->setText("Stop\nFeeding");
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
    char data[2] = {DataControl::Operate::Feeding, DataControl::Section::Rise};

    sendDataToServer(data);
}

void OperateUI::btnSoupClicked(){
    char data[2] = {DataControl::Operate::Feeding, DataControl::Section::Soup};

    sendDataToServer(data);
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
    txData.clear();
    txData.append(Qt::Key_N);
    txData.append(Qt::Key_O);

    txData.append(DataControl::OpMode::OperateMode);
    txData.append(data[0]);
    txData.append(data[1]);

    txData.append(Qt::Key_N);
    txData.append(Qt::Key_E);

    qDebug() << "txData : " << txData;

    static_cast<TcpClient*>(tcpClient)->socket->write(txData);
}
