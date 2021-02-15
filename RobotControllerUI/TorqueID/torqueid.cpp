#include "torqueid.h"
#include "ui_torqueid.h"

TorqueID::TorqueID(void* _tcp, QWidget *parent) : QMainWindow(parent), ui(new Ui::TorqueID)
{
    ui->setupUi(this);
	tcp = static_cast<TcpClient*>(_tcp);

    connect(ui->txtMass, SIGNAL(editingFinished()), this, SLOT(sbTorqueConstEditingFinished()));
    connect(ui->txtTorqueConst, SIGNAL(editingFinished()), this, SLOT(sbTorqueConstEditingFinished()));
}

TorqueID::~TorqueID(){
}

void TorqueID::sbTorqueConstEditingFinished()
{
//    txData.clear();
//    txData.append(Qt::Key_N);
//    txData.append(Qt::Key_T);

//    txData.append(DataControl::OpMode::TorqueID);
//    txData.append(QByteArray::number(ui->txtMass->text().toDouble(), 'f', 6));
//    txData.append(QByteArray::number(ui->txtTorqueConst->text().toDouble(), 'f', 6));

//    txData.append(Qt::Key_N);
//    txData.append(Qt::Key_E);

//    qDebug() << "txData : " << txData;

//    static_cast<TcpClient*>(tcp)->sendData(txData.toStdString(), txData.size());

    int16_t indx = 0;
    char bufSend[MAXSENDBUFSIZE] = {0,};
    memset(bufSend, 0, MAXSENDBUFSIZE);
    bufSend[indx++] = Qt::Key_N;
    bufSend[indx++] = Qt::Key_O;

    bufSend[indx++] = DataControl::OpMode::OperateMode;
    memcpy(bufSend + indx, to_string(ui->txtMass->text().toDouble()).c_str(), 8);
    indx += 8;
    memcpy(bufSend + indx, to_string(ui->txtTorqueConst->text().toDouble()).c_str(), 8);
    indx += 8;

    bufSend[indx++] = Qt::Key_N;
    bufSend[indx++] = Qt::Key_E;

    qDebug() << "txData : " << QByteArray::fromRawData(bufSend, indx);

    static_cast<TcpClient*>(tcp)->sendData(bufSend, indx);
}
