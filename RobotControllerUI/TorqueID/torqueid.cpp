#include "torqueid.h"
#include "ui_torqueid.h"

TorqueID::TorqueID(void* _tcp, QWidget *parent) : QMainWindow(parent), ui(new Ui::TorqueID)
{
    ui->setupUi(this);
	tcp = static_cast<TcpClient*>(_tcp);

	connect(ui->sbTorqueConst, SIGNAL(editingFinished()), this, SLOT(sbTorqueConstEditingFinished()));
}

TorqueID::~TorqueID(){
}

void TorqueID::sbTorqueConstEditingFinished()
{
	txData.clear();
	txData.append(Qt::Key_N);
	txData.append(Qt::Key_T);

    txData.append(DataControl::OpMode::TorqueID);
	txData.append(QByteArray::number(ui->sbMass->value(), 'f', 6));
	txData.append(QByteArray::number(ui->sbTorqueConst->value(), 'f', 6));

	txData.append(Qt::Key_N);
	txData.append(Qt::Key_E);

	qDebug() << "txData : " << txData;

	static_cast<TcpClient*>(tcp)->socket->write(txData);
}
