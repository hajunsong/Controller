#ifndef TORQUEIDE_H
#define TORQUEIDE_H

#include <QMainWindow>
#include <QDebug>
#include <QWidget>

#include "TcpSocket/tcpclient.h"
#include "DataControl/datacontrol.h"

namespace Ui{
class TorqueID;
}

class TorqueID : public QMainWindow
{
    Q_OBJECT

public:
	explicit TorqueID(void* _tcp, QWidget *parent = nullptr);
	~TorqueID();

public slots:
	// spin box event
	void sbTorqueConstEditingFinished();

private:
	Ui::TorqueID *ui;
	void *tcp;
	QByteArray txData;
};

#endif
