#pragma once

#include <QObject>
#include <QTcpSocket>
#include <QHostAddress>
#include <QtDebug>

class TcpClient : public QObject
{
    Q_OBJECT
public:
    explicit TcpClient(QObject *parent = nullptr);
    ~TcpClient();

	QTcpSocket *socket;
	void setIpAddress(QString address);
    void setPort(quint16 num);

public slots:
    void connectToServer();

private:
	QString ipAddress;
    quint16 port;
};

