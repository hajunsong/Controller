#pragma once

#include <QTcpServer>
#include <QDebug>
#include <QTcpSocket>

#include <QtCore/qglobal.h>

#if defined(TCPSERVERLIB_LIBRARY)
#  define TCPSERVERLIB_EXPORT Q_DECL_EXPORT
#else
#  define TCPSERVERLIB_EXPORT Q_DECL_IMPORT
#endif

class TcpServer : public QTcpServer
{
	Q_OBJECT
public:
	explicit TcpServer(QObject *parent = nullptr);
	QTcpServer *server;
	QTcpSocket *socket;
	void startServer();
	void setIpAddress(QString address);
	void setPort(quint16 num);

signals:
	void error(QTcpSocket::SocketError socketerror);
    void connectedClient();

protected:
	void incomingConnection(qintptr socketDescriptor);

private:
	QString ipAddress;
	quint16 portNum;
};

