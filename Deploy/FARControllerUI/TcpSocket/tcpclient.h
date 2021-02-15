#pragma once

#include <QObject>
#include <QTcpSocket>
#include <QHostAddress>
#include <QtDebug>

#include <QtCore/qglobal.h>

#if defined(TCPCLIENTLIB_LIBRARY)
#  define TCPCLIENTLIB_EXPORT Q_DECL_EXPORT
#else
#  define TCPCLIENTLIB_EXPORT Q_DECL_IMPORT
#endif

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

