#pragma once

#include <QTcpServer>
#include <QDebug>
#include <QTcpSocket>
#include <QTimer>
#include <QtMath>

class TcpServer : public QTcpServer
{
	Q_OBJECT
public:
	explicit TcpServer(QObject *parent = nullptr);
    ~TcpServer();
	void startServer();
    void setting(QString IP, quint16 port);
    QTcpSocket *socket;
    bool isConnected();
    void setConnected(bool connected);
    void sendData(QByteArray data);

signals:
	void error(QTcpSocket::SocketError socketerror);
    void connectedClient();

public slots:
	void readyRead();
	void disconnected();

protected:
	void incomingConnection(qintptr socketDescriptor);

private:
    QString ip;
    quint16 port;
    bool connectState;
};

