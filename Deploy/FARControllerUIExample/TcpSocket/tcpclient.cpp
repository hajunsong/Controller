#include "tcpclient.h"

TcpClient::TcpClient(QObject *parent) : QObject(parent) {
    socket = new QTcpSocket;
}

TcpClient::~TcpClient() {
    if (socket->isOpen()){
        socket->close();
    }
    socket->deleteLater();
    delete socket;
}

void TcpClient::connectToServer() {
    socket->connectToHost(ipAddress, port);
}

void TcpClient::setIpAddress(QString address) {
    ipAddress = address;
}

void TcpClient::setPort(quint16 num){
    port = num;
}
