#include "keyinputkeypadclass.h"
#include "MainWindow/mainwindow.h"
#include "ui_mainwindow.h"

KeyinputKeypadClass::KeyinputKeypadClass(void* _ui, void* _tcp_client)
{
    ui = _ui;
    tcpClient = _tcp_client;
}

void KeyinputKeypadClass::KeypadKeyInput(QKeyEvent* keyevt)
{
    int k = keyevt->key();
    switch(k)
    {
        case Qt::Key_0:
        {
            char data[2] = {0, 0};
            data[0] = DataControl::Operate::StartFeeding;

            int16_t indx = 0;
            memset(static_cast<TcpClient*>(tcpClient)->bufSend, 0, MAXSENDBUFSIZE);
            static_cast<TcpClient*>(tcpClient)->bufSend[indx++] = Qt::Key_N;
            static_cast<TcpClient*>(tcpClient)->bufSend[indx++] = Qt::Key_O;

            static_cast<TcpClient*>(tcpClient)->bufSend[indx++] = DataControl::OpMode::OperateMode;
            static_cast<TcpClient*>(tcpClient)->bufSend[indx++] = data[0];
            static_cast<TcpClient*>(tcpClient)->bufSend[indx++] = data[1];

            static_cast<TcpClient*>(tcpClient)->bufSend[indx++] = Qt::Key_N;
            static_cast<TcpClient*>(tcpClient)->bufSend[indx++] = Qt::Key_E;

            qDebug() << "txData : " << QByteArray::fromRawData(static_cast<TcpClient*>(tcpClient)->bufSend, indx);

            static_cast<TcpClient*>(tcpClient)->sendData(static_cast<TcpClient*>(tcpClient)->bufSend, indx);
            break;
        }
        case Qt::Key_1:
        {
            char data[2] = {DataControl::Operate::Feeding, DataControl::Section::Rice1};
            int16_t indx = 0;
            memset(static_cast<TcpClient*>(tcpClient)->bufSend, 0, MAXSENDBUFSIZE);
            static_cast<TcpClient*>(tcpClient)->bufSend[indx++] = Qt::Key_N;
            static_cast<TcpClient*>(tcpClient)->bufSend[indx++] = Qt::Key_O;

            static_cast<TcpClient*>(tcpClient)->bufSend[indx++] = DataControl::OpMode::OperateMode;
            static_cast<TcpClient*>(tcpClient)->bufSend[indx++] = data[0];
            static_cast<TcpClient*>(tcpClient)->bufSend[indx++] = data[1];

            static_cast<TcpClient*>(tcpClient)->bufSend[indx++] = Qt::Key_N;
            static_cast<TcpClient*>(tcpClient)->bufSend[indx++] = Qt::Key_E;

            qDebug() << "txData : " << QByteArray::fromRawData(static_cast<TcpClient*>(tcpClient)->bufSend, indx);

            static_cast<TcpClient*>(tcpClient)->sendData(static_cast<TcpClient*>(tcpClient)->bufSend, indx);
            break;
        }
        case Qt::Key_2:
        {
            char data[2] = {DataControl::Operate::Feeding, DataControl::Section::Rice2};
            int16_t indx = 0;
            memset(static_cast<TcpClient*>(tcpClient)->bufSend, 0, MAXSENDBUFSIZE);
            static_cast<TcpClient*>(tcpClient)->bufSend[indx++] = Qt::Key_N;
            static_cast<TcpClient*>(tcpClient)->bufSend[indx++] = Qt::Key_O;

            static_cast<TcpClient*>(tcpClient)->bufSend[indx++] = DataControl::OpMode::OperateMode;
            static_cast<TcpClient*>(tcpClient)->bufSend[indx++] = data[0];
            static_cast<TcpClient*>(tcpClient)->bufSend[indx++] = data[1];

            static_cast<TcpClient*>(tcpClient)->bufSend[indx++] = Qt::Key_N;
            static_cast<TcpClient*>(tcpClient)->bufSend[indx++] = Qt::Key_E;

            qDebug() << "txData : " << QByteArray::fromRawData(static_cast<TcpClient*>(tcpClient)->bufSend, indx);

            static_cast<TcpClient*>(tcpClient)->sendData(static_cast<TcpClient*>(tcpClient)->bufSend, indx);
            break;
        }
        case Qt::Key_3:
        {
            char data[2] = {DataControl::Operate::Feeding, DataControl::Section::Rice3};
            int16_t indx = 0;
            memset(static_cast<TcpClient*>(tcpClient)->bufSend, 0, MAXSENDBUFSIZE);
            static_cast<TcpClient*>(tcpClient)->bufSend[indx++] = Qt::Key_N;
            static_cast<TcpClient*>(tcpClient)->bufSend[indx++] = Qt::Key_O;

            static_cast<TcpClient*>(tcpClient)->bufSend[indx++] = DataControl::OpMode::OperateMode;
            static_cast<TcpClient*>(tcpClient)->bufSend[indx++] = data[0];
            static_cast<TcpClient*>(tcpClient)->bufSend[indx++] = data[1];

            static_cast<TcpClient*>(tcpClient)->bufSend[indx++] = Qt::Key_N;
            static_cast<TcpClient*>(tcpClient)->bufSend[indx++] = Qt::Key_E;

            qDebug() << "txData : " << QByteArray::fromRawData(static_cast<TcpClient*>(tcpClient)->bufSend, indx);

            static_cast<TcpClient*>(tcpClient)->sendData(static_cast<TcpClient*>(tcpClient)->bufSend, indx);
            break;
        }
        case Qt::Key_4:
        {
            char data[2] = {DataControl::Operate::Feeding, DataControl::Section::Rice4};
            int16_t indx = 0;
            memset(static_cast<TcpClient*>(tcpClient)->bufSend, 0, MAXSENDBUFSIZE);
            static_cast<TcpClient*>(tcpClient)->bufSend[indx++] = Qt::Key_N;
            static_cast<TcpClient*>(tcpClient)->bufSend[indx++] = Qt::Key_O;

            static_cast<TcpClient*>(tcpClient)->bufSend[indx++] = DataControl::OpMode::OperateMode;
            static_cast<TcpClient*>(tcpClient)->bufSend[indx++] = data[0];
            static_cast<TcpClient*>(tcpClient)->bufSend[indx++] = data[1];

            static_cast<TcpClient*>(tcpClient)->bufSend[indx++] = Qt::Key_N;
            static_cast<TcpClient*>(tcpClient)->bufSend[indx++] = Qt::Key_E;

            qDebug() << "txData : " << QByteArray::fromRawData(static_cast<TcpClient*>(tcpClient)->bufSend, indx);

            static_cast<TcpClient*>(tcpClient)->sendData(static_cast<TcpClient*>(tcpClient)->bufSend, indx);
            break;
        }
        case Qt::Key_5:
        {
            char data[2] = {DataControl::Operate::Feeding, DataControl::Section::Rice5};
            int16_t indx = 0;
            memset(static_cast<TcpClient*>(tcpClient)->bufSend, 0, MAXSENDBUFSIZE);
            static_cast<TcpClient*>(tcpClient)->bufSend[indx++] = Qt::Key_N;
            static_cast<TcpClient*>(tcpClient)->bufSend[indx++] = Qt::Key_O;

            static_cast<TcpClient*>(tcpClient)->bufSend[indx++] = DataControl::OpMode::OperateMode;
            static_cast<TcpClient*>(tcpClient)->bufSend[indx++] = data[0];
            static_cast<TcpClient*>(tcpClient)->bufSend[indx++] = data[1];

            static_cast<TcpClient*>(tcpClient)->bufSend[indx++] = Qt::Key_N;
            static_cast<TcpClient*>(tcpClient)->bufSend[indx++] = Qt::Key_E;

            qDebug() << "txData : " << QByteArray::fromRawData(static_cast<TcpClient*>(tcpClient)->bufSend, indx);

            static_cast<TcpClient*>(tcpClient)->sendData(static_cast<TcpClient*>(tcpClient)->bufSend, indx);
            break;
        }
        case Qt::Key_6:
        {
            char data[2] = {DataControl::Operate::Feeding, DataControl::Section::Rice6};
            int16_t indx = 0;
            memset(static_cast<TcpClient*>(tcpClient)->bufSend, 0, MAXSENDBUFSIZE);
            static_cast<TcpClient*>(tcpClient)->bufSend[indx++] = Qt::Key_N;
            static_cast<TcpClient*>(tcpClient)->bufSend[indx++] = Qt::Key_O;

            static_cast<TcpClient*>(tcpClient)->bufSend[indx++] = DataControl::OpMode::OperateMode;
            static_cast<TcpClient*>(tcpClient)->bufSend[indx++] = data[0];
            static_cast<TcpClient*>(tcpClient)->bufSend[indx++] = data[1];

            static_cast<TcpClient*>(tcpClient)->bufSend[indx++] = Qt::Key_N;
            static_cast<TcpClient*>(tcpClient)->bufSend[indx++] = Qt::Key_E;

            qDebug() << "txData : " << QByteArray::fromRawData(static_cast<TcpClient*>(tcpClient)->bufSend, indx);

            static_cast<TcpClient*>(tcpClient)->sendData(static_cast<TcpClient*>(tcpClient)->bufSend, indx);
            break;
        }
        case Qt::Key_7:
        {
            char data[2] = {DataControl::Operate::Feeding, DataControl::Section::Rice7};
            int16_t indx = 0;
            memset(static_cast<TcpClient*>(tcpClient)->bufSend, 0, MAXSENDBUFSIZE);
            static_cast<TcpClient*>(tcpClient)->bufSend[indx++] = Qt::Key_N;
            static_cast<TcpClient*>(tcpClient)->bufSend[indx++] = Qt::Key_O;

            static_cast<TcpClient*>(tcpClient)->bufSend[indx++] = DataControl::OpMode::OperateMode;
            static_cast<TcpClient*>(tcpClient)->bufSend[indx++] = data[0];
            static_cast<TcpClient*>(tcpClient)->bufSend[indx++] = data[1];

            static_cast<TcpClient*>(tcpClient)->bufSend[indx++] = Qt::Key_N;
            static_cast<TcpClient*>(tcpClient)->bufSend[indx++] = Qt::Key_E;

            qDebug() << "txData : " << QByteArray::fromRawData(static_cast<TcpClient*>(tcpClient)->bufSend, indx);

            static_cast<TcpClient*>(tcpClient)->sendData(static_cast<TcpClient*>(tcpClient)->bufSend, indx);
            break;
        }
        case Qt::Key_8:
        {
            char data[2] = {DataControl::Operate::Feeding, DataControl::Section::Rice8};
            int16_t indx = 0;
            memset(static_cast<TcpClient*>(tcpClient)->bufSend, 0, MAXSENDBUFSIZE);
            static_cast<TcpClient*>(tcpClient)->bufSend[indx++] = Qt::Key_N;
            static_cast<TcpClient*>(tcpClient)->bufSend[indx++] = Qt::Key_O;

            static_cast<TcpClient*>(tcpClient)->bufSend[indx++] = DataControl::OpMode::OperateMode;
            static_cast<TcpClient*>(tcpClient)->bufSend[indx++] = data[0];
            static_cast<TcpClient*>(tcpClient)->bufSend[indx++] = data[1];

            static_cast<TcpClient*>(tcpClient)->bufSend[indx++] = Qt::Key_N;
            static_cast<TcpClient*>(tcpClient)->bufSend[indx++] = Qt::Key_E;

            qDebug() << "txData : " << QByteArray::fromRawData(static_cast<TcpClient*>(tcpClient)->bufSend, indx);

            static_cast<TcpClient*>(tcpClient)->sendData(static_cast<TcpClient*>(tcpClient)->bufSend, indx);
            break;
        }
        case Qt::Key_9:
        {
            char data[2] = {DataControl::Operate::Feeding, DataControl::Section::Rice9};
            int16_t indx = 0;
            memset(static_cast<TcpClient*>(tcpClient)->bufSend, 0, MAXSENDBUFSIZE);
            static_cast<TcpClient*>(tcpClient)->bufSend[indx++] = Qt::Key_N;
            static_cast<TcpClient*>(tcpClient)->bufSend[indx++] = Qt::Key_O;

            static_cast<TcpClient*>(tcpClient)->bufSend[indx++] = DataControl::OpMode::OperateMode;
            static_cast<TcpClient*>(tcpClient)->bufSend[indx++] = data[0];
            static_cast<TcpClient*>(tcpClient)->bufSend[indx++] = data[1];

            static_cast<TcpClient*>(tcpClient)->bufSend[indx++] = Qt::Key_N;
            static_cast<TcpClient*>(tcpClient)->bufSend[indx++] = Qt::Key_E;

            qDebug() << "txData : " << QByteArray::fromRawData(static_cast<TcpClient*>(tcpClient)->bufSend, indx);

            static_cast<TcpClient*>(tcpClient)->sendData(static_cast<TcpClient*>(tcpClient)->bufSend, indx);
            break;
        }
        case Qt::Key_Plus:

            break;

        case Qt::Key_Minus:

            break;

        case Qt::Key_division:

            break;

        case Qt::Key_multiply:

            break;

        case Qt::Key_Enter:

            break;

        case Qt::Key_NumLock:

            break;

        case Qt::Key_Insert:

            break;

        case Qt::Key_Delete:

            break;

        case Qt::Key_End:

            break;

        case Qt::Key_Down:

            break;

        case Qt::Key_PageDown:

            break;

        case Qt::Key_Left:

            break;

        case Qt::Key_Right:

            break;

        case Qt::Key_Home:

            break;

        case Qt::Key_PageUp:

            break;

        default:

            break;
    }
}
