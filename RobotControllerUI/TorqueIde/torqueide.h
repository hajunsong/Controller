#ifndef TORQUEIDE_H
#define TORQUEIDE_H

#include <QMainWindow>
#include <QDebug>
#include <QWidget>
#include "TcpSocket/tcpclient.h"
#include "Input/keyinputclass.h"

namespace Ui{
class TorqueIde;
}

class TorqueIde : public QMainWindow
{
    Q_OBJECT

public:
    explicit TorqueIde(TcpClient *client, QWidget *parent = nullptr);
    ~TorqueIde();

public slots:
    // event
    void keyPressEvent(QKeyEvent *event);
    void closeEvent(QCloseEvent* event);

    // button event
    void btnInitClicked();
    void btnDeinitClicked();
    void btnUpClicked();
    void btnDownClicked();
    void btnSetOffsetClicked();
    void btnSaveClicked();
    void btnSetMassClicked();

    // server event
    void readMessage();

private:
    Ui::TorqueIde *ui;
    TcpClient *tcpClient;
    KeyInputClass *keyInputClass;
    QByteArray txData;

    enum{Init=1, Deinit, Up, Down, Offset, Run};
};

#endif
