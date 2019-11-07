#ifndef TORQUEIDE_H
#define TORQUEIDE_H

#include <QMainWindow>
#include <QDebug>
#include <QWidget>

#include "MainWindow/mainwindow.h"

namespace Ui{
class TorqueIde;
}

class TorqueIde : public QMainWindow
{
    Q_OBJECT

public:
    explicit TorqueIde(QWidget *parent = nullptr);
    ~TorqueIde();

private:
    Ui::TorqueIde *ui;
};

#endif
