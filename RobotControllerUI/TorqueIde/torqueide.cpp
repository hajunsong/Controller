#include "torqueide.h"
#include "ui_torqueide.h"

TorqueIde::TorqueIde(QWidget *parent) : QMainWindow(parent), ui(new Ui::TorqueIde)
{
    ui->setupUi(this);
//    mainwindow = _mainwindow;
}

TorqueIde::~TorqueIde(){

}
