#include <QCoreApplication>
#include "ControlMain/controlmain.h"

static ControlMain *controlMain;

int main(int argc, char** argv)
{
    QCoreApplication a(argc, argv);

    controlMain = new ControlMain();
    controlMain->start();

    return a.exec();
}
