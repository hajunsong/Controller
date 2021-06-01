QT += core gui widgets network

TARGET = RobotControllerUI
TEMPLATE = app

CONFIG += c++11

SOURCES += \
    main.cpp \

HEADERS += \
    DataControl/datacontrol.h \
    FileIO/fileio.h \
    Logger/logger.h \
    Settings/customsettings.h \
    TcpSocket/tcpclient.h \
    TcpSocket/tcpserver.h \
    Input/keyinputclass.h \
    Input/keyinputnomodifiedclass.h \
    Input/keyinputkeypadclass.h \
    MainWindow/mainwindow.h \
    TorqueID/torqueid.h \
    OperateUI/operateui.h \

FORMS += \
    MainWindow/mainwindow.ui \
    OperateUI/operateui.ui \
    TorqueID/torqueid.ui \

LIBS += -L$$PWD/lib/ -lRobotControllerUI

INCLUDEPATH += $$PWD/.
DEPENDPATH += $$PWD/.
