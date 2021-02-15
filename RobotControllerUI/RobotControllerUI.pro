#-------------------------------------------------
#
# Project created by QtCreator 2019-08-19T12:42:15
#
#-------------------------------------------------

QT += core gui widgets network

TARGET = RobotControllerUI
TEMPLATE = app

CONFIG += c++11

SOURCES += \
    DataControl/datacontrol.cpp \
    FileIO/fileio.cpp \
    Logger/logger.cpp \
    Settings/customsettings.cpp \
    TcpSocket/tcpclient.cpp \
    TcpSocket/tcpserver.cpp \
    Input/keyinputclass.cpp \
    Input/keyinputnomodifiedclass.cpp \
    Input/keyinputkeypadclass.cpp \
    MainWindow/mainwindow.cpp \
    TorqueID/torqueid.cpp \
    OperateUI/operateui.cpp \
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
