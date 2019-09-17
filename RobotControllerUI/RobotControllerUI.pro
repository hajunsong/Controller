#-------------------------------------------------
#
# Project created by QtCreator 2019-08-19T12:42:15
#
#-------------------------------------------------

QT       += core gui widgets network

TARGET = RobotControllerUI
TEMPLATE = app

CONFIG += c++11 console

SOURCES += \
        DataControl/datacontrol.cpp \
        FileIO/fileio.cpp \
        Logger/logger.cpp \
        Settings/customsettings.cpp \
        TcpSocket/tcpclient.cpp \
        main.cpp \
        mainwindow.cpp

HEADERS += \
        DataControl/datacontrol.h \
        FileIO/fileio.h \
        Logger/logger.h \
        Settings/customsettings.h \
        TcpSocket/tcpclient.h \
        mainwindow.h

FORMS += \
        mainwindow.ui
