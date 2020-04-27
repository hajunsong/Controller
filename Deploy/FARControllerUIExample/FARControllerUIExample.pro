QT += core gui widgets network

TARGET = RobotControllerUIExample
TEMPLATE = app
CONFIG += c++11

SOURCES += \
    main.cpp \
    MainWindow/mainwindow.cpp \
    TcpSocket/tcpclient.cpp \

HEADERS += \
    MainWindow/mainwindow.h \
    TcpSocket/tcpclient.h \

FORMS += \
    MainWindow/mainwindow.ui \
