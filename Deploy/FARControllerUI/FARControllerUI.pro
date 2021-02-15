QT       += core gui widgets network

CONFIG += c++11

SOURCES += \
    main.cpp \

DEFINES += QT_NO_VERSION_TAGGING

HEADERS += \
    DataControl/datacontrol.h \
    Logger/logger.h \
    Settings/customsettings.h \
    TcpSocket/tcpclient.h \
    TcpSocket/tcpserver.h \
    Input/keyinputclass.h \
    Input/keyinputnomodifiedclass.h \
    MainWindow/mainwindow.h \
    OperateUI/operateui.h \
    TorqueID/torqueid.h \

LIBS += -L/$$PWD/lib -lRobotControllerUILib
