QT += core gui widgets network

CONFIG += c++11

TEMPLATE = lib
DEFINES += \
    DATACONTROLLIB_LIBRARY \
    KEYINPUTLIB_LIBRARY \
    KEYINPUTNOMODIFIEDLIB_LIBRARY \
    LOGGERLIB_LIBRARY \
    MAINWINDOWLIB_LIBRARY \
    OPERATEUILIB_LIBRARY \
    CUSTOMSETTINGSLIB_LIBRARY \
    TCPCLIENTLIB_LIBRARY \
    TCPSERVERLIB_LIBRARY \
    TORQUEIDLIB_LIBRARY \

DESTDIR = $$PWD/../Deploy/FARControllerUI/lib

SOURCE_PATH = $$PWD/../RobotControllerUI

SOURCES += \
    $$SOURCE_PATH/DataControl/datacontrol.cpp \
    $$SOURCE_PATH/Logger/logger.cpp \
    $$SOURCE_PATH/Settings/customsettings.cpp \
    $$SOURCE_PATH/TcpSocket/tcpclient.cpp \
    $$SOURCE_PATH/TcpSocket/tcpserver.cpp \
    $$SOURCE_PATH/Input/keyinputclass.cpp \
    $$SOURCE_PATH/Input/keyinputnomodifiedclass.cpp \
    $$SOURCE_PATH/MainWindow/mainwindow.cpp \
    $$SOURCE_PATH/OperateUI/operateui.cpp \
    $$SOURCE_PATH/TorqueID/torqueid.cpp \

HEADERS += \
    $$SOURCE_PATH/DataControl/datacontrol.h \
    $$SOURCE_PATH/Logger/logger.h \
    $$SOURCE_PATH/Settings/customsettings.h \
    $$SOURCE_PATH/TcpSocket/tcpclient.h \
    $$SOURCE_PATH/TcpSocket/tcpserver.h \
    $$SOURCE_PATH/Input/keyinputclass.h \
    $$SOURCE_PATH/Input/keyinputnomodifiedclass.h \
    $$SOURCE_PATH/MainWindow/mainwindow.h \
    $$SOURCE_PATH/OperateUI/operateui.h \
    $$SOURCE_PATH/TorqueID/torqueid.h \

FORMS += \
    $$SOURCE_PATH/MainWindow/mainwindow.ui \
    $$SOURCE_PATH/OperateUI/operateui.ui \
    $$SOURCE_PATH/TorqueID/torqueid.ui

INCLUDEPATH += \
    $$SOURCE_PATH \
    $$SOURCE_PATH/DataControl \
    $$SOURCE_PATH/Logger \
    $$SOURCE_PATH/MainWindow \
    $$SOURCE_PATH/OperateUI \
    $$SOURCE_PATH/Settings \
    $$SOURCE_PATH/TcpSocket \
    $$SOURCE_PATH/TorqueID \
