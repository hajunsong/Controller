QT -= gui
QT += core network

CONFIG += c++11
CONFIG -= app_bundle

TEMPLATE = lib
DEFINES += \
    DATACONTROLLIB_LIBRARY \
    DYNAMIXELLIB_LIBRARY \
    FILEIOLIB_LIBRARY \
    KEYINPUTLIB_LIBRARY \
    ROBOTARMLIB_LIBRARY \
    CONTROLMAINLIB_LIBRARY \
    TCPSERVERLIB_LIBRARY \

DESTDIR = $$PWD/../Deploy/RobotController/lib

target.path = /home/user/Project/RobotController
INSTALLS += target

include($$PWD/NRMKLib.pri)

SOURCES += \
    DataControl/datacontrol.cpp \
    Dynamixel/dynamixel.cpp \
    FileIO/fileio.cpp \
    RobotArm/robotarm.cpp \
    RobotArm/numerical.cpp \
    Input/keyinput.cpp \
    ControlMain/controlmain.cpp \
    TcpServer/tcpserver.cpp \
    CustomFunc/controlmain_custom.cpp \
    CustomFunc/tcpserver_custom.cpp \

DEPLOY_PATH = /home/keti/Project/Controller/Deploy

HEADERS += \
    $$DEPLOY_PATH/RobotController/DataControl/datacontrol.h \
    $$DEPLOY_PATH/RobotController/Dynamixel/dynamixel.h \
    $$DEPLOY_PATH/RobotController/FileIO/fileio.h \
    $$DEPLOY_PATH/RobotController/RobotArm/robotarm.h \
    $$DEPLOY_PATH/RobotController/RobotArm/numerical.h \
    $$DEPLOY_PATH/RobotController/Input/keyinput.h \
    $$DEPLOY_PATH/RobotController/ControlMain/controlmain.h \
    $$DEPLOY_PATH/RobotController/TcpServer/tcpserver.h \
    $$DEPLOY_PATH/RobotController/CustomFunc/controlmain_custom.h \
    $$DEPLOY_PATH/RobotController/CustomFunc/tcpserver_custom.h \

LIBS += \
    -L$$NRMK_LIB_PATH/ -lNRMKHelperi686 \
    -L$$NRMK_POCO_LIB_PATH/i686/ -lPocoNet \
    -L$$NRMK_POCO_LIB_PATH/i686/ -lPocoFoundation \
    -L$$NRMK_XENO_PATH/lib/ -lnative \
    -L$$NRMK_XENO_PATH/lib/ -lrtdm \
    -L$$NRMK_XENO_PATH/lib/ -lxenomai \

INCLUDEPATH += \
    $$NRMK_PATH \
    $$NRMK_POCO_LIB_PATH/i686 \
    $$NRMK_XENO_PATH/include \
    $$NRMK_POCO_INC_PATH \
    $$NRMK_HELPER_INC_PATH \
    $$DEPLOY_PATH/RobotController \
    $$DEPLOY_PATH/RobotController/DataControl \
    $$DEPLOY_PATH/RobotController/Dynamixel \
    $$DEPLOY_PATH/RobotController/FileIO \
    $$DEPLOY_PATH/RobotController/RobotArm \
    $$DEPLOY_PATH/RobotController/Input \
    $$DEPLOY_PATH/RobotController/ControlMain \
    $$DEPLOY_PATH/RobotController/TcpServer \
    $$DEPLOY_PATH/RobotController/CustomFunc \

DEPENDPATH += \
    $$NRMK_PATH \
    $$NRMK_POCO_LIB_PATH/i686 \
    $$NRMK_XENO_PATH/include \
    $$NRMK_POCO_INC_PATH \
    $$NRMK_HELPER_INC_PATH \

PRE_TARGETDEPS += \
    $$NRMK_LIB_PATH/libNRMKHelperi686.a
