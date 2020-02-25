QT -= gui
QT += core network widgets

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
    CUSTOMSETTINGSLIB_LIBRARY \

DESTDIR = $$PWD/../Deploy/FARController/lib

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
    Settings/customsettings.cpp \

DEPLOY_PATH = /home/keti/Project/Controller/Deploy

HEADERS += \
    $$DEPLOY_PATH/FARController/DataControl/datacontrol.h \
    $$DEPLOY_PATH/FARController/Dynamixel/dynamixel.h \
    $$DEPLOY_PATH/FARController/FileIO/fileio.h \
    $$DEPLOY_PATH/FARController/RobotArm/robotarm.h \
    $$DEPLOY_PATH/FARController/RobotArm/numerical.h \
    $$DEPLOY_PATH/FARController/Input/keyinput.h \
    $$DEPLOY_PATH/FARController/ControlMain/controlmain.h \
    $$DEPLOY_PATH/FARController/TcpServer/tcpserver.h \
    $$DEPLOY_PATH/FARController/CustomFunc/controlmain_custom.h \
    $$DEPLOY_PATH/FARController/CustomFunc/tcpserver_custom.h \
    $$DEPLOY_PATH/FARController/Settings/customsettings.h \

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
    $$DEPLOY_PATH/FARController \
    $$DEPLOY_PATH/FARController/DataControl \
    $$DEPLOY_PATH/FARController/Dynamixel \
    $$DEPLOY_PATH/FARController/FileIO \
    $$DEPLOY_PATH/FARController/RobotArm \
    $$DEPLOY_PATH/FARController/Input \
    $$DEPLOY_PATH/FARController/ControlMain \
    $$DEPLOY_PATH/FARController/TcpServer \
    $$DEPLOY_PATH/FARController/CustomFunc \
    $$DEPLOY_PATH/FARController/Settings \

DEPENDPATH += \
    $$NRMK_PATH \
    $$NRMK_POCO_LIB_PATH/i686 \
    $$NRMK_XENO_PATH/include \
    $$NRMK_POCO_INC_PATH \
    $$NRMK_HELPER_INC_PATH \

PRE_TARGETDEPS += \
    $$NRMK_LIB_PATH/libNRMKHelperi686.a
