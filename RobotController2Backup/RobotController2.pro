QT -= gui

CONFIG += c++11 console
CONFIG -= app_bundle

include($$PWD/NRMKLib.pri)

target.path = /home/user/Project/RobotController
INSTALLS += target

TARGET = FARController2
#TEMPLATE = lib

DXL_PATH = $$PWD/Dynamixel

LIBS += \
    -L$$NRMK_LIB_PATH/ -lNRMKHelperi686 \
    -L$$NRMK_POCO_LIB_PATH/i686/ -lPocoNet \
    -L$$NRMK_POCO_LIB_PATH/i686/ -lPocoFoundation \
    -L$$NRMK_XENO_PATH/lib/ -lnative \
    -L$$NRMK_XENO_PATH/lib/ -lrtdm \
    -L$$NRMK_XENO_PATH/lib/ -lxenomai \
    -L$$NRMK_LIB_PATH -lNRMKhw_tp \
    -L$$DXL_PATH -ldxl_x86_cpp \

INCLUDEPATH += \
    $$NRMK_PATH \
    $$NRMK_POCO_LIB_PATH/i686 \
    $$NRMK_XENO_PATH/include \
    $$NRMK_POCO_INC_PATH \
    $$NRMK_HELPER_INC_PATH \
    $$NRMK_HELPER_INC_PATH/hw \
    $$DXL_PATH \

DEPENDPATH += \
    $$NRMK_PATH \
    $$NRMK_POCO_LIB_PATH/i686 \
    $$NRMK_XENO_PATH/include \
    $$NRMK_POCO_INC_PATH \
    $$NRMK_HELPER_INC_PATH \
    $$NRMK_HELPER_INC_PATH/hw \
    $$DXL_PATH \

PRE_TARGETDEPS += \
    $$NRMK_LIB_PATH/libNRMKHelperi686.a

SOURCES += main.cpp \
    ControlMain/controlmain.cpp \
    CustomFunc/controlmain_custom.cpp \
    DataControl/datacontrol.cpp \
    Dynamixel/dynamixel.cpp \
    FileIO/fileio.cpp \
    RobotArm/numerical.cpp \
    RobotArm/robotarm.cpp \
    TcpServer/tcpserver.cpp \

HEADERS += \
    ControlMain/controlmain.h \
    CustomFunc/controlmain_custom.h \
    DataControl/datacontrol.h \
    Dynamixel/dynamixel.h \
    FileIO/fileio.h \
    RobotArm/numerical.h \
    RobotArm/robotarm.h \
    TcpServer/tcpserver.h \
