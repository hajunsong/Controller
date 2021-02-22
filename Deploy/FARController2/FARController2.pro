QT -= gui
QT += core

CONFIG += c++11 console
CONFIG -= app_bundle

include($$PWD/NRMKLib.pri)

target.path = /home/user/Project/RobotController
INSTALLS += target

TARGET = FARController2

SOURCES += main.cpp \
    CustomFunc/controlmain_custom.cpp \

HEADERS += \
    ControlMain/controlmain.h \
    Dynamixel/dynamixel.h \
    DataControl/datacontrol.h \
    TcpServer/tcpserver.h \
    RobotArm/robotarm.h \
    RobotArm/numerical.h \
    FileIO/fileio.h \
    CustomFunc/controlmain_custom.h \

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

LIBS += -L$$PWD/./ -lFARController2

INCLUDEPATH += $$PWD/.
DEPENDPATH += $$PWD/.
