QT -= gui
QT += core

CONFIG += c++11 console
CONFIG -= app_bundle

include($$PWD/NRMKLib.pri)

target.path = /home/user/Project/RobotController
INSTALLS += target

SOURCES += main.cpp \
        Dynamixel/H42_20_S300_RA.cpp \
        Dynamixel/XH_540_Series.cpp \
        ControlMain/controlmain.cpp \
        DataControl/datacontrol.cpp \
        TcpServer/tcpserver.cpp \
        RobotArm/robotarm.cpp \
        Numerical/numerical.cpp \
        Input/keyinput.cpp \
        FileIO/fileio.cpp \

HEADERS += \
    Dynamixel/H42_20_S300_RA.h \
    Dynamixel/XH_540_Series.h \
    ControlMain/controlmain.h \
    DataControl/datacontrol.h \
    TcpServer/tcpserver.h \
    RobotArm/robotarm.h \
    Numerical/numerical.h \
    Input/keyinput.h \
    FileIO/fileio.h \

DXL_PATH = $$PWD/Dynamixel

LIBS += -L$$NRMK_LIB_PATH/ -lNRMKHelperi686 \
    -L$$NRMK_POCO_LIB_PATH/i686/ -lPocoNet \
    -L$$NRMK_POCO_LIB_PATH/i686/ -lPocoFoundation \
    -L$$NRMK_XENO_PATH/lib/ -lnative \
    -L$$NRMK_XENO_PATH/lib/ -lrtdm \
    -L$$NRMK_XENO_PATH/lib/ -lxenomai \
    -L$$DXL_PATH/lib -ldxl_x86_cpp \

INCLUDEPATH += $$NRMK_PATH \
            $$NRMK_POCO_LIB_PATH/i686 \
            $$NRMK_XENO_PATH/include \
            $$NRMK_POCO_INC_PATH \
            $$NRMK_HELPER_INC_PATH \
            $$DXL_PATH \

DEPENDPATH += $$NRMK_PATH \
            $$NRMK_POCO_LIB_PATH/i686 \
            $$NRMK_XENO_PATH/include \
            $$NRMK_POCO_INC_PATH \
            $$NRMK_HELPER_INC_PATH \
            $$DXL_PATH \

PRE_TARGETDEPS += $$NRMK_LIB_PATH/libNRMKHelperi686.a
