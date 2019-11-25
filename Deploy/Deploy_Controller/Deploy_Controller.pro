QT -= gui
QT += core network

CONFIG += c++11 console
CONFIG -= app_bundle

include($$PWD/NRMKLib.pri)

target.path = /home/user/Project/RobotController
INSTALLS += target

SOURCES += main.cpp \
        ControlMain/controlmain.cpp \
        TcpServer/tcpserver.cpp \

HEADERS += \
        ControlMain/controlmain.h \
        DataControl/datacontrol.h \
        TcpServer/tcpserver.h \
        FileIO/fileio.h \
        DataControl/datacontrol.h \
        RobotArm/robotarm.h \
        RobotArm/numerical.h \
        Dynamixel/dynamixel.h \

LIBS += -L$$NRMK_LIB_PATH/ -lNRMKHelperi686 \
    -L$$NRMK_POCO_LIB_PATH/i686/ -lPocoNet \
    -L$$NRMK_POCO_LIB_PATH/i686/ -lPocoFoundation \
    -L$$NRMK_XENO_PATH/lib/ -lnative \
    -L$$NRMK_XENO_PATH/lib/ -lrtdm \
    -L$$NRMK_XENO_PATH/lib/ -lxenomai \

INCLUDEPATH += $$NRMK_PATH \
            $$NRMK_POCO_LIB_PATH/i686 \
            $$NRMK_XENO_PATH/include \
            $$NRMK_POCO_INC_PATH \
            $$NRMK_HELPER_INC_PATH \

DEPENDPATH += $$NRMK_PATH \
            $$NRMK_POCO_LIB_PATH/i686 \
            $$NRMK_XENO_PATH/include \
            $$NRMK_POCO_INC_PATH \
            $$NRMK_HELPER_INC_PATH \

PRE_TARGETDEPS += $$NRMK_LIB_PATH/libNRMKHelperi686.a

LIBS += -L$$PWD/FileIO/fileio_lib/ -lFileIOLib

INCLUDEPATH += $$PWD/FileIO/fileio_lib
DEPENDPATH += $$PWD/FileIO/fileio_lib

LIBS += -L$$PWD/DataControl/datacontrol_lib/ -lDataControlLib

INCLUDEPATH += $$PWD/DataControl/datacontrol_lib
DEPENDPATH += $$PWD/DataControl/datacontrol_lib

LIBS += -L$$PWD/RobotArm/robotarm_lib/ -lRobotArmLib

INCLUDEPATH += $$PWD/RobotArm/robotarm_lib
DEPENDPATH += $$PWD/RobotArm/robotarm_lib

LIBS += -L$$PWD/Dynamixel/dynamixel_lib/ -lDynamixelLib

INCLUDEPATH += $$PWD/Dynamixel/dynamixel_lib
DEPENDPATH += $$PWD/Dynamixel/dynamixel_lib

LIBS += -L$$PWD/Dynamixel/dynamixel_lib/ -ldxl_x86_cpp

INCLUDEPATH += $$PWD/Dynamixel/dynamixel_lib
DEPENDPATH += $$PWD/Dynamixel/dynamixel_lib
