QT -= gui
QT += core network widgets

CONFIG += c++11 console
CONFIG -= app_bundle

include($$PWD/NRMKLib.pri)

target.path = /home/user/Project/RobotController
INSTALLS += target

SOURCES += main.cpp \
    ControlMain/controlmain.cpp \
    CustomFunc/tcpserver_custom.cpp \
    CustomFunc/controlmain_custom.cpp \

HEADERS += \
    ControlMain/controlmain.h \
    Dynamixel/dynamixel.h \
    DataControl/datacontrol.h \
    TcpServer/tcpserver.h \
    RobotArm/robotarm.h \
    RobotArm/numerical.h \
    Input/keyinput.h \
    FileIO/fileio.h \
    CustomFunc/tcpserver_custom.h \
    CustomFunc/controlmain_custom.h \
    Settings/customsettings.h \

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

DEPENDPATH += \
    $$NRMK_PATH \
    $$NRMK_POCO_LIB_PATH/i686 \
    $$NRMK_XENO_PATH/include \
    $$NRMK_POCO_INC_PATH \
    $$NRMK_HELPER_INC_PATH \

PRE_TARGETDEPS += \
    $$NRMK_LIB_PATH/libNRMKHelperi686.a

unix:!macx: LIBS += -L$$PWD/lib/ -lRobotControllerLib

INCLUDEPATH += $$PWD/.
DEPENDPATH += $$PWD/.

unix:!macx: LIBS += -L$$PWD/Dynamixel/ -ldxl_x86_cpp

INCLUDEPATH += $$PWD/Dynamixel
DEPENDPATH += $$PWD/Dynamixel
