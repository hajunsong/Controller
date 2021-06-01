QT -= gui
QT += core

CONFIG += c++11
CONFIG -= app_bundle

include($$PWD/NRMKLib.pri)

#target.path = /usr/local/lib
target.path = /home/user/Project/RobotController
INSTALLS += target

TARGET = DARControllerTest
TEMPLATE = app
DESTDIR = $$PWD/bin/

#TEMPLATE = lib
#DESTDIR = $$PWD/lib/

SOURCES += \
    src/main.cpp \
    src/controlmain.cpp \
    src/dynamixel.cpp \
    src/datacontrol.cpp \
    src/tcpserver.cpp \
    src/robotarm.cpp \
    src/numerical.cpp \
    src/fileio.cpp \
    src/controlmain_custom.cpp \

HEADERS += \
    include/controlmain.h \
    include/dynamixel.h \
    include/datacontrol.h \
    include/tcpserver.h \
    include/robotarm.h \
    include/numerical.h \
    include/fileio.h \
    include/controlmain_custom.h \

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
    $$PWD/include

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
