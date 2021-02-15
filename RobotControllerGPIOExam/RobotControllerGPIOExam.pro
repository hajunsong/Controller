QT -= gui
QT += core

CONFIG += c++11 console
CONFIG -= app_bundle

include($$PWD/NRMKLib.pri)

target.path = /home/user/Project/RobotController
INSTALLS += target

SOURCES += \
        main.cpp

LIBS += \
	-L$$NRMK_XENO_PATH/lib/ -lnative \
	-L$$NRMK_XENO_PATH/lib/ -lrtdm \
	-L$$NRMK_XENO_PATH/lib/ -lxenomai \
        -L$$NRMK_LIB_PATH -lNRMKhw_tp \

INCLUDEPATH += \
    $$NRMK_PATH \
    $$NRMK_XENO_PATH/include \
    $$NRMK_HELPER_INC_PATH/hw \

DEPENDPATH += \
    $$NRMK_PATH \
    $$NRMK_XENO_PATH/include \
    $$NRMK_HELPER_INC_PATH/hw \

#TARGET = RobotController
