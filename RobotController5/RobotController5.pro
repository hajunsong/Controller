#-------------------------------------------------
#
# Project created by QtCreator 2021-03-15T21:46:44
#
#-------------------------------------------------

QT += core
QT -= gui

TARGET = RobotController
TEMPLATE = app
DESTDIR = $$PWD/bin

INCLUDEPATH += \
    /opt/xenomai-2.6.4/include \
    $$PWD/include \

LIBS += -lpthread -L/opt/xenomai-2.6.4/lib -lxenomai -lnative
DEFINES += __LINUX_ARM_ARCH__

target.path = /mnt/mtd5/daincube
INSTALLS += target

SOURCES += \
    src/main.cpp \
    src/controlmain.cpp \
    src/custom_slave_information.cpp \
    src/daininterface.cpp \
    src/ecatmaster.cpp \
    src/ecatslave.cpp \

HEADERS  += \
    include/controlmain.h \
    include/ecatmaster.h \
    include/globalstruct.h \
    include/daininterface.h \
    include/esi.h \
    include/ecatslave.h \

