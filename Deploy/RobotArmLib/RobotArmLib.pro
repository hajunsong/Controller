QT -= gui

TEMPLATE = lib
DEFINES += ROBOTARMLIB_LIBRARY

CONFIG += c++11

DESTDIR = $$PWD/robotarm_lib

SOURCES += \
    robotarm.cpp \
    numerical.cpp


HEADERS += \
    robotarm.h \
    numerical.h
