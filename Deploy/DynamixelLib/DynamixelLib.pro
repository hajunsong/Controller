QT -= gui

TEMPLATE = lib
DEFINES += DYNAMIXELLIB_LIBRARY

DESTDIR = $$PWD/lib

CONFIG += c++11

SOURCES += \
    dynamixellib.cpp

HEADERS += \
    dynamixellib.h \
