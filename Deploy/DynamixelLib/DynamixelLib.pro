QT -= gui

TEMPLATE = lib
DEFINES += DYNAMIXELLIB_LIBRARY

DESTDIR = $$PWD/dynamixel_lib

CONFIG += c++11

SOURCES += \
    dynamixel.cpp

HEADERS += \ \
    dynamixel.h
