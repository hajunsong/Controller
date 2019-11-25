QT -= gui

TEMPLATE = lib
DEFINES += KEYINPUTLIB_LIBRARY

CONFIG += c++11

DESTDIR = $$PWD/keyinput_lib


SOURCES += \
    keyinput.cpp

HEADERS += \
    keyinput.h
