QT -= gui

TEMPLATE = lib
DEFINES += FILEIOLIB_LIBRARY

DESTDIR = $$PWD/fileio_lib

CONFIG += c++11

SOURCES += \
    fileio.cpp

HEADERS += \
    fileio.h
