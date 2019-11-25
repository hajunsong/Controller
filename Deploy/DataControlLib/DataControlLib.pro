QT -= gui

TEMPLATE = lib
DEFINES += DATACONTROLLIB_LIBRARY

DESTDIR = $$PWD/datacontrol_lib

CONFIG += c++11

include(/home/hajun/Project/Controller/Deploy/Deploy_Controller/NRMKLib.pri)

SOURCES += \
    datacontrol.cpp

HEADERS += \
    datacontrol.h \

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

LIBS += -L$$PWD/../Deploy_Controller/FileIO/fileio_lib/ -lFileIOLib

INCLUDEPATH += $$PWD/../Deploy_Controller/FileIO/fileio_lib
DEPENDPATH += $$PWD/../Deploy_Controller/FileIO/fileio_lib
