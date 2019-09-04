QT -= gui

CONFIG += c++11 console
CONFIG -= app_bundle

include(NRMKLib.pri)

target.path = /home/user/Project/RobotController
INSTALLS += target

LIBS += -L$$PWD/../../../../../opt/neuromeka/NRMKFoundation/lib/ -lNRMKHelperi686

INCLUDEPATH += $$PWD/../../../../../opt/neuromeka/NRMKFoundation
DEPENDPATH += $$PWD/../../../../../opt/neuromeka/NRMKFoundation

PRE_TARGETDEPS += $$PWD/../../../../../opt/neuromeka/NRMKFoundation/lib/libNRMKHelperi686.a

LIBS += -L$$PWD/../../../../../opt/neuromeka/NRMKFoundation/core/3rdparty/Poco/lib/i686/ -lPocoNet

INCLUDEPATH += $$PWD/../../../../../opt/neuromeka/NRMKFoundation/core/3rdparty/Poco/lib/i686
DEPENDPATH += $$PWD/../../../../../opt/neuromeka/NRMKFoundation/core/3rdparty/Poco/lib/i686

LIBS += -L$$PWD/../../../../../opt/neuromeka/NRMKFoundation/core/3rdparty/Poco/lib/i686/ -lPocoFoundation

INCLUDEPATH += $$PWD/../../../../../opt/neuromeka/NRMKFoundation/core/3rdparty/Poco/lib/i686
DEPENDPATH += $$PWD/../../../../../opt/neuromeka/NRMKFoundation/core/3rdparty/Poco/lib/i686

LIBS += -L$$PWD/../../../../../opt/neuromeka/NRMKPlatformPC2/bin/i686/xenomai/lib/ -lnative

INCLUDEPATH += $$PWD/../../../../../opt/neuromeka/NRMKPlatformPC2/bin/i686/xenomai/include
DEPENDPATH += $$PWD/../../../../../opt/neuromeka/NRMKPlatformPC2/bin/i686/xenomai/include

LIBS += -L$$PWD/../../../../../opt/neuromeka/NRMKPlatformPC2/bin/i686/xenomai/lib/ -lrtdm

INCLUDEPATH += $$PWD/../../../../../opt/neuromeka/NRMKPlatformPC2/bin/i686/xenomai/include
DEPENDPATH += $$PWD/../../../../../opt/neuromeka/NRMKPlatformPC2/bin/i686/xenomai/include

LIBS += -L$$PWD/../../../../../opt/neuromeka/NRMKPlatformPC2/bin/i686/xenomai/lib/ -lxenomai

INCLUDEPATH += $$PWD/../../../../../opt/neuromeka/NRMKPlatformPC2/bin/i686/xenomai/include
DEPENDPATH += $$PWD/../../../../../opt/neuromeka/NRMKPlatformPC2/bin/i686/xenomai/include

INCLUDEPATH += $$PWD/../../../../../opt/neuromeka/NRMKFoundation/core/3rdparty/Poco/include
DEPENDPATH += $$PWD/../../../../../opt/neuromeka/NRMKFoundation/core/3rdparty/Poco/include

INCLUDEPATH += $$PWD/../../../../..//opt/neuromeka/NRMKFoundation/helper/include
DEPENDPATH += $$PWD/../../../../..//opt/neuromeka/NRMKFoundation/helper/include

LIBS += -ldxl_x86_cpp

SOURCES += \
        Dynamixel/H42_20_S300_RA.cpp \
        Dynamixel/XH_540_Series.cpp \
        main.cpp

HEADERS += \
    Dynamixel/H42_20_S300_RA.h \
    Dynamixel/XH_540_Series.h
