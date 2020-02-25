#pragma once

#include <QSettings>
#include <QFile>
#include <QDesktopServices>
#include <QApplication>
#include <QWidget>
#include <QMainWindow>
#include <QtDebug>

#include "DataControl/datacontrol.h"

#include <QtCore/qglobal.h>

#if defined(CUSTOMSETTINGSLIB_LIBRARY)
#  define CUSTOMSETTINGSLIB_EXPORT Q_DECL_EXPORT
#else
#  define CUSTOMSETTINGSLIB_EXPORT Q_DECL_IMPORT
#endif

class CustomSettings : public QSettings{
public:
    CustomSettings();
    void saveConfigFile(int32_t* present_enc);
    bool loadConfigFile(int32_t* present_enc);
    QString configFile();
private:
    int32_t enc[NUM_JOINT];
};
