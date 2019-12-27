#pragma once

#include <QSettings>
#include <QFile>
#include <QDesktopServices>
#include <QApplication>
#include <QWidget>
#include <QMainWindow>
#include <QtDebug>

#include "DataControl/datacontrol.h"

class CustomSettings : public QSettings{
public:
    CustomSettings();
    void saveConfigFile(int32_t* present_enc);
    bool loadConfigFile(int32_t* present_enc);
    QString configFile();
private:
    int32_t enc[NUM_JOINT];
};
