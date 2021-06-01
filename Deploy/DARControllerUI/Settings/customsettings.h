#pragma once

#include <QSettings>
#include <QFile>
#include <QDesktopServices>
#include <QApplication>
#include <QWidget>
#include <QMainWindow>

#include <QtCore/qglobal.h>

#if defined(CUSTOMSETTINGSLIB_LIBRARY)
#  define CUSTOMSETTINGSLIB_EXPORT Q_DECL_EXPORT
#else
#  define CUSTOMSETTINGSLIB_EXPORT Q_DECL_IMPORT
#endif

class CustomSettings : public QSettings{
public:
    CustomSettings(void* _ui, void* _ui_op);
    void saveConfigFile();
    void loadConfigFile();
    QString configFile();

private:
    void *ui;
    void *ui_op;
};
