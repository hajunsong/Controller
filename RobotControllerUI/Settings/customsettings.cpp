#include "MainWindow/mainwindow.h"
#include "ui_mainwindow.h"
#include "customsettings.h"

CustomSettings::CustomSettings(void *_ui)
{
    ui = static_cast<Ui::MainWindow*>(_ui);
}

void CustomSettings::saveConfigFile()
{
    QSettings settings(configFile(), QSettings::IniFormat);

//    settings.setValue("geometry", saveGeometry());
//    settings.setValue("windowState", saveState());

    QString ip = static_cast<Ui::MainWindow*>(ui)->txtIP->text();
    QString port = static_cast<Ui::MainWindow*>(ui)->txtPORT->text();

    settings.setValue("IP", ip);
    settings.setValue("PORT", port);
    settings.sync();
}

void CustomSettings::loadConfigFile()
{
    if (!QFile::exists(configFile())) return;

    QSettings settings(configFile(), QSettings::IniFormat);

//    restoreGeometry(settings.value("geometry").toByteArray());
//    restoreState(settings.value("windowState").toByteArray());

    QString ip = settings.value("IP").toString();
    QString port = settings.value("PORT").toString();

    static_cast<Ui::MainWindow*>(ui)->txtIP->setText(ip);
    static_cast<Ui::MainWindow*>(ui)->txtPORT->setText(port);
}

QString CustomSettings::configFile()
{
    QString filePath = qApp->applicationDirPath() + "/config.ini";
    return filePath;
}

