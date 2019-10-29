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
    int num_joint = static_cast<Ui::MainWindow*>(ui)->cbNumJoint->currentIndex();
    int num_dof = static_cast<Ui::MainWindow*>(ui)->cbNumDOF->currentIndex();
    int module_type = static_cast<Ui::MainWindow*>(ui)->cbModuleType->currentIndex();
    int joint_mode = static_cast<Ui::MainWindow*>(ui)->cbJointMode->currentIndex();

    settings.setValue("IP", ip);
    settings.setValue("PORT", port);
    settings.setValue("NUM_JOINT", num_joint);
    settings.setValue("NUM_DOF", num_dof);
    settings.setValue("MODULE_TYPE", module_type);
    settings.setValue("JOINT_MODE", joint_mode);
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
    int num_joint = settings.value("NUM_JOINT").toInt();
    int num_dof = settings.value("NUM_DOF").toInt();
    int module_type = settings.value("MODULE_TYPE").toInt();
    int joint_mode = settings.value("JOINT_MODE").toInt();

    static_cast<Ui::MainWindow*>(ui)->txtIP->setText(ip);
    static_cast<Ui::MainWindow*>(ui)->txtPORT->setText(port);
    static_cast<Ui::MainWindow*>(ui)->cbNumJoint->setCurrentIndex(num_joint);
    static_cast<Ui::MainWindow*>(ui)->cbNumDOF->setCurrentIndex(num_dof);
    static_cast<Ui::MainWindow*>(ui)->cbModuleType->setCurrentIndex(module_type);
    static_cast<Ui::MainWindow*>(ui)->cbJointMode->setCurrentIndex(joint_mode);
}

QString CustomSettings::configFile()
{
    QString filePath = qApp->applicationDirPath() + "/config.ini";
    return filePath;
}

