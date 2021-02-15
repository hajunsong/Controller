#include "MainWindow/mainwindow.h"
#include "ui_mainwindow.h"
#include "customsettings.h"
#include "OperateUI/operateui.h"
#include "ui_operateui.h"

CustomSettings::CustomSettings(void *_ui, void *_ui_op)
{
    ui = static_cast<Ui::MainWindow*>(_ui);
    ui_op = static_cast<Ui::OperateUI*>(_ui_op);
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
    QString tablet_ip = static_cast<Ui::OperateUI*>(ui_op)->txtIP->text();
    QString tablet_port = static_cast<Ui::OperateUI*>(ui_op)->txtPORT->text();

    settings.setValue("IP", ip);
    settings.setValue("PORT", port);
    settings.setValue("NUM_JOINT", num_joint);
    settings.setValue("NUM_DOF", num_dof);
    settings.setValue("MODULE_TYPE", module_type);
    settings.setValue("JOINT_MODE", joint_mode);
    settings.setValue("TABLET_IP", tablet_ip);
    settings.setValue("TABLET_PORT", tablet_port);

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
    QString tablet_ip = settings.value("TABLET_IP").toString();
    QString tablet_port = settings.value("TABLET_PORT").toString();

    static_cast<Ui::MainWindow*>(ui)->txtIP->setText(ip);
    static_cast<Ui::MainWindow*>(ui)->txtPORT->setText(port);
    static_cast<Ui::MainWindow*>(ui)->cbNumJoint->setCurrentIndex(num_joint);
    static_cast<Ui::MainWindow*>(ui)->cbNumDOF->setCurrentIndex(num_dof);
    static_cast<Ui::MainWindow*>(ui)->cbModuleType->setCurrentIndex(module_type);
    static_cast<Ui::MainWindow*>(ui)->cbJointMode->setCurrentIndex(joint_mode);
    static_cast<Ui::OperateUI*>(ui_op)->txtIP->setText(tablet_ip);
    static_cast<Ui::OperateUI*>(ui_op)->txtPORT->setText(tablet_port);
}

QString CustomSettings::configFile()
{
    QString filePath = qApp->applicationDirPath() + "/config.ini";
//    qDebug() << filePath;
    return filePath;
}

