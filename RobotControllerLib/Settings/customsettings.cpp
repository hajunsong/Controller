#include "customsettings.h"

CustomSettings::CustomSettings()
{
}

void CustomSettings::saveConfigFile(int32_t* present_enc)
{
    QSettings settings(configFile(), QSettings::IniFormat);

    settings.setValue("q1", present_enc[0]);
    settings.setValue("q2", present_enc[1]);
    settings.setValue("q3", present_enc[2]);
    settings.setValue("q4", present_enc[3]);
    settings.setValue("q5", present_enc[4]);
    settings.setValue("q6", present_enc[5]);
    settings.sync();

//    settings.setValue("geometry", saveGeometry());
//    settings.setValue("windowState", saveState());

//    QString ip = static_cast<Ui::MainWindow*>(ui)->txtIP->text();
//    QString port = static_cast<Ui::MainWindow*>(ui)->txtPORT->text();
//    int num_joint = static_cast<Ui::MainWindow*>(ui)->cbNumJoint->currentIndex();
//    int num_dof = static_cast<Ui::MainWindow*>(ui)->cbNumDOF->currentIndex();
//    int module_type = static_cast<Ui::MainWindow*>(ui)->cbModuleType->currentIndex();
//    int joint_mode = static_cast<Ui::MainWindow*>(ui)->cbJointMode->currentIndex();

//    settings.setValue("IP", ip);
//    settings.setValue("PORT", port);
//    settings.setValue("NUM_JOINT", num_joint);
//    settings.setValue("NUM_DOF", num_dof);
//    settings.setValue("MODULE_TYPE", module_type);
//    settings.setValue("JOINT_MODE", joint_mode);
//    settings.sync();
}

bool CustomSettings::loadConfigFile(int32_t* present_enc)
{
    bool exist = false;
    exist = QFile::exists(configFile());
    if (!exist){
        return exist;
    }

    QSettings settings(configFile(), QSettings::IniFormat);

    present_enc[0] = settings.value("q1").toInt();
    present_enc[1] = settings.value("q2").toInt();
    present_enc[2] = settings.value("q3").toInt();
    present_enc[3] = settings.value("q4").toInt();
    present_enc[4] = settings.value("q5").toInt();
    present_enc[5] = settings.value("q6").toInt();

    return exist;

//    restoreGeometry(settings.value("geometry").toByteArray());
//    restoreState(settings.value("windowState").toByteArray());

//    QString ip = settings.value("IP").toString();
//    QString port = settings.value("PORT").toString();
//    int num_joint = settings.value("NUM_JOINT").toInt();
//    int num_dof = settings.value("NUM_DOF").toInt();
//    int module_type = settings.value("MODULE_TYPE").toInt();
//    int joint_mode = settings.value("JOINT_MODE").toInt();

//    static_cast<Ui::MainWindow*>(ui)->txtIP->setText(ip);
//    static_cast<Ui::MainWindow*>(ui)->txtPORT->setText(port);
//    static_cast<Ui::MainWindow*>(ui)->cbNumJoint->setCurrentIndex(num_joint);
//    static_cast<Ui::MainWindow*>(ui)->cbNumDOF->setCurrentIndex(num_dof);
//    static_cast<Ui::MainWindow*>(ui)->cbModuleType->setCurrentIndex(module_type);
//    static_cast<Ui::MainWindow*>(ui)->cbJointMode->setCurrentIndex(joint_mode);
}

QString CustomSettings::configFile()
{
    QString filePath = qApp->applicationDirPath() + "/config.ini";
    return filePath;
}

