#include "keyinputnomodifiedclass.h"
#include "MainWindow/mainwindow.h"
#include "ui_mainwindow.h"
#include "MainWindow/mainwindow.h"
#include "TorqueID/torqueid.h"
#include "OperateUI/operateui.h"

KeyinputNoModifiedClass::KeyinputNoModifiedClass(void *_ui, void* _ui_op, void* _torque_id)
{
    ui = static_cast<Ui::MainWindow*>(_ui);
    ui_op = static_cast<OperateUI*>(_ui_op);
    torque_id = static_cast<TorqueID*>(_torque_id);
}

void KeyinputNoModifiedClass::FuncKeyInput(QKeyEvent* keys)
{
    switch(keys->key())
    {
        case Qt::Key_Escape:

            break;

        case Qt::Key_Space:

            break;

        case Qt::Key_PageDown:


            break;

        case Qt::Key_PageUp:


            break;

        case Qt::Key_Home:


            break;

        case Qt::Key_End:


            break;

        case Qt::Key_Insert:


            break;

        case Qt::Key_Delete:


            break;

        case Qt::Key_Back:


            break;

        case Qt::Key_Plus:


            break;

        case Qt::Key_Minus:


            break;

        case Qt::Key_division:


            break;

        case Qt::Key_multiply:


            break;

        case Qt::Key_Up:


            break;

        case Qt::Key_Down:


            break;

        case Qt::Key_Left:


            break;

        case Qt::Key_Right:


            break;
    }
}

void KeyinputNoModifiedClass::NumberKeyInput(QKeyEvent* keys)
{
    switch(keys->key())
    {

        case Qt::Key_1:
//            qDebug() << "Pressed 1";
            if (static_cast<Ui::MainWindow*>(ui)->tabWidget->count() == 3){
                static_cast<OperateUI*>(ui_op)->btnTeachingClicked();
            }

            break;

        case Qt::Key_2:
//            qDebug() << "Pressed 2";
            if (static_cast<Ui::MainWindow*>(ui)->tabWidget->count() == 3){
                static_cast<OperateUI*>(ui_op)->btnFeedingClicked();
            }

            break;

        case Qt::Key_3:


            break;

        case Qt::Key_4:


            break;

        case Qt::Key_5:


            break;

        case Qt::Key_6:


            break;

        case Qt::Key_7:


            break;

        case Qt::Key_8:


            break;

        case Qt::Key_9:


            break;

        case Qt::Key_0:


            break;


        default:
            break;
    }
}

void KeyinputNoModifiedClass::FxKeyInput(QKeyEvent* keys)
{
    switch(keys->key())
    {

        case Qt::Key_F1:


            break;

        case Qt::Key_F2:


            break;

        case Qt::Key_F3:


            break;

        case Qt::Key_F4:


            break;

        case Qt::Key_F5:

            break;

        case Qt::Key_F6:


            break;

        case Qt::Key_F7:
        {
            QString path = qApp->applicationDirPath();
            if (path.contains("keti") || path.contains("hajun")){
                if (static_cast<Ui::MainWindow*>(ui)->gbTrajectory->isEnabled()){
                    static_cast<Ui::MainWindow*>(ui)->tvPathData->model()->removeRows(0, static_cast<Ui::MainWindow*>(ui)->tvPathData->model()->rowCount());
                    static_cast<Ui::MainWindow*>(ui)->tvPathData->model()->insertRows(0,7);

                    double path[7*8] = {
                        0.0, -0.205112, 0.105523,  0.001374, 1.570796,  0.000000, -1.570796, 0.3,
                        1.0, -0.284822, 0.077250, -0.041363, 1.570796,  0.698131, -1.570796, 0.3,
                        2.0, -0.284822, 0.077250, -0.088270, 1.570796,  0.698131, -1.570796, 0.3,
                        3.0, -0.285133, 0.026167, -0.088270, 1.570796,  0.698131, -1.570796, 0.3,
                        4.0, -0.285133, 0.015167, -0.080270, 1.564223, -0.782312, -1.566155, 0.3,
                        5.0, -0.285133, 0.015167,  0.011729, 1.564223, -0.782312, -1.566155, 0.3,
                        6.0, -0.205422, 0.026833,  0.058549, 1.564223, -0.782312, -1.566155, 0.3
                    };

                    for(int i = 0; i < 7; i++){
                        for(int j = 0; j < 8; j++){
                            QModelIndex indx = static_cast<Ui::MainWindow*>(ui)->tvPathData->model()->index(i, j, QModelIndex());
                            static_cast<Ui::MainWindow*>(ui)->tvPathData->model()->setData(indx, QString::number(path[i*8 + j]));
                        }
                    }
                }
            }
        }
            break;

        case Qt::Key_F8:
        {
            QString path = qApp->applicationDirPath();
            if (path.contains("keti") || path.contains("hajun")){
//                if (static_cast<Ui::MainWindow*>(ui)->tabWidget->count() == 2){
//                    static_cast<Ui::MainWindow*>(ui)->tabWidget->addTab(static_cast<OperateUI*>(ui_op), "Operate");
                    static_cast<Ui::MainWindow*>(ui)->tabWidget->setCurrentIndex(2);
//                    static_cast<OperateUI*>(ui_op)->btnStartClciked();
//                }
//                else if(static_cast<Ui::MainWindow*>(ui)->tabWidget->count() == 3){
//                    static_cast<OperateUI*>(ui_op)->init();
//                    static_cast<Ui::MainWindow*>(ui)->tabWidget->removeTab(2);
//                    static_cast<Ui::MainWindow*>(ui)->tabWidget->setCurrentIndex(0);
//                }
            }
        }

            break;

        case Qt::Key_F9:
        {
            QString path = qApp->applicationDirPath();
            if (path.contains("keti") || path.contains("hajun")){
                static_cast<Ui::MainWindow*>(ui)->cbCartAbs->setChecked(true);
                double path[6] = {-0.208, 0.1750735,   0.07, 1.5707963, 0.0, -2.094399};
                static_cast<Ui::MainWindow*>(ui)->txtCCmd1->setText(QString::number(path[0]*1000));
                static_cast<Ui::MainWindow*>(ui)->txtCCmd2->setText(QString::number(path[1]*1000));
                static_cast<Ui::MainWindow*>(ui)->txtCCmd3->setText(QString::number(path[2]*1000));
                static_cast<Ui::MainWindow*>(ui)->txtCCmd4->setText(QString::number(path[3]*57.295779513));
                static_cast<Ui::MainWindow*>(ui)->txtCCmd5->setText(QString::number(path[4]*57.295779513));
                static_cast<Ui::MainWindow*>(ui)->txtCCmd6->setText(QString::number(path[5]*57.295779513));
                static_cast<Ui::MainWindow*>(ui)->txtCartesianMoveTime->setText("1.0");
                static_cast<Ui::MainWindow*>(ui)->txtCartesianMoveAccTime->setText("0.3");
            }
            break;
        }


        case Qt::Key_F10:
        {
            QString path = qApp->applicationDirPath();
            if (path.contains("keti") || path.contains("hajun")){
                if (static_cast<Ui::MainWindow*>(ui)->gbTrajectory->isEnabled()){
                    static_cast<Ui::MainWindow*>(ui)->tvPathData->model()->removeRows(0, static_cast<Ui::MainWindow*>(ui)->tvPathData->model()->rowCount());
                    static_cast<Ui::MainWindow*>(ui)->tvPathData->model()->insertRows(0,3);

                    double path[3*8] = {
                        0.0, -0.124, 0.2590735, 0.124, 1.5707963, 0.0, -2.094399, 0.3,
                        1.0, -0.292, 0.0910735, 0.254, 1.5707963, 0.0, -2.094399, 0.3,
                        2.0, -0.124, 0.2590735, 0.124, 1.5707963, 0.0, -2.094399, 0.3,
                    };

                    for(int i = 0; i < 3; i++){
                        for(int j = 0; j < 8; j++){
                            QModelIndex indx = static_cast<Ui::MainWindow*>(ui)->tvPathData->model()->index(i, j, QModelIndex());
                            static_cast<Ui::MainWindow*>(ui)->tvPathData->model()->setData(indx, QString::number(path[i*8 + j]));
                        }
                    }
                }
            }
            break;
        }

        case Qt::Key_F11:
        {
            QString path = qApp->applicationDirPath();
            if (path.contains("keti") || path.contains("hajun")){
                if (static_cast<Ui::MainWindow*>(ui)->gbTrajectory->isEnabled()){
                    static_cast<Ui::MainWindow*>(ui)->tvPathData->model()->removeRows(0, static_cast<Ui::MainWindow*>(ui)->tvPathData->model()->rowCount());
                    static_cast<Ui::MainWindow*>(ui)->tvPathData->model()->insertRows(0,6);

                    double DEG2RAD = 0.017453293;
                    double path[6*8] = {
                        0.0, -0.344364, 0.06765161, 0.1605722, -85.0291*DEG2RAD, 81.32067*DEG2RAD, 46.09656*DEG2RAD, 0.5,
                        2.0, -0.181359, 0.05066838, 0.07403771, -105.843*DEG2RAD, 83.58465*DEG2RAD, 61.37882*DEG2RAD, 0.3,
                        5.0, -0.31464, 0.1832746, -0.0115738, -93.0952*DEG2RAD, -2.23626*DEG2RAD, 87.29555*DEG2RAD, 0.3,
                        6.0, -0.339996, -0.11472, 0.09039965, -87.9638*DEG2RAD, 55.81264*DEG2RAD, 90.47109*DEG2RAD, 0.3,
                        8.0, -0.164176, -0.0287427, 0.06642572, -102.579*DEG2RAD, 35.9976*DEG2RAD, 56.58169*DEG2RAD, 0.5,
                        10.0, -0.344364, 0.06765161, 0.1605722, -85.0291*DEG2RAD, 81.32067*DEG2RAD, 46.09656*DEG2RAD, 0.5


                    };

                    for(int i = 0; i < 6; i++){
                        for(int j = 0; j < 8; j++){
                            QModelIndex indx = static_cast<Ui::MainWindow*>(ui)->tvPathData->model()->index(i, j, QModelIndex());
                            static_cast<Ui::MainWindow*>(ui)->tvPathData->model()->setData(indx, QString::number(path[i*8 + j]));
                        }
                    }
                }
            }
            break;
        }

        case Qt::Key_F12:
        {
            QString path = qApp->applicationDirPath();
            if (path.contains("keti") || path.contains("hajun")){
                if (static_cast<TorqueID*>(torque_id)->isHidden()){
                    static_cast<TorqueID*>(torque_id)->show();
                }
            }
            break;
        }

        default:
            break;
    }
}

void KeyinputNoModifiedClass::AlphabetKeyInput(QKeyEvent* keys)
{
    switch(keys->key())
    {

        case Qt::Key_A:
            qDebug() << "Pressed A";

            break;

        case Qt::Key_B:
            qDebug() << "Pressed B";

            break;

        case Qt::Key_C:


            break;

        case Qt::Key_D:


            break;

        case Qt::Key_E:


            break;

        case Qt::Key_F:


            break;

        case Qt::Key_G:


            break;

        case Qt::Key_H:


            break;

        case Qt::Key_I:


            break;

        case Qt::Key_J:


            break;

        case Qt::Key_K:


            break;

        case Qt::Key_L:


            break;

        default:
            break;
    }
}

void KeyinputNoModifiedClass::UserCustomKeyInput(QKeyEvent* keys)
{
    switch(keys->key())
    {
        case Qt::Key_Up:


            break;

        case Qt::Key_Down:


            break;

        case Qt::Key_Left:


            break;

        case Qt::Key_Right:


            break;

        case Qt::Key_A:


            break;

        case Qt::Key_S:


            break;

        case Qt::Key_D:


            break;

        case Qt::Key_W:


            break;

        case Qt::Key_Q:


            break;

        case Qt::Key_E:


            break;

        case Qt::Key_Z:


            break;

        case Qt::Key_X:


            break;

        case Qt::Key_C:


            break;

        case Qt::Key_Space:

            break;

        case Qt::Key_PageUp:


            break;

        case Qt::Key_PageDown:


            break;

        case Qt::Key_Home:


            break;

        case Qt::Key_End:


            break;

        case Qt::Key_Plus:


            break;

        case Qt::Key_Minus:


            break;

        case Qt::Key_0:


            break;

        default:
            break;
    }
}
