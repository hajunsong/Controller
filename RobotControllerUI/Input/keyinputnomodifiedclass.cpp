#include "keyinputnomodifiedclass.h"
#include "MainWindow/mainwindow.h"
#include "ui_mainwindow.h"
#include "MainWindow/mainwindow.h"
#include "TorqueID/torqueid.h"

KeyinputNoModifiedClass::KeyinputNoModifiedClass(void *_ui, void* _tcp)
{
    ui = static_cast<Ui::MainWindow*>(_ui);
	tcp = static_cast<TcpClient*>(_tcp);
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


            break;

        case Qt::Key_2:


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

            break;

        case Qt::Key_F8:


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
                        0.0, -0.124, 0.2590735, -0.014, 1.5707963, 0.0, -2.094399, 0.3,
                        1.0, -0.292, 0.0910735,  0.154, 1.5707963, 0.0, -2.094399, 0.3,
                        2.0, -0.124, 0.2590735, -0.014, 1.5707963, 0.0, -2.094399, 0.3,
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

                    double path[6*8] = {
                        0.0, -0.208, 0.1750735,   0.07, 1.5707963, 0.0, -2.094399, 0.3,
                        1.0, -0.124, 0.2590735, -0.014, 1.5707963, 0.0, -2.094399, 0.3,
                        2.0, -0.292, 0.2590735, -0.014, 1.5707963, 0.0, -2.094399, 0.3,
                        3.0, -0.292, 0.0910735,  0.154, 1.5707963, 0.0, -2.094399, 0.3,
                        4.0, -0.124, 0.0910735,  0.154, 1.5707963, 0.0, -2.094399, 0.3,
                        5.0, -0.208, 0.1750735,   0.07, 1.5707963, 0.0, -2.094399, 0.3
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
				TorqueID *torqueID = new TorqueID(tcp);
				torqueID->show();
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


            break;

        case Qt::Key_B:


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
