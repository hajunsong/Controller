#include "keyinputnomodifiedclass.h"
#include "MainWindow/mainwindow.h"
#include "ui_mainwindow.h"

KeyinputNoModifiedClass::KeyinputNoModifiedClass(void *_ui)
{
    ui = static_cast<Ui::MainWindow*>(_ui);
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


            break;

        case Qt::Key_F10:


            break;

        case Qt::Key_F11:


            break;

        case Qt::Key_F12:
        {
            QString path = qApp->applicationDirPath();
            if (path.contains("keti")){
                qDebug() << "Pressed F12";
                if (static_cast<Ui::MainWindow*>(ui)->gbTorqueIDE->isHidden()){
                    static_cast<Ui::MainWindow*>(ui)->gbTorqueIDE->setEnabled(true);
                    static_cast<Ui::MainWindow*>(ui)->gbTorqueIDE->setHidden(false);
                }
                else{
                    static_cast<Ui::MainWindow*>(ui)->gbTorqueIDE->setEnabled(false);
                    static_cast<Ui::MainWindow*>(ui)->gbTorqueIDE->setHidden(true);
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
