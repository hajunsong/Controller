#include "keyinputkeypadclass.h"
#include "MainWindow/mainwindow.h"
#include "ui_mainwindow.h"

KeyinputKeypadClass::KeyinputKeypadClass(void* _ui)
{
    ui = _ui;
}

void KeyinputKeypadClass::KeypadKeyInput(QKeyEvent* keyevt)
{
    int k = keyevt->key();
    switch(k)
    {
        case Qt::Key_0:

            break;

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

        case Qt::Key_Plus:

            break;

        case Qt::Key_Minus:

            break;

        case Qt::Key_division:

            break;

        case Qt::Key_multiply:

            break;

        case Qt::Key_Enter:

            break;

        case Qt::Key_NumLock:

            break;

        case Qt::Key_Insert:

            break;

        case Qt::Key_Delete:

            break;

        case Qt::Key_End:

            break;

        case Qt::Key_Down:

            break;

        case Qt::Key_PageDown:

            break;

        case Qt::Key_Left:

            break;

        case Qt::Key_Right:

            break;

        case Qt::Key_Home:

            break;

        case Qt::Key_PageUp:

            break;

        default:

            break;
    }
}
