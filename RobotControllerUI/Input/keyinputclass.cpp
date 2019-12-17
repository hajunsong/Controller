#include "Input/keyinputclass.h"
#include "MainWindow/mainwindow.h"
#include "ui_mainwindow.h"
#include "TcpSocket/tcpclient.h"
#include <qstandarditemmodel.h>
#include <qtableview.h>

KeyInputClass::KeyInputClass(void* _ui, void* _tcp)
{
    ui = static_cast<Ui::MainWindow*>(_ui);
	tcp = static_cast<TcpClient*>(_tcp);

	keynone = new KeyinputNoModifiedClass(ui, tcp);
//    keyalt = new KeyinputAltClass(ui);
//    keyctrl = new KeyinputControlClass(ui);
//    keyshift = new KeyinputShiftClass(ui);
//    keyshiftctrl = new KeyinputShiftControlClass(ui);
//    keyshiftalt = new KeyinputShiftAltClass(ui);
//    keyctrlalt = new KeyinputCtrlAltClass(ui);
//    keypad = new KeyinputKeypadClass(ui);

}

void KeyInputClass::InputKeyboard(QKeyEvent* keyevt)
{
    switch(keyevt->modifiers() & Qt::KeyboardModifierMask)
    {
        case Qt::NoModifier:
            keynone->AlphabetKeyInput(keyevt);
            keynone->FuncKeyInput(keyevt);
            keynone->FxKeyInput(keyevt);
            keynone->NumberKeyInput(keyevt);
            keynone->UserCustomKeyInput(keyevt);

            break;

//        case Qt::ShiftModifier:
//            keyshift->AlphabetKeyInput(keyevt);
//            keyshift->FuncKeyInput(keyevt);
//            keyshift->FxKeyInput(keyevt);
//            keyshift->NumberKeyInput(keyevt);
//            keyshift->UserCustomKeyInput(keyevt);
//            break;

//        case Qt::ControlModifier:
//            keyctrl->AlphabetKeyInput(keyevt);
//            keyctrl->FuncKeyInput(keyevt);
//            keyctrl->FxKeyInput(keyevt);
//            keyctrl->NumberKeyInput(keyevt);
//            keyctrl->UserCustomKeyInput(keyevt);

//            break;

//        case Qt::AltModifier:
//            keyalt->AlphabetKeyInput(keyevt);
//            keyalt->FuncKeyInput(keyevt);
//            keyalt->FxKeyInput(keyevt);
//            keyalt->NumberKeyInput(keyevt);
//            keyalt->UserCustomKeyInput(keyevt);

//            break;

//        case (Qt::ControlModifier + Qt::ShiftModifier):
//            keyshiftctrl->AlphabetKeyInput(keyevt);
//            keyshiftctrl->FuncKeyInput(keyevt);
//            keyshiftctrl->FxKeyInput(keyevt);
//            keyshiftctrl->NumberKeyInput(keyevt);
//            keyshiftctrl->UserCustomKeyInput(keyevt);

//            break;

//        case (Qt::AltModifier + Qt::ShiftModifier):
//            keyshiftalt->AlphabetKeyInput(keyevt);
//            keyshiftalt->FuncKeyInput(keyevt);
//            keyshiftalt->FxKeyInput(keyevt);
//            keyshiftalt->NumberKeyInput(keyevt);
//            keyshiftalt->UserCustomKeyInput(keyevt);

//            break;

//        case (Qt::ControlModifier + Qt::AltModifier):
//            keyctrlalt->AlphabetKeyInput(keyevt);
//            keyctrlalt->FuncKeyInput(keyevt);
//            keyctrlalt->FxKeyInput(keyevt);
//            keyctrlalt->NumberKeyInput(keyevt);
//            keyctrlalt->UserCustomKeyInput(keyevt);

//            break;

//        case Qt::KeypadModifier:
//            keypad->KeypadKeyInput(keyevt);

//            break;
    }

}


