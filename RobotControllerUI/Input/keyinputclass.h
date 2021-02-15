#ifndef KEYINPUTCLASS_H
#define KEYINPUTCLASS_H
#include <QKeyEvent>

//#include "keyinputaltclass.h"
//#include "keyinputcontrolclass.h"
#include "keyinputnomodifiedclass.h"
//#include "keyinputshiftclass.h"
//#include "keyinputshiftaltclass.h"
//#include "keyinputshiftcontrolclass.h"
//#include "keyinputctrlaltclass.h"
#include "keyinputkeypadclass.h"

#include <QtCore/qglobal.h>

#if defined(KEYINPUTLIB_LIBRARY)
#  define KETINPUTLIB_EXPORT Q_DECL_EXPORT
#else
#  define KETINPUTLIB_EXPORT Q_DECL_IMPORT
#endif

class KeyInputClass
{
public:
    KeyInputClass(void* _ui, void* _ui_op, void* _torque_id, void* _tcp_client);
    void InputKeyboard(QKeyEvent *keyevt);

private:

    KeyinputNoModifiedClass* keynone;
//    KeyinputAltClass* keyalt;
//    KeyinputControlClass* keyctrl;
//    KeyinputShiftClass* keyshift;
//    KeyinputShiftControlClass* keyshiftctrl;
//    KeyinputShiftAltClass* keyshiftalt;
//    KeyinputCtrlAltClass* keyctrlalt;
    KeyinputKeypadClass* keypad;

    void* ui;
    void* ui_op;
    void* torque_id;
    void* tcpClient;
};

#endif // KEYINPUTCLASS_H
