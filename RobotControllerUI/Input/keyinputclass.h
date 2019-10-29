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
//#include "keyinputkeypadclass.h"

class KeyInputClass
{
public:
    KeyInputClass(void* _ui, void* _torque_ide);
    void InputKeyboard(QKeyEvent *keyevt);

private:

    KeyinputNoModifiedClass* keynone;
//    KeyinputAltClass* keyalt;
//    KeyinputControlClass* keyctrl;
//    KeyinputShiftClass* keyshift;
//    KeyinputShiftControlClass* keyshiftctrl;
//    KeyinputShiftAltClass* keyshiftalt;
//    KeyinputCtrlAltClass* keyctrlalt;
//    KeyinputKeypadClass* keypad;

    void* ui;
    void* torque_ide;
};

#endif // KEYINPUTCLASS_H
