#ifndef KEYINPUTSHIFTCONTROLCLASS_H
#define KEYINPUTSHIFTCONTROLCLASS_H
#include <QKeyEvent>

class KeyinputShiftControlClass
{
public:
    KeyinputShiftControlClass(void* _ui);
    void FuncKeyInput(QKeyEvent* keys);
    void NumberKeyInput(QKeyEvent* keys);
    void FxKeyInput(QKeyEvent* keys);
    void AlphabetKeyInput(QKeyEvent* keys);
    void UserCustomKeyInput(QKeyEvent* keys);

private:
    void* ui;
};

#endif // KEYINPUTSHIFTCONTROLCLASS_H
