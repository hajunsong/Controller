#ifndef KEYINPUTCONTROLCLASS_H
#define KEYINPUTCONTROLCLASS_H
#include <QKeyEvent>

class KeyinputControlClass
{
public:
    KeyinputControlClass(void* _ui);
    void FuncKeyInput(QKeyEvent* keys);
    void NumberKeyInput(QKeyEvent* keys);
    void FxKeyInput(QKeyEvent* keys);
    void AlphabetKeyInput(QKeyEvent* keys);
    void UserCustomKeyInput(QKeyEvent* keys);

private:
    void* ui;
};

#endif // KEYINPUTCONTROLCLASS_H
