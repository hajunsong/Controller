#ifndef KEYINPUTCTRLALTCLASS_H
#define KEYINPUTCTRLALTCLASS_H
#include <QKeyEvent>

class KeyinputCtrlAltClass
{
public:
    KeyinputCtrlAltClass(void* _ui);
    void FuncKeyInput(QKeyEvent* keys);
    void NumberKeyInput(QKeyEvent* keys);
    void FxKeyInput(QKeyEvent* keys);
    void AlphabetKeyInput(QKeyEvent* keys);
    void UserCustomKeyInput(QKeyEvent* keys);

private:
    void* ui;
};

#endif // KEYINPUTCTRLALTCLASS_H
