#ifndef KEYINPUTSHIFTALTCLASS_H
#define KEYINPUTSHIFTALTCLASS_H
#include <QKeyEvent>

class KeyinputShiftAltClass
{
public:
    KeyinputShiftAltClass(void* _ui);
    void FuncKeyInput(QKeyEvent* keys);
    void NumberKeyInput(QKeyEvent* keys);
    void FxKeyInput(QKeyEvent* keys);
    void AlphabetKeyInput(QKeyEvent* keys);
    void UserCustomKeyInput(QKeyEvent* keys);

private:
    void* ui;
};

#endif // KEYINPUTSHIFTALTCLASS_H
