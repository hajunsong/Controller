#ifndef KEYINPUTSHIFTCLASS_H
#define KEYINPUTSHIFTCLASS_H
#include <QKeyEvent>

class KeyinputShiftClass
{
public:
    KeyinputShiftClass(void* _ui);
    void FuncKeyInput(QKeyEvent* keys);
    void NumberKeyInput(QKeyEvent* keys);
    void FxKeyInput(QKeyEvent* keys);
    void AlphabetKeyInput(QKeyEvent* keys);
    void UserCustomKeyInput(QKeyEvent* keys);

private:
    void* ui;
};

#endif // KEYINPUTSHIFTCLASS_H
