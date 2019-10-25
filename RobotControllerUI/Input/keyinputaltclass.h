#ifndef KEYINPUTALTCLASS_H
#define KEYINPUTALTCLASS_H
#include <QKeyEvent>

class KeyinputAltClass
{
public:
    KeyinputAltClass(void* _ui);
    void FuncKeyInput(QKeyEvent* keys);
    void NumberKeyInput(QKeyEvent* keys);
    void FxKeyInput(QKeyEvent* keys);
    void AlphabetKeyInput(QKeyEvent* keys);
    void UserCustomKeyInput(QKeyEvent* keys);

private:
    void* ui;
};

#endif // KEYINPUTALTCLASS_H
