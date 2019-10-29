#ifndef KEYINPUTNOMODIFIEDCLASS_H
#define KEYINPUTNOMODIFIEDCLASS_H
#include <QKeyEvent>


class KeyinputNoModifiedClass
{
public:
    KeyinputNoModifiedClass(void* _ui, void* torque_ide);
    void FuncKeyInput(QKeyEvent* keys);
    void NumberKeyInput(QKeyEvent* keys);
    void FxKeyInput(QKeyEvent* keys);
    void AlphabetKeyInput(QKeyEvent* keys);
    void UserCustomKeyInput(QKeyEvent* keys);

private:
    void* ui;
    void* torque_ide;
};

#endif // KEYINPUTNOMODIFIEDCLASS_H
