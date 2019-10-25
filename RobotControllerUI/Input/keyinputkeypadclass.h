#ifndef KEYINPUTKEYPADCLASS_H
#define KEYINPUTKEYPADCLASS_H
#include <QKeyEvent>

class KeyinputKeypadClass
{
public:
    KeyinputKeypadClass(void *_ui);
    void KeypadKeyInput(QKeyEvent* keyevt);
private:
    void* ui;
};

#endif // KEYINPUTKEYPADCLASS_H
