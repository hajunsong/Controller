#ifndef KEYINPUTKEYPADCLASS_H
#define KEYINPUTKEYPADCLASS_H
#include <QKeyEvent>

class KeyinputKeypadClass
{
public:
    KeyinputKeypadClass(void *_ui, void* _tcp_client);
    void KeypadKeyInput(QKeyEvent* keyevt);
private:
    void* ui;
    void* tcpClient;
};

#endif // KEYINPUTKEYPADCLASS_H
