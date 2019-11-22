#ifndef KEYINPUTCLASS_H
#define KEYINPUTCLASS_H

#include <QThread>

#include <stdio.h>
#include <unistd.h>
#include <termios.h>

class KeyInput : public QThread
{
    Q_OBJECT
public:
    KeyInput();
    ~KeyInput();
    void run();

signals:
    void KeyPressed(char ch);
};

#endif // KEYINPUTCLASS_H
