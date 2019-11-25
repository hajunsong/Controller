#ifndef KEYINPUT_H
#define KEYINPUT_H

#include <QtCore/qglobal.h>

#if defined(KEYINPUTLIB_LIBRARY)
#  define KEYINPUTLIB_EXPORT Q_DECL_EXPORT
#else
#  define KEYINPUTLIB_EXPORT Q_DECL_IMPORT
#endif

#include <QThread>

#include <stdio.h>
#include <unistd.h>
#include <termios.h>

class KEYINPUTLIB_EXPORT KeyInput : QThread
{
    Q_OBJECT
public:
    KeyInput();
    ~KeyInput();
    void run();

signals:
    void KeyPressed(char ch);
};

#endif // KEYINPUT_H
