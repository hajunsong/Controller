#ifndef KEYINPUTNOMODIFIEDCLASS_H
#define KEYINPUTNOMODIFIEDCLASS_H
#include <QKeyEvent>
#include <QThread>

#include <QtCore/qglobal.h>

#if defined(KEYINPUTNOMODIFIEDLIB_LIBRARY)
#  define KEYINPUTNOMODIFIEDLIB_EXPORT Q_DECL_EXPORT
#else
#  define KEYINPUTNOMODIFIEDLIB_EXPORT Q_DECL_IMPORT
#endif

class KeyinputNoModifiedClass
{
public:
    KeyinputNoModifiedClass(void* _ui, void* _ui_op, void* _torque_id);
    void FuncKeyInput(QKeyEvent* keys);
    void NumberKeyInput(QKeyEvent* keys);
    void FxKeyInput(QKeyEvent* keys);
    void AlphabetKeyInput(QKeyEvent* keys);
    void UserCustomKeyInput(QKeyEvent* keys);

private:
    void* ui;
    void* ui_op;
    void* torque_id;
};

#endif // KEYINPUTNOMODIFIEDCLASS_H
