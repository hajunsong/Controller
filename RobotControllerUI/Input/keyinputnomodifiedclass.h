#ifndef KEYINPUTNOMODIFIEDCLASS_H
#define KEYINPUTNOMODIFIEDCLASS_H
#include <QKeyEvent>


class KeyinputNoModifiedClass
{
public:
	KeyinputNoModifiedClass(void* _ui, void* _tcp);
    void FuncKeyInput(QKeyEvent* keys);
    void NumberKeyInput(QKeyEvent* keys);
    void FxKeyInput(QKeyEvent* keys);
    void AlphabetKeyInput(QKeyEvent* keys);
    void UserCustomKeyInput(QKeyEvent* keys);

private:
    void* ui;
	void* tcp;
};

#endif // KEYINPUTNOMODIFIEDCLASS_H
