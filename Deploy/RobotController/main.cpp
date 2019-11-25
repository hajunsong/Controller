#include <QCoreApplication>
#include "Input/keyinput.h"
#include "ControlMain/controlmain.h"

static KeyInput *keyInput;
static ControlMain *controlMain;

void keyInputSlot(char key){
    if (key == 'q' || key == 27){
        printf("Pressed 'q' or 'ESC'\n");
        delete controlMain;
        usleep(1000000);
        delete keyInput;
        exit(0);
    }
}

int main(int argc, char** argv)
{
    QCoreApplication a(argc, argv);

    keyInput = new KeyInput();
    QObject::connect(keyInput, &KeyInput::KeyPressed, keyInputSlot);
    keyInput->start();

    controlMain = new ControlMain();
    controlMain->start();

    return a.exec();
}
