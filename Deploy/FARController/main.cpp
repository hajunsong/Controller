#include <QCoreApplication>
#include "Input/keyinput.h"
#include "ControlMain/controlmain.h"
#include "Settings/customsettings.h"

static KeyInput *keyInput;
static ControlMain *controlMain;
static CustomSettings customSettings;

void keyInputSlot(char key){
    if (key == 'q' || key == 27){
        printf("Pressed 'q' or 'ESC'\n");
        delete controlMain;
        usleep(1000000);
        delete keyInput;
        exit(0);
    }
    else if(key == 's'){
        printf("Save present enc pulse\n");
        int32_t enc[NUM_JOINT] = {0,};
        controlMain->getPresentEnc(enc);
        customSettings.saveConfigFile(enc);
    }
}

int main(int argc, char** argv)
{
    QCoreApplication a(argc, argv);

    keyInput = new KeyInput();
    QObject::connect(keyInput, &KeyInput::KeyPressed, keyInputSlot);
    keyInput->start();

    controlMain = new ControlMain();

    int32_t enc[NUM_JOINT] = {0,};
    bool file_exist = false;
    file_exist = customSettings.loadConfigFile(enc);
    if(file_exist){
        printf("Exist save file\n");
        controlMain->setOffsetEnc(enc);
    }
    controlMain->start();

    return a.exec();
}
