#include "keyinput.h"

static struct termios oldSettings;
static struct termios newSettings;

/* Initialize new terminal i/o settings */
void initTermios(int echo){
    tcgetattr(0, &oldSettings); /* grab old terminal i/o settings */
    newSettings = oldSettings; /* make new settings same as old settings */
    newSettings.c_lflag &= static_cast<uint>(~ICANON); /* disable buffered i/o */
    newSettings.c_lflag &= echo ? ECHO : static_cast<uint>(~ECHO); /* set echo mode */
    tcsetattr(0, TCSANOW, &newSettings); /* use these new terminal i/o settings now */
}

/* Restore old terminal i/o settings */
void resetTermios(void){
    tcsetattr(0, TCSANOW, &oldSettings);
}

/* Read 1 character without echo */
char getch(void){
    return static_cast<char>(getchar());
}

KeyInput::KeyInput()
{
    initTermios(0);
}

KeyInput::~KeyInput(){
    resetTermios();
}

void KeyInput::run(){
    forever{
        char key = getch();
        emit KeyPressed(key);
    }
}

