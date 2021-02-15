#include "ControlMain/controlmain.h"
#include <pthread.h>

#include "X11/Xlib.h"
#include "X11/Xutil.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <iostream>

using namespace std;

#define USE_KEY_HOOK 0

#if USE_KEY_HOOK

void* TaskCode(void* parg)
{
    int keycode = *((int*)parg);
//    cout<< "\n\n" << keycode << "\n\n";
//    system("espeak -v en " "\"a\"");
    switch(keycode){
        case 24: // q
        {
            break;
        }
        case 38: // a
        {
            break;
        }
        case 39: // s
        {
            break;
        }
        case 52: // z
        {
            break;
        }
        case 53: // x
        {
            break;
        }
    }

    delete (int*)parg;
    return 0;
}

void SendKeyEvent(Display *display, XEvent event)
{
    Window current_focus_window;
    XKeyEvent& xkey = event.xkey;

    int current_focus_revert;
    XGetInputFocus(display, &current_focus_window, &current_focus_revert);
    xkey.state = Mod2Mask;

    XSendEvent(display, InputFocus,  True, xkey.type, (XEvent *)(&event));
}

int GrabKey(Display* display, Window grab_window, int keycode)
{
    unsigned int    modifiers       = Mod2Mask; // numlock on
    //Window          grab_window     = DefaultRootWindow(display);
    Bool            owner_events    = True;
    int             pointer_mode    = GrabModeAsync;
    int             keyboard_mode   = GrabModeAsync;

    XGrabKey(display, keycode, modifiers, grab_window, owner_events, pointer_mode, keyboard_mode);
    return keycode;
}

void UngrabKey(Display* display, Window grab_window, int keycode)
{
    unsigned int    modifiers       = Mod2Mask; // numlock on

   // Window grab_window = DefaultRootWindow(display);
    XUngrabKey(display,keycode,modifiers,grab_window);
}


void Action(int keycode)
{
    pthread_t thread;
    int* pthread_arg = new int;

    *pthread_arg = keycode;
    pthread_create(&thread,0, TaskCode, (void*) pthread_arg);
}
#else

static int getch()
{
#if defined(__linux__) || defined(__APPLE__)
  struct termios oldt, newt;
  int ch;
  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;
  newt.c_lflag &= ~(ICANON | ECHO);
  tcsetattr(STDIN_FILENO, TCSANOW, &newt);
  ch = getchar();
  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
  return ch;
#elif defined(_WIN32) || defined(_WIN64)
  return _getch();
#endif
}

const char KEY_Q = 113;
const char KEY_Z = 122;
const char KEY_ESC = 27;
const char KEY_SPC = 32;
#endif

static ControlMain *controlMain;

static void* keyinput_func(void* arg){
    ControlMain *pControlMain = static_cast<ControlMain*>(arg);
#if !USE_KEY_HOOK
    char key_value = 0;

    while(1){
        key_value = getch();
#if 0
        printf("key value : %d\n", key_value);
#else
        if (key_value == KEY_Q || key_value == KEY_ESC){
              break;
        }
        else if(key_value == KEY_Z){
            if(pControlMain->dataControl->obi_mode){
                pControlMain->unsetObiMode();
            }
            else{
                pControlMain->setObiMode();
            }
        }
        else{
            pControlMain->putObiMode(key_value);
        }
#endif
        usleep(1000);
    }

    return nullptr;
#else

    Display*    display = XOpenDisplay(0);
    Window      root    = DefaultRootWindow(display);
    XEvent      event;

    int keycode_a = XKeysymToKeycode(display,'a');
    GrabKey(display,root,keycode_a);
    int keycode_s = XKeysymToKeycode(display,'s');
    GrabKey(display,root,keycode_s);
    int keycode_z = XKeysymToKeycode(display,'z');
    GrabKey(display,root,keycode_z);
    int keycode_x = XKeysymToKeycode(display,'x');
    GrabKey(display,root,keycode_x);
    int keycode_q = XKeysymToKeycode(display,'q');
    GrabKey(display,root,keycode_q);

    XSelectInput(display, root, KeyPressMask | KeyReleaseMask);

    bool run = true;
    while(run)
    {
        XNextEvent(display, &event);
        switch(event.type)
        {
            case KeyPress:
//                Action(event.xkey.keycode);
//                cout<< "\n\n" << event.xkey.keycode << "\n\n";

                if(event.xkey.keycode == 24){ // q
                    run = false;
                }
                else if(event.xkey.keycode == 52){ // z
                    if(pControlMain->dataControl->obi_mode){
                        pControlMain->unsetObiMode();
                    }
                    else{
                        pControlMain->setObiMode();
                    }
                }
                else{
                    pControlMain->putObiMode(event.xkey.keycode);
                }
            case KeyRelease:
                SendKeyEvent(display,event);
            default:
                break;
        }
    }

    XCloseDisplay(display);
#endif
}

int main()
{
    controlMain = new ControlMain();

    pthread_t keyinput_thread;
    pthread_create(&keyinput_thread, nullptr, keyinput_func, controlMain);

    controlMain->start();

    pthread_join(keyinput_thread, nullptr);

    delete controlMain;

    return 0;
}
