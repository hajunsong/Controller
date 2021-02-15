#include <QCoreApplication>
#include "ControlMain/controlmain.h"
#include <pthread.h>
#include <thread>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <iostream>

using namespace std;

#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <unistd.h>
#include <sys/mman.h>
#include <sys/io.h>

#include <native/task.h>
#include <native/timer.h>
#include <native/mutex.h>
#include <rtdk.h>

#include "NRMKhw_tp.h"	//NRMK gpio library

RT_TASK gpio_out_task, key_input_task;
uint8_t gpio0_state = 0, gpio1_state = 0, gpio0_state_old = 0, gpio1_state_old = 0;
TP_PORT gpio0 = TP_PORT::port0;
TP_PORT gpio1 = TP_PORT::port1;

static char key_value = 0;
static bool gpio_task_run = false;

#define USE_KEY_HOOK 0
#define USE_EXT_SW 1

#if USE_KEY_HOOK

#include "X11/Xlib.h"
#include "X11/Xutil.h"

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

//const char KEY_Q = 113;
//const char KEY_Z = 122;
//const char KEY_X = 120;
//const char KEY_C = 99;
//const char KEY_A = 97;
//const char KEY_S = 115;
//const char KEY_D = 100;
//const char KEY_ESC = 27;
//const char KEY_SPC = 32;
#endif

static ControlMain *controlMain;

void keyinput_func(void* arg){
    ControlMain *pControlMain = static_cast<ControlMain*>(arg);
#if !USE_KEY_HOOK

    rt_task_set_periodic(NULL, TM_NOW, 1e6);	//cycle: 50us
    bool run = true;
    pControlMain->dataControl->KITECHData.camera_request = false;
    while(run){
        rt_task_wait_period(NULL);
        key_value = getch();
        rt_printf("key value : %d\n", key_value);

        if (key_value == KEY_Q || key_value == KEY_ESC){
            gpio_task_run = false;
            run = false;
            break;
        }
        else if(key_value == KEY_A){
//            pControlMain->flag_on(key_value);
//            key_value = 0;
            pControlMain->dataControl->KITECHData.camera_request = true;
        }
        else if(key_value == KEY_S){
            pControlMain->dataControl->KITECHData.camera_request = false;
        }
//        else{
//            pControlMain->putObiMode(key_value);
//        }
    }
#else

    Display*    display = XOpenDisplay(0);
    Window      root    = DefaultRootWindow(display);
    XEvent      event;

    int keycode_a = XKeysymToKeycode(display,'a');
    GrabKey(display,root,keycode_a);
    int keycode_s = XKeysymToKeycode(display,'s');
    GrabKey(display,root,keycode_s);
    //    int keycode_z = XKeysymToKeycode(display,'z');
    //    GrabKey(display,root,keycode_z);
    //    int keycode_x = XKeysymToKeycode(display,'x');
    //    GrabKey(display,root,keycode_x);
    int keycode_q = XKeysymToKeycode(display,'q');
    GrabKey(display,root,keycode_q);
    //    int keycode_d = XKeysymToKeycode(display,'d');
    //    GrabKey(display,root,keycode_d);
    //    int keycode_c = XKeysymToKeycode(display,'c');
    //    GrabKey(display,root,keycode_c);

    XSelectInput(display, root, KeyPressMask | KeyReleaseMask);

    rt_task_set_periodic(NULL, TM_NOW, 500e6);	//cycle: 50us

    bool run = true;
    while(run)
    {
        rt_task_wait_period(NULL);

        XNextEvent(display, &event);
        switch(event.type)
        {
            case KeyPress:
                if(event.xkey.keycode == 24){ // q
                    run = false;
                    gpio_task_run = false;
                    break;
                }
                //                else if(event.xkey.keycode == 38){
                //                    key_value = KEY_A;
                //                }
                //                else if(event.xkey.keycode == 39){
                //                    key_value = KEY_S;
                //                }
                //                else if(event.xkey.keycode == 52){ // z
                //                    if(pControlMain->dataControl->obi_mode){
                //                        pControlMain->unsetObiMode();
                //                    }
                //                    else{
                //                        pControlMain->setObiMode();
                //                    }
                //                }
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

#if USE_EXT_SW
//output task
void gpio_out_run(void *arg)
{
    ControlMain *pControlMain = static_cast<ControlMain*>(arg);

    rt_task_set_periodic(NULL, TM_NOW, 100e6);	//cycle: 50us

    key_value = 0;
    gpio0_state_old = 0;
    gpio1_state_old = 0;

    tp_writeport(gpio0, 0x00);
    tp_writeport(gpio1, 0x01);

    int step = 0;
    unsigned int count = 0;
    bool led = true;

    gpio_task_run = true;
    pControlMain->dataControl->KITECHData.tablet_connect = false;

    unsigned int led_onoff_count = 5;

    while (gpio_task_run)
    {
        rt_task_wait_period(NULL);

        gpio0_state = tp_readport(gpio0);
        gpio1_state = tp_readport(gpio1);

        count++;
        if(count < led_onoff_count){
            led = true;
        }
        else if(count >= led_onoff_count && count < led_onoff_count*2){
            led = false;
        }
        else{
            count = 0;
        }

        if((gpio0_state) != (gpio0_state_old)){
            rt_printf("GPIO 0 : \t");
            rt_printf((gpio0_state & pin07) == pin07 ? "1\t" : "0\t");
            rt_printf((gpio0_state & pin06) == pin06 ? "1\t" : "0\t");
            rt_printf((gpio0_state & pin05) == pin05 ? "1\t" : "0\t");
            rt_printf((gpio0_state & pin04) == pin04 ? "1\t" : "0\t");
            rt_printf((gpio0_state & pin03) == pin03 ? "1\t" : "0\t");
            rt_printf((gpio0_state & pin02) == pin02 ? "1\t" : "0\t");
            rt_printf((gpio0_state & pin01) == pin01 ? "1\t" : "0\t");
            rt_printf((gpio0_state & pin00) == pin00 ? "1\t" : "0\t");
            rt_printf("\n");
            rt_printf("GPIO 1 : \t");
            rt_printf((gpio1_state & pin07) == pin07 ? "1\t" : "0\t");
            rt_printf((gpio1_state & pin06) == pin06 ? "1\t" : "0\t");
            rt_printf((gpio1_state & pin05) == pin05 ? "1\t" : "0\t");
            rt_printf((gpio1_state & pin04) == pin04 ? "1\t" : "0\t");
            rt_printf((gpio1_state & pin03) == pin03 ? "1\t" : "0\t");
            rt_printf((gpio1_state & pin02) == pin02 ? "1\t" : "0\t");
            rt_printf((gpio1_state & pin01) == pin01 ? "1\t" : "0\t");
            rt_printf((gpio1_state & pin00) == pin00 ? "1\t" : "0\t");
            rt_printf("\n");
            rt_printf("\n");

            if((~gpio0_state & pin00) == pin00)
            {
                key_value = KEY_Z;
                gpio1_state |= pin01;
            }
            else if((~gpio0_state & pin01) == pin01){
                key_value = KEY_X;
                gpio1_state |= pin02;
            }
            else if((~gpio0_state & pin02) == pin02){
                key_value = KEY_C;
                gpio1_state |= pin03;
            }
            //            else{
            //                gpio1_state &= ~(pin00|pin01|pin02);
            //            }
            tp_writeport(gpio1, gpio1_state);
            usleep(5000);

            rt_printf("key value : %d\n", key_value);

            if(key_value == KEY_Z){
                gpio0_state &= ~(pin03|pin04|pin05);
                tp_writeport(gpio0, gpio0_state);
                usleep(5000);

                if(pControlMain->dataControl->obi_mode || pControlMain->dataControl->demo_mode){
                    pControlMain->unsetObiMode();
                    step = 0;
                }
                else{
                    pControlMain->setObiMode();
                    gpio0_state |= pin03;
                    tp_writeport(gpio0, gpio0_state);
                    step = 1;
                }
            }

            else if(key_value == KEY_X){
                if(pControlMain->dataControl->operateMode.mode == DataControl::Operate::Start){
                    pControlMain->dataControl->ClientToServer.opMode = DataControl::OpMode::OperateMode;
                    pControlMain->dataControl->operateMode.mode = DataControl::Operate::StartTeaching;
                    gpio0_state |= pin04;
                    tp_writeport(gpio0, gpio0_state);
                    usleep(5000);
                    step = 2;
                }
                else if(pControlMain->dataControl->operateMode.mode == DataControl::Operate::StartTeaching){
                    pControlMain->dataControl->ClientToServer.opMode = DataControl::OpMode::OperateMode;
                    pControlMain->dataControl->operateMode.mode = DataControl::Operate::StopTeaching;
                    usleep(10000);
                    while(!(pControlMain->dataControl->ClientToServer.opMode == DataControl::OpMode::Wait)){
                        usleep(10000);
                    }
                    gpio0_state &= ~pin04;
                    tp_writeport(gpio0, gpio0_state);
                    step = 3;
                }
            }
            else if(key_value == KEY_C){
                if(pControlMain->dataControl->operateMode.mode == DataControl::Operate::StopTeaching){
                    pControlMain->dataControl->ClientToServer.opMode = DataControl::OpMode::OperateMode;
                    pControlMain->dataControl->operateMode.mode = DataControl::Operate::StartFeeding;

                    gpio0_state &= ~(pin03|pin04|pin05);
                    tp_writeport(gpio0, gpio0_state);
                    usleep(5000);

                    gpio0_state |= (pin04|pin05);
                    tp_writeport(gpio0, gpio0_state);
                    usleep(5000);
                    step = 4;
                }
            }
            else{
                pControlMain->putObiMode(key_value);
            }
        }

        gpio1_state &= ~(pin01|pin02|pin03);
        tp_writeport(gpio1, gpio1_state);
        switch(step){
            case 0:
                gpio1_state = led ? gpio1_state|pin01 : gpio1_state & ~(pin01|pin02|pin03);
                tp_writeport(gpio1, gpio1_state);
                break;
            case 1:
                gpio1_state = led ? gpio1_state|pin02 : gpio1_state & ~(pin01|pin02|pin03);
                tp_writeport(gpio1, gpio1_state);
                break;
            case 2:
                //                    gpio1_state = led ? gpio1_state|pin02 : gpio1_state & ~(pin01|pin02|pin03);
                //                    tp_writeport(gpio1, gpio1_state);
                gpio1_state |= pin02;
                tp_writeport(gpio1, gpio1_state);
                break;
            case 3:
                gpio1_state = led ? gpio1_state|pin03 : gpio1_state & ~(pin01|pin02|pin03);
                tp_writeport(gpio1, gpio1_state);
                break;
            case 4:
//                if(!pControlMain->tcpServerLatte->isConnected()){
                pControlMain->dataControl->KITECHData.tablet_check = true;
                if(!pControlMain->dataControl->KITECHData.tablet_connect){
                    gpio0_state = led ? gpio0_state|0x30 : gpio0_state&(0xCF);
                    tp_writeport(gpio0, gpio0_state);
                }
                else{
                    gpio0_state |= 0x30;
                    tp_writeport(gpio0, gpio0_state);
                    pControlMain->dataControl->KITECHData.tablet_check = false;
                }
                break;
            default:
                break;
        }

        key_value = 0;
        gpio0_state_old = gpio0_state;
        gpio1_state_old = gpio1_state;

        //            gpio1_state &= ~(pin01|pin02|pin03);
        //            tp_writeport(gpio1, gpio1_state);
        //        }
    }

}
#endif

struct sigaction sigIntHandler;
static int sig = 0;

void my_handler(int s){
    if(sig == 0){
        sig = s;

        rt_task_delete(&gpio_out_task);
//        rt_task_delete(&key_input_task);

        printf("gpio task finished\n");
        tp_writeport(gpio0, 0x00);
        tp_writeport(gpio1, 0x00);

        delete controlMain;
        printf("Finished FAR Program\n");
        exit(1);
    }
}

int main()
{
    sigIntHandler.sa_handler = my_handler;
    sigemptyset(&sigIntHandler.sa_mask);
    sigIntHandler.sa_flags = 0;

    printf("Start FAR Program\n");
    sigaction(SIGINT, &sigIntHandler, NULL);

    controlMain = new ControlMain();

    controlMain->start();

    mlockall(MCL_CURRENT|MCL_FUTURE);

    /* Perform auto-init of rt_print buffers if the task doesn't do so */
    rt_print_auto_init(1);

    if (tp_gpio_init()!=0)	//initialize port, must be called before used
    {
        printf("GPIO Init. Error!\n");
        return 1;
    }

    tp_gpio_set_dir(gpio0, 0x38);
    tp_gpio_set_dir(gpio1, 0x0F);

    printf("Running, check the GPIO state....\n");

    //create and start a realtime periodic task for writing gpio
    rt_task_create(&gpio_out_task, "gpio_out", 0, 99, 0);
    rt_task_start(&gpio_out_task, &gpio_out_run, controlMain);

//    rt_task_create(&key_input_task, "key_input", 0, 98, 0);
//    rt_task_start(&key_input_task, &keyinput_func, controlMain);

    pause();

    return 0;
}
