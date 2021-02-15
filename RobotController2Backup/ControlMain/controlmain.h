#ifndef CONTROLMAIN_H
#define CONTROLMAIN_H

#include <stdio.h>
#include <signal.h>
#include <unistd.h>
#include <sys/mman.h>

#include <native/task.h>
#include <native/timer.h>
#include <rtdk.h>

#include "NRMKhw_tp.h"	//NRMK gpio library

#include <pthread.h>

#include "DataControl/datacontrol.h"
#include "TcpServer/tcpserver.h"
#include "CustomFunc/controlmain_custom.h"
#include "Dynamixel/dynamixel.h"
#include "RobotArm/robotarm.h"

class ControlMain
{
public:
    ControlMain();
    ~ControlMain();
    void start();
    static void rtGpioFunc(void *arg);
    static void rtKeyFunc(void *arg);
    static void* init_func(void* arg);
    static void rtRobotFunc(void* arg);

    DataControl *dataControl;
    ControlMainCustom *controlMainCustom;
    TcpServer *tcpServerClient;
    TcpServer *tcpServerLatte;

private:
    RobotArm *robotArm;
    DxlControl *module;

    RT_TASK rtGpioTask, rtKeyTask, rtRobotTask;
    uint8_t gpio0_state, gpio1_state;
    TP_PORT gpio0, gpio1;
    pthread_t init_thread;

    void startGpioTask();
    void startTcpTask();
    void startKeyTask();
    void startRobotTask();
    void moduleInitFAR();
};

inline static int getch()
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

#endif // CONTROLMAIN_H
