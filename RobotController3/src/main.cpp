#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <signal.h>

#include "controlmain.h"

ControlMain *controlMain;
struct sigaction sigIntHandler;
static int sig = 0;

void my_handler(int s){
    if(sig == 0){
        sig = s;
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

    sigaction(SIGINT, &sigIntHandler, NULL);

    printf("Start FAR Program\n");

    controlMain = new ControlMain();
    controlMain->start();

    pause();

    return 0;
}
