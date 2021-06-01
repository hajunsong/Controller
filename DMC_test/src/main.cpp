#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <signal.h>
#include "userinterface.h"

struct sigaction sigIntHandler;
static int sig = 0;

void my_handler(int s)
{
    if (sig == 0)
    {
        sig = s;
        printf("Finished\n");
        exit(1);
    }
}

int main()
{
    sigIntHandler.sa_handler = my_handler;
    sigemptyset(&sigIntHandler.sa_mask);
    sigIntHandler.sa_flags = 0;

    sigaction(SIGINT, &sigIntHandler, NULL);

    printf("Start Program\n");

    userinterface w;
    w.start();

    pause();

    return 0;
}
