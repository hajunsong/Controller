#pragma once

#include <iostream>

#include "globalstruct.h"
#include "ecatslave.h"
#include "ecatmaster.h"

#include <pthread.h>

using namespace std;


class ControlMain
{
public:
    ControlMain();
    ~ControlMain();
    void start();

    ecatmaster *ecatMaster;

    pthread_t threadStateUpdate;
    static void* f_oper_stateUpdateTimer_Callback(void* arg);

private :
};
