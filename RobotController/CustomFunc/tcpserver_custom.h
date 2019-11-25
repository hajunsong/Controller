#pragma once

#include <stdio.h>
#include <stdlib.h>

//-xenomai-///////////////////////////////////////////////////////////////
#include <native/task.h>
#include <native/timer.h>
#include <native/mutex.h>
#include <rtdk.h>		//The rdtk real-time printing library
/****************************************************************************/

class TcpServerCustom{
public:
    TcpServerCustom();
    ~TcpServerCustom();

    static void comm_run[ [noreturn] ](void *arg);
    void comm_stop();
    RT_TASK comm_task;
};
