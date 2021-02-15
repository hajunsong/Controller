#pragma once

#include <stdio.h>
#include <stdlib.h>

//-xenomai-///////////////////////////////////////////////////////////////
#include <native/task.h>
#include <native/timer.h>
#include <native/mutex.h>
#include <rtdk.h>		//The rdtk real-time printing library
/****************************************************************************/

class ControlMainCustom{
public:
    ControlMainCustom();
    ~ControlMainCustom();

    int robot_run(void *arg);
};

