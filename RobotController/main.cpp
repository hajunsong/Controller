#include "Dynamixel/XH_540_Series.h"

#include <stdio.h>
#include <signal.h>
#include <unistd.h>
#include <sys/mman.h>

#include <native/task.h>
#include <native/timer.h>
#include <rtdk.h>

RT_TASK demo_task, test_task;
DxlControl *module;

/* NOTE: error handling omitted. */

void demo(void *arg)
{
    RTIME now, previous, dxl_time1, dxl_time2;

    int32_t dxl_present_position[6] = {0,};

    DxlControl *module = static_cast<DxlControl*>(arg);

    module->init();

    module->setGroupSyncWriteTorqueEnable(1);

    // Write goal position
    module->setGoalPosition(150000, 5);

    rt_task_set_periodic(nullptr, TM_NOW, 5e6);
    previous = rt_timer_read();

    while (1)
    {
        rt_task_wait_period(nullptr);
        now = rt_timer_read();

        dxl_time1 = rt_timer_read();
        module->getGroupSyncReadPresentPosition(dxl_present_position);
        dxl_time2 = rt_timer_read();

        rt_printf("[ID:%03d] PresPos:%03d\n", 0, dxl_present_position[0]);
        rt_printf("[ID:%03d] PresPos:%03d\n", 1, dxl_present_position[1]);
        rt_printf("[ID:%03d] PresPos:%03d\n", 2, dxl_present_position[2]);
        rt_printf("[ID:%03d] PresPos:%03d\n", 3, dxl_present_position[3]);
        rt_printf("[ID:%03d] PresPos:%03d\n", 4, dxl_present_position[4]);
        rt_printf("[ID:%03d] PresPos:%03d\n", 5, dxl_present_position[5]);

        rt_printf("Time since last turn: %ld.%06ld ms\n",
                  (long)(now - previous) / 1000000,
                  (long)(now - previous) % 1000000);
        rt_printf("DXL Time since last turn: %ld.%06ld ms\n",
                  (long)(dxl_time2 - dxl_time1) / 1000000,
                  (long)(dxl_time2 - dxl_time1) % 1000000);
        previous = now;
    }
}

void test(void *arg)
{
    RTIME now, previous;

    rt_task_set_periodic(nullptr, TM_NOW, 1e6);
    previous = rt_timer_read();

    while (1)
    {
        rt_task_wait_period(nullptr);
        now = rt_timer_read();

        // rt_printf("test Time since last turn: %ld.%06ld ms\n",
        //           (long)(now - previous) / 1000000,
        //           (long)(now - previous) % 1000000);
        previous = now;
    }
}

void catch_signal(int sig)
{
    printf("catch signal : %d\n", sig);

    module->setGroupSyncWriteTorqueEnable(0);

    rt_task_suspend(&demo_task);
    rt_task_suspend(&test_task);
    usleep(1000e3);
    printf("Suspended rt task\n");

    delete module;
}

int main()
{
    rt_print_auto_init(1);
    printf("Program started\n");

    signal(SIGTERM, catch_signal);
    signal(SIGINT, catch_signal);

    /* Avoids memory swapping for this program */
    mlockall(MCL_CURRENT | MCL_FUTURE);

    module = new DxlControl();

    rt_task_create(&demo_task, "trivial", 0, 99, 0);
    rt_task_create(&test_task, "test", 0, 50, 0);

    rt_task_start(&demo_task, &demo, module);
    rt_task_start(&test_task, &test, nullptr);

    pause();

    rt_task_delete(&demo_task);
    rt_task_delete(&test_task);
    usleep(1000e3);
    printf("Deleted rt task\n");

    return 0;
}
