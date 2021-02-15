/* NRMKFoundation, Copyright 2016- Neuromeka. All rights reserved.
 *
 * This library is commercial and cannot be redistributed, and/or modified
 * WITHOUT ANY ALLOWANCE OR PERMISSION OF Neuromeka
 */

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

RT_TASK gpio_out_task;
uint8_t gpio0_state = 0, gpio1_state = 0;
TP_PORT gpio0 = TP_PORT::port0;
TP_PORT gpio1 = TP_PORT::port1;

//output task
void gpio_out_run(void *arg)
{
    bool temp = false;
    tp_writeport(gpio1, 0xFF);
    rt_task_set_periodic(NULL, TM_NOW, 500e6);	//cycle: 50us
    while (1)
    {
        rt_task_wait_period(NULL);
        gpio0_state = tp_readport(gpio0);
        gpio1_state = tp_readport(gpio1);

        temp ^= true;
        if(temp){
            tp_writeport(gpio0, 0xF8);
            tp_writeport(gpio1, 0xFF);
        }
        else{
            tp_writeport(gpio0, 0x00);
            tp_writeport(gpio1, 0x00);
        }

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
    }
}

void catch_signal(int sig)
{
    rt_task_delete(&gpio_out_task);
    exit(1);
}

int main()
{
//    usleep(1000000);
    signal(SIGTERM, catch_signal);
    signal(SIGINT, catch_signal);
    /* Avoids memory swapping for this program */
    mlockall(MCL_CURRENT|MCL_FUTURE);

    /* Perform auto-init of rt_print buffers if the task doesn't do so */
    rt_print_auto_init(1);

    if (tp_gpio_init()!=0)	//initialize port, must be called before used
    {
        printf("GPIO Init. Error!\n");
        return 1;
    }

    tp_gpio_set_dir(gpio0, 0xF8);
    tp_gpio_set_dir(gpio1, 0xFF);

    printf("Running, check the GPIO state....\n");

    //create and start a realtime periodic task for writing gpio
    rt_task_create(&gpio_out_task, "gpio_out", 0, 50, 0);
    rt_task_start(&gpio_out_task, &gpio_out_run, NULL);

    pause();

    rt_task_delete(&gpio_out_task);

    return 0;
}


