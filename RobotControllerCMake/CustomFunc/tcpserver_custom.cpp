#include "tcpserver_custom.h"
#include "TcpServer/tcpserver.h"

TcpServerCustom::TcpServerCustom(){
}

TcpServerCustom::~TcpServerCustom(){
    comm_stop();
}

void TcpServerCustom::comm_run(void *arg){
    RTIME now, previous;
    previous = rt_timer_read();
    TcpServer* pThis = static_cast<TcpServer*>(arg);

    rt_task_set_periodic(&pThis->tcpServerCustom->comm_task, TM_NOW, 50e6);

    pThis->comm_thread_run = true;

    int ret = 0;
    while(pThis->comm_thread_run){
        rt_task_wait_period(nullptr); //wait for next cycle
        if (pThis->isConnected()){
//            rt_printf("opMode : %d\n", pThis->dataControl->ClientToServer.opMode);
            if(pThis->dataControl->ClientToServer.opMode >= 2){
                ret = pThis->sendData();
            }

            now = rt_timer_read();

//            rt_printf("Server Comm Time : %ld.%06ld\n", (long)(now - previous) / 1000000, (long)(now - previous) % 1000000);

            previous = now;

            if(ret == -1){
                pThis->comm_thread_run = false;
                break;
            }
        }
        else{
            rt_printf("Disconnected comm\n");
        }
    }
}

void TcpServerCustom::comm_stop(){
    rt_task_suspend(&comm_task);
    printf("Tcp Comm RT Task Stop\n");
    rt_task_delete(&comm_task);
    printf("Tcp Comm RT Task Delete\n");
}
