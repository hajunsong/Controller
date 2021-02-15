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
    NRMKHelper::TcpServer* pTcpServer = static_cast<NRMKHelper::TcpServer*>(arg);
    pTcpServer->comm_thread_run = false;

    rt_task_set_periodic(&pTcpServer->tcpServerCustom->comm_task, TM_NOW, 100e6);

    pTcpServer->comm_thread_run = true;
    while(pTcpServer->comm_thread_run){
        if (pTcpServer->isConnected()){
            rt_task_wait_period(nullptr); //wait for next cycle

            if (pTcpServer->dataControl->ClientToServer.opMode >= 2){
                pTcpServer->sendData();
            }

            now = rt_timer_read();
            previous = now;
        }
    }
}

void TcpServerCustom::comm_stop(){
    rt_task_suspend(&comm_task);
    printf("Tcp Comm RT Task Stop\n");
    rt_task_delete(&comm_task);
    printf("Tcp Comm RT Task Delete\n");
}
