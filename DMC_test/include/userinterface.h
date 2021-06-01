#ifndef USERINTERFACE_H
#define USERINTERFACE_H

#include "globalstruct.h"
#include "ecatslave.h"
#include "ecatmaster.h"

#include <string.h>
#include <pthread.h>

class userinterface
{
public:
    userinterface();
    ~userinterface();

    ecatmaster v_global_ethercat_control;

    void start();
    void on_v_action_pb_ECAT_start_clicked();
    void on_v_action_pb_SERVO_BT_clicked();
    void f_oper_stateUpdateTimer_Callback();
    void on_v_action_pb_servo_RUN_clicked();

    void on_v_action_pb_servo_STOP_clicked();

    pthread_t timer;

private :
    int   v_global_cur_all_state   = REQ_SERVO_OFF;  // current all slave state
    int   v_global_prev_all_state  = -1;  			// preview all slave state
    int   v_global_prev_ecat_state = -1;


    dec_slave_state_t    v_global_prev_slave_state[MAX_SLAVES];
    dec_slave_state_t    v_global_prev_online_slave_state[MAX_SLAVES];


    int v_global_recheck_flag = 0;

//    QTimer *v_global_stateUpdateTimer;
};

#endif // USERINTERFACE_H
