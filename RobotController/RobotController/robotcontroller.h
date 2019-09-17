#ifndef CONTROLLERMAIN_H
#define CONTROLLERMAIN_H

#include <stdio.h>
#include <signal.h>
#include <unistd.h>
#include <sys/mman.h>
#include <native/task.h>
#include <native/timer.h>
#include <rtdk.h>

#include <iostream>
#include <vector>

#include <QTimer>
#include <QObject>
#include <QDebug>
#include <QThread>

#include "TcpServer/tcpserver.h"
#include "DataControl/datacontrol.h"
#include "RobotArm/robotarm.h"

#if(MODULE_TYPE == 1)
#include "Dynamixel/XH_540_Series.h"
#elif(MODULE_TYPE == 2)
#include "Dynamixel/H42_20_S300_RA.h"
#endif

class RobotController : public QObject{
    Q_OBJECT

public:
    explicit RobotController(QObject *parent = nullptr);
    ~RobotController();
    void start();

private:
    void robot_RT();
    static void robot_run(void *arg);

    NRMKHelper::TcpServer *tcpServer;
    DataControl *dataControl;

    RT_TASK robot_task;
    bool robot_thread_run;

    QTimer *dxlTimer;
    DxlControl *module;

    RobotArm *robotArm;

    void moduleInitSEA();
    void moduleInitFAR();
    bool module_init;
    bool old_end_pose_update;

    void robotInitialize();
    void robotServoOn(char enable);
    void robotKinematics();
    void robotDynamics();
    void robotJointMove(char mode, double desJoint[NUM_JOINT]);
    void robotCartesianMove(char mode, double desCartesian[NUM_DOF]);

    void goalReach(int32_t desired_position[NUM_DOF], int32_t present_position[NUM_DOF], bool *goal_reach);

    unsigned char data_index;
    uint8_t module_indx;

public slots:
    void dxlTimeout();
};

#endif
