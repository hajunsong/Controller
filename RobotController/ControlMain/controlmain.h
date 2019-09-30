#include <stdio.h>
#include <signal.h>
#include <unistd.h>
#include <sys/mman.h>

#include <native/task.h>
#include <native/timer.h>
#include <rtdk.h>

#include <pthread.h>

#include "DataControl/datacontrol.h"
#if(MODULE_TYPE == 1)
#include "Dynamixel/XH_540_Series.h"
#elif(MODULE_TYPE == 2)
#include "Dynamixel/H42_20_S300_RA.h"
#endif
#include "TcpServer/tcpserver.h"
#include "RobotArm/robotarm.h"

#include <QObject>
#include <QTimer>

class ControlMain : public QObject{
    Q_OBJECT
public:
    explicit ControlMain(QObject *parent = nullptr);
    ~ControlMain();
    void start();
private:
    void robot_RT();
    static void robot_run[[ noreturn ]] (void *arg);

    NRMKHelper::TcpServer *tcpServer;
    DataControl *dataControl;

    RT_TASK robot_task;
    bool robot_thread_run;

    RobotArm *robotArm;

    DxlControl *module;
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
    void robotRun(int16_t type);
    void robotReady(int16_t type);

    void goalReach(double desired_pose[NUM_DOF], double present_pose[NUM_DOF], bool *goal_reach);

    unsigned char data_indx;
    uint8_t module_indx;

    QTimer *dxlTimer;

    bool ready_pose;
    bool cartesian_move_flag;
    int delay;

public slots:
    void dxlTimeout();
};

