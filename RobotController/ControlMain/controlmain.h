#include <stdio.h>
#include <signal.h>
#include <unistd.h>
#include <sys/mman.h>

#include <native/task.h>
#include <native/timer.h>
#include <rtdk.h>

#include <pthread.h>

#include "DataControl/datacontrol.h"
#include "Dynamixel/dynamixel.h"
#include "TcpServer/tcpserver.h"
#include "RobotArm/robotarm.h"
#include "CustomFunc/controlmain_custom.h"

#include <QObject>
#include <QTimer>

#include <QtCore/qglobal.h>

#if defined(CONTROLMAINLIB_LIBRARY)
#  define CONTROLMAINLIB_EXPORT Q_DECL_EXPORT
#else
#  define CONTROLMAINLIB_EXPORT Q_DECL_IMPORT
#endif

class ControlMain : public QObject{
    Q_OBJECT
public:
    explicit ControlMain(QObject *parent = nullptr);
    ~ControlMain();
    void start();
    bool robot_thread_run;
    RT_TASK robot_task;
    ControlMainCustom *controlMainCustom;
    void robot_RT_stop();
    DataControl *dataControl;

    void robotInitialize();
    void robotServoOn(char enable);
    void robotKinematics();
    void robotDynamics();
    void robotJointMove(char mode, double desJoint[NUM_JOINT]);
    void robotCartesianMove(char mode, double desCartesian[NUM_DOF]);
    void robotPathGenerate();
    void robotRun();
    void robotReady();
	void robotSPGC();
    void robotOperate();

    DxlControl *module;
    char data_indx;

    NRMKHelper::TcpServer *tcpServer;

    void getPresentEnc(int32_t enc[NUM_JOINT]);
    void setOffsetEnc(int32_t enc[NUM_JOINT]);
private:
    QTimer *dxlTimer;
    void robot_RT();

    RobotArm *robotArm;

    void moduleInitSEA();
    void moduleInitFAR();
    bool module_init;
	bool old_end_pose_update;

    void goalReach(double desired_pose[NUM_DOF], double present_pose[NUM_DOF], bool *goal_reach);
    void path_generator(double x0, double xf, double tf, double ta, double h, std::vector<double> *path);

    uint8_t module_indx;

    bool ready_pose;
    bool cartesian_move_flag;
    int delay_cnt, delay_cnt_max;

signals:
    void disconnectClientSignal();

public slots:
    void dxlTimeout();
    void disconnectClient();
};

