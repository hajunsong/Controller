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

class ControlMain{
public:
    ControlMain();
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
    void robotVSD();

    DxlControl *module;
    char data_indx;

    TcpServer *tcpServer;

    void getPresentEnc(int32_t enc[NUM_JOINT]);
    void setOffsetEnc(int32_t enc[NUM_JOINT]);

    static void* init_func(void* arg);
    bool init_thread_run;

    void setObiMode();
    void unsetObiMode();
    void putObiMode(char key);
private:
    void robot_RT();

    RobotArm *robotArm;

    void moduleInitSEA();
    void moduleInitFAR();
	bool old_end_pose_update;

    void goalReach(double desired_pose[NUM_DOF], double present_pose[NUM_DOF], bool *goal_reach);
    void path_generator(double x0, double xf, double tf, double ta, double h, std::vector<double> *path);

    uint8_t module_indx;

    bool ready_pose;
    bool cartesian_move_flag;
    int delay_cnt, delay_cnt_max;
    int fork_cnt, fork_cnt_max;
    int32_t origin_pos;
    double step_size;

    double goal_current[NUM_JOINT];

    pthread_t init_thread;
};

