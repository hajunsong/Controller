#ifndef DYNAMIXELLIB_H
#define DYNAMIXELLIB_H

#if defined(__linux__) || defined(__APPLE__)
#include <fcntl.h>
#include <termios.h>
#define STDIN_FILENO 0
#elif defined(_WIN32) || defined(_WIN64)
#include <conio.h>
#endif

#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include "dynamixel_sdk/dynamixel_sdk.h"

#include <QtCore/qglobal.h>

#if defined(DYNAMIXELLIB_LIBRARY)
#  define DYNAMIXELLIB_EXPORT Q_DECL_EXPORT
#else
#  define DYNAMIXELLIB_EXPORT Q_DECL_IMPORT
#endif

using namespace std;
typedef unsigned int uint;

namespace FAR{

// Control table address
const uint8_t ADDR_TORQUE_ENABLE = 64;
const uint8_t ADDR_GOAL_POSITION = 116;
const uint8_t ADDR_GOAL_VELOCITY = 104;
const uint8_t ADDR_GOAL_CURRENT = 102;
const uint8_t ADDR_PRESENT_POSITION = 132;
const uint8_t ADDR_PRESENT_VELOCITY = 128;
const uint8_t ADDR_PRESENT_CURRENT = 126;
const uint8_t ADDR_PRESENT_INPUT_VOLTAGE = 144;
const uint8_t ADDR_OPERATING_MODE = 11;
const uint8_t ADDR_LED = 65;
const uint8_t ADDR_REALTIME_TICK = 120;
const uint8_t ADDR_BAUD_RATE = 8;
const uint8_t ADDR_VELOCITY_LIMIT =	44;
const uint8_t ADDR_CURRRENT_LIMIT =	38;
const uint8_t ADDR_PROFILE_VELOCITY = 112;
const uint8_t ADDR_PROFILE_ACCELERATION = 108;
const uint8_t ADDR_POSITION_P_GAIN = 84;
const uint8_t ADDR_POSITION_I_GAIN = 82;
const uint8_t ADDR_POSITION_D_GAIN = 80;
const uint8_t ADDR_RETURN_DELAY_TIME = 9;
const uint16_t ADDR_INDIRECTADDRESS_FOR_READ = 168;
const uint16_t ADDR_INDIRECTADDRESS_FOR_WRITE = 578;
const uint16_t ADDR_INDIRECTDATA_FOR_READ = 224;
const uint16_t ADDR_INDIRECTDATA_FOR_WRITE = 634;

// Lenght address
const uint8_t LEN_PRESENT_POSITION = 4;
const uint8_t LEN_PRESENT_VELOCITY = 4;
const uint8_t LEN_PRESENT_CURRENT = 2;
const uint8_t LEN_INDIRECTADDRESS_FOR_READ = LEN_PRESENT_POSITION + LEN_PRESENT_VELOCITY + LEN_PRESENT_CURRENT;
const uint8_t LEN_TORQUE_ENABLE = 1;
const uint8_t LEN_OPERATING_MODE = 1;
const uint8_t LEN_GOAL_POSITION = 4;
const uint8_t LEN_GOAL_CURRENT = 2;
const uint8_t LEN_INDIRECTADDRESS_FOR_WRITE = LEN_TORQUE_ENABLE + LEN_GOAL_POSITION + LEN_GOAL_CURRENT;

// Protocol version
#define PROTOCOL_VERSION 2.0

const char DEVICENAME[] = "/dev/ttyUSB0";
const int32_t BAUDRATE = 4000000;

const uint8_t BR_DF = 1;
const uint8_t BR_1M = 3;
const uint8_t BR_2M	= 4;
const uint8_t BR_4M = 6;
const uint8_t BR_45M = 7;

const uint8_t TORQUE_ENABLE = 1;
const uint8_t TORQUE_DISABLE = 0;

const uint16_t FACTORYRST_DEFAULTBAUDRATE = 57600;               // Dynamixel baudrate set by factoryreset
const uint8_t OPERATION_MODE = 0x01;                // 0xFF : reset all values
                                                    // 0x01 : reset all values except ID
                                                    // 0x02 : reset all values except ID and baudrate

class DYNAMIXELLIB_EXPORT DxlControl
{
public:
    DxlControl();
    ~DxlControl();

    int dxl_comm_result;// = COMM_TX_FAIL;
    uint8_t dxl_error;// = 0;	// Dynamixel error
    uint8_t single_id;

    void init();
    int dxl_init(uint8_t ID, uint8_t operating_mode);
    void dxl_deinit(uint8_t ID, int32_t home_pos);
    void dxl_deinit(uint8_t ID);
    void dxl_searching();

    void setLED(uint8_t ID, uint8_t on_off);
    void setGoalPosition(uint8_t ID, int32_t goal_position);
    void setGoalVelocity(uint8_t ID, int32_t goal_velocity);
    void setOperateMode(uint8_t ID, uint8_t mode);
    void setTorqueEnable(uint8_t ID, uint8_t enable);

    void getPresentPosition(uint8_t ID, int32_t *present_position_ptr);
    void getPresentVelocity(uint8_t ID, int32_t *present_velocity_ptr);
    void getPresentCurrent(uint8_t ID, int16_t *present_current_ptr);

	void initGroupSyncReadIndirectAddress(uint8_t ID);
	void getGroupSyncReadIndirectAddress(int32_t *present_position, int32_t *present_velocity, int16_t *present_current, uint8_t num_joint);
	void getGroupSyncReadIndirectAddress(int32_t *present_position, int32_t *present_velocity, int16_t *present_current, uint8_t num_joint, uint8_t ID);

    void initGroupSyncWriteIndirectAddress(uint8_t ID);
    void setGroupSyncWriteIndirectAddress(uint8_t *torque_enable, int32_t *goal_position, int16_t *goal_current, uint8_t num_joint);

    void setGroupSyncWriteTorqueEnable(uint8_t enable, uint8_t num_joint);
    void setGroupSyncWriteOperatingMode(uint8_t enable, uint8_t num_joint);
    void setGroupSyncWriteGoalPosition(int32_t *goalPosition, uint8_t num_joint);
    void setGroupSyncWriteGoalCurrent(int16_t *goalCurrent, uint8_t num_joint);

    void getGroupSyncReadPresentPosition(int32_t *present_position, uint8_t num_joint);

private:
    dynamixel::PortHandler *portHandler;
    dynamixel::PacketHandler *packetHandler;

    bool init_flag;
};

const double POSITION_UNIT = 0.088;	// [deg]
const double VELOCITY_UNIT = 0.229;	// [RPM]

enum JointOpMode{ current_mode = 0, velocity_mode, position_mode = 3, extended_position_mode, current_based_position_mode, pwm_mode = 16 };

const double TORQUE_CONSTANT_W270 = 2.8;
const double TORQUE_CONSTANT_W150 = 1.65;
const double TORQUE_CONSTANT_V270 = 8.0;
const double TORQUE_CONSTANT_V350 = 4.7;

}

using namespace FAR;

#endif // DYNAMIXELLIB_H
