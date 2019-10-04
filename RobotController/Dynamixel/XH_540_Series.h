#pragma once

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

using namespace std;
typedef unsigned int uint;

namespace FAR{

enum { current_mode = 0, velocity_mode, position_mode = 3, extended_position_mode, current_based_position_mode, pwm_mode = 16 };

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

// Lenght address
const uint8_t LEN_PRESENT_POSITION = 4;
const uint8_t LEN_PRESENT_VELOCITY = 4;
const uint8_t LEN_GOAL_POSITION = 4;
const uint8_t LEN_TORQUE_ENABLE = 1;

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

class DxlControl
{
public:
    DxlControl();
    ~DxlControl();

    int dxl_comm_result;// = COMM_TX_FAIL;
    uint8_t dxl_error;// = 0;	// Dynamixel error
    uint32_t moving_threshold;// = 0;
    int16_t current_limit;

    void init();
    int dxl_init(uint8_t ID, bool factory_reset);
    void dxl_deinit(uint8_t ID, int32_t home_pos);
    void dxl_deinit(uint8_t ID);

    void setLEDon(uint8_t ID);
    void setLEDoff(uint8_t ID);
    void setGoalPosition(int32_t goal_position, uint8_t ID);
    void setGoalVelocity(int32_t goal_velocity, uint8_t ID);
    void setOperateMode(uint8_t mode, uint8_t ID);
    void setTorqueEnable(uint8_t enable, uint8_t ID);

    void getPresentPosition(uint8_t ID, int32_t *present_position_ptr);
    int32_t getPresentVelocity(uint8_t ID);
    int16_t getPresentCurrent(uint8_t ID);
    double getPresentVoltage(uint8_t ID);
    void getGroupSyncReadPresentPosition(int32_t *presentPosition);
    void getGroupSyncReadPresentVelocity(int32_t *presentPosition);
    void setGroupSyncWriteGoalPosition(int32_t *goalPosition);
    void setGroupSyncWriteTorqueEnable(uint8_t enable);

private:
    dynamixel::PortHandler *portHandler;
    dynamixel::PacketHandler *packetHandler;
    dynamixel::GroupSyncRead *groupSyncReadPos;
    dynamixel::GroupSyncRead *groupSyncReadVel;

    bool init_flag;
};

const double POSITION_UNIT = 0.088;	// [deg]
const double VELOCITY_UNIT = 0.229;	// [RPM]
}

using namespace FAR;
