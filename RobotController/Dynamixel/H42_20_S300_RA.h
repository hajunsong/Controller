#pragma once

#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include "dynamixel_sdk/dynamixel_sdk.h"

namespace SEA{

enum { current_mode = 0, velocity_mode, position_mode = 3, extended_position_mode, current_based_position_mode, pwm_mode = 16 };

// Control table address
// ROM Area
const uint8_t ADDR_MODEL_NUMBER            = 0;
const uint8_t ADDR_MODEL_INFORMATION       = 2;
const uint8_t ADDR_FIRMWARE_VERSION        = 6;
const uint8_t ADDR_ID                      = 7;
const uint8_t ADDR_BAUD_RATE               = 8;
const uint8_t ADDR_RETURN_DELAY_TIME       = 9;
const uint8_t ADDR_DRIVE_MODE              = 10;
const uint8_t ADDR_OPERATING_MODE          = 11;
const uint8_t ADDR_SENCONDARY_ID           = 12;
const uint8_t ADDR_HOMING_OFFSET           = 20;
const uint8_t ADDR_MOVING_THRESHOLD        = 24;
const uint8_t ADDR_TEMPERATURE_LIMIT       = 31;
const uint8_t ADDR_MAX_VOLTAGE_LIMIT       = 32;
const uint8_t ADDR_MIN_VOLTAGE_LIMIT       = 34;
const uint8_t ADDR_PWM_LIMIT               = 36;
const uint8_t ADDR_CURRENT_LIMIT           = 38;
//const uint8_t ADDR_ACCELERATION_LIMIT      = 40;
const uint8_t ADDR_VELOCITY_LIMIT          = 44;
const uint8_t ADDR_MAX_POSITION_LIMIT      = 48;
const uint8_t ADDR_MIN_POSITION_LIMIT      = 52;
const uint8_t ADDR_SHUTDOWN                = 58;
const uint8_t ADDR_INDIRECT_ADDR_START     = 168;
// RAM Area
const uint16_t ADDR_TORQUE_ENABLE          = 512;
const uint16_t ADDR_LED_R                  = 513;
const uint16_t ADDR_LED_G                  = 514;
const uint16_t ADDR_LED_B                  = 515;
const uint16_t ADDR_STATUS_RETURN_LEVEL    = 516;
const uint16_t ADDR_REGISTERED_INSTRUCTION = 517;
const uint16_t ADDR_HARDWARE_ERROR_STATUS  = 518;
const uint16_t ADDR_VELOCITY_I_GAIN        = 524;
const uint16_t ADDR_VELOCITY_P_GAIN        = 526;
const uint16_t ADDR_POSITION_P_GAIN        = 528;
const uint16_t ADDR_POSITION_I_GAIN        = 532;
const uint16_t ADDR_POSITION_D_GAIN        = 530;
const uint16_t ADDR_FEEDFORWARD_2ND_GAIN   = 536;
const uint16_t ADDR_FEEDFORWARD_1ST_GAIN   = 538;
const uint16_t ADDR_BUS_WATCHDOG           = 546;
const uint16_t ADDR_GOAL_PWM               = 548;
const uint16_t ADDR_GOAL_CURRENT           = 550;
const uint16_t ADDR_GOAL_VELOCITY          = 552;
const uint16_t ADDR_PROFILE_ACCELERATION   = 556;
const uint16_t ADDR_PROFILE_VELOCITY	   = 560;
const uint16_t ADDR_GOAL_POSITION          = 564;
const uint16_t ADDR_REALTIME_TICK          = 568;
const uint16_t ADDR_MOVING                 = 570;
const uint16_t ADDR_MOVING_STATUS          = 571;
const uint16_t ADDR_PRESENT_PWM            = 572;
const uint16_t ADDR_PRESENT_CURRENT        = 574;
const uint16_t ADDR_PRESENT_VELOCITY       = 576;
const uint16_t ADDR_PRESENT_POSITION       = 580;
const uint16_t ADDR_VELOCITY_TRAJECTORY    = 584;
const uint16_t ADDR_POSITION_TRAJECTORY    = 588;
const uint16_t ADDR_PRESENT_INPUT_VOLTAGE  = 592;
const uint16_t ADDR_PRESENT_TEMPERATURE    = 594;
const uint16_t ADDR_INDIRECT_DATA_START    = 634;

// Lenght address
const uint8_t LEN_PRESENT_POSITION = 4;
const uint8_t LEN_PRESENT_VELOCITY = 4;
const uint8_t LEN_PRESENT_CURRENT = 2;
const uint8_t LEN_PRESENT_TEMPERATURE = 1;
const uint8_t LEN_HARDWARE_ERR_STATUS = 1;
const uint8_t LEN_GOAL_POSITION	= 4;

// Protocol version
const float PROTOCOL_VERSION = 2.0;

const uint32_t BAUDRATE = 1000000;
const char DEVICENAME[] = "/dev/ttyUSB0";

const uint8_t BR_DF = 1;
const uint8_t BR_1M = 3;
const uint8_t BR_2M	= 4;
const uint8_t BR_4M = 6;
const uint8_t BR_45M = 7;

const uint8_t TORQUE_ENABLE = 1;
const uint8_t TORQUE_DISABLE = 0;

const uint16_t FACTORYRST_DEFAULTBAUDRATE = 57600;  // Dynamixel baudrate set by factoryreset
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
    int dxl_init(uint8_t ID);
    void dxl_deinit(uint8_t ID, int32_t home_pos);
    void dxl_deinit(uint8_t ID);

    void setLEDon(uint8_t ID);
    void setLEDoff(uint8_t ID);
    void setPosition(int32_t goal_position, uint8_t ID);
    void setVelocity(int32_t goal_velocity, uint8_t ID);
    void setOperateMode(uint8_t mode, uint8_t ID);
    void setTorqueEnable(uint8_t enable, uint8_t ID);

    int32_t getPresentPosition(uint8_t ID);
    int32_t getPresentVelocity(uint8_t ID);
    int16_t getPresentCurrent(uint8_t ID);
    double getPresentVoltage(uint8_t ID);
    void getGroupSyncReadPresentPosition(int32_t *presentPosition);
    void getGroupSyncReadPresentVelocity(int32_t *presentPosition);
    void setGroupSyncWriteGoalPosition(int32_t *goalPosition);

    dynamixel::PortHandler *portHandler;
    dynamixel::PacketHandler *packetHandler;
    dynamixel::GroupSyncRead *groupSyncReadPos;
    dynamixel::GroupSyncRead *groupSyncReadVel;
private:
};

const double POSITION_UNIT = 0.088;	// [deg]
const double VELOCITY_UNIT = 0.229;	// [RPM]
//const double DEG2RAD = M_PI/180.0;
//const double RAD2DEG = 180.0/M_PI;

}

using namespace SEA;
