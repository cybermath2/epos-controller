#pragma once

// don't include this file twice - it is bad practice to put definitions into a
// header instead of externalizing and linking them, but we don't care

#include <stdint.h>
#include <netinet/in.h>

#include "deps/definitions.h"

struct cob_id {
        uint16_t id;
        uint8_t sid;
};

#define MAX_STR_SIZE 128

// port settings
const char *DEV_NAME    = "EPOS4";
const char *PROTO_NAME  = "CANopen";
const char *IF_NAME     = "CAN_mcp251x 0";
const char *PORT_NAME   = "CAN0";

// port settings
const uint32_t BAUDRATE = 1000000; // 1000 kbit/s
const uint32_t TIMEOUT  = 500; // 500 ms

// node settings
const uint16_t NODE_ID_YAW 	= 3;
const uint16_t NODE_ID_PITCH 	= 2;
const uint16_t NODE_ID_ROLL 	= 1;

// motor settings
const uint16_t MOTOR_TYPE = MT_EC_SINUS_COMMUTATED_MOTOR; // motor-specific
const uint32_t NOMINAL_CURRENT = 0; // motor-specific
const uint32_t OUTPUT_CURRENT_LIMIT = 0; // user-specific
const uint8_t  NUMBER_OF_POLE_PAIRS = 0; // motor-specific
const uint16_t THERMAL_TIME_CONSTANT = 0; // motor-specific
const uint32_t TORQUE_CONSTANT = 0; // motor-specific
const uint32_t MAX_MOTOR_SPEED = 0; // user-specific
const uint32_t MAX_GEAR_INPUT_SPEED = 0; // user-specific

// position sensor settings
// application settings
// ...
const int32_t POS_DEG_MIN = -45;
const int32_t POS_DEG_MAX =  45;

// network settings
const char *SERVER_IP = "127.0.0.1";
const in_port_t SERVER_PORT = 5001;

// movement constants and unit conversion
const uint32_t PPM_MAX_VELOCITY = 1000;

// TODO find the exact values
const double DEGTOINC = 50000.0 / 45.0;
const double INCTODEG = 1.0 / DEGTOINC;
const double RPMTOVEL = 100.0;
const double VELTORPM = 1.0 / RPMTOVEL;

// COB-IDs for objects which cannot be configured directly through the library
const struct cob_id COB_ID_NUMBER_OF_POLE_PAIRS = { .id = 0x3001, .sid = 0x03 };
const struct cob_id COB_ID_TORQUE_CONSTANT      = { .id = 0x3001, .sid = 0x05 };
const struct cob_id COB_ID_MAX_MOTOR_SPEED      = { .id = 0x6080, .sid = 0x00 };
const struct cob_id COB_ID_MAX_GEAR_INPUT_SPEED = { .id = 0x3003, .sid = 0x03 };

const struct cob_id COB_ID_UNIT_POS = { .id = 0x60A8 };
const struct cob_id COB_ID_UNIT_VEL = { .id = 0x60A9 };
const struct cob_id COB_ID_UNIT_ACC = { .id = 0x60AA };

const struct cob_id COB_ID_FAULT_CODE 	  = { .id = 0x605E };
const struct cob_id COB_ID_ERROR_REGISTER = { .id = 0x1001 };
const struct cob_id COB_ID_STATUS_WORD 	  = { .id = 0x6041 };

// software-defined position limit
const struct cob_id 
COB_ID_MIN_POS = { .id = 0x607D, .sid = 0x01 };
const struct cob_id COB_ID_MAX_POS = { .id = 0x607D, .sid = 0x02 };
