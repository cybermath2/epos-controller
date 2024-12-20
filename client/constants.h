#ifndef CONSTANTS_H
#define CONSTANTS_H

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
static const char *DEV_NAME    = "EPOS4";
static const char *PROTO_NAME  = "CANopen";
static const char *IF_NAME     = "CAN_mcp251x 0";
static const char *PORT_NAME   = "CAN0";

// port settings
static const uint32_t BAUDRATE = 1000000; // 1000 kbit/s
static const uint32_t TIMEOUT  = 500; // 500 ms

// node settings
static const uint16_t NODE_ID_PITCH 	= 1;
static const uint16_t NODE_ID_ROLL 	= 2;
static const uint16_t NODE_ID_YAW 	= 3;

// motor settings
static const uint16_t MOTOR_TYPE = MT_EC_SINUS_COMMUTATED_MOTOR; // motor-specific
static const uint32_t NOMINAL_CURRENT = 0; // motor-specific
static const uint32_t OUTPUT_CURRENT_LIMIT = 0; // user-specific
static const uint8_t  NUMBER_OF_POLE_PAIRS = 0; // motor-specific
static const uint16_t THERMAL_TIME_CONSTANT = 0; // motor-specific
static const uint32_t TORQUE_CONSTANT = 0; // motor-specific
static const uint32_t MAX_MOTOR_SPEED = 0; // user-specific
static const uint32_t MAX_GEAR_INPUT_SPEED = 0; // user-specific

// position sensor settings
// application settings
// ...
static const int32_t POS_DEG_MIN = 0;
static const int32_t POS_DEG_MAX = 0;

// network settings
static const char *SERVER_IP = "137.248.121.40";
static const in_port_t SERVER_PORT = 5001;

// movement constants and unit conversion
static const uint32_t PPM_MAX_VELOCITY = 500;

// TODO find the exact values
static const double DEGTOINC = 50000.0 / 45.0;
static const double INCTODEG = 1.0 / DEGTOINC;
static const double RPMTOVEL = 100.0;
static const double VELTORPM = 1.0 / RPMTOVEL;

// COB-IDs for objects which cannot be configured directly through the library
static const struct cob_id COB_ID_NUMBER_OF_POLE_PAIRS = { .id = 0x3001, .sid = 0x03 };
static const struct cob_id COB_ID_TORQUE_CONSTANT      = { .id = 0x3001, .sid = 0x05 };
static const struct cob_id COB_ID_MAX_MOTOR_SPEED      = { .id = 0x6080, .sid = 0x00 };
static const struct cob_id COB_ID_MAX_GEAR_INPUT_SPEED = { .id = 0x3003, .sid = 0x03 };

static const struct cob_id COB_ID_UNIT_POS = { .id = 0x60A8 };
static const struct cob_id COB_ID_UNIT_VEL = { .id = 0x60A9 };
static const struct cob_id COB_ID_UNIT_ACC = { .id = 0x60AA };

static const struct cob_id COB_ID_FAULT_CODE 	  = { .id = 0x605E };
static const struct cob_id COB_ID_ERROR_REGISTER = { .id = 0x1001 };
static const struct cob_id COB_ID_STATUS_WORD 	  = { .id = 0x6041 };
static const struct cob_id COB_ID_STO_STATES 	  = { .id = 0x3202, .sid = 0x01 };

// software-defined position limit
static const struct cob_id COB_ID_MIN_POS = { .id = 0x607D, .sid = 0x01 };
static const struct cob_id COB_ID_MAX_POS = { .id = 0x607D, .sid = 0x02 };

#endif