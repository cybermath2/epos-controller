#include "util.h"

#include "settings.h"
#include "deps/Definitions.h"

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

void set(const struct cob_id *id, void *port, uint16_t node_id, void *p, size_t n)
{
        uint32_t bytes_written;
        uint32_t err;
        if (!VCS_SetObject(port, node_id, id->id, id->sid, p, n, &bytes_written, &err)) {
                die(err, "failed to set object %x-%x", id->id, id->sid);
        }
}


void get(void *buf, size_t n, void *port,
         uint16_t node_id, const struct cob_id *id)
{
        uint32_t bytes_read;
        uint32_t err;
        if (!VCS_GetObject(port, node_id, id->id, id->sid, buf, n, &bytes_read, &err)) {
                die(err, "failed to get object %x-%x", id->id, id->sid);
        }
}

void die(uint32_t err, const char *what, ...)
{
        va_list args;
        va_start(args, what);
        fprintf(stderr, "|-> ");
        vfprintf(stderr, what, args);
        fprintf(stderr, "(err code 0x%x)\n", err);
        va_end(args);

        if (err != 0) {
                char err_info[MAX_STR_SIZE];
                if (VCS_GetErrorInfo(err, err_info, MAX_STR_SIZE)) {
                        fprintf(stderr, " (%s)", err_info);
                }
        }

        fprintf(stderr, "\n");
        exit(err);
}

void driver_info_dump(void)
{
        printf("getting driver information...\n");

        char lib_name[MAX_STR_SIZE];
        char lib_version[MAX_STR_SIZE];

        uint32_t err;
        if (!VCS_GetDriverInfo(lib_name, MAX_STR_SIZE,
                               lib_version, MAX_STR_SIZE, &err)) {
                die(err, "failed to get driver information");
        }

        printf("driver name='%s' (version '%s')\n", lib_name, lib_version);
}

void node_info_dump(void *port, uint16_t node_id)
{
        printf("getting node information...\n");

        // units (position, velocity, acceleration)
        uint32_t err;
        uint32_t unit;
        char buf_unit[16];
        // position
        get(&unit, sizeof(unit), port, node_id, &COB_ID_UNIT_POS);
        str_unit(buf_unit, sizeof(buf_unit), unit);
        printf("position unit: %s\n", buf_unit);

        // velocity
        get(&unit, sizeof(unit), port, node_id, &COB_ID_UNIT_POS);
        str_unit(buf_unit, sizeof(buf_unit), unit);
        printf("velocity unit: %s\n", buf_unit);

        // acceleration
        get(&unit, sizeof(unit), port, node_id, &COB_ID_UNIT_POS);
        str_unit(buf_unit, sizeof(buf_unit), unit);
        printf("acceleration unit: %s\n", buf_unit);

        // motor parameters
        uint16_t motor_type;
        if (!VCS_GetMotorType(port, node_id, &motor_type, &err)) {
                die(err, "failed to get motor type");
        }

        printf("|-> motor type=%d\n", motor_type);
        uint32_t nominal_current;
        uint32_t output_current_limit;
        uint16_t thermal_time_constant;

        if (!VCS_GetDcMotorParameterEx(port, node_id, &nominal_current,
                                       &output_current_limit, &thermal_time_constant, &err)) {
                die(err, "failed to get motor parameters");
        }

        printf("|-> nominal current=%d, output current limit=%d, thermal time constant winding=%d\n",
               nominal_current, output_current_limit, thermal_time_constant);

        if (motor_type == MT_EC_BLOCK_COMMUTATED_MOTOR || motor_type == MT_EC_SINUS_COMMUTATED_MOTOR) {
                uint8_t number_of_pole_pairs;
                get(&number_of_pole_pairs, sizeof(number_of_pole_pairs), port,
                    node_id, &COB_ID_NUMBER_OF_POLE_PAIRS);
                printf("|-> number of pole pairs=%d\n", number_of_pole_pairs);
        }

        uint32_t torque_constant;
        uint32_t max_motor_speed;
        uint32_t max_gear_input_speed;
        get(&torque_constant, sizeof(torque_constant), port, node_id, &COB_ID_TORQUE_CONSTANT);
        get(&max_motor_speed, sizeof(max_motor_speed), port, node_id, &COB_ID_MAX_MOTOR_SPEED);
        get(&max_gear_input_speed, sizeof(max_gear_input_speed), port, node_id, &COB_ID_MAX_GEAR_INPUT_SPEED);

        printf("|-> torque constant=%d, max motor speed=%d, max gear input speed=%d\n",
               torque_constant, max_motor_speed, max_gear_input_speed);
}

void str_motor_type(char *buf, size_t n, uint16_t motor_type)
{
        const char *str = "";

        switch (motor_type) {
                case MT_DC_MOTOR:
                        str = "brushed DC motor"; break;
                case MT_EC_SINUS_COMMUTATED_MOTOR:
                        str = "EC motor sinus commutated"; break;
                case MT_EC_BLOCK_COMMUTATED_MOTOR:
                        str = "EC motor block commutated"; break;
                default:
                        break;
        }

        strncpy(buf, str, n);
}

void str_unit_dim(char *buf, size_t n, uint8_t dimension)
{
        const char *str = "";

        switch (dimension) {
                case 0x00:
                        str = "-"; break;
                case 0x01:
                        str = "m"; break;
                case 0x02:
                        str = "kg"; break;
                case 0x03:
                        str = "s"; break;
                case 0x04:
                        str = "A"; break;
                case 0x47:
                        str = "min"; break;
                case 0x57:
                        str = "(s^2)"; break;
                case 0xB4:
                        str = "rev"; break;
                case 0xB5:
                        str = "inc"; break;
                case 0xAC:
                        str = "steps"; break;
                case 0xC0:
                        str = "rpm"; break;
                default:
                        break;
        }

        strncpy(buf, str, n);
}

void str_unit_not(char *buf, size_t n, uint8_t notation)
{
        const char *str = "";

        switch (notation) {
                case 0x06:
                        str = "M"; break;
                case 0x03:
                        str = "k"; break;
                case 0x02:
                        str = "h"; break;
                case 0x01:
                        str = "da"; break;
                case 0x00:
                        str = "-"; break;
                case 0xFF:
                        str = "d"; break;
                case 0xFE:
                        str = "c"; break;
                case 0xFD:
                        str = "m"; break;
                case 0xFC:
                        str = "-"; break;
                case 0xFB:
                        str = "-"; break;
                case 0xFA:
                        str = "u"; break;
                default:
                        break;
        }

        strncpy(buf, str, n);
}

void str_unit(char *buf, size_t n, uint32_t unit)
{
        uint8_t denom   = (unit >> 8)  & 0xff;
        uint8_t num     = (unit >> 16) & 0xff;
        uint8_t prefix  = (unit >> 24) & 0xff;

        char buf_denom[8];
        char buf_num[8];
        char buf_prefix[8];

        str_unit_dim(buf_denom, sizeof(buf_denom), denom);
        str_unit_dim(buf_num, sizeof(buf_num), num);
        str_unit_dim(buf_prefix, sizeof(buf_prefix), prefix);

        snprintf(buf, n, "(%s%s)/%s", buf_prefix, buf_num, buf_denom);
}
