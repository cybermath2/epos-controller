#include <stdarg.h>
#include <stdio.h>
#include <stdbool.h>
#include <string.h>

#include "constants.h"
#include "main.h"


void set(const struct cob_id *id, uint16_t node_id, void *p, size_t n)
{
        uint32_t bytes_written;
        uint32_t err;
        if (!VCS_SetObject(port, node_id, id->id, id->sid, p, n, &bytes_written, &err)) {
                die(err, "failed to set object %X-%X", id->id, id->sid);
        }
}


void get(void *buf, size_t n, uint16_t node_id, const struct cob_id *id)
{
        uint32_t bytes_read;
        uint32_t err;
        if (!VCS_GetObject(port, node_id, id->id, id->sid, buf, n, &bytes_read, &err)) {
                die(err, "failed to get object %X-%X", id->id, id->sid);
        }
}

void die(uint32_t err, const char *what, ...)
{
        va_list args;
        va_start(args, what);
        fprintf(stderr, "\033[1;31m");
        vfprintf(stderr, what, args);
        fprintf(stderr, " (error code 0x%X)", err);
        va_end(args);

        if (err != 0) {
                char err_info[MAX_STR_SIZE];
                if (VCS_GetErrorInfo(err, err_info, MAX_STR_SIZE)) {
                        fprintf(stderr, ": %s\n", err_info);
                }
        }

        if (port == NULL) {
            // only print node-specific errors if port is set
            goto end;
        }

	fprintf(stderr, "node-specific errors follow:\n");

	uint16_t ids[] = { NODE_ID_YAW, NODE_ID_PITCH, NODE_ID_ROLL };

	for (size_t i = 0; i < sizeof(ids) / sizeof(ids[0]); i++) {
		uint16_t id = ids[i];
		uint8_t num_err;
		if (VCS_GetNbOfDeviceError(port, id, &num_err, &err)) {
			fprintf(stderr, "node %u\n", id);

			for (uint8_t i = 1; i <= num_err; i++) {
				uint32_t device_err;
				char err_info[MAX_STR_SIZE];
				if (!VCS_GetDeviceErrorCode(port, id, num_err, &device_err, &err) ||
				    !VCS_GetErrorInfo(device_err, err_info, MAX_STR_SIZE)) {
					continue;
				}

				fprintf(stderr, "|-> (error code 0x%X): %s\n", device_err, err_info);
			}
		}
	}

end:
        fprintf(stderr, "\033[1;0m\n");
        exit(1);
}

void print_velocity(uint16_t node_id)
{
        int32_t velocity;
        uint32_t err;
        if (!VCS_GetVelocityIs(port, node_id, &velocity, &err)) {
                die(err, "failed to get velocity is");
        }

        printf("current velocity: %d\n", velocity);
}

void print_position(uint16_t node_id)
{
        int32_t position;
        uint32_t err;
        if (!VCS_GetPositionIs(port, node_id, &position, &err)) {
                die(err, "failed to get position is");
        }

        printf("current position: %d\n", position);
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

        // available device, protocol and interface names
        char name[MAX_STR_SIZE];
        int eos = false;
        if (!VCS_GetDeviceNameSelection(true, name, MAX_STR_SIZE, &eos, &err)) {
                die(err, "failed to get available device names");
        }

        printf("possible device names: '%s', ", name);

        while (!eos) {
                if (!VCS_GetDeviceNameSelection(false, name, MAX_STR_SIZE, &eos, &err)) {
                        die(err, "failed to get available device names");
                }
                printf("'%s', ", name);
        }

        printf("\n");

        if (!VCS_GetProtocolStackNameSelection((char*) DEV_NAME, true, name, MAX_STR_SIZE, &eos, &err)) {
                die(err, "failed to get available protocol stack names ");
        }

        printf("possible protocol stack names: '%s', ", name);

        while (!eos) {
                if (!VCS_GetProtocolStackNameSelection((char*) DEV_NAME, false, name, MAX_STR_SIZE, &eos, &err)) {
                        die(err, "failed to get available protocol stack names");
                }
                printf("'%s', ", name);
        }

        printf("\n");

        if (!VCS_GetInterfaceNameSelection((char*) DEV_NAME, (char*) PROTO_NAME, true, name, MAX_STR_SIZE, &eos, &err)) {
                die(err, "failed to get available interface names");
        }

        printf("possible interface names: '%s', ", name);

        while (!eos) {
                if (!VCS_GetInterfaceNameSelection((char*) DEV_NAME, (char*) PROTO_NAME, false, name, MAX_STR_SIZE, &eos, &err)) {
                        die(err, "failed to get available interface names");
                }
                printf("'%s', ", name);
        }

        printf("\n");
}

void node_info_dump(uint16_t node_id)
{
        printf("getting node information for node %u...\n", node_id);

	if (node_id == 0xFFFF) {
		printf("invalid node id: %u\n", node_id);
		return;
	}

        // motor parameters
        uint16_t motor_type;
        uint32_t err;
        if (!VCS_GetMotorType(port, node_id, &motor_type, &err)) {
                die(err, "failed to get motor type");
        }

        printf("motor type=%d\n", motor_type);
        uint32_t nominal_current;
        uint32_t output_current_limit;
        uint16_t thermal_time_constant;

        if (!VCS_GetDcMotorParameterEx(port, node_id, &nominal_current,
                                &output_current_limit, &thermal_time_constant, &err)) {
                die(err, "failed to get motor parameters");
        }

        printf("nominal current=%d, output current limit=%d, thermal time constant winding=%d\n",
                        nominal_current, output_current_limit, thermal_time_constant);

        if (motor_type == MT_EC_BLOCK_COMMUTATED_MOTOR || motor_type == MT_EC_SINUS_COMMUTATED_MOTOR) {
                uint8_t number_of_pole_pairs;
                get(&number_of_pole_pairs, sizeof(number_of_pole_pairs), node_id, &COB_ID_NUMBER_OF_POLE_PAIRS);
                printf("number of pole pairs=%d\n", number_of_pole_pairs);
        }

        uint32_t torque_constant;
        uint32_t max_motor_speed;
        uint32_t max_gear_input_speed;
        get(&torque_constant, sizeof(torque_constant), node_id, &COB_ID_TORQUE_CONSTANT);
        get(&max_motor_speed, sizeof(max_motor_speed), node_id, &COB_ID_MAX_MOTOR_SPEED);
        get(&max_gear_input_speed, sizeof(max_gear_input_speed), node_id, &COB_ID_MAX_GEAR_INPUT_SPEED);

        printf("torque constant=%d, max motor speed=%d, max gear input speed=%d\n",
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

uint16_t axis_to_node_id(const char *axis)
{
        int node_id = -1;
        if (strcmp(axis, "yaw") == 0) {
                node_id = NODE_ID_YAW;
        } else if (strcmp(axis, "pitch") == 0) {
                node_id = NODE_ID_PITCH;
        } else if (strcmp(axis, "roll") == 0) {
                node_id = NODE_ID_ROLL;
        }

        return node_id;
}
