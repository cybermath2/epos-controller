#include "constants.h"

#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <stdbool.h>
#include <string.h>
#include <errno.h>
#include <time.h>
#include <unistd.h>
#include <signal.h>

#include <net/if.h>
#include <arpa/inet.h>
#include <netinet/in.h>

#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/socket.h>

// SocketCAN
#include <linux/can.h>
#include <linux/can/raw.h>

static int8_t mode_last = 0xFF;
static void *port = NULL;

// utility -------------------------------------------------------------------------------
void set(const struct cob_id *id, uint16_t node_id, void *p, size_t n);
void get(void *buf, size_t n, uint16_t node_id, const struct cob_id *id);
void die(uint32_t err, const char *what, ...);

void print_velocity(uint16_t node_id);
void print_position(uint16_t node_id);

void driver_info_dump(void);
void node_info_dump(uint16_t node_id);
void str_motor_type(char *buf, size_t n, uint16_t motor_type);
uint16_t axis_to_node_id(const char *axis);

// communications ------------------------------------------------------------------------
void port_open(void);
void port_close(void);
void port_configure(void);
void node_reset(uint16_t node_id);
void node_configure(uint16_t node_id);

void can_read_loop(const char *port_name);

// command processing -------------------------------------------------------------------
void comm_start(void);
void comm_loop_enter(int fd);
void command_process(const char *cmd);

void int_handler(int signo);
void exit_gracefully(void);

// motion -------------------------------------------------------------------------------
void motion_halt(void);
void motion_position(const char *axis, bool relative, double deg);
void motion_velocity(const char *axis, double rpm);
void move_to_initial_position(void);

int main(int argc, char *argv[])
{
        driver_info_dump();
        //can_read_loop(PORT_NAME_SOCK);

	port_open();
        port_configure();

        //node_reset(NODE_ID_YAW);
        node_reset(NODE_ID_PITCH);
        //node_reset(NODE_ID_ROLL);

        //node_configure(NODE_ID_YAW);
        //node_configure(NODE_ID_PITCH);
        //node_configure(NODE_ID_ROLL);

        //node_info_dump(NODE_ID_YAW);
        //node_info_dump(NODE_ID_PITCH);
        //node_info_dump(NODE_ID_ROLL);

        uint32_t err;
	int32_t position;
	if (!VCS_GetPositionIs(port, NODE_ID_YAW, &position, &err)) {
		die(err, "failed to get position is");
	}
	printf("current position: %d\n");

        signal(SIGINT, int_handler);
        comm_start();

	exit_gracefully();
        return 0;
}

// utility ------------------------------------------------------------------------------
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

        if (!VCS_GetProtocolStackNameSelection(DEV_NAME, true, name, MAX_STR_SIZE, &eos, &err)) {
                die(err, "failed to get available protocol stack names ");
        }

        printf("possible protocol stack names: '%s', ", name);

        while (!eos) {
                if (!VCS_GetProtocolStackNameSelection(DEV_NAME, false, name, MAX_STR_SIZE, &eos, &err)) {
                        die(err, "failed to get available protocol stack names");
                }
                printf("'%s', ", name);
        }

        printf("\n");

        if (!VCS_GetInterfaceNameSelection(DEV_NAME, PROTO_NAME, true, name, MAX_STR_SIZE, &eos, &err)) {
                die(err, "failed to get available interface names");
        }

        printf("possible interface names: '%s', ", name);

        while (!eos) {
                if (!VCS_GetInterfaceNameSelection(DEV_NAME, PROTO_NAME, false, name, MAX_STR_SIZE, &eos, &err)) {
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

void port_open(void)
{
        printf("opening device '%s' using protocol '%s' on interface '%s' and"
                        " port '%s'...\n", DEV_NAME, PROTO_NAME, IF_NAME, PORT_NAME);

        uint32_t err;
        port = VCS_OpenDevice(DEV_NAME, PROTO_NAME, IF_NAME, PORT_NAME, &err);
        if (!port) {
                die(err, "failed to open port");
        }

        printf("port opened: handle=0x%X\n", port);
}

void port_close(void)
{
        printf("closing port 0x%p...\n", port);

        uint32_t err;
        if (!VCS_CloseDevice(port, &err)) {
                die(err, "failed to close port");
        }
}

void port_configure(void)
{
        printf("setting baudrate to %dkbits/s and timeout to %ums...\n",
                        BAUDRATE / 1000, TIMEOUT);

        uint32_t err;
        if (!VCS_SetProtocolStackSettings(port, BAUDRATE, TIMEOUT, &err)) {
                die(err, "failed to set port settings");
        }
}

void node_reset(uint16_t node_id)
{
        printf("resetting node %u...\n", node_id);

	if (node_id == 0xFFFF) {
		printf("invalid node id: %u\n", node_id);
		return;
	}

        uint32_t err;
        int32_t is_fault;
        if (!VCS_GetFaultState(port, node_id, &is_fault, &err)) {
                die(err, "failed to get fault state");
        }

        if (is_fault) {
                // clear fault
                printf("fault state detected - clearing...\n");
                if (!VCS_ClearFault(port, node_id, &err)) {
                        die(err, "failed to get fault state");
                }
        }

	sleep(1);

        int32_t is_enabled;
        if (!VCS_GetEnableState(port, node_id, &is_enabled, &err)) {
                die(err, "failed to get enable state");
        }

        if (!is_enabled) {
                // enable device
                printf("device not enabled - enabling now...\n");
                if (!VCS_SetEnableState(port, node_id, &err)) {
                        die(err, "failed to set enable state");
                }

        }

	printf("enabled device\n");
}

void node_configure(uint16_t node_id)
{
        printf("configuring node %u...\n", node_id);

	if (node_id == 0xFFFF) {
		printf("invalid node id: %u\n", node_id);
		return;
	}

	// don't need to do this
	/*
        printf("configuring motor parameters...\n");

        uint32_t err;
        uint32_t bytes_written;
        if (!VCS_SetMotorType(port, node_id, MOTOR_TYPE, &err) ||
                        !VCS_SetDcMotorParameterEx(port, node_id, NOMINAL_CURRENT,
                                OUTPUT_CURRENT_LIMIT, THERMAL_TIME_CONSTANT, &err)) {
                die(err, "failed to configure motor");
        }

        set(&COB_ID_TORQUE_CONSTANT, node_id, &TORQUE_CONSTANT, sizeof(TORQUE_CONSTANT));
        set(&COB_ID_MAX_MOTOR_SPEED, node_id, &MAX_MOTOR_SPEED, sizeof(MAX_MOTOR_SPEED));
        set(&COB_ID_MAX_GEAR_INPUT_SPEED, node_id, &MAX_GEAR_INPUT_SPEED, sizeof(MAX_GEAR_INPUT_SPEED));

        printf("configured motor with MOTOR_TYPE=%d, NOMINAL_CURRENT=%d, "
                        "OUTPUT_CURRENT_LIMIT=%d, THERMAL_TIME_CONSTANT_WINDING=%d, "
                        "NUMBER_OF_POLE_PAIRS=%d, MAX_MOTOR_SPEED=%d, "
                        "MAX_GEAR_INPUT_SPEED=%d\n", MOTOR_TYPE, NOMINAL_CURRENT,
                        OUTPUT_CURRENT_LIMIT, THERMAL_TIME_CONSTANT,
                        NUMBER_OF_POLE_PAIRS, MAX_MOTOR_SPEED, MAX_GEAR_INPUT_SPEED);

        if (MOTOR_TYPE == MT_EC_BLOCK_COMMUTATED_MOTOR || MOTOR_TYPE == MT_EC_SINUS_COMMUTATED_MOTOR) {
                // brushless DC (EC) motor for which the number of pole pairs
                // needs to be configured as well
                printf("using brushless DC motor - setting "
                                "NUMBER_OF_POLE_PAIRS=%d...\n", NUMBER_OF_POLE_PAIRS);
                set(&COB_ID_NUMBER_OF_POLE_PAIRS, node_id,
                                &NUMBER_OF_POLE_PAIRS, sizeof(NUMBER_OF_POLE_PAIRS));

        }
	*/

	printf("setting motion range of [%d, %d]...\n", POS_DEG_MIN, POS_DEG_MAX);
        set(&COB_ID_MIN_POS, node_id, &POS_DEG_MIN, sizeof(POS_DEG_MIN));
        set(&COB_ID_MAX_POS, node_id, &POS_DEG_MAX, sizeof(POS_DEG_MAX));
}


void can_read_loop(const char *port_name)
{
        // https://github.com/craigpeacock/CAN-Examples/blob/master/canreceive.c

	// this will come in handy once additional data not available through
	// the EPOS library will need to be read

        int sock;
        if ((sock = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
                die(0, "failed to create CAN socket");
        }

        struct ifreq ifr;
        strcpy(ifr.ifr_name, port_name);
        ioctl(sock, SIOCGIFINDEX, &ifr);

        struct sockaddr_can addr = {
                .can_family = AF_CAN,
                .can_ifindex = ifr.ifr_ifindex
        };

        if (bind(sock, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
                die(0, "failed to bind CAN socket");
        }

        while (true) {
                struct can_frame frame;
                int nread = read(sock, &frame, sizeof(frame));
                if (nread < 0) {
                        die(0, "failed to read from CAN socket");
                }

                printf("0x%03X [%d] ", frame.can_id, frame.can_dlc);
                for (size_t i = 0; i < frame.can_dlc; i++) {
                        printf("%02X ", frame.data[i]);
                }

                printf("\n");
        }

}

void comm_start(void)
{
        printf("starting communications...\n");

	int client_fd;
	if ((client_fd = socket(AF_INET, SOCK_STREAM, 0)) == -1) {
		die(0, "failed to create client_fd");
	}

	printf("created socket with client_fd = %d\n", client_fd);

	struct sockaddr_in serv_addr = {
		.sin_family = AF_INET,
		.sin_port = htons(SERVER_PORT)
	};

	if (inet_pton(AF_INET, SERVER_IP, &serv_addr.sin_addr) <= 0) {
		die(0, "failed to populate servr_addr struct");
	}

	if (connect(client_fd, (struct sockaddr*)&serv_addr, sizeof(serv_addr)) == -1) {
		die(0, "failed to connect to server");
	}

	printf("trying to connect to %s:%u...\n", SERVER_IP, SERVER_PORT);
	printf("connected to server with fd %d\n", client_fd);

	comm_loop_enter(client_fd);
}

void comm_loop_enter(int fd)
{
	printf("entering command loop...\n");
        FILE *stream = fdopen(fd, "r"); // makes it easier to scan lines

        char linebuf[MAX_STR_SIZE];
        while (true) {
                if (fgets(linebuf, MAX_STR_SIZE, stream) == NULL) {
                        fprintf(stderr, "disconnected\n");
                        exit(1);
                }

                size_t len = strlen(linebuf);
                if (len < MAX_STR_SIZE && len > 0) {
                        linebuf[len - 1] = '\0'; // replace newline
                }

                command_process((const char*)linebuf);
        }
}

void command_process(const char *cmd)
{
        printf("processing command '%s'...\n", cmd);

        double d;
        char s[MAX_STR_SIZE];
        if (strcmp(cmd, "position home") == 0) {
                // home position
                printf("moving to home position\n");
        } else if (sscanf(cmd, "position absolute %s %lf", s, &d) == 2) {
                // absolute position
                printf("%s axis: moving to absolute position %lf deg\n", s, d);
                motion_position(s, true, d);
        } else if (sscanf(cmd, "position relative %s %lf", s, &d) == 2) {
                // relative position
                printf("%s axis: moving to relative position %lf deg\n", s, d);
                motion_position(s, false, d);
        } else if (sscanf(cmd, "velocity %s %lf", s, &d) == 2) {
                // velocity
                printf("%s axis: moving with velocity %lf rpm\n", s, d);
                motion_velocity(s, d);
        } else if (strcmp(cmd, "halt") == 0) {
		// halt
		printf("halting movement for all axes\n");
		motion_halt();
	} else {
                printf("invalid command: '%s'\n", cmd);
        }
}

void int_handler(int signo)
{
	exit_gracefully();
}

void exit_gracefully(void)
{
	printf("shutting down...\n");

	motion_halt();

        uint32_t err;
        if (!VCS_SetDisableState(port, NODE_ID_YAW, &err)
          || VCS_SetDisableState(port, NODE_ID_PITCH, &err)
          || VCS_SetDisableState(port, NODE_ID_ROLL, &err)) {
        	die(err, "failed to set disable state");
        }

	printf("state set to disabled\n");

	port_close();
	printf("port closed\n");

	exit(1);
}

// motion -------------------------------------------------------------------------------
void motion_halt(void)
{
	uint32_t err;
	switch (mode_last) {
		case OMD_PROFILE_POSITION_MODE:
			if (!VCS_HaltPositionMovement(port, NODE_ID_YAW, &err) ||
			    !VCS_HaltPositionMovement(port, NODE_ID_PITCH, &err) ||
			    !VCS_HaltPositionMovement(port, NODE_ID_ROLL, &err)) {
				die(err, "failed to halt profile position movement for one or more axes");
			}
			break;

		case OMD_PROFILE_VELOCITY_MODE:
			if (!VCS_HaltVelocityMovement(port, NODE_ID_YAW, &err) ||
			    !VCS_HaltVelocityMovement(port, NODE_ID_PITCH, &err) ||
			    !VCS_HaltVelocityMovement(port, NODE_ID_ROLL, &err)) {
				die(err, "failed to halt profile velocity movement for one or more axes");
			}
			break;
	}
}

void motion_position(const char *axis, bool absolute, double deg)
{
	// TODO better safety (also use VCS_SetMaxAcceleration()...)

	// TODO move this up into caller
        uint16_t node_id = axis_to_node_id(axis);
        if (node_id == 0xFFFF) {
                printf("invalid node id: '%s'\n", axis);
                return;
        }

	printf("moving to new position %lf\n", deg);

        uint32_t err;
	// activate mode only if necessary
	mode_last = OMD_PROFILE_POSITION_MODE;

	if (!VCS_ActivateProfilePositionMode(port, node_id, &err)) {
		die(err, "failed to activate profile position mode");
	}

        // TODO move into node_configure()
        if (!VCS_SetPositionProfile(port, node_id, PPM_MAX_VELOCITY, 1000, 1000, &err)) {
                die(err, "failed to set position profile");
        }

        if (!VCS_MoveToPosition(port, node_id, deg * DEGTOINC, absolute, true, &err)) {
                die(err, "failed to move to position");
        }
}

void motion_velocity(const char *axis, double rpm)
{
        uint16_t node_id = axis_to_node_id(axis);
        if (node_id == 0xFFFF) {
                printf("invalid node id: '%s'\n", axis);
                return;
        }

        uint32_t err;
	// activate mode only if necessary
	mode_last = OMD_PROFILE_VELOCITY_MODE;

	if (!VCS_ActivateProfileVelocityMode(port, node_id, &err)) {
		die(err, "failed to activate profile velocity mode");
	}

        if (!VCS_SetVelocityProfile(port, node_id, 1000, 1000, &err)) {
                die(err, "failed to set velocity profile");
        }

        if (!VCS_MoveWithVelocity(port, node_id, rpm * RPMTOVEL, &err)) {
                die(err, "failed to move with velocity");
        }

        if (!VCS_HaltVelocityMovement(port, node_id, &err)) {
                die(err, "failed to halt velocity movement");
        }
}
