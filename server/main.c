#include "constants.h"

#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <stdbool.h>
#include <string.h>
#include <errno.h>
#include <time.h>
#include <unistd.h>

#include <net/if.h>
#include <netinet/in.h>

#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/socket.h>

// SocketCAN
#include <linux/can.h>
#include <linux/can/raw.h>

// utility -------------------------------------------------------------------------------
void set(const struct cob_id *id, void *port, uint16_t node_id, void *p, size_t n);
void get(void *buf, size_t n, void *port, uint16_t node_id, const struct cob_id *id);
void die(uint32_t err, const char *what, ...);

void print_velocity(void *port, uint16_t node_id);
void print_position(void *port, uint16_t node_id);

void driver_info_dump(void);
void node_info_dump(void *port, uint16_t node_id);
void str_motor_type(char *buf, size_t n, uint16_t motor_type);
int axis_stoi(const char *axis);

// communications ------------------------------------------------------------------------
void *port_open(void);
void port_close(void *port);
void port_configure(void *port);
void node_reset(void *port, uint16_t node_id);
void node_configure(void *port, uint16_t node_id);

void can_read_loop(const char *port_name);

// command processing -------------------------------------------------------------------
void comm_start(void *port);
void comm_loop_enter(void *port, int server_fd, int client_fd);
void command_process(void *port, const char *cmd);

// motion -------------------------------------------------------------------------------
void motion_position(void *port, const char *axis, bool relative, double deg);
void motion_velocity(void *port, const char *axis, double rpm);
void move_to_initial_position(void *port);
void node_test_position_relative(void *port, uint16_t node_id);

int main(int argc, char *argv[])
{
        /* driver_info_dump(); */
        //can_read_loop(PORT_NAME_SOCK);

        void *port = port_open();
        /* port_configure(port); */

        node_reset(port, NODE_ID_YAW);
        node_info_dump(port, NODE_ID_YAW);

        int err;
        if (!VCS_SetDisableState(port, NODE_ID_YAW, &err)) {
        	die(err, "failed to set disable state");
        }

        // not needed since all parameters are stored in non-volatile memory
        // node_configure(port, node_id);

        comm_start(port);

        port_close(port);
        return 0;
}

// utility ------------------------------------------------------------------------------
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
        fprintf(stderr, "\033[1;31m");
        vfprintf(stderr, what, args);
        fprintf(stderr, " (error code 0x%x)", err);
        va_end(args);

        if (err != 0) {
                char err_info[MAX_STR_SIZE];
                if (VCS_GetErrorInfo(err, err_info, MAX_STR_SIZE)) {
                        fprintf(stderr, ": %s", err_info);
                }
        }

        fprintf(stderr, "\n\033[1;0m");
        exit(1);
}

void print_velocity(void *port, uint16_t node_id)
{
        int32_t velocity;
        uint32_t err;
        if (!VCS_GetVelocityIs(port, node_id, &velocity, &err)) {
                die(err, "failed to get velocity is");
        }

        printf("current velocity: %d\n", velocity);
}

void print_position(void *port, uint16_t node_id)
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

void node_info_dump(void *port, uint16_t node_id)
{
        printf("getting node information...\n");

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
                get(&number_of_pole_pairs, sizeof(number_of_pole_pairs), port,
                                node_id, &COB_ID_NUMBER_OF_POLE_PAIRS);
                printf("number of pole pairs=%d\n", number_of_pole_pairs);
        }

        uint32_t torque_constant;
        uint32_t max_motor_speed;
        uint32_t max_gear_input_speed;
        get(&torque_constant, sizeof(torque_constant), port, node_id, &COB_ID_TORQUE_CONSTANT);
        get(&max_motor_speed, sizeof(max_motor_speed), port, node_id, &COB_ID_MAX_MOTOR_SPEED);
        get(&max_gear_input_speed, sizeof(max_gear_input_speed), port, node_id, &COB_ID_MAX_GEAR_INPUT_SPEED);

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

int axis_stoi(const char *axis)
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

void *port_open(void)
{
        printf("opening device '%s' using protocol '%s' on interface '%s' and"
                        " port '%s'...\n", DEV_NAME, PROTO_NAME, IF_NAME, PORT_NAME);

        uint32_t err;
        void *port = VCS_OpenDevice(DEV_NAME, PROTO_NAME, IF_NAME, PORT_NAME, &err);
        if (!port) {
                die(err, "failed to open port");
        }

        printf("port opened: handle=0x%p\n", port);
        return port;
}

void port_close(void *port)
{
        printf("closing port 0x%p...\n", port);

        uint32_t err;
        if (!VCS_CloseDevice(port, &err)) {
                die(err, "failed to close port");
        }
}

void port_configure(void *port)
{
        printf("setting baudrate to %dkbits/s and timeout to %ums...\n",
                        BAUDRATE / 1000, TIMEOUT);

        uint32_t err;
        if (!VCS_SetProtocolStackSettings(port, BAUDRATE, TIMEOUT, &err)) {
                die(err, "failed to set port settings");
        }
}

void node_reset(void *port, uint16_t node_id)
{
        printf("resetting node %u...\n", node_id);

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
}

void node_configure(void *port, uint16_t node_id)
{
        printf("configuring node %u...\n", node_id);

        printf("configuring motor parameters...\n");

        uint32_t err;
        uint32_t bytes_written;
        if (!VCS_SetMotorType(port, node_id, MOTOR_TYPE, &err) ||
                        !VCS_SetDcMotorParameterEx(port, node_id, NOMINAL_CURRENT,
                                OUTPUT_CURRENT_LIMIT, THERMAL_TIME_CONSTANT, &err)) {
                die(err, "failed to configure motor");
        }

        set(&COB_ID_TORQUE_CONSTANT, port, node_id, &TORQUE_CONSTANT, sizeof(TORQUE_CONSTANT));
        set(&COB_ID_MAX_MOTOR_SPEED, port, node_id, &MAX_MOTOR_SPEED, sizeof(MAX_MOTOR_SPEED));
        set(&COB_ID_MAX_GEAR_INPUT_SPEED, port, node_id, &MAX_GEAR_INPUT_SPEED, sizeof(MAX_GEAR_INPUT_SPEED));

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
                set(&COB_ID_NUMBER_OF_POLE_PAIRS, port, node_id,
                                &NUMBER_OF_POLE_PAIRS, sizeof(NUMBER_OF_POLE_PAIRS));

        }

        uint16_t data;
        uint32_t bytes_read;
        if (!VCS_GetObject(port, node_id, 0x2200, 0x1, &data, sizeof(data),
                                &bytes_read, &err)) {
                die(err, "failed to get position must");
        }

        printf("%u\n", data);
}


void can_read_loop(const char *port_name)
{
        // https://github.com/craigpeacock/CAN-Examples/blob/master/canreceive.c

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

void node_test_1rpm(void *port, uint16_t node_id)
{
        uint32_t err;
        if (!VCS_ActivateVelocityMode(port, node_id, &err)) {
                die(err, "failed to set operational mode to PVM");
        }

        // 10'000rpm/s
        if (!VCS_SetVelocityProfile(port, node_id, 1, 1, &err)) {
                die(err, "failed to set velocity profile");
        }

        // move with 1rpm
        if (!VCS_MoveWithVelocity(port, node_id, 1, &err)) {
                die(err, "failed to move with target velocity");
        }
}

void comm_start(void *port)
{
        printf("entering communication loop...\n");

        int server_fd = socket(AF_INET, SOCK_STREAM, 0);
        if (server_fd == -1) {
                die(0, "failed to instantiate sockfd");
        }

        printf("created socket with sockfd=%d\n", server_fd);

        const int enable = 1;
        if (setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT, &enable, sizeof(enable)) == -1) {
                die(0, "failed to set socket options");
        }

        const struct sockaddr_in sockaddr = {
                .sin_addr.s_addr = INADDR_ANY,
                .sin_family = AF_INET,
                .sin_port = htons(RECV_PORT),
        };

        if (bind(server_fd, (const struct sockaddr*)&sockaddr, sizeof(sockaddr)) == -1) {
                die(0, "failed to bind socket");
        }

        if (listen(server_fd, 1) == -1) {
                die(0, "failed to put socket in listen mode");
        }

        while (1) {
                struct sockaddr_in client_addr;
                socklen_t client_addr_len;
                printf("listening for new connection on port %hu...\n", RECV_PORT);
                int client_fd = accept(server_fd, (struct sockaddr*)&client_addr, &client_addr_len);
                if (client_fd == -1) {
                        die(0, "failed to accept connection");
                }

                printf("accepted new connection with client_fd=%d\n", client_fd);
                comm_loop_enter(port, server_fd, client_fd);
        }
}

void comm_loop_enter(void *port, int server_fd, int client_fd)
{
        printf("entering command loop...\n");

        uint8_t buf[NET_BUF_SIZE];
        size_t buflen = 0;
        while (true) {
                if (buflen >= sizeof(buf)) {
                        // buf is full, empty it
                        buflen = 0;
                }

                ssize_t nread = read(client_fd, buf + buflen, sizeof(buf) - buflen);
                if (nread == 0) {
                        printf("connection closed by peer\n");
                        break;
                } else if (nread == -1) {
                        printf("error detected: %d (%s)\n", errno, strerror(errno));
                        break;
                }

                char *end = strchr(buf, '\n');
                if (end != NULL) {
                        // newline in buf - split into two and process first part
                        *end = '\0';
                        command_process(port, buf);

                        // copy remaining buffer into front part and update buflen
                        size_t cmd_len = strlen(buf) + 1; // null byte
                        strcpy(buf, end + 1);
                        buflen -= cmd_len;
                } else {
                        buflen += nread;
                }
        }
}

void command_process(void *port, const char *cmd)
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
                motion_position(port, s, false, d);
        } else if (sscanf(cmd, "position relative %s %lf", s, &d) == 2) {
                // relative position
                printf("%s axis: moving to relative position %lf deg\n", s, d);
                motion_position(port, s, true, d);
        } else if (sscanf(cmd, "velocity %s %lf", s, &d) == 2) {
                // velocity
                printf("%s axis: moving with velocity %lf rpm\n", s, d);
                motion_velocity(port, s, d);
        } else {
                printf("invalid command: '%s'\n", cmd);
        }
}

// motion -------------------------------------------------------------------------------
void motion_position(void *port, const char *axis, bool relative, double deg)
{
        int node_id = axis_stoi(axis);
        if (node_id == -1) {
                printf("invalid axis: '%s'\n", axis);
                return;
        }

        uint32_t err;
        if (!VCS_ActivateProfilePositionMode(port, node_id, &err)) {
                die(err, "failed to activate profile position mode");
        }

        if (!VCS_SetPositionProfile(port, node_id, PPM_MAX_VELOCITY, 1000, 1000, &err)) {
                die(err, "failed to set position profile");
        }

        if (!VCS_MoveToPosition(port, node_id, deg * DEGTOINC, relative, true, &err)) {
                die(err, "failed to move to position");
        }
}

void motion_velocity(void *port, const char *axis, double rpm)
{
        int node_id = axis_stoi(axis);
        if (node_id == -1) {
                printf("invalid axis: '%s'\n", axis);
                return;
        }

        uint32_t err;
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

void node_test_position_relative(void *port, uint16_t node_id)
{
        struct timespec ts = {
                .tv_sec = 0,
                .tv_nsec = 100000000
        };

        for (int i = 0; i < 50; i++) {
                print_velocity(port, node_id);
                nanosleep(&ts, NULL);
        }
}
