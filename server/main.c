#include "constants.h"

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <netinet/in.h>

// utility ------------------------------------------------------------------------------
void set(const struct cob_id *id, void *port, uint16_t node_id, void *p, size_t n);
void get(void *buf, size_t n, void *port, uint16_t node_id, const struct cob_id *id);
void die(uint32_t err, const char *what, ...);

void driver_info_dump(void);
void node_info_dump(void *port, uint16_t node_id);
void str_motor_type(char *buf, size_t n, uint16_t motor_type);

void str_unit_dim(char *buf, size_t n, uint8_t dimension);
void str_unit_not(char *buf, size_t n, uint8_t notation);
void str_unit(char *buf, size_t n, uint32_t unit);

// communications ------------------------------------------------------------------------

// Open a communication port to CAN bus.
void *port_open(void);

// Close the communication port handle.
void port_close(void *port);

// Set port settings like baudrate and timeout.
void port_configure(void *port);

// Reset node, i.e. change NMT state to pre-operational
// (see https://www.can-cia.org/can-knowledge/canopen/network-management/ for
// details).
void node_reset(void *port, uint16_t node_id);

// Configure node parameters by writing parameters to object dictionary.
// Parameters configured are mode-independent parameters like motor type and
// number of pole pairs and sensor configuration.
void node_configure(void *port, uint16_t node_id);

// Test a node by entering profile velocity mode (PVM) and setting the target
// velocity to 1rpm.
void node_test_1rpm(void *port, uint16_t node_id);

// start listening for and accepting incoming TCP connections and forwarding
// commands to the EPOS
void comm_start(void);

// receive commands for connection until closed
void comm_loop_enter(int server_fd, int client_fd);

int main(int argc, char *argv[])
{
        driver_info_dump();

        void *port = port_open();
        port_configure(port);

        node_reset(port, 0);
        node_info_dump(port, 0);
        // not needed since all parameters are stored in non-volatile memory
        // node_configure(port, node_id);

        /* node_test_1rpm(port, node_id); */
        /* comm_start(); */

        port_close(port);
        return 0;
}

void *port_open(void)
{
        printf("opening port '%s' using protocol '%s' on interface '%s' and"
               " port '%s'...\n", DEV_NAME, PROTO_NAME, IF_NAME, PORT_NAME);

        uint32_t err;
        void *port = VCS_OpenDevice(DEV_NAME, PROTO_NAME, IF_NAME, PORT_NAME, &err);
        if (!port) {
                die(err, "failed to open port");
        }

        printf("|-> port opened: handle=0x%p\n", port);
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
        if (!VCS_SendNMTService(port, node_id, NCS_RESET_NODE, &err)) {
                die(err, "failed to reset node");
        }
}

void node_configure(void *port, uint16_t node_id)
{
        printf("configuring node %u...\n", node_id);

        printf("|-> configuring motor parameters...\n");

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

        printf("|-> configured motor with MOTOR_TYPE=%d, NOMINAL_CURRENT=%d, "
               "OUTPUT_CURRENT_LIMIT=%d, THERMAL_TIME_CONSTANT_WINDING=%d, "
               "NUMBER_OF_POLE_PAIRS=%d, MAX_MOTOR_SPEED=%d, "
               "MAX_GEAR_INPUT_SPEED=%d\n", MOTOR_TYPE, NOMINAL_CURRENT,
               OUTPUT_CURRENT_LIMIT, THERMAL_TIME_CONSTANT,
               NUMBER_OF_POLE_PAIRS, MAX_MOTOR_SPEED, MAX_GEAR_INPUT_SPEED);

        if (MOTOR_TYPE == MT_EC_BLOCK_COMMUTATED_MOTOR || MOTOR_TYPE == MT_EC_SINUS_COMMUTATED_MOTOR) {
                // brushless DC (EC) motor for which the number of pole pairs
                // needs to be configured as well
                printf("|-> using brushless DC motor - setting "
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

void comm_loop_enter(int server_fd, int client_fd)
{
        uint8_t buf[NET_BUF_SIZE];
        ssize_t nread = 1;
        while (nread != 0 && nread != -1) {
                nread = read(client_fd, buf, sizeof(buf));
                printf("received %ld bytes: [", nread);
                for (size_t i = 0; i < nread; i++) {
                        if (i > 0)
                                printf(", ");
                        printf("0x%x", buf[i]);
                }
                printf("]\n");
        }
}

void comm_start(void)
{
        printf("entering communication loop...\n");

        int server_fd = socket(AF_INET, SOCK_STREAM, 0);
        if (server_fd == -1) {
                die(0, "failed to instantiate sockfd");
        }

        printf("|-> created socket with sockfd=%d\n", server_fd);

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
                printf("listening for new connection...\n");
                int client_fd = accept(server_fd, (struct sockaddr*)&client_addr, &client_addr_len);
                if (client_fd == -1) {
                        die(0, "failed to accept connection");
                }

                printf("accepted new connection with client_fd=%d\n", client_fd);
                comm_loop_enter(server_fd, client_fd);
        }
}
