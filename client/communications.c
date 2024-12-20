#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include <unistd.h>
#include <string.h>

#include <sys/ioctl.h>
#include <sys/socket.h>

// SocketCAN
#include <linux/can.h>
#include <linux/can/raw.h>

#ifndef __USE_MISC
#define __USE_MISC 1
#include <net/if.h>
#undef __USE_MISC
#else
#include <net/if.h>
#endif

#include "constants.h"
#include "main.h"


void *port = NULL;

void port_open(void)
{
        printf("opening device '%s' using protocol '%s' on interface '%s' and"
                        " port '%s'...\n", DEV_NAME, PROTO_NAME, IF_NAME, PORT_NAME);

        uint32_t err;
        port = VCS_OpenDevice((char*) DEV_NAME, (char*) PROTO_NAME, (char*) IF_NAME, (char*) PORT_NAME, &err);
        if (!port) {
                die(err, "failed to open port");
        }

        printf("port opened: handle=0x%p\n", port);
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

        // TODO software position limits
	printf("setting motion range of [%d, %d]...\n", POS_DEG_MIN, POS_DEG_MAX);
        set(&COB_ID_MIN_POS, node_id, (void*)&POS_DEG_MIN, sizeof(POS_DEG_MIN));
        set(&COB_ID_MAX_POS, node_id, (void*)&POS_DEG_MAX, sizeof(POS_DEG_MAX));
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