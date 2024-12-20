#include "constants.h"
#include "main.h"

#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <stdbool.h>
#include <string.h>
#include <errno.h>
#include <time.h>
#include <unistd.h>
#include <signal.h>
#include <math.h>
#include <pthread.h>

#include <arpa/inet.h>
#include <netinet/in.h>

#ifndef __USE_MISC
#define __USE_MISC 1
#include <net/if.h>
#undef __USE_MISC
#else
#include <net/if.h>
#endif

#include <sys/ioctl.h>
#include <sys/socket.h>

// SocketCAN
#include <linux/can.h>
#include <linux/can/raw.h>


void brakes(bool);
void positions();

void cq(){
        while (true)
        {
                char input[128];
                scanf("%s", input);
                uint16_t axis;
                switch (input[0])
                {
                case 'y':
                        axis = NODE_ID_YAW;
                        break;
                case 'r':
                        axis = NODE_ID_ROLL;
                        break;
                case 'p':
                        axis = NODE_ID_PITCH;
                        break;
                case 'b':
                        brakes(true);
                        continue;
                case 'm':
                        brakes(false);
                        continue;
                case 'c':
                        positions();
                        continue;
                default:
                        continue;
                }

                if (input[1] == 'v'){
                        double rpm = atof(&input[2]);
                        if (fabs(rpm) > 5)
                                rpm = 5.0;
                        motion_velocity(axis, rpm);
                } else if (input[1] == 'p'){
                        bool absolute = false;
                        if (input[2] == 'a'){
                                absolute = true;
                        } else if (input[2] == 'r'){
                                absolute = false;
                        } else {
                                continue;
                        }
                        
                        double angle = atof(&input[3]);
                        motion_position(axis, absolute, angle);
                } else {
                        continue;
                }
        }
        
}

int can_fd;

void setup_can(const char *port_name) {
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
        can_fd = sock;
}

struct position pos = {};

void positions() {
        printf("the current yaw is %f\n", pos.yaw);
        printf("the current pitch is %f\n", pos.pitch);
        printf("the current roll is %f\n", pos.roll);
}

void *position_loop(void *args){
        struct can_frame frame;
        float val;
        uint8_t *valptr;
        while (1)
        {
                read(can_fd, &frame, sizeof(frame));
                valptr = (uint8_t *) &val;
                valptr[0] = frame.data[3];
                valptr[1] = frame.data[2];
                valptr[2] = frame.data[1];
                valptr[3] = frame.data[0];
                switch (frame.can_id)
                {
                case 0x579:
                        pos.yaw = val;
                        break;
                case 0x580:
                        pos.pitch = val;
                        break;
                case 0x581:
                        pos.roll = val;
                        break;
                default:
                        break;
                }
        }
};

int main(int argc, char *argv[])
{
        driver_info_dump();

        //can_read_loop("can0");

	port_open();
        port_configure();
        setup_can("can0");
        positions();
        pthread_t position_thread;
        pthread_create(&position_thread, NULL, position_loop, NULL);

	//uint8_t sto_state;
	//get(&sto_state, sizeof(sto_state), NODE_ID_YAW, &COB_ID_STO_STATES);
	//printf("0x%X\n", sto_state);

        node_reset(NODE_ID_YAW);
        node_reset(NODE_ID_PITCH);
        node_reset(NODE_ID_ROLL);

        node_configure(NODE_ID_YAW);
        node_configure(NODE_ID_PITCH);
        node_configure(NODE_ID_ROLL);

        node_info_dump(NODE_ID_YAW);
        node_info_dump(NODE_ID_PITCH);
        node_info_dump(NODE_ID_ROLL);

        uint32_t err;
	int32_t position;
	if (!VCS_GetPositionIs(port, NODE_ID_ROLL, &position, &err)) {
		die(err, "failed to get position is");
	}
	printf("current position: %d\n", position);

        signal(SIGINT, int_handler);
        sleep(10);
        move_to_initial_position();
        cq();
        //comm_start();
	exit_gracefully();
        return 0;
}

// utility ------------------------------------------------------------------------------

void brakes(bool on){
        struct can_frame frame = {
                .can_id = 0x604,
                .can_dlc = 8,
                .data = {}
        };
        frame.data[4] = !on;
        
        write(can_fd, &frame, sizeof(frame));
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
                motion_position(axis_to_node_id(s), true, d);
        } else if (sscanf(cmd, "position relative %s %lf", s, &d) == 2) {
                // relative position
                printf("%s axis: moving to relative position %lf deg\n", s, d);
                motion_position(axis_to_node_id(s), false, d);
        } else if (sscanf(cmd, "velocity %s %lf", s, &d) == 2) {
                // velocity
                printf("%s axis: moving with velocity %lf rpm\n", s, d);
                motion_velocity(axis_to_node_id(s), d);
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
         | !VCS_SetDisableState(port, NODE_ID_PITCH, &err)
         | !VCS_SetDisableState(port, NODE_ID_ROLL, &err)) {
                port_close();
        	die(err, "failed to set disable state");
        }

	printf("state set to disabled\n");

	port_close();
	printf("port closed\n");

        brakes(true);
        close(can_fd);
	exit(1);
}

// motion -------------------------------------------------------------------------------