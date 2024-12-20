#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>

extern void *port;

// utility ----------------------------------------------------------------------------------------
void set(const struct cob_id *id, uint16_t node_id, void *p, size_t n);
void get(void *buf, size_t n, uint16_t node_id, const struct cob_id *id);
void die(uint32_t err, const char *what, ...);

void print_velocity(uint16_t node_id);
void print_position(uint16_t node_id);

void driver_info_dump(void);
void node_info_dump(uint16_t node_id);
void str_motor_type(char *buf, size_t n, uint16_t motor_type);
uint16_t axis_to_node_id(const char *axis);

// communications ---------------------------------------------------------------------------------
void port_open(void);                  // open a handle to the CAN port for communication
void port_close(void);                 // close the handle
void port_configure(void);             // set port parameters (baud rate, timeout)
void node_reset(uint16_t node_id);     // clear fault state and enable node
void node_configure(uint16_t node_id); // set node parameters (range of motion, acceleration...)

void can_read_loop(const char *port_name); // read and process raw can frames from the port

// command processing -----------------------------------------------------------------------------
void comm_start(void);                 // connect to server
void comm_loop_enter(int fd);
void command_process(const char *cmd);

void int_handler(int signo);
void exit_gracefully(void);

// motion -----------------------------------------------------------------------------------------
void motion_halt(void);
void motion_position(uint16_t node_id, bool relative, double deg);
void motion_velocity(uint16_t node_id, double rpm);
void move_to_initial_position(void);

// new
struct position {
    float yaw;
    float pitch;
    float roll;
};

extern struct position pos;

void brakes(bool);
void positions();