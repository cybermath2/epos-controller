# EPOS controller

## About
This piece of software represents the interface between the motion cueueing
program and the EPOS4 motor control devices. It is able to process various
simple commands over TCP/IP and is intended to run in the background as a
daemon. We chose TCP/IP as the interface to be independent of local IPC
mechanism and to allow distributed control of the simulator axes in case this
is desired in the future.

## Building and dependencies

The EPOS Command Library header can be found in server/deps/ and serves as an
interface to its functions as well as a provider for constant definitions.
Dynamic libraries for ARMv{6,7,8} as well as i386 and amd64 for Linux to link
against can be found in lib/lib/. Please note these can be installed on the
system through the lib/install.sh script or by simply moving the dynamic
library files (libEposCmd.so.6.8.1.0, libftd2xx.so.1.4.8) into the
corresponding folder, usually /usr/local/lib/ on Linux systems.

The provided Makefile compiles the program and links it against the dynamic
libraries. The program can be built using `make` and the tree can be cleaned
using `make clean`.

## Technical background

We will give an overview of the main components and go further into detail on
potentially troublesome parts as well as on the communication protocol.

### CAN

All devices are connected together on a CAN bus and communicate through the
CANopen protocol. CAN resides on the physical and data link layer and CANopen
implements the network layer and above layers. CAN support for the Raspberry Pi
is achieved through a CAN head with two 3-pin connections (H,L,G) sitting on
top of it and kernel drivers exposing a CAN0 and CAN1 interface. This project
uses a baud rate of 1000kbit/s and a timeout of 500ms, so every interface and
software port should be configured accordingly. CAN0 is connected to CANOpen
which is used for motion control and CAN1 to CANAerospace which is used for
instruments and control input. The driver used is `CAN_mcp251x 0`. A script to
set up an interface can be found in scripts/.

Attention shold be given to proper wiring and a correct termination resistance
of 120 Ohms at the end of the bus. This can be realized by toggling DIP switch
7 to the ON position. Wiring errors usually manifest themselves in BUS-OFF
errors and a NO-CARRIER state on the interface. Interface state can be queried
using `ip -details link`.

Useful tools for debugging a CAN bus can be found in
[linux-can/can-utils](https://github.com/linux-can/can-utils) and include
`candump` for dumping CAN packages (useful to check whether a bus is dead as
devices shold be configured to send a periodic heartbeat message), `cangen` for
generation random traffic and `cansend` for sending a single CAN frame.

### EPOS4

Each of the three axes (yaw, pitch, roll) has one motor responsible for turning
it. This motor is controlled by an EPOS4 which has various control parameters
stored in non-volatile memory and works using three basic modes of operation:

1. position: move to an absolute or relative position
2. velocity: move with a target velocity
3. acceleration: apply a fixed acceleration to a motor

The user first initializes the EPOS4 by clearing a potential fault state and
enabling the device if it is currently disabled. Then the mode of operation is
set as well as profile parameters like acceleration and deceleration, maximum
velocity and so on. Only then can the device receive commands, compute a
trajectory and execute it with the help of various feedback loops from internal
sensors. A 5 second long move with a fixed velocity target for example could be
done in the following way:

1. get handle to CAN interface (hereinafter referred to by "port"), set
   baudrate and timeout
2. device in fault state? then clear fault
3. device disabled? then enable
4. set operation mode velocity
5. set velocity profile parameters (e.g. acceleration = deceleration =
   10'000rpm/s)
6. set target velocity (e.g. 250rpm)
7. sleep(5)
8. halt movement
9. disable

Controllers and torque are only activated when the device is in the enabled
state. This means that while the device is enabled, it cannot be moved by hand
and doesn't require an outside braking force to hold its position.

The author of this document can in no way be held responsible for any material
or bodily harm incurred by improper use and/or faulty setups.

## Command set

This program receives and processes commands over TCP/IP with only one active
connection allowed at one time. Available commands are as follows:

| Command             | Arg 0     | Arg 1        | Description                                                |
|---------------------|-----------|--------------|------------------------------------------------------------|
| `position home`     | -         | -            | Seek home position for all three axes                      |
| `position absolute` | axis (%s) | target (%lf) | Seek an absolute target position in degrees for given axis |
| `position absolute` | axis (%s) | target (%lf) | Seek a relative target position in degrees for given axis  |
| `velocity`          | axis (%s) | target (%lf) | Set velocity for given axis                                |
