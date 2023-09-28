---
geometry: margin=30mm
---

# EPOS controller (`./epos-controller/`)

This piece of software represents the interface between the motion cueueing
program and the EPOS4 motor control devices. It is able to process various
simple commands over TCP/IP and is intended to run in the background as a
daemon. We chose TCP/IP as the interface to be independent of local IPC
mechanism and to allow distributed control of the simulator axes in case this
is desired in the future.

![EPOS4 with two connected CAN cables and STO connector](./res/epos_1.jpg){width=80%}

## Further reading (`./manuals/`)

| Name                                                                     | Description                                                               |
|--------------------------------------------------------------------------|---------------------------------------------------------------------------|
| [Application Notes](./manuals/EPOS4-Application-Notes-Collection-En.pdf) | Brief introductions to various topics and a general overview of procedure |
| [Firmware Specification](./manuals/EPOS4-Firmware-Specification-En.pdf)  | Firmware details like error codes, commands, modes and the state machine  |
| [Hardware Reference](./manuals/EPOS4-50-5-Hardware-Reference-En.pdf)     | Hardware details like pin assignments and wiring                          |
| [Command Library](./manuals/EPOS-Command-Library-En.pdf)                 | Command library documentation                                             |

## Building and dependencies

The EPOS Command Library header can be found in `./client/deps/` and serves as an
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
implements the network layer and above layers in the OSI model. CAN support for
the Raspberry Pi is achieved through a CAN head with two 3-pin connections
(H,L,G) sitting on top of it and kernel drivers exposing a CAN0 and CAN1
interface. This project uses a baud rate of 1000kbit/s and a timeout of 500ms,
so every interface and software port should be configured accordingly. CAN0 is
connected to CANOpen which is used for motion control and CAN1 to CANaerospace
which is used for instruments and control input. The driver used is
`CAN_mcp251x 0`. A script to set up an interface can be found in `./scripts/`.

Attention shold be given to proper wiring and a correct termination resistance
of 120 Ohms at the end of the bus. This can be realized by toggling DIP switch
7 to the ON position. DIP switches are also used for setting the id of each
node. Wiring errors usually manifest themselves in BUS-OFF errors and a
NO-CARRIER state on the interface. Interface state can be queried using `ip
-details link`.

![EPOS4 with DIP switches 2 (node id = 2) and 7 (120 Ohms termination resistance) set](./res/epos_0.jpg){width=80%}

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
connection allowed at one time. On startup, it connects to the RasCanServer via
the `SERVER_IP` and `PORT` specified in `./client/constants.h`. Available
commands are as follows:

| Command             | Arg 0     | Arg 1        | Description                                                |
|---------------------|-----------|--------------|------------------------------------------------------------|
| `halt`              | -         | _            | Halt movement for all axes                                 |
| `position home`     | -         | -            | Seek home position for all three axes (NOT YET IMPLEMENTED)|
| `position absolute` | axis (%s) | target (%lf) | Seek an absolute target position in degrees for given axis |
| `position absolute` | axis (%s) | target (%lf) | Seek a relative target position in degrees for given axis  |
| `velocity`          | axis (%s) | target (%lf) | Set velocity for given axis                                |

## TODO

- implement homing with the help of external position sensors
- find accurate values for DEGTOINC and RPMTOVEL
- deal with position wrapping: contrary to what one might expect, commanding the
  EPOS to move to position 360deg from position 0deg results in the EPOS doing a
  whole turn instead of doing nothing. This is a problem when position input
  like the heading wraps around and causes the respective axis to do a
  full-speed, 360deg turn
- make movement more smooth by using velocities instead of positions
- in case a text-based TCP protocol is too slow, replace by/supplement with binary UDP protocol
- add additional commands
- if you feel fancy, restructure the code into multiple files

# RasCanServer (`simulator-interface/RasCanServer`)

The RasCanServer is the central part mediating communications between all other
components. Currently its job includes passing data between the
SimulatorInterface on the Windows machine and the CANaerospace bus by means of
the `../Pi und Arduino/Pi/ThreadTest.py` script as well as intercepting
motion-relevant messages, calculating a motion profile and sending these
commands to the epos\_controller.

```
                       epos_controller
                              ^
       ThreadTest.py <-> RasCanServer <-> SimulatorInterface
```

## Build instructions

Since the project is written in C#, we use the dotnet SDK to build a runnable
package. The following commands can be used to create a bundle and upload it to
the Raspberry Pi using the `rsync` utility:

```
dotnet publish --runtime linux-arm --self-contained && \
rsync -avu bin/Debug/net6.0/linux-arm/ \
mrfas@137.248.121.40:/home/mrfas/rascanserver
```

## TODO

- proper motion cueing for a three-axes flight simulator should be implemented
  to accurately model forces experienced by the pilot mid-flight
- the program crashes in some cases (probably a race condition), this should be
  debugged and fixed

# SimulatorInterface (`./simulator-interface/SimulatorInterface/`)

SimulatorInterface is responsible for communicating with the flight simulator
through the ISimAdapter and for managing input and output variables as defined
in `./datamodel/SimvarsInput.xml` and `./datamodel/SimvarsOutput.xml`. On
startup, it connects to the RasCanServer to send and retrieve flight simulator
data. To ensure proper redirection of variables defined in
`./datamodel/SimvarsOutput.xml` to the CANaerospace bus, make sure to set
`<iCanId>` and also `<eTransmissionSlot>`, otherwise no data will be sent. For
reference on how to implement ISimAdapter and navigate this callback jungle,
refer to `./steering/SteeringAdapter.cs`.

## TODO

- move `useSimulatortype` and other variables into an external config file so
  one doesn't need to recompile everything

# Heading-Indicator and adjacent instruments

The Heading-Indicator case contains the Arduino. The case has two serial ports,
one for the CAN-Bus and the other one for the connection with four additional
instruments:

- Airspeed-Indicator
- Variometer
- Turn and Bank indicator (Turn and TurnBall), which has two separate
  connections for the Turn-indicator and the TurnBall.

![All five instruments are connected to the same
Arduino](./res/adjacent_instruments.jpg)

The Arduino receives the data from the Flight Simulator via the PI and
recalculates them to PWM values (0 to 255) for the Output-Pins on the Arduino.
For the Variometer, Airspeed indicator and Turn and Bank indicator the
recalculation is not straight forward, since they have a Hysteresis-Offset (see
example diagrams for Variometer-Climb rate and Airspeed):

![](./res/pwm_climbrate.png){width=50%}
![](./res/pwm_airspeed.png){width=50%}

This can be dealt with either linear interpolation or a modelling approach,
which incorporates for e.g., the Hysteresis-Offset.

The control for the Variometer, Airspeed Indicator and Turn and Bank indicator
is simply just done over the output pins on the Arduino. For the
Heading-Indicator a stepper-motor is included in the case, which controls the
movement of the Heading-Indicator disc. Additionally an encoder for a
push-button is included, which allows for e.g., to change the Heading-Indicator
position or reset it to a default value.

# CANaerospace Node-Service-protocol

CANaerospace allows the implementation of different services via CANAerospace.
Node-Services are divided into two categories:

- NSH (High Priority Node Service Data)
- NSL (Low Priority Node Service Data)

Both are differentiated through the Node Service Channel used. Channels from
0-35 (NSH) and 100-115 (NSL). One Channel corresponds to two Node-Service-IDs,
one for the request and one for the response, for e.g., Channel 0 has the Node
Service Request ID of 128 and a Node Service Response ID 129. The definition of
the Node Service IDs is part of the CANaerospace Identifier-Distribution (see
CANaerospace specification). Which services are implemented, and which service
uses which channels are user defined, the only service which has mandatory
implementation is the IDS-service.

## CANaerospace-frames for Node-Services

CANaerospace frame consists of 8 bytes:

![](./res/canas_frame.png)

- Node-ID: In case of Node-services the Node-ID will be the Node-ID of the Node
  which receives the request (Also 0 for broadcast possible).
- Datatype: The Datatype gives the type of data included in the message data
  part.
- Service-Code: Gives the used service (for service types see CANaerospace
  specification).
- Message-Code: Usage depended on specific service.

Note that the Node-ID, Service-Code and Message-Code definitions differ from
normal CANaerospace data operation ((NOD) Normal Operation Data), which is used
e.g., for sending data from the Flight-Simulator to the different instruments.
In general nodes should answer within 100ms to a Node-Service request, if not
otherwise defined in the CANaerospace specification.

## IDS-service (Identification service):

The IDS-service can be used as a sign-of-life indication between the
CANaerospace nodes and to check if all Nodes use the same CANAerospace
Identifier-Distribution/Header-Types. What happens e.g., for missing signs of
life or if different CANAersopace Identifer-Distributions are used is user
defined. The IDS service should be implemented on Channel 0. The request ID in
this case is 128 and the response ID is 129. For the specific frame definition
see page 14 of the CANAerospace specification.

## Other services:

Besides IDS CANaerospace allows other services defined in the specification or
user defined. The channels for these are also user defined, in general each
node should have its own Node Service Channel (this means a Node Service
Request Id and a Node Service Response Id). Specification defined services used
in this project are (besides IDS):

- (BSS) Baudrate Setting Service: For setting the Baudrate for the Node-Id for
  CAN-communication (see page 20 in the CANaerospace specification for the
  frame definition).
- Node-ID Setting Service (NIS): For setting the Node-Id of the node (see page
  21 in the CANaerospace specification for the frame definition).
- User defined Services: The Service codes from 100 to 255 can be user defined
  e.g., for sending calibration data.

## TODO:

- CANaerospace Node services:
    - The Node-services must be implemented on all CANaerospace nodes.
    - In case of the IDS-service, the logic of failure handling has to be
      implemented:
        - A missing sign-of-life of a node could produce a warning for the
          pilot.
        - Distribution-Identifier/Header-type synchronization in case differing
          settings are found in the nodes (There are already predefined
          Node-Services which can be used in this case).
    - User defined calibration services has to be implemented.
- A calibration application must be created, which allows the calibration of
  the different instruments.

# 3D parts and models

FreeCAD software was used to create CAD models. Each self-created file was
saved as the following 3 file types:

- `FCStd` (for editing in Freecad)
- `.step` (for production at Feinmechanikwerkstatt)
- `.stl` (for easy import and review in Blender etc.)

Self-created models can be produced via the precision mechanics workshop (FMW).
The FMW requires the respective model as ".step" file. Platform and plate for
PC & control cabinet are self-created CAD files, optimized and produced by the
FMW.

## SimulatorBackside

The file "SimulatorBackside" is an image of the cockpit backside under the seat
area with the protruding metal floor on which the platform and plate rest.
Relevant to draw the platform and plate to fit.

![](./res/simulator_backside.png){width=80%}

### Backside measurements

It is based on following measurements:

![](./res/measurement_0.jpeg){width=50%}
![](./res/measurement_1.jpeg){width=50%}

### Platform

The platform is a modified replica of the original metal plate. This way its
ensured to fit and cover our needs.

![](./res/platform_original.jpeg){width=80%}

It is attached to the protruding base plate of the flight simulator. At the 4
corners are columns, on which the upper plate for the PC and control cabinet is
attached. In addition, there is a square the fan attachment to the platform.

![](./res/platform_part.png){width=80%}

### Platform plate

The next part is the plate that is plugged on the platform on which the control
cabinet and PC sit. It has a total width of 355mm, and the separator wall is
500mm high. The short side is 120mm wide, with a rectangular recess which is
300mm long and 50mm wide. The recess is important for guiding the power cables
into the power box. The long side is 234mm wide. The PC is placed on this side.

![The plate to go on top of the platform](./res/platform_plate.png){width=80%}

![Dimensions of the switch cabinet.](./res/switch_cabinet.jpeg){width=80%}

### Blender file view

Blender file view – used to check if the models fit in the back. When importing
any file into Blender, make sure to edit the scale parameter to 0.01 to get
accurate representation.

![](./res/blender_0.png){width=50%}
![](./res/blender_1.png){width=50%}

![Current State – with all objects placed into the Simulator](./res/backside.jpg){width=80%}


## TODO

We do not have a Raspberry Pi case yet. The current idea is to make a minimal
case that can be plugged onto a pegboard to enable free dynamic placement.
Since the simulators pitch and roll axis are limited to 30-40 degrees of
movement, high case walls are not required. Just enough to keep the Pi in place
should be sufficient.

![Raspberry Pi 4B measurements](./res/raspi_dimensions.jpeg){width=50%}
![3D Model of Raspberry 4B (CAN Hat not shown!), Source: Andrä](./res/raspi_3d.png){width=50%}

We do not have a pegboard yet. The idea is to put it on the platform so that
devices can be plugged freely onto the surface under the area of the PC and
control cabinet.

Additionally, a pegboard friendly case for the network switch will be needed.
Depending on project demand, further models might be needed, but for now it is
all listed above.
