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
