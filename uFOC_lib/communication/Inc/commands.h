#ifndef COMMANDS_H
#define COMMANDS_H

#include "commands.h"
#include "communication.h"

#define SET_CONTROL_STATE 0x00
#define SET_USER_ANGLE_UNITS 0x01
#define GET_USER_ANGLE_UNITS 0X02


// Current control
#define SET_TORQUE_CURRENT_TARGET 0x06
#define SET_TORQUE_CURRENT_SOFT_LIMIT 0x07
#define GET_TORQUE_CURRENT_SOFT_LIMIT 0x08

#define SET_CURRENT_KP 0x09 // United for TORQUE and FLUX currents control
#define SET_CURRENT_KI 0x0A // United for TORQUE and FLUX currents control


// Velocity control
#define SET_ANGULAR_VELOCITY_TARGET 0x10
#define SET_ANGULAR_VELOCITY_SOFT_LIMIT 0x11
#define GET_ANGULAR_VELOCITY_SOFT_LIMIT 0x12
#define GET_ANGULAR_VELOCITY 0x13

#define SET_ANGULAR_VELOCITY_KP 0x14
#define SET_ANGULAR_VELOCITY_KI 0x15


// Position control
#define SET_POSITION_TARGET 0x1A
#define GET_POSITION_TARGET 0x1B
#define GET_POSITION 0x1C
#define GET_RELATIVE_POSITION 0x1D

#define SET_POSITION_KP 0x1E
#define SET_POSITION_KI 0x1F
#define SET_POSITION_KD 0x20


// ENCODER
#define GET_ELECTRICAL_OFFSET 0x26
#define SET_ELECTRICAL_OFFSET 0x27
#define CALIBRATE_ELECTRICAL_OFFSET 0x28


void handle_communication(config_t* config);


#endif