#ifndef CONFIG_H
#define CONFIG_H

#include "pi_controller.h"
#include "encoder.h"

// NOTE: While user/client can set user angle units to radians, degress or rotations
// all internal angle units are in rotations so all HARD LIMITS and PID regulator 
// constants must count with rotation units
// Time units are always seconds so default velocity is in rotations per second
// Currents units are always ampers


#define CAN_COMMUNICATION_DEVICE_ID 0x0 // Device ID for CAN daisy chaining
#define CAN_COMMUNICATION_STD_ID 0x123

#define DEFAULT_USER_UNITS ROTATIONS // Choose from enum AngleUnits
// #define DEFAULT_CONTROL_STATE_AFTER_INIT POSITION_CONTROL // Choose from enum ControlState


#define MOTOR_MAGNETIC_PAIRS 11 
#define ENCODER_ELECTRICAL_OFFSET 23562 //17061
#define ENCODER_INVERT_DIR true 

#define V_BUS_NOMINAL 11.5f // Power supply (V) / sqrt(3)

// Current loop PI regulator values
#define PI_CURRENT_KP 3.0f
#define PI_CURRENT_KI 500.0f
#define PI_CURRENT_HARD_LIMIT 1.0f // Ampers
#define PI_CURRENT_SOFT_LIMIT PI_CURRENT_HARD_LIMIT // Replace PI_CURRENT_HARD_LIMIT to set your own initial value


// Velocity loop PI regulator values
#define PI_VELOCITY_KP 0.00232f * 60.0f
#define PI_VELOCITY_KI 0.28f * 60.0f
#define PI_VELOCITY_HARD_LIMIT 1000.0f / 60.0f // THIS VALUE MUST BE IN ROTATIONS PER SECOND 
#define PI_VELOCITY_SOFT_LIMIT PI_VELOCITY_HARD_LIMIT // Replace PI_VELOCITY_HARD_LIMIT to set your own initial value


// Position loop PID regulator
#define PI_POSITION_KP 33.0f
#define PI_POSITION_KI 0.0f * 60.0f
#define PI_POSITION_KD 0.12f


typedef struct {
    PIController pi_d_current_axis;
    PIController pi_q_current_axis;
    PIController pi_angular_velocity;
    PIDController pid_position;
} regulators_t;

typedef enum ControlState {
    // This enum is purposely sorted in this order
    // If you chooze one state, all the previous states are (and must be) also enabled
    NO_CONTROL,
    CURRENT_CONTROL,
    VELOCITY_CONTROL,
    POSITION_CONTROL
};

typedef enum AngleUnits {
    RADIANS,
    ROTATIONS,
    DEGREES
};

typedef struct{
    regulators_t regulators;
    // Current
    float torque_current_target;
    float torque_current_soft_limit;
    
    float flux_current_target;
    float flux_current_soft_limit;

    // Angular velocity
    float angular_velocity_target;
    float angular_velocity_soft_limit;
    
    // Position
    float position_target;

    enum ControlState control_state;
    enum AngleUnits user_angle_units;

    encoder_t encoder;

} config_t;


// Utility function
static inline float clampf(float x, float lo, float hi)
{
    if (x < lo) {
        return lo;
    }
    if (x > hi) {
        return hi;
    }
    return x;
}


config_t init_config();


void set_control_state(config_t* config, enum ControlState new_control_state);

// CURRENT SETTERS/GETTERS
void set_torque_current_target(config_t* config, float new_torque_current_target);
// void get_torque_current_target(config_t* config);
void set_torque_current_soft_limit(config_t* config, float new_torque_current_soft_limit);
float get_torque_current_soft_limit(config_t* config);
void set_current_kp(config_t* config, float new_kp);
void set_current_ki(config_t* config, float new_ki);


// ANGULAR VELOCITY SETTERS/GETTERS
void set_angular_velocity_target(config_t* config, float new_angular_velocity_target);
void set_angular_velocity_soft_limit(config_t* config, float new_angular_velocity_soft_limit);
float get_angular_velocity_soft_limit(config_t* config);
void set_angular_velocity_kp(config_t* config, float new_kp); 
void set_angular_velocity_ki(config_t* config, float new_ki); 


// POSITION SETTERS/GETTERS
void set_position_target(config_t* config, float new_position_target);
float get_position_target(config_t* config);
float get_position(config_t* config);

void set_position_kp(config_t* config, float new_kp);
void set_position_ki(config_t* config, float new_ki);
void set_position_kd(config_t* config, float new_kd);



// Encoder
float get_electrical_offset(config_t* config);
float set_electrical_offset(config_t* config, float new_electrical_offset);





// Unit conversion utils
void set_user_angle_units(config_t* config, enum AngleUnits new_user_angle_units);
enum AngleUnits get_user_angle_units(config_t* config);



float convert_rotations_to_user_units(config_t* config, float value_to_convert);
float rotations_to_degrees(float rotations_value);
float rotations_to_radians(float rotations);


float convert_user_units_to_rotations(config_t* config, float value_to_convert);
float radians_to_rotations(float radian_value);
float degrees_to_rotations(float degrees_value);


#endif