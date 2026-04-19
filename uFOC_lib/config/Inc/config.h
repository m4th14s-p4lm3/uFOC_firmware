#ifndef CONFIG_H
#define CONFIG_H

#include "pi_controller.h"
#include "encoder.h"

#define CAN_COMMUNICATION_DEVICE_ID 0x0 // Device ID for CAN daisy chaining
#define CAN_COMMUNICATION_STD_ID 0x123

#define MOTOR_MAGNETIC_PAIRS 11 
#define ENCODER_ELECTRICAL_OFFSET 118107
#define ENCODER_INVERT_DIR true 

#define V_BUS_NOMINAL 11.5f // power suply (V) / sqrt(3)


// Current loop PI regulator values
#define PI_CURRENT_KP 3.0f
#define PI_CURRENT_KI 2300.0f
#define PI_CURRENT_HARD_LIMIT 1.0f
#define PI_CURRENT_SOFT_LIMIT PI_CURRENT_HARD_LIMIT // Replace PI_CURRENT_HARD_LIMIT to set your own initial value


// Velocity loop PI regulator values
#define PI_VELOCITY_KP 0.00232f
#define PI_VELOCITY_KI 0.28f
#define PI_VELOCITY_HARD_LIMIT 1000.0f 
#define PI_VELOCITY_SOFT_LIMIT PI_VELOCITY_HARD_LIMIT // Replace PI_VELOCITY_HARD_LIMIT to set your own initial value


// Position loop PID regulator
#define PI_POSITION_KP 2000.0f
#define PI_POSITION_KI 0.0f
#define PI_POSITION_KD 7.0f


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
    enum AngleUnits units;

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

// void set_position_pid_constants()
// float get_position_pid_constants()



// Encoder
// Get electrical offset
// Set electrical offset
// Calibrate electrical offset
// Get current Position


// Get current Relative position
// Get current velocity

#endif