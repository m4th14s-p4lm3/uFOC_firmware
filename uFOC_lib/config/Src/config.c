#include "config.h"
#include "pi_controller.h"
#include "driver.h"

config_t init_config(){
    config_t config = {
        .torque_current_target = 0.0f,
        .torque_current_soft_limit = PI_CURRENT_SOFT_LIMIT,
        .flux_current_target = 0.0f,
        .flux_current_soft_limit = PI_CURRENT_SOFT_LIMIT,


        .angular_velocity_target = 0.0f,
        .angular_velocity_soft_limit = PI_VELOCITY_SOFT_LIMIT,
        .position_target = 0.0f
    };

    // ---- < INIT REGULATORS > -----
    // Current regulators
    pi_init(&config.regulators.pi_d_current_axis, PI_CURRENT_KP, PI_CURRENT_KI, V_BUS_NOMINAL); 
    pi_init(&config.regulators.pi_q_current_axis, PI_CURRENT_KP, PI_CURRENT_KI, V_BUS_NOMINAL); 
    // Velocity regulator
    pi_init(&config.regulators.pi_angular_velocity, PI_VELOCITY_KP, PI_VELOCITY_KI,  PI_CURRENT_SOFT_LIMIT);
    // Position regulator
    pid_init(&config.regulators.pid_position, PI_POSITION_KP, PI_POSITION_KI, PI_POSITION_KD, PI_VELOCITY_SOFT_LIMIT);

    config.control_state = NO_CONTROL; // Initial state - Keep it on NO_CONTROL if you are doing calibration 

    return config;
}

void set_control_state(config_t* config, enum ControlState new_control_state){
    config->control_state = new_control_state;
    if (new_control_state == NO_CONTROL){
        pwm_set(0.5, 0.5, 0.5);
    }
}


// ---- <CURRENT SETTERS/GETTERS> ----
void set_torque_current_target(config_t* config, float new_torque_current_target){
    new_torque_current_target = clampf(new_torque_current_target, -config->torque_current_soft_limit, config->torque_current_soft_limit);
    config->torque_current_target = new_torque_current_target;
}

void set_torque_current_soft_limit(config_t* config, float new_torque_current_soft_limit){
    new_torque_current_soft_limit = clampf(new_torque_current_soft_limit, -PI_CURRENT_HARD_LIMIT, PI_CURRENT_HARD_LIMIT); // Clamp HARD LIMIT
    config->torque_current_soft_limit = new_torque_current_soft_limit; // Update stored value
    pi_set_out_max(&config->regulators.pi_angular_velocity, new_torque_current_soft_limit); // Update regulator
    config->torque_current_target = clampf(config->torque_current_target, -new_torque_current_soft_limit, new_torque_current_soft_limit); // Clamp current target value
}

void set_current_kp(config_t* config, float new_kp){
    config->regulators.pi_q_current_axis.kp = new_kp;
    config->regulators.pi_d_current_axis.kp = new_kp;
}

void set_current_ki(config_t* config, float new_ki){
    config->regulators.pi_q_current_axis.ki = new_ki;
    config->regulators.pi_d_current_axis.ki = new_ki;
}

float get_torque_current_soft_limit(config_t* config){
    return config->torque_current_soft_limit;
}
// ---- </CURRENT SETTERS/GETTERS> ----


// ---- <ANGULAR VELOCITY SETTERS/GETTERS> -----
void set_angular_velocity_target(config_t* config, float new_angular_velocity_target){
    new_angular_velocity_target = clampf(new_angular_velocity_target, -config->angular_velocity_soft_limit, config->angular_velocity_soft_limit);
    config->angular_velocity_target = new_angular_velocity_target;
}

void set_angular_velocity_soft_limit(config_t* config, float new_angular_velocity_soft_limit){
    new_angular_velocity_soft_limit = clampf(new_angular_velocity_soft_limit, -PI_VELOCITY_HARD_LIMIT, PI_VELOCITY_HARD_LIMIT); // Clamp HARD LIMIT
    config->angular_velocity_soft_limit = new_angular_velocity_soft_limit;
    pid_set_out_max(&config->regulators.pid_position, new_angular_velocity_soft_limit);
    config->angular_velocity_target = clampf(config->angular_velocity_target, -config->angular_velocity_soft_limit, config->angular_velocity_soft_limit);

}

float get_angular_velocity_soft_limit(config_t* config){
    return config->angular_velocity_soft_limit;
}

void set_angular_velocity_kp(config_t* config, float new_kp){
    config->regulators.pi_angular_velocity.kp = new_kp;
}
void set_angular_velocity_ki(config_t* config, float new_ki){
    config->regulators.pi_angular_velocity.ki = new_ki;
}

// ---- </ANGULAR VELOCITY SETTERS/GETTERS> -----


// ----- <POSITION SETTERS/GETTERS> ------
void set_position_target(config_t* config, float new_position_target){
    config->position_target = new_position_target;
}

float get_position_target(config_t* config){
    return config->position_target;
}

float get_position(config_t* config){
    float position = encoder_get_turns(&config->encoder);

    return position;
}

void set_position_kp(config_t* config, float new_kp){
    config->regulators.pid_position.kp = new_kp;
}
void set_position_ki(config_t* config, float new_ki){
    config->regulators.pid_position.ki = new_ki;
}
void set_position_kd(config_t* config, float new_kd){
    config->regulators.pid_position.kd = new_kd;
}

// ----- </POSITION SETTERS/GETTERS> ------

