#ifndef PI_CONTROLLER_H
#define PI_CONTROLLER_H


// PI
typedef struct {
    float kp;
    float ki;
    float integral;
    float out_min;
    float out_max;
} PIController;

void pi_init(PIController *pi, float kp, float ki, float out_min, float out_max);
void pi_reset(PIController *pi);
float pi_update(PIController *pi, float error, float dt);


// PID
typedef struct {
    float kp;
    float ki;
    float kd;

    float integral;
    float prev_error;
    int has_prev_error;

    float out_min;
    float out_max;
} PIDController;

void pid_init(PIDController *pid, float kp, float ki, float kd, float out_min, float out_max);
void pid_reset(PIDController *pid);
float pid_update(PIDController *pid, float error, float dt);


#endif