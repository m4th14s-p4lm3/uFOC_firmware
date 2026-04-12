#include "pi_controller.h"

static inline float clampf(float x, float min_val, float max_val)
{
    if (x < min_val) return min_val;
    if (x > max_val) return max_val;
    return x;
}

void pi_init(PIController *pi, float kp, float ki, float out_min, float out_max)
{
    pi->kp = kp;
    pi->ki = ki;
    pi->integral = 0.0f;
    pi->out_min = out_min;
    pi->out_max = out_max;
}

void pi_reset(PIController *pi)
{
    pi->integral = 0.0f;
}

float pi_update(PIController *pi, float error, float dt)
{
    float p = pi->kp * error;
    float i_new = pi->integral + pi->ki * dt * error;
    float u = p + i_new;

    if (u > pi->out_max) {
        if (error < 0.0f) {
            pi->integral = i_new;
        }
        return pi->out_max;
    }

    if (u < pi->out_min) {
        if (error > 0.0f) {
            pi->integral = i_new;
        }
        return pi->out_min;
    }

    pi->integral = i_new;
    return u;
}

// PID



void pid_init(PIDController *pid, float kp, float ki, float kd, float out_min, float out_max)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;

    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
    pid->has_prev_error = 0;

    pid->out_min = out_min;
    pid->out_max = out_max;
}

void pid_reset(PIDController *pid)
{
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
    pid->has_prev_error = 0;
}

float pid_update(PIDController *pid, float error, float dt)
{
    float p, i_new, d, u;

    p = pid->kp * error;

    i_new = pid->integral + pid->ki * dt * error;

    if (dt > 0.0f && pid->has_prev_error) {
        d = pid->kd * (error - pid->prev_error) / dt;
    } else {
        d = 0.0f;
    }

    u = p + i_new + d;

    if (u > pid->out_max) {
        if (error < 0.0f) {
            pid->integral = i_new;
        }
        pid->prev_error = error;
        pid->has_prev_error = 1;
        return pid->out_max;
    }

    if (u < pid->out_min) {
        if (error > 0.0f) {
            pid->integral = i_new;
        }
        pid->prev_error = error;
        pid->has_prev_error = 1;
        return pid->out_min;
    }

    pid->integral = i_new;
    pid->prev_error = error;
    pid->has_prev_error = 1;

    return u;
}