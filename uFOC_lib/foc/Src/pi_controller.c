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
    pi->integral += pi->ki * error * dt;
    pi->integral = clampf(pi->integral, pi->out_min, pi->out_max);

    float output = pi->kp * error + pi->integral;
    output = clampf(output, pi->out_min, pi->out_max);

    return output;
}