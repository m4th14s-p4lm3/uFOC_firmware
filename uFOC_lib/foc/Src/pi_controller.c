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



// float pi_update(PIController *pi, float error, float dt)
// {
//     float xlim = pi->out_max;
//     int wup = (error > -xlim && error < xlim) ? 1 : 0;

//     pi->integral += wup * pi->ki * dt * error;

//     float output = pi->kp * error + pi->integral;
//     output = clampf(output, -xlim, xlim);

//     return output;
// }