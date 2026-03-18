#ifndef PI_CONTROLLER_H
#define PI_CONTROLLER_H

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

#endif