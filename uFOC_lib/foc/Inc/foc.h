#ifndef FOC_H
#define FOC_H
#include "pi_controller.h"

void clarke_transform(float ia, float ib, float ic, float *i_alpha, float *i_beta);
void inv_clarke_transform(float v_alpha, float v_beta, float *va, float *vb, float *vc);

void park_transform(float i_alpha, float i_beta, float theta, float *id, float *iq);
void inv_park_transform(float vd, float vq, float theta, float *v_alpha, float *v_beta);

void foc_compute_voltages(float id_ref, float iq_ref,               // Input reference currents
                          float* theta_e,                           // Measured input electric angle
                          float *ia, float *ib, float *ic,          // Measured input current
                          float* va, float* vb, float* vc,          // Output voltages
                          PIController* pi_d, PIController* pi_q);  // PI regulators

void svpwm(float* va, float* vb, float* vc,                 // Input voltages from FOC
        float* va_svpwm, float* vb_svpwm, float* vc_svpwm); // Output svpwm voltages 


void init_sin_table();
float fast_sin(float x);
float fast_cos(float x);


#endif