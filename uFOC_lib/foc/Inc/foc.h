#ifndef FOC_H
#define FOC_H

void clarke_transform(float ia, float ib, float ic, float *i_alpha, float *i_beta);
void inv_clarke_transform(float v_alpha, float v_beta, float *va, float *vb, float *vc);

void park_transform(float i_alpha, float i_beta, float theta, float *id, float *iq);
void inv_park_transform(float vd, float vq, float theta, float *v_alpha, float *v_beta);




void init_sin_table();
float fast_sin(float x);
float fast_cos(float x);


#endif