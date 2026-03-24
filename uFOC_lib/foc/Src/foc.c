#include <math.h>
#include "foc.h"

void clarke_transform(float ia, float ib, float ic, 
                        float *i_alpha, float *i_beta) {
    const float c = 0.86602540378f; // sqrt(3)/2 - precalculated value

    *i_alpha = (2.0f / 3.0f) * (ia - 0.5f * ib - 0.5f * ic);
    *i_beta  = (2.0f / 3.0f) * (c * ib - c * ic);
}

void inv_clarke_transform(float v_alpha, float v_beta,
                          float *va, float *vb, float *vc){
    const float c = 0.86602540378f; // sqrt(3)/2 - precalculated value

    *va = v_alpha;
    *vb = -0.5f * v_alpha + c * v_beta;
    *vc = -0.5f * v_alpha - c * v_beta;
}

void park_transform(float i_alpha, float i_beta, float theta,
                    float *id, float *iq){
    float ct = cosf(theta);
    float st = sinf(theta);

    *id =  i_alpha * ct + i_beta * st;
    *iq = -i_alpha * st + i_beta * ct;
}

void inv_park_transform(float vd, float vq, float theta,
                        float *v_alpha, float *v_beta)
{
    float ct = cosf(theta);
    float st = sinf(theta);

    *v_alpha = vd * ct - vq * st;
    *v_beta  = vd * st + vq * ct;
}