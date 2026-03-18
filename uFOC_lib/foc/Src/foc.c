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
    float cg = cosf(theta);
    float sg = sinf(theta);

    *id =  i_alpha * cg + i_beta * sg;
    *iq = -i_alpha * sg + i_beta * cg;
}

void inv_park_transform(float vd, float vq, float theta,
                        float *v_alpha, float *v_beta)
{
    float cg = cosf(theta);
    float sg = sinf(theta);

    *v_alpha = vd * cg - vq * sg;
    *v_beta  = vd * sg + vq * cg;
}