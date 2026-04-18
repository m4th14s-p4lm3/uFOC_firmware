#include <math.h>
#include "foc.h"
#include "pi_controller.h"

#define SIN_TABLE_SIZE 128
#define TWO_PI   6.28318530718f
#define HALF_PI  1.57079632679f
#define SQRT3_2  0.86602540378f
#define TWO_THIRDS 0.6666666667f
#define SIN_MASK (SIN_TABLE_SIZE - 1)
#define SIN_SCALE ((float)SIN_TABLE_SIZE / TWO_PI)

static float sin_table[SIN_TABLE_SIZE];

void init_sin_table(void)
{
    for (int i = 0; i < SIN_TABLE_SIZE; i++) {
        sin_table[i] = sinf(TWO_PI * (float)i / (float)SIN_TABLE_SIZE);
    }
}


static inline void fast_sincos(float theta, float *s, float *c)
{
    /* Wrap theta into [0, 2π) — handles negative values and > 2π */
    theta -= TWO_PI * floorf(theta * (1.0f / TWO_PI));

    float index = theta * SIN_SCALE;
    int i = (int)index;
    float frac = index - (float)i;

    int i0 = i & SIN_MASK;
    int i1 = (i + 1) & SIN_MASK;

    float s1 = sin_table[i0];
    float s2 = sin_table[i1];
    *s = s1 + frac * (s2 - s1);

    int q = SIN_TABLE_SIZE / 4;
    int c0 = (i0 + q) & SIN_MASK;
    int c1 = (i1 + q) & SIN_MASK;

    float c1v = sin_table[c0];
    float c2v = sin_table[c1];
    *c = c1v + frac * (c2v - c1v);
}

void clarke_transform(float ia, float ib, float ic,
                      float *i_alpha, float *i_beta)
{
    *i_alpha = TWO_THIRDS * (ia - 0.5f * ib - 0.5f * ic);
    *i_beta  = TWO_THIRDS * (SQRT3_2 * (ib - ic));
}

void inv_clarke_transform(float v_alpha, float v_beta,
                          float *va, float *vb, float *vc)
{
    *va = v_alpha;
    *vb = -0.5f * v_alpha + SQRT3_2 * v_beta;
    *vc = -0.5f * v_alpha - SQRT3_2 * v_beta;
}

void park_transform(float i_alpha, float i_beta, float theta,
                    float *id, float *iq)
{
    float st, ct;
    fast_sincos(theta, &st, &ct);

    *id =  i_alpha * ct + i_beta * st;
    *iq = -i_alpha * st + i_beta * ct;
}

void inv_park_transform(float vd, float vq, float theta,
                        float *v_alpha, float *v_beta)
{
    float st, ct;
    fast_sincos(theta, &st, &ct);

    *v_alpha = vd * ct - vq * st;
    *v_beta  = vd * st + vq * ct;
}

void foc_compute_voltages(float id_ref, float iq_ref, float* theta_e, 
                    float *ia, float *ib, float *ic,
                    float* va, float* vb, float* vc,
                    PIController* pi_d, PIController* pi_q)
                {
  float i_alpha, i_beta;    
  clarke_transform(*ia, *ib, *ic, &i_alpha, &i_beta);
  float id, iq;
  park_transform(i_alpha, i_beta, *theta_e, &id, &iq);
  

  float err_d = id_ref - id;
  float err_q = iq_ref - iq;
  
  float vd = pi_update(pi_d, err_d, 0.0002f);
  float vq = pi_update(pi_q, err_q, 0.0002f); 
  
  float v_alpha, v_beta;
  inv_park_transform(vd, vq, *theta_e, &v_alpha, &v_beta);
  inv_clarke_transform(v_alpha, v_beta, va, vb, vc);
}

void svpwm(float* va, float* vb, float* vc,
        float* va_svpwm, float* vb_svpwm, float* vc_svpwm)
{
    float vmax = fmaxf(*va, fmaxf(*vb, *vc));
    float vmin = fminf(*va, fminf(*vb, *vc));
    float v0 = -0.5f * (vmax + vmin);
    

    *va_svpwm = *va + v0;
    *vb_svpwm = *vb + v0;
    *vc_svpwm = *vc + v0;
}