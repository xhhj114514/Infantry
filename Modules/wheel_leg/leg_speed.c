#include "wheel_leg_math.h"

#include <math.h>

/* Analytic derivative of the five-bar leg forward kinematics. */
void WheelLegSpeed(float dphi1, float dphi4, float phi1, float phi4, float speed[2])
{
    float b_out_tmp;
    float out_tmp;
    float t10_tmp;
    float t12_tmp;
    float t2;
    float t21;
    float t22;
    float t23;
    float t24;
    float t28;
    float t3;
    float t32;
    float t4;
    float t44;
    float t47;
    float t48;
    float t5;
    float t52;
    float t53;
    float t59;
    float t60;
    float t61;
    float t70;
    float t71;
    float t76;

    t2 = cosf(phi1);
    t3 = cosf(phi4);
    t4 = sinf(phi1);
    t5 = sinf(phi4);
    t10_tmp = t2 / 20.0f;
    t12_tmp = t4 / 20.0f;
    t21 = t2 * 0.0105f;
    t22 = t3 * 0.0105f;
    t23 = t4 * 0.0105f;
    t24 = t5 * 0.0105f;
    t28 = t12_tmp - t5 / 20.0f;
    out_tmp = t3 / 20.0f - t10_tmp;
    t32 = t23 - t24;
    b_out_tmp = t22 - t21;
    t44 = t28 * t28 + (out_tmp + 0.06f) * (out_tmp + 0.06f);
    t47 = t2 * t28 / 10.0f + t4 * (out_tmp + 0.06f) / 10.0f;
    t48 = t3 * t28 / 10.0f + t5 * (out_tmp + 0.06f) / 10.0f;
    t52 = 1.0f / ((b_out_tmp + 0.0126f) + t44);
    t53 = t52 * t52;
    t59 = sqrtf((t32 * t32 + (b_out_tmp + 0.0126f) * (b_out_tmp + 0.0126f)) - t44 * t44);
    t60 = 1.0f / t59;
    t61 = (t24 - t23) + t59;
    t28 = atanf(t52 * t61) * 2.0f;
    t70 = cosf(t28);
    t71 = sinf(t28);
    t76 = 1.0f / (t53 * (t61 * t61) + 1.0f);
    t47 = (t23 + t47) * t53 * t61 +
          t52 * (t21 - t60 * ((t2 * t32 * 0.021f + t4 * (b_out_tmp + 0.0126f) * 0.021f) -
                              t44 * t47 * 2.0f) /
                             2.0f);
    t28 = (t24 + t48) * t53 * t61 +
          t52 * (t22 - t60 * ((t3 * t32 * 0.021f + t5 * (b_out_tmp + 0.0126f) * 0.021f) -
                              t44 * t48 * 2.0f) /
                             2.0f);
    t21 = t12_tmp + t71 * 0.105f;
    out_tmp = t10_tmp + t70 * 0.105f;
    t59 = t70 * t76;
    t23 = t59 * t28;
    t2 = t71 * t76;
    t4 = t2 * t28;
    t28 = -t10_tmp - t70 * 0.105f;
    t60 = (t28 + 0.03f) * (t28 + 0.03f);
    t61 = 1.0f / (t28 + 0.03f);
    t48 = 1.0f / sqrtf(t21 * t21 + (out_tmp - 0.03f) * (out_tmp - 0.03f));
    t52 = 1.0f / (t21 * t21 + t60);
    t53 = t21 * (1.0f / t60);
    t59 = t10_tmp - t59 * t47 * 0.21f;
    t28 = t12_tmp - t2 * t47 * 0.21f;
    speed[0] = dphi4 * t48 * (t21 * t23 * 0.42f - (out_tmp - 0.03f) * t4 * 0.42f) / 2.0f +
               dphi1 * t48 * (t21 * t59 * 2.0f - (out_tmp - 0.03f) * t28 * 2.0f) / 2.0f;
    speed[1] = dphi4 * t60 * t52 * (t61 * (0.0f - t23 * 0.21f) + t53 * (t4 * 0.21f)) -
               dphi1 * t60 * t52 * (t61 * t59 - t53 * t28);
}
