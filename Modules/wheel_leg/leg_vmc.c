#include "wheel_leg_math.h"

#include <math.h>

/* Inverse Jacobian mapping from virtual-leg force/torque to joint torques. */
void WheelLegVMC(float force, float hip_torque, float phi1, float phi4, float joint_torque[2])
{
    float a_tmp;
    float t104;
    float t123_tmp;
    float t124_tmp;
    float t131;
    float t143;
    float t146;
    float t148;
    float t16_tmp_tmp;
    float t180;
    float t181;
    float t182;
    float t18_tmp_tmp;
    float t33_tmp;
    float t34_tmp;
    float t35_tmp;
    float t36_tmp;
    float t51_tmp;
    float t54_tmp;
    float t5_tmp;
    float t64_tmp;
    float t75;
    float t79;
    float t7_tmp;
    float t81;
    float t82;
    float t83;
    float t84;
    float t90;
    float t91;
    float t93;
    float t94;

    t148 = cosf(phi1);
    t5_tmp = cosf(phi4);
    t182 = sinf(phi1);
    t7_tmp = sinf(phi4);
    t16_tmp_tmp = t148 / 20.0f;
    t18_tmp_tmp = t182 / 20.0f;
    t33_tmp = t148 * 0.0105f;
    t34_tmp = t5_tmp * 0.0105f;
    t35_tmp = t182 * 0.0105f;
    t36_tmp = t7_tmp * 0.0105f;
    t104 = t18_tmp_tmp - t7_tmp / 20.0f;
    t51_tmp = t5_tmp / 20.0f - t16_tmp_tmp + 0.06f;
    t54_tmp = t35_tmp - t36_tmp;
    t64_tmp = t34_tmp - t33_tmp + 0.0126f;
    t75 = t104 * t104 + t51_tmp * t51_tmp;
    t81 = t148 * t104 / 10.0f + t182 * t51_tmp / 10.0f;
    t82 = t5_tmp * t104 / 10.0f + t7_tmp * t51_tmp / 10.0f;
    t79 = t75;
    t83 = t81;
    t84 = t82;
    t90 = 1.0f / (t64_tmp + t75);
    t91 = t90 * t90;
    t93 = 1.0f / (t64_tmp + t79);
    t94 = t93 * t93;
    t104 = sqrtf((t54_tmp * t54_tmp + t64_tmp * t64_tmp) - t75 * t75);
    t146 = 1.0f / t104;
    t104 += t36_tmp - t35_tmp;
    t51_tmp = atanf(t90 * t104) * 2.0f;
    t123_tmp = cosf(t51_tmp);
    t124_tmp = sinf(t51_tmp);
    t131 = 1.0f / (t91 * (t104 * t104) + 1.0f);
    t143 = 1.0f / (t94 * (t104 * t104) + 1.0f);
    t180 = (t35_tmp + t81) * t91 * t104 +
           t90 * (t33_tmp - t146 * ((t148 * t54_tmp * 0.021f + t182 * t64_tmp * 0.021f) -
                                    t75 * t81 * 2.0f) /
                                   2.0f);
    t181 = (t36_tmp + t82) * t91 * t104 +
           t90 * (t34_tmp - t146 * ((t5_tmp * t54_tmp * 0.021f + t7_tmp * t64_tmp * 0.021f) -
                                    t75 * t82 * 2.0f) /
                                   2.0f);
    a_tmp = t18_tmp_tmp + t124_tmp * 0.105f;
    t51_tmp = t16_tmp_tmp + t123_tmp * 0.105f - 0.03f;
    t182 = (t35_tmp + t83) * t94 * t104 +
           t93 * (t33_tmp - t146 * ((t148 * t54_tmp * 0.021f + t182 * t64_tmp * 0.021f) -
                                    t79 * t83 * 2.0f) /
                                   2.0f);
    t94 = (t36_tmp + t84) * t94 * t104 +
          t93 * (t34_tmp - t146 * ((t5_tmp * t54_tmp * 0.021f + t7_tmp * t64_tmp * 0.021f) -
                                   t79 * t84 * 2.0f) /
                                  2.0f);
    t146 = t18_tmp_tmp + t124_tmp * 0.105f;
    t148 = t16_tmp_tmp + t123_tmp * 0.105f - 0.03f;
    t104 = -t16_tmp_tmp - t123_tmp * 0.105f + 0.03f;
    t82 = t104 * t104;
    t84 = 1.0f / t104;
    t93 = t123_tmp * t143;
    t75 = t124_tmp * t143;
    t104 = a_tmp * a_tmp;
    t81 = force * (1.0f / sqrtf(t104 + t51_tmp * t51_tmp));
    t91 = t123_tmp * t131;
    t90 = t124_tmp * t131;
    t51_tmp = a_tmp * (1.0f / t82);
    t104 = hip_torque * t82 * (1.0f / (t104 + t82));
    joint_torque[0] = t81 *
                          (t146 * (t16_tmp_tmp - t93 * t182 * 0.21f) * 2.0f -
                           t148 * (t18_tmp_tmp - t75 * t182 * 0.21f) * 2.0f) /
                          2.0f -
                      t104 * (t84 * (t16_tmp_tmp - t91 * t180 * 0.21f) -
                              t51_tmp * (t18_tmp_tmp - t90 * t180 * 0.21f));
    joint_torque[1] = t81 * (t93 * t146 * t94 * 0.42f - t75 * t148 * t94 * 0.42f) / 2.0f +
                      t104 * (t84 * (0.0f - t91 * t181 * 0.21f) +
                              t51_tmp * (t90 * t181 * 0.21f));
}
