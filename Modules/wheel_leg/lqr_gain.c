#include "wheel_leg_math.h"

/* Cubic gain schedule generated from the original wheel-leg LQR model. */
void WheelLegLQRGain(float leg_length, float gain[12])
{
    const float l2 = leg_length * leg_length;
    const float l3 = l2 * leg_length;

    gain[0] = ((leg_length * -3.69957685f - l2 * 43.9885101f) + l3 * 99.9583f) - 0.0313686468f;
    gain[1] = ((leg_length * 0.157740936f + l2 * 0.643602967f) - l3 * 3.82848287f) - 0.0167115517f;
    gain[2] = ((leg_length * -0.267484248f - l2 * 7.52513933f) + l3 * 10.4716768f) - 0.00230771117f;
    gain[3] = ((leg_length * 0.0107927797f + l2 * 0.0552131534f) - l3 * 0.203815028f) - 0.000111954025f;
    gain[4] = ((leg_length * -2.81462908f - l2 * 15.948432f) + l3 * 61.2644043f) + 0.0244205408f;
    gain[5] = ((leg_length * -0.0117965024f - l2 * 0.0319111384f) - l3 * 0.15966849f) + 0.0242980551f;
    gain[6] = ((leg_length * -2.28098559f - l2 * 16.7198353f) + l3 * 56.8550911f) + 0.00142440468f;
    gain[7] = ((leg_length * 0.151891083f - l2 * 0.993286073f) + l3 * 2.26152825f) + 0.00626859302f;
    gain[8] = ((leg_length * -16.6314487f + l2 * 103.742058f) - l3 * 256.677185f) + 1.40490592f;
    gain[9] = ((leg_length * 0.710763872f - l2 * 5.29697132f) + l3 * 14.2568178f) + 1.65064549f;
    gain[10] = ((leg_length * -0.753411055f + l2 * 4.84317493f) - l3 * 12.2170048f) + 0.0655577183f;
    gain[11] = ((leg_length * 0.00581070222f - l2 * 0.0383336358f) + l3 * 0.0981179625f) + 0.076992251f;
}
