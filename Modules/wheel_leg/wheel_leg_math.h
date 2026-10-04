#ifndef WHEEL_LEG_MATH_H
#define WHEEL_LEG_MATH_H

void WheelLegPosition(float phi1, float phi4, float position[2]);
void WheelLegSpeed(float dphi1, float dphi4, float phi1, float phi4, float speed[2]);
void WheelLegVMC(float force, float hip_torque, float phi1, float phi4, float joint_torque[2]);
void WheelLegLQRGain(float leg_length, float gain[12]);

#endif // WHEEL_LEG_MATH_H
