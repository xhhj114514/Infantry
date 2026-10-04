#ifndef CHASSIS_TEST_MOTOR_OFFSETS_OVERRIDE_H
#define CHASSIS_TEST_MOTOR_OFFSETS_OVERRIDE_H

/* Optional second test build: exercise manual calibration with nonzero offsets.
 * Include the real hardware definitions first, then change only this fixture.
 * This header is never included by production sources. */
#include "motor_def.h"
#undef WHEEL_LEG_MOTOR_OFFSETS_RAD
#define WHEEL_LEG_MOTOR_OFFSETS_RAD {0.23f, -0.41f, 0.37f, -0.29f, 0.61f, -0.53f}

#endif
