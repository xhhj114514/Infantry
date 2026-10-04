#ifndef CHASSIS_TEST_CONTROLLER_H
#define CHASSIS_TEST_CONTROLLER_H

#include "bsp_can.h"

typedef enum
{
    PID_Integral_Limit = 1U << 0,
    PID_Trapezoid_Intergral = 1U << 2,
} PID_Improvement_e;

typedef struct
{
    float Kp;
    float Ki;
    float Kd;
    float MaxOut;
    float IntegralLimit;
    PID_Improvement_e Improve;
} PID_Init_Config_s;

typedef struct { float pid_ref; } PIDInstance;

#endif
