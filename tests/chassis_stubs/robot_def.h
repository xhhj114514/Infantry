#ifndef CHASSIS_TEST_ROBOT_DEF_H
#define CHASSIS_TEST_ROBOT_DEF_H

#include <stdint.h>

typedef enum { CHASSIS_ZERO_FORCE = 0, CHASSIS_ROTATE } chassis_mode_e;

typedef struct
{
    float vx;
    float vy;
    float wz;
    chassis_mode_e chassis_mode;
} Chassis_Ctrl_Cmd_s;

typedef struct
{
    unsigned int enemy_color;
    uint16_t robot_level;
    uint8_t power_flag;
} Chassis_Upload_Data_s;

#endif
