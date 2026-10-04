#ifndef CHASSIS_TEST_BSP_CAN_H
#define CHASSIS_TEST_BSP_CAN_H

#include <stdint.h>

typedef struct { unsigned int unused; } CAN_HandleTypeDef;
typedef struct { unsigned int unused; } CANInstance;
typedef struct
{
    CAN_HandleTypeDef *can_handle;
    uint32_t tx_id;
    uint32_t rx_id;
    void (*can_module_callback)(CANInstance *);
    void *id;
} CAN_Init_Config_s;

extern CAN_HandleTypeDef hcan2;

#endif
