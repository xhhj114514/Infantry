#ifndef CHASSIS_H
#define CHASSIS_H

#include <stdint.h>
#include "wheel_leg_control.h"

typedef struct
{
    float length_m;  // Distance from the midpoint between the two joint bases.
    float angle_rad; // Direction from local +X; vertical +Y is pi/2.
    uint8_t valid;
} ChassisLegPose_s;

typedef struct
{
    /* Indexed by WheelLegMotorIndex_e; offline angles and invalid poses are NAN. */
    float motor_angle_rad[WHEEL_LEG_MOTOR_COUNT];
    ChassisLegPose_s left_leg;
    ChassisLegPose_s right_leg;
    uint8_t motor_online_mask; // Bit i corresponds to WheelLegMotorIndex_e i.
} ChassisLegPosition_s;

/**
 * @brief 底盘应用初始化,请在开启rtos之前调用(目前会被RobotInit()调用)
 * 
 */
void ChassisInit(void);

/**
 * @brief LegPosition 验证任务,读取关节反馈并解算左右腿位置,保持电机停止
 * 
 */
void ChassisTask(void);

/** Read leg lengths (m), angles (rad) and validity, or watch leg_position in the debugger. */
const ChassisLegPosition_s *ChassisGetLegPosition(void);

/** Retained command API; the position validation task does not apply motor output. */
void ChassisSetCommand(float speed_mps, float yaw_speed_rad_s, float leg_length_m);

/** Retained controller enable API; validation always keeps all motors stopped. */
void ChassisEnable(uint8_t enabled);

#endif // CHASSIS_H
