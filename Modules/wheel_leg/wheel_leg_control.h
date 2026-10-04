#ifndef WHEEL_LEG_CONTROL_H
#define WHEEL_LEG_CONTROL_H

#include <stdint.h>

#define WHEEL_LEG_MOTOR_COUNT 6
#define WHEEL_LEG_CONTROL_PERIOD_S 0.004f
#define WHEEL_LEG_DEFAULT_LENGTH_M 0.070f
#define WHEEL_LEG_MIN_LENGTH_M 0.068f
#define WHEEL_LEG_MAX_LENGTH_M 0.115f
#define WHEEL_LEG_MAX_SPEED_MPS 0.60f
#define WHEEL_LEG_MAX_YAW_SPEED_RAD_S 0.50f

#define WHEEL_LEG_LEFT_JOINT_0_OFFSET_RAD -0.5f
#define WHEEL_LEG_LEFT_JOINT_1_OFFSET_RAD 5.9f
#define WHEEL_LEG_RIGHT_JOINT_0_OFFSET_RAD 0.0f
#define WHEEL_LEG_RIGHT_JOINT_1_OFFSET_RAD 0.0f


typedef enum
{
    WHEEL_LEG_LEFT_JOINT_0 = 0,
    WHEEL_LEG_LEFT_JOINT_1 = 2,
    WHEEL_LEG_LEFT_WHEEL = 1,
    WHEEL_LEG_RIGHT_JOINT_0 = 3,
    WHEEL_LEG_RIGHT_JOINT_1 = 5,
    WHEEL_LEG_RIGHT_WHEEL = 4,
} WheelLegMotorIndex_e;

typedef enum
{
    WHEEL_LEG_FAULT_NONE = 0,
    WHEEL_LEG_FAULT_NOT_READY = 1U << 0,
    WHEEL_LEG_FAULT_KINEMATICS = 1U << 1,
    WHEEL_LEG_FAULT_PROTECTION = 1U << 2,
} WheelLegFault_e;

typedef struct
{
    float angle_rad[WHEEL_LEG_MOTOR_COUNT];
    float speed_rad_s[WHEEL_LEG_MOTOR_COUNT];
    float yaw_rad;
    float pitch_rad;
    float roll_rad;
    float yaw_speed_rad_s;
    float pitch_speed_rad_s;
    float roll_speed_rad_s;
    float vertical_accel_mps2;
    uint32_t now_ms;
    uint8_t sensors_ready;
} WheelLegFeedback_s;

typedef struct
{
    float torque_nm[WHEEL_LEG_MOTOR_COUNT];
    uint8_t output_enabled;
} WheelLegOutput_s;

typedef struct
{
    float angle_rad;
    float length_m;
    float angular_speed_rad_s;
    float length_speed_mps;
    float length_accel_mps2;
} WheelLegPose_s;

typedef struct
{
    float theta;
    float theta_speed;
    float position_m;
    float speed_mps;
    float pitch;
    float pitch_speed;
} WheelLegState_s;

typedef struct
{
    WheelLegPose_s left_leg;
    WheelLegPose_s right_leg;
    WheelLegState_s state;
    float target_position_m;
    float target_speed_mps;
    float target_yaw_rad;
    float target_leg_length_m;
    float left_support_force_n;
    float right_support_force_n;
    uint8_t touching_ground;
    uint8_t cushioning;
    uint8_t fault_flags;
} WheelLegTelemetry_s;

void WheelLegControlInit(void);
void WheelLegSetCommand(float speed_mps, float yaw_speed_rad_s, float leg_length_m);
void WheelLegSetEnabled(uint8_t enabled);
void WheelLegControlStep(const WheelLegFeedback_s *feedback, WheelLegOutput_s *output);
const WheelLegTelemetry_s *WheelLegGetTelemetry(void);

#endif // WHEEL_LEG_CONTROL_H
