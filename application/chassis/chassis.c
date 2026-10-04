#include "chassis.h"

#include "message_center.h"
#include "robot_def.h"
#include "smotor.h"
#include "motor_def.h"
#include "wheel_leg_math.h"

#include <math.h>
#include <stddef.h>

typedef struct
{
    SMotorInstance *motor;
    float offset_angle_rad;
    float direction;
} ChassisMotor_s;

static const uint8_t motor_id[WHEEL_LEG_MOTOR_COUNT] = WHEEL_LEG_MOTOR_IDS;
static float motor_offset_rad[WHEEL_LEG_MOTOR_COUNT] = WHEEL_LEG_MOTOR_OFFSETS_RAD;
static const float motor_direction[WHEEL_LEG_MOTOR_COUNT] = WHEEL_LEG_MOTOR_DIRECTIONS;

static ChassisMotor_s chassis_motor[WHEEL_LEG_MOTOR_COUNT];
static ChassisLegPosition_s leg_position;
static Publisher_t *chassis_pub;
static Chassis_Upload_Data_s chassis_feedback;

static uint8_t RegisterMotor(uint8_t index)
{
    const uint8_t is_wheel = index == WHEEL_LEG_LEFT_WHEEL || index == WHEEL_LEG_RIGHT_WHEEL;
    Motor_Init_Config_s config = {
        .can_init_config = {
            .can_handle = WHEEL_LEG_CAN_HANDLE,
            .tx_id = motor_id[index],
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type = ANGLE_LOOP,
            .close_loop_type = ANGLE_LOOP | SPEED_LOOP,
            .motor_reverse_flag = motor_direction[index] < 0.0f ?
                                      MOTOR_DIRECTION_REVERSE : MOTOR_DIRECTION_NORMAL,
            .feedback_reverse_flag = FEEDBACK_DIRECTION_NORMAL,
        },
        .controller_param_init_config = {
            .angle_PID = {
                .Kp = CHASSIS_MOTOR_POSITION_PID_KP,
                .Ki = CHASSIS_MOTOR_POSITION_PID_KI,
                .Kd = CHASSIS_MOTOR_POSITION_PID_KD,
                .MaxOut = CHASSIS_MOTOR_POSITION_MAX_SPEED_RPM,
                .IntegralLimit = CHASSIS_MOTOR_POSITION_MAX_SPEED_RPM,
                .Improve = CHASSIS_MOTOR_PID_IMPROVE,
            },
            .speed_PID = {
                .Kp = CHASSIS_MOTOR_SPEED_PID_KP,
                .Ki = CHASSIS_MOTOR_SPEED_PID_KI,
                .Kd = CHASSIS_MOTOR_SPEED_PID_KD,
                .MaxOut = is_wheel ? WHEEL_LEG_WHEEL_MAX_VOLTAGE_V * 1000.0f :
                                     SMOTOR_MAX_VOLTAGE_MV,
                .IntegralLimit = is_wheel ? WHEEL_LEG_WHEEL_MAX_VOLTAGE_V * 1000.0f :
                                           SMOTOR_MAX_VOLTAGE_MV,
                .Improve = CHASSIS_MOTOR_PID_IMPROVE,
            },
        },
    };
    SMotor_Torque_Config_s torque_config = {
        .max_voltage_v = is_wheel ? WHEEL_LEG_WHEEL_MAX_VOLTAGE_V : WHEEL_LEG_JOINT_MAX_VOLTAGE_V,
        .torque_ratio_nm_per_v = is_wheel ? WHEEL_LEG_WHEEL_TORQUE_RATIO_NM_PER_V :
                                            WHEEL_LEG_JOINT_TORQUE_RATIO_NM_PER_V,
        .output_ratio = WHEEL_LEG_MOTOR_OUTPUT_RATIO,
        .calc_rev_volt = is_wheel ? SMotorCalcRevVolt2805 : SMotorCalcRevVolt4310,
    };

    chassis_motor[index].motor = SMotorInit(&config);
    SMotorStop(chassis_motor[index].motor);
    chassis_motor[index].offset_angle_rad = motor_offset_rad[index];
    chassis_motor[index].direction = motor_direction[index];
    if (chassis_motor[index].motor == NULL ||
        !SMotorConfigureTorque(chassis_motor[index].motor, &torque_config))
        return 0;

    SMotorSetPos(chassis_motor[index].motor, CHASSIS_MOTOR_TARGET_POSITION_RAD);
    SMotorStop(chassis_motor[index].motor);
    return 1;
}

static void FillMotorFeedback(void)
{
    leg_position.motor_online_mask = 0;

    for (uint8_t i = 0; i < WHEEL_LEG_MOTOR_COUNT; ++i)
    {
        const ChassisMotor_s *binding = &chassis_motor[i];
        SMotorStop(binding->motor);
        leg_position.motor_angle_rad[i] = NAN;
        if (!SMotorIsOnline(binding->motor))
            continue;

        leg_position.motor_online_mask |= (uint8_t)(1U << i);
        // Use the absolute encoder angle; SMotor's boot-time zero is not the linkage zero.
        leg_position.motor_angle_rad[i] =
            (binding->motor->measure.position_rad - binding->offset_angle_rad) * binding->direction;
    }
}

static void UpdateLegPosition(uint8_t joint_0, uint8_t joint_1, ChassisLegPose_s *leg)
{
    const uint8_t joint_mask = (uint8_t)((1U << joint_0) | (1U << joint_1));
    float position[2];

    leg->length_m = NAN;
    leg->angle_rad = NAN;
    leg->valid = 0;
    if ((leg_position.motor_online_mask & joint_mask) != joint_mask)
        return;
    if (!isfinite(leg_position.motor_angle_rad[joint_0]) ||
        !isfinite(leg_position.motor_angle_rad[joint_1]))
        return;

    // Joint 1 is phi1 and joint 0 is phi4, matching the wheel-leg controller.
    WheelLegPosition(leg_position.motor_angle_rad[joint_1],
                     leg_position.motor_angle_rad[joint_0], position);
    if (!isfinite(position[0]) || !isfinite(position[1]) || position[0] <= 0.0f)
        return;

    leg->length_m = position[0];
    leg->angle_rad = position[1];
    leg->valid = 1;
}

void ChassisInit(void)
{
    WheelLegControlInit();
    WheelLegSetCommand(0.0f, 0.0f, WHEEL_LEG_DEFAULT_LENGTH_M);
    WheelLegSetEnabled(0);
    motor_offset_rad[WHEEL_LEG_LEFT_JOINT_0] =  WHEEL_LEG_LEFT_JOINT_0_OFFSET_RAD;
    motor_offset_rad[WHEEL_LEG_LEFT_JOINT_1] =
        WHEEL_LEG_LEFT_JOINT_1_OFFSET_RAD;
    
    motor_offset_rad[WHEEL_LEG_RIGHT_JOINT_0] =
        WHEEL_LEG_RIGHT_JOINT_0_OFFSET_RAD;
    motor_offset_rad[WHEEL_LEG_RIGHT_JOINT_1] =
        WHEEL_LEG_RIGHT_JOINT_1_OFFSET_RAD;
    for (uint8_t i = 0; i < WHEEL_LEG_MOTOR_COUNT; ++i)
        RegisterMotor(i);

    FillMotorFeedback();
    UpdateLegPosition(WHEEL_LEG_LEFT_JOINT_0, WHEEL_LEG_LEFT_JOINT_1, &leg_position.left_leg);
    UpdateLegPosition(WHEEL_LEG_RIGHT_JOINT_0, WHEEL_LEG_RIGHT_JOINT_1, &leg_position.right_leg);
    chassis_pub = PubRegister("chassis_feed", sizeof(Chassis_Upload_Data_s));
}

const ChassisLegPosition_s *ChassisGetLegPosition(void)
{
    return &leg_position;
}

void ChassisSetCommand(float speed_mps, float yaw_speed_rad_s, float leg_length_m)
{
    WheelLegSetCommand(speed_mps, yaw_speed_rad_s, leg_length_m);
}

void ChassisEnable(uint8_t enabled)
{
    WheelLegSetEnabled(enabled);
    for (uint8_t i = 0; i < WHEEL_LEG_MOTOR_COUNT; ++i)
        SMotorStop(chassis_motor[i].motor);
}

void ChassisTask(void)
{
    FillMotorFeedback();
    UpdateLegPosition(WHEEL_LEG_LEFT_JOINT_0, WHEEL_LEG_LEFT_JOINT_1, &leg_position.left_leg);
    UpdateLegPosition(WHEEL_LEG_RIGHT_JOINT_0, WHEEL_LEG_RIGHT_JOINT_1, &leg_position.right_leg);

    chassis_feedback.power_flag = 0;
    PubPushMessage(chassis_pub, &chassis_feedback);
}
