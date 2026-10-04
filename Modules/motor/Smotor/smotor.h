#ifndef SMOTOR_H
#define SMOTOR_H

#include "bsp_can.h"
#include "controller.h"
#include "daemon.h"
#include "motor_def.h"

#define SMOTOR_MOTOR_CNT 16
#define SMOTOR_MIN_ID 1
#define SMOTOR_MAX_ID 8
#define SMOTOR_ZERO_SAMPLE_COUNT 100U

typedef float (*SMotorRevVoltCalc)(float speed_rad_s);

typedef enum
{
    SMOTOR_CONTROL_SPEED = 0,
    SMOTOR_CONTROL_POSITION,
    SMOTOR_CONTROL_VOLTAGE,
    SMOTOR_CONTROL_TORQUE,
} SMotor_Control_Mode_e;

typedef struct
{
    float max_voltage_v;
    float torque_ratio_nm_per_v;
    float output_ratio;
    SMotorRevVoltCalc calc_rev_volt;
} SMotor_Torque_Config_s;

typedef struct
{
    float total_angle_rad;
    float position_rad;
    float speed_rad_s;
    float speed_rpm;
} SMotor_Measure_s;

typedef struct
{
    SMotor_Measure_s measure;
    Motor_Control_Setting_s motor_settings;
    Motor_Controller_s motor_controller;

    CANInstance *motor_can_instance;
    DaemonInstance *daemon;

    uint8_t motor_id;
    uint8_t sender_group;
    uint8_t message_num;
    uint8_t feedback_received;
    Motor_Working_Type_e stop_flag;

    SMotor_Control_Mode_e control_mode;
    SMotor_Torque_Config_s torque_config;
    float torque_ref_nm;

    float zero_angle_rad;
    float zero_sample_sum_rad;
    uint16_t zero_sample_count;
    uint8_t zero_ready;
} SMotorInstance;

/**
 * @brief Register an Smotor. can_init_config.tx_id is the motor ID (1-8).
 */
SMotorInstance *SMotorInit(Motor_Init_Config_s *config);

/** @brief Set the speed reference in rpm when SPEED_LOOP is enabled. */
void SMotorSetRef(SMotorInstance *motor, float ref);

/**
 * @brief Set the position reference in rad relative to the calibrated zero.
 * @note ANGLE_LOOP and SPEED_LOOP must be configured. Output remains zero until
 *       the first SMOTOR_ZERO_SAMPLE_COUNT feedback samples establish the zero.
 */
void SMotorSetPos(SMotorInstance *motor, float position_rad);

/** @brief Use the latest measured motor position as zero. Returns 0 before feedback is available. */
uint8_t SMotorSetZero(SMotorInstance *motor);

/** @brief Return non-zero after automatic or manual zero calibration completes. */
uint8_t SMotorIsZeroReady(const SMotorInstance *motor);

/**
 * @brief Configure torque-to-voltage conversion and back-EMF compensation.
 * @note max_voltage_v is the effective phase-voltage limit in volts.
 */
uint8_t SMotorConfigureTorque(SMotorInstance *motor, const SMotor_Torque_Config_s *config);

/** @brief Switch to torque mode and set the logical torque reference in N*m. */
void SMotorSetTorque(SMotorInstance *motor, float torque_nm);

/** @brief Back-EMF fits ported from the original ESP32 driver; input is rad/s. */
float SMotorCalcRevVolt4310(float speed_rad_s);
float SMotorCalcRevVolt2805(float speed_rad_s);

void SMotorStop(SMotorInstance *motor);
void SMotorEnable(SMotorInstance *motor);

/** @brief Return non-zero after feedback has been received and before timeout. */
uint8_t SMotorIsOnline(const SMotorInstance *motor);

/** @brief Calculate the speed PID and transmit the shared voltage frames. */
void SMotorControl(void);

#endif // SMOTOR_H
