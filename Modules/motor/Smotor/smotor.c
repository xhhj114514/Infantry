#include "smotor.h"

#include "bsp_log.h"
#include "general_def.h"

#include <stdlib.h>
#include <string.h>

#define SMOTOR_SENDER_GROUP_CNT 4
#define SMOTOR_RAD_S_TO_RPM (1.0f / RPM_2_RAD_PER_SEC)

static uint8_t smotor_idx;
static SMotorInstance *smotor_instance[SMOTOR_MOTOR_CNT] = {NULL};

/* CAN1 ID1-4, CAN1 ID5-8, CAN2 ID1-4, CAN2 ID5-8. */
static CANInstance smotor_sender[SMOTOR_SENDER_GROUP_CNT] = {
    [0] = {.can_handle = &hcan1, .txconf.StdId = 0x100, .txconf.IDE = CAN_ID_STD, .txconf.RTR = CAN_RTR_DATA, .txconf.DLC = 8},
    [1] = {.can_handle = &hcan1, .txconf.StdId = 0x200, .txconf.IDE = CAN_ID_STD, .txconf.RTR = CAN_RTR_DATA, .txconf.DLC = 8},
    [2] = {.can_handle = &hcan2, .txconf.StdId = 0x100, .txconf.IDE = CAN_ID_STD, .txconf.RTR = CAN_RTR_DATA, .txconf.DLC = 8},
    [3] = {.can_handle = &hcan2, .txconf.StdId = 0x200, .txconf.IDE = CAN_ID_STD, .txconf.RTR = CAN_RTR_DATA, .txconf.DLC = 8},
};
static uint8_t smotor_sender_enabled[SMOTOR_SENDER_GROUP_CNT] = {0};

float SMotorCalcRevVolt4310(float speed_rad_s)
{
    return 0.000002f * speed_rad_s * speed_rad_s * speed_rad_s -
           0.0002f * speed_rad_s * speed_rad_s +
           0.1521f * speed_rad_s;
}

float SMotorCalcRevVolt2805(float speed_rad_s)
{
    return 0.0000003f * speed_rad_s * speed_rad_s * speed_rad_s -
           0.00007f * speed_rad_s * speed_rad_s +
           0.0275f * speed_rad_s;
}

static int16_t SMotorReadI16LE(const uint8_t *data)
{
    return (int16_t)((uint16_t)data[0] | ((uint16_t)data[1] << 8));
}

static int32_t SMotorReadI32LE(const uint8_t *data)
{
    uint32_t value = (uint32_t)data[0] |
                     ((uint32_t)data[1] << 8) |
                     ((uint32_t)data[2] << 16) |
                     ((uint32_t)data[3] << 24);
    return (int32_t)value;
}

static void SMotorDecode(CANInstance *can_instance)
{
    SMotorInstance *motor = (SMotorInstance *)can_instance->id;

    if (can_instance->rx_len < 6)
        return;

    motor->measure.total_angle_rad = (float)SMotorReadI32LE(can_instance->rx_buff) * 0.001f;

    if (!motor->zero_ready)
    {
        motor->zero_sample_sum_rad += motor->measure.total_angle_rad;
        ++motor->zero_sample_count;
        if (motor->zero_sample_count >= SMOTOR_ZERO_SAMPLE_COUNT)
        {
            motor->zero_angle_rad = motor->zero_sample_sum_rad / (float)motor->zero_sample_count;
            motor->zero_ready = 1;
        }
    }
    motor->measure.position_rad = motor->zero_ready ?
                                      motor->measure.total_angle_rad - motor->zero_angle_rad : 0.0f;
    motor->measure.speed_rad_s = (float)SMotorReadI16LE(can_instance->rx_buff + 4) * 0.1f;
    motor->measure.speed_rpm = motor->measure.speed_rad_s * SMOTOR_RAD_S_TO_RPM;
    motor->feedback_received = 1;
    DaemonReload(motor->daemon);
}

static void SMotorLostCallback(void *motor_ptr)
{
    SMotorInstance *motor = (SMotorInstance *)motor_ptr;
    uint16_t can_bus = motor->motor_can_instance->can_handle == &hcan1 ? 1 : 2;
    motor->stop_flag = MOTOR_STOP;
    LOGWARNING("[smotor] Motor lost, can bus [%d], id [%d]", can_bus, motor->motor_id);
}

static void SMotorAssignSender(SMotorInstance *motor, CAN_HandleTypeDef *can_handle)
{
    uint8_t bus_offset = can_handle == &hcan1 ? 0 : 2;
    uint8_t id_offset = motor->motor_id - SMOTOR_MIN_ID;

    motor->sender_group = bus_offset + (id_offset / 4);
    motor->message_num = id_offset % 4;
    smotor_sender_enabled[motor->sender_group] = 1;
}

SMotorInstance *SMotorInit(Motor_Init_Config_s *config)
{
    if (config == NULL)
        return NULL;

    uint32_t configured_id = config->can_init_config.tx_id;

    if (smotor_idx >= SMOTOR_MOTOR_CNT || configured_id < SMOTOR_MIN_ID || configured_id > SMOTOR_MAX_ID ||
        (config->can_init_config.can_handle != &hcan1 && config->can_init_config.can_handle != &hcan2))
    {
        LOGERROR("[smotor] Invalid config or too many motors");
        return NULL;
    }
    uint8_t motor_id = (uint8_t)configured_id;

    for (size_t i = 0; i < smotor_idx; ++i)
    {
        if (smotor_instance[i]->motor_id == motor_id &&
            smotor_instance[i]->motor_can_instance->can_handle == config->can_init_config.can_handle)
        {
            LOGERROR("[smotor] ID crash, id [%d]", motor_id);
            return NULL;
        }
    }

    SMotorInstance *motor = (SMotorInstance *)malloc(sizeof(SMotorInstance));
    if (motor == NULL)
        return NULL;
    memset(motor, 0, sizeof(SMotorInstance));

    motor->motor_id = motor_id;
    motor->motor_settings = config->controller_setting_init_config;
    if ((motor->motor_settings.close_loop_type & ANGLE_AND_SPEED_LOOP) == ANGLE_AND_SPEED_LOOP)
        motor->control_mode = SMOTOR_CONTROL_POSITION;
    else if (motor->motor_settings.close_loop_type & SPEED_LOOP)
        motor->control_mode = SMOTOR_CONTROL_SPEED;
    else
        motor->control_mode = SMOTOR_CONTROL_VOLTAGE;
    PIDInit(&motor->motor_controller.speed_PID, &config->controller_param_init_config.speed_PID);
    PIDInit(&motor->motor_controller.angle_PID, &config->controller_param_init_config.angle_PID);
    motor->motor_controller.other_angle_feedback_ptr = config->controller_param_init_config.other_angle_feedback_ptr;
    motor->motor_controller.other_speed_feedback_ptr = config->controller_param_init_config.other_speed_feedback_ptr;
    SMotorAssignSender(motor, config->can_init_config.can_handle);

    CAN_Init_Config_s can_config = config->can_init_config;
    can_config.tx_id = motor_id <= 4 ? 0x100 : 0x200;
    can_config.rx_id = 0x100 + motor_id;
    can_config.can_module_callback = SMotorDecode;
    can_config.id = motor;
    motor->motor_can_instance = CANRegister(&can_config);

    Daemon_Init_Config_s daemon_config = {
        .callback = SMotorLostCallback,
        .owner_id = motor,
        .reload_count = 5,
    };
    motor->daemon = DaemonRegister(&daemon_config);

    SMotorEnable(motor);
    smotor_instance[smotor_idx++] = motor;
    return motor;
}

void SMotorSetRef(SMotorInstance *motor, float ref)
{
    if (motor != NULL)
    {
        motor->motor_controller.pid_ref = ref;
        motor->control_mode = (motor->motor_settings.close_loop_type & SPEED_LOOP) ?
                                  SMOTOR_CONTROL_SPEED : SMOTOR_CONTROL_VOLTAGE;
    }
}

void SMotorSetPos(SMotorInstance *motor, float position_rad)
{
    if (motor == NULL)
        return;

    motor->motor_controller.pid_ref = position_rad;
    motor->control_mode = SMOTOR_CONTROL_POSITION;
}

uint8_t SMotorSetZero(SMotorInstance *motor)
{
    if (motor == NULL || !motor->feedback_received)
        return 0;

    motor->zero_angle_rad = motor->measure.total_angle_rad;
    motor->zero_sample_sum_rad = 0.0f;
    motor->zero_sample_count = SMOTOR_ZERO_SAMPLE_COUNT;
    motor->zero_ready = 1;
    motor->measure.position_rad = 0.0f;

    motor->motor_controller.angle_PID.Iout = 0.0f;
    motor->motor_controller.angle_PID.ITerm = 0.0f;
    motor->motor_controller.angle_PID.Last_ITerm = 0.0f;
    return 1;
}

uint8_t SMotorIsZeroReady(const SMotorInstance *motor)
{
    return motor != NULL && motor->zero_ready;
}

uint8_t SMotorConfigureTorque(SMotorInstance *motor, const SMotor_Torque_Config_s *config)
{
    if (motor == NULL || config == NULL || config->max_voltage_v <= 0.0f ||
        config->torque_ratio_nm_per_v <= 0.0f || config->output_ratio < 0.0f)
        return 0;

    motor->torque_config = *config;
    return 1;
}

void SMotorSetTorque(SMotorInstance *motor, float torque_nm)
{
    if (motor == NULL || motor->torque_config.torque_ratio_nm_per_v <= 0.0f)
        return;

    motor->torque_ref_nm = torque_nm;
    motor->control_mode = SMOTOR_CONTROL_TORQUE;
}

void SMotorStop(SMotorInstance *motor)
{
    if (motor != NULL)
        motor->stop_flag = MOTOR_STOP;
}

void SMotorEnable(SMotorInstance *motor)
{
    if (motor != NULL)
        motor->stop_flag = MOTOR_ENALBED;
}

uint8_t SMotorIsOnline(const SMotorInstance *motor)
{
    return motor != NULL && motor->feedback_received && DaemonIsOnline(motor->daemon);
}

static float SMotorCalculateTorqueVoltageMv(SMotorInstance *motor)
{
    const SMotor_Torque_Config_s *config = &motor->torque_config;
    float direction = motor->motor_settings.motor_reverse_flag == MOTOR_DIRECTION_REVERSE ? -1.0f : 1.0f;
    float logical_speed_rad_s = motor->measure.speed_rad_s * direction;
    float voltage_v = motor->torque_ref_nm / config->torque_ratio_nm_per_v * config->output_ratio;

    if (config->calc_rev_volt != NULL)
    {
        if (logical_speed_rad_s >= 0.0f)
            voltage_v += config->calc_rev_volt(logical_speed_rad_s);
        else
            voltage_v -= config->calc_rev_volt(-logical_speed_rad_s);
    }

    LIMIT_MIN_MAX(voltage_v, -config->max_voltage_v, config->max_voltage_v);
    return voltage_v * 1000.0f * direction;
}

void SMotorControl(void)
{
    for (size_t i = 0; i < smotor_idx; ++i)
    {
        SMotorInstance *motor = smotor_instance[i];
        float ref = motor->motor_controller.pid_ref;
        float output_mv;

        if (motor->control_mode != SMOTOR_CONTROL_TORQUE &&
            motor->motor_settings.motor_reverse_flag == MOTOR_DIRECTION_REVERSE)
            ref = -ref;

        if (motor->control_mode == SMOTOR_CONTROL_TORQUE)
        {
            output_mv = SMotorCalculateTorqueVoltageMv(motor);
        }
        else if (motor->control_mode == SMOTOR_CONTROL_POSITION)
        {
            if (!motor->zero_ready)
            {
                output_mv = 0.0f;
            }
            else
            {
                float position_feedback = motor->measure.position_rad;
                if (motor->motor_settings.angle_feedback_source == OTHER_FEED &&
                    motor->motor_controller.other_angle_feedback_ptr != NULL)
                    position_feedback = *motor->motor_controller.other_angle_feedback_ptr;

                float speed_ref = PIDCalculate(&motor->motor_controller.angle_PID,
                                               position_feedback, ref);
                float speed_feedback = motor->measure.speed_rpm;
                if (motor->motor_settings.speed_feedback_source == OTHER_FEED &&
                    motor->motor_controller.other_speed_feedback_ptr != NULL)
                    speed_feedback = *motor->motor_controller.other_speed_feedback_ptr;
                output_mv = PIDCalculate(&motor->motor_controller.speed_PID,
                                         speed_feedback, speed_ref);
            }
        }
        else if (motor->control_mode == SMOTOR_CONTROL_SPEED)
        {
            float speed_feedback = motor->measure.speed_rpm;
            if (motor->motor_settings.speed_feedback_source == OTHER_FEED &&
                motor->motor_controller.other_speed_feedback_ptr != NULL)
                speed_feedback = *motor->motor_controller.other_speed_feedback_ptr;
            output_mv = PIDCalculate(&motor->motor_controller.speed_PID, speed_feedback, ref);
        }
        else
            output_mv = ref;

        if (motor->control_mode != SMOTOR_CONTROL_TORQUE &&
            motor->motor_settings.feedback_reverse_flag == FEEDBACK_DIRECTION_REVERSE)
            output_mv = -output_mv;

        if (motor->control_mode != SMOTOR_CONTROL_TORQUE)
            LIMIT_MIN_MAX(output_mv, -SMOTOR_MAX_VOLTAGE_MV, SMOTOR_MAX_VOLTAGE_MV);
        LIMIT_MIN_MAX(output_mv, -(float)INT16_MAX, (float)INT16_MAX);
        int16_t voltage_mv = motor->stop_flag == MOTOR_STOP ? 0 : (int16_t)output_mv;
        uint16_t raw_voltage = (uint16_t)voltage_mv;
        uint8_t offset = (uint8_t)(motor->message_num * 2);

        /* Smotor firmware copies int16_t directly into CAN data: little endian. */
        smotor_sender[motor->sender_group].tx_buff[offset] = (uint8_t)(raw_voltage & 0xff);
        smotor_sender[motor->sender_group].tx_buff[offset + 1] = (uint8_t)(raw_voltage >> 8);
    }

    for (size_t i = 0; i < SMOTOR_SENDER_GROUP_CNT; ++i)
    {
        if (smotor_sender_enabled[i])
            CANTransmit(&smotor_sender[i], 1);
    }
}
