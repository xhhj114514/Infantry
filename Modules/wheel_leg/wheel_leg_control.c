#include "wheel_leg_control.h"

#include "general_def.h"
#include "wheel_leg_math.h"

#include <math.h>
#include <string.h>

#define WHEEL_RADIUS_M 0.0325f
#define LEG_MASS_KG 0.062f
#define GRAVITY_MPS2 9.8f
#define PROTECTION_ANGLE_RAD (PI * 0.25f)
#define PROTECTION_RECOVERY_MS 4000U

typedef struct
{
    float kp;
    float ki;
    float kd;
    float error;
    float last_error;
    float integral;
    float max_integral;
    float output;
    float max_output;
    float deadzone;
    float error_lpf_ratio;
} WheelLegPID_s;

typedef struct
{
    WheelLegPID_s inner;
    WheelLegPID_s outer;
    float output;
} WheelLegCascadePID_s;

typedef struct
{
    WheelLegCascadePID_s leg_angle_pid;
    WheelLegCascadePID_s leg_length_pid;
    WheelLegCascadePID_s yaw_pid;
    WheelLegCascadePID_s roll_pid;
    WheelLegTelemetry_s telemetry;
    float speed_command_mps;
    float yaw_speed_command_rad_s;
    float leg_length_command_m;
    float last_left_length_speed;
    float last_right_length_speed;
    uint32_t last_touch_time_ms;
    uint32_t protection_safe_since_ms;
    uint8_t enabled;
    uint8_t initialized;
    uint8_t startup_standup;
    uint8_t protection_active;
} WheelLegController_s;

static WheelLegController_s controller;

static float Clamp(float value, float minimum, float maximum)
{
    if (value < minimum)
        return minimum;
    if (value > maximum)
        return maximum;
    return value;
}

static void PIDInit(WheelLegPID_s *pid, float kp, float ki, float kd, float max_integral, float max_output)
{
    memset(pid, 0, sizeof(*pid));
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->max_integral = max_integral;
    pid->max_output = max_output;
    pid->error_lpf_ratio = 1.0f;
}

static void PIDClear(WheelLegPID_s *pid)
{
    pid->error = 0.0f;
    pid->last_error = 0.0f;
    pid->integral = 0.0f;
    pid->output = 0.0f;
}

static void CascadePIDClear(WheelLegCascadePID_s *pid)
{
    PIDClear(&pid->inner);
    PIDClear(&pid->outer);
    pid->output = 0.0f;
}

static float PIDCalculate(WheelLegPID_s *pid, float reference, float feedback)
{
    pid->last_error = pid->error;
    pid->error = fabsf(reference - feedback) < pid->deadzone ? 0.0f : reference - feedback;
    pid->error = pid->error * pid->error_lpf_ratio + pid->last_error * (1.0f - pid->error_lpf_ratio);
    pid->output = (pid->error - pid->last_error) * pid->kd + pid->error * pid->kp;
    pid->integral = Clamp(pid->integral + pid->error * pid->ki,
                          -pid->max_integral, pid->max_integral);
    pid->output = Clamp(pid->output + pid->integral, -pid->max_output, pid->max_output);
    return pid->output;
}

static float CascadePIDCalculate(WheelLegCascadePID_s *pid, float position_reference,
                                 float position_feedback, float speed_feedback)
{
    const float speed_reference = PIDCalculate(&pid->outer, position_reference, position_feedback);
    pid->output = PIDCalculate(&pid->inner, speed_reference, speed_feedback);
    return pid->output;
}

static void ClearAllPIDs(void)
{
    CascadePIDClear(&controller.leg_angle_pid);
    CascadePIDClear(&controller.leg_length_pid);
    CascadePIDClear(&controller.yaw_pid);
    CascadePIDClear(&controller.roll_pid);
}

static void ZeroOutput(WheelLegOutput_s *output)
{
    memset(output, 0, sizeof(*output));
}

static uint8_t ValuesAreFinite(const WheelLegPose_s *left, const WheelLegPose_s *right)
{
    return isfinite(left->angle_rad) && isfinite(left->length_m) &&
           isfinite(left->angular_speed_rad_s) && isfinite(left->length_speed_mps) &&
           isfinite(left->length_accel_mps2) && isfinite(right->angle_rad) &&
           isfinite(right->length_m) && isfinite(right->angular_speed_rad_s) &&
           isfinite(right->length_speed_mps) && isfinite(right->length_accel_mps2);
}

static uint8_t UpdateLegPose(const WheelLegFeedback_s *feedback)
{
    WheelLegPose_s *left = &controller.telemetry.left_leg;
    WheelLegPose_s *right = &controller.telemetry.right_leg;
    float position[2];
    float speed[2];

    WheelLegPosition(feedback->angle_rad[WHEEL_LEG_LEFT_JOINT_1],
                     feedback->angle_rad[WHEEL_LEG_LEFT_JOINT_0], position);
    left->length_m = position[0];
    left->angle_rad = position[1];
    WheelLegSpeed(feedback->speed_rad_s[WHEEL_LEG_LEFT_JOINT_1],
                  feedback->speed_rad_s[WHEEL_LEG_LEFT_JOINT_0],
                  feedback->angle_rad[WHEEL_LEG_LEFT_JOINT_1],
                  feedback->angle_rad[WHEEL_LEG_LEFT_JOINT_0], speed);
    left->length_speed_mps = speed[0];
    left->angular_speed_rad_s = speed[1];

    WheelLegPosition(feedback->angle_rad[WHEEL_LEG_RIGHT_JOINT_1],
                     feedback->angle_rad[WHEEL_LEG_RIGHT_JOINT_0], position);
    right->length_m = position[0];
    right->angle_rad = position[1];
    WheelLegSpeed(feedback->speed_rad_s[WHEEL_LEG_RIGHT_JOINT_1],
                  feedback->speed_rad_s[WHEEL_LEG_RIGHT_JOINT_0],
                  feedback->angle_rad[WHEEL_LEG_RIGHT_JOINT_1],
                  feedback->angle_rad[WHEEL_LEG_RIGHT_JOINT_0], speed);
    right->length_speed_mps = speed[0];
    right->angular_speed_rad_s = speed[1];

    if (!controller.initialized)
    {
        controller.last_left_length_speed = left->length_speed_mps;
        controller.last_right_length_speed = right->length_speed_mps;
    }
    left->length_accel_mps2 =
        ((left->length_speed_mps - controller.last_left_length_speed) / WHEEL_LEG_CONTROL_PERIOD_S) * 0.5f +
        left->length_accel_mps2 * 0.5f;
    right->length_accel_mps2 =
        ((right->length_speed_mps - controller.last_right_length_speed) / WHEEL_LEG_CONTROL_PERIOD_S) * 0.5f +
        right->length_accel_mps2 * 0.5f;
    controller.last_left_length_speed = left->length_speed_mps;
    controller.last_right_length_speed = right->length_speed_mps;

    return ValuesAreFinite(left, right);
}

static void UpdateState(const WheelLegFeedback_s *feedback)
{
    WheelLegState_s *state = &controller.telemetry.state;
    const WheelLegPose_s *left = &controller.telemetry.left_leg;
    const WheelLegPose_s *right = &controller.telemetry.right_leg;

    state->pitch = feedback->pitch_rad;
    state->pitch_speed = feedback->pitch_speed_rad_s;
    state->position_m =
        (feedback->angle_rad[WHEEL_LEG_LEFT_WHEEL] + feedback->angle_rad[WHEEL_LEG_RIGHT_WHEEL]) *
        0.5f * WHEEL_RADIUS_M;
    state->speed_mps =
        (feedback->speed_rad_s[WHEEL_LEG_LEFT_WHEEL] + feedback->speed_rad_s[WHEEL_LEG_RIGHT_WHEEL]) *
        0.5f * WHEEL_RADIUS_M;
    state->theta = (left->angle_rad + right->angle_rad) * 0.5f - PI * 0.5f - feedback->pitch_rad;
    state->theta_speed = (left->angular_speed_rad_s + right->angular_speed_rad_s) * 0.5f -
                         feedback->pitch_speed_rad_s;
}

static void ResetAroundCurrentState(const WheelLegFeedback_s *feedback)
{
    controller.telemetry.target_position_m = controller.telemetry.state.position_m;
    controller.telemetry.target_speed_mps = 0.0f;
    controller.telemetry.target_yaw_rad = feedback->yaw_rad;
    controller.telemetry.target_leg_length_m = controller.leg_length_command_m;
    controller.last_touch_time_ms = feedback->now_ms;
    controller.telemetry.touching_ground = 1;
    controller.telemetry.cushioning = 0;
    controller.startup_standup = 1;
    ClearAllPIDs();
}

static void UpdateTargets(void)
{
    WheelLegTelemetry_s *telemetry = &controller.telemetry;
    const float leg_length = (telemetry->left_leg.length_m + telemetry->right_leg.length_m) * 0.5f;
    const float speed_slope_step = -(leg_length - 0.07f) * 0.02f + 0.002f;
    const float speed_delta = controller.speed_command_mps - telemetry->target_speed_mps;

    if (fabsf(speed_delta) < speed_slope_step)
        telemetry->target_speed_mps = controller.speed_command_mps;
    else
        telemetry->target_speed_mps += speed_delta > 0.0f ? speed_slope_step : -speed_slope_step;

    telemetry->target_position_m += telemetry->target_speed_mps * WHEEL_LEG_CONTROL_PERIOD_S;
    telemetry->target_position_m = Clamp(telemetry->target_position_m,
                                         telemetry->state.position_m - 0.1f,
                                         telemetry->state.position_m + 0.1f);
    telemetry->target_speed_mps = Clamp(telemetry->target_speed_mps,
                                        telemetry->state.speed_mps - 0.3f,
                                        telemetry->state.speed_mps + 0.3f);
    telemetry->target_yaw_rad += controller.yaw_speed_command_rad_s * WHEEL_LEG_CONTROL_PERIOD_S;
    telemetry->target_leg_length_m = controller.leg_length_command_m;
}

static void UpdateGroundDetector(const WheelLegFeedback_s *feedback, float left_force, float right_force)
{
    WheelLegTelemetry_s *telemetry = &controller.telemetry;
    telemetry->left_support_force_n = left_force + LEG_MASS_KG * GRAVITY_MPS2 -
                                      LEG_MASS_KG * (telemetry->left_leg.length_accel_mps2 -
                                                     feedback->vertical_accel_mps2);
    telemetry->right_support_force_n = right_force + LEG_MASS_KG * GRAVITY_MPS2 -
                                       LEG_MASS_KG * (telemetry->right_leg.length_accel_mps2 -
                                                      feedback->vertical_accel_mps2);

    uint8_t touching = telemetry->left_support_force_n > 3.0f && telemetry->right_support_force_n > 3.0f;
    if (!touching && (uint32_t)(feedback->now_ms - controller.last_touch_time_ms) < 1000U)
        touching = 1;
    if (!telemetry->touching_ground && touching)
    {
        telemetry->target_position_m = telemetry->state.position_m;
        telemetry->cushioning = 1;
        controller.last_touch_time_ms = feedback->now_ms;
    }
    if (telemetry->cushioning &&
        (telemetry->left_leg.length_m + telemetry->right_leg.length_m) * 0.5f <
            telemetry->target_leg_length_m)
        telemetry->cushioning = 0;
    telemetry->touching_ground = touching;
}

static uint8_t ApplyProtection(const WheelLegFeedback_s *feedback, WheelLegOutput_s *output)
{
    const float left_theta = controller.telemetry.left_leg.angle_rad - feedback->pitch_rad - PI * 0.5f;
    const float right_theta = controller.telemetry.right_leg.angle_rad - feedback->pitch_rad - PI * 0.5f;
    const uint8_t outside = left_theta < -PROTECTION_ANGLE_RAD || left_theta > PROTECTION_ANGLE_RAD ||
                            right_theta < -PROTECTION_ANGLE_RAD || right_theta > PROTECTION_ANGLE_RAD ||
                            feedback->pitch_rad > PROTECTION_ANGLE_RAD ||
                            feedback->pitch_rad < -PROTECTION_ANGLE_RAD;

    if (outside && !controller.startup_standup)
    {
        controller.protection_active = 1;
        controller.protection_safe_since_ms = 0;
    }
    else if (controller.startup_standup)
    {
        if (!outside || left_theta < -PROTECTION_ANGLE_RAD || right_theta > PROTECTION_ANGLE_RAD)
            controller.startup_standup = 0;
    }

    if (!controller.protection_active)
        return 0;

    if (outside)
    {
        controller.protection_safe_since_ms = 0;
    }
    else if (controller.protection_safe_since_ms == 0U)
    {
        controller.protection_safe_since_ms = feedback->now_ms;
    }
    else if ((uint32_t)(feedback->now_ms - controller.protection_safe_since_ms) >= PROTECTION_RECOVERY_MS)
    {
        controller.protection_active = 0;
        ResetAroundCurrentState(feedback);
        ZeroOutput(output);
        return 1;
    }

    controller.telemetry.fault_flags |= WHEEL_LEG_FAULT_PROTECTION;
    ZeroOutput(output);
    return 1;
}

void WheelLegControlInit(void)
{
    memset(&controller, 0, sizeof(controller));
    controller.enabled = 1;
    controller.leg_length_command_m = WHEEL_LEG_DEFAULT_LENGTH_M;

    PIDInit(&controller.roll_pid.inner, 0.7f, 0.0f, 2.5f, 0.0f, 5.0f);
    PIDInit(&controller.roll_pid.outer, 10.0f, 0.0f, 0.0f, 0.0f, 3.0f);
    controller.roll_pid.inner.error_lpf_ratio = 0.1f;

    PIDInit(&controller.yaw_pid.inner, 0.01f, 0.0f, 0.0f, 0.0f, 0.1f);
    PIDInit(&controller.yaw_pid.outer, 10.0f, 0.0f, 0.0f, 0.0f, 2.0f);

    PIDInit(&controller.leg_length_pid.inner, 10.0f, 1.0f, 30.0f, 2.0f, 10.0f);
    PIDInit(&controller.leg_length_pid.outer, 5.0f, 0.0f, 0.0f, 0.0f, 0.5f);
    controller.leg_length_pid.inner.error_lpf_ratio = 0.5f;

    PIDInit(&controller.leg_angle_pid.inner, 0.08f, 0.0f, 0.12f, 0.0f, 1.0f);
    PIDInit(&controller.leg_angle_pid.outer, 12.0f, 0.0f, 0.0f, 0.0f, 20.0f);
    controller.leg_angle_pid.outer.error_lpf_ratio = 0.5f;
}

void WheelLegSetCommand(float speed_mps, float yaw_speed_rad_s, float leg_length_m)
{
    controller.speed_command_mps = Clamp(speed_mps, -WHEEL_LEG_MAX_SPEED_MPS, WHEEL_LEG_MAX_SPEED_MPS);
    controller.yaw_speed_command_rad_s = Clamp(yaw_speed_rad_s,
                                               -WHEEL_LEG_MAX_YAW_SPEED_RAD_S,
                                               WHEEL_LEG_MAX_YAW_SPEED_RAD_S);
    controller.leg_length_command_m = Clamp(leg_length_m,
                                            WHEEL_LEG_MIN_LENGTH_M,
                                            WHEEL_LEG_MAX_LENGTH_M);
}

void WheelLegSetEnabled(uint8_t enabled)
{
    controller.enabled = enabled != 0U;
    if (!controller.enabled)
    {
        controller.initialized = 0;
        controller.protection_active = 0;
        controller.protection_safe_since_ms = 0;
        ClearAllPIDs();
    }
}

void WheelLegControlStep(const WheelLegFeedback_s *feedback, WheelLegOutput_s *output)
{
    ZeroOutput(output);
    controller.telemetry.fault_flags = WHEEL_LEG_FAULT_NONE;

    if (feedback == NULL || !controller.enabled || !feedback->sensors_ready)
    {
        controller.telemetry.fault_flags |= WHEEL_LEG_FAULT_NOT_READY;
        controller.initialized = 0;
        ClearAllPIDs();
        return;
    }

    if (!UpdateLegPose(feedback))
    {
        controller.telemetry.fault_flags |= WHEEL_LEG_FAULT_KINEMATICS;
        controller.initialized = 0;
        ClearAllPIDs();
        return;
    }
    UpdateState(feedback);

    if (!controller.initialized)
    {
        ResetAroundCurrentState(feedback);
        controller.initialized = 1;
        return;
    }

    UpdateTargets();

    WheelLegTelemetry_s *telemetry = &controller.telemetry;
    const float leg_length = (telemetry->left_leg.length_m + telemetry->right_leg.length_m) * 0.5f;
    const float leg_length_speed =
        (telemetry->left_leg.length_speed_mps + telemetry->right_leg.length_speed_mps) * 0.5f;
    float scheduled_gain[12];
    float gain[2][6] = {{0.0f}};
    WheelLegLQRGain(leg_length, scheduled_gain);

    if (telemetry->touching_ground)
    {
        for (uint8_t state_index = 0; state_index < 6; ++state_index)
        {
            gain[0][state_index] = scheduled_gain[state_index * 2];
            gain[1][state_index] = scheduled_gain[state_index * 2 + 1];
        }
    }
    else
    {
        gain[1][0] = scheduled_gain[1] * -2.0f;
        gain[1][1] = scheduled_gain[3] * -10.0f;
    }

    float state_vector[6] = {
        telemetry->state.theta,
        telemetry->state.theta_speed,
        telemetry->state.position_m - telemetry->target_position_m,
        telemetry->state.speed_mps - telemetry->target_speed_mps,
        telemetry->state.pitch,
        telemetry->state.pitch_speed,
    };
    float lqr_wheel_torque = 0.0f;
    float lqr_hip_torque = 0.0f;
    for (uint8_t i = 0; i < 6; ++i)
    {
        lqr_wheel_torque += gain[0][i] * state_vector[i];
        lqr_hip_torque += gain[1][i] * state_vector[i];
    }

    const float yaw_output = CascadePIDCalculate(&controller.yaw_pid,
                                                 telemetry->target_yaw_rad,
                                                 feedback->yaw_rad,
                                                 feedback->yaw_speed_rad_s);
    if (telemetry->touching_ground)
    {
        output->torque_nm[WHEEL_LEG_LEFT_WHEEL] = -lqr_wheel_torque * 0.3f - yaw_output;
        output->torque_nm[WHEEL_LEG_RIGHT_WHEEL] = -lqr_wheel_torque * 0.3f + yaw_output;
    }

    const float length_reference = telemetry->touching_ground && !telemetry->cushioning ?
                                       telemetry->target_leg_length_m : 0.12f;
    const float length_output = CascadePIDCalculate(&controller.leg_length_pid,
                                                    length_reference, leg_length, leg_length_speed);
    const float roll_output = CascadePIDCalculate(&controller.roll_pid, 0.0f,
                                                  feedback->roll_rad, feedback->roll_speed_rad_s);
    float left_force = length_output +
                       (telemetry->touching_ground && !telemetry->cushioning ? 6.0f - roll_output : 0.0f);
    float right_force = length_output +
                        (telemetry->touching_ground && !telemetry->cushioning ? 6.0f + roll_output : 0.0f);
    if (telemetry->left_leg.length_m > 0.12f)
        left_force -= (telemetry->left_leg.length_m - 0.12f) * 100.0f;
    if (telemetry->right_leg.length_m > 0.12f)
        right_force -= (telemetry->right_leg.length_m - 0.12f) * 100.0f;

    UpdateGroundDetector(feedback, left_force, right_force);

    const float leg_angle_output = CascadePIDCalculate(
        &controller.leg_angle_pid, 0.0f,
        telemetry->left_leg.angle_rad - telemetry->right_leg.angle_rad,
        telemetry->left_leg.angular_speed_rad_s - telemetry->right_leg.angular_speed_rad_s);
    const float left_hip_torque = lqr_hip_torque -
                                  leg_angle_output * (telemetry->left_leg.length_m / 0.07f);
    const float right_hip_torque = lqr_hip_torque +
                                   leg_angle_output * (telemetry->right_leg.length_m / 0.07f);
    float left_joint_torque[2];
    float right_joint_torque[2];
    WheelLegVMC(left_force, left_hip_torque,
                feedback->angle_rad[WHEEL_LEG_LEFT_JOINT_1],
                feedback->angle_rad[WHEEL_LEG_LEFT_JOINT_0], left_joint_torque);
    WheelLegVMC(right_force, right_hip_torque,
                feedback->angle_rad[WHEEL_LEG_RIGHT_JOINT_1],
                feedback->angle_rad[WHEEL_LEG_RIGHT_JOINT_0], right_joint_torque);
    output->torque_nm[WHEEL_LEG_LEFT_JOINT_0] = -left_joint_torque[0];
    output->torque_nm[WHEEL_LEG_LEFT_JOINT_1] = -left_joint_torque[1];
    output->torque_nm[WHEEL_LEG_RIGHT_JOINT_0] = -right_joint_torque[0];
    output->torque_nm[WHEEL_LEG_RIGHT_JOINT_1] = -right_joint_torque[1];

    for (uint8_t i = 0; i < WHEEL_LEG_MOTOR_COUNT; ++i)
    {
        if (!isfinite(output->torque_nm[i]))
        {
            controller.telemetry.fault_flags |= WHEEL_LEG_FAULT_KINEMATICS;
            ZeroOutput(output);
            return;
        }
    }
    output->output_enabled = 1;
    ApplyProtection(feedback, output);
}

const WheelLegTelemetry_s *WheelLegGetTelemetry(void)
{
    return &controller.telemetry;
}
