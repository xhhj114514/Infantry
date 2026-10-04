/* Host integration test of the real chassis adapter and forward kinematics.
 * Build (from repository root, GCC; create build/codex-verify first):
 * gcc -std=c11 -Wall -Wextra -Werror -Itests/chassis_stubs \
 *   -Iapplication/chassis -IModules/motor -IModules/motor/Smotor \
 *   -IModules/wheel_leg tests/chassis_leg_position_test.c \
 *   application/chassis/chassis.c Modules/wheel_leg/leg_position.c \
 *   -lm -o build/codex-verify/chassis_leg_position_test.exe
 * Repeat with -include tests/chassis_stubs/motor_offsets_override.h to test
 * nonzero calibration offsets. No HAL or IMU implementation is linked.
 */
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "chassis.h"
#include "message_center.h"
#include "robot_def.h"
#include "smotor.h"

#define PI_F 3.14159265358979323846f

static int failures;
static unsigned int checks;
#define CHECK(condition) do { ++checks; if (!(condition)) { \
    fprintf(stderr, "%s:%d: failed: %s\n", __FILE__, __LINE__, #condition); \
    ++failures; } } while (0)

CAN_HandleTypeDef hcan2;
static SMotorInstance motors[WHEEL_LEG_MOTOR_COUNT];
static Motor_Init_Config_s registered_config[WHEEL_LEG_MOTOR_COUNT];
static unsigned int stop_calls[WHEEL_LEG_MOTOR_COUNT];
static unsigned int init_calls;
static unsigned int torque_config_calls;
static unsigned int enable_calls;
static unsigned int torque_set_calls;
static unsigned int control_step_calls;
static unsigned int publication_calls;
static unsigned int control_init_calls;
static unsigned int command_calls;
static unsigned int subregister_calls;
static unsigned int subget_calls;
static unsigned int set_position_calls;
static int failed_init_index = -1;
static uint8_t online[WHEEL_LEG_MOTOR_COUNT];
static uint8_t controller_enabled;
static float last_speed_mps;
static float last_yaw_speed_rad_s;
static float last_length_m;
static Publisher_t publisher;
static Chassis_Upload_Data_s last_published;
static WheelLegTelemetry_s controller_telemetry;
static const uint8_t expected_ids[WHEEL_LEG_MOTOR_COUNT] = WHEEL_LEG_MOTOR_IDS;
static const float offsets[WHEEL_LEG_MOTOR_COUNT] = WHEEL_LEG_MOTOR_OFFSETS_RAD;
static const float directions[WHEEL_LEG_MOTOR_COUNT] = WHEEL_LEG_MOTOR_DIRECTIONS;

static int MotorIndex(const SMotorInstance *motor)
{
    for (int i = 0; i < WHEEL_LEG_MOTOR_COUNT; ++i)
        if (motor == &motors[i])
            return i;
    return -1;
}

SMotorInstance *SMotorInit(Motor_Init_Config_s *config)
{
    unsigned int index = init_calls++;
    CHECK(index < WHEEL_LEG_MOTOR_COUNT);
    if (index >= WHEEL_LEG_MOTOR_COUNT)
        return NULL;
    registered_config[index] = *config;
    if ((int)index == failed_init_index)
        return NULL;
    motors[index].motor_id = (uint8_t)config->can_init_config.tx_id;
    motors[index].motor_settings = config->controller_setting_init_config;
    /* Existing nonzero references must never result in enabled output. */
    motors[index].torque_ref_nm = 12.0f;
    motors[index].stop_flag = MOTOR_ENALBED;
    return &motors[index];
}

uint8_t SMotorConfigureTorque(SMotorInstance *motor, const SMotor_Torque_Config_s *config)
{
    CHECK(MotorIndex(motor) >= 0);
    ++torque_config_calls;
    motor->torque_config = *config;
    return 1;
}

void SMotorSetPos(SMotorInstance *motor, float position_rad)
{
    CHECK(MotorIndex(motor) >= 0);
    CHECK(position_rad == CHASSIS_MOTOR_TARGET_POSITION_RAD);
    ++set_position_calls;
    motor->control_mode = SMOTOR_CONTROL_POSITION;
    motor->motor_controller.pid_ref = position_rad;
}

void SMotorStop(SMotorInstance *motor)
{
    int index = MotorIndex(motor);
    if (motor == NULL)
        return;
    CHECK(index >= 0);
    if (index >= 0)
    {
        ++stop_calls[index];
        motor->stop_flag = MOTOR_STOP;
    }
}

void SMotorEnable(SMotorInstance *motor)
{
    ++enable_calls;
    if (motor != NULL)
        motor->stop_flag = MOTOR_ENALBED;
}

void SMotorSetTorque(SMotorInstance *motor, float torque_nm)
{
    ++torque_set_calls;
    if (motor != NULL)
        motor->torque_ref_nm = torque_nm;
}

uint8_t SMotorIsOnline(const SMotorInstance *motor)
{
    int index = MotorIndex(motor);
    return index >= 0 && online[index];
}

float SMotorCalcRevVolt4310(float speed_rad_s) { return speed_rad_s; }
float SMotorCalcRevVolt2805(float speed_rad_s) { return speed_rad_s; }

Publisher_t *PubRegister(char *name, uint8_t data_len)
{
    CHECK(strcmp(name, "chassis_feed") == 0);
    CHECK(data_len == sizeof(Chassis_Upload_Data_s));
    return &publisher;
}

uint8_t PubPushMessage(Publisher_t *pub, void *data)
{
    CHECK(pub == &publisher);
    last_published = *(Chassis_Upload_Data_s *)data;
    ++publication_calls;
    return 1;
}

Subscriber_t *SubRegister(char *name, uint8_t data_len)
{
    (void)name;
    (void)data_len;
    ++subregister_calls;
    return NULL;
}

uint8_t SubGetMessage(Subscriber_t *subscriber, void *data)
{
    (void)subscriber;
    (void)data;
    ++subget_calls;
    return 0;
}

void WheelLegControlInit(void)
{
    ++control_init_calls;
    memset(&controller_telemetry, 0, sizeof(controller_telemetry));
}

void WheelLegSetCommand(float speed_mps, float yaw_speed_rad_s, float leg_length_m)
{
    ++command_calls;
    last_speed_mps = speed_mps;
    last_yaw_speed_rad_s = yaw_speed_rad_s;
    last_length_m = leg_length_m;
    controller_telemetry.target_leg_length_m = leg_length_m;
}

void WheelLegSetEnabled(uint8_t enabled) { controller_enabled = enabled; }

void WheelLegControlStep(const WheelLegFeedback_s *feedback, WheelLegOutput_s *output)
{
    (void)feedback;
    ++control_step_calls;
    memset(output, 0, sizeof(*output));
    output->output_enabled = 1;
    for (int i = 0; i < WHEEL_LEG_MOTOR_COUNT; ++i)
        output->torque_nm[i] = 10.0f;
}

const WheelLegTelemetry_s *WheelLegGetTelemetry(void) { return &controller_telemetry; }

static void CheckNear(float actual, double expected, double tolerance)
{
    CHECK(isfinite(actual));
    CHECK(fabs((double)actual - expected) <= tolerance);
}

static void CheckInvalidPose(const ChassisLegPose_s *pose)
{
    CHECK(pose->valid == 0);
    CHECK(isnan(pose->length_m));
    CHECK(isnan(pose->angle_rad));
}

/* Independent geometry oracle: intersect two radius-0.105 m circles around
 * the ends of the 0.05 m driven links. This uses Cartesian midpoint/normal
 * geometry and double precision, not the generated angle formula. */
static void CheckPose(const ChassisLegPose_s *pose, float phi1, float phi4)
{
    double bx = 0.05 * cos((double)phi1);
    double by = 0.05 * sin((double)phi1);
    double dx = 0.06 + 0.05 * cos((double)phi4);
    double dy = 0.05 * sin((double)phi4);
    double chord_x = dx - bx;
    double chord_y = dy - by;
    double chord_length = hypot(chord_x, chord_y);
    double height = sqrt(0.105 * 0.105 - chord_length * chord_length / 4.0);
    double cx = (bx + dx) / 2.0 - height * chord_y / chord_length;
    double cy = (by + dy) / 2.0 + height * chord_x / chord_length;

    CHECK(pose->valid == 1);
    CheckNear(pose->length_m, hypot(cx - 0.03, cy), 2.0e-6);
    CheckNear(pose->angle_rad, atan2(cy, cx - 0.03), 2.0e-5);
}

static void SetLogicalAngle(int index, float angle)
{
    motors[index].measure.total_angle_rad = offsets[index] + angle / directions[index];
    /* The driver-calibrated position and motor speed are deliberately unrelated. */
    motors[index].measure.position_rad = -91.0f;
    motors[index].measure.speed_rad_s = 900.0f;
}

static void SetLegAngles(float left_phi1, float left_phi4, float right_phi1, float right_phi4)
{
    SetLogicalAngle(WHEEL_LEG_LEFT_JOINT_1, left_phi1);
    SetLogicalAngle(WHEEL_LEG_LEFT_JOINT_0, left_phi4);
    SetLogicalAngle(WHEEL_LEG_RIGHT_JOINT_1, right_phi1);
    SetLogicalAngle(WHEEL_LEG_RIGHT_JOINT_0, right_phi4);
}

static void CheckStopped(void)
{
    CHECK(enable_calls == 0);
    CHECK(torque_set_calls == 0);
    CHECK(control_step_calls == 0);
    for (int i = 0; i < WHEEL_LEG_MOTOR_COUNT; ++i)
        if (i != failed_init_index)
            CHECK(motors[i].stop_flag == MOTOR_STOP);
}

static void RunTask(void)
{
    unsigned int old_stop_calls[WHEEL_LEG_MOTOR_COUNT];
    unsigned int old_publication_calls = publication_calls;
    memcpy(old_stop_calls, stop_calls, sizeof(stop_calls));
    /* Simulate an unrelated previous state; each task must actively stop motors. */
    for (int i = 0; i < WHEEL_LEG_MOTOR_COUNT; ++i)
        motors[i].stop_flag = MOTOR_ENALBED;
    ChassisTask();
    for (int i = 0; i < WHEEL_LEG_MOTOR_COUNT; ++i)
        if (i != failed_init_index)
            CHECK(stop_calls[i] > old_stop_calls[i]);
    CHECK(publication_calls == old_publication_calls + 1);
    CHECK(last_published.power_flag == 0);
    CheckStopped();
}

static void CheckRegistration(void)
{
    CHECK(init_calls == WHEEL_LEG_MOTOR_COUNT);
    CHECK(torque_config_calls == WHEEL_LEG_MOTOR_COUNT);
    CHECK(set_position_calls == WHEEL_LEG_MOTOR_COUNT);
    CHECK(control_init_calls == 1);
    CHECK(command_calls == 1);
    CHECK(controller_enabled == 0);
    CHECK(last_speed_mps == 0.0f);
    CHECK(last_yaw_speed_rad_s == 0.0f);
    CheckNear(last_length_m, WHEEL_LEG_DEFAULT_LENGTH_M, 1.0e-7);
    for (int i = 0; i < WHEEL_LEG_MOTOR_COUNT; ++i)
    {
        const Motor_Init_Config_s *config = &registered_config[i];
        const SMotor_Torque_Config_s *torque = &motors[i].torque_config;
        int is_wheel = i == WHEEL_LEG_LEFT_WHEEL || i == WHEEL_LEG_RIGHT_WHEEL;
        float voltage_v = is_wheel ? WHEEL_LEG_WHEEL_MAX_VOLTAGE_V : WHEEL_LEG_JOINT_MAX_VOLTAGE_V;
        CHECK(config->can_init_config.can_handle == &hcan2);
        CHECK(config->can_init_config.tx_id == expected_ids[i]);
        CHECK(config->controller_setting_init_config.outer_loop_type == ANGLE_LOOP);
        CHECK(config->controller_setting_init_config.close_loop_type == (ANGLE_LOOP | SPEED_LOOP));
        CHECK(config->controller_setting_init_config.motor_reverse_flag ==
              (directions[i] < 0.0f ? MOTOR_DIRECTION_REVERSE : MOTOR_DIRECTION_NORMAL));
        CHECK(config->controller_param_init_config.angle_PID.Kp == CHASSIS_MOTOR_POSITION_PID_KP);
        CHECK(config->controller_param_init_config.angle_PID.Ki == CHASSIS_MOTOR_POSITION_PID_KI);
        CHECK(config->controller_param_init_config.angle_PID.Kd == CHASSIS_MOTOR_POSITION_PID_KD);
        CHECK(config->controller_param_init_config.angle_PID.MaxOut == CHASSIS_MOTOR_POSITION_MAX_SPEED_RPM);
        CHECK(config->controller_param_init_config.speed_PID.Kp == CHASSIS_MOTOR_SPEED_PID_KP);
        CHECK(config->controller_param_init_config.speed_PID.Ki == CHASSIS_MOTOR_SPEED_PID_KI);
        CHECK(config->controller_param_init_config.speed_PID.Kd == CHASSIS_MOTOR_SPEED_PID_KD);
        CheckNear(config->controller_param_init_config.speed_PID.MaxOut, voltage_v * 1000.0f, 0.001);
        CHECK(torque->max_voltage_v == voltage_v);
        CheckNear(torque->torque_ratio_nm_per_v,
                  is_wheel ? WHEEL_LEG_WHEEL_TORQUE_RATIO_NM_PER_V : WHEEL_LEG_JOINT_TORQUE_RATIO_NM_PER_V,
                  1.0e-7);
        CheckNear(torque->output_ratio, WHEEL_LEG_MOTOR_OUTPUT_RATIO, 1.0e-7);
        CHECK(torque->calc_rev_volt == (is_wheel ? SMotorCalcRevVolt2805 : SMotorCalcRevVolt4310));
    }
    CheckStopped();
}

static void CheckNullRegistration(void)
{
    init_calls = torque_config_calls = set_position_calls = 0;
    memset(online, 1, sizeof(online));
    failed_init_index = WHEEL_LEG_LEFT_JOINT_0;
    ChassisInit();
    CHECK(init_calls == WHEEL_LEG_MOTOR_COUNT);
    CHECK(torque_config_calls == WHEEL_LEG_MOTOR_COUNT - 1);
    SetLegAngles(0.8f, 0.2f, 1.0f, -0.35f);
    ChassisEnable(1);
    RunTask();
    CHECK((ChassisGetLegPosition()->motor_online_mask & (1U << failed_init_index)) == 0);
    CHECK(isnan(ChassisGetLegPosition()->motor_angle_rad[failed_init_index]));
    CheckInvalidPose(&ChassisGetLegPosition()->left_leg);
    CheckPose(&ChassisGetLegPosition()->right_leg, 1.0f, -0.35f);
}

int main(void)
{
    const ChassisLegPosition_s *position;
    const float poses[][4] = {
        {0.0f, 0.0f, PI_F / 2.0f, PI_F / 2.0f},
        {0.8f, 0.2f, 1.0f, -0.35f},
        {0.3f, 1.2f, -PI_F / 2.0f, -PI_F / 2.0f},
        {PI_F / 2.0f, 0.0f, 0.0f, PI_F / 2.0f},
    };

    ChassisInit();
    CheckRegistration();
    position = ChassisGetLegPosition();
    CHECK(position != NULL);
    CHECK(position->motor_online_mask == 0);
    CheckInvalidPose(&position->left_leg);
    CheckInvalidPose(&position->right_leg);
    for (int i = 0; i < WHEEL_LEG_MOTOR_COUNT; ++i)
        CHECK(isnan(position->motor_angle_rad[i]));
    RunTask();

    memset(online, 1, sizeof(online));
    for (unsigned int sample = 0; sample < sizeof(poses) / sizeof(poses[0]); ++sample)
    {
        const float *angles = poses[sample];
        SetLegAngles(angles[0], angles[1], angles[2], angles[3]);
        SetLogicalAngle(WHEEL_LEG_LEFT_WHEEL, 0.71f);
        SetLogicalAngle(WHEEL_LEG_RIGHT_WHEEL, -0.82f);
        RunTask();
        CHECK(position->motor_online_mask == 0x3f);
        CheckPose(&position->left_leg, angles[0], angles[1]);
        CheckPose(&position->right_leg, angles[2], angles[3]);
        CheckNear(position->motor_angle_rad[WHEEL_LEG_LEFT_JOINT_1], angles[0], 1.0e-6);
        CheckNear(position->motor_angle_rad[WHEEL_LEG_LEFT_JOINT_0], angles[1], 1.0e-6);
        CheckNear(position->motor_angle_rad[WHEEL_LEG_RIGHT_JOINT_1], angles[2], 1.0e-6);
        CheckNear(position->motor_angle_rad[WHEEL_LEG_RIGHT_JOINT_0], angles[3], 1.0e-6);
        CheckNear(position->motor_angle_rad[WHEEL_LEG_LEFT_WHEEL], 0.71f, 1.0e-6);
        CheckNear(position->motor_angle_rad[WHEEL_LEG_RIGHT_WHEEL], -0.82f, 1.0e-6);
    }
    /* Fixed asymmetric reference values also detect swapped joint inputs. */
    CheckNear(position->left_leg.length_m, 0.1196279197, 2.0e-6);
    CheckNear(position->left_leg.angle_rad, 1.0402091753, 2.0e-5);
    CheckNear(position->right_leg.length_m, 0.0873487870, 2.0e-6);
    CheckNear(position->right_leg.angle_rad, 2.6007009037, 2.0e-5);

    /* Wheel feedback and the absent IMU cannot invalidate either leg. */
    online[WHEEL_LEG_LEFT_WHEEL] = online[WHEEL_LEG_RIGHT_WHEEL] = 0;
    RunTask();
    CHECK(position->motor_online_mask == 0x1b);
    CHECK(isnan(position->motor_angle_rad[WHEEL_LEG_LEFT_WHEEL]));
    CHECK(isnan(position->motor_angle_rad[WHEEL_LEG_RIGHT_WHEEL]));
    CheckPose(&position->left_leg, PI_F / 2.0f, 0.0f);
    CheckPose(&position->right_leg, 0.0f, PI_F / 2.0f);

    /* Each joint loss invalidates only its own leg and clears stale data. */
    for (int joint = 0; joint < WHEEL_LEG_MOTOR_COUNT; ++joint)
    {
        if (joint == WHEEL_LEG_LEFT_WHEEL || joint == WHEEL_LEG_RIGHT_WHEEL)
            continue;
        online[joint] = 0;
        RunTask();
        CHECK(position->motor_online_mask == (uint8_t)(0x1b & ~(1U << joint)));
        CHECK(isnan(position->motor_angle_rad[joint]));
        if (joint <= WHEEL_LEG_LEFT_JOINT_1)
        {
            CheckInvalidPose(&position->left_leg);
            CheckPose(&position->right_leg, 0.0f, PI_F / 2.0f);
        }
        else
        {
            CheckInvalidPose(&position->right_leg);
            CheckPose(&position->left_leg, PI_F / 2.0f, 0.0f);
        }
        online[joint] = 1;
        RunTask();
        CheckPose(&position->left_leg, PI_F / 2.0f, 0.0f);
        CheckPose(&position->right_leg, 0.0f, PI_F / 2.0f);
    }

    motors[WHEEL_LEG_LEFT_JOINT_1].measure.total_angle_rad = NAN;
    RunTask();
    CHECK(position->motor_online_mask == 0x1b);
    CheckInvalidPose(&position->left_leg);
    CheckPose(&position->right_leg, 0.0f, PI_F / 2.0f);
    SetLogicalAngle(WHEEL_LEG_LEFT_JOINT_1, PI_F / 2.0f);
    motors[WHEEL_LEG_RIGHT_JOINT_0].measure.total_angle_rad = INFINITY;
    RunTask();
    CheckPose(&position->left_leg, PI_F / 2.0f, 0.0f);
    CheckInvalidPose(&position->right_leg);
    SetLogicalAngle(WHEEL_LEG_RIGHT_JOINT_0, PI_F / 2.0f);
    RunTask();
    CheckPose(&position->right_leg, 0.0f, PI_F / 2.0f);

    ChassisSetCommand(0.4f, -0.3f, 0.1f);
    CheckNear(last_speed_mps, 0.4f, 1.0e-7);
    CheckNear(last_yaw_speed_rad_s, -0.3f, 1.0e-7);
    CheckNear(last_length_m, 0.1f, 1.0e-7);
    ChassisEnable(1);
    CHECK(controller_enabled == 1);
    CheckStopped();
    RunTask();
    RunTask();
    ChassisEnable(0);
    CHECK(controller_enabled == 0);
    RunTask();

    memset(online, 0, sizeof(online));
    RunTask();
    CHECK(position->motor_online_mask == 0);
    CheckInvalidPose(&position->left_leg);
    CheckInvalidPose(&position->right_leg);
    CHECK(subregister_calls == 0);
    CHECK(subget_calls == 0);
    CheckNullRegistration();

    printf("chassis leg position: %u checks, %d failures\n", checks, failures);
    return failures == 0 ? 0 : 1;
}
