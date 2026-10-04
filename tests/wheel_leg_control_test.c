#include <math.h>
#include <stdio.h>
#include <string.h>

#include "wheel_leg_control.h"
#include "wheel_leg_math.h"

static int FindStandingJointAngles(float *phi1, float *phi4)
{
    float best_error = 1000.0f;
    for (int i = 0; i <= 120; ++i)
    {
        const float candidate_phi1 = -3.0f + 0.05f * (float)i;
        for (int j = 0; j <= 120; ++j)
        {
            const float candidate_phi4 = -3.0f + 0.05f * (float)j;
            float position[2];
            WheelLegPosition(candidate_phi1, candidate_phi4, position);
            if (!isfinite(position[0]) || !isfinite(position[1]))
                continue;
            const float error = fabsf(position[0] - WHEEL_LEG_DEFAULT_LENGTH_M) +
                                0.02f * fabsf(position[1] - 1.5707963268f);
            if (error < best_error)
            {
                best_error = error;
                *phi1 = candidate_phi1;
                *phi4 = candidate_phi4;
            }
        }
    }
    return best_error < 0.01f;
}

static int OutputIsFinite(const WheelLegOutput_s *output)
{
    for (int i = 0; i < WHEEL_LEG_MOTOR_COUNT; ++i)
    {
        if (!isfinite(output->torque_nm[i]))
            return 0;
    }
    return 1;
}

int main(void)
{
    WheelLegFeedback_s feedback;
    WheelLegOutput_s output;
    float phi1 = 0.0f;
    float phi4 = 0.0f;
    int failures = 0;

    if (!FindStandingJointAngles(&phi1, &phi4))
    {
        puts("unable to find a valid standing pose");
        return 1;
    }

    memset(&feedback, 0, sizeof(feedback));
    feedback.angle_rad[WHEEL_LEG_LEFT_JOINT_0] = phi4;
    feedback.angle_rad[WHEEL_LEG_LEFT_JOINT_1] = phi1;
    feedback.angle_rad[WHEEL_LEG_RIGHT_JOINT_0] = phi4;
    feedback.angle_rad[WHEEL_LEG_RIGHT_JOINT_1] = phi1;

    WheelLegControlInit();
    WheelLegControlStep(&feedback, &output);
    failures += output.output_enabled != 0;
    failures += (WheelLegGetTelemetry()->fault_flags & WHEEL_LEG_FAULT_NOT_READY) == 0;

    feedback.sensors_ready = 1;
    feedback.now_ms = 1000;
    WheelLegControlStep(&feedback, &output);
    failures += output.output_enabled != 0; // First valid sample only centers the controller.

    feedback.now_ms += 4;
    WheelLegControlStep(&feedback, &output);
    failures += output.output_enabled == 0;
    failures += !OutputIsFinite(&output);

    feedback.pitch_rad = 1.0f;
    feedback.now_ms += 4;
    WheelLegControlStep(&feedback, &output);
    failures += output.output_enabled != 0;
    failures += (WheelLegGetTelemetry()->fault_flags & WHEEL_LEG_FAULT_PROTECTION) == 0;

    feedback.pitch_rad = 0.0f;
    feedback.now_ms += 4;
    WheelLegControlStep(&feedback, &output);
    failures += output.output_enabled != 0;
    feedback.now_ms += 4000;
    WheelLegControlStep(&feedback, &output);
    failures += output.output_enabled != 0; // Recovery cycle is intentionally held at zero.
    feedback.now_ms += 4;
    WheelLegControlStep(&feedback, &output);
    failures += output.output_enabled == 0;

    feedback.sensors_ready = 0;
    feedback.now_ms += 4;
    WheelLegControlStep(&feedback, &output);
    failures += output.output_enabled != 0;

    feedback.sensors_ready = 1;
    WheelLegSetEnabled(0);
    feedback.now_ms += 4;
    WheelLegControlStep(&feedback, &output);
    failures += output.output_enabled != 0;

    printf("phi1=%g phi4=%g failures=%d\n", phi1, phi4, failures);
    return failures == 0 ? 0 : 1;
}
