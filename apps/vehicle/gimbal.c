/*
 * 云台 yaw 目标跟踪 PID。
 *
 * 右摇杆水平积分出目标角；云台 IMU 的 yaw/角速度作为反馈。
 */

#include "gimbal.h"
#include "config.h"
#include <math.h>

static float target_yaw;
static float integral;
static float rate_filtered;
static signed short last_duty;
static float gimbal_kp = GIMBAL_KP;
static float gimbal_ki = GIMBAL_KI;
static float gimbal_kd = GIMBAL_KD;
static float gimbal_deadband = GIMBAL_DEADBAND_DEG;

static float wrap180(float angle)
{
    /* ±inf 会让 while 永不结束（防御性：yaw 链路异常时不要挂死）。 */
    if (angle > 1.0e5f || angle < -1.0e5f)
        return 0.0f;
    while (angle > 180.0f)
        angle -= 360.0f;
    while (angle < -180.0f)
        angle += 360.0f;
    return angle;
}

static float clampf(float value, float limit)
{
    if (value > limit)
        return limit;
    if (value < -limit)
        return -limit;
    return value;
}

void gimbal_reset(void)
{
    integral = 0.0f;
    rate_filtered = 0.0f;
    last_duty = 0;
}

void gimbal_set_gains(float kp, float ki, float kd)
{
    gimbal_kp = kp;
    gimbal_ki = ki;
    gimbal_kd = kd;
    integral = 0.0f;
}

void gimbal_get_gains(float *kp, float *ki, float *kd)
{
    *kp = gimbal_kp;
    *ki = gimbal_ki;
    *kd = gimbal_kd;
}

void gimbal_set_deadband(float deg)
{
    gimbal_deadband = deg;
}

float gimbal_get_deadband(void)
{
    return gimbal_deadband;
}

void gimbal_set_target(float yaw)
{
    target_yaw = wrap180(yaw);
    integral = 0.0f;
}

float gimbal_get_target(void)
{
    return target_yaw;
}

signed short gimbal_update(signed short right_h, float gimbal_yaw,
                           float gimbal_rate, float dt)
{
    float error;
    float candidate;
    float duty;
    float delta;

    if (right_h > -JOYSTICK_DEADBAND && right_h < JOYSTICK_DEADBAND)
        right_h = 0;

    target_yaw = wrap180(target_yaw +
                         RIGHT_H_SIGN * (float)right_h / JOYSTICK_MAX *
                             GIMBAL_MAX_RATE_DPS * dt);

    error = wrap180(target_yaw - gimbal_yaw);
    if (fabsf(error) < gimbal_deadband)
        error = 0.0f;

    candidate = clampf(integral + error * dt, GIMBAL_INTEGRAL_LIMIT);
    rate_filtered += GIMBAL_RATE_FILTER_ALPHA * (gimbal_rate - rate_filtered);
    duty = gimbal_kp * error + gimbal_ki * candidate -
           gimbal_kd * rate_filtered;

    if ((duty > (float)DUTY_MAX && error > 0.0f) ||
        (duty < -(float)DUTY_MAX && error < 0.0f))
    {
        /* 饱和且误差继续增大：冻结积分。 */
    }
    else
    {
        integral = candidate;
    }

    duty = clampf(duty, (float)DUTY_MAX);
    duty *= GIMBAL_DIR_SIGN;

    if ((duty >= 0.0f) != (last_duty >= 0))
        last_duty = 0;

    delta = duty - (float)last_duty;
    if (delta > (float)GIMBAL_MAX_DUTY_STEP)
        delta = (float)GIMBAL_MAX_DUTY_STEP;
    else if (delta < -(float)GIMBAL_MAX_DUTY_STEP)
        delta = -(float)GIMBAL_MAX_DUTY_STEP;

    last_duty = (signed short)((float)last_duty + delta);
    return last_duty;
}
