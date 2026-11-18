/*
 * 四全向轮 X 型底盘控制。
 *
 * 轮序：P74=左前 FL，P75=左后 RL，P76=右后 RR，P77=右前 FR。
 * +X 向前，+Y 向左，+ω 逆时针。
 *
 * 小陀螺模式下左摇杆平移优先：先压缩自转，保证前进/后退仍有效。
 */

#include "chassis.h"
#include "config.h"
#include <math.h>

static float spin_integral;
static float spin_integral_prev;
static signed short last_wheel[4];
static float translation[4];
static float rotation[4];
static float target[4];

static float vx;
static float vy;
static float relative;
static float body_vx;
static float body_vy;
static float omega_dps;
static float rate_error;
static float rotation_duty;
static float max_translation;
static float max_rotation;
static float scale;
static float delta;

static float wrap180(float angle)
{
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

void chassis_reset_spin(void)
{
    spin_integral = 0.0f;
}

void chassis_stop(void)
{
    spin_integral = 0.0f;
    last_wheel[0] = 0;
    last_wheel[1] = 0;
    last_wheel[2] = 0;
    last_wheel[3] = 0;
}

/*
 * 轮速混合、平移优先归一化、方向符号与斜率限制。
 * 输入 vx/vy 是车体系占空比，omega_dps 是绕 +z（逆时针为正）的角速度。
 */
static void chassis_output(float body_vx, float body_vy, float omega_dps,
                           unsigned char spin_mode, signed short wheel_out[4])
{
    unsigned char index;

    rotation_duty = clampf(ROT_GAIN_DUTY_PER_DPS * omega_dps, (float)DUTY_MAX);

    translation[0] = body_vx - body_vy;
    translation[1] = body_vx + body_vy;
    translation[2] = body_vx - body_vy;
    translation[3] = body_vx + body_vy;

    rotation[0] = -rotation_duty;
    rotation[1] = -rotation_duty;
    rotation[2] = rotation_duty;
    rotation[3] = rotation_duty;

    max_translation = fabsf(translation[0]);
    if (fabsf(translation[1]) > max_translation)
        max_translation = fabsf(translation[1]);
    if (fabsf(translation[2]) > max_translation)
        max_translation = fabsf(translation[2]);
    if (fabsf(translation[3]) > max_translation)
        max_translation = fabsf(translation[3]);

    max_rotation = fabsf(rotation[0]);
    if (fabsf(rotation[1]) > max_rotation)
        max_rotation = fabsf(rotation[1]);
    if (fabsf(rotation[2]) > max_rotation)
        max_rotation = fabsf(rotation[2]);
    if (fabsf(rotation[3]) > max_rotation)
        max_rotation = fabsf(rotation[3]);

    if (max_translation >= (float)DUTY_MAX)
    {
        scale = (float)DUTY_MAX / max_translation;
        for (index = 0; index < 4; index++)
        {
            translation[index] *= scale;
            rotation[index] = 0.0f;
        }
        if (spin_mode)
            spin_integral = spin_integral_prev;
    }
    else if (max_rotation > 0.0f && max_translation + max_rotation > (float)DUTY_MAX)
    {
        scale = ((float)DUTY_MAX - max_translation) / max_rotation;
        for (index = 0; index < 4; index++)
            rotation[index] *= scale;
        if (spin_mode)
            spin_integral = spin_integral_prev;
    }

    target[0] = (translation[0] + rotation[0]) * WHEEL_SIGN_FL;
    target[1] = (translation[1] + rotation[1]) * WHEEL_SIGN_RL;
    target[2] = (translation[2] + rotation[2]) * WHEEL_SIGN_RR;
    target[3] = (translation[3] + rotation[3]) * WHEEL_SIGN_FR;

    for (index = 0; index < 4; index++)
    {
        if ((target[index] >= 0.0f) != (last_wheel[index] >= 0))
            last_wheel[index] = 0;

        delta = target[index] - (float)last_wheel[index];
        if (delta > (float)MAX_DUTY_STEP)
            delta = (float)MAX_DUTY_STEP;
        else if (delta < -(float)MAX_DUTY_STEP)
            delta = -(float)MAX_DUTY_STEP;

        last_wheel[index] = (signed short)((float)last_wheel[index] + delta);
        wheel_out[index] = last_wheel[index];
    }
}

void chassis_update(signed short left_v, signed short left_h,
                    float chassis_yaw, float gimbal_yaw,
                    float chassis_rate, unsigned char spin_mode,
                    float dt, signed short wheel_out[4])
{
    if (left_v > -JOYSTICK_DEADBAND && left_v < JOYSTICK_DEADBAND)
        left_v = 0;
    if (left_h > -JOYSTICK_DEADBAND && left_h < JOYSTICK_DEADBAND)
        left_h = 0;

    vx = LEFT_V_SIGN * (float)left_v / JOYSTICK_MAX * MAX_TRANSLATE_DUTY;
    vy = LEFT_H_SIGN * (float)left_h / JOYSTICK_MAX * MAX_TRANSLATE_DUTY;

    relative = wrap180(chassis_yaw - gimbal_yaw) * 0.0174532925f;
    body_vx = vx * cosf(relative) + vy * sinf(relative);
    body_vy = -vx * sinf(relative) + vy * cosf(relative);

    if (spin_mode)
    {
        spin_integral_prev = spin_integral;
        rate_error = SPIN_RATE_DPS - chassis_rate;
        spin_integral += rate_error * dt;
        if (spin_integral > SPIN_INTEGRAL_LIMIT)
            spin_integral = SPIN_INTEGRAL_LIMIT;
        else if (spin_integral < -SPIN_INTEGRAL_LIMIT)
            spin_integral = -SPIN_INTEGRAL_LIMIT;

        omega_dps = SPIN_RATE_DPS + SPIN_KP * rate_error + SPIN_KI * spin_integral;
    }
    else
    {
        spin_integral = 0.0f;
        omega_dps = -CHASSIS_YAW_DAMP * chassis_rate;
    }

    chassis_output(body_vx, body_vy, omega_dps, spin_mode, wheel_out);
}

void chassis_update_velocity(float vx_mps, float vy_mps, float wz_rad_s,
                             float dt, signed short wheel_out[4])
{
    (void)dt;

    body_vx = clampf(vx_mps / V2_MAX_LINEAR_MPS, 1.0f) * MAX_TRANSLATE_DUTY;
    body_vy = clampf(vy_mps / V2_MAX_LINEAR_MPS, 1.0f) * MAX_TRANSLATE_DUTY;
    omega_dps = wz_rad_s * 57.2957795f;

    chassis_output(body_vx, body_vy, omega_dps, 0u, wheel_out);
}
