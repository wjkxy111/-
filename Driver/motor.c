#include "motor.h"

/*
 * 新 SysConfig 引脚对应关系：
 * 左轮 PWM：PB2  -> GPIO_Motor_C0_IDX
 * 右轮 PWM：PB3  -> GPIO_Motor_C1_IDX
 *
 * 左轮方向：PA1 / PA0
 * 右轮方向：PA27 / PA26
 * STBY：PA30
 *
 * 注意：
 * 你的 SysConfig 中没有 GPIO_MOTOR_PIN_L1_PORT 这种宏，
 * 方向引脚和 STBY 统一使用 GPIO_MOTOR_PORT。
 */

void Motor_On(void)
{
    DL_GPIO_setPins(GPIO_MOTOR_PORT, GPIO_MOTOR_PIN_STBY_PIN);
}

void Motor_Off(void)
{
    DL_GPIO_clearPins(GPIO_MOTOR_PORT, GPIO_MOTOR_PIN_STBY_PIN);

    DL_GPIO_clearPins(GPIO_MOTOR_PORT, GPIO_MOTOR_PIN_L1_PIN);
    DL_GPIO_clearPins(GPIO_MOTOR_PORT, GPIO_MOTOR_PIN_L2_PIN);
    DL_GPIO_clearPins(GPIO_MOTOR_PORT, GPIO_MOTOR_PIN_R1_PIN);
    DL_GPIO_clearPins(GPIO_MOTOR_PORT, GPIO_MOTOR_PIN_R2_PIN);

    DL_TimerA_setCaptureCompareValue(Motor_INST, LEGACY_MOTOR_PWM_PERIOD, GPIO_Motor_C0_IDX);
    DL_TimerA_setCaptureCompareValue(Motor_INST, LEGACY_MOTOR_PWM_PERIOD, GPIO_Motor_C1_IDX);
}

void Set_Speed(uint8_t side, int8_t duty)
{
    uint32_t compareValue = 0;

    // 限幅，防止 duty 超过 -100 ~ 100
    if (duty > 100) duty = 100;
    if (duty < -100) duty = -100;

    if (duty == 0)
    {
        compareValue = LEGACY_MOTOR_PWM_PERIOD;

        if (side == 0)
        {
            DL_TimerA_setCaptureCompareValue(Motor_INST, compareValue, GPIO_Motor_C0_IDX);

            DL_GPIO_clearPins(GPIO_MOTOR_PORT, GPIO_MOTOR_PIN_L1_PIN);
            DL_GPIO_clearPins(GPIO_MOTOR_PORT, GPIO_MOTOR_PIN_L2_PIN);
        }
        else
        {
            DL_TimerA_setCaptureCompareValue(Motor_INST, compareValue, GPIO_Motor_C1_IDX);

            DL_GPIO_clearPins(GPIO_MOTOR_PORT, GPIO_MOTOR_PIN_R1_PIN);
            DL_GPIO_clearPins(GPIO_MOTOR_PORT, GPIO_MOTOR_PIN_R2_PIN);
        }

        return;
    }

    // 你的 PWM 是反向占空比：compare 越小，占空比越大
    if (duty > 0)
    {
        compareValue = LEGACY_MOTOR_PWM_PERIOD - (LEGACY_MOTOR_PWM_PERIOD * duty / 100);
    }
    else
    {
        compareValue = LEGACY_MOTOR_PWM_PERIOD - (LEGACY_MOTOR_PWM_PERIOD * (-duty) / 100);
    }

    if (side == 0)
    {
        // 左轮
        DL_TimerA_setCaptureCompareValue(Motor_INST, compareValue, GPIO_Motor_C0_IDX);

        if (duty > 0)
        {
            // 左轮前进
            DL_GPIO_clearPins(GPIO_MOTOR_PORT, GPIO_MOTOR_PIN_L1_PIN);
            DL_GPIO_setPins(GPIO_MOTOR_PORT, GPIO_MOTOR_PIN_L2_PIN);
        }
        else
        {
            // 左轮后退
            DL_GPIO_setPins(GPIO_MOTOR_PORT, GPIO_MOTOR_PIN_L1_PIN);
            DL_GPIO_clearPins(GPIO_MOTOR_PORT, GPIO_MOTOR_PIN_L2_PIN);
        }
    }
    else
    {
        // 右轮
        DL_TimerA_setCaptureCompareValue(Motor_INST, compareValue, GPIO_Motor_C1_IDX);

        if (duty > 0)
        {
            // 右轮前进
            DL_GPIO_setPins(GPIO_MOTOR_PORT, GPIO_MOTOR_PIN_R1_PIN);
            DL_GPIO_clearPins(GPIO_MOTOR_PORT, GPIO_MOTOR_PIN_R2_PIN);
        }
        else
        {
            // 右轮后退
            DL_GPIO_clearPins(GPIO_MOTOR_PORT, GPIO_MOTOR_PIN_R1_PIN);
            DL_GPIO_setPins(GPIO_MOTOR_PORT, GPIO_MOTOR_PIN_R2_PIN);
        }
    }
}
