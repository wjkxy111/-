#ifndef MOTOR_H
#define MOTOR_H

#include "ti_msp_dl_config.h"

/* 旧版未参与当前构建；新代码请使用 Driver/Hardware/motor.h 的闭环接口。 */
#define LEGACY_MOTOR_PWM_PERIOD (3199U)

#define MOTOR_LEFT   (0U)
#define MOTOR_RIGHT  (1U)

void Motor_On(void);
void Motor_Off(void);
void Set_Speed(uint8_t side, int8_t duty);

#endif
