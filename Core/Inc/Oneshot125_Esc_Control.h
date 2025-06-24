/*
 * Oneshot125_Esc_Control.h
 *
 *  Created on: Jan 11, 2025
 *      Author: GIGABYTE
 */

#ifndef INC_ONESHOT125_ESC_CONTROL_H_
#define INC_ONESHOT125_ESC_CONTROL_H_

#ifdef __cplusplus
 extern "C" {
#endif
#define Calibrate_with_RTOS 0
#define MINIMUM_ESC_PWM 10500
#define MAXIMUM_ESC_PWM 21000
#define MAXIMUM_SPEED 10500
#define MINIMUM_SPEED 0

#include "main.h"
#include "cmsis_os.h"

typedef enum
{
	CHANNEL1=TIM_CHANNEL_1,
	CHANNEL2=TIM_CHANNEL_2,
	CHANNEL3=TIM_CHANNEL_3,
	CHANNEL4=TIM_CHANNEL_4,
}Timer_channel_enumTypedef;

typedef struct
{
	uint32_t speed1;
	uint32_t speed2;
	uint32_t speed3;
	uint32_t speed4;
}Motor_speed_Typedef;

void Calibration(TIM_HandleTypeDef *TIM);
void escNormalOneshot125(TIM_HandleTypeDef *htim,uint32_t speed,Timer_channel_enumTypedef CHANNEL);
void Control4Motor(TIM_HandleTypeDef *htim,Motor_speed_Typedef *speed);
#ifdef __cplusplus
}
#endif
#endif /* INC_ONESHOT125_ESC_CONTROL_H_ */
