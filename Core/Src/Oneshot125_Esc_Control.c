/*
 * Oneshot125_Esc_Control.c
 *
 *  Created on: Jan 11, 2025
 *      Author: GIGABYTE
 */


#include "Oneshot125_Esc_Control.h"

//this is init function, only use before turn on scheduler
void Calibration(TIM_HandleTypeDef *htim)
{

	HAL_Delay(1000);
	htim->Instance->CCR1=MAXIMUM_ESC_PWM;
	htim->Instance->CCR2=MAXIMUM_ESC_PWM;
	htim->Instance->CCR3=MAXIMUM_ESC_PWM;
	htim->Instance->CCR4=MAXIMUM_ESC_PWM;
	HAL_Delay(7000);
	htim->Instance->CCR1=MINIMUM_ESC_PWM;
	htim->Instance->CCR2=MINIMUM_ESC_PWM;
	htim->Instance->CCR3=MINIMUM_ESC_PWM;
	htim->Instance->CCR4=MINIMUM_ESC_PWM;
	HAL_Delay(8000);
	HAL_Delay(1000);
}

void escNormalOneshot125(TIM_HandleTypeDef *htim,uint32_t speed,Timer_channel_enumTypedef CHANNEL)
{
	if(speed>=MAXIMUM_SPEED)
	{
		speed=MAXIMUM_SPEED;
	}
	else if(speed<=MINIMUM_SPEED)
	{
		speed=MINIMUM_SPEED;
	}
	switch(CHANNEL)
	{
	case CHANNEL1:
		htim->Instance->CCR1=speed+MINIMUM_ESC_PWM;
		break;
	case CHANNEL2:
			htim->Instance->CCR2=speed+MINIMUM_ESC_PWM;
			break;
	case CHANNEL3:
			htim->Instance->CCR3=speed+MINIMUM_ESC_PWM;
			break;
	case CHANNEL4:
			htim->Instance->CCR4=speed+MINIMUM_ESC_PWM;
			break;
	}
}

void Control4Motor(TIM_HandleTypeDef *htim,Motor_speed_Typedef *speed)
{
	escNormalOneshot125(htim, speed->speed1, CHANNEL1);
	escNormalOneshot125(htim, speed->speed2, CHANNEL2);
	escNormalOneshot125(htim, speed->speed3, CHANNEL3);
	escNormalOneshot125(htim, speed->speed4, CHANNEL4);

}

