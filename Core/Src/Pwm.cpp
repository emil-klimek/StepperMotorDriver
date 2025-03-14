
#include <Pwm.h>
#include "main.h"

//32768000
#define CPU_FREQUENCY 429491
#define COUNTER_PERIOD 65535

extern TIM_HandleTypeDef htim3;

extern "C" void Error_Handler(void);

Pwm::Pwm()
	 :sConfigOC{0}
{
	htim3.Init.Prescaler = (CPU_FREQUENCY / (1 * (COUNTER_PERIOD+1))) - 1;
	htim3.Init.Period = COUNTER_PERIOD;
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 1.0 * COUNTER_PERIOD;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;

	 if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
	 {
	     Error_Handler();
	 }

	 HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
	 HAL_TIM_Base_Start_IT(&htim3);
}

void Pwm::write(float value)
{
		//pulse = value * counter period;
	  sConfigOC.Pulse = value * COUNTER_PERIOD;
	  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
	  {
		  Error_Handler();
	  }

}

void Pwm::period(float seconds)
{
	htim3.Init.Prescaler = (CPU_FREQUENCY / (seconds * (COUNTER_PERIOD+1))) - 1;
	if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
	{
		Error_Handler();
	}

}

