
#include <Pwm.h>
#include "main.h"


#define CPU_FREQUENCY 8000000
#define COUNTER_PERIOD 100

extern TIM_HandleTypeDef htim3;

extern "C" void Error_Handler(void);


int prescaler(float value)
{
	return CPU_FREQUENCY * (value) / (COUNTER_PERIOD+1) - 1;
}




Pwm::Pwm()
	 :sConfigOC{0}
{

	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
	HAL_TIM_Base_Start_IT(&htim3);


	TIM3->PSC = 0;//prescaler(1.0);
	TIM3->CCR2 =  1.0 * COUNTER_PERIOD;
	TIM3->ARR = COUNTER_PERIOD;
	//TIM3->CCR1 = 1.0 * COUNTER_PERIOD; //Pulse
	//TIM3->CCR2 = 0;

}

void Pwm::write(float value)
{
	TIM3->CCR2 = value * COUNTER_PERIOD; //Pulse
}

void Pwm::period(float seconds)
{
	int p = prescaler(seconds);
	TIM3->PSC = p; //Prescaler

}


