#ifndef PWM_H
#define PWM_H

#include "main.h"

class Pwm
{
public:
	Pwm();

	//counter period = 1000

	void write(float value);
	void period(float seconds);

private:
	TIM_OC_InitTypeDef sConfigOC;
};

#endif
