#include <Common/Component.h>
#include <Actuators/StepperMotor.h>

#include <L6474_config.h>
#include <L6474_def.h>
#include <motor_def.h>
#include <L6474.h>

#include <stdlib.h>
#include <stdio.h>

#define STEPS_1 (400 * 8)   /* 1 revolution given a 400 steps motor configured at 1/8 microstep mode. */

/* Delay in milliseconds. */
#define DELAY_1 1000
#define DELAY_2 2000
#define DELAY_3 6000
#define DELAY_4 8000

/* Speed in pps (Pulses Per Second).
   In Full Step mode: 1 pps = 1 step/s).
   In 1/N Step Mode:  N pps = 1 step/s). */
#define SPEED_1 2400
#define SPEED_2 1200

L6474_init_t init = {
    160,                              /* Acceleration rate in pps^2. Range: (0..+inf). */
    160,                              /* Deceleration rate in pps^2. Range: (0..+inf). */
    1600,                             /* Maximum speed in pps. Range: (30..10000]. */
    800,                              /* Minimum speed in pps. Range: [30..10000). */
    250,                              /* Torque regulation current in mA. Range: 31.25mA to 4000mA. */
    L6474_OCD_TH_750mA,               /* Overcurrent threshold (OCD_TH register). */
    L6474_CONFIG_OC_SD_ENABLE,        /* Overcurrent shutwdown (OC_SD field of CONFIG register). */
    L6474_CONFIG_EN_TQREG_TVAL_USED,  /* Torque regulation method (EN_TQREG field of CONFIG register). */
    L6474_STEP_SEL_1_8,               /* Step selection (STEP_SEL field of STEP_MODE register). */
    L6474_SYNC_SEL_1_2,               /* Sync selection (SYNC_SEL field of STEP_MODE register). */
    L6474_FAST_STEP_12us,             /* Fall time value (T_FAST field of T_FAST register). Range: 2us to 32us. */
    L6474_TOFF_FAST_8us,              /* Maximum fast decay time (T_OFF field of T_FAST register). Range: 2us to 32us. */
    3,                                /* Minimum ON time in us (TON_MIN register). Range: 0.5us to 64us. */
    21,                               /* Minimum OFF time in us (TOFF_MIN register). Range: 0.5us to 64us. */
    L6474_CONFIG_TOFF_044us,          /* Target Swicthing Period (field TOFF of CONFIG register). */
    L6474_CONFIG_SR_320V_us,          /* Slew rate (POW_SR field of CONFIG register). */
    L6474_CONFIG_INT_16MHZ,           /* Clock setting (OSC_CLK_SEL field of CONFIG register). */
    L6474_ALARM_EN_OVERCURRENT |
    L6474_ALARM_EN_THERMAL_SHUTDOWN |
    L6474_ALARM_EN_THERMAL_WARNING |
    L6474_ALARM_EN_UNDERVOLTAGE |
    L6474_ALARM_EN_SW_TURN_ON |
    L6474_ALARM_EN_WRONG_NPERF_CMD    /* Alarm (ALARM_EN register). */
};

L6474 *motor;

void flag_irq_handler(void)
{
    /* Set ISR flag. */
    motor->isr_flag = TRUE;

    /* Get the value of the status register. */
    unsigned int status = motor->get_status();

    /* Check NOTPERF_CMD flag: if set, the command received by SPI can't be performed. */
    /* This often occures when a command is sent to the L6474 while it is not in HiZ state. */
    if ((status & L6474_STATUS_NOTPERF_CMD) == L6474_STATUS_NOTPERF_CMD) {
        printf("    WARNING: \"FLAG\" interrupt triggered. Non-performable command detected when updating L6474's registers while not in HiZ state.\r\n");
    }

    /* Reset ISR flag. */
    motor->isr_flag = FALSE;
}

extern "C" void L6474_create()
{
	motor = new L6474;
	if(motor->init(&init) != COMPONENT_OK)
	{
		printf("failed to initialize\r\n");
	}
	else
	{
		printf("ok\r\n");
	}

	motor->attach_flag_irq(&flag_irq_handler);
	motor->enable_flag_irq();
}



int position = 0;
int diff = 0;
int speed = 1000;
extern "C" void L6474_move()
{
    motor->attach_flag_irq(&flag_irq_handler);
    motor->enable_flag_irq();

     motor->set_parameter(L6474_TVAL, 500);


     position = motor->get_position();

     motor->move(StepperMotor::FWD, STEPS_1 / 8);
     motor->wait_while_active();

     diff = motor->get_position() - position;
     position = motor->get_position();

     //motor->move(StepperMotor::BWD, STEPS_1 / 8);
     //motor->wait_while_active();
     //position = motor->get_position();

     motor->set_max_speed(10000);
     motor->set_min_speed(1000);
}



extern "C" void L6474_run()
{

	motor->set_max_speed(20000);

	speed = 1000;
	float s = 1;
	const float min = 5000, max = 9000;

	while(true)
	{
		for(int i =0; i<10;++i) {
			motor->move(StepperMotor::FWD, STEPS_1 / 8);
			motor->wait_while_active();
		}

		if(speed == min)
		{
			s = 1;
		}
		else if(speed == max)
		{
			s = -1;
		}

		speed = speed + 1000 * s;
		motor->set_min_speed(speed);
		printf("speed: %i\r\n", speed);
	}

}

