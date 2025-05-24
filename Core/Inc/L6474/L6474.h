#ifndef L6474_H
#define L6474_H

#include <main.h>
#include <Pwm.h>


class L6474 : public StepperMotor
{
public:
	L6474();
	virtual ~L6474(void) {}
	virtual int init(void *init = NULL)
	{
		return (int) L6474_Init((void *) init);
	}


	virtual int read_id(uint8_t *id = NULL)
	{
		return (int) L6474_ReadID((uint8_t *) id);
	}

	virtual unsigned int get_status(void)
	{
		return (unsigned int) L6474_CmdGetStatus();
	}

	virtual float get_parameter(unsigned int parameter)
	{
        unsigned int register_value = (unsigned int) L6474_CmdGetParam((L6474_Registers_t) parameter);
        float value;

        switch ((L6474_Registers_t) parameter) {
            case L6474_TVAL:
                value = L6474_Par_to_Tval_Current((float) register_value);
                break;
            case L6474_TON_MIN:
            case L6474_TOFF_MIN:
                value = L6474_Par_to_Tmin_Time((float) register_value);
                break;
            default:
                value = (float) register_value;
                break;
        }

        return value;
	}
	virtual signed int get_position(void)
	{
		return (signed int) L6474_GetPosition();
	}

	virtual signed int get_mark(void)
	{
		return (signed int) L6474_GetMark();
	}

	virtual unsigned int get_speed(void)
	{
		return (unsigned int) L6474_GetCurrentSpeed();
	}

	virtual unsigned int get_max_speed(void)
	{
		return (unsigned int) L6474_GetMaxSpeed();
	}

	virtual unsigned int get_min_speed(void)
	{
		return (unsigned int) L6474_GetMinSpeed();
	}

	virtual unsigned int get_acceleration(void)
	{
		return (unsigned int) L6474_GetAcceleration();
	}

	virtual unsigned int get_deceleration(void)
	{
		return (unsigned int) L6474_GetDeceleration();
	}

	virtual direction_t get_direction(void)
	{
		return (direction_t) (L6474_GetDirection() == FORWARD ? StepperMotor::FWD : StepperMotor::BWD);
	}

	virtual void set_parameter(unsigned int parameter, float value)
	{
        float register_value;

        switch ((L6474_Registers_t) parameter) {
            case L6474_TVAL:
                register_value = L6474_Tval_Current_to_Par(value);
                break;
            case L6474_TON_MIN:
            case L6474_TOFF_MIN:
                register_value = L6474_Tmin_Time_to_Par(value);
                break;
            default:
                register_value = value;
                break;
        }

        L6474_CmdSetParam((L6474_Registers_t) parameter, (unsigned int) register_value);
	}

	virtual void set_home(void)
	{
		L6474_SetHome();
	}

	virtual void set_mark(void)
	{
		L6474_SetMark();
	}

	virtual bool set_max_speed(unsigned int speed)
	{
		L6474_SetMaxSpeed((unsigned int) speed);
		return true;
	}

	virtual bool set_min_speed(unsigned int speed)
	{
		L6474_SetMinSpeed((unsigned int) speed);
		return true;
	}

	virtual bool set_acceleration(unsigned int acceleration)
	{
		L6474_SetAcceleration((unsigned int) acceleration);
		return true;
	}

	virtual bool set_deceleration(unsigned int deceleration)
	{
		L6474_SetDeceleration((unsigned int) deceleration);
		return true;
	}

	virtual void go_to(signed int position)
	{
		L6474_GoTo((signed int) position);
	}

	virtual void go_home(void)
	{
		L6474_GoHome();
	}

	virtual void go_mark(void)
	{
		L6474_GoMark();
	}
	virtual void run(direction_t direction)
	{
		L6474_Run((motorDir_t) (direction == StepperMotor::FWD ? FORWARD : BACKWARD));
	}

	virtual void move(direction_t direction, unsigned int steps)
	{
		L6474_Move((motorDir_t) (direction == StepperMotor::FWD ? FORWARD : BACKWARD), (unsigned int) steps);
	}

	virtual void soft_stop(void)
	{
		L6474_SoftStop();
	}

	virtual void hard_stop(void)
	{
		L6474_HardStop();
	}

	virtual void soft_hiz(void)
	{
        L6474_SoftStop();
        L6474_CmdDisable();
	}

	virtual void hard_hiz(void)
	{
        L6474_HardStop();
        L6474_CmdDisable();
	}

	virtual void wait_while_active(void)
	{
		L6474_WaitWhileActive();
	}

	virtual motorState_t get_device_state(void)
	{
		return (motorState_t) L6474_GetDeviceState();
	}

	virtual uint16_t read_status_register(void)
	{
		return (uint16_t) L6474_ReadStatusRegister();
	}

	virtual bool set_step_mode(step_mode_t step_mode)
	{
        if ((motorStepMode_t) step_mode > STEP_MODE_1_16) {
            return false;
        }

        soft_hiz();
        L6474_SelectStepMode((motorStepMode_t) step_mode);
        return true;
	}

	virtual void attach_error_handler(void (*fptr)(uint16_t error))
	{
		L6474_AttachErrorHandler((void (*)(uint16_t error)) fptr);
	}

	virtual void enable(void)
	{
		L6474_CmdEnable();
	}

	virtual void disable(void)
	{
		L6474_CmdDisable();
	}

	virtual uint8_t get_fw_version(void)
	{
		return (uint8_t) L6474_GetFwVersion();
	}

	void attach_flag_irq(void (*fptr)(void));
	void enable_flag_irq(void);
	void disable_flag_irq(void);

protected:
	void L6474_AttachErrorHandler(void (*callback)(uint16_t error));
	status_t L6474_Init(void *init);
	status_t L6474_ReadID(uint8_t *id);
	uint16_t L6474_GetAcceleration(void);
	uint16_t L6474_GetCurrentSpeed(void);
	uint16_t L6474_GetDeceleration(void);
	motorState_t L6474_GetDeviceState(void);
	uint8_t L6474_GetFwVersion(void);
	int32_t L6474_GetMark(void);
	uint16_t L6474_GetMaxSpeed(void);
	uint16_t L6474_GetMinSpeed(void);
	int32_t L6474_GetPosition(void);
	void L6474_GoHome(void);
	void L6474_GoMark(void);
	void L6474_GoTo(int32_t targetPosition);
	void L6474_HardStop(void);
	void L6474_Move(motorDir_t direction, uint32_t stepCount);
	void L6474_Run(motorDir_t direction);
	bool L6474_SetAcceleration(uint16_t newAcc);
	bool L6474_SetDeceleration(uint16_t newDec);
	void L6474_SetHome(void);
	void L6474_SetMark(void);
	bool L6474_SetMaxSpeed(uint16_t newMaxSpeed);
	bool L6474_SetMinSpeed(uint16_t newMinSpeed);
	bool L6474_SoftStop(void);
	void L6474_WaitWhileActive(void);
	void L6474_CmdDisable(void);
	void L6474_CmdEnable(void);
	uint32_t L6474_CmdGetParam(L6474_Registers_t parameter);
	uint16_t L6474_CmdGetStatus(void);
	void L6474_CmdNop(void);
	void L6474_CmdSetParam(L6474_Registers_t parameter, uint32_t value);
	uint16_t L6474_ReadStatusRegister(void);
	void L6474_SelectStepMode(motorStepMode_t stepMod);
	motorDir_t L6474_GetDirection(void);
	void L6474_SetDirection(motorDir_t direction);
	void L6474_ApplySpeed(uint16_t newSpeed);
	void L6474_ComputeSpeedProfile(uint32_t nbSteps);
	int32_t L6474_ConvertPosition(uint32_t abs_position_reg);
	void L6474_ErrorHandler(uint16_t error);
	void L6474_SendCommand(uint8_t param);
	void L6474_SetRegisterToPredefinedValues(void);
	void L6474_SetRegisterToInitializationValues(L6474_init_t *init);
	void L6474_WriteBytes(uint8_t *pByteToTransmit, uint8_t *pReceivedByte);
	void L6474_SetDeviceParamsToPredefinedValues(void);
	void L6474_StartMovement(void);
	void L6474_StepClockHandler(void);
	float L6474_Tval_Current_to_Par(float current_mA);
	float L6474_Par_to_Tval_Current(float Tval);
	float L6474_Tmin_Time_to_Par(float ton_min_us);
	float L6474_Par_to_Tmin_Time(float Tmin);

	//unimplemented
	status_t Read(uint8_t* pBuffer, uint16_t NumBytesToRead);
	status_t Write(uint8_t* pBuffer, uint16_t NumBytesToWrite);
	status_t ReadWrite(uint8_t* pBufferToRead, uint8_t* pBufferToWrite, uint16_t NumBytes);
	void L6474_Delay(uint32_t delay)
	{
		HAL_Delay(delay);
	}

	void L6474_EnableIrq(void)
	{
		__enable_irq();
	}

	void L6474_DisableIrq(void)
	{
		__disable_irq();
	}

	void L6474_PwmInit(void) {}
	void L6474_PwmSetFreq(uint16_t frequency);
	void L6474_PwmStop(void);
	void L6474_ReleaseReset(void)
	{
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET);
	}

	void L6474_Reset(void)
	{
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);
	}
	void L6474_SetDirectionGpio(uint8_t gpioState)
	{
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PinState(gpioState));
	}

	uint8_t L6474_SpiWriteBytes(uint8_t *pByteToTransmit, uint8_t *pReceivedByte)
	{
		return (uint8_t) (ReadWrite(pReceivedByte, pByteToTransmit, number_of_devices) == COMPONENT_OK ? 0 : 1);
	}

    //InterruptIn flag_irq;
    //DigitalOut standby_reset;
    //DigitalOut direction;
    //PwmOut pwm;
	Pwm pwm;

    //Ticker ticker;
    //DigitalOut ssel;
    //DevSPI &dev_spi;
    uint8_t who_am_i;
    void (*error_handler_callback)(uint16_t error);
    deviceParams_t device_prm;
    uint8_t device_instance;

    /* Static data. */
    static uint8_t number_of_devices;
    static uint8_t spi_tx_bursts[L6474_CMD_ARG_MAX_NB_BYTES][MAX_NUMBER_OF_DEVICES];
    static uint8_t spi_rx_bursts[L6474_CMD_ARG_MAX_NB_BYTES][MAX_NUMBER_OF_DEVICES];

public:

    /* Static data. */
    static bool spi_preemtion_by_isr;
    static bool isr_flag;

};

#endif
