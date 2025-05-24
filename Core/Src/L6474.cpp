#include <Common/Component.h>
#include <Actuators/StepperMotor.h>

#include <L6474_config.h>
#include <L6474_def.h>
#include <motor_def.h>
#include <L6474.h>

#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include <IrqHandlers.h>

#include "main.h"

extern IrqHandlers irqHandlers;
extern SPI_HandleTypeDef hspi1;
extern DMA_HandleTypeDef hdma_spi1_rx;

/* Error while initialising the SPI. */
#define L6474_ERROR_0        (0x8000)

/* Error of bad SPI transaction. */
#define L6474_ERROR_1        (0x8001)

/* Maximum number of steps. */
#define MAX_STEPS            (0x7FFFFFFF)

/* Maximum frequency of the PWMs in Hz. */
#define L6474_MAX_PWM_FREQ   (10000)

/* Minimum frequency of the PWMs in Hz. */
#define L6474_MIN_PWM_FREQ   (2)


uint8_t L6474::number_of_devices = 0;

/* ISR flags used to restart an interrupted SPI transfer when an error is reported. */
bool L6474::spi_preemtion_by_isr = FALSE;
bool L6474::isr_flag = FALSE;

/* SPI Transmission for Daisy-Chain Configuration. */
uint8_t L6474::spi_tx_bursts[L6474_CMD_ARG_MAX_NB_BYTES][MAX_NUMBER_OF_DEVICES];
uint8_t L6474::spi_rx_bursts[L6474_CMD_ARG_MAX_NB_BYTES][MAX_NUMBER_OF_DEVICES];

L6474::L6474()
{
	 if (!(number_of_devices < MAX_NUMBER_OF_DEVICES)) {
		 printf("Instantiation of the L6474 component failed: it can be stacked up to %d times.\r\n", MAX_NUMBER_OF_DEVICES);
		 exit(EXIT_FAILURE);
	 }

     error_handler_callback = 0;
     device_instance = number_of_devices++;
     memset(spi_tx_bursts, 0, L6474_CMD_ARG_MAX_NB_BYTES * MAX_NUMBER_OF_DEVICES * sizeof(uint8_t));
     memset(spi_rx_bursts, 0, L6474_CMD_ARG_MAX_NB_BYTES * MAX_NUMBER_OF_DEVICES * sizeof(uint8_t));
}

void L6474::attach_flag_irq(void (*fptr)(void))
{
	irqHandlers.irq.func = fptr;
}

void L6474::enable_flag_irq(void)
{
	irqHandlers.irq.enabled = true;
}

void L6474::disable_flag_irq(void)
{
	irqHandlers.irq.enabled = false;
}

void L6474::L6474_AttachErrorHandler(void (*callback)(uint16_t error))
{
  error_handler_callback = (void (*)(uint16_t error)) callback;
}

status_t L6474::L6474_Init(void *init)
{
  /* Initialise the PWMs used for the Step clocks ----------------------------*/
  L6474_PwmInit();

  /* Initialise the L6474s ------------------------------------------------*/

  /* Standby-reset deactivation */
  L6474_ReleaseReset();

  /* Let a delay after reset */
  L6474_Delay(1);

  /* Set device parameters to the predefined values from "l6474_target_config.h". */
  //L6474_SetDeviceParamsToPredefinedValues();

  if (init == NULL)
    /* Set device registers to the predefined values from "l6474_target_config.h". */
    L6474_SetRegisterToPredefinedValues();
  else
    /* Set device registers to the passed initialization values. */
    L6474_SetRegisterToInitializationValues((L6474_init_t *) init);

  /* Disable L6474 powerstage */
  L6474_CmdDisable();

  /* Get Status to clear flags after start up */
  L6474_CmdGetStatus();

  return COMPONENT_OK;
}

status_t L6474::L6474_ReadID(uint8_t *id)
{
  *id = device_instance;

  return COMPONENT_OK;
}

uint16_t L6474::L6474_GetAcceleration(void)
{
  return (device_prm.acceleration);
}

uint16_t L6474::L6474_GetCurrentSpeed(void)
{
  return device_prm.speed;
}

uint16_t L6474::L6474_GetDeceleration(void)
{
  return (device_prm.deceleration);
}

motorState_t L6474::L6474_GetDeviceState(void)
{
  return device_prm.motionState;
}

uint8_t L6474::L6474_GetFwVersion(void)
{
  return (L6474_FW_VERSION);
}

int32_t L6474::L6474_GetMark(void)
{
  return L6474_ConvertPosition(L6474_CmdGetParam(L6474_MARK));
}

uint16_t L6474::L6474_GetMaxSpeed(void)
{
  return (device_prm.maxSpeed);
}

uint16_t L6474::L6474_GetMinSpeed(void)
{
  return (device_prm.minSpeed);
}

int32_t L6474::L6474_GetPosition(void)
{
  return L6474_ConvertPosition(L6474_CmdGetParam(L6474_ABS_POS));
}

void L6474::L6474_GoHome(void)
{
  L6474_GoTo(0);
}

void L6474::L6474_GoMark(void)
{
    uint32_t mark;

    mark = L6474_ConvertPosition(L6474_CmdGetParam(L6474_MARK));
    L6474_GoTo(mark);
}

void L6474::L6474_GoTo(int32_t targetPosition)
{
  motorDir_t direction;
  int32_t steps;

  /* Eventually deactivate motor */
  if (device_prm.motionState != INACTIVE)
  {
    L6474_HardStop();
  }

  /* Get current position */
  device_prm.currentPosition = L6474_ConvertPosition(L6474_CmdGetParam(L6474_ABS_POS));

  /* Compute the number of steps to perform */
  steps = targetPosition - device_prm.currentPosition;

  if (steps >= 0)
  {
    device_prm.stepsToTake = steps;
    direction = FORWARD;
  }
  else
  {
    device_prm.stepsToTake = -steps;
    direction = BACKWARD;
  }

  if (steps != 0)
  {
    device_prm.commandExecuted = MOVE_CMD;

    /* Direction setup */
    L6474_SetDirection(direction);

    L6474_ComputeSpeedProfile(device_prm.stepsToTake);

    /* Motor activation */
    L6474_StartMovement();
  }
}

void L6474::L6474_HardStop(void)
{
  /* Disable corresponding PWM */
  L6474_PwmStop();

  /* Set inactive state */
  device_prm.motionState = INACTIVE;
  device_prm.commandExecuted = NO_CMD;
  device_prm.stepsToTake = MAX_STEPS;
}

void L6474::L6474_Move(motorDir_t direction, uint32_t stepCount)
{
  /* Eventually deactivate motor */
  if (device_prm.motionState != INACTIVE)
  {
    L6474_HardStop();
  }

  if (stepCount != 0)
  {
    device_prm.stepsToTake = stepCount;

    device_prm.commandExecuted = MOVE_CMD;

    device_prm.currentPosition = L6474_ConvertPosition(L6474_CmdGetParam(L6474_ABS_POS));

    /* Direction setup */
    L6474_SetDirection(direction);

    L6474_ComputeSpeedProfile(stepCount);

    /* Motor activation */
    L6474_StartMovement();
  }
}

void L6474::L6474_Run(motorDir_t direction)
{
  /* Eventually deactivate motor */
  if (device_prm.motionState != INACTIVE)
  {
    L6474_HardStop();
  }

  /* Direction setup */
  L6474_SetDirection(direction);

  device_prm.commandExecuted = RUN_CMD;

  /* Motor activation */
  L6474_StartMovement();
}

bool L6474::L6474_SetAcceleration(uint16_t newAcc)
{
  bool cmdExecuted = FALSE;
  if ((newAcc != 0)&&
      ((device_prm.motionState == INACTIVE)||
       (device_prm.commandExecuted == RUN_CMD)))
  {
    device_prm.acceleration = newAcc;
    cmdExecuted = TRUE;
  }
  return cmdExecuted;
}

bool L6474::L6474_SetDeceleration(uint16_t newDec)
{
  bool cmdExecuted = FALSE;
  if ((newDec != 0)&&
      ((device_prm.motionState == INACTIVE)||
       (device_prm.commandExecuted == RUN_CMD)))
  {
    device_prm.deceleration = newDec;
    cmdExecuted = TRUE;
  }
  return cmdExecuted;
}

void L6474::L6474_SetHome(void)
{
  L6474_CmdSetParam(L6474_ABS_POS, 0);
}

void L6474::L6474_SetMark(void)
{
  uint32_t mark = L6474_CmdGetParam(L6474_ABS_POS);
  L6474_CmdSetParam(L6474_MARK, mark);
}

bool L6474::L6474_SetMaxSpeed(uint16_t newMaxSpeed)
{
  bool cmdExecuted = FALSE;
  if ((newMaxSpeed >= L6474_MIN_PWM_FREQ)&&
      (newMaxSpeed <= L6474_MAX_PWM_FREQ) &&
      (device_prm.minSpeed <= newMaxSpeed) &&
      ((device_prm.motionState == INACTIVE)||
       (device_prm.commandExecuted == RUN_CMD)))
  {
    device_prm.maxSpeed = newMaxSpeed;
    cmdExecuted = TRUE;
  }
  return cmdExecuted;
}

bool L6474::L6474_SetMinSpeed(uint16_t newMinSpeed)
{
  bool cmdExecuted = FALSE;
  if ((newMinSpeed >= L6474_MIN_PWM_FREQ)&&
      (newMinSpeed <= L6474_MAX_PWM_FREQ) &&
      (newMinSpeed <= device_prm.maxSpeed) &&
      ((device_prm.motionState == INACTIVE)||
       (device_prm.commandExecuted == RUN_CMD)))
  {
    device_prm.minSpeed = newMinSpeed;
    cmdExecuted = TRUE;
  }
  return cmdExecuted;
}

bool L6474::L6474_SoftStop(void)
{
  bool cmdExecuted = FALSE;
  if (device_prm.motionState != INACTIVE)
  {
    device_prm.commandExecuted = SOFT_STOP_CMD;
    cmdExecuted = TRUE;
  }
  return (cmdExecuted);
}

void L6474::L6474_WaitWhileActive(void)
{
  /* Wait while motor is running */
  while (L6474_GetDeviceState() != INACTIVE);
}

void L6474::L6474_CmdDisable(void)
{
  L6474_SendCommand(L6474_DISABLE);
}

void L6474::L6474_CmdEnable(void)
{
  L6474_SendCommand(L6474_ENABLE);
}

uint32_t L6474::L6474_CmdGetParam(L6474_Registers_t parameter)
{
  uint32_t i;
  uint32_t spiRxData;
  uint8_t maxArgumentNbBytes = 0;
  uint8_t spiIndex = number_of_devices - device_instance - 1;
  bool itDisable = FALSE;

  do
  {
    spi_preemtion_by_isr = FALSE;
    if (itDisable)
    {
      /* re-enable L6474_EnableIrq if disable in previous iteration */
      L6474_EnableIrq();
      itDisable = FALSE;
    }

    for (i = 0; i < number_of_devices; i++)
    {
      spi_tx_bursts[0][i] = L6474_NOP;
      spi_tx_bursts[1][i] = L6474_NOP;
      spi_tx_bursts[2][i] = L6474_NOP;
      spi_tx_bursts[3][i] = L6474_NOP;
      spi_rx_bursts[1][i] = 0;
      spi_rx_bursts[2][i] = 0;
      spi_rx_bursts[3][i] = 0;
    }

    switch (parameter)
    {
      case L6474_ABS_POS: ;
      case L6474_MARK:
        spi_tx_bursts[0][spiIndex] = ((uint8_t)L6474_GET_PARAM )| (parameter);
        maxArgumentNbBytes = 3;
        break;
      case L6474_EL_POS: ;
      case L6474_CONFIG: ;
      case L6474_STATUS:
        spi_tx_bursts[1][spiIndex] = ((uint8_t)L6474_GET_PARAM )| (parameter);
        maxArgumentNbBytes = 2;
        break;
      default:
        spi_tx_bursts[2][spiIndex] = ((uint8_t)L6474_GET_PARAM )| (parameter);
        maxArgumentNbBytes = 1;
    }

    /* Disable interruption before checking */
    /* pre-emption by ISR and SPI transfers*/
    L6474_DisableIrq();
    itDisable = TRUE;
  } while (spi_preemtion_by_isr); // check pre-emption by ISR

  for (i = L6474_CMD_ARG_MAX_NB_BYTES-1-maxArgumentNbBytes;
       i < L6474_CMD_ARG_MAX_NB_BYTES;
       i++)
  {
     L6474_WriteBytes(&spi_tx_bursts[i][0], &spi_rx_bursts[i][0]);
  }

  spiRxData = ((uint32_t)spi_rx_bursts[1][spiIndex] << 16) |
              (spi_rx_bursts[2][spiIndex] << 8) |
              (spi_rx_bursts[3][spiIndex]);

  /* re-enable L6474_EnableIrq after SPI transfers*/
  L6474_EnableIrq();

  return (spiRxData);
}

uint16_t L6474::L6474_CmdGetStatus(void)
{
  uint32_t i;
  uint16_t status;
  uint8_t spiIndex = number_of_devices - device_instance - 1;
  bool itDisable = FALSE;

  do
  {
    spi_preemtion_by_isr = FALSE;
    if (itDisable)
    {
      /* re-enable L6474_EnableIrq if disable in previous iteration */
      L6474_EnableIrq();
      itDisable = FALSE;
    }

    for (i = 0; i < number_of_devices; i++)
    {
       spi_tx_bursts[0][i] = L6474_NOP;
       spi_tx_bursts[1][i] = L6474_NOP;
       spi_tx_bursts[2][i] = L6474_NOP;
       spi_rx_bursts[1][i] = 0;
       spi_rx_bursts[2][i] = 0;
    }
    spi_tx_bursts[0][spiIndex] = L6474_GET_STATUS;

    /* Disable interruption before checking */
    /* pre-emption by ISR and SPI transfers*/
    L6474_DisableIrq();
    itDisable = TRUE;
  } while (spi_preemtion_by_isr); // check pre-emption by ISR

  for (i = 0; i < L6474_CMD_ARG_NB_BYTES_GET_STATUS + L6474_RSP_NB_BYTES_GET_STATUS; i++)
  {
     L6474_WriteBytes(&spi_tx_bursts[i][0], &spi_rx_bursts[i][0]);
  }
  status = (spi_rx_bursts[1][spiIndex] << 8) | (spi_rx_bursts[2][spiIndex]);

  /* re-enable L6474_EnableIrq after SPI transfers*/
  L6474_EnableIrq();

  return (status);
}

void L6474::L6474_CmdNop(void)
{
  L6474_SendCommand(L6474_NOP);
}

void L6474::L6474_CmdSetParam(L6474_Registers_t parameter, uint32_t value)
{
  uint32_t i;
  uint8_t maxArgumentNbBytes = 0;
  uint8_t spiIndex = number_of_devices - device_instance - 1;
  bool itDisable = FALSE;
  do
  {
    spi_preemtion_by_isr = FALSE;
    if (itDisable)
    {
      /* re-enable L6474_EnableIrq if disable in previous iteration */
      L6474_EnableIrq();
      itDisable = FALSE;
    }

    for (i = 0; i < number_of_devices; i++)
    {
      spi_tx_bursts[0][i] = L6474_NOP;
      spi_tx_bursts[1][i] = L6474_NOP;
      spi_tx_bursts[2][i] = L6474_NOP;
      spi_tx_bursts[3][i] = L6474_NOP;
    }

    switch (parameter)
    {
      case L6474_ABS_POS: ;
      case L6474_MARK:
          spi_tx_bursts[0][spiIndex] = parameter;
          spi_tx_bursts[1][spiIndex] = (uint8_t)(value >> 16);
          spi_tx_bursts[2][spiIndex] = (uint8_t)(value >> 8);
          maxArgumentNbBytes = 3;
          break;
      case L6474_EL_POS: ;
      case L6474_CONFIG:
          spi_tx_bursts[1][spiIndex] = parameter;
          spi_tx_bursts[2][spiIndex] = (uint8_t)(value >> 8);
          maxArgumentNbBytes = 2;
          break;
      default:
          spi_tx_bursts[2][spiIndex] = parameter;
          maxArgumentNbBytes = 1;
          break;
    }
    spi_tx_bursts[3][spiIndex] = (uint8_t)(value);

    /* Disable interruption before checking */
    /* pre-emption by ISR and SPI transfers*/
    L6474_DisableIrq();
    itDisable = TRUE;
  } while (spi_preemtion_by_isr); // check pre-emption by ISR

  /* SPI transfer */
  for (i = L6474_CMD_ARG_MAX_NB_BYTES-1-maxArgumentNbBytes;
       i < L6474_CMD_ARG_MAX_NB_BYTES;
       i++)
  {
     L6474_WriteBytes(&spi_tx_bursts[i][0],&spi_rx_bursts[i][0]);
  }
  /* re-enable L6474_EnableIrq after SPI transfers*/
  L6474_EnableIrq();
}

uint16_t L6474::L6474_ReadStatusRegister(void)
{
  return (L6474_CmdGetParam(L6474_STATUS));
}

void L6474::L6474_SelectStepMode(motorStepMode_t stepMod)
{
  uint8_t stepModeRegister;
  L6474_STEP_SEL_t l6474StepMod;

  switch (stepMod)
  {
    case STEP_MODE_FULL:
      l6474StepMod = L6474_STEP_SEL_1;
      break;
    case STEP_MODE_HALF:
      l6474StepMod = L6474_STEP_SEL_1_2;
      break;
    case STEP_MODE_1_4:
      l6474StepMod = L6474_STEP_SEL_1_4;
      break;
    case STEP_MODE_1_8:
      l6474StepMod = L6474_STEP_SEL_1_8;
      break;
    case STEP_MODE_1_16:
    default:
      l6474StepMod = L6474_STEP_SEL_1_16;
      break;
  }

  /* Eventually deactivate motor */
  if (device_prm.motionState != INACTIVE)
  {
    L6474_HardStop();
  }

  /* Read Step mode register and clear STEP_SEL field */
  stepModeRegister = (uint8_t)(0xF8 & L6474_CmdGetParam(L6474_STEP_MODE)) ;

  /* Apply new step mode */
  L6474_CmdSetParam(L6474_STEP_MODE, stepModeRegister | (uint8_t)l6474StepMod);

  /* Reset abs pos register */
  L6474_SetHome();
}

motorDir_t L6474::L6474_GetDirection(void)
{
  return device_prm.direction;
}

void L6474::L6474_SetDirection(motorDir_t direction)
{
  if (device_prm.motionState == INACTIVE)
  {
    device_prm.direction = direction;
    L6474_SetDirectionGpio(direction);
  }
}

void L6474::L6474_ApplySpeed(uint16_t newSpeed)
{
  if (newSpeed < L6474_MIN_PWM_FREQ)
  {
    newSpeed = L6474_MIN_PWM_FREQ;
  }
  if (newSpeed > L6474_MAX_PWM_FREQ)
  {
    newSpeed = L6474_MAX_PWM_FREQ;
  }

  device_prm.speed = newSpeed;

  L6474_PwmSetFreq(newSpeed);
}

void L6474::L6474_ComputeSpeedProfile(uint32_t nbSteps)
{
  uint32_t reqAccSteps;
  uint32_t reqDecSteps;

  /* compute the number of steps to get the targeted speed */
  uint16_t minSpeed = device_prm.minSpeed;
  reqAccSteps = (device_prm.maxSpeed - minSpeed);
  reqAccSteps *= (device_prm.maxSpeed + minSpeed);
  reqDecSteps = reqAccSteps;
  reqAccSteps /= (uint32_t)device_prm.acceleration;
  reqAccSteps /= 2;

  /* compute the number of steps to stop */
  reqDecSteps /= (uint32_t)device_prm.deceleration;
  reqDecSteps /= 2;

  if(( reqAccSteps + reqDecSteps ) > nbSteps)
  {
    /* Triangular move  */
    /* reqDecSteps = (Pos * Dec) /(Dec+Acc) */
    uint32_t dec = device_prm.deceleration;
    uint32_t acc = device_prm.acceleration;

    reqDecSteps =  ((uint32_t) dec * nbSteps) / (acc + dec);
    if (reqDecSteps > 1)
    {
      reqAccSteps = reqDecSteps - 1;
      if(reqAccSteps == 0)
      {
        reqAccSteps = 1;
      }
    }
    else
    {
      reqAccSteps = 0;
    }
    device_prm.endAccPos = reqAccSteps;
    device_prm.startDecPos = reqDecSteps;
  }
  else
  {
    /* Trapezoidal move */
    /* accelerating phase to endAccPos */
    /* steady phase from  endAccPos to startDecPos */
    /* decelerating from startDecPos to stepsToTake*/
    device_prm.endAccPos = reqAccSteps;
    device_prm.startDecPos = nbSteps - reqDecSteps - 1;
  }
}

int32_t L6474::L6474_ConvertPosition(uint32_t abs_position_reg)
{
  int32_t operation_result;

  if (abs_position_reg & L6474_ABS_POS_SIGN_BIT_MASK)
  {
    /* Negative register value */
    abs_position_reg = ~abs_position_reg;
    abs_position_reg += 1;

    operation_result = (int32_t) (abs_position_reg & L6474_ABS_POS_VALUE_MASK);
    operation_result = -operation_result;
  }
  else
  {
    operation_result = (int32_t) abs_position_reg;
  }

  return operation_result;
}

void L6474::L6474_ErrorHandler(uint16_t error)
{
  if (error_handler_callback != 0)
  {
    (void) error_handler_callback(error);
  }
  else
  {
    /* Aborting the program. */
    exit(EXIT_FAILURE);
  }
}

void L6474::L6474_SendCommand(uint8_t param)
{
  uint32_t i;
  bool itDisable = FALSE;
  uint8_t spiIndex = number_of_devices - device_instance - 1;

  do
  {
    spi_preemtion_by_isr = FALSE;
    if (itDisable)
    {
      /* re-enable L6474_EnableIrq if disable in previous iteration */
      L6474_EnableIrq();
      itDisable = FALSE;
    }

    for (i = 0; i < number_of_devices; i++)
    {
      spi_tx_bursts[3][i] = L6474_NOP;
    }
    spi_tx_bursts[3][spiIndex] = param;

    /* Disable interruption before checking */
    /* pre-emption by ISR and SPI transfers*/
    L6474_DisableIrq();
    itDisable = TRUE;
  } while (spi_preemtion_by_isr); // check pre-emption by ISR

  L6474_WriteBytes(&spi_tx_bursts[3][0], &spi_rx_bursts[3][0]);

  /* re-enable L6474_EnableIrq after SPI transfers*/
  L6474_EnableIrq();
}

void L6474::L6474_SetRegisterToPredefinedValues(void)
{
  L6474_CmdSetParam(
                    L6474_ABS_POS,
                    0);
  L6474_CmdSetParam(
                    L6474_EL_POS,
                    0);
  L6474_CmdSetParam(
                    L6474_MARK,
                    0);
  switch (device_instance)
  {
    case 0:
      L6474_CmdSetParam(
                        L6474_TVAL,
                        L6474_Tval_Current_to_Par(L6474_CONF_PARAM_TVAL_DEVICE_0));
      L6474_CmdSetParam(
                        L6474_T_FAST,
                        (uint8_t)L6474_CONF_PARAM_TOFF_FAST_DEVICE_0 |
                        (uint8_t)L6474_CONF_PARAM_FAST_STEP_DEVICE_0);
      L6474_CmdSetParam(
                        L6474_TON_MIN,
                        L6474_Tmin_Time_to_Par(L6474_CONF_PARAM_TON_MIN_DEVICE_0)
                        );
      L6474_CmdSetParam(
                        L6474_TOFF_MIN,
                        L6474_Tmin_Time_to_Par(L6474_CONF_PARAM_TOFF_MIN_DEVICE_0));
      L6474_CmdSetParam(
                        L6474_OCD_TH,
                        L6474_CONF_PARAM_OCD_TH_DEVICE_0);
      L6474_CmdSetParam(
                        L6474_STEP_MODE,
                        (uint8_t)L6474_CONF_PARAM_STEP_SEL_DEVICE_0 |
                        (uint8_t)L6474_CONF_PARAM_SYNC_SEL_DEVICE_0);
      L6474_CmdSetParam(
                        L6474_ALARM_EN,
                        L6474_CONF_PARAM_ALARM_EN_DEVICE_0);
      L6474_CmdSetParam(
                        L6474_CONFIG,
                        (uint16_t)L6474_CONF_PARAM_CLOCK_SETTING_DEVICE_0 |
                        (uint16_t)L6474_CONF_PARAM_TQ_REG_DEVICE_0 |
                        (uint16_t)L6474_CONF_PARAM_OC_SD_DEVICE_0 |
                        (uint16_t)L6474_CONF_PARAM_SR_DEVICE_0 |
                        (uint16_t)L6474_CONF_PARAM_TOFF_DEVICE_0);
      break;
    case 1:
      L6474_CmdSetParam(
                        L6474_TVAL,
                        L6474_Tval_Current_to_Par(L6474_CONF_PARAM_TVAL_DEVICE_1));
      L6474_CmdSetParam(
                        L6474_T_FAST,
                        (uint8_t)L6474_CONF_PARAM_TOFF_FAST_DEVICE_1 |
                        (uint8_t)L6474_CONF_PARAM_FAST_STEP_DEVICE_1);
      L6474_CmdSetParam(
                        L6474_TON_MIN,
                        L6474_Tmin_Time_to_Par(L6474_CONF_PARAM_TON_MIN_DEVICE_1));
      L6474_CmdSetParam(
                        L6474_TOFF_MIN,
                        L6474_Tmin_Time_to_Par(L6474_CONF_PARAM_TOFF_MIN_DEVICE_1));
      L6474_CmdSetParam(
                        L6474_OCD_TH,
                        L6474_CONF_PARAM_OCD_TH_DEVICE_1);
      L6474_CmdSetParam(
                        L6474_STEP_MODE,
                        (uint8_t)L6474_CONF_PARAM_STEP_SEL_DEVICE_1 |
                        (uint8_t)L6474_CONF_PARAM_SYNC_SEL_DEVICE_1);
      L6474_CmdSetParam(
                        L6474_ALARM_EN,
                        L6474_CONF_PARAM_ALARM_EN_DEVICE_1);
      L6474_CmdSetParam(
                        L6474_CONFIG,
                        (uint16_t)L6474_CONF_PARAM_CLOCK_SETTING_DEVICE_1 |
                        (uint16_t)L6474_CONF_PARAM_TQ_REG_DEVICE_1 |
                        (uint16_t)L6474_CONF_PARAM_OC_SD_DEVICE_1 |
                        (uint16_t)L6474_CONF_PARAM_SR_DEVICE_1 |
                        (uint16_t)L6474_CONF_PARAM_TOFF_DEVICE_1);
      break;
    case 2:
      L6474_CmdSetParam(
                        L6474_TVAL,
                        L6474_Tval_Current_to_Par(L6474_CONF_PARAM_TVAL_DEVICE_2));
      L6474_CmdSetParam(
                        L6474_T_FAST,
                        (uint8_t)L6474_CONF_PARAM_TOFF_FAST_DEVICE_2 |
                        (uint8_t)L6474_CONF_PARAM_FAST_STEP_DEVICE_2);
      L6474_CmdSetParam(
                        L6474_TON_MIN,
                        L6474_Tmin_Time_to_Par(L6474_CONF_PARAM_TON_MIN_DEVICE_2));
      L6474_CmdSetParam(
                        L6474_TOFF_MIN,
                        L6474_Tmin_Time_to_Par(L6474_CONF_PARAM_TOFF_MIN_DEVICE_2));
      L6474_CmdSetParam(
                        L6474_OCD_TH,
                        L6474_CONF_PARAM_OCD_TH_DEVICE_2);
      L6474_CmdSetParam(
                        L6474_STEP_MODE,
                        (uint8_t)L6474_CONF_PARAM_STEP_SEL_DEVICE_2 |
                        (uint8_t)L6474_CONF_PARAM_SYNC_SEL_DEVICE_2);
      L6474_CmdSetParam(
                        L6474_ALARM_EN,
                        L6474_CONF_PARAM_ALARM_EN_DEVICE_2);
      L6474_CmdSetParam(
                        L6474_CONFIG,
                        (uint16_t)L6474_CONF_PARAM_CLOCK_SETTING_DEVICE_2 |
                        (uint16_t)L6474_CONF_PARAM_TQ_REG_DEVICE_2 |
                        (uint16_t)L6474_CONF_PARAM_OC_SD_DEVICE_2 |
                        (uint16_t)L6474_CONF_PARAM_SR_DEVICE_2 |
                        (uint16_t)L6474_CONF_PARAM_TOFF_DEVICE_2);
      break;
    default: ;
  }
}

void L6474::L6474_SetRegisterToInitializationValues(L6474_init_t *init)
{
  L6474_CmdSetParam(
                    L6474_ABS_POS,
                    0
                    );
  L6474_CmdSetParam(
                    L6474_EL_POS,
                    0
                    );
  L6474_CmdSetParam(
                    L6474_MARK,
                    0
                    );
  L6474_CmdSetParam(
                    L6474_TVAL,
                    L6474_Tval_Current_to_Par(init->torque_regulation_current_mA)
                    );
  L6474_CmdSetParam(
                    L6474_T_FAST,
                    (uint8_t) init->maximum_fast_decay_time |
                    (uint8_t) init->fall_time
                    );
  L6474_CmdSetParam(
                    L6474_TON_MIN,
                    L6474_Tmin_Time_to_Par(init->minimum_ON_time_us)
                    );
  L6474_CmdSetParam(
                    L6474_TOFF_MIN,
                    L6474_Tmin_Time_to_Par(init->minimum_OFF_time_us)
                    );
  L6474_CmdSetParam(
                    L6474_OCD_TH,
                    init->overcurrent_threshold
                    );
  L6474_CmdSetParam(
                    L6474_STEP_MODE,
                    (uint8_t) init->step_selection |
                    (uint8_t) init->sync_selection
                    );
  L6474_CmdSetParam(
                    L6474_ALARM_EN,
                    init->alarm
                    );
  L6474_CmdSetParam(
                    L6474_CONFIG,
                    (uint16_t) init->clock |
                    (uint16_t) init->torque_regulation_method |
                    (uint16_t) init->overcurrent_shutwdown |
                    (uint16_t) init->slew_rate |
                    (uint16_t) init->target_swicthing_period
                    );
  L6474_SetAcceleration((uint16_t) init->acceleration_pps_2);
  L6474_SetDeceleration((uint16_t) init->deceleration_pps_2);
  L6474_SetMaxSpeed((uint16_t) init->maximum_speed_pps);
  L6474_SetMinSpeed((uint16_t) init->minimum_speed_pps);
}

void L6474::L6474_WriteBytes(uint8_t *pByteToTransmit, uint8_t *pReceivedByte)
{
  if (L6474_SpiWriteBytes(pByteToTransmit, pReceivedByte) != 0)
  {
    L6474_ErrorHandler(L6474_ERROR_1);
  }

  if (isr_flag)
  {
    spi_preemtion_by_isr = TRUE;
  }
}

void L6474::L6474_SetDeviceParamsToPredefinedValues(void)
{
  switch (device_instance)
  {
    case 0:
      device_prm.acceleration = L6474_CONF_PARAM_ACC_DEVICE_0;
      device_prm.deceleration = L6474_CONF_PARAM_DEC_DEVICE_0;
      device_prm.maxSpeed = L6474_CONF_PARAM_MAX_SPEED_DEVICE_0;
      device_prm.minSpeed = L6474_CONF_PARAM_MIN_SPEED_DEVICE_0;
      break;

    case 1:
      device_prm.acceleration = L6474_CONF_PARAM_ACC_DEVICE_1;
      device_prm.deceleration = L6474_CONF_PARAM_DEC_DEVICE_1;
      device_prm.maxSpeed = L6474_CONF_PARAM_MAX_SPEED_DEVICE_1;
      device_prm.minSpeed = L6474_CONF_PARAM_MIN_SPEED_DEVICE_1;
      break;

    case 2:
      device_prm.acceleration = L6474_CONF_PARAM_ACC_DEVICE_2;
      device_prm.deceleration = L6474_CONF_PARAM_DEC_DEVICE_2;
      device_prm.maxSpeed = L6474_CONF_PARAM_MAX_SPEED_DEVICE_2;
      device_prm.minSpeed = L6474_CONF_PARAM_MIN_SPEED_DEVICE_2;
      break;
  }

  device_prm.accu = 0;
  device_prm.currentPosition = 0;
  device_prm.endAccPos = 0;
  device_prm.relativePos = 0;
  device_prm.startDecPos = 0;
  device_prm.stepsToTake = 0;
  device_prm.speed = 0;
  device_prm.commandExecuted = NO_CMD;
  device_prm.direction = FORWARD;
  device_prm.motionState = INACTIVE;
}

void L6474::L6474_StartMovement(void)
{
  /* Enable L6474 powerstage */
  L6474_CmdEnable();
  if (device_prm.endAccPos != 0)
  {
    device_prm.motionState = ACCELERATING;
  }
  else
  {
    device_prm.motionState = DECELERATING;
  }
  device_prm.accu = 0;
  device_prm.relativePos = 0;
  L6474_ApplySpeed(device_prm.minSpeed);
}

void L6474::L6474_StepClockHandler(void)
{
  /* Set isr flag */
  isr_flag = TRUE;

  /* Incrementation of the relative position */
  device_prm.relativePos++;

  switch (device_prm.motionState)
  {
    case ACCELERATING:
    {
        uint32_t relPos = device_prm.relativePos;
        uint32_t endAccPos = device_prm.endAccPos;
        uint16_t speed = device_prm.speed;
        uint32_t acc = ((uint32_t)device_prm.acceleration << 16);

        if ((device_prm.commandExecuted == SOFT_STOP_CMD)||
            ((device_prm.commandExecuted != RUN_CMD)&&
             (relPos == device_prm.startDecPos)))
        {
          device_prm.motionState = DECELERATING;
          device_prm.accu = 0;
        }
        else if ((speed >= device_prm.maxSpeed)||
                 ((device_prm.commandExecuted != RUN_CMD)&&
                  (relPos == endAccPos)))
        {
          device_prm.motionState = STEADY;
        }
        else
        {
          bool speedUpdated = FALSE;
          /* Go on accelerating */
          if (speed == 0) speed =1;
          device_prm.accu += acc / speed;
          while (device_prm.accu >= (0X10000L))
          {
            device_prm.accu -= (0X10000L);
            speed +=1;
            speedUpdated = TRUE;
          }

          if (speedUpdated)
          {
            if (speed > device_prm.maxSpeed)
            {
              speed = device_prm.maxSpeed;
            }
            device_prm.speed = speed;
            L6474_ApplySpeed(device_prm.speed);
          }
        }
        break;
    }
    case STEADY:
    {
      uint16_t maxSpeed = device_prm.maxSpeed;
      uint32_t relativePos = device_prm.relativePos;
      if  ((device_prm.commandExecuted == SOFT_STOP_CMD)||
           ((device_prm.commandExecuted != RUN_CMD)&&
            (relativePos >= (device_prm.startDecPos))) ||
           ((device_prm.commandExecuted == RUN_CMD)&&
            (device_prm.speed > maxSpeed)))
      {
        device_prm.motionState = DECELERATING;
        device_prm.accu = 0;
      }
      else if ((device_prm.commandExecuted == RUN_CMD)&&
               (device_prm.speed < maxSpeed))
      {
        device_prm.motionState = ACCELERATING;
        device_prm.accu = 0;
      }
      break;
    }
    case DECELERATING:
    {
      uint32_t relativePos = device_prm.relativePos;
      uint16_t speed = device_prm.speed;
      uint32_t deceleration = ((uint32_t)device_prm.deceleration << 16);
      if (((device_prm.commandExecuted == SOFT_STOP_CMD)&&(speed <=  device_prm.minSpeed))||
          ((device_prm.commandExecuted != RUN_CMD)&&
           (relativePos >= device_prm.stepsToTake)))
      {
        /* Motion process complete */
        L6474_HardStop();
      }
      else if ((device_prm.commandExecuted == RUN_CMD)&&
               (speed <= device_prm.maxSpeed))
      {
        device_prm.motionState = STEADY;
      }
      else
      {
        /* Go on decelerating */
        if (speed > device_prm.minSpeed)
        {
          bool speedUpdated = FALSE;
          if (speed == 0) speed =1;
          device_prm.accu += deceleration / speed;
          while (device_prm.accu >= (0X10000L))
          {
            device_prm.accu -= (0X10000L);
            if (speed > 1)
            {
              speed -=1;
            }
            speedUpdated = TRUE;
          }

          if (speedUpdated)
          {
            if (speed < device_prm.minSpeed)
            {
              speed = device_prm.minSpeed;
            }
            device_prm.speed = speed;
            L6474_ApplySpeed(device_prm.speed);
          }
        }
      }
      break;
    }
    default:
    {
      break;
    }
  }
  /* Set isr flag */
  isr_flag = FALSE;
}

float L6474::L6474_Tval_Current_to_Par(float current_mA)
{
  return ((float)(((current_mA - 31.25f) / 31.25f) + 0.5f));
}

float L6474::L6474_Par_to_Tval_Current(float Tval)
{
  return ((float)((Tval - 0.5f) * 31.25f + 31.25f));
}

float L6474::L6474_Tmin_Time_to_Par(float ton_min_us)
{
  return ((float)(((ton_min_us - 0.5f) * 2.0f) + 0.5f));
}

float L6474::L6474_Par_to_Tmin_Time(float Tmin)
{
  return ((float)(((Tmin - 0.5f) / 2.0f) + 0.5f));
}


status_t L6474::Read(uint8_t* pBuffer, uint16_t NumBytesToRead)
{
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
	HAL_StatusTypeDef result = HAL_SPI_Receive(&hspi1, pBuffer, NumBytesToRead, 100);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
	if(result != HAL_OK)
	{
		return COMPONENT_ERROR;
	}
	else
	{
		return COMPONENT_OK;
	}
}

status_t L6474::Write(uint8_t* pBuffer, uint16_t NumBytesToWrite)
{
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
	HAL_StatusTypeDef result = HAL_SPI_Transmit(&hspi1, pBuffer, NumBytesToWrite,100);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
	if(result != HAL_OK)
	{
		return COMPONENT_ERROR;
	}
	else
	{
		return COMPONENT_OK;
	}
}


status_t L6474::ReadWrite(uint8_t* pBufferToRead, uint8_t* pBufferToWrite, uint16_t NumBytes)
{

	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
	HAL_StatusTypeDef result = HAL_OK;
	//result = HAL_SPI_Transmit_DMA(&hspi1, pBufferToRead, NumBytes);

	//if(result == HAL_OK)
	//{
	//	result = HAL_SPI_Receive_DMA(&hspi1, pBufferToWrite, NumBytes);
	//}
	result = HAL_SPI_TransmitReceive(&hspi1, pBufferToWrite, pBufferToRead, NumBytes,100);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
	//HAL_Delay(100);

	//while(wTransferState == TRANSFER_WAIT) {}



	if(result != HAL_OK)
	{
		return COMPONENT_ERROR;
	}
	else
	{
		return COMPONENT_OK;
	}

}




void L6474::L6474_PwmSetFreq(uint16_t frequency)
{
    /* Computing the period of PWM. */
    double period = 1.0f / frequency;
    //printf("frequency: %i, period: %f\n", frequency, period);

    /* Setting the period and the duty-cycle of PWM. */
    pwm.period(period);
    pwm.write(0.5f);

    /* Setting a callback with the same period of PWM's, to update the state machine. */
    //ticker.attach(Callback<void()>(this, &L6474::L6474_StepClockHandler), period);



    irqHandlers.tim3.userptr=this;
	irqHandlers.tim3.func = [](void *userData)
	{
		((L6474*)userData)->L6474_StepClockHandler();
	};


}
void L6474::L6474_PwmStop(void)
{
	pwm.write(0.0f);
	irqHandlers.tim3.func = NULL;
	irqHandlers.tim3.userptr = NULL;
}


