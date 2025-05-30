#ifndef L6474_DEF_H
#define L6474_DEF_H

#include "L6474_config.h"
#include "../Common/motor_def.h"

#define L6474_FW_VERSION                        (5)

/// L6474 max number of bytes of command & arguments to set a parameter
#define L6474_CMD_ARG_MAX_NB_BYTES              (4)

/// L6474 command + argument bytes number for GET_STATUS command
#define L6474_CMD_ARG_NB_BYTES_GET_STATUS       (1)

/// L6474 response bytes number
#define L6474_RSP_NB_BYTES_GET_STATUS           (2)

/// L6474 value mask for ABS_POS register
#define L6474_ABS_POS_VALUE_MASK    ((uint32_t) 0x003FFFFF)

/// L6474 sign bit mask for ABS_POS register
#define L6474_ABS_POS_SIGN_BIT_MASK ((uint32_t) 0x00200000)

#ifdef __cplusplus
 extern "C" {
#endif

 /* Types ---------------------------------------------------------------------*/

 /** @defgroup L6474_Exported_Types
   * @{
   */

 /** @defgroup L6474_Fast_Decay_Time_Options
   * @{
   */
 ///TOFF_FAST values for T_FAST register
 typedef enum {
   L6474_TOFF_FAST_2us = ((uint8_t) 0x00 << 4),
   L6474_TOFF_FAST_4us = ((uint8_t) 0x01 << 4),
   L6474_TOFF_FAST_6us = ((uint8_t) 0x02 << 4),
   L6474_TOFF_FAST_8us = ((uint8_t) 0x03 << 4),
   L6474_TOFF_FAST_10us = ((uint8_t) 0x04 << 4),
   L6474_TOFF_FAST_12us = ((uint8_t) 0x05 << 4),
   L6474_TOFF_FAST_14us = ((uint8_t) 0x06 << 4),
   L6474_TOFF_FAST_16us = ((uint8_t) 0x07 << 4),
   L6474_TOFF_FAST_18us = ((uint8_t) 0x08 << 4),
   L6474_TOFF_FAST_20us = ((uint8_t) 0x09 << 4),
   L6474_TOFF_FAST_22us = ((uint8_t) 0x0A << 4),
   L6474_TOFF_FAST_24us = ((uint8_t) 0x0B << 4),
   L6474_TOFF_FAST_26us = ((uint8_t) 0x0C << 4),
   L6474_TOFF_FAST_28us = ((uint8_t) 0x0D << 4),
   L6474_TOFF_FAST_30us = ((uint8_t) 0x0E << 4),
   L6474_TOFF_FAST_32us = ((uint8_t) 0x0F << 4)
 } L6474_TOFF_FAST_t;
 /**
   * @}
   */

 /** @defgroup L6474_Fall_Step_Time_Options
   * @{
   */
 ///FAST_STEP values for T_FAST register
 typedef enum {
   L6474_FAST_STEP_2us = ((uint8_t) 0x00),
   L6474_FAST_STEP_4us = ((uint8_t) 0x01),
   L6474_FAST_STEP_6us = ((uint8_t) 0x02),
   L6474_FAST_STEP_8us = ((uint8_t) 0x03),
   L6474_FAST_STEP_10us = ((uint8_t) 0x04),
   L6474_FAST_STEP_12us = ((uint8_t) 0x05),
   L6474_FAST_STEP_14us = ((uint8_t) 0x06),
   L6474_FAST_STEP_16us = ((uint8_t) 0x07),
   L6474_FAST_STEP_18us = ((uint8_t) 0x08),
   L6474_FAST_STEP_20us = ((uint8_t) 0x09),
   L6474_FAST_STEP_22us = ((uint8_t) 0x0A),
   L6474_FAST_STEP_24us = ((uint8_t) 0x0B),
   L6474_FAST_STEP_26us = ((uint8_t) 0x0C),
   L6474_FAST_STEP_28us = ((uint8_t) 0x0D),
   L6474_FAST_STEP_30us = ((uint8_t) 0x0E),
   L6474_FAST_STEP_32us = ((uint8_t) 0x0F)
 } L6474_FAST_STEP_t;
 /**
   * @}
   */

 /** @defgroup L6474_Overcurrent_Threshold_options
   * @{
   */
 ///OCD_TH register
 typedef enum {
   L6474_OCD_TH_375mA  = ((uint8_t) 0x00),
   L6474_OCD_TH_750mA  = ((uint8_t) 0x01),
   L6474_OCD_TH_1125mA = ((uint8_t) 0x02),
   L6474_OCD_TH_1500mA = ((uint8_t) 0x03),
   L6474_OCD_TH_1875mA = ((uint8_t) 0x04),
   L6474_OCD_TH_2250mA = ((uint8_t) 0x05),
   L6474_OCD_TH_2625mA = ((uint8_t) 0x06),
   L6474_OCD_TH_3000mA = ((uint8_t) 0x07),
   L6474_OCD_TH_3375mA = ((uint8_t) 0x08),
   L6474_OCD_TH_3750mA = ((uint8_t) 0x09),
   L6474_OCD_TH_4125mA = ((uint8_t) 0x0A),
   L6474_OCD_TH_4500mA = ((uint8_t) 0x0B),
   L6474_OCD_TH_4875mA = ((uint8_t) 0x0C),
   L6474_OCD_TH_5250mA = ((uint8_t) 0x0D),
   L6474_OCD_TH_5625mA = ((uint8_t) 0x0E),
   L6474_OCD_TH_6000mA = ((uint8_t) 0x0F)
 } L6474_OCD_TH_t;
 /**
   * @}
   */

 /** @defgroup L6474_STEP_MODE_Register_Masks
   * @{
   */
 ///STEP_MODE register
 typedef enum {
   L6474_STEP_MODE_STEP_SEL = ((uint8_t) 0x07),
   L6474_STEP_MODE_SYNC_SEL = ((uint8_t) 0x70)
 } L6474_STEP_MODE_Masks_t;
 /**
   * @}
   */

 /** @defgroup L6474_STEP_SEL_Options_For_STEP_MODE_Register
   * @{
   */
 ///STEP_SEL field of STEP_MODE register
 typedef enum {
   L6474_STEP_SEL_1    = ((uint8_t) 0x08),  //full step
   L6474_STEP_SEL_1_2  = ((uint8_t) 0x09),  //half step
   L6474_STEP_SEL_1_4  = ((uint8_t) 0x0A),  //1/4 microstep
   L6474_STEP_SEL_1_8  = ((uint8_t) 0x0B),  //1/8 microstep
   L6474_STEP_SEL_1_16 = ((uint8_t) 0x0C)   //1/16 microstep
 } L6474_STEP_SEL_t;
 /**
   * @}
   */

 /** @defgroup L6474_SYNC_SEL_Options_For_STEP_MODE_Register
   * @{
   */
 ///SYNC_SEL field of STEP_MODE register
 typedef enum {
   L6474_SYNC_SEL_1_2    = ((uint8_t) 0x80),
   L6474_SYNC_SEL_1      = ((uint8_t) 0x90),
   L6474_SYNC_SEL_2      = ((uint8_t) 0xA0),
   L6474_SYNC_SEL_4      = ((uint8_t) 0xB0),
   L6474_SYNC_SEL_8      = ((uint8_t) 0xC0),
   L6474_SYNC_SEL_UNUSED = ((uint8_t) 0xD0)
 } L6474_SYNC_SEL_t;
 /**
   * @}
   */

 /** @defgroup L6474_ALARM_EN_Register_Options
   * @{
   */
 ///ALARM_EN register
 typedef enum {
   L6474_ALARM_EN_OVERCURRENT      = ((uint8_t) 0x01),
   L6474_ALARM_EN_THERMAL_SHUTDOWN = ((uint8_t) 0x02),
   L6474_ALARM_EN_THERMAL_WARNING  = ((uint8_t) 0x04),
   L6474_ALARM_EN_UNDERVOLTAGE     = ((uint8_t) 0x08),
   L6474_ALARM_EN_SW_TURN_ON       = ((uint8_t) 0x40),
   L6474_ALARM_EN_WRONG_NPERF_CMD  = ((uint8_t) 0x80)
 } L6474_ALARM_EN_t;
 /**
   * @}
   */

 /** @defgroup L6474_CONFIG_Register_Masks
   * @{
   */
 ///CONFIG register
 typedef enum {
   L6474_CONFIG_OSC_SEL  = ((uint16_t) 0x0007),
   L6474_CONFIG_EXT_CLK  = ((uint16_t) 0x0008),
   L6474_CONFIG_EN_TQREG = ((uint16_t) 0x0020),
   L6474_CONFIG_OC_SD    = ((uint16_t) 0x0080),
   L6474_CONFIG_POW_SR   = ((uint16_t) 0x0300),
   L6474_CONFIG_TOFF      = ((uint16_t) 0x7C00)
 } L6474_CONFIG_Masks_t;
 /**
   * @}
   */

 /** @defgroup L6474_Clock_Source_Options_For_CONFIG_Register
   * @{
   */
 ///Clock source option for CONFIG register
 typedef enum {
   L6474_CONFIG_INT_16MHZ = ((uint16_t) 0x0000),
   L6474_CONFIG_INT_16MHZ_OSCOUT_2MHZ   = ((uint16_t) 0x0008),
   L6474_CONFIG_INT_16MHZ_OSCOUT_4MHZ   = ((uint16_t) 0x0009),
   L6474_CONFIG_INT_16MHZ_OSCOUT_8MHZ   = ((uint16_t) 0x000A),
   L6474_CONFIG_INT_16MHZ_OSCOUT_16MHZ  = ((uint16_t) 0x000B),
   L6474_CONFIG_EXT_8MHZ_XTAL_DRIVE     = ((uint16_t) 0x0004),
   L6474_CONFIG_EXT_16MHZ_XTAL_DRIVE    = ((uint16_t) 0x0005),
   L6474_CONFIG_EXT_24MHZ_XTAL_DRIVE    = ((uint16_t) 0x0006),
   L6474_CONFIG_EXT_32MHZ_XTAL_DRIVE    = ((uint16_t) 0x0007),
   L6474_CONFIG_EXT_8MHZ_OSCOUT_INVERT  = ((uint16_t) 0x000C),
   L6474_CONFIG_EXT_16MHZ_OSCOUT_INVERT = ((uint16_t) 0x000D),
   L6474_CONFIG_EXT_24MHZ_OSCOUT_INVERT = ((uint16_t) 0x000E),
   L6474_CONFIG_EXT_32MHZ_OSCOUT_INVERT = ((uint16_t) 0x000F)
 } L6474_CONFIG_OSC_MGMT_t;
 /**
   * @}
   */

 /** @defgroup L6474_External_Torque_Regulation_Options_For_CONFIG_Register
   * @{
   */
 ///External Torque regulation options for CONFIG register
 typedef enum {
   L6474_CONFIG_EN_TQREG_TVAL_USED = ((uint16_t) 0x0000),
   L6474_CONFIG_EN_TQREG_ADC_OUT = ((uint16_t) 0x0020)
 } L6474_CONFIG_EN_TQREG_t;
 /**
   * @}
   */

 /** @defgroup L6474_Over_Current_Shutdown_Options_For_CONFIG_Register
   * @{
   */
 ///Over Current Shutdown options for CONFIG register
 typedef enum {
   L6474_CONFIG_OC_SD_DISABLE = ((uint16_t) 0x0000),
   L6474_CONFIG_OC_SD_ENABLE  = ((uint16_t) 0x0080)
 } L6474_CONFIG_OC_SD_t;
 /**
   * @}
   */

 /** @defgroup L6474_Power_Bridge_Output_Slew_Rate_Options
   * @{
   */
 /// POW_SR values for CONFIG register
 typedef enum {
   L6474_CONFIG_SR_320V_us    =((uint16_t)0x0000),
   L6474_CONFIG_SR_075V_us    =((uint16_t)0x0100),
   L6474_CONFIG_SR_110V_us    =((uint16_t)0x0200),
   L6474_CONFIG_SR_260V_us    =((uint16_t)0x0300)
 } L6474_CONFIG_POW_SR_t;
 /**
   * @}
   */

 /** @defgroup L6474_Off_Time_Options
   * @{
   */
 /// TOFF values for CONFIG register
 typedef enum {
   L6474_CONFIG_TOFF_004us   = (((uint16_t) 0x01) << 10),
   L6474_CONFIG_TOFF_008us   = (((uint16_t) 0x02) << 10),
   L6474_CONFIG_TOFF_012us  = (((uint16_t) 0x03) << 10),
   L6474_CONFIG_TOFF_016us  = (((uint16_t) 0x04) << 10),
   L6474_CONFIG_TOFF_020us  = (((uint16_t) 0x05) << 10),
   L6474_CONFIG_TOFF_024us  = (((uint16_t) 0x06) << 10),
   L6474_CONFIG_TOFF_028us  = (((uint16_t) 0x07) << 10),
   L6474_CONFIG_TOFF_032us  = (((uint16_t) 0x08) << 10),
   L6474_CONFIG_TOFF_036us  = (((uint16_t) 0x09) << 10),
   L6474_CONFIG_TOFF_040us  = (((uint16_t) 0x0A) << 10),
   L6474_CONFIG_TOFF_044us  = (((uint16_t) 0x0B) << 10),
   L6474_CONFIG_TOFF_048us  = (((uint16_t) 0x0C) << 10),
   L6474_CONFIG_TOFF_052us  = (((uint16_t) 0x0D) << 10),
   L6474_CONFIG_TOFF_056us  = (((uint16_t) 0x0E) << 10),
   L6474_CONFIG_TOFF_060us  = (((uint16_t) 0x0F) << 10),
   L6474_CONFIG_TOFF_064us  = (((uint16_t) 0x10) << 10),
   L6474_CONFIG_TOFF_068us  = (((uint16_t) 0x11) << 10),
   L6474_CONFIG_TOFF_072us  = (((uint16_t) 0x12) << 10),
   L6474_CONFIG_TOFF_076us  = (((uint16_t) 0x13) << 10),
   L6474_CONFIG_TOFF_080us  = (((uint16_t) 0x14) << 10),
   L6474_CONFIG_TOFF_084us  = (((uint16_t) 0x15) << 10),
   L6474_CONFIG_TOFF_088us  = (((uint16_t) 0x16) << 10),
   L6474_CONFIG_TOFF_092us  = (((uint16_t) 0x17) << 10),
   L6474_CONFIG_TOFF_096us  = (((uint16_t) 0x18) << 10),
   L6474_CONFIG_TOFF_100us = (((uint16_t) 0x19) << 10),
   L6474_CONFIG_TOFF_104us = (((uint16_t) 0x1A) << 10),
   L6474_CONFIG_TOFF_108us = (((uint16_t) 0x1B) << 10),
   L6474_CONFIG_TOFF_112us = (((uint16_t) 0x1C) << 10),
   L6474_CONFIG_TOFF_116us = (((uint16_t) 0x1D) << 10),
   L6474_CONFIG_TOFF_120us = (((uint16_t) 0x1E) << 10),
   L6474_CONFIG_TOFF_124us = (((uint16_t) 0x1F) << 10)
 } L6474_CONFIG_TOFF_t;
 /**
   * @}
   */

 /** @defgroup L6474_STATUS_Register_Bit_Masks
   * @{
   */
 ///STATUS Register Bit Masks
 typedef enum {
   L6474_STATUS_HIZ         = (((uint16_t) 0x0001)),
   L6474_STATUS_DIR         = (((uint16_t) 0x0010)),
   L6474_STATUS_NOTPERF_CMD = (((uint16_t) 0x0080)),
   L6474_STATUS_WRONG_CMD   = (((uint16_t) 0x0100)),
   L6474_STATUS_UVLO        = (((uint16_t) 0x0200)),
   L6474_STATUS_TH_WRN      = (((uint16_t) 0x0400)),
   L6474_STATUS_TH_SD       = (((uint16_t) 0x0800)),
   L6474_STATUS_OCD         = (((uint16_t) 0x1000))
 } L6474_STATUS_Masks_t;
 /**
   * @}
   */

 /** @defgroup L6474_Direction_Field_Of_STATUS_Register
   * @{
   */
 ///Diretion field of STATUS register
 typedef enum {
   L6474_STATUS_DIR_FORWARD = (((uint16_t) 0x0001) << 4),
   L6474_STATUS_DIR_REVERSE = (((uint16_t) 0x0000) << 4)
 } L6474_STATUS_DIR_t;
 /**
   * @}
   */

 /** @defgroup L6474_Internal_Register_Addresses
   * @{
   */
 /// Internal L6474 register addresses
 typedef enum {
   L6474_ABS_POS        = ((uint8_t) 0x01),
   L6474_EL_POS         = ((uint8_t) 0x02),
   L6474_MARK           = ((uint8_t) 0x03),
   L6474_RESERVED_REG01 = ((uint8_t) 0x04),
   L6474_RESERVED_REG02 = ((uint8_t) 0x05),
   L6474_RESERVED_REG03 = ((uint8_t) 0x06),
   L6474_RESERVED_REG04 = ((uint8_t) 0x07),
   L6474_RESERVED_REG05 = ((uint8_t) 0x08),
   L6474_RESERVED_REG06 = ((uint8_t) 0x15),
   L6474_TVAL           = ((uint8_t) 0x09),
   L6474_RESERVED_REG07 = ((uint8_t) 0x0A),
   L6474_RESERVED_REG08 = ((uint8_t) 0x0B),
   L6474_RESERVED_REG09 = ((uint8_t) 0x0C),
   L6474_RESERVED_REG10 = ((uint8_t) 0x0D),
   L6474_T_FAST         = ((uint8_t) 0x0E),
   L6474_TON_MIN        = ((uint8_t) 0x0F),
   L6474_TOFF_MIN       = ((uint8_t) 0x10),
   L6474_RESERVED_REG11 = ((uint8_t) 0x11),
   L6474_ADC_OUT        = ((uint8_t) 0x12),
   L6474_OCD_TH         = ((uint8_t) 0x13),
   L6474_RESERVED_REG12 = ((uint8_t) 0x14),
   L6474_STEP_MODE      = ((uint8_t) 0x16),
   L6474_ALARM_EN       = ((uint8_t) 0x17),
   L6474_CONFIG         = ((uint8_t) 0x18),
   L6474_STATUS         = ((uint8_t) 0x19),
   L6474_RESERVED_REG13 = ((uint8_t) 0x1A),
   L6474_RESERVED_REG14 = ((uint8_t) 0x1B),
   L6474_INEXISTENT_REG = ((uint8_t) 0x1F)
 } L6474_Registers_t;
 /**
   * @}
   */

 /** @defgroup L6474_Command_Set
   * @{
   */
 /// L6474 command set
 typedef enum {
   L6474_NOP           = ((uint8_t) 0x00),
   L6474_SET_PARAM     = ((uint8_t) 0x00),
   L6474_GET_PARAM     = ((uint8_t) 0x20),
   L6474_ENABLE        = ((uint8_t) 0xB8),
   L6474_DISABLE       = ((uint8_t) 0xA8),
   L6474_GET_STATUS    = ((uint8_t) 0xD0),
   L6474_RESERVED_CMD1 = ((uint8_t) 0xEB),
   L6474_RESERVED_CMD2 = ((uint8_t) 0xF8)
 } L6474_Commands_t;

 /**
  * @brief  L6474 driver initialization structure definition.
  */
 /* ACTION --------------------------------------------------------------------*
  * Declare here the component's initialization structure, if any, one         *
  * variable per line without initialization.                                  *
  *                                                                            *
  * Example:                                                                   *
  *   typedef struct                                                           *
  *   {                                                                        *
  *     int frequency;                                                         *
  *     int update_mode;                                                       *
  *   } COMPONENT_Init_t;                                                      *
  *----------------------------------------------------------------------------*/
 typedef struct
 {
   /* Acceleration rate in pps^2. Range: (0..+inf). */
   int acceleration_pps_2;

   /* Deceleration rate in pps^2. Range: (0..+inf). */
   int deceleration_pps_2;

   /* Maximum speed in pps. Range: (30..10000]. */
   int maximum_speed_pps;

   /* Minimum speed in pps. Range: [30..10000). */
   int minimum_speed_pps;

   /* Torque regulation current in mA. Range: 31.25mA to 4000mA. */
   float torque_regulation_current_mA;

   /* Overcurrent threshold (OCD_TH register). */
   L6474_OCD_TH_t overcurrent_threshold;

   /* Overcurrent shutwdown (OC_SD field of CONFIG register). */
   L6474_CONFIG_OC_SD_t overcurrent_shutwdown;

   /* Torque regulation method (EN_TQREG field of CONFIG register). */
   L6474_CONFIG_EN_TQREG_t torque_regulation_method;

   /* Step selection (STEP_SEL field of STEP_MODE register). */
   L6474_STEP_SEL_t step_selection;

   /* Sync selection (SYNC_SEL field of STEP_MODE register). */
   L6474_SYNC_SEL_t sync_selection;

   /* Fall time value (T_FAST field of T_FAST register). Range: 2us to 32us. */
   L6474_FAST_STEP_t fall_time;

   /* Maximum fast decay time (T_OFF field of T_FAST register). Range: 2us to 32us. */
   L6474_TOFF_FAST_t maximum_fast_decay_time;

   /* Minimum ON time in us (TON_MIN register). Range: 0.5us to 64us. */
   float minimum_ON_time_us;

   /* Minimum OFF time in us (TOFF_MIN register). Range: 0.5us to 64us. */
   float minimum_OFF_time_us;

   /* Target Swicthing Period (field TOFF of CONFIG register). */
   L6474_CONFIG_TOFF_t target_swicthing_period;

   /* Slew rate (POW_SR field of CONFIG register). */
   L6474_CONFIG_POW_SR_t slew_rate;

   /* Clock setting (OSC_CLK_SEL field of CONFIG register). */
   L6474_CONFIG_OSC_MGMT_t clock;

   /* Alarm (ALARM_EN register). */
   int alarm;
 } L6474_init_t;

 /**
  * @brief  L6474 driver data structure definition.
  */
 /* ACTION --------------------------------------------------------------------*
  * Declare here the structure of component's data, if any, one variable per   *
  * line without initialization.                                               *
  *                                                                            *
  * Example:                                                                   *
  *   typedef struct                                                           *
  *   {                                                                        *
  *       int T0_out;                                                          *
  *       int T1_out;                                                          *
  *       float T0_degC;                                                       *
  *       float T1_degC;                                                       *
  *   } COMPONENT_Data_t;                                                      *
  *----------------------------------------------------------------------------*/
 typedef struct
 {
   /// Function pointer to flag interrupt call back
   void (*flagInterruptCallback)(void);
   /// Function pointer to error handler call back
   void (*errorHandlerCallback)(uint16_t error);
   bool spiPreemtionByIsr; // = FALSE;
   bool isrFlag; // = FALSE;
   /// L6474 Device Paramaters structure
   deviceParams_t devicePrm; //[MAX_NUMBER_OF_DEVICES];
   uint8_t number_of_devices;
   uint8_t device_instance;
   uint8_t spiTxBursts[L6474_CMD_ARG_MAX_NB_BYTES][MAX_NUMBER_OF_DEVICES];
   uint8_t spiRxBursts[L6474_CMD_ARG_MAX_NB_BYTES][MAX_NUMBER_OF_DEVICES];
 } L6474_Data_t;


 /* Functions -----------------------------------------------------------------*/

 /** @addtogroup BSP
  * @{
  */

 /** @addtogroup Components
  * @{
  */

 /** @addtogroup L6474
  * @{
  */

 /** @defgroup L6474_Imported_Functions L6474_Imported_Functions
  * @{
  */

 /* ACTION --------------------------------------------------------------------*
  * Declare here extern platform-dependent APIs you might need (e.g.: I/O and  *
  * interrupt related functions), and implement them in a glue-logic file on   *
  * the target environment, for example within the "x_nucleo_board.c" file.    *
  * E.g.:                                                                      *
  *   extern status_t COMPONENT_IO_Init (void *handle);                        *
  *   extern status_t COMPONENT_IO_Read (handle, buf, regadd, bytes);          *
  *   extern status_t COMPONENT_IO_Write(handle, buf, regadd, bytes);          *
  *   extern void     COMPONENT_IO_ITConfig(void);                             *
  *----------------------------------------------------------------------------*/
 ///Delay of the requested number of milliseconds
 extern void L6474_Delay(void *handle, uint32_t delay);
 ///Enable Irq
 extern void L6474_EnableIrq(void *handle);
 ///Disable Irq
 extern void L6474_DisableIrq(void *handle);
 ///Set PWM1 frequency and start it
 extern void L6474_Pwm1SetFreq(void *handle, uint16_t newFreq);
 ///Set PWM2 frequency and start it
 extern void L6474_Pwm2SetFreq(void *handle, uint16_t newFreq);
 ///Set PWM3 frequency and start it
 extern void L6474_Pwm3SetFreq(void *handle, uint16_t newFreq);
 ///Init the PWM
 extern void L6474_PwmInit(void *handle);
 ///Stop the PWM
 extern void L6474_PwmStop(void *handle);
 ///Reset the L6474 reset pin
 extern void L6474_ReleaseReset(void *handle);
 ///Set the L6474 reset pin
 extern void L6474_Reset(void *handle);
 ///Set direction GPIO
 extern void L6474_SetDirectionGpio(void *handle, uint8_t gpioState);
 ///Write bytes to the L6474s via SPI
 extern uint8_t L6474_SpiWriteBytes(void *handle, uint8_t *pByteToTransmit, uint8_t *pReceivedByte);

 #ifdef __cplusplus
   }
 #endif



#endif
