#ifndef L6474_CONFIG_H
#define L6474_CONFIG_H

#ifdef __cplusplus
 extern "C" {
#endif


/* Definitions ---------------------------------------------------------------*/

/** @addtogroup BSP
  * @{
  */

/** @addtogroup L6474
  * @{
  */

/** @addtogroup L6474_Exported_Constants
  * @{
  */

/** @defgroup Predefined_L6474_Registers_Values
  * @{
  */

/// The maximum number of devices in the daisy chain
#define MAX_NUMBER_OF_DEVICES                 (3)

/************************ Speed Profile  *******************************/

/// Acceleration rate in step/s2 for device 0 (must be greater than 0)
#define L6474_CONF_PARAM_ACC_DEVICE_0        (160)
/// Acceleration rate in step/s2 for device 1 (must be greater than 0)
#define L6474_CONF_PARAM_ACC_DEVICE_1        (160)
/// Acceleration rate in step/s2 for device 2 (must be greater than 0)
#define L6474_CONF_PARAM_ACC_DEVICE_2        (160)

/// Deceleration rate in step/s2 for device 0 (must be greater than 0)
#define L6474_CONF_PARAM_DEC_DEVICE_0        (160)
/// Deceleration rate in step/s2 for device 1 (must be greater than 0)
#define L6474_CONF_PARAM_DEC_DEVICE_1        (160)
/// Deceleration rate in step/s2 for device 2 (must be greater than 0)
#define L6474_CONF_PARAM_DEC_DEVICE_2        (160)

/// Maximum speed in step/s for device 0 (30 step/s < Maximum speed <= 10 000 step/s )
#define L6474_CONF_PARAM_MAX_SPEED_DEVICE_0  (1600)
/// Maximum speed in step/s for device 1 (30 step/s < Maximum speed <= 10 000 step/s )
#define L6474_CONF_PARAM_MAX_SPEED_DEVICE_1  (1600)
/// Maximum speed in step/s for device 2 (30 step/s < Maximum speed <= 10 000 step/s )
#define L6474_CONF_PARAM_MAX_SPEED_DEVICE_2  (1600)

/// Minimum speed in step/s for device 0 (30 step/s <= Minimum speed < 10 000 step/s)
#define L6474_CONF_PARAM_MIN_SPEED_DEVICE_0  (800)
/// Minimum speed in step/s for device 1 (30 step/s <= Minimum speed < 10 000 step/s)
#define L6474_CONF_PARAM_MIN_SPEED_DEVICE_1  (800)
/// Minimum speed in step/s for device 2 (30 step/s <= Minimum speed < 10 000 step/s)
#define L6474_CONF_PARAM_MIN_SPEED_DEVICE_2  (800)


/************************ Phase Current Control *******************************/

// Current value that is assigned to the torque regulation DAC
/// TVAL register value for device 0 (range 31.25mA to 4000mA)
#define L6474_CONF_PARAM_TVAL_DEVICE_0  (250)
/// TVAL register value for device 1 (range 31.25mA to 4000mA)
#define L6474_CONF_PARAM_TVAL_DEVICE_1  (250)
/// TVAL register value for device 2 (range 31.25mA to 4000mA)
#define L6474_CONF_PARAM_TVAL_DEVICE_2  (250)

/// Fall time value (T_FAST field of T_FAST register) for device 0 (range 2us to 32us)
#define L6474_CONF_PARAM_FAST_STEP_DEVICE_0  (L6474_FAST_STEP_12us)
/// Fall time value (T_FAST field of T_FAST register) for device 1 (range 2us to 32us)
#define L6474_CONF_PARAM_FAST_STEP_DEVICE_1  (L6474_FAST_STEP_12us)
/// Fall time value (T_FAST field of T_FAST register) for device 2 (range 2us to 32us)
#define L6474_CONF_PARAM_FAST_STEP_DEVICE_2  (L6474_FAST_STEP_12us)

/// Maximum fast decay time (T_OFF field of T_FAST register) for device 0 (range 2us to 32us)
#define L6474_CONF_PARAM_TOFF_FAST_DEVICE_0  (L6474_TOFF_FAST_8us)
/// Maximum fast decay time (T_OFF field of T_FAST register) for device 1 (range 2us to 32us)
#define L6474_CONF_PARAM_TOFF_FAST_DEVICE_1  (L6474_TOFF_FAST_8us)
/// Maximum fast decay time (T_OFF field of T_FAST register) for device 2 (range 2us to 32us)
#define L6474_CONF_PARAM_TOFF_FAST_DEVICE_2  (L6474_TOFF_FAST_8us)

/// Minimum ON time (TON_MIN register) for device 0 (range 0.5us to 64us)
#define L6474_CONF_PARAM_TON_MIN_DEVICE_0 (3)
/// Minimum ON time (TON_MIN register) for device 1 (range 0.5us to 64us)
#define L6474_CONF_PARAM_TON_MIN_DEVICE_1 (3)
/// Minimum ON time (TON_MIN register) for device 2 (range 0.5us to 64us)
#define L6474_CONF_PARAM_TON_MIN_DEVICE_2 (3)

/// Minimum OFF time (TOFF_MIN register) for device 0 (range 0.5us to 64us)
#define L6474_CONF_PARAM_TOFF_MIN_DEVICE_0 (21)
/// Minimum OFF time (TOFF_MIN register) for device 1 (range 0.5us to 64us)
#define L6474_CONF_PARAM_TOFF_MIN_DEVICE_1 (21)
/// Minimum OFF time (TOFF_MIN register) for device 2 (range 0.5us to 64us)
#define L6474_CONF_PARAM_TOFF_MIN_DEVICE_2 (21)

/******************************* Others ***************************************/

/// Overcurrent threshold settings for device 0 (OCD_TH register)
#define L6474_CONF_PARAM_OCD_TH_DEVICE_0  (L6474_OCD_TH_750mA)
/// Overcurrent threshold settings for device 1 (OCD_TH register)
#define L6474_CONF_PARAM_OCD_TH_DEVICE_1  (L6474_OCD_TH_750mA)
/// Overcurrent threshold settings for device 2 (OCD_TH register)
#define L6474_CONF_PARAM_OCD_TH_DEVICE_2  (L6474_OCD_TH_750mA)

/// Alarm settings for device 0 (ALARM_EN register)
#define L6474_CONF_PARAM_ALARM_EN_DEVICE_0  (L6474_ALARM_EN_OVERCURRENT |\
                                             L6474_ALARM_EN_THERMAL_SHUTDOWN |\
                                             L6474_ALARM_EN_THERMAL_WARNING |\
                                             L6474_ALARM_EN_UNDERVOLTAGE |\
                                             L6474_ALARM_EN_SW_TURN_ON |\
                                             L6474_ALARM_EN_WRONG_NPERF_CMD)

///Alarm settings for device 1 (ALARM_EN register)
#define L6474_CONF_PARAM_ALARM_EN_DEVICE_1  (L6474_ALARM_EN_OVERCURRENT |\
                                             L6474_ALARM_EN_THERMAL_SHUTDOWN |\
                                             L6474_ALARM_EN_THERMAL_WARNING |\
                                             L6474_ALARM_EN_UNDERVOLTAGE |\
                                             L6474_ALARM_EN_SW_TURN_ON |\
                                             L6474_ALARM_EN_WRONG_NPERF_CMD)

/// Alarm settings for device 2 (ALARM_EN register)
#define L6474_CONF_PARAM_ALARM_EN_DEVICE_2  (L6474_ALARM_EN_OVERCURRENT |\
                                             L6474_ALARM_EN_THERMAL_SHUTDOWN |\
                                             L6474_ALARM_EN_THERMAL_WARNING |\
                                             L6474_ALARM_EN_UNDERVOLTAGE |\
                                             L6474_ALARM_EN_SW_TURN_ON |\
                                             L6474_ALARM_EN_WRONG_NPERF_CMD)

/// Step selection settings for device 0 (STEP_SEL field of STEP_MODE register)
#define L6474_CONF_PARAM_STEP_SEL_DEVICE_0  (L6474_STEP_SEL_1_16)
/// Step selection settings for device 1 (STEP_SEL field of STEP_MODE register)
#define L6474_CONF_PARAM_STEP_SEL_DEVICE_1  (L6474_STEP_SEL_1_16)
/// Step selection settings for device 2 (STEP_SEL field of STEP_MODE register)
#define L6474_CONF_PARAM_STEP_SEL_DEVICE_2  (L6474_STEP_SEL_1_16)

/// Synch. selection settings for device 0 (SYNC_SEL field of STEP_MODE register)
#define L6474_CONF_PARAM_SYNC_SEL_DEVICE_0  (L6474_SYNC_SEL_1_2)
/// Synch. selection settings for device 1 (SYNC_SEL field of STEP_MODE register)
#define L6474_CONF_PARAM_SYNC_SEL_DEVICE_1  (L6474_SYNC_SEL_1_2)
/// Synch. selection settings for device 2 (SYNC_SEL field of STEP_MODE register)
#define L6474_CONF_PARAM_SYNC_SEL_DEVICE_2  (L6474_SYNC_SEL_1_2)

/// Target Swicthing Period for device 0 (field TOFF of CONFIG register)
#define L6474_CONF_PARAM_TOFF_DEVICE_0  (L6474_CONFIG_TOFF_044us)
/// Target Swicthing Period for device 1 (field TOFF of CONFIG register)
#define L6474_CONF_PARAM_TOFF_DEVICE_1  (L6474_CONFIG_TOFF_044us)
/// Target Swicthing Period for device 2 (field TOFF of CONFIG register)
#define L6474_CONF_PARAM_TOFF_DEVICE_2  (L6474_CONFIG_TOFF_044us)

/// Slew rate for device 0 (POW_SR field of CONFIG register)
#define L6474_CONF_PARAM_SR_DEVICE_0  (L6474_CONFIG_SR_320V_us)
/// Slew rate for device 1 (POW_SR field of CONFIG register)
#define L6474_CONF_PARAM_SR_DEVICE_1  (L6474_CONFIG_SR_320V_us)
/// Slew rate for device 2 (POW_SR field of CONFIG register)
#define L6474_CONF_PARAM_SR_DEVICE_2  (L6474_CONFIG_SR_320V_us)

/// Over current shutwdown enabling for device 0 (OC_SD field of CONFIG register)
#define L6474_CONF_PARAM_OC_SD_DEVICE_0  (L6474_CONFIG_OC_SD_ENABLE)
/// Over current shutwdown enabling for device 1 (OC_SD field of CONFIG register)
#define L6474_CONF_PARAM_OC_SD_DEVICE_1  (L6474_CONFIG_OC_SD_ENABLE)
/// Over current shutwdown enabling for device 2 (OC_SD field of CONFIG register)
#define L6474_CONF_PARAM_OC_SD_DEVICE_2  (L6474_CONFIG_OC_SD_ENABLE)

/// Torque regulation method for device 0 (EN_TQREG field of CONFIG register)
#define L6474_CONF_PARAM_TQ_REG_DEVICE_0  (L6474_CONFIG_EN_TQREG_TVAL_USED)
///Torque regulation method for device 1 (EN_TQREG field of CONFIG register)
#define L6474_CONF_PARAM_TQ_REG_DEVICE_1  (L6474_CONFIG_EN_TQREG_TVAL_USED)
/// Torque regulation method for device 2 (EN_TQREG field of CONFIG register)
#define L6474_CONF_PARAM_TQ_REG_DEVICE_2  (L6474_CONFIG_EN_TQREG_TVAL_USED)

/// Clock setting for device 0 (OSC_CLK_SEL field of CONFIG register)
#define L6474_CONF_PARAM_CLOCK_SETTING_DEVICE_0  (L6474_CONFIG_INT_16MHZ)
/// Clock setting for device 1 (OSC_CLK_SEL field of CONFIG register)
#define L6474_CONF_PARAM_CLOCK_SETTING_DEVICE_1  (L6474_CONFIG_INT_16MHZ)
/// Clock setting for device 2 (OSC_CLK_SEL field of CONFIG register)
#define L6474_CONF_PARAM_CLOCK_SETTING_DEVICE_2  (L6474_CONFIG_INT_16MHZ)

#ifdef __cplusplus
  }
#endif

#endif
