#ifndef STEPPER_MOTOR_H
#define STEPPER_MOTOR_H

#include <Component.h>

class StepperMotor : public Component {
public:

    /**
     * @brief Rotation modes.
     */
    typedef enum {
        BWD = 0, /* Backward. */
        FWD = 1  /* Forward. */
    } direction_t;

    /**
     * @brief Step modes.
     */
    typedef enum {
        STEP_MODE_FULL = 0, /* Full-step. */
        STEP_MODE_HALF,     /* Half-step. */
        STEP_MODE_1_4,      /* 1/4 microstep. */
        STEP_MODE_1_8,      /* 1/8 microstep. */
        STEP_MODE_1_16,     /* 1/16 microstep. */
        STEP_MODE_1_32,     /* 1/32 microstep. */
        STEP_MODE_1_64,     /* 1/64 microstep. */
        STEP_MODE_1_128,    /* 1/128 microstep. */
        STEP_MODE_1_256,    /* 1/256 microstep. */
        STEP_MODE_UNKNOWN,  /* Unknown. */
        STEP_MODE_WAVE      /* Full-step one-phase-on. */
    } step_mode_t;

    /**
     * @brief  Getting the status.
     * @param  None.
     * @retval The status.
     */
    virtual unsigned int get_status(void) = 0;

    /**
     * @brief  Getting the position.
     * @param  None.
     * @retval The position.
     */
    virtual signed int get_position(void) = 0;

    /**
     * @brief  Getting the marked position.
     * @param  None.
     * @retval The marked position.
     */
    virtual signed int get_mark(void) = 0;

    /**
     * @brief  Getting the current speed in pps.
     * @param  None.
     * @retval The current speed in pps.
     */
    virtual unsigned int get_speed(void) = 0;

    /**
     * @brief  Getting the maximum speed in pps.
     * @param  None.
     * @retval The maximum speed in pps.
     */
    virtual unsigned int get_max_speed(void) = 0;

    /**
     * @brief  Getting the minimum speed in pps.
     * @param  None.
     * @retval The minimum speed in pps.
     */
    virtual unsigned int get_min_speed(void) = 0;

    /**
     * @brief  Getting the acceleration in pps^2.
     * @param  None.
     * @retval The acceleration in pps^2.
     */
    virtual unsigned int get_acceleration(void) = 0;

    /**
     * @brief  Getting the deceleration in pps^2.
     * @param  None.
     * @retval The deceleration in pps^2.
     */
    virtual unsigned int get_deceleration(void) = 0;

    /**
     * @brief  Getting the direction of rotation.
     * @param  None.
     * @retval The direction of rotation.
     */
    virtual direction_t get_direction(void) = 0;

    /**
     * @brief  Setting the current position to be the home position.
     * @param  None.
     * @retval None.
     */
    virtual void set_home(void) = 0;

    /**
     * @brief  Setting the current position to be the marked position.
     * @param  None.
     * @retval None.
     */
    virtual void set_mark(void) = 0;

    /**
     * @brief  Setting the maximum speed in pps.
     * @param  speed The maximum speed in pps.
     * @retval "true" in case of success, "false" otherwise.
     */
    virtual bool set_max_speed(unsigned int speed) = 0;

    /**
     * @brief  Setting the minimum speed in pps.
     * @param  speed The minimum speed in pps.
     * @retval "true" in case of success, "false" otherwise.
     */
    virtual bool set_min_speed(unsigned int speed) = 0;

    /**
     * @brief  Setting the acceleration in pps^2.
     * @param  acceleration The acceleration in pps^2.
     * @retval "true" in case of success, "false" otherwise.
     */
    virtual bool set_acceleration(unsigned int acceleration) = 0;

    /**
     * @brief  Setting the deceleration in pps^2.
     * @param  deceleration The deceleration in pps^2.
     * @retval "true" in case of success, "false" otherwise.
     */
    virtual bool set_deceleration(unsigned int deceleration) = 0;

    /**
     * @brief  Setting the Step Mode.
     * @param  step_mode The Step Mode.
     * @retval "true" in case of success, "false" otherwise.
     */
    virtual bool set_step_mode(step_mode_t step_mode) = 0;

    /**
     * @brief  Going to a specified position.
     * @param  position The desired position.
     * @retval None.
     */
    virtual void go_to(signed int position) = 0;

    /**
     * @brief  Going to the home position.
     * @param  None.
     * @retval None.
     */
    virtual void go_home(void) = 0;

    /**
     * @brief  Going to the marked position.
     * @param  None.
     * @retval None.
     */
    virtual void go_mark(void) = 0;

    /**
     * @brief  Running the motor towards a specified direction.
     * @param  direction The direction of rotation.
     * @retval None.
     */
    virtual void run(direction_t direction) = 0;

    /**
     * @brief  Moving the motor towards a specified direction for a certain number of steps.
     * @param  direction The direction of rotation.
     * @param  steps The desired number of steps.
     * @retval None.
     */
    virtual void move(direction_t direction, unsigned int steps) = 0;

    /**
     * @brief  Stopping the motor through an immediate deceleration up to zero speed.
     * @param  None.
     * @retval None.
     */
    virtual void soft_stop(void) = 0;

    /**
     * @brief  Stopping the motor through an immediate infinite deceleration.
     * @param  None.
     * @retval None.
     */
    virtual void hard_stop(void) = 0;

    /**
     * @brief  Disabling the power bridge after performing a deceleration to zero.
     * @param  None.
     * @retval None.
     */
    virtual void soft_hiz(void) = 0;

    /**
     * @brief  Disabling the power bridge immediately.
     * @param  None.
     * @retval None.
     */
    virtual void hard_hiz(void) = 0;

    /**
     * @brief  Waiting while the motor is active.
     * @param  None.
     * @retval None.
     */
    virtual void wait_while_active(void) = 0;

    /**
     * @brief Destructor.
     */
    virtual ~StepperMotor() {};
};

#endif
