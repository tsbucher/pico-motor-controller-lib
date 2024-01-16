/*
 * Copyright (c) 2021 Antonio González
 * Copyright (c) 2023 Daniel Bucher
 *
 * SPDX-License-Identifier: BSD-3-Clause
 * 
 */

#ifndef _STEPPER_H_
#define _STEPPER_H_

#include "pico/stdlib.h"
#include "includemyPicolib_config.h"
#include "hardware/gpio.h"
#include "hardware/timer.h"

#if Use_Stepper_Motor_With_Repeating_Timer

typedef enum {
    forward = -1,
    backward = 1
} stepper_direction_t;

typedef void *stepper_Handle_t;

typedef enum {
    Single,     //wave drive modus
    Power,      //normal full step
    Half        //half step
} stepper_mode_t;

/// @brief Initialise a stepper motor
/// @param gpio_A1 GPIO to control coil pair A
/// @param gpio_B1 GPIO to control coil pair B
/// @param gpio_A2 GPIO to control coil pair A
/// @param gpio_B2 GPIO to control coil pair B
/// @param steps_per_revolution Number of steps for one revolution
/// @param stepping_mode Firing sequence mode (single or power or half)
stepper_Handle_t stepper_init(uint8_t gpio_A1, uint8_t gpio_B1, uint8_t gpio_A2, uint8_t gpio_B2,
                              uint16_t steps_per_revolution,
                              stepper_mode_t stepping_mode);


/// @brief Set motor speed in RPM
/// @param stepper Handel from a stepper motor
/// @param rpm Speed in revolutions per minute
void stepper_set_speed_rpm(stepper_Handle_t stepper, float rpm);

/// @brief Accelerates a stepper motor to the specified final speed
/// @param stepper Handel from a stepper motor
/// @param endspeed Final speed
/// @param time_ms Acceleration time
void stepper_accelerate(stepper_Handle_t stepper,float endspeed, int time_ms);


/// @brief Rotate the motor one step(or halfstep)
/// @param stepper Handel from a stepper motor
void stepper_step_once(stepper_Handle_t stepper);


/// @brief De-activate all coils
/// @param stepper Handel from a stepper motor
void stepper_release(stepper_Handle_t stepper);


/// @brief Rotate the motor these many steps
/// @param stepper Handel from a stepper motor
/// @param steps How many steps to rotate. The sign indicates direction
void stepper_rotate_steps(stepper_Handle_t stepper, int32_t steps);


/// @brief Rotate the motor these many degrees
/// @param stepper Handel from a stepper motor
/// @param degrees The degrees (angle) to rotate. The sign indicates direction
/// 
/// Because the motor rotates at discrete steps, the actual angle rotated
/// will often be an approximation to that requested, with an error that
/// depends on the number of steps per revolution of the motor and the modus used. E.g. a
/// motor with 200 steps per revolution will have an error of up to
/// 360/200 = 1.8 degrees with the power or singel stepping modus.
void stepper_rotate_degrees(stepper_Handle_t stepper, float degrees);

/// @brief reset the actual stepper Position to zero
/// @param stepper Handel from a stepper motor
void stepper_reset_position(stepper_Handle_t stepper);

/// @brief get the actual position from a stepper motor
/// @param stepper Handel from a stepper motor
/// @return the position in steps
///         (when the number of steps reaches steps_per_revolution it wraps back to zero) 
int32_t stepper_get_position(stepper_Handle_t stepper);

/// @brief get the actual position from a stepper motor in degrees
/// @param stepper Handel from a stepper motor
/// @return the position in degrees
///         (the position zero is 0°, the angel is positive in clockwise direction)
int16_t stepper_get_position_degrees(stepper_Handle_t stepper);

/// @brief cheks if the stepper motor is turning
/// @param stepper Handel from a stepper motor
/// @return true when the motor is moving/ false when the motor is stopt
bool stepper_is_moving(stepper_Handle_t stepper);

/// @brief Deinitialise a stepper motor
/// @param stepper Initialise a stepper motor
/// @return NULL
stepper_Handle_t stepper_deinit(stepper_Handle_t stepper);

#endif

#endif
