/*
 * Copyright (c) 2023 Daniel Bucher
 *
 * SPDX-License-Identifier: BSD-3-Clause
 * 
 */

#ifndef PIO_STEPPER_H_
#define PIO_STEPPER_H_

#include "pico/stdlib.h"
#include "includemyPicolib_config.h"
#include "hardware/gpio.h"
#include "hardware/timer.h"

#if Use_Stepper_Motor_With_PIO

typedef enum {
    stop,
    clockwise,
    counterclockwise,
} pio_stepper_direction_t;  //rotarydirection of a stepper motor


typedef enum {
    singel = 0,     //wave drive modus
    power = 1,      //normal full step
    half = 2,       //half step
} pio_stepper_mode_t;

typedef void *pio_stepper_Handle_t;

/// @brief Initialise a stepper motor
/// @param stepper Handel from a stepper motor
/// @param gpio_A1 GPIO to control coil pair A
/// @param gpio_B1 GPIO to control coil pair B
/// @param gpio_A2 GPIO to control coil pair A
/// @param gpio_B2 GPIO to control coil pair B
/// @param steps_per_revolution Number of steps for one revolution
/// @param stepping_mode Firing sequence mode (power, singel or half)
pio_stepper_Handle_t pio_stepper_init(uint8_t gpio_A1, uint8_t gpio_B1, uint8_t gpio_A2, uint8_t gpio_B2,
                                      uint16_t steps_per_revolution,
                                      pio_stepper_mode_t stepping_mode);


/// @brief Set motor speed in RPM
/// @param stepper Handel from a stepper motor
/// @param rpm Speed in revolutions per minute
void pio_stepper_set_speed_rpm(pio_stepper_Handle_t stepper, float rpm);


/// @brief Change the stepping mode from a stepper motor
/// @param stepper Handel from a stepper motor
/// @param stepping_mode Firing sequence mode (singel, power or half)
///
/// Caution when the mode is changed to or from half during rotation!  
/// The counted steps are afterwords halfsteps and the rotations angele
/// isn't correct after the rotation
void pio_stepper_set_mode(pio_stepper_Handle_t stepper, pio_stepper_mode_t stepping_mode);


/// @brief Change the rotation direction from a stepper motor
/// @param stepper Handel from a stepper motor
/// @param direction Rotation direction
void pio_stepper_set_direction(pio_stepper_Handle_t stepper, pio_stepper_direction_t direction);


/// @brief Accelerates a stepper motor to the specified final speed
/// @param stepper Handel from a stepper motor
/// @param endspeed Final speed
/// @param time_ms Acceleration time
void pio_stepper_accelerate(pio_stepper_Handle_t stepper,float endspeed, int time_ms);


/// @brief Rotate the motor one step(or halfstep)
/// @param stepper Handel from a stepper motor
/// @return false if the motor is already moving and true if it has workd
bool pio_stepper_step_once(pio_stepper_Handle_t stepper);


/// @brief De-activate all coils
/// @param stepper Handel from a stepper motor
void pio_stepper_release(pio_stepper_Handle_t stepper);


/// @brief Rotate the motor these many steps
/// @param stepper Handel from a stepper motor
/// @param steps How many steps to rotate. The sign indicates direction
/// @return 
bool pio_stepper_rotate_steps(pio_stepper_Handle_t stepper, int32_t steps);


/// @brief Rotate the motor these many degrees
/// @param stepper Handel from a stepper motor
/// @param degrees The degrees (angle) to rotate. The sign indicates direction
void pio_stepper_rotate_degrees(pio_stepper_Handle_t stepper, float degrees);


/// @brief Check if th stepper motor is turnig in th moment
/// @param stepper Handel from a stepper motor
/// @return True, if the motor turns in the moment
bool pio_stepper_is_moving(pio_stepper_Handle_t stepper);


#endif

#endif
