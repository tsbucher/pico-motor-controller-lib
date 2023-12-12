/*
 * Copyright (c) 2023 Daniel Bucher
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef my_DCMotor
#define my_DCMotor

#include "pico/stdlib.h"
#include "includemyPicolib_config.h"

#if Use_DC_Motor


/// @brief Direction for the dc-motor
typedef enum
{
    Stopp,
    Clockwise,
    Counterclockwise,
} dcmotor_direction_t;

/// @brief Handel to a dc-Motor
typedef void *dcmotor_Handle_t;

/// @brief Modus for a dc-Motor
typedef enum
{
    fast_decay,
    slow_decay
} dcmotor_mode_t;

/// @brief Initialise a dc-motor
/// @param gpio_in1 GPIO for one direction signal
/// @param gpio_in2 GPIO for the other direction signal
/// @param gpio_pwm GPIO for the PWM Signal
/// @param dc_mode  Mode off H-Bridge
/// @return a handle for the dc-motor(dcmotor_Handle_t) or NULL when somthing went wrong
dcmotor_Handle_t dcmotor_init(uint8_t gpio_in1, uint8_t gpio_in2, uint8_t gpio_pwm, dcmotor_mode_t dc_mode);

/// @brief deinitialises a dc-motor
/// @param dcmotor Handle from a dc-motor
/// @return NULL
dcmotor_Handle_t dcmotor_deinit(dcmotor_Handle_t dcmotor);

/// @brief Set the motor to a value and direction
/// @param dir Direction (stopp, forward, backward)
/// @param dcmotor Handle from a dc-motor
/// @param value value (0-65535)
void dcmotor_set(dcmotor_Handle_t dcmotor, dcmotor_direction_t dir, uint16_t value);

/// @brief Set the motor to a value(only in the actually configured direction)
/// @param dcmotor Handle from a dc-motor
/// @param value value (0-65535)
void dcmotor_change_speed(dcmotor_Handle_t dcmotor, uint16_t value);

/// @brief Set the motor to a value(the ___ declares the direction)
/// @param dcmotor Handle from a dc-motor
/// @param value value  (-65535-65535)
void dcmotor_set_value(dcmotor_Handle_t dcmotor, int32_t value);

/// @brief Stops a dc-motor
/// @param dcmotor Handle from a dc-motor
void dcmotor_stop(dcmotor_Handle_t dcmotor);

#endif

#endif