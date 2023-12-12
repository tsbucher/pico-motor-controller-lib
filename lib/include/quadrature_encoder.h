/*
 * Copyright (c) 2023 Daniel Bucher
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */


#ifndef Quadratur_Encoder_h_
#define Quadratur_Encoder_h_

#include <stdio.h>
#include "hardware/pio.h"

#if Use_Quadrature_Encoder

/// @brief Handle for a quadratue encoder
typedef void *quadrature_encoder_Handle_t;

/// @brief initialise a quadratur encoder
/// @param gpio_pin1 First wire from the Encoder(the second has to be de next bigger gpio number)
/// @param max_step_rate max_step_rate is used to lower the clock of the state machine to save power
///                      if the application doesn't require a very high sampling rate. 
///                      Passing zero will set the clock to the maximum
/// @return a handle for the encoder(quadratur_encoder_Handle_t) or NULL when somthing went wrong
quadrature_encoder_Handle_t Quadratur_Encoder_Init(uint8_t gpio_pin1, int max_step_rate, uint16_t puls_per_rotation);

/// @brief get the actual count from the quadratu encoder
/// @param encoder Handle from an encoder
/// @return (int32_t)the actual count value
int32_t Quadratur_Encoder_get_count(quadrature_encoder_Handle_t encoder);

/// @brief get the speed from an encoder
/// @param encoder Handle from an encoder
/// @return (float)speed in rpm
float Quadratur_Encoder_get_speed(quadrature_encoder_Handle_t encoder);

/// @brief starts repeated scanning of an encoder 
/// @param encoder Handle from an encoder
/// @return true when everything works well/ false when an erroer has occurred
bool Quadratur_Encoder_start(quadrature_encoder_Handle_t encoder);

/// @brief stops repeated scanning of an encoder 
/// @param encoder Handle from an encoder
void Quadratur_Encoder_stop(quadrature_encoder_Handle_t encoder);

/// @brief deinitialises a quadratur encoder 
/// @param encoder Handle from an encoder
/// @return NULL
quadrature_encoder_Handle_t Quadratur_Encoder_Deinit(quadrature_encoder_Handle_t encoder);

#endif

#endif