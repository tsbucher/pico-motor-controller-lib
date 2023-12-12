/*
 * Copyright (c) 2023 Daniel Bucher
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef myAdc
#define myAdc

#include "pico/stdlib.h"
#include "includemyPicolib_config.h"

#if Use_Adc

typedef void *adc_sensor_Handle_t; 

/// @brief initialise a ADC for a GPIO Pin
/// @param gpio the GPIO numer from the analog input Pin
/// @return a adc_Handle(adc_sensor_Handle_t)
adc_sensor_Handle_t my_adc_init(uint gpio);

/// @brief make a sample by one adc
/// @param sensor handle from an ADC object
void my_adc_sampel(adc_sensor_Handle_t sensor);

/// @brief make a sample and get the result from a adc pin
/// @param sensor handle from an ADC object
/// @return the actual value ath that pin
/// Ths function use the read function which needs 2 mikro seconds to get the value.
uint16_t my_adc_sampelandread(adc_sensor_Handle_t sensor);

/// @brief get the last value from an adc pin
/// @param sensor handle from an ADC object
/// @return the latest sampled value from this adc pin
uint16_t my_adc_get(adc_sensor_Handle_t sensor);

/// @brief deinitialise a ADC for a GPIO Pin
/// @param sensor handle from an ADC object
/// @return NULL
adc_sensor_Handle_t my_adc_deinit(adc_sensor_Handle_t sensor);

#endif
#endif