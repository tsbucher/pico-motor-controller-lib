/*
 * Copyright (c) 2023 Daniel Bucher
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef my_Led_h
#define my_Led_h

#include "pico/stdlib.h"
#include "includemyPicolib_config.h"

#if Use_Led

/// @brief Modus der LED
typedef enum 
{
    OFF,
    ON,
}LED_Status_t;


/// @brief Initalisiert die LED
void myLed_Init(void);

LED_Status_t get_Led_Status(void);

/// @brief Steuert die Led auf vom Raspberry Pi Pico
/// @param led_Status (OFF, ON, Blinky_slow, Blinky_fast)
void Led_set(LED_Status_t led_Status);

#endif

#endif