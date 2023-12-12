/*
 * Copyright (c) 2023 Daniel Bucher
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */


#ifndef my_Button_h
#define my_Button_h

#include "pico/stdlib.h"
#include "includemyPicolib_config.h"

#if Use_Button


//define the size or number of diffrent buttons to be used
#define buttonlist_storagesize 4
#define debounce_delay_time 20

typedef enum {
    none,
    pull_up,
    pull_down
} pull_t;

typedef void * button_Handle_t;

void button_irq_handler(void);

/// @brief This function init a button_t object with diffrent configuration options
/// @param gpio the GPIO-Number wher the button is conected
/// @param event sets the events which trigger an interrupt (enum gpio_irq_level e.g. GPIO_IRQ_EDGE_FALL)
/// @param pull sets the pull up/down from the gpio pin (none, pull_up, pull_down)
/// @param debounce if True, debouncing is initialized
/// @return a handle for the button(button_Handle_t) or NULL when somthing went wrong
///
/// This function does not necessarily have to be used for buttons.
/// Other digital inputs (such as switches, limit switches or light barriers) are also possible
button_Handle_t button_init(uint8_t gpio, uint32_t event, pull_t pull, bool debounce);

/// @brief Get the inputvalue from a button
/// @param button Handle from an button
/// @return the bool value from the input pin
bool button_get(button_Handle_t button);

/// @brief deinitialises a button
/// @param button Handle from an button
/// @return NULL
button_Handle_t button_deinit(button_Handle_t button);

#endif

#endif