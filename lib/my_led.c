/*
 * Copyright (c) 2023 Daniel Bucher
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
#if Use_Led

#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "my_led.h"

static LED_Status_t Led_Status;

void myLed_Init(void){
    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
    Led_set(OFF);
    printf("LOG_LED initalisiert\n");
}

LED_Status_t get_Led_Status(void){
    return Led_Status;
}

void Led_set(LED_Status_t led_Status){
    Led_Status = led_Status;
    switch (led_Status)
    {   
        case OFF:{
            gpio_put(PICO_DEFAULT_LED_PIN, false);
            break;
        } 
        case ON:{
            gpio_put(PICO_DEFAULT_LED_PIN, true);
            break;
        }       
        default:{
            break;
        }
    }      
}

#endif


