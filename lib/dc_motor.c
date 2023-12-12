/*
 * Copyright (c) 2023 Daniel Bucher
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#if Use_DC_Motor

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/gpio.h"
#include "dc_motor.h"
#include "myhandletype.h"

typedef struct
{
    my_handletype_t handletype;
    uint32_t gpio_mask;
    uint32_t f_sequence;
    uint32_t b_sequence;
    uint32_t s_sequence;
    uint8_t pwm_gpio;
    dcmotor_direction_t direction;
} dcmotor_t;

// Init gpio and pwm for a motor
dcmotor_Handle_t dcmotor_init(uint8_t gpio_in1, uint8_t gpio_in2, uint8_t gpio_pwm, dcmotor_mode_t dc_mode)
{
    dcmotor_t *handle;
    handle = (dcmotor_t *)malloc(sizeof(dcmotor_t));
    if (handle == NULL) // if malloc failed, will return NULL pointer 
    { 
        return handle;
    }
    memset(handle, 0, sizeof(dcmotor_t));
    handle->handletype = dcmotor_type;
    // Initialise GPIO. Use a bitmask to manipulate all pins at the same time.
    handle->gpio_mask = (1 << gpio_in1) | (1 << gpio_in2);
    gpio_init_mask(handle->gpio_mask | (1 << gpio_pwm));
    gpio_set_dir_out_masked(handle->gpio_mask | (1 << gpio_pwm));

    // Initialise PWM GPIO
    gpio_set_function(gpio_pwm, GPIO_FUNC_PWM);       // Tell the pin that the PWM is in charge of its value.
    uint slice_num = pwm_gpio_to_slice_num(gpio_pwm); // Figure out which slice we just connected to the pin
    pwm_config config = pwm_get_default_config();     // Get some sensible defaults for the slice configuration. By default, the
                                                      // counter is allowed to wrap over its maximum range (0 to 2**16-1)
    pwm_config_set_clkdiv(&config, 4.f);              // Set divider, reduces counter clock to sysclock/this value
    pwm_config_set_phase_correct(&config, true);      // Set phase correct modus
    pwm_init(slice_num, &config, true);               // Load the configuration into our PWM slice, and set it running.
    handle->pwm_gpio = gpio_pwm;

    // Set the konfiguration for the 
    if (dc_mode = fast_decay)
    {
        handle->f_sequence = (1 << gpio_in1);
        handle->b_sequence = (1 << gpio_in2);
        handle->s_sequence = (1 << gpio_in1) | (1 << gpio_in2);
    }
    else if (dc_mode = slow_decay)
    {
        handle->f_sequence = (1 << gpio_in1);
        handle->b_sequence = (1 << gpio_in2);
        handle->s_sequence = 0;
    }
    handle->direction = Stopp;  //set default direction stop

    return handle;
}

dcmotor_Handle_t dcmotor_deinit(dcmotor_Handle_t dcmotor)
{
    assert(dcmotor != NULL);
    dcmotor_t *m = (dcmotor_t *)dcmotor;
    if(m->handletype != dcmotor_type){
        printf("Errore: Wrong handletype, should be a handle of a dc-motor. file \"%s\", line %d\n",__FILE__,__LINE__);
        return dcmotor;
    }
    gpio_deinit(m->pwm_gpio);
    m->handletype = null;
    free(dcmotor);
    return NULL;
}

/// set the outputs for the motor (motor, direction, value(0>=Value>=65535))
void dcmotor_set(dcmotor_Handle_t dcmotor, dcmotor_direction_t dir, uint16_t value)
{
    assert(dcmotor != NULL);
    dcmotor_t *m = (dcmotor_t *)dcmotor;
    if(m->handletype != dcmotor_type){
        printf("Errore: Wrong handletype, should be a handle of a dc-motor. file \"%s\", line %d\n",__FILE__,__LINE__);
        return;
    }
    m->direction = dir;
    dcmotor_change_speed(m, value);
}

void dcmotor_change_speed(dcmotor_Handle_t dcmotor, uint16_t value)
{
    assert(dcmotor != NULL);
    dcmotor_t *m = (dcmotor_t *)dcmotor;
    if(m->handletype != dcmotor_type){
        printf("Errore: Wrong handletype, should be a handle of a dc-motor. file \"%s\", line %d\n",__FILE__,__LINE__);
        return;
    }
    switch (m->direction)
    {
    case Stopp:
    {
        gpio_put_masked(m->gpio_mask, m->s_sequence);
        pwm_set_gpio_level(m->pwm_gpio, 0);
        break;
    }
    case Clockwise:
    {
        gpio_put_masked(m->gpio_mask, m->f_sequence);
        pwm_set_gpio_level(m->pwm_gpio, value);
        break;
    }
    case Counterclockwise:
    {
        gpio_put_masked(m->gpio_mask, m->b_sequence);
        pwm_set_gpio_level(m->pwm_gpio, value);
        break;
    }
    }
}

void dcmotor_set_value(dcmotor_Handle_t dcmotor, int32_t value)
{
    assert(dcmotor != NULL);
    dcmotor_t *m = (dcmotor_t *)dcmotor;
    if(m->handletype != dcmotor_type){
        printf("Errore: Wrong handletype, should be a handle of a dc-motor. file \"%s\", line %d\n",__FILE__,__LINE__);
        return;
    }
    if (value > 0)
    {
        m->direction = Clockwise;
    }
    else if (value < 0)
    {
        value = value * -1;
        m->direction = Counterclockwise;
    }
    else
    {
        m->direction = Stopp;
    }
    if (value > 65535)
        value = 65535;
    dcmotor_change_speed(m, (uint16_t)value);
}

void dcmotor_stop(dcmotor_Handle_t dcmotor)
{
    assert(dcmotor != NULL);
    dcmotor_t *m = (dcmotor_t *)dcmotor;
    if(m->handletype != dcmotor_type){
        printf("Errore: Wrong handletype, should be a handle of a dc-motor. file \"%s\", line %d\n",__FILE__,__LINE__);
        return;
    }
    gpio_put_masked(m->gpio_mask, m->s_sequence);
    pwm_set_gpio_level(m->pwm_gpio, 0);
    m->direction = Stopp;
}
#endif
