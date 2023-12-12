/*
 * Copyright (c) 2023 Daniel Bucher
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#if Use_Quadrature_Encoder

#include "includemyPicolib_config.h"
#include <stdlib.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "quadrature_encoder.pio.h"
#include "quadrature_encoder.h"
#include "pico/critical_section.h"
#include "myhandletype.h"

PIO pio = Quadrature_Encoder_PIO;

static bool pio_initalisiert = false;

typedef struct
{
    my_handletype_t handletype;
    uint16_t puls_per_rotation;
    uint sm;
    int old_value;
    uint64_t old_time;
    int new_value;
    uint64_t new_time;
    bool on;
    struct repeating_timer timer;
    critical_section_t crit_sec;
} quadrature_encoder_t;

bool repeating_timer_callback_quadratur_encoder(struct repeating_timer *t)
{
    quadrature_encoder_t *encoder = (quadrature_encoder_t *)(t->user_data);
    critical_section_enter_blocking(&encoder->crit_sec);
    encoder->old_value = encoder->new_value;
    encoder->old_time = encoder->new_time;
    encoder->new_value = Quadratur_Encoder_get_count(encoder);
    encoder->new_time = time_us_64();
    critical_section_exit(&encoder->crit_sec);
    return true;
}

quadrature_encoder_Handle_t Quadratur_Encoder_Init(uint8_t gpio_pin1, int max_step_rate, uint16_t puls_per_rotation)
{   
    if (!pio_initalisiert){
        pio_add_program(pio, &quadrature_encoder_program);
        pio_initalisiert = true;
    }
    quadrature_encoder_t *handle;
    handle = (quadrature_encoder_t *)malloc(sizeof(quadrature_encoder_t));
    if (handle == NULL)
    { // if malloc failed, will return NULL pointer 
        return handle;
    }
    memset(handle, 0, sizeof(quadrature_encoder_t)); 
    handle->handletype = quadraturencoder_type;
    handle->puls_per_rotation = puls_per_rotation;
    handle->sm = pio_claim_unused_sm(pio, false);
    if (handle->sm == -1){
        printf("Errore: there was no free state machine. file \"%s\", line %d\n",__FILE__,__LINE__);
        return NULL;
    }
    quadrature_encoder_program_init(pio, handle->sm, gpio_pin1, max_step_rate);
    critical_section_init(&handle->crit_sec);
    return handle;
}

int32_t Quadratur_Encoder_get_count(quadrature_encoder_Handle_t encoder)
{
    assert(encoder != NULL);
    quadrature_encoder_t *q = (quadrature_encoder_t *)encoder;
    if(q->handletype != quadraturencoder_type){
        printf("Errore: Wrong handletype, should be a handle of a quadratur_encoder. file \"%s\", line %d\n",__FILE__,__LINE__);
        return 0;
    }
    return (quadrature_encoder_get_count(pio, q->sm));
}

float static Quadratur_Encoder_get_speed_once(quadrature_encoder_Handle_t encoder)
{
    assert(encoder != NULL);
    quadrature_encoder_t *q = (quadrature_encoder_t *)encoder;
    if(q->handletype != quadraturencoder_type){
        printf("Errore: Wrong handletype, should be a handle of a quadratur_encoder. file \"%s\", line %d\n",__FILE__,__LINE__);
        return 0;
    }
    int old_value = Quadratur_Encoder_get_count(q);
    sleep_ms(Quadrature_Encoder_Periode);
    int new_value = Quadratur_Encoder_get_count(q);
    int delta = new_value - old_value;
    old_value = new_value;
    return ((float)delta * 1000 / Quadrature_Encoder_Periode * 60 / (q->puls_per_rotation));
}

float Quadratur_Encoder_get_speed(quadrature_encoder_Handle_t encoder)
{
    assert(encoder != NULL);
    quadrature_encoder_t *q = (quadrature_encoder_t *)encoder;
    if(q->handletype != quadraturencoder_type){
        printf("Errore: Wrong handletype, should be a handle of a quadratur_encoder. file \"%s\", line %d\n",__FILE__,__LINE__);
        return 0;
    }
    if (q->on)
    {   
        critical_section_enter_blocking(&q->crit_sec);
        float delta_count = (q->new_value-(q->old_value));
        float delta_time = ((float)us_to_ms(q->new_time-(q->old_time))/(float)1000);
        critical_section_exit(&q->crit_sec);
        return ((delta_count / delta_time / (float)(q->puls_per_rotation)) * 60);
    }
    else
    {
        return Quadratur_Encoder_get_speed_once(q);
    }
}

bool Quadratur_Encoder_start(quadrature_encoder_Handle_t encoder)
{
    assert(encoder != NULL);
    quadrature_encoder_t *q = (quadrature_encoder_t *)encoder;
    if(q->handletype != quadraturencoder_type){
        printf("Errore: Wrong handletype, should be a handle of a quadratur_encoder. file \"%s\", line %d\n",__FILE__,__LINE__);
        return false;
    }
    critical_section_enter_blocking(&q->crit_sec);
    q->old_value = Quadratur_Encoder_get_count(encoder);
    q->old_time = time_us_64();
    critical_section_exit(&q->crit_sec);
    if (!add_repeating_timer_ms(-Quadrature_Encoder_Periode, repeating_timer_callback_quadratur_encoder, q, &q->timer))
    {
        printf("Errore: failed to initialise a new timer. file \"%s\", line %d\n",__FILE__,__LINE__);
        return false;
    }
    q->on = true;
    return true;
}

void Quadratur_Encoder_stop(quadrature_encoder_Handle_t encoder)
{
    assert(encoder != NULL);
    quadrature_encoder_t *q = (quadrature_encoder_t *)encoder;
    if(q->handletype != quadraturencoder_type){
        printf("Errore: Wrong handletype, should be a handle of a quadratur_encoder. file \"%s\", line %d\n",__FILE__,__LINE__);
        return ;
    }
    cancel_repeating_timer(&q->timer);
    q->on = false;
}

quadrature_encoder_Handle_t Quadratur_Encoder_Deinit(quadrature_encoder_Handle_t encoder){
    assert(encoder!= NULL);
    quadrature_encoder_t *q = (quadrature_encoder_t *)encoder;
    if(q->handletype != quadraturencoder_type){
        printf("Errore: Wrong handletype, should be a handle of a quadratur_encoder. file \"%s\", line %d\n",__FILE__,__LINE__);
        return encoder;
    }
    cancel_repeating_timer(&q->timer);
    critical_section_deinit(&q->crit_sec);
    q->handletype = null;
    free(encoder);
    return NULL;
}

#endif
