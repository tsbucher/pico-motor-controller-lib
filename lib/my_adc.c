/*
 * Copyright (c) 2023 Daniel Bucher
 * Copyright (c) 2021 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
#if Use_Adc

#include <stdio.h>
#include <stdlib.h>
#include <string.h> 
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/adc.h"
#include "hardware/dma.h"
#include "myhandletype.h"


#include "my_adc.h"

typedef struct {
    my_handletype_t handletype;
    uint adc_num;
    uint16_t lastresult;
#if Adc_Mean_Enable
    uint16_t buffer[Adc_Mean_Num];
#endif
 } adcsensor_t;


int64_t alarm_callback_adc(alarm_id_t id, void *sensor) {
    adcsensor_t mysensor = *(adcsensor_t*)sensor;
    mysensor.lastresult = (uint16_t) adc_hw->result;
    return 0;
}

adc_sensor_Handle_t my_adc_init(uint gpio){
    adcsensor_t *handle;
    handle = (adcsensor_t *)malloc(sizeof(adcsensor_t));
    if (handle == NULL) // if malloc failed, will return NULL pointer 
    { 
        return handle;
    }
    memset(handle, 0, sizeof(adcsensor_t));
    handle->handletype = adc_type;
    if (26 <= gpio && gpio <= 28){
        adc_gpio_init(gpio);                
        handle->adc_num = gpio - 26;       
        return handle; 
    }
    else{
        return NULL;
    }    
}

void my_adc_sampel(adc_sensor_Handle_t sensor){
    assert(sensor != NULL);
    adcsensor_t *s = (adcsensor_t *)sensor;
    if(s->handletype != adc_type){
        printf("Errore: Wrong handletype, should be a handle of a adc_sensor. file \"%s\", line %d\n",__FILE__,__LINE__);
        return ;
    }
#if Adc_Mean_Enable
    adc_select_input(s->adc_num);
    adc_fifo_setup(
        true,    // Write each completed conversion to the sample FIFO
        true,    // Enable DMA data request (DREQ)
        1,       // DREQ (and IRQ) asserted when at least 1 sample present
        false,   // We won't see the ERR bit because of 8 bit reads; disable.
        true     // Shift each sample to 8 bits when pushing to FIFO
    );
     // Set up the DMA to start transferring data as soon as it appears in FIFO
    uint dma_chan = dma_claim_unused_channel(true);
    dma_channel_config cfg = dma_channel_get_default_config(s->adc_num);

    // Reading from constant address, writing to incrementing byte addresses
    channel_config_set_transfer_data_size(&cfg, DMA_SIZE_8);
    channel_config_set_read_increment(&cfg, false);
    channel_config_set_write_increment(&cfg, true);

    // Pace transfers based on availability of ADC samples
    channel_config_set_dreq(&cfg, DREQ_ADC);

    dma_channel_configure(dma_chan, &cfg,
        s->buffer,    // dst
        &adc_hw->fifo,  // src
        Adc_Mean_Num,  // transfer count
        true            // start immediately
    );
    adc_run(true);

    dma_channel_wait_for_finish_blocking(dma_chan);
    adc_run(false);
    adc_fifo_drain();
#else 
    adc_select_input(s->adc_num);
    hw_set_bits(&adc_hw->cs, ADC_CS_START_ONCE_BITS);
    add_alarm_in_us(3,alarm_callback_adc,s,true);
#endif
}


uint16_t my_adc_sampelandread(adc_sensor_Handle_t sensor){
    assert(sensor != NULL);
    adcsensor_t *s = (adcsensor_t *)sensor;
    if(s->handletype != adc_type){
        printf("Errore: Wrong handletype, should be a handle of a adc_sensor. file \"%s\", line %d\n",__FILE__,__LINE__);
        return 0;
    }
    adc_select_input(s->adc_num);
    s->lastresult = adc_read();
    return s->lastresult;
}

uint16_t my_adc_get(adc_sensor_Handle_t sensor){
    assert(sensor != NULL);
    adcsensor_t *s = (adcsensor_t *)sensor;
    if(s->handletype != adc_type){
        printf("Errore: Wrong handletype, should be a handle of a adc_sensor. file \"%s\", line %d\n",__FILE__,__LINE__);
        return 0;
    }
    return s->lastresult;
}

adc_sensor_Handle_t my_adc_deinit(adc_sensor_Handle_t sensor){
    assert(sensor != NULL);
    adcsensor_t *s = (adcsensor_t *)sensor;
    if(s->handletype != adc_type){
        printf("Errore: Wrong handletype, should be a handle of a adc_sensor. file \"%s\", line %d\n",__FILE__,__LINE__);
        return sensor;
    }
    gpio_deinit((s->adc_num + 26));
    s->handletype = null;
    free(sensor);
    return NULL;
}

#endif





