/*
 * Copyright (c) 2023 Daniel Bucher
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
#if Use_Button

#include "button.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "myhandletype.h"

typedef struct {
    my_handletype_t handletype;
    uint8_t gpio;
    uint32_t event;
    bool debounce;
    bool status;
} button_t ;

static int buttoncounter = 0;
static button_t buttonlist[buttonlist_storagesize];

int64_t alarm_callback_button(alarm_id_t id, void *sw);

int64_t alarm_callback_button(alarm_id_t id, void *sw) {
    button_t mybutton = *(button_t*)sw;
    gpio_set_irq_enabled(mybutton.gpio, mybutton.event, true); 
    return 0;
}

void button_irq_handler(void) {   
    /*if (gpio_get_irq_event_mask(buttonlist[1].gpio ) &((buttonlist[1].event)) {
         gpio_acknowledge_irq(buttonlist[1].gpio, buttonlist[1].event);
        // handle the IRQ
        if(buttonlist[1].debounce){
            gpio_set_irq_enabled(buttonlist[1].gpio, buttonlist[1].event, false);
            add_alarm_in_ms(debounce_delay_time, alarm_callback_button, &(buttonlist[1]), false);
        }
    }
    //add for every button to use*/

    /*wenn every handler makes the same thing*/
    for(int i = 0;i < buttoncounter; i++){
        if (gpio_get_irq_event_mask(buttonlist[i].gpio ) &(GPIO_IRQ_EDGE_RISE)) {
        gpio_acknowledge_irq(buttonlist[i].gpio, GPIO_IRQ_EDGE_RISE);
        buttonlist[i].status = true;
        // printf("button %d,rising edge\n", i);
        if(buttonlist[i].debounce){
            gpio_set_irq_enabled(buttonlist[i].gpio, buttonlist[i].event, false);
            add_alarm_in_ms(debounce_delay_time, alarm_callback_button, &(buttonlist[i]), false);
        }
    }else if (gpio_get_irq_event_mask(buttonlist[i].gpio ) &(GPIO_IRQ_EDGE_FALL)) {
        gpio_acknowledge_irq(buttonlist[i].gpio, GPIO_IRQ_EDGE_FALL);
        buttonlist[i].status = false;
        // printf("button %d,falling edge\n", i);
        if(buttonlist[i].debounce){
            gpio_set_irq_enabled(buttonlist[i].gpio, buttonlist[i].event, false);
            add_alarm_in_ms(debounce_delay_time, alarm_callback_button, &(buttonlist[i]), false);
        }
    }
    }
}

button_Handle_t button_init(uint8_t gpio, uint32_t event, pull_t pull, bool debounce){
    if(buttoncounter == buttonlist_storagesize) return NULL;
    button_t *handle = &(buttonlist[buttoncounter]);
    buttonlist[buttoncounter].gpio = gpio;
    buttonlist[buttoncounter].event = (event | GPIO_IRQ_EDGE_FALL | GPIO_IRQ_EDGE_RISE);
    buttonlist[buttoncounter].debounce = debounce;
    
    handle->handletype = button_type;
    gpio_init(gpio);
    gpio_set_dir(gpio, GPIO_IN);
    if (pull == pull_up){
        gpio_pull_up(gpio);
    } 
    else if (pull == pull_down){
        gpio_pull_down(gpio);
    }
    sleep_ms(10);
    buttonlist[buttoncounter].status = gpio_get(gpio);
    buttoncounter ++;
    gpio_set_irq_enabled(gpio, handle->event, true);   
    gpio_add_raw_irq_handler(gpio, button_irq_handler);
    irq_set_enabled(IO_IRQ_BANK0, true);
    return handle; 
}

bool button_get(button_Handle_t button){
    assert(button != NULL);
    button_t *b = (button_t *)button;
    if(b->handletype != button_type){
        printf("Errore: Wrong handletype, should be a handle of a button. file \"%s\", line %d\n",__FILE__,__LINE__);
        return false;
    }
    return b->status;
}

button_Handle_t button_deinit(button_Handle_t button){
    assert(button != NULL);
    button_t *b = (button_t *)button;
    if(b->handletype != button_type){
        printf("Errore: Wrong handletype, should be a handle of a button. file \"%s\", line %d\n",__FILE__,__LINE__);
        return button;
    }
    gpio_set_irq_enabled(b->gpio,b->event, false);
    gpio_deinit(b->gpio);
    b->handletype = null;
    return NULL;
}

#endif

