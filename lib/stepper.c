/*
 * Copyright (c) 2021 Antonio González
 * Copyright (c) 2023 Daniel Bucher
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
#if Use_Stepper_Motor_With_Repeating_Timer

#include "stepper.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "myhandletype.h"

typedef struct {
    my_handletype_t handletype;
    int32_t position;
    float speed;
    uint32_t gpio_mask;
    bool halfsteps;
    uint32_t stepping_sequence[8];
    stepper_direction_t direction;
    int32_t steps;
    uint16_t steps_per_revolution;
    #if Stepper_Accelerate_Repeating_Timer
    uint8_t count;
    float delta;
    struct repeating_timer timer2;
    #endif
    struct repeating_timer timer;
} stepper_t;



bool repeating_timer_callback_stepper(struct repeating_timer *t){
    stepper_Handle_t stepper = (stepper_Handle_t)(t->user_data);
    stepper_t *s = (stepper_t*)stepper;
    // printf("Repeating timer callback at: %lld with steps :%d \n", time_us_64(), s->steps);
    if (s->steps == 0) {
        cancel_repeating_timer(t); 
    }else{
        s->steps -= s->direction;
        stepper_step_once(stepper);
        if (s->steps == 0) {
            cancel_repeating_timer(t); 
        }
    }
}
#if Stepper_Accelerate_Repeating_Timer
bool repeating_timer_callback_stepper_a(struct repeating_timer *t){
    stepper_t *s = (stepper_t*)(t->user_data);
    if (s->count <= 0) {
        cancel_repeating_timer(t); 
    }else{
        s->speed = (s->speed + s->delta);
        stepper_set_speed_rpm(s, s->speed );
        s->count --;
        if (s->count <= 0) {
            cancel_repeating_timer(t); 
        }
    }
}
#endif

stepper_Handle_t stepper_init(uint8_t gpio_A1, uint8_t gpio_B1, uint8_t gpio_A2, uint8_t gpio_B2,
                              uint16_t steps_per_revolution,
                              stepper_mode_t stepping_mode)
{
    stepper_t *handle;
    handle = (stepper_t*)malloc(sizeof(stepper_t)); 

    if (handle == NULL) // if malloc failed, will return NULL pointer 
    { 
        return handle;
    }
    memset(handle, 0, sizeof(stepper_t));
    handle->handletype = stepper_type;
    
    handle->gpio_mask = (1 << gpio_A1) | (1 << gpio_A2) |
                   (1 << gpio_B1) | (1 << gpio_B2);
    gpio_init_mask(handle->gpio_mask);
    gpio_set_dir_out_masked(handle->gpio_mask);
    
    // Initialise stepping parameters. The stepping sequences are
    // bitmasks to activate the motor pins. These firing sequences can
    // be "single" or "power" (After Scherz and Monk 2013, Fig 14.8):
    // or the sequences can also be halfstepping
    // 
    // Single stepping      Power stepping         Halfstepping
    //      Coil                 Coil                  Coil
    // Step A1 B1 A2 B2     Step A1 B1 A2 B2     Step A1 B1 A2 B2  
    //   0   1  0  0  0       0   1  1  0  0       0   1  0  0  1  
    //   1   0  1  0  0       1   0  1  1  0       1   1  0  0  0              
    //   2   0  0  1  0       2   0  0  1  1       2   1  1  0  0 
    //   3   0  0  0  1       3   1  0  0  1       3   0  1  0  0      
    //                                             4   0  1  1  0                              
    //                                             5   0  0  1  0                      
    //                                             6   0  0  1  1                          
    //                                             7   0  0  0  1 
    
    if (stepping_mode == Single){
        handle->halfsteps = false;
        handle->stepping_sequence[0] = 1 << gpio_B2;
        handle->stepping_sequence[1] = 1 << gpio_A2;
        handle->stepping_sequence[2] = 1 << gpio_B1;
        handle->stepping_sequence[3] = 1 << gpio_A1;
    } else if (stepping_mode == Power) {
        handle->halfsteps = false;
        handle->stepping_sequence[0] = (1 << gpio_A1) | (1 << gpio_B2);
        handle->stepping_sequence[1] = (1 << gpio_A2) | (1 << gpio_B2);
        handle->stepping_sequence[2] = (1 << gpio_A2) | (1 << gpio_B1);
        handle->stepping_sequence[3] = (1 << gpio_A1) | (1 << gpio_B1);
    }   else if (stepping_mode == Half) {
        handle->halfsteps = true;
        handle->stepping_sequence[0] = (1 << gpio_B2);
        handle->stepping_sequence[1] = (1 << gpio_A2) | (1 << gpio_B2);
        handle->stepping_sequence[2] = (1 << gpio_A2);
        handle->stepping_sequence[3] = (1 << gpio_A2) | (1 << gpio_B1);
        handle->stepping_sequence[4] = (1 << gpio_B1);
        handle->stepping_sequence[5] = (1 << gpio_A1) | (1 << gpio_B1);
        handle->stepping_sequence[6] = (1 << gpio_A1);
        handle->stepping_sequence[7] = (1 << gpio_A1) | (1 << gpio_B2);
    }
    handle->steps_per_revolution = steps_per_revolution;
    // Initiallize motor at position 0.
    handle->position = 0;
    handle->steps = 0;
    handle->speed = 1;
    gpio_put_masked(handle->gpio_mask, handle->stepping_sequence[0]);
    return handle;
}

void stepper_set_speed_rpm(stepper_Handle_t stepper, float rpm){
    assert(stepper!=NULL);
    stepper_t *s = (stepper_t*)stepper;
    if(s->handletype != stepper_type){
        printf("Errore: Wrong handletype, should be a handle of a stepper. file \"%s\", line %d\n",__FILE__,__LINE__);
        return ;
    }
    if (rpm <= 0) return;
    s->speed = rpm;
    int64_t delay_us;
    if (s->halfsteps){
        delay_us = (6e7 / (float)(s->steps_per_revolution * 2) / rpm);
    }else{
        delay_us = (6e7 / (float)s->steps_per_revolution / rpm);
    }    
    if ((s->timer).alarm_id){
        (s->timer).delay_us = delay_us;
    }else if (s->steps > 0){
        if(!add_repeating_timer_us(-(delay_us), repeating_timer_callback_stepper, s, &(s->timer)))
            printf("Errore: failed to initialise a new timer. file \"%s\", line %d\n",__FILE__,__LINE__);
    }
}

void stepper_accelerate(stepper_Handle_t stepper,float endspeed, int time_ms){
    assert(stepper!=NULL);
    stepper_t *s = (stepper_t*)stepper;
    if(s->handletype != stepper_type){
        printf("Errore: Wrong handletype, should be a handle of a stepper. file \"%s\", line %d\n",__FILE__,__LINE__);
        return ;
    }
    float delta;
    delta = (endspeed - s->speed)/20;
    #if Stepper_Accelerate_Repeating_Timer
        s->delta = delta;
        s->count = 20;
        if (!s->timer2.alarm_id){
        if(!add_repeating_timer_ms(-(time_ms/20), repeating_timer_callback_stepper_a, s, &(s->timer2)))
            printf("Errore: failed to initialise a new timer. file \"%s\", line %d\n",__FILE__,__LINE__);
    }  
    #else
    for (int i = 1; i <= 20;i++){      
            stepper_set_speed_rpm(s, (actspeed + i * delta));
            sleep_ms(time_ms/20);
    }    
    #endif
}

void stepper_step_once(stepper_Handle_t stepper) {
    assert(stepper!=NULL);
    stepper_t *s = (stepper_t*)stepper;
    if(s->handletype != stepper_type){
        printf("Errore: Wrong handletype, should be a handle of a stepper. file \"%s\", line %d\n",__FILE__,__LINE__);
        return ;
    }
    s->position += s->direction;
    if (s->position >= s->steps_per_revolution) {
        s->position = 0;
    } else if (s->position < 0) {
        s->position = s->steps_per_revolution - 1;
    }
    if (s->halfsteps){
        gpio_put_masked(s->gpio_mask, s->stepping_sequence[s->position % 8]);
    }else{
        gpio_put_masked(s->gpio_mask, s->stepping_sequence[s->position % 4]);
    }
}


void stepper_release(stepper_Handle_t stepper) {
    assert(stepper!=NULL);
    stepper_t *s = (stepper_t*)stepper;
    if(s->handletype != stepper_type){
        printf("Errore: Wrong handletype, should be a handle of a stepper. file \"%s\", line %d\n",__FILE__,__LINE__);
        return ;
    }
    cancel_repeating_timer(&s->timer2);
    cancel_repeating_timer(&s->timer);
    s->steps = 0;
    gpio_put_masked(s->gpio_mask, 0);
}

void stepper_rotate_steps(stepper_Handle_t stepper, int32_t steps) {
    assert(stepper!=NULL);
    stepper_t *s = (stepper_t*)stepper;
    if(s->handletype != stepper_type){
        printf("Errore: Wrong handletype, should be a handle of a stepper. file \"%s\", line %d\n",__FILE__,__LINE__);
        return ;
    }
    s->steps += steps;
    if (s->steps > 0) {
        s->direction = forward;
    } else if (s->steps < 0){
        s->direction = backward;
    }else{
        return;
    }
    if (((s->timer).alarm_id) == 0){
        int64_t delay_us;
        if (s->speed == 0) return;
        if (s->halfsteps){
            delay_us = (6e7 / (float)(s->steps_per_revolution*2) / s->speed) / 2;
        }else{
            delay_us = (6e7 / (float)s->steps_per_revolution / s->speed);
        }    
        (s->timer).delay_us = delay_us;
        if(!add_repeating_timer_us(-(delay_us), repeating_timer_callback_stepper, s, &(s->timer)))
            printf("Errore: failed to initialise a new timer. file \"%s\", line %d\n",__FILE__,__LINE__);
    }    
}

void stepper_rotate_degrees(stepper_Handle_t stepper, float degrees){
    assert(stepper!=NULL);
    stepper_t *s = (stepper_t*)stepper;
    if(s->handletype != stepper_type){
        printf("Errore: Wrong handletype, should be a handle of a stepper. file \"%s\", line %d\n",__FILE__,__LINE__);
        return ;
    }
    int32_t steps;
    if (s->halfsteps){
        steps = (int32_t)(degrees / (360.0 / (s->steps_per_revolution * 2)));
    }else{
        steps = (int32_t)(degrees / (360.0 / s->steps_per_revolution));
    }
    stepper_rotate_steps(s, steps);
}

bool stepper_is_moving(stepper_Handle_t stepper){
    assert(stepper!=NULL);
    stepper_t *s = (stepper_t*)stepper;
    if(s->handletype != stepper_type){
        printf("Errore: Wrong handletype, should be a handle of a stepper. file \"%s\", line %d\n",__FILE__,__LINE__);
        return true;
    }
    if ((s->timer).alarm_id){
        return true;
    }else{
        return false;
    }
}

int32_t stepper_get_position(stepper_Handle_t stepper){
    assert(stepper!=NULL);
    stepper_t *s = (stepper_t*)stepper;
    if(s->handletype != stepper_type){
        printf("Errore: Wrong handletype, should be a handle of a stepper. file \"%s\", line %d\n",__FILE__,__LINE__);
        return 0;
    }
    return s->position;
}

void stepper_reset_position(stepper_Handle_t stepper){
    assert(stepper!=NULL);
    stepper_t *s = (stepper_t*)stepper;
    if(s->handletype != stepper_type){
        printf("Errore: Wrong handletype, should be a handle of a stepper. file \"%s\", line %d\n",__FILE__,__LINE__);
        return ;
    }
    s->position = 0;
}


int16_t stepper_get_position_degrees(stepper_Handle_t stepper){
    assert(stepper!=NULL);
    stepper_t *s = (stepper_t*)stepper;
    if(s->handletype != stepper_type){
        printf("Errore: Wrong handletype, should be a handle of a stepper. file \"%s\", line %d\n",__FILE__,__LINE__);
        return 0;
    }
    return (int16_t)(360 * s->position / (int)s->steps_per_revolution);
}

stepper_Handle_t stepper_deinit(stepper_Handle_t stepper){
    stepper_release(stepper);
    assert(stepper!=NULL);
    stepper_t *s = (stepper_t*)stepper;
    if(s->handletype != stepper_type){
        printf("Errore: Wrong handletype, should be a handle of a stepper. file \"%s\", line %d\n",__FILE__,__LINE__);
        return stepper;
    }
    for(uint i=0;i<NUM_BANK0_GPIOS;i++) {
        if (s->gpio_mask & 1) {
            gpio_deinit(i);
        }
        s->gpio_mask >>= 1;
    }
    s->handletype = null;
    free(stepper); 
    return NULL;
}

#endif