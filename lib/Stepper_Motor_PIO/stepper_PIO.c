
/*
 * Copyright (c) 2023 Daniel Bucher
 *
 * SPDX-License-Identifier: BSD-3-Clause
 * 
 */

#if Use_Stepper_Motor_With_PIO

#include "stepper_PIO.h"
#include "pio_stepper.h"
#include <stdlib.h>
#include <string.h>
#include "myhandletype.h"

static bool is_moving;
static bool is_initialised = false; //is used to set the maximal initalised pio_Steppers to one

typedef struct {
    my_handletype_t handletype;
    bool moving;                            
    double speed;                                                          
    pio_stepper_direction_t direction;                      
    uint16_t steps_per_revolution;                      
    pio_stepper_mode_t mode;                    
    #if Stepper_Accelerate_Repeating_Timer                  
    uint8_t count;                                  
    float delta;                                        
    struct repeating_timer timer2;                          
    #endif      
} pio_stepper_t;

// Position control interrupts
void pio0_interrupt_handler() {
    pio_interrupt_clear(pio_0, 0) ;
    is_moving = false; 
}

#if Stepper_Accelerate_Repeating_Timer
bool repeating_timer_callback_pio_stepper_a(struct repeating_timer *t){
    pio_stepper_t *s = (pio_stepper_t*)(t->user_data);
    if (s->count <= 0) {
        cancel_repeating_timer(t); 
    }else{
        s->speed = (s->speed + s->delta);
        pio_stepper_set_speed_rpm(s, s->speed);
        s->count --;
        if (s->count <= 0) {
            cancel_repeating_timer(t); 
        }
    }
}
#endif

pio_stepper_Handle_t pio_stepper_init(uint8_t gpio_A1, uint8_t gpio_B1, uint8_t gpio_A2, uint8_t gpio_B2,
                  uint16_t steps_per_revolution,
                  pio_stepper_mode_t stepping_mode)
{
    if (is_initialised){
        printf("Errore: Ther is already one pio-stepper initialised. Because there are only a few PIO state machines, only one PIO-controlled stepper is possible. file \"%s\", line %d\n",__FILE__,__LINE__);
        return NULL;        
    }
    if (gpio_B1 != gpio_A1 +1){
        printf("Errore: The B1 pin has to be next to the A1 pin. file \"%s\", line %d\n",__FILE__,__LINE__);
        return NULL;        
    }
    if (gpio_A2 != gpio_A1 +2){
        printf("Errore: The A2 pin has to be next to the B1 pin. file \"%s\", line %d\n",__FILE__,__LINE__);
        return NULL;        
    }
    if (gpio_B2 != gpio_A1 +3){
        printf("Errore: The B2 Pin has to be next to the A2 pin. file \"%s\", line %d\n",__FILE__,__LINE__);
        return NULL;        
    }

    pio_stepper_t *handle;
    handle = (pio_stepper_t*)malloc(sizeof(pio_stepper_t)); 
    if (handle == NULL) // if malloc failed, will return NULL pointer 
    { 
        return handle;
    }
    memset(handle, 0, sizeof(pio_stepper_t));
    handle->handletype = pio_stepper_type;
    setupMotor1(gpio_A1, pio0_interrupt_handler);
    handle->steps_per_revolution = steps_per_revolution;
    handle->moving = false;
    handle->mode = stepping_mode;
    handle->speed = 0;

    is_initialised = true;
    return handle;
}

void pio_stepper_set_speed_rpm(pio_stepper_Handle_t stepper, float rpm){
    assert(stepper!=NULL);
    pio_stepper_t *s = (pio_stepper_t*)stepper;
    if(s->handletype != pio_stepper_type){
        printf("Errore: Wrong handletype, should be a handle of a pio-stepper. file \"%s\", line %d\n",__FILE__,__LINE__);
        return;
    }
    if (rpm <= 0) return;
    s->speed = rpm;
    if (s->mode == half){
        SET_SPEED_MOTOR_1((6e7 / (float)(s->steps_per_revolution*2) / rpm) * 125);
    }else{
        SET_SPEED_MOTOR_1((6e7 / (float)s->steps_per_revolution / rpm) * 125);
    }

}

void pio_stepper_set_mode(pio_stepper_Handle_t stepper, pio_stepper_mode_t stepping_mode) {
    assert(stepper!=NULL);
    pio_stepper_t *s = (pio_stepper_t*)stepper;
    if(s->handletype != pio_stepper_type){
        printf("Errore: Wrong handletype, should be a handle of a pio-stepper. file \"%s\", line %d\n",__FILE__,__LINE__);
        return;
    }
    pio_stepper_mode_t modeold = s->mode;
    if (modeold == stepping_mode){
        return;
    }else{
        s->mode = stepping_mode;
        pio_stepper_set_speed_rpm(s,s->speed);
        if (s->mode == half){
            SET_DIRECTION_MOTOR_h(s->direction);
        }else if (s->mode == singel){
            SET_DIRECTION_MOTOR_s(s->direction);
        }
        else if (s->mode == power){
            SET_DIRECTION_MOTOR_p(s->direction);
        }
    }
}

void pio_stepper_set_direction(pio_stepper_Handle_t stepper, pio_stepper_direction_t direction) {
    assert(stepper!=NULL);
    pio_stepper_t *s = (pio_stepper_t*)stepper;
    if(s->handletype != pio_stepper_type){
        printf("Errore: Wrong handletype, should be a handle of a pio-stepper. file \"%s\", line %d\n",__FILE__,__LINE__);
        return;
    }
    pio_stepper_direction_t directionold = s->direction;
    if (directionold == direction){
        return;
    }else{
        s->direction = direction;
        if (s->mode == half){
            SET_DIRECTION_MOTOR_h(s->direction);
        }else if (s->mode == singel){
            SET_DIRECTION_MOTOR_s(s->direction);
        }
        else if (s->mode == power){
            SET_DIRECTION_MOTOR_p(s->direction);
        }
    }
}

void pio_stepper_accelerate(pio_stepper_Handle_t stepper,float endspeed, int time_ms){
    assert(stepper!=NULL);
    pio_stepper_t *s = (pio_stepper_t*)stepper;
    if(s->handletype != pio_stepper_type){
        printf("Errore: Wrong handletype, should be a handle of a pio-stepper. file \"%s\", line %d\n",__FILE__,__LINE__);
        return;
    }
    float delta;
    delta = (endspeed - s->speed)/20;    

    #if Stepper_Accelerate_Repeating_Timer
        s->delta = delta;
        s->count = 20;
        if (!s->timer2.alarm_id){
        if(!add_repeating_timer_ms(-(time_ms/20), repeating_timer_callback_pio_stepper_a, s, &(s->timer2)))
            printf("Errore: failed to initialise a new timer. file \"%s\", line %d\n",__FILE__,__LINE__);
    }  
    #else
    for (int i = 1; i <= 20;i++){      
            pio_stepper_set_speed_rpm(s, (actspeed + i * delta));
            sleep_ms(time_ms/20);
    }    
    #endif
}

bool pio_stepper_step_once(pio_stepper_Handle_t stepper) {
    assert(stepper!=NULL);
    pio_stepper_t *s = (pio_stepper_t*)stepper;
    if(s->handletype != pio_stepper_type){
        printf("Errore: Wrong handletype, should be a handle of a pio-stepper. file \"%s\", line %d\n",__FILE__,__LINE__);
        return false;
    }
    if (s->moving) return false;
    is_moving = true;
    MOVE_STEPS_MOTOR_1(1);
    return true;      
}

void pio_stepper_release(pio_stepper_Handle_t stepper) {
    assert(stepper!=NULL);
    pio_stepper_t *s = (pio_stepper_t*)stepper;
    if(s->handletype != pio_stepper_type){
        printf("Errore: Wrong handletype, should be a handle of a pio-stepper. file \"%s\", line %d\n",__FILE__,__LINE__);
        return;
    }
    cancel_repeating_timer(&s->timer2);
    SET_DIRECTION_MOTOR_p(stop);
}

bool pio_stepper_rotate_steps(pio_stepper_Handle_t stepper, int32_t steps) {
    assert(stepper!=NULL);
    pio_stepper_t *s = (pio_stepper_t*)stepper;
    if(s->handletype != pio_stepper_type){
        printf("Errore: Wrong handletype, should be a handle of a pio-stepper. file \"%s\", line %d\n",__FILE__,__LINE__);
        return false;
    }
    if(s->moving) return false;
    if (steps > 0) {
        s->direction = clockwise;
    } else if (steps < 0){
        s->direction = counterclockwise;
        steps = steps * (-1);
    }else{
        return false;
    }
    if (s->mode == half){
            SET_DIRECTION_MOTOR_h(s->direction);
    }else if (s->mode == singel){
        SET_DIRECTION_MOTOR_s(s->direction);
    }
    else if (s->mode == power){
        SET_DIRECTION_MOTOR_p(s->direction);
    }
    is_moving = true;
    MOVE_STEPS_MOTOR_1(steps);
    return true;      
}

void pio_stepper_rotate_degrees(pio_stepper_Handle_t stepper, float degrees){
    assert(stepper!=NULL);
    pio_stepper_t *s = (pio_stepper_t*)stepper;
    if(s->handletype != pio_stepper_type){
        printf("Errore: Wrong handletype, should be a handle of a pio-stepper. file \"%s\", line %d\n",__FILE__,__LINE__);
        return;
    }
    int32_t steps;
    if (s->mode == half){
        steps = (int32_t)(degrees / (360.0 / (s->steps_per_revolution*2)));
    }else{
        steps = (int32_t)(degrees / (360.0 / (s->steps_per_revolution)));
    }
    pio_stepper_rotate_steps(s, steps);
}

bool pio_stepper_is_moving(pio_stepper_Handle_t stepper){
    assert(stepper!=NULL);
    pio_stepper_t *s = (pio_stepper_t*)stepper;
    if(s->handletype != pio_stepper_type){
        printf("Errore: Wrong handletype, should be a handle of a pio-stepper. file \"%s\", line %d\n",__FILE__,__LINE__);
        return true;
    }
    s->moving = is_moving;
    if (s->moving){
        return true;
    }else{
        return false;
    }
}


#endif
