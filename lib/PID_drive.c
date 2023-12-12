/*
 * Copyright (c) 2023 Daniel Bucher
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
#if Use_PID_Driver

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "includemyPicolib_config.h"
#include "dc_motor.h"
#include "PID_drive.h"
#include "quadrature_encoder.h"
#include "pico/critical_section.h"
#include "myhandletype.h"

typedef struct
{
    my_handletype_t handletype;
    float targetSpeed;      
    
    //PID-parameters
    float Kp, Ki, Kd1, Kd2;     

    //old values
    float i,d;        //last i-part and d-part
    int16_t eOld;       //controll-error 

    //Valus to get the speed
    int32_t oldcount;
    uint64_t oldtime;

    //Handle for the motor and encoder
    dcmotor_Handle_t dcmotor;
    quadrature_encoder_Handle_t encoder;

    //Timer and critical section
    struct repeating_timer timer;
    critical_section_t crit_sec;
} pid_drive_t;

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

/// @brief intern function for the PID-Drive
/// @param pid_drive 
void static PID_doWork(pid_drive_Handle_t pid_drive);

bool repeating_timer_callback_pid(struct repeating_timer *t)
{
    pid_drive_Handle_t handle = (pid_drive_Handle_t)(t->user_data);
    PID_doWork(handle);
    return true;
}

void static PID_doWork(pid_drive_Handle_t pid_drive)
{   
    assert(pid_drive != NULL);
    pid_drive_t *d = (pid_drive_t *)pid_drive;
    if(d->handletype != piddrive_type){
        printf("Errore: Wrong handletype, should be a handle of a pid_drive. file \"%s\", line %d\n",__FILE__,__LINE__);
        return;
    }
    assert(d->encoder != NULL);
    quadrature_encoder_t *q = (quadrature_encoder_t *)d->encoder;
    if(q->handletype != quadraturencoder_type){
        printf("Errore: Wrong handletype, should be a handle of a initalized quadratur encoder. file \"%s\", line %d\n",__FILE__,__LINE__);
        return;
    }
    assert(d->dcmotor != NULL);
    dcmotor_t *m = (dcmotor_t *)d->dcmotor;
    if(m->handletype != dcmotor_type){
        printf("Errore: Wrong handletype, should be a handle of a initalized dc-motor. file \"%s\", line %d\n",__FILE__,__LINE__);
        return;
    }
    int32_t count;
    uint64_t time;
    critical_section_enter_blocking(&d->crit_sec);
    count = Quadratur_Encoder_get_count(d->encoder);
    time = time_us_64(); 
    critical_section_exit(&d->crit_sec);

    float delta_count = (count-(d->oldcount));
    float delta_time = ((float)us_to_ms(time-(d->oldtime))/(float)1000);
    float speed = (delta_count / delta_time / (float)(q->puls_per_rotation)) * 60;
    d->oldtime = time;
    d->oldcount = count;

    if ((d->targetSpeed < 15)&&(d->targetSpeed > -15)){
        d->targetSpeed = 0;
    }
    if((d->targetSpeed == 0)&&(speed <= 0.5)){
        dcmotor_set_value(d->dcmotor, 0);
        return;
    }
    int32_t value;
    int16_t eNow;
    eNow = (int16_t)(d->targetSpeed - speed);
    value = (int32_t)(d->Kp * eNow);                    // P-Part
    //printf("Ki = %f\n", d->Ki);
    if (d->Ki != 0){                                    // I-Part
        d->i += d->Ki*(float)(eNow + d->eOld); 
        value += (int)d->i;
        //printf("i = %f\n", d->i);
    }
    if (d->Kd1 != 0){                                   // D-Part on the controlled speed
        d->d = d->d * d->Kd1 + d->Kd2 * (eNow - d->eOld);
        value += (int)d->d;
    }
    d->eOld = eNow;                                     // update old controller error 
       
    if (value > MOTOR_MAX_VALUE)
    {
        value = MOTOR_MAX_VALUE;
        d->i -= d->Ki*(float)(eNow + d->eOld);                 // anti wind-up fo I-Part when saturation occurs 
    }
    else if (value < -MOTOR_MAX_VALUE)
    {
        value = -MOTOR_MAX_VALUE;
        d->i -= d->Ki*(float)(eNow + d->eOld);                 // anti wind-up fo I-Part when saturation occurs
    }
    //printf("soll_speed = %f, ist_speed = %f, e = %d, i = %f, d = %f, u = %d\n",d->targetSpeed, speed, eNow, d->i, d->d, value); //this line can be used to test the PID-Parameters
    dcmotor_set_value(d->dcmotor, value);
}

pid_drive_Handle_t PID_drive_Init(dcmotor_Handle_t dcmotor, quadrature_encoder_Handle_t encoder)
{
    if ((dcmotor == NULL)||(encoder== NULL)){
        return NULL;
    }
    pid_drive_t *handle;
    handle = (pid_drive_t *)malloc(sizeof(pid_drive_t));

    if (handle == NULL)
    { // if malloc failed, will return NULL pointer 
        return handle;
    }
    memset(handle, 0, sizeof(pid_drive_t)); 
    handle->handletype = piddrive_type;
    dcmotor_t *m = (dcmotor_t *)dcmotor;
    if(m->handletype != dcmotor_type){
        printf("Errore: Wrong handletype, should be a handle of a initalized dc-motor. file \"%s\", line %d\n",__FILE__,__LINE__);
        return NULL;
    }
    quadrature_encoder_t *q = (quadrature_encoder_t *)encoder;
    if(q->handletype != quadraturencoder_type){
        printf("Errore: Wrong handletype, should be a handle of a initalized quadratur encoder. file \"%s\", line %d\n",__FILE__,__LINE__);
        return NULL;
    }
    handle->dcmotor = dcmotor;
    handle->encoder = encoder;

    handle->Kp = 0;
    handle->Ki = 0;
    handle->Kd1 = 0;
    handle->i = 0;
    handle->targetSpeed = 0;
    critical_section_init(&handle->crit_sec);

    critical_section_enter_blocking(&handle->crit_sec);
    handle->oldcount = Quadratur_Encoder_get_count(handle->encoder);
    handle->oldtime = time_us_64(); 
    critical_section_exit(&handle->crit_sec);

    if (!add_repeating_timer_ms(-PID_PERIOD, repeating_timer_callback_pid, handle, &handle->timer))
        printf("Errore: failed to initialise a new timer. file \"%s\", line %d\n",__FILE__,__LINE__);
    sleep_ms(5);
    return handle;
}

void PID_drive_SetParameters(pid_drive_Handle_t pid_drive,float Kp, float Ki, float Kd, float N)
{
    assert(pid_drive != NULL);
    pid_drive_t *d = (pid_drive_t *)pid_drive;
    if(d->handletype != piddrive_type){
        printf("Errore: Wrong handletype, should be a handle of a pid_drive. file \"%s\", line %d\n",__FILE__,__LINE__);
        return ;
    }
    d->Kp = Kp;
    d->Ki = Ki *0.5f * ((float)PID_PERIOD/(float)1000);
    d->Kd1 = Kd / (Kd + Kp* ((float)PID_PERIOD/(float)1000) * N);
    d->Kd2 = (N * Kd) / ((Kd/Kp) + ((float)PID_PERIOD/(float)1000) * N);
}

void PID_drive_SetParameters_ideal(pid_drive_Handle_t pid_drive,float Kp, float Ti, float Td, float N)
{
    assert(pid_drive != NULL);
    pid_drive_t *d = (pid_drive_t *)pid_drive;
    if(d->handletype != piddrive_type){
        printf("Errore: Wrong handletype, should be a handle of a pid_drive. file \"%s\", line %d\n",__FILE__,__LINE__);
        return ;
    }
    d->Kp = Kp;
    if (Ti == 0){
        d->Ki = 0;
    }else {
        d->Ki = (Kp / Ti) *0.5f * ((float)PID_PERIOD/(float)1000);
    }
    d->Kd1 = Td / (Td + ((float)PID_PERIOD/(float)1000) * N);
    d->Kd2 = (Td *N * Kp) / (Td + ((float)PID_PERIOD/(float)1000) * N);
}

void PID_drive_SetSpeed(pid_drive_Handle_t pid_drive,float speed)
{
    assert(pid_drive != NULL);
    pid_drive_t *d = (pid_drive_t *)pid_drive;
    if(d->handletype != piddrive_type){
        printf("Errore: Wrong handletype, should be a handle of a pid_drive. file \"%s\", line %d\n",__FILE__,__LINE__);
        return ;
    }
    d->targetSpeed = speed;
}

pid_drive_Handle_t PID_drive_Deinit(pid_drive_Handle_t pid_drive)
{
    assert(pid_drive != NULL);
    pid_drive_t *d = (pid_drive_t *)pid_drive;
    if(d->handletype != piddrive_type){
        printf("Errore: Wrong handletype, should be a handle of a pid_drive. file \"%s\", line %d\n",__FILE__,__LINE__);
        return pid_drive;
    }
    cancel_repeating_timer(&d->timer);
    critical_section_deinit(&d->crit_sec);
    d->handletype = null;
    free(pid_drive);
    return NULL;
}

#endif