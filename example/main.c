/*
 * Copyright (c) 2023 Daniel Bucher
 *
 * SPDX-License-Identifier: BSD-3-Clause
 * 
 */

#include <stdio.h>
#include "pico/stdlib.h"

// #include "uart.h"

#include "platform.h"
#include "includemyPicolib.h"

typedef enum
{
    Initstate,
    Autostate,
    Manualstate_1,
    Manualstate_2,
} states_t;

typedef enum
{
    DC_2,
    DC_1,    
    Stepper_1,
    PIO_Stepper_2,
} motor_type_t;

typedef struct {
    dcmotor_direction_t direction;
    float speed;
} motor_t;

void main()
{
    stdio_init_all();
    /*******************************************************************************/
    // wait 5 seconds, enough time to connect via USB
    int i = 5;
    while (true)
    {
        sleep_ms(1000);
        printf("%d", i);
        if (i <= 0)
        {
            break;
        }
        i--;
    }

    #if Use_Quadrature_Encoder
    extern quadrature_encoder_Handle_t quadraturencoder_motor1;
    extern quadrature_encoder_Handle_t quadraturencoder_motor2;
    #endif
    #if Use_DC_Motor
    extern dcmotor_Handle_t dcmotor1;
    extern const dcmotor_mode_t dcmotor1_mode;

    extern dcmotor_Handle_t dcmotor2;
    extern const dcmotor_mode_t dcmotor2_mode;
    #endif
    #if Use_PID_Driver
    extern pid_drive_Handle_t pid_drive1;
    extern pid_drive_Handle_t pid_drive2;
    #endif
    #if Use_Stepper_Motor_With_Repeating_Timer
    extern stepper_Handle_t stepper1;
    #endif
    #if Use_Stepper_Motor_With_PIO
    extern pio_stepper_Handle_t stepper2;
    #endif
    #if Use_Button
    extern button_Handle_t button1;
    extern button_Handle_t button2;
    extern button_Handle_t button3;
    extern button_Handle_t switch1;
    #endif
    #if Use_Adc
    extern adc_sensor_Handle_t poti1;
    extern adc_sensor_Handle_t poti2;
    #endif
    
    //for the State maschine
    states_t oldstate = Manualstate_1;
    states_t state, nextinit_state, oldnextinit_state, nextstate = Initstate;

    //for the Manual states
    uint16_t value_poti1, value_old1, value_poti2,value_old2;
    float delta;
    stepper_direction_t direction = forward;
    uint32_t counter = 0;
    int16_t angel;
    float sollspeed = 0;
    uint16_t position = 0;
    motor_t motors[4];
    motor_type_t motor;
    bool lock;

    My_Platform_Init();
    printf("################################################################################\n");
    printf("System initalisiert!\n");
    printf("Um zu Starten den Schalter von links nach rechts bewegen!\n");
    while(gpio_get(Switch)){
        tight_loop_contents();
    }
    while(!gpio_get(Switch)){
        tight_loop_contents();
    }
    while ((true))
    {
        switch(state){
            case Initstate:{
                if(oldstate != state){
                    printf("################################################################################\n");
                    printf("Auswahl Modus aktiv\n");
                    printf("Mit der Taste 1 kann der AUTO Modus gewählt werden\n");
                    printf("Mit der Taste 2 kann der Manuel 1 Modus gewählt werden\n");
                    printf("Mit der Taste 3 kann der Manuel 2 Modus gewählt werden\n");
                    printf("Um die Auswahl zu bestätigen und das System zu starten muss der Schalter nach links gelegt werden\n");
                    printf("Wird der Schalter wider zurückgelegt, werden die Motoren gestoppt und der Modus kann wieder neu gewählt werden.\n");

                    stepper_release(stepper1);
                    pio_stepper_release(stepper2);
                    pio_stepper_set_mode(stepper2,power);
                    stepper_reset_position(stepper1);
                    PID_drive_SetSpeed(pid_drive1, 0);
                    PID_drive_SetSpeed(pid_drive2, 0);
                    sleep_ms(200);
                    dcmotor_stop(dcmotor1);
                    dcmotor_stop(dcmotor2);
                } 
                if (!button_get(button1)){
                    nextinit_state = Autostate;
                }else if(!button_get(button2)){
                    nextinit_state = Manualstate_1;
                }else if(!button_get(button3)){
                    nextinit_state = Manualstate_2;
                }
                if (oldnextinit_state != nextinit_state){
                    switch (nextinit_state){
                        case Autostate:{
                            printf("Auto Modus angewählt\n");
                            break;
                        }
                        case Manualstate_1:{
                            printf("Manual 1  Modus angewählt\n");
                            break;
                        }
                        case Manualstate_2:{
                            printf("Manuel 2  Modus angewählt\n");
                            break;
                        }
                    }
                }
                oldnextinit_state = nextinit_state;
                    
                if(!gpio_get(Switch)){
                    nextstate = nextinit_state;
                }
                break;
            }
            case Autostate:{
                if(oldstate != state){
                    printf("################################################################################\n");
                    printf("Modus Auto aktiv\n");
                    printf("Das Vorzeigemodell zeigt die verschiedenen Bewegungen und Geschwindikeiten\n");
                    counter = 0;
                    stepper_release(stepper1);
                    pio_stepper_release(stepper2);
                    pio_stepper_set_mode(stepper2,power);
                    stepper_reset_position(stepper1);
                    PID_drive_SetSpeed(pid_drive1, 0);
                    PID_drive_SetSpeed(pid_drive2, 0);
                    sleep_ms(200);
                    dcmotor_stop(dcmotor1);
                    dcmotor_stop(dcmotor2);
                }                 
                if (!stepper_is_moving(stepper1)){
                    stepper_release(stepper1);
                    stepper_set_speed_rpm(stepper1, 0.1);
                    if(direction == forward){
                        direction = backward;
                        stepper_rotate_degrees(stepper1, 360);
                    }else if(direction == backward){
                        direction = forward;
                        stepper_rotate_degrees(stepper1, -360);
                    }
                    stepper_accelerate(stepper1 ,20,2000);
                }
                position = stepper_get_position_degrees(stepper1);
                if((stepper_is_moving(stepper1)) && (direction == forward) && (position<30) && (position >20)){
                    stepper_accelerate(stepper1,1,400);
                }else if((stepper_is_moving(stepper1)) && (direction == backward) && (position>330) && (position < 340)){
                    stepper_accelerate(stepper1,1,400);
                }
                if (counter == 0)
                {
                    PID_drive_SetSpeed(pid_drive1, 100);
                    PID_drive_SetSpeed(pid_drive2, 100);
                    pio_stepper_set_speed_rpm(stepper2, 0.5);
                    pio_stepper_rotate_degrees(stepper2, 5);
                }
                else if (counter == 500)
                {
                    PID_drive_SetSpeed(pid_drive1, 200);
                    PID_drive_SetSpeed(pid_drive2, -100);
                    pio_stepper_accelerate(stepper2, 1, 1);
                    pio_stepper_rotate_degrees(stepper2, 175);
                }
                else if (counter == 1000)
                {
                    PID_drive_SetSpeed(pid_drive1, 220);
                    PID_drive_SetSpeed(pid_drive2, -200);
                    pio_stepper_set_mode(stepper2, singel);
                }
                else if (counter == 1500)
                {
                    PID_drive_SetSpeed(pid_drive1, 0);
                    PID_drive_SetSpeed(pid_drive2, 0);
                    pio_stepper_set_mode(stepper2, half);
                }
                if (counter == 3000){
                    counter = 0;
                }else{
                    counter ++;
                }
                sleep_ms(10);
                
                if(gpio_get(Switch)){
                    nextstate = Initstate;
                }
                break;
            }
            case Manualstate_1:{
                if(oldstate != state){
                    printf("################################################################################\n");
                    printf("Modus Manual 1 aktiv\n");
                    printf("Mit der Taste 1 und dem Poti 1 kann die Position für den Stepper 1 vorgegeben werden.\n");
                    printf("Mit der Taste 2 und dem Poti 2 kann die Drezahl für den DC-Motor 1 vorgegeben werden.\n");
                    printf("Mit der Taste 3 können die Motoren gestoppt werden und die aktuelle Position des Steppers 1\n als neue Null Position gewählt werden.\n");
                    stepper_release(stepper1);
                    pio_stepper_release(stepper2);
                    pio_stepper_set_mode(stepper2,power);
                    stepper_reset_position(stepper1);
                    PID_drive_SetSpeed(pid_drive1, 0);
                    PID_drive_SetSpeed(pid_drive2, 0);
                    sleep_ms(200);
                    dcmotor_stop(dcmotor1);
                    dcmotor_stop(dcmotor2);
                    stepper_set_speed_rpm(stepper1, 15);
                } 
                if(gpio_get(Switch)){
                    nextstate = Initstate;
                }                
                if (!button_get(button1)){
                    value_poti1 = my_adc_sampelandread(poti1);
                    delta = (value_old1 - value_poti1);
                    value_old1 = value_poti1;
                    if ((delta > 50) || (delta < -50)){
                        stepper_release(stepper1);
                        angel = 315-(value_poti1 / 13);
                        stepper_rotate_degrees(stepper1, (float)(angel-stepper_get_position_degrees(stepper1))); 
                        printf("Stepper fährt auf die Position %i Grad.\n", angel);                                        
                    }
                }
                if (!button_get(button2)){
                    value_poti2 = my_adc_sampelandread(poti2);
                    delta = (value_old2 - value_poti2);
                    value_old2 = value_poti2;
                    if ((delta > 50) || (delta < -50)){
                        sollspeed = (float)270-(float)value_poti2 / (float)8;
                        PID_drive_SetSpeed(pid_drive1, sollspeed);
                        printf("DC-Motor 1 auf %f rpm geregelt.\n", sollspeed);
                    }
                }
                if (!button_get(button3)){
                    stepper_reset_position(stepper1);
                    stepper_release(stepper1);
                    PID_drive_SetSpeed(pid_drive1, 0);
                    value_old1 = 0;
                    value_old2 = 0;
                    printf("Alle Motoren gestoppt, Aktuelle Stepper 1 Positon ist der neue Nullpunkt.\n");
                }
                break;
            }
            case Manualstate_2:{
                if(oldstate != state){
                    printf("################################################################################\n");
                    printf("Modus Manual 2 aktiv\n");
                    printf("Mit der Taste 3 kann ein Motor angewählt werden\n");
                    printf("Mit der Taste 1 kann der angewählte Motor Schrittweise im Uhrzeigersinn beschleunigt werden\n");
                    printf("Mit der Taste 2 kann der angewählte Motor Schrittweise im Gegenuhrzeigersinn beschleunigt werden\n");
                    counter = 0;
                    stepper_release(stepper1);
                    pio_stepper_release(stepper2);
                    stepper_reset_position(stepper1);
                    pio_stepper_set_mode(stepper2,power);
                    PID_drive_SetSpeed(pid_drive1, 0);
                    PID_drive_SetSpeed(pid_drive2, 0);
                    sleep_ms(200);
                    dcmotor_stop(dcmotor1);
                    dcmotor_stop(dcmotor2);
                    for (motor = 0;i<=3;i++){
                        motors[i].direction = Stopp;
                        motors[i].speed = 0;
                    }
                    motor = 0;
                } 
                if(gpio_get(Switch)){
                    nextstate = Initstate;
                }

                if (!button_get(button3)){
                    if (lock == false){
                        if (motor == 3){
                            motor = 0;
                        }else{
                            motor ++;
                        }
                        printf("Angewählter motor:%i, aktuelle Drezahl :%frpm\n",motor, motors[motor].speed);
                        lock = true;
                    }
                }
                if (button_get(button3)){
                    lock = false;
                }

                if (!button_get(button1)){
                    switch (motor){
                        case DC_1:{
                            if (motors[motor].speed == 200){
                                printf("Maximale Geschwindikeit ereicht!\n");
                                break;
                            } 
                            motors[motor].speed += 20;
                            PID_drive_SetSpeed(pid_drive1, motors[motor].speed);
                            break;
                        }case DC_2:{
                            if (motors[motor].speed == 200){
                                printf("Maximale Geschwindikeit ereicht!\n");
                                break;
                            } 
                            motors[motor].speed += 20;
                            PID_drive_SetSpeed(pid_drive2, motors[motor].speed);
                            break;                            
                        }case Stepper_1:{
                            if (motors[motor].speed == 20) {
                                printf("Maximale Geschwindikeit ereicht!\n");
                                break;
                            } 
                            motors[motor].speed += 2;
                            if (motors[motor].speed == 0){
                                stepper_release(stepper1);
                                motors[motor].direction = Stopp;
                            }else if(motors[motor].speed > 0){
                                stepper_set_speed_rpm(stepper1,motors[motor].speed);
                                motors[motor].direction = Clockwise;
                                stepper_rotate_degrees(stepper1,360);
                            }else {
                                stepper_set_speed_rpm(stepper1,-motors[motor].speed);
                                motors[motor].direction = Counterclockwise;
                                stepper_rotate_degrees(stepper1,-360);
                            }
                            break;                            
                        }case PIO_Stepper_2:{
                            if (motors[motor].speed == 20) {
                                printf("Maximale Geschwindikeit ereicht!\n");
                                break;
                            } 
                            motors[motor].speed += 2;
                            if (motors[motor].speed == 0){
                                pio_stepper_release(stepper2);
                                motors[motor].direction = Stopp;
                            }else if(motors[motor].speed > 0){
                                pio_stepper_set_speed_rpm(stepper2,motors[motor].speed);
                                motors[motor].direction = Clockwise;
                                pio_stepper_rotate_degrees(stepper2,45);
                            }else {
                                pio_stepper_set_speed_rpm(stepper2,-motors[motor].speed);
                                motors[motor].direction = Counterclockwise;
                                pio_stepper_rotate_degrees(stepper2,-45);
                            }
                            break;                            
                        }            
                    }
                    printf("richtung Uhrzeigersinn, Motor:%i, aktuelle Drezahl :%frpm\n",motor, motors[motor].speed);
                    sleep_ms(200);
                }

                if (!button_get(button2)){
                    switch (motor){
                        case DC_1:{
                            if (motors[motor].speed == -200) {
                                printf("Maximale Geschwindikeit ereicht!\n");
                                break;
                            } 
                            motors[motor].speed -= 20;
                            PID_drive_SetSpeed(pid_drive1, motors[motor].speed);
                            break;
                        }case DC_2:{
                            if (motors[motor].speed == -200) {
                                printf("Maximale Geschwindikeit ereicht!\n");
                                break;
                            } 
                            motors[motor].speed -= 20;
                            PID_drive_SetSpeed(pid_drive2, motors[motor].speed);
                            break;                            
                        }case Stepper_1:{
                            if (motors[motor].speed == -20) {
                                printf("Maximale Geschwindikeit ereicht!\n");
                                break;
                            } 
                            motors[motor].speed -= 2;
                            if (motors[motor].speed == 0){
                                stepper_release(stepper1);
                                motors[motor].direction = Stopp;
                            }else if(motors[motor].speed > 0){
                                stepper_set_speed_rpm(stepper1,motors[motor].speed);
                                motors[motor].direction = Clockwise;
                                stepper_rotate_degrees(stepper1,360);
                            }else {
                                stepper_set_speed_rpm(stepper1,-motors[motor].speed);
                                motors[motor].direction = Counterclockwise;
                                stepper_rotate_degrees(stepper1,-360);
                            }
                            break;                            
                        }case PIO_Stepper_2:{
                            if (motors[motor].speed == -20) {
                                printf("Maximale Geschwindikeit ereicht!\n");
                                break;
                            } 
                            motors[motor].speed -= 2;
                            if (motors[motor].speed == 0){
                                pio_stepper_release(stepper2);
                                motors[motor].direction = Stopp;
                            }else if(motors[motor].speed > 0){
                                pio_stepper_set_speed_rpm(stepper2,motors[motor].speed);
                                motors[motor].direction = Clockwise;
                                pio_stepper_rotate_degrees(stepper2,45);
                            }else {
                                pio_stepper_set_speed_rpm(stepper2,-motors[motor].speed);
                                motors[motor].direction = Counterclockwise;
                                pio_stepper_rotate_degrees(stepper2,-45);
                            }
                            break;                            
                        }            
                    }
                    printf("richtung Gegenhrzeigersinn, Motor:%i, aktuelle Drezahl :%frpm\n",motor, motors[motor].speed);
                    sleep_ms(200);
                }              
            }
        }
        oldstate = state;
        state = nextstate;
        tight_loop_contents();
    }
}