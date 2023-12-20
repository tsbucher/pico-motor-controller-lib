#include <stdio.h>
#include "pico/stdlib.h"
#include "includemyPicolib_config.h"
#include "platform.h"
#include "my_adc.h"
#include "hardware/adc.h"
#include "my_led.h"
#include "includemyPicolib.h"

#if Use_Quadrature_Encoder
quadrature_encoder_Handle_t quadraturencoder_motor1;
quadrature_encoder_Handle_t quadraturencoder_motor2;
#endif
#if Use_DC_Motor
dcmotor_Handle_t dcmotor1;
const dcmotor_mode_t dcmotor1_mode = fast_decay;

dcmotor_Handle_t dcmotor2;
const dcmotor_mode_t dcmotor2_mode = slow_decay;
#endif
#if Use_PID_Driver
pid_drive_Handle_t pid_drive1;
pid_drive_Handle_t pid_drive2;
#endif
#if Use_Stepper_Motor_With_Repeating_Timer
stepper_Handle_t stepper1;
#endif
#if Use_Stepper_Motor_With_PIO
pio_stepper_Handle_t stepper2;
#endif
#if Use_Button
button_Handle_t button1;
button_Handle_t button2;
button_Handle_t button3;
button_Handle_t switch1;
#endif
#if Use_Adc
adc_sensor_Handle_t poti1;
adc_sensor_Handle_t poti2;
#endif

void My_Platform_Init(void){
    #if Use_Adc
    adc_init();
    #endif
    #if Use_Led
#if Use_Led
    myLed_Init();
    #endif 
    
    /********************************************************************************************************/
    #if Use_Button
    //Button Initalisieren
    button1 =  button_init(Button1,GPIO_IRQ_EDGE_FALL ,pull_up, true);
    if (button1 == NULL) printf("Error\n");
    button2 =  button_init(Button2,GPIO_IRQ_EDGE_FALL,pull_up, true);
    if (button2 == NULL) printf("Error\n");
    button3 =  button_init(Button3,GPIO_IRQ_EDGE_FALL,pull_up, true);
    if (button3 == NULL) printf("Error\n");
    switch1 =  button_init(Switch,GPIO_IRQ_EDGE_FALL,pull_up, true);
    if (switch1 == NULL) printf("Error\n");
    printf("Button initalisiert\n");
    #endif
    /********************************************************************************************************/
    #if Use_Quadrature_Encoder
    //Encoder initalisieren
    quadraturencoder_motor1 = Quadratur_Encoder_Init(DC_Motor1_Encoder_A, 0, Puls_Per_Revolution);
    if (!Quadratur_Encoder_start(quadraturencoder_motor1)) printf("Errore");
    quadraturencoder_motor2 = Quadratur_Encoder_Init(DC_Motor2_Encoder_A, 0, Puls_Per_Revolution);
    if (!Quadratur_Encoder_start(quadraturencoder_motor2)) printf("Errore");
    printf("Encoder initalisiert\n");
    #endif
    /********************************************************************************************************/
    #if Use_DC_Motor
    //Dcmotoren initalisieren
    dcmotor1 = dcmotor_init(DC_Motor1_In1, DC_Motor1_In2, DC_Motor1_PWM, dcmotor1_mode);
    if (dcmotor1 == NULL) printf("Error\n");
    dcmotor2 = dcmotor_init(DC_Motor2_In1, DC_Motor2_In2, DC_Motor2_PWM, dcmotor2_mode);
    if (dcmotor1 == NULL) printf("Error\n");
    printf("DC-Motoren initalisiert\n");
    #endif
    /********************************************************************************************************/
    #if Use_PID_Driver
    //PID-Regler initalisiert
    pid_drive1 = PID_drive_Init(dcmotor1, quadraturencoder_motor1);
    if (pid_drive1 == NULL) printf("Error\n");
    PID_drive_SetParameters_ideal(pid_drive1, PID_Kp, PID_Ti,0.005,PID_N);
    pid_drive2 = PID_drive_Init(dcmotor2, quadraturencoder_motor2);
    if (pid_drive2 == NULL) printf("Error\n");
    PID_drive_SetParameters(pid_drive2, PID_Kp, PID_Ki,PID_Kd,PID_N);
    printf("PID-Regler initalisiert\n");
    #endif
    /********************************************************************************************************/
    #if Use_Stepper_Motor_With_Repeating_Timer
    //Stepper Initalisiert
    stepper1 = stepper_init(Stepper1_A1, Stepper1_B1, Stepper1_A2, Stepper1_B2, Steps_Per_Revolution, Single);
    if (stepper1 == NULL) printf("Error\n");
    printf("stepper initalisiert\n");
    #endif
    /********************************************************************************************************/
    #if Use_Stepper_Motor_With_PIO
    // PIO-Stepper Initalisieren
    stepper2 = pio_stepper_init(Stepper2_A1, Stepper2_B1, Stepper2_A2, Stepper2_B2, Steps_Per_Revolution, power);
    if (stepper2 == NULL) printf("Error\n");
    printf("PIO-Stepper initalisiert\n");
    #endif
    /********************************************************************************************************/
    #if Use_Adc
    // ADC initalisieren
    poti1 =  my_adc_init(Potentiometer1);
    poti2 =  my_adc_init(Potentiometer2);
    printf("ADC initalisiert\n");
    #endif
#endif   
#if Use_Adc
    adc_init();
#endif 
}

void My_Platform_Test(void){    
    /********************************************************************************************************/
    #if Use_Button
    //Button Test
    printf("Button Test:\n");
    for(int i = 0; i<100;i++){
        printf("button 1: %d\n",button_get(button1));
        printf("button 2: %d\n",button_get(button2));
        printf("button 3: %d\n",button_get(button3));
        printf("switch 1: %d\n",button_get(switch1));
        sleep_ms(100);
    }
    #endif
    /*******************************************************************************************************/ 
    #if (Use_DC_Motor & Use_Quadrature_Encoder)
    //DC-Motoren und Encoder Test   
    #if Use_PID_Driver
    //PID-Regler initalisiert
    pid_drive1 = PID_drive_Deinit(pid_drive1);
    pid_drive2 = PID_drive_Deinit(pid_drive2);
    #endif
    printf("DC-Motoren und Encoder Test:\n");
    dcmotor_set(dcmotor1, Clockwise, 40000);
    printf("Motor uhrzeigersinn 40000\n");
    sleep_ms(2000);
    printf("speed: %f\n", Quadratur_Encoder_get_speed(quadraturencoder_motor1));
    dcmotor_change_speed(dcmotor1, 25000);
    printf("Motor uhrzeigersinn 25000\n");
    sleep_ms(500);
    for (size_t i = 0; i < 4; i++)
    {
        printf("speed: %f\n", Quadratur_Encoder_get_speed(quadraturencoder_motor1));
        sleep_ms(1000);
    }
    dcmotor_stop(dcmotor1);
    sleep_ms(2000);

    dcmotor_set(dcmotor2, Clockwise, 40000);
    printf("Motor uhrzeigersinn 40000\n");
    sleep_ms(2000);
    printf("speed: %f\n", Quadratur_Encoder_get_speed(quadraturencoder_motor2));
    dcmotor_change_speed(dcmotor2, 25000);
    printf("Motor uhrzeigersinn 25000\n");
    sleep_ms(500);
    for (size_t i = 0; i < 4; i++)
    {
        printf("speed: %f\n", Quadratur_Encoder_get_speed(quadraturencoder_motor2));
        sleep_ms(1000);
    }
    dcmotor_stop(dcmotor2);
    sleep_ms(2000);
    #endif
    /********************************************************************************************************/
    #if Use_PID_Driver
    //PID-Controller Test
    #if Use_PID_Driver
    //PID-Regler initalisiert
    pid_drive1 = PID_drive_Init(dcmotor1, quadraturencoder_motor1);
    if (pid_drive1 == NULL) printf("Error\n");
    PID_drive_SetParameters_ideal(pid_drive1, PID_Kp, PID_Ti,0.005,PID_N);
    pid_drive2 = PID_drive_Init(dcmotor2, quadraturencoder_motor2);
    if (pid_drive2 == NULL) printf("Error\n");
    PID_drive_SetParameters(pid_drive2, PID_Kp, PID_Ki,PID_Kd,PID_N);
    printf("PID-Regler initalisiert\n");
    #endif
    printf("PID-Regler Test:\n");
    PID_drive_SetSpeed(pid_drive1, 100);
    PID_drive_SetSpeed(pid_drive2, 100);
    for (int i  = 0; i<50;i++){
        printf("speed1: %f\n", Quadratur_Encoder_get_speed(quadraturencoder_motor1));
        printf("speed2: %f\n", Quadratur_Encoder_get_speed(quadraturencoder_motor2));
        sleep_ms(100);
    }    
    PID_drive_SetSpeed(pid_drive1, 0);
    PID_drive_SetSpeed(pid_drive2, 0);
    for (int i  = 0; i<50;i++){
        printf("speed1: %f\n", Quadratur_Encoder_get_speed(quadraturencoder_motor1));
        printf("speed2: %f\n", Quadratur_Encoder_get_speed(quadraturencoder_motor2));
        sleep_ms(100);
    }   
    PID_drive_Deinit(pid_drive2);
    #endif
    /********************************************************************************************************/
    #if Use_Stepper_Motor_With_PIO
    // PIO-Stepper Test
    printf("PIO-Stepper test:\n");
    pio_stepper_set_speed_rpm(stepper2, 4);
    pio_stepper_rotate_degrees(stepper2, 4000);
    printf("Schrittmotor2 drehen um zwei umdrehung");
    #if Stepper_Accelerate_Repeating_Timer
    pio_stepper_accelerate(stepper2, 10, 1000);
    #else
    pio_stepper_set_speed_rpm(stepper2, 10);   
    #endif
    sleep_ms(1000);
    pio_stepper_set_mode(stepper2, power);
    printf("Schrittmotor2 change power\n");
    sleep_ms(2000);
    pio_stepper_set_direction(stepper2, counterclockwise);
    printf("Schrittmotor2 change direction\n");
    sleep_ms(2000);
    pio_stepper_set_direction(stepper2, clockwise);
    printf("Schrittmotor2 change direction\n");
    sleep_ms(5000);
    pio_stepper_set_mode(stepper2, half);
    printf("Schrittmotor2 change power\n");
    sleep_ms(5000);
    pio_stepper_set_mode(stepper2, singel);
    printf("Schrittmotor2 change singel\n");
    sleep_ms(5000);
    #if Stepper_Accelerate_Repeating_Timer
    pio_stepper_accelerate(stepper2, 0.01, 1000);
    #else
    pio_stepper_set_speed_rpm(stepper2, 0.01);   
    #endif
    sleep_ms(4000);
    pio_stepper_release(stepper2);
    printf("Schrittmotor2 releas\n");
    #endif
    /********************************************************************************************************/
    #if (Use_Stepper_Motor_With_Repeating_Timer && Use_Stepper_Motor_With_PIO)
    // Stepper Test
    printf("stepper Test:\n");
    stepper_set_speed_rpm(stepper1,0.1);
    pio_stepper_set_speed_rpm(stepper2,0.1);
    pio_stepper_rotate_degrees(stepper2, 360);
    stepper_rotate_degrees(stepper1, 360);
    sleep_ms(2000);    
    #if Stepper_Accelerate_Repeating_Timer
    pio_stepper_accelerate(stepper2,15,1000);
    stepper_accelerate(stepper1 ,15,1000);
    #else
    stepper_set_speed_rpm(stepper1, 12);   
    pio_stepper_set_speed_rpm(stepper2, 12);   
    #endif
    
    sleep_ms(10000);
    #if Stepper_Accelerate_Repeating_Timer
    bool d = false;
    bool once = false;
    for (size_t i = 0; i < 4; ){
        if(!stepper_is_moving(stepper1)){
            stepper_set_speed_rpm(stepper1, 1);
            pio_stepper_set_speed_rpm(stepper2, 1);
            if (d){
                stepper_rotate_degrees(stepper1,360);
                pio_stepper_rotate_degrees(stepper2,360);
                stepper_accelerate(stepper1 ,15,2000);
                pio_stepper_accelerate(stepper2,15,2000);
                i++;
                d = false;
            }else{
                stepper_rotate_degrees(stepper1,-360);
                pio_stepper_rotate_degrees(stepper2,-360);
                stepper_accelerate(stepper1 ,15,2000);
                pio_stepper_accelerate(stepper2,15,2000);
                i++;
                d = true;
            }
        }
        sleep_ms(100);
    }
    #endif
    stepper_release(stepper1);
    pio_stepper_release(stepper2);
    printf("Schrittmotoren releas\n");
    sleep_ms(2000);
    #endif
    /********************************************************************************************************/
    #if Use_Adc
    // ADC Test
    printf("ADC Test:\n");
    printf("Poti 1 : %d\n", my_adc_sampelandread(poti1));
    printf("Poti 2 : %d\n", my_adc_sampelandread(poti2));
    my_adc_sampel(poti1);
    my_adc_sampel(poti2);
    printf("Poti 1 : %d\n", my_adc_get(poti1));
    printf("Poti 2 : %d\n", my_adc_get(poti2));
    #endif
    /********************************************************************************************************/
    #if (Use_PID_Driver && Use_Button && Use_Adc && Use_Stepper_Motor_With_Repeating_Timer)
    // Stepper and PID-Controller with ADC Test
    printf("Test everything:\n");

    stepper_set_speed_rpm(stepper1, 8);

    uint16_t wert,altwert;
    int16_t angel;
    float delta;
    int16_t angel_old = 10;
    int16_t toleranz = 0;
    float sollspeed = 0;
    while(true){
        if (!button_get(button1)){
            wert = my_adc_sampelandread(poti1);
            toleranz = wert / 40 +10;
            delta = (altwert - wert);
            if ((delta > toleranz) || (delta > -toleranz)){
                angel = (wert / 16) - 128;
                delta = (float)(angel - angel_old);
                angel_old = angel;
                stepper_rotate_degrees(stepper1, delta);
                sollspeed = (float)(wert-2000) / (float)12 ;
                PID_drive_SetSpeed(pid_drive1, sollspeed);
            }
            altwert = wert;
            sleep_ms(100);
        }
        if (!button_get(button2)){
            sleep_ms(100);
            stepper_reset_position(stepper1);
        }
    }
    #endif
    /********************************************************************************************************/
}



 
 



