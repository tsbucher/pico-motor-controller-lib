#include <stdio.h>
#include "pico/stdlib.h"

#include "platform.h"

#include "includemyPicolib.h"


typedef enum
{
    Initstate,
    Autostate,
    Manualstate,
} states_t;

void main()
{
    stdio_init_all();
    /*******************************************************************************/
    // 5 Sekunden warten, genug Zeit um Verbindung zu erstellen
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

    My_Platform_Init();
    My_Platform_Test();

      
    // states_t state = Initstate;
    // states_t nextstate;

    // while (true){
    //     switch(state){
    //         case Initstate:{
    //             myLed_Init();
    //             Led_set(ON);
    //             button1 =  button_init(Button1,GPIO_IRQ_EDGE_FALL,pull_up, true);
    //             button2 =  button_init(Button2,GPIO_IRQ_EDGE_FALL,pull_up, true);
    //             button3 =  button_init(Button3,GPIO_IRQ_EDGE_FALL,pull_up, true);
    //             switch1 =  button_init(Switch,GPIO_IRQ_EDGE_FALL,pull_up, true);
    //             poti1 =  my_adc_init(Potentiometer1);
    //             poti2 =  my_adc_init(Potentiometer1);
    //             dcmotor1 = dcmotor_init(DC_Motor1_In1, DC_Motor1_In2, DC_Motor1_PWM, fast_decay);
    //             dcmotor2 = dcmotor_init(DC_Motor2_In1, DC_Motor2_In2, DC_Motor2_PWM, fast_decay);
    //             quadraturencoder_motor1 = Quadratur_Encoder_Init(DC_Motor1_Encoder_A, 0, Puls_Per_Revolution);
    //             quadraturencoder_motor2 = Quadratur_Encoder_Init(DC_Motor2_Encoder_A, 0, Puls_Per_Revolution);
    //             stepper1 = stepper_init(Stepper1_A1, Stepper1_B1, Stepper1_A2, Stepper1_B2, Steps_Per_Revolution, Single);
    //             stepper2 = pio_stepper_init(Stepper2_A1, Stepper2_B1, Stepper2_A2, Stepper2_B2, Steps_Per_Revolution, power);
    //             nextstate = Autostate;
    //             if(button_get(switch1)){
    //                 nextstate = Autostate;
    //             }else{
    //                 nextstate = Manualstate;
    //             }
    //             break;
    //         }
    //         case Autostate:{
    //             if(!gpio_get(Switch)){
    //                 nextstate = Manualstate;
    //             }
    //             break;
    //         }
    //         case Manualstate:{



    //             if(gpio_get(Switch)){
    //                 nextstate = Autostate;
    //             }
    //             break;
    //         }
    //     }
    // }

    while ((true))
    {
        /* code */
    }
}