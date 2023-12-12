#ifndef my_config
#define my_config


/*********************************************************************************/
/*Configuration*/

/*********************************************************************************/
/*on board LED*/
#define Use_Led             (1)
/*********************************************************************************/
/*ADC*/
#define Use_Adc             (1)
#if Use_Adc
    #define Adc_Mean_Enable (0)
    #define Adc_Mean_Num    (100)
#endif
/*********************************************************************************/
/*Switch(Digital Input)*/
#define Use_Button          (1)
/*********************************************************************************/
/*Quadraturencoder*/
#define Use_Quadrature_Encoder                  (1)
#if Use_Quadrature_Encoder
    #define Quadrature_Encoder_PIO              (pio1) 
    #define Quadrature_Encoder_Periode          (25)  /*in ms*/
#endif
/*********************************************************************************/
/*DC_Motor*/
#define Use_DC_Motor                                (1)
#if Use_DC_Motor
    #if Use_Quadrature_Encoder
        #define Use_PID_Driver                      (1)
        #define PID_PERIOD                          (50)                      //sampling time in ms
    #endif    
#endif
/*********************************************************************************/
/*Step_Motor*/
#define Use_Stepper_Motor                           (1)
#if Use_Stepper_Motor
    #define Use_Stepper_Motor_With_Repeating_Timer  (1)
    #define Stepper_Accelerate_Repeating_Timer      (1)
    #define Use_Stepper_Motor_With_PIO              (1)
    #if Use_Stepper_Motor_With_PIO
        #if Use_Quadrature_Encoder
            #define Stepper_PIO                     ((Quadrature_Encoder_PIO == pio0)?pio1:pio0)
        #else
            #define Stepper_PIO                     (pio0)
        #endif
    #endif
#endif
/*********************************************************************************/
#endif