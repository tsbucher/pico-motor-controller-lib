#ifndef my_Handletyp_h
#define my_Handletyp_h
typedef enum
{
    null,
    dcmotor_type,
    stepper_type,
    pio_stepper_type,
    button_type,
    adc_type,
    piddrive_type,
    quadraturencoder_type
} my_handletype_t;  

#endif