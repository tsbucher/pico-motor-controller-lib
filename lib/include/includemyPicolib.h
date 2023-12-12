#ifndef my_include_h
#define my_include_h

#include "includemyPicolib_config.h"
#if Use_Stepper_Motor_With_Repeating_Timer
#include "stepper.h"
#endif
#if Use_Stepper_Motor_With_PIO
#include "pio_stepper.h"
#endif
#if Use_DC_Motor
#include "dc_motor.h"
#endif
#if Use_Button
#include "button.h"
#endif
#if Use_Adc
#include "my_adc.h"
#endif
#if Use_Quadrature_Encoder
#include "quadrature_encoder.h"
#endif
#if Use_Led
#include "my_led.h"
#endif
#if Use_PID_Driver
#include "PID_drive.h"
#endif

// #include "includemyPicolib_config.h"
// #include "stepper.h"
// #include "pio_stepper.h"
// #include "dc_motor.h"
// #include "button.h"
// #include "my_adc.h"
// #include "quadrature_encoder.h"
// #include "my_led.h"
// #include "PID_drive.h"
#endif