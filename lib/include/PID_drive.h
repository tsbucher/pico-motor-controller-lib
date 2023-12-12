/*
 * Copyright (c) 2023 Daniel Bucher
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef PID_Drive_h_
#define PID_Drive_h_

#include "pico/stdlib.h"
#include "includemyPicolib_config.h"
#include "dc_motor.h"
#include "quadrature_encoder.h"

#if Use_PID_Driver 

#define MOTOR_MAX_VALUE      65535                 


typedef void *pid_drive_Handle_t;


/// @brief initialise a PID-Controller for a dc-motor with an encoder
/// @param dcmotor the initialized handle of the dc-motor
/// @param encoder the initialized handle of the encoder
/// @return a handle for the PID-controller(pid_drive_Handle_t) or NULL when somthing went wrong
pid_drive_Handle_t PID_drive_Init(dcmotor_Handle_t dcmotor, quadrature_encoder_Handle_t encoder);

/// @brief set the parameter for the pid_drive
/// @param pid_drive Handle from an pid_drive object
/// @param Kp 
/// @param Ki 
/// @param Kd 
/// @param N 
/// C(s) = Kp+Ki/s+Kd*s/((Kd/(N*Kp))*s+1))
void PID_drive_SetParameters(pid_drive_Handle_t pid_drive,float Kp, float Ki, float Kd, float N);

/// @brief set the parameter for the pid_drive
/// @param pid_drive Handle from an pid_drive object
/// @param Kp 
/// @param Ti 
/// @param Td 
/// @param N 
/// C(s) = Kp*(1+1/(Ti*s)+Td*s/((Td/N)*s+1))
void PID_drive_SetParameters_ideal(pid_drive_Handle_t pid_drive,float Kp, float Ti, float Td, float N);

/// @brief sets the refference speed(Target speed)
/// @param pid_drive Handle from an pid_drive object
/// @param speed target speed for the controller
void PID_drive_SetSpeed(pid_drive_Handle_t pid_drive,float speed);

/// @brief 
/// @param pid_drive Handle from an pid_drive object
/// @return NULL
pid_drive_Handle_t PID_drive_Deinit(pid_drive_Handle_t pid_drive);

#endif

#endif