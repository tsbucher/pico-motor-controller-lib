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
    Manualstate,
} states_t;

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

    My_Platform_Init();
    My_Platform_Test();

    
    while ((true))
    {
        tight_loop_contents();
        /* code */
    }
}