//#############################################################################
//
// FILE:   empty_driverlib_main.c
//
//! \addtogroup driver_example_list
//! <h1>Empty Project Example</h1> 
//!
//! This example is an empty project setup for Driverlib development.
//!
//
//#############################################################################
//
//
// $Copyright:
// Copyright (C) 2025 Texas Instruments Incorporated - http://www.ti.com/
//
// Redistribution and use in source and binary forms, with or without 
// modification, are permitted provided that the following conditions 
// are met:
// 
//   Redistributions of source code must retain the above copyright 
//   notice, this list of conditions and the following disclaimer.
// 
//   Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in the 
//   documentation and/or other materials provided with the   
//   distribution.
// 
//   Neither the name of Texas Instruments Incorporated nor the names of
//   its contributors may be used to endorse or promote products derived
//   from this software without specific prior written permission.
// 
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT 
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
// $
//#############################################################################

//
// Included Files
//
#include "driverlib.h"
#include "device.h"
#include "board.h"
#include "c2000ware_libraries.h"


volatile float target_phase_shift_pu = 0.25f;
volatile uint16_t phase_ticks = 0;

void delay_loop(void)
{
    DEVICE_DELAY_US(500000);
}
//
// Main
//
void main(void)
{

    //
    // Initialize device clock and peripherals
    //
    Device_init();

    //
    // Disable pin locks and enable internal pull-ups.
    //
    Device_initGPIO();

    //
    // Initialize PIE and clear PIE registers. Disables CPU interrupts.
    //
    Interrupt_initModule();

    //
    // Initialize the PIE vector table with pointers to the shell Interrupt
    // Service Routines (ISR).
    //
    Interrupt_initVectorTable();

    //
    // PinMux and Peripheral Initialization
    //
    Board_init();

    //
    // C2000Ware Library initialization
    //
    C2000Ware_libraries_init();

    //
    // Enable Global Interrupt (INTM) and real time interrupt (DBGM)
    //
    EINT;
    ERTM;

    while(1)
    {

        //clamp the target phase shift to be within -1.0 to 1.0 pu
        if(target_phase_shift_pu > 1.0f) target_phase_shift_pu = 1.0f;
        if(target_phase_shift_pu < -1.0f) target_phase_shift_pu = -1.0f;


        //calculate the number of timer ticks corresponding to the target phase shift
        float abs_phase = target_phase_shift_pu;
        if(abs_phase < 0) abs_phase = -abs_phase;
        phase_ticks = (uint16_t)(abs_phase * 240); 



        // set the count direction based on the sign of the target phase shift
        EPWM_SyncCountMode count_dir = EPWM_COUNT_MODE_UP_AFTER_SYNC;
        if (target_phase_shift_pu < 0.0f) count_dir = EPWM_COUNT_MODE_DOWN_AFTER_SYNC;


        // update the EPWM sync settings to apply the new phase shift
        EPWM_setPhaseShift(myEPWM3_BASE, phase_ticks);
        EPWM_setCountModeAfterSync(myEPWM3_BASE, count_dir);

        EPWM_setPhaseShift(myEPWM3_BASE, phase_ticks);
        EPWM_setCountModeAfterSync(myEPWM3_BASE, count_dir);

    }
}

//
// End of File
//
