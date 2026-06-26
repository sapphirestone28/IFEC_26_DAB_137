// #############################################################################
//  FILE:   main.c
//  TITLE:  1kW Bidirectional DAB Controller (F2800137)
//  Revised: current filter now feeds the controller; bounded ISR wait
// #############################################################################

#include "driverlib.h"
#include "device.h"
#include "board.h"
#include "c2000ware_libraries.h"

// Custom modules
#include "dab_hal.h"
#include "dab_scaling.h"
#include "dab_filter.h"
#include "dab_control.h"

// ---------------------------------------------------------
// GLOBALS
// ---------------------------------------------------------
DAB_HAL_RawSensors_t dab_sensors;
extern DAB_Measurements_t dab_meas;   // defined in dab_scaling.c

// ISR profiling
volatile uint32_t isr_start_time = 0;
volatile uint32_t isr_end_time   = 0;
volatile uint32_t isr_total_cycles = 0;

// Debugger flag: set to 1 in CCS Expressions window to clear trips.
volatile int clear_hardware_faults = 0;

// ---------------------------------------------------------
// THE 50 kHz FAST ISR
// ---------------------------------------------------------
__interrupt void DAB_Fast_ISR(void)
{
    // 0. Start profiling timer
    isr_start_time = CPUTimer_getTimerCount(myCPUTIMER0_BASE);

    // SOFTWARE "AND" GUARD:
        // ADCA (Voltage) triggered this ISR. Wait for ADCC (Current) to finish.
        {
            uint16_t guard = 0;
            // Check myADC2_BASE
            while (ADC_getInterruptStatus(myADC2_BASE, ADC_INT_NUMBER1) == false) {
                if (++guard > 1000U) break;   // timeout -> proceed
            }
        }

    // 1. HARDWARE: read raw sensors
    DAB_HAL_readSensors(&dab_sensors);

    // 2. FILTER: 50-pt moving average + IIR on RAW voltages (ADC counts)
    float smooth_V_bus_raw = DAB_Run_Voltage_Filter(&filt_V_bus, dab_sensors.V_bus_raw);
    float smooth_V_bat_raw = DAB_Run_Voltage_Filter(&filt_V_bat, dab_sensors.V_bat_raw);

    // 3. SCALING: counts -> Volts/Amps and per-unit
    DAB_Scale_Sensors(smooth_V_bus_raw, smooth_V_bat_raw, &dab_sensors, &dab_meas);

    // 4. FILTER: fast IIR on the SCALED CURRENTS IN AMPS (what control uses),
    //    then refresh the per-unit and power values from the filtered current.
    dab_meas.I_bus_A  = DAB_Run_Fast_IIR(&filt_I_bus, dab_meas.I_bus_A);
    dab_meas.I_bat_A  = DAB_Run_Fast_IIR(&filt_I_bat, dab_meas.I_bat_A);

    dab_meas.I_bus_pu = dab_meas.I_bus_A / I_BASE;
    dab_meas.I_bat_pu = dab_meas.I_bat_A / I_BASE;
    dab_meas.P_bat_W  = dab_meas.V_bat_V * dab_meas.I_bat_A;
    dab_meas.P_bat_pu = dab_meas.P_bat_W / P_BASE;

    // 5. CONTROL: CC-CP-CV + direction logic
    float phase_command = 0.0f;
    int   direction     = DAB_DIR_CHARGE;
    DAB_Run_Control(&dab_meas, &phase_command, &direction);

    // 6. HARDWARE: apply phase shift to the secondary ePWMs
    DAB_HAL_updatePhaseShift(phase_command, direction);

    // 7. CLEAR INTERRUPTS
    ADC_clearInterruptStatus(myADC1_BASE, ADC_INT_NUMBER1);
    ADC_clearInterruptStatus(myADC2_BASE, ADC_INT_NUMBER1);
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP1);

    // 8. End profiling (timer counts DOWN: start - end = cycles consumed)
    isr_end_time     = CPUTimer_getTimerCount(myCPUTIMER0_BASE);
    isr_total_cycles = isr_start_time - isr_end_time;
}

// ---------------------------------------------------------
// UTILITY
// ---------------------------------------------------------
void delay_loop(void)
{
    DEVICE_DELAY_US(100000);   // 100 ms
}

// ---------------------------------------------------------
// MAIN
// ---------------------------------------------------------
void main(void)
{
    // 1. Standard TI init
    Device_init();
    Device_initGPIO();
    Interrupt_initModule();
    Interrupt_initVectorTable();
    Board_init();
    C2000Ware_libraries_init();

    // 2. Custom DAB modules
    DAB_HAL_Init();      // PWMs disabled
    DAB_Filter_Init();   // buffers cleared, alphas set
    DAB_Control_Init();  // PI Kp/Ki, output clamps, integrators reset

    // 3. Register the fast ISR
    Interrupt_register(INT_myADC1_1, &DAB_Fast_ISR);

    // 4. Global interrupts
    EINT;
    ERTM;

    // 5. Enable gate drivers
    DAB_HAL_enablePWM();

    // 6. Background loop (~10 Hz)
    while (1)
    {
        if (clear_hardware_faults == 1)
        {
            DAB_HAL_clearHardwareTrips();
            clear_hardware_faults = 0;
        }

        GPIO_togglePin(LED_DEBUG);
        delay_loop();
    }
}

// End of File
