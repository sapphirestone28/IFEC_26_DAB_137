/*
 * dab_scaling.h
 *
 *  Created on: 18-Jun-2026
 *      Author: Kaus
 */

#ifndef DAB_SCALING_H_
#define DAB_SCALING_H_

#include "driverlib.h"
#include "device.h"
#include "board.h"
#include "dab_hal.h"

#define V_BUS_BASE 400.0f // Base voltage for per unit calculations
#define V_BAT_BASE 400.0f // Base voltage for per unit calculations
#define I_BASE     10.0f  // Base current for per unit calculations
#define P_BASE   1000.0f  // Base power for per unit calculations (1 kW)

// Calibration constants for ADC conversion
#define ADC_TO_PIN_VOLTS (3.3f / 4096.0f) // 12-bit ADC with 3.3V reference

// Voltage Calibration
#define V_GAIN           445.017f     // 444.44 * 1.0013
#define V_OFFSET         0.86f        // +0.86V static offset

// Current Calibrations
#define I_SENSITIVITY    0.047316f    // 0.0476 / 1.006
#define I_OFFSET         0.206f       // +0.206A static offset


typedef struct
{
    // Physical values
    float V_bus_V;
    float V_bat_V;
    float I_bus_A;   // Corrected suffix to _A
    float I_bat_A;   // Corrected suffix to _A
    float P_bat_W;

    // Per unit values for PI Control and math 
    float V_bus_pu;
    float V_bat_pu;
    float I_bus_pu;
    float I_bat_pu;
    float P_bat_pu;  // Added for Constant Power mode
} DAB_Measurements_t;

// Function Prototype updated to accept the pre-filtered raw voltages
void DAB_Scale_Sensors(float smooth_V_bus_raw, float smooth_V_bat_raw, DAB_HAL_RawSensors_t* raw_sensors, DAB_Measurements_t* measurements);

#endif /* DAB_SCALING_H_ */
