/*
 * dab_scaling.c
 *
 *  Created on: 18-Jun-2026
 *      Author: Kaus
 */

#include "dab_scaling.h"

// Initialize the measurements structure
DAB_Measurements_t dab_meas = {0};

void DAB_Scale_Sensors(float smooth_V_bus_raw, float smooth_V_bat_raw, DAB_HAL_RawSensors_t* raw_sensors, DAB_Measurements_t* measurements)
{
    // -------------------------------------------------------------
    // 1. VOLTAGE SCALING
    // Formula: V_actual = (V_pin * V_GAIN) + V_OFFSET
    // Note: We use the smoothed floats from Mentor's filter here!
    // -------------------------------------------------------------
    float V_bus_pin = smooth_V_bus_raw * ADC_TO_PIN_VOLTS;
    measurements->V_bus_V = (V_bus_pin * V_GAIN) + V_OFFSET;

    float V_bat_pin = smooth_V_bat_raw * ADC_TO_PIN_VOLTS;
    measurements->V_bat_V = (V_bat_pin * V_GAIN) + V_OFFSET;

    // -------------------------------------------------------------
    // 2. CURRENT SCALING
    // Formula: I_actual = (V_pin / I_SENSITIVITY) + I_OFFSET
    // Note: We use the raw hardware struct here. PPB already subtracted
    // the 1.635V offset. We add I_OFFSET to fix the 0.206A hardware drift.
    // -------------------------------------------------------------
    float I_bus_pin = (float)raw_sensors->I_bus_raw * ADC_TO_PIN_VOLTS;
    measurements->I_bus_A = (I_bus_pin / I_SENSITIVITY) + I_OFFSET;

    float I_bat_pin = (float)raw_sensors->I_bat_raw * ADC_TO_PIN_VOLTS;
    measurements->I_bat_A = (I_bat_pin / I_SENSITIVITY) + I_OFFSET;

    // Calculate Power
    measurements->P_bat_W = measurements->V_bat_V * measurements->I_bat_A;

    // -------------------------------------------------------------
    // 3. PER UNIT CALCULATIONS
    // -------------------------------------------------------------
    measurements->V_bus_pu = measurements->V_bus_V / V_BUS_BASE;
    measurements->V_bat_pu = measurements->V_bat_V / V_BAT_BASE;
    
    measurements->I_bus_pu = measurements->I_bus_A / I_BASE;
    measurements->I_bat_pu = measurements->I_bat_A / I_BASE;

    measurements->P_bat_pu = measurements->P_bat_W / P_BASE;
}
