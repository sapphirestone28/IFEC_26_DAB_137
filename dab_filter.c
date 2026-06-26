/*
 * dab_filter.c
 *
 *  Created on: 19-Jun-2026
 *      Author: Kaus
 */


#include "dab_filter.h"

Voltage_Filter_t   filt_V_bus;
Voltage_Filter_t   filt_V_bat;
Fast_IIR_Filter_t filt_I_bus;
Fast_IIR_Filter_t filt_I_bat;

void DAB_Filter_Init(void)
{
    int i;
    
    // Initialize Voltage Filters (Alpha = 0.385)
    filt_V_bus.alpha = 0.385f;
    filt_V_bus.sum = 0;
    filt_V_bus.index = 0;
    filt_V_bus.prev_out = 0.0f;
    
    filt_V_bat.alpha = 0.385f;
    filt_V_bat.sum = 0;
    filt_V_bat.index = 0;
    filt_V_bat.prev_out = 0.0f;

    for(i = 0; i < VOLTAGE_BUF_SIZE; i++) {
        filt_V_bus.buffer[i] = 0;
        filt_V_bat.buffer[i] = 0;
    }

    // Initialize Fast Current Filters (Alpha = 0.20 to 0.40)
    filt_I_bus.alpha = 0.30f;
    filt_I_bus.prev_out = 0.0f;
    
    filt_I_bat.alpha = 0.30f;
    filt_I_bat.prev_out = 0.0f;
}

// -------------------------------------------------------------
// VOLTAGE FILTER
// -------------------------------------------------------------
float DAB_Run_Voltage_Filter(Voltage_Filter_t *filt, uint16_t new_adc_sample)
{
    // 1. Moving Average
    filt->sum -= filt->buffer[filt->index];
    filt->buffer[filt->index] = new_adc_sample;
    filt->sum += new_adc_sample;
    
    // Optimized circular buffer (Faster than Modulo % operator)
    filt->index++;
    if(filt->index >= VOLTAGE_BUF_SIZE) {
        filt->index = 0;
    }

    float moving_avg = (float)filt->sum * VOLTAGE_BUF_INV;

    // 2. IIR Filter
    float out = (filt->alpha * moving_avg) + ((1.0f - filt->alpha) * filt->prev_out);
    filt->prev_out = out;

    // Returns a highly smoothed ADC count
    return out; 
}

// -------------------------------------------------------------
// FAST IIR FILTER (For Currents only)
// -------------------------------------------------------------
float DAB_Run_Fast_IIR(Fast_IIR_Filter_t *filt, float input)
{
    float out = (filt->alpha * input) + ((1.0f - filt->alpha) * filt->prev_out);
    filt->prev_out = out;
    return out;
}
