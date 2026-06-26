/*
 * dab_control.h
 *
 *  Created on: 20-Jun-2026
 *      Author: Kaus
 *  Revised:  correctness fixes (PI naming, clamps, anti-windup, slew)
 */

#ifndef DAB_CONTROL_H_
#define DAB_CONTROL_H_

#include "DCLF32.h"
#include "dab_scaling.h"

// ---------------------------------------------
// Control parameters and limits
// ---------------------------------------------
#define MAX_PHASE_PU   0.25f     // DAB power transfer peaks at 90 deg
#define MIN_PHASE_PU   0.0f

// ---------------------------------------------
// Slew rate limit for the current reference
// Max change in current per ISR (20 us @ 50 kHz).
// 2.5 A over 10 ms = 2.5 / 500 = 0.005 A per ISR.

// ---------------------------------------------
#define MAX_I_SLEW_PER_ISR   0.005f

// ---------------------------------------------
// AFE mode detection thresholds (hysteresis band 380..420 V)
// ---------------------------------------------
#define THRESH_G2V_ENTER   420.0f   // V_bus > this  -> charge  (G2V)
#define THRESH_V2G_ENTER   380.0f   // V_bus < this  -> discharge (V2G)

// ---------------------------------------------
// Charge-mode setpoints / limits
// ---------------------------------------------
#define V_BAT_CV_TARGET    450.0f   // CV regulation target (V)
#define I_CC_LIMIT_A       2.5f     // CC ceiling (A)
#define P_CHG_LIMIT_W      1000.0f  // CP ceiling (W)

// ---------------------------------------------
// Discharge-mode setpoints / limits
// ---------------------------------------------
#define P_DCHG_LIMIT_W     1000.0f  // discharge power (W)
#define I_DCHG_LIMIT_A     2.85f    // hardware current ceiling (A)
#define V_BAT_TAPER_START  360.0f   // begin under-voltage taper (V)
#define V_BAT_TAPER_END    345.0f   // zero current at/below this (V)

// ---------------------------------------------
// PI controllers (names MUST match the definitions in dab_control.c)
// ---------------------------------------------
extern DCL_PI pi_g2v_voltage;   // CV outer loop  (charge)
extern DCL_PI pi_g2v_current;   // CC inner loop  (charge)
extern DCL_PI pi_v2g_current;   // inner loop     (discharge)

// ---------------------------------------------
// Function prototypes
// ---------------------------------------------
void DAB_Control_Init(void);
void DAB_Run_Control(DAB_Measurements_t *meas, float *out_phase, int *out_dir);

#endif /* DAB_CONTROL_H_ */
