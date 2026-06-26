/*
 * dab_control.c
 *
 *  Created on: 20-Jun-2026
 *      Author: Kaus
 *  Revised:  correctness fixes
 *    - PI instance names match the header (link error fixed)
 *    - CV/CC/CP selection via dynamic Umax => correct anti-windup + handover
 *    - I_target can never go negative
 *    - all three integrators fully reset on a direction swap (DCL_resetPI)
 *    - inner-loop output clamped by the controller (Umax/Umin set in Init)
 *    - explicit standby handling
 */

#include "dab_control.h"
#include "DCLF32.h"
#include <math.h>

// ---------------------------------------------------------
// CONTROLLER INSTANCES  (direct-style: no sps/css required)
// ---------------------------------------------------------
DCL_PI pi_g2v_voltage = {0};   // CV outer loop  (charge)
DCL_PI pi_g2v_current = {0};   // CC inner loop  (charge)
DCL_PI pi_v2g_current = {0};   // inner loop     (discharge)
// ---------------------------------------------------------
// STATE & SLEW VARIABLES
// ---------------------------------------------------------
// 0 = Standby, +1 = Charge (G2V), -1 = Discharge (V2G)
static int   active_dab_mode = 0;
static float I_ref_slewed    = 0.0f;   // slewed reference fed to the inner loop

// ---------------------------------------------------------
// INIT
// ---------------------------------------------------------
// NOTE: direct-style tuning. Because we write pi_g2v_voltage.Umax every ISR,
// do NOT call DCL_updatePI() on these controllers (it would overwrite Umax
// from the shadow set). Also keep DCL_ERROR_HANDLING/TESTPOINTS disabled,
// since sps/css are not wired up here.
void DAB_Control_Init(void)
{
    // ---- Inner current loops: output is phase in PU, clamp to [0, 0.25] ----
    pi_g2v_current.Kp   = 0.01f;            // <-- TUNE on bench
    pi_g2v_current.Ki   = 0.001f;           // <-- TUNE (Ki absorbs the timestep)
    pi_g2v_current.Umax = MAX_PHASE_PU;     // 0.25
    pi_g2v_current.Umin = MIN_PHASE_PU;     // 0.0
    DCL_resetPI(&pi_g2v_current);

    pi_v2g_current.Kp   = 0.01f;            // <-- TUNE
    pi_v2g_current.Ki   = 0.001f;           // <-- TUNE
    pi_v2g_current.Umax = MAX_PHASE_PU;
    pi_v2g_current.Umin = MIN_PHASE_PU;
    DCL_resetPI(&pi_v2g_current);

    // ---- CV outer loop: output is a CURRENT command (A) ----
    // Umax is overwritten each ISR with the live CC/CP limit (see below).
    pi_g2v_voltage.Kp   = 0.05f;            // <-- TUNE
    pi_g2v_voltage.Ki   = 0.005f;           // <-- TUNE
    pi_g2v_voltage.Umax = I_CC_LIMIT_A;
    pi_g2v_voltage.Umin = 0.0f;             // CV command can never go negative
    DCL_resetPI(&pi_g2v_voltage);

    active_dab_mode = 0;
    I_ref_slewed    = 0.0f;
}

// ---------------------------------------------------------
// MAIN CONTROL STEP  (called once per 50 kHz ISR)
// ---------------------------------------------------------
void DAB_Run_Control(DAB_Measurements_t *meas, float *out_phase, int *out_dir)
{
    float I_target      = 0.0f;
    int   requested_mode = active_dab_mode;

    // ======================================================
    // 1. AFE INTENTION DETECTION (hysteresis 380..420 V)
    // ======================================================
    if (meas->V_bus_V > THRESH_G2V_ENTER) {
        requested_mode = DAB_DIR_CHARGE;
    }
    else if (meas->V_bus_V < THRESH_V2G_ENTER) {
        requested_mode = DAB_DIR_DISCHARGE;
    }
    // else: stay in the current mode (deadband)

    // ======================================================
    // 2. ZERO-DWELL DIRECTION-SWAP MANAGER
    //    Ramp to zero current before changing direction.
    // ======================================================
    if (requested_mode != active_dab_mode)
    {
        I_target = 0.0f;   // force ramp-down regardless of mode

        if (I_ref_slewed <= 0.01f)
        {
            active_dab_mode = requested_mode;   // safe to swap now

            // Full reset of ALL controllers (i10, i11, i6) so no stale
            // integral or frozen anti-windup carries across the swap.
            DCL_resetPI(&pi_g2v_voltage);
            DCL_resetPI(&pi_g2v_current);
            DCL_resetPI(&pi_v2g_current);
        }
    }
    else
    {
        // ==================================================
        // 3. TARGET CURRENT (we are settled in a mode)
        // ==================================================
        if (active_dab_mode == DAB_DIR_CHARGE)
        {;
            // CC and CP limits, both in Amps.

            float safe_v_bat = fmaxf(meas->V_bat_V, 10.0f);
            float cc_cp_limit = fminf(I_CC_LIMIT_A, P_CHG_LIMIT_W / safe_v_bat);


            // Clamp the CV loop output to the live CC/CP limit. The DCL
            // anti-windup (i6) then freezes the CV integrator whenever
            // CC/CP is the active limit, giving a clean CC->CV handover.
            pi_g2v_voltage.Umax = cc_cp_limit;
            pi_g2v_voltage.Umin = 0.0f;

            // Result is min(CV, CC, CP), guaranteed >= 0.
            I_target = DCL_runPI_C6(&pi_g2v_voltage,
                                    V_BAT_CV_TARGET, meas->V_bat_V);
        }
        else if (active_dab_mode == DAB_DIR_DISCHARGE)
        {
            float safe_v_bat = fmaxf(meas->V_bat_V, 10.0f);
            I_target = P_DCHG_LIMIT_W / safe_v_bat;
            I_target = fminf(I_target, I_DCHG_LIMIT_A);

            // Under-voltage taper: linear ramp to 0 A between 360 V and 345 V.
            if (meas->V_bat_V < V_BAT_TAPER_START) {
                float span  = V_BAT_TAPER_START - V_BAT_TAPER_END;   // 15 V
                float taper = (meas->V_bat_V - V_BAT_TAPER_END)
                              * (I_DCHG_LIMIT_A / span);
                I_target = fminf(I_target, taper);
            }
            if (meas->V_bat_V <= V_BAT_TAPER_END) {
                I_target = 0.0f;
            }

            if (I_target < 0.0f) I_target = 0.0f;   // safety clamp
        }
        else
        {
            // Standby: no power.
            I_target = 0.0f;
        }
    }

    // ======================================================
    // 4. SLEW-RATE LIMITER (step the reference toward target)
    // ======================================================
    if (I_ref_slewed < I_target) {
        I_ref_slewed += MAX_I_SLEW_PER_ISR;
        if (I_ref_slewed > I_target) I_ref_slewed = I_target;
    }
    else if (I_ref_slewed > I_target) {
        I_ref_slewed -= MAX_I_SLEW_PER_ISR;
        if (I_ref_slewed < I_target) I_ref_slewed = I_target;
    }
    if (I_ref_slewed < 0.0f) I_ref_slewed = 0.0f;   // magnitude, never negative

    // ======================================================
    // 5. INNER CURRENT LOOP (runs on the SLEWED reference)
    // ======================================================
    *out_dir = active_dab_mode;

    if (active_dab_mode == DAB_DIR_CHARGE) {
        *out_phase = DCL_runPI_C6(&pi_g2v_current,
                                  I_ref_slewed, meas->I_bat_A);
    }
    else if (active_dab_mode == DAB_DIR_DISCHARGE) {
        *out_phase = DCL_runPI_C6(&pi_v2g_current,
                                  I_ref_slewed, fabsf(meas->I_bat_A));
    }
    else {
        *out_phase = 0.0f;   // standby
    }
}
