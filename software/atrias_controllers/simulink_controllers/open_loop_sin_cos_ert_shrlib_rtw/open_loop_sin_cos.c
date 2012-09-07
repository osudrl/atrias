/*
 * File: open_loop_sin_cos.c
 *
 * Code generated for Simulink model 'open_loop_sin_cos'.
 *
 * Model version                  : 1.16
 * Simulink Coder version         : 8.2 (R2012a) 29-Dec-2011
 * TLC version                    : 8.2 (Jan 25 2012)
 * C/C++ source code generated on : Wed Sep  5 17:03:40 2012
 *
 * Target selection: ert_shrlib.tlc
 * Embedded hardware selection: 32-bit Generic
 * Emulation hardware selection:
 *    Differs from embedded hardware (MATLAB Host)
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#include "open_loop_sin_cos.h"
#include "open_loop_sin_cos_private.h"

/* External outputs (root outports fed by signals with auto storage) */
ExternalOutputs_open_loop_sin_c open_loop_sin_cos_Y;

/* Real-time model */
RT_MODEL_open_loop_sin_cos open_loop_sin_cos_M_;
RT_MODEL_open_loop_sin_cos *const open_loop_sin_cos_M = &open_loop_sin_cos_M_;

/* Model step function */
void open_loop_sin_cos_step(void)
{
  /* Outport: '<Root>/Out1' incorporates:
   *  Sin: '<Root>/Sine Wave'
   */
  open_loop_sin_cos_Y.Out1 = sin(open_loop_sin_cos_P.SineWave_Freq *
    open_loop_sin_cos_M->Timing.t[0] + open_loop_sin_cos_P.SineWave_Phase) *
    open_loop_sin_cos_P.SineWave_Amp + open_loop_sin_cos_P.SineWave_Bias;

  /* Outport: '<Root>/Out2' incorporates:
   *  Sin: '<Root>/Cosine Wave'
   */
  open_loop_sin_cos_Y.Out2 = sin(open_loop_sin_cos_P.CosineWave_Freq *
    open_loop_sin_cos_M->Timing.t[0] + open_loop_sin_cos_P.CosineWave_Phase) *
    open_loop_sin_cos_P.CosineWave_Amp + open_loop_sin_cos_P.CosineWave_Bias;

  /* Update absolute time for base rate */
  /* The "clockTick0" counts the number of times the code of this task has
   * been executed. The absolute time is the multiplication of "clockTick0"
   * and "Timing.stepSize0". Size of "clockTick0" ensures timer will not
   * overflow during the application lifespan selected.
   */
  open_loop_sin_cos_M->Timing.t[0] =
    (++open_loop_sin_cos_M->Timing.clockTick0) *
    open_loop_sin_cos_M->Timing.stepSize0;

  {
    /* Update absolute timer for sample time: [0.001s, 0.0s] */
    /* The "clockTick1" counts the number of times the code of this task has
     * been executed. The resolution of this integer timer is 0.001, which is the step size
     * of the task. Size of "clockTick1" ensures timer will not overflow during the
     * application lifespan selected.
     */
    open_loop_sin_cos_M->Timing.clockTick1++;
  }
}

/* Model initialize function */
void open_loop_sin_cos_initialize(void)
{
  /* Registration code */

  /* initialize real-time model */
  (void) memset((void *)open_loop_sin_cos_M, 0,
                sizeof(RT_MODEL_open_loop_sin_cos));

  {
    /* Setup solver object */
    rtsiSetSimTimeStepPtr(&open_loop_sin_cos_M->solverInfo,
                          &open_loop_sin_cos_M->Timing.simTimeStep);
    rtsiSetTPtr(&open_loop_sin_cos_M->solverInfo, &rtmGetTPtr
                (open_loop_sin_cos_M));
    rtsiSetStepSizePtr(&open_loop_sin_cos_M->solverInfo,
                       &open_loop_sin_cos_M->Timing.stepSize0);
    rtsiSetErrorStatusPtr(&open_loop_sin_cos_M->solverInfo, ((const char_T **)
      (&rtmGetErrorStatus(open_loop_sin_cos_M))));
    rtsiSetRTModelPtr(&open_loop_sin_cos_M->solverInfo, open_loop_sin_cos_M);
  }

  rtsiSetSimTimeStep(&open_loop_sin_cos_M->solverInfo, MAJOR_TIME_STEP);
  rtsiSetSolverName(&open_loop_sin_cos_M->solverInfo,"FixedStepDiscrete");
  rtmSetTPtr(open_loop_sin_cos_M, &open_loop_sin_cos_M->Timing.tArray[0]);
  open_loop_sin_cos_M->Timing.stepSize0 = 0.001;

  /* external outputs */
  (void) memset((void *)&open_loop_sin_cos_Y, 0,
                sizeof(ExternalOutputs_open_loop_sin_c));
}

/* Model terminate function */
void open_loop_sin_cos_terminate(void)
{
  /* (no terminate code required) */
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
