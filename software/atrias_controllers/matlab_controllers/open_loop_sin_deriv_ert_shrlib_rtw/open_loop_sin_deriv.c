/*
 * File: open_loop_sin_deriv.c
 *
 * Code generated for Simulink model 'open_loop_sin_deriv'.
 *
 * Model version                  : 1.14
 * Simulink Coder version         : 8.2 (R2012a) 29-Dec-2011
 * TLC version                    : 8.2 (Jan 25 2012)
 * C/C++ source code generated on : Thu Sep  6 12:03:13 2012
 *
 * Target selection: ert_shrlib.tlc
 * Embedded hardware selection: 32-bit Generic
 * Emulation hardware selection:
 *    Differs from embedded hardware (MATLAB Host)
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#include "open_loop_sin_deriv.h"
#include "open_loop_sin_deriv_private.h"

/* Block signals (auto storage) */
BlockIO_open_loop_sin_deriv open_loop_sin_deriv_B;

/* Block states (auto storage) */
D_Work_open_loop_sin_deriv open_loop_sin_deriv_DWork;

/* External outputs (root outports fed by signals with auto storage) */
ExternalOutputs_open_loop_sin_d open_loop_sin_deriv_Y;

/* Real-time model */
RT_MODEL_open_loop_sin_deriv open_loop_sin_deriv_M_;
RT_MODEL_open_loop_sin_deriv *const open_loop_sin_deriv_M =
  &open_loop_sin_deriv_M_;

/* Model step function */
void open_loop_sin_deriv_step(void)
{
  /* local block i/o variables */
  real_T rtb_DirtyDerivative;

  /* Sin: '<Root>/Sine Wave' */
  open_loop_sin_deriv_B.SineWave = sin(open_loop_sin_deriv_P.SineWave_Freq *
    open_loop_sin_deriv_M->Timing.t[0] + open_loop_sin_deriv_P.SineWave_Phase) *
    open_loop_sin_deriv_P.SineWave_Amp + open_loop_sin_deriv_P.SineWave_Bias;

  /* SignalConversion: '<Root>/ConcatBufferAtVector ConcatenateIn1' */
  open_loop_sin_deriv_B.VectorConcatenate[0] = open_loop_sin_deriv_B.SineWave;

  /* DiscreteStateSpace: '<Root>/DirtyDerivative' */
  {
    rtb_DirtyDerivative = (open_loop_sin_deriv_P.DirtyDerivative_C[0])*
      open_loop_sin_deriv_DWork.DirtyDerivative_DSTATE[0]
      + (open_loop_sin_deriv_P.DirtyDerivative_C[1])*
      open_loop_sin_deriv_DWork.DirtyDerivative_DSTATE[1]
      + (open_loop_sin_deriv_P.DirtyDerivative_C[2])*
      open_loop_sin_deriv_DWork.DirtyDerivative_DSTATE[2]
      + (open_loop_sin_deriv_P.DirtyDerivative_C[3])*
      open_loop_sin_deriv_DWork.DirtyDerivative_DSTATE[3];
    rtb_DirtyDerivative += open_loop_sin_deriv_P.DirtyDerivative_D*
      open_loop_sin_deriv_B.SineWave;
  }

  /* Gain: '<Root>/Gain' */
  open_loop_sin_deriv_B.VectorConcatenate[1] = open_loop_sin_deriv_P.Gain_Gain *
    rtb_DirtyDerivative;

  /* Outport: '<Root>/Out1' */
  open_loop_sin_deriv_Y.Out1[0] = open_loop_sin_deriv_B.VectorConcatenate[0];
  open_loop_sin_deriv_Y.Out1[1] = open_loop_sin_deriv_B.VectorConcatenate[1];

  /* Update for DiscreteStateSpace: '<Root>/DirtyDerivative' */
  {
    real_T xnew[4];
    xnew[0] = (open_loop_sin_deriv_P.DirtyDerivative_A[0])*
      open_loop_sin_deriv_DWork.DirtyDerivative_DSTATE[0]
      + (open_loop_sin_deriv_P.DirtyDerivative_A[1])*
      open_loop_sin_deriv_DWork.DirtyDerivative_DSTATE[1]
      + (open_loop_sin_deriv_P.DirtyDerivative_A[2])*
      open_loop_sin_deriv_DWork.DirtyDerivative_DSTATE[2]
      + (open_loop_sin_deriv_P.DirtyDerivative_A[3])*
      open_loop_sin_deriv_DWork.DirtyDerivative_DSTATE[3];
    xnew[0] += (open_loop_sin_deriv_P.DirtyDerivative_B[0])*
      open_loop_sin_deriv_B.SineWave;
    xnew[1] = (open_loop_sin_deriv_P.DirtyDerivative_A[4])*
      open_loop_sin_deriv_DWork.DirtyDerivative_DSTATE[0]
      + (open_loop_sin_deriv_P.DirtyDerivative_A[5])*
      open_loop_sin_deriv_DWork.DirtyDerivative_DSTATE[1]
      + (open_loop_sin_deriv_P.DirtyDerivative_A[6])*
      open_loop_sin_deriv_DWork.DirtyDerivative_DSTATE[2]
      + (open_loop_sin_deriv_P.DirtyDerivative_A[7])*
      open_loop_sin_deriv_DWork.DirtyDerivative_DSTATE[3];
    xnew[1] += (open_loop_sin_deriv_P.DirtyDerivative_B[1])*
      open_loop_sin_deriv_B.SineWave;
    xnew[2] = (open_loop_sin_deriv_P.DirtyDerivative_A[8])*
      open_loop_sin_deriv_DWork.DirtyDerivative_DSTATE[0]
      + (open_loop_sin_deriv_P.DirtyDerivative_A[9])*
      open_loop_sin_deriv_DWork.DirtyDerivative_DSTATE[1]
      + (open_loop_sin_deriv_P.DirtyDerivative_A[10])*
      open_loop_sin_deriv_DWork.DirtyDerivative_DSTATE[2]
      + (open_loop_sin_deriv_P.DirtyDerivative_A[11])*
      open_loop_sin_deriv_DWork.DirtyDerivative_DSTATE[3];
    xnew[2] += (open_loop_sin_deriv_P.DirtyDerivative_B[2])*
      open_loop_sin_deriv_B.SineWave;
    xnew[3] = (open_loop_sin_deriv_P.DirtyDerivative_A[12])*
      open_loop_sin_deriv_DWork.DirtyDerivative_DSTATE[0]
      + (open_loop_sin_deriv_P.DirtyDerivative_A[13])*
      open_loop_sin_deriv_DWork.DirtyDerivative_DSTATE[1]
      + (open_loop_sin_deriv_P.DirtyDerivative_A[14])*
      open_loop_sin_deriv_DWork.DirtyDerivative_DSTATE[2]
      + (open_loop_sin_deriv_P.DirtyDerivative_A[15])*
      open_loop_sin_deriv_DWork.DirtyDerivative_DSTATE[3];
    xnew[3] += (open_loop_sin_deriv_P.DirtyDerivative_B[3])*
      open_loop_sin_deriv_B.SineWave;
    (void) memcpy(&open_loop_sin_deriv_DWork.DirtyDerivative_DSTATE[0], xnew,
                  sizeof(real_T)*4);
  }

  /* Update absolute time for base rate */
  /* The "clockTick0" counts the number of times the code of this task has
   * been executed. The absolute time is the multiplication of "clockTick0"
   * and "Timing.stepSize0". Size of "clockTick0" ensures timer will not
   * overflow during the application lifespan selected.
   */
  open_loop_sin_deriv_M->Timing.t[0] =
    (++open_loop_sin_deriv_M->Timing.clockTick0) *
    open_loop_sin_deriv_M->Timing.stepSize0;

  {
    /* Update absolute timer for sample time: [0.001s, 0.0s] */
    /* The "clockTick1" counts the number of times the code of this task has
     * been executed. The resolution of this integer timer is 0.001, which is the step size
     * of the task. Size of "clockTick1" ensures timer will not overflow during the
     * application lifespan selected.
     */
    open_loop_sin_deriv_M->Timing.clockTick1++;
  }
}

/* Model initialize function */
void open_loop_sin_deriv_initialize(void)
{
  /* Registration code */

  /* initialize real-time model */
  (void) memset((void *)open_loop_sin_deriv_M, 0,
                sizeof(RT_MODEL_open_loop_sin_deriv));

  {
    /* Setup solver object */
    rtsiSetSimTimeStepPtr(&open_loop_sin_deriv_M->solverInfo,
                          &open_loop_sin_deriv_M->Timing.simTimeStep);
    rtsiSetTPtr(&open_loop_sin_deriv_M->solverInfo, &rtmGetTPtr
                (open_loop_sin_deriv_M));
    rtsiSetStepSizePtr(&open_loop_sin_deriv_M->solverInfo,
                       &open_loop_sin_deriv_M->Timing.stepSize0);
    rtsiSetErrorStatusPtr(&open_loop_sin_deriv_M->solverInfo, ((const char_T **)
                           (&rtmGetErrorStatus(open_loop_sin_deriv_M))));
    rtsiSetRTModelPtr(&open_loop_sin_deriv_M->solverInfo, open_loop_sin_deriv_M);
  }

  rtsiSetSimTimeStep(&open_loop_sin_deriv_M->solverInfo, MAJOR_TIME_STEP);
  rtsiSetSolverName(&open_loop_sin_deriv_M->solverInfo,"FixedStepDiscrete");
  rtmSetTPtr(open_loop_sin_deriv_M, &open_loop_sin_deriv_M->Timing.tArray[0]);
  open_loop_sin_deriv_M->Timing.stepSize0 = 0.001;

  /* block I/O */
  (void) memset(((void *) &open_loop_sin_deriv_B), 0,
                sizeof(BlockIO_open_loop_sin_deriv));

  /* states (dwork) */
  (void) memset((void *)&open_loop_sin_deriv_DWork, 0,
                sizeof(D_Work_open_loop_sin_deriv));

  /* external outputs */
  (void) memset(&open_loop_sin_deriv_Y.Out1[0], 0,
                2U*sizeof(real_T));

  /* InitializeConditions for DiscreteStateSpace: '<Root>/DirtyDerivative' */
  open_loop_sin_deriv_DWork.DirtyDerivative_DSTATE[0] =
    open_loop_sin_deriv_P.DirtyDerivative_X0[0];
  open_loop_sin_deriv_DWork.DirtyDerivative_DSTATE[1] =
    open_loop_sin_deriv_P.DirtyDerivative_X0[1];
  open_loop_sin_deriv_DWork.DirtyDerivative_DSTATE[2] =
    open_loop_sin_deriv_P.DirtyDerivative_X0[2];
  open_loop_sin_deriv_DWork.DirtyDerivative_DSTATE[3] =
    open_loop_sin_deriv_P.DirtyDerivative_X0[3];
}

/* Model terminate function */
void open_loop_sin_deriv_terminate(void)
{
  /* (no terminate code required) */
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
