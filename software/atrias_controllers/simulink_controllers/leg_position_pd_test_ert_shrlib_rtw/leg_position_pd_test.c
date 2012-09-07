/*
 * File: leg_position_pd_test.c
 *
 * Code generated for Simulink model 'leg_position_pd_test'.
 *
 * Model version                  : 1.24
 * Simulink Coder version         : 8.2 (R2012a) 29-Dec-2011
 * TLC version                    : 8.2 (Jan 25 2012)
 * C/C++ source code generated on : Thu Sep  6 22:36:51 2012
 *
 * Target selection: ert_shrlib.tlc
 * Embedded hardware selection: 32-bit Generic
 * Emulation hardware selection:
 *    Differs from embedded hardware (MATLAB Host)
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#include "leg_position_pd_test.h"
#include "leg_position_pd_test_private.h"

/* Block states (auto storage) */
D_Work_leg_position_pd_test leg_position_pd_test_DWork;

/* External inputs (root inport signals with auto storage) */
ExternalInputs_leg_position_pd_ leg_position_pd_test_U;

/* External outputs (root outports fed by signals with auto storage) */
ExternalOutputs_leg_position_pd leg_position_pd_test_Y;

/* Real-time model */
RT_MODEL_leg_position_pd_test leg_position_pd_test_M_;
RT_MODEL_leg_position_pd_test *const leg_position_pd_test_M =
  &leg_position_pd_test_M_;

/* Model step function */
void leg_position_pd_test_step(void)
{
  /* local block i/o variables */
  real_T rtb_positionErrors[2];
  real_T rtb_deKA;
  real_T rtb_LegAngleP;
  real_T rtb_uLA_uKA_idx;
  real_T unnamed_idx;

  /* Sum: '<Root>/Sum2' incorporates:
   *  Gain: '<Root>/CoordTranformation1'
   *  Inport: '<Root>/desiredAnglesLA_KA'
   *  Inport: '<Root>/motorAnglesAB'
   */
  rtb_positionErrors[0] = leg_position_pd_test_U.desiredAnglesLA_KA[0] -
    (leg_position_pd_test_P.CoordTranformation1_Gain[0] *
     leg_position_pd_test_U.motorAnglesAB[0] +
     leg_position_pd_test_P.CoordTranformation1_Gain[2] *
     leg_position_pd_test_U.motorAnglesAB[1]);
  rtb_positionErrors[1] = leg_position_pd_test_U.desiredAnglesLA_KA[1] -
    (leg_position_pd_test_P.CoordTranformation1_Gain[1] *
     leg_position_pd_test_U.motorAnglesAB[0] +
     leg_position_pd_test_P.CoordTranformation1_Gain[3] *
     leg_position_pd_test_U.motorAnglesAB[1]);

  /* Gain: '<Root>/LegAngleP' */
  rtb_LegAngleP = leg_position_pd_test_P.LegAngleP_Gain * rtb_positionErrors[0];

  /* DiscreteStateSpace: '<Root>/DirtyDerivativeLA' */
  {
    rtb_deKA = (leg_position_pd_test_P.DirtyDerivativeLA_C[0])*
      leg_position_pd_test_DWork.DirtyDerivativeLA_DSTATE[0]
      + (leg_position_pd_test_P.DirtyDerivativeLA_C[1])*
      leg_position_pd_test_DWork.DirtyDerivativeLA_DSTATE[1]
      + (leg_position_pd_test_P.DirtyDerivativeLA_C[2])*
      leg_position_pd_test_DWork.DirtyDerivativeLA_DSTATE[2]
      + (leg_position_pd_test_P.DirtyDerivativeLA_C[3])*
      leg_position_pd_test_DWork.DirtyDerivativeLA_DSTATE[3];
    rtb_deKA += leg_position_pd_test_P.DirtyDerivativeLA_D*rtb_positionErrors[0];
  }

  /* Sum: '<Root>/Sum' incorporates:
   *  Gain: '<Root>/LegAngleD'
   */
  rtb_uLA_uKA_idx = leg_position_pd_test_P.LegAngleD_Gain * rtb_deKA +
    rtb_LegAngleP;

  /* Gain: '<Root>/KneeAngleP' */
  rtb_LegAngleP = leg_position_pd_test_P.KneeAngleP_Gain * rtb_positionErrors[1];

  /* DiscreteStateSpace: '<Root>/DirtyDerivativeKA' */
  {
    rtb_deKA = (leg_position_pd_test_P.DirtyDerivativeKA_C[0])*
      leg_position_pd_test_DWork.DirtyDerivativeKA_DSTATE[0]
      + (leg_position_pd_test_P.DirtyDerivativeKA_C[1])*
      leg_position_pd_test_DWork.DirtyDerivativeKA_DSTATE[1]
      + (leg_position_pd_test_P.DirtyDerivativeKA_C[2])*
      leg_position_pd_test_DWork.DirtyDerivativeKA_DSTATE[2]
      + (leg_position_pd_test_P.DirtyDerivativeKA_C[3])*
      leg_position_pd_test_DWork.DirtyDerivativeKA_DSTATE[3];
    rtb_deKA += leg_position_pd_test_P.DirtyDerivativeKA_D*rtb_positionErrors[1];
  }

  /* Sum: '<Root>/Sum1' incorporates:
   *  Gain: '<Root>/KneeAngleD'
   */
  rtb_LegAngleP += leg_position_pd_test_P.KneeAngleD_Gain * rtb_deKA;

  /* Gain: '<Root>/CoordTransformation2' incorporates:
   *  Saturate: '<Root>/Saturation'
   */
  unnamed_idx = leg_position_pd_test_P.CoordTransformation2_Gain[0] *
    rtb_uLA_uKA_idx + leg_position_pd_test_P.CoordTransformation2_Gain[2] *
    rtb_LegAngleP;
  rtb_LegAngleP = leg_position_pd_test_P.CoordTransformation2_Gain[1] *
    rtb_uLA_uKA_idx + leg_position_pd_test_P.CoordTransformation2_Gain[3] *
    rtb_LegAngleP;

  /* Saturate: '<Root>/Saturation' */
  if (unnamed_idx >= leg_position_pd_test_P.Saturation_UpperSat) {
    /* Outport: '<Root>/uA_uB' incorporates:
     *  Gain: '<Root>/CoordTransformation2'
     */
    leg_position_pd_test_Y.uA_uB[0] = leg_position_pd_test_P.Saturation_UpperSat;
  } else if (unnamed_idx <= leg_position_pd_test_P.Saturation_LowerSat) {
    /* Outport: '<Root>/uA_uB' incorporates:
     *  Gain: '<Root>/CoordTransformation2'
     */
    leg_position_pd_test_Y.uA_uB[0] = leg_position_pd_test_P.Saturation_LowerSat;
  } else {
    /* Outport: '<Root>/uA_uB' incorporates:
     *  Gain: '<Root>/CoordTransformation2'
     */
    leg_position_pd_test_Y.uA_uB[0] = unnamed_idx;
  }

  if (rtb_LegAngleP >= leg_position_pd_test_P.Saturation_UpperSat) {
    /* Outport: '<Root>/uA_uB' incorporates:
     *  Gain: '<Root>/CoordTransformation2'
     */
    leg_position_pd_test_Y.uA_uB[1] = leg_position_pd_test_P.Saturation_UpperSat;
  } else if (rtb_LegAngleP <= leg_position_pd_test_P.Saturation_LowerSat) {
    /* Outport: '<Root>/uA_uB' incorporates:
     *  Gain: '<Root>/CoordTransformation2'
     */
    leg_position_pd_test_Y.uA_uB[1] = leg_position_pd_test_P.Saturation_LowerSat;
  } else {
    /* Outport: '<Root>/uA_uB' incorporates:
     *  Gain: '<Root>/CoordTransformation2'
     */
    leg_position_pd_test_Y.uA_uB[1] = rtb_LegAngleP;
  }

  /* Update for DiscreteStateSpace: '<Root>/DirtyDerivativeLA' */
  {
    real_T xnew[4];
    xnew[0] = (leg_position_pd_test_P.DirtyDerivativeLA_A[0])*
      leg_position_pd_test_DWork.DirtyDerivativeLA_DSTATE[0]
      + (leg_position_pd_test_P.DirtyDerivativeLA_A[1])*
      leg_position_pd_test_DWork.DirtyDerivativeLA_DSTATE[1]
      + (leg_position_pd_test_P.DirtyDerivativeLA_A[2])*
      leg_position_pd_test_DWork.DirtyDerivativeLA_DSTATE[2]
      + (leg_position_pd_test_P.DirtyDerivativeLA_A[3])*
      leg_position_pd_test_DWork.DirtyDerivativeLA_DSTATE[3];
    xnew[0] += (leg_position_pd_test_P.DirtyDerivativeLA_B[0])*
      rtb_positionErrors[0];
    xnew[1] = (leg_position_pd_test_P.DirtyDerivativeLA_A[4])*
      leg_position_pd_test_DWork.DirtyDerivativeLA_DSTATE[0]
      + (leg_position_pd_test_P.DirtyDerivativeLA_A[5])*
      leg_position_pd_test_DWork.DirtyDerivativeLA_DSTATE[1]
      + (leg_position_pd_test_P.DirtyDerivativeLA_A[6])*
      leg_position_pd_test_DWork.DirtyDerivativeLA_DSTATE[2]
      + (leg_position_pd_test_P.DirtyDerivativeLA_A[7])*
      leg_position_pd_test_DWork.DirtyDerivativeLA_DSTATE[3];
    xnew[1] += (leg_position_pd_test_P.DirtyDerivativeLA_B[1])*
      rtb_positionErrors[0];
    xnew[2] = (leg_position_pd_test_P.DirtyDerivativeLA_A[8])*
      leg_position_pd_test_DWork.DirtyDerivativeLA_DSTATE[0]
      + (leg_position_pd_test_P.DirtyDerivativeLA_A[9])*
      leg_position_pd_test_DWork.DirtyDerivativeLA_DSTATE[1]
      + (leg_position_pd_test_P.DirtyDerivativeLA_A[10])*
      leg_position_pd_test_DWork.DirtyDerivativeLA_DSTATE[2]
      + (leg_position_pd_test_P.DirtyDerivativeLA_A[11])*
      leg_position_pd_test_DWork.DirtyDerivativeLA_DSTATE[3];
    xnew[2] += (leg_position_pd_test_P.DirtyDerivativeLA_B[2])*
      rtb_positionErrors[0];
    xnew[3] = (leg_position_pd_test_P.DirtyDerivativeLA_A[12])*
      leg_position_pd_test_DWork.DirtyDerivativeLA_DSTATE[0]
      + (leg_position_pd_test_P.DirtyDerivativeLA_A[13])*
      leg_position_pd_test_DWork.DirtyDerivativeLA_DSTATE[1]
      + (leg_position_pd_test_P.DirtyDerivativeLA_A[14])*
      leg_position_pd_test_DWork.DirtyDerivativeLA_DSTATE[2]
      + (leg_position_pd_test_P.DirtyDerivativeLA_A[15])*
      leg_position_pd_test_DWork.DirtyDerivativeLA_DSTATE[3];
    xnew[3] += (leg_position_pd_test_P.DirtyDerivativeLA_B[3])*
      rtb_positionErrors[0];
    (void) memcpy(&leg_position_pd_test_DWork.DirtyDerivativeLA_DSTATE[0], xnew,
                  sizeof(real_T)*4);
  }

  /* Update for DiscreteStateSpace: '<Root>/DirtyDerivativeKA' */
  {
    real_T xnew[4];
    xnew[0] = (leg_position_pd_test_P.DirtyDerivativeKA_A[0])*
      leg_position_pd_test_DWork.DirtyDerivativeKA_DSTATE[0]
      + (leg_position_pd_test_P.DirtyDerivativeKA_A[1])*
      leg_position_pd_test_DWork.DirtyDerivativeKA_DSTATE[1]
      + (leg_position_pd_test_P.DirtyDerivativeKA_A[2])*
      leg_position_pd_test_DWork.DirtyDerivativeKA_DSTATE[2]
      + (leg_position_pd_test_P.DirtyDerivativeKA_A[3])*
      leg_position_pd_test_DWork.DirtyDerivativeKA_DSTATE[3];
    xnew[0] += (leg_position_pd_test_P.DirtyDerivativeKA_B[0])*
      rtb_positionErrors[1];
    xnew[1] = (leg_position_pd_test_P.DirtyDerivativeKA_A[4])*
      leg_position_pd_test_DWork.DirtyDerivativeKA_DSTATE[0]
      + (leg_position_pd_test_P.DirtyDerivativeKA_A[5])*
      leg_position_pd_test_DWork.DirtyDerivativeKA_DSTATE[1]
      + (leg_position_pd_test_P.DirtyDerivativeKA_A[6])*
      leg_position_pd_test_DWork.DirtyDerivativeKA_DSTATE[2]
      + (leg_position_pd_test_P.DirtyDerivativeKA_A[7])*
      leg_position_pd_test_DWork.DirtyDerivativeKA_DSTATE[3];
    xnew[1] += (leg_position_pd_test_P.DirtyDerivativeKA_B[1])*
      rtb_positionErrors[1];
    xnew[2] = (leg_position_pd_test_P.DirtyDerivativeKA_A[8])*
      leg_position_pd_test_DWork.DirtyDerivativeKA_DSTATE[0]
      + (leg_position_pd_test_P.DirtyDerivativeKA_A[9])*
      leg_position_pd_test_DWork.DirtyDerivativeKA_DSTATE[1]
      + (leg_position_pd_test_P.DirtyDerivativeKA_A[10])*
      leg_position_pd_test_DWork.DirtyDerivativeKA_DSTATE[2]
      + (leg_position_pd_test_P.DirtyDerivativeKA_A[11])*
      leg_position_pd_test_DWork.DirtyDerivativeKA_DSTATE[3];
    xnew[2] += (leg_position_pd_test_P.DirtyDerivativeKA_B[2])*
      rtb_positionErrors[1];
    xnew[3] = (leg_position_pd_test_P.DirtyDerivativeKA_A[12])*
      leg_position_pd_test_DWork.DirtyDerivativeKA_DSTATE[0]
      + (leg_position_pd_test_P.DirtyDerivativeKA_A[13])*
      leg_position_pd_test_DWork.DirtyDerivativeKA_DSTATE[1]
      + (leg_position_pd_test_P.DirtyDerivativeKA_A[14])*
      leg_position_pd_test_DWork.DirtyDerivativeKA_DSTATE[2]
      + (leg_position_pd_test_P.DirtyDerivativeKA_A[15])*
      leg_position_pd_test_DWork.DirtyDerivativeKA_DSTATE[3];
    xnew[3] += (leg_position_pd_test_P.DirtyDerivativeKA_B[3])*
      rtb_positionErrors[1];
    (void) memcpy(&leg_position_pd_test_DWork.DirtyDerivativeKA_DSTATE[0], xnew,
                  sizeof(real_T)*4);
  }
}

/* Model initialize function */
void leg_position_pd_test_initialize(void)
{
  /* Registration code */

  /* initialize error status */
  rtmSetErrorStatus(leg_position_pd_test_M, (NULL));

  /* states (dwork) */
  (void) memset((void *)&leg_position_pd_test_DWork, 0,
                sizeof(D_Work_leg_position_pd_test));

  /* external inputs */
  (void) memset((void *)&leg_position_pd_test_U, 0,
                sizeof(ExternalInputs_leg_position_pd_));

  /* external outputs */
  (void) memset(&leg_position_pd_test_Y.uA_uB[0], 0,
                2U*sizeof(real_T));

  /* InitializeConditions for DiscreteStateSpace: '<Root>/DirtyDerivativeLA' */
  leg_position_pd_test_DWork.DirtyDerivativeLA_DSTATE[0] =
    leg_position_pd_test_P.DirtyDerivativeLA_X0[0];
  leg_position_pd_test_DWork.DirtyDerivativeLA_DSTATE[1] =
    leg_position_pd_test_P.DirtyDerivativeLA_X0[1];
  leg_position_pd_test_DWork.DirtyDerivativeLA_DSTATE[2] =
    leg_position_pd_test_P.DirtyDerivativeLA_X0[2];
  leg_position_pd_test_DWork.DirtyDerivativeLA_DSTATE[3] =
    leg_position_pd_test_P.DirtyDerivativeLA_X0[3];

  /* InitializeConditions for DiscreteStateSpace: '<Root>/DirtyDerivativeKA' */
  leg_position_pd_test_DWork.DirtyDerivativeKA_DSTATE[0] =
    leg_position_pd_test_P.DirtyDerivativeKA_X0[0];
  leg_position_pd_test_DWork.DirtyDerivativeKA_DSTATE[1] =
    leg_position_pd_test_P.DirtyDerivativeKA_X0[1];
  leg_position_pd_test_DWork.DirtyDerivativeKA_DSTATE[2] =
    leg_position_pd_test_P.DirtyDerivativeKA_X0[2];
  leg_position_pd_test_DWork.DirtyDerivativeKA_DSTATE[3] =
    leg_position_pd_test_P.DirtyDerivativeKA_X0[3];
}

/* Model terminate function */
void leg_position_pd_test_terminate(void)
{
  /* (no terminate code required) */
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
