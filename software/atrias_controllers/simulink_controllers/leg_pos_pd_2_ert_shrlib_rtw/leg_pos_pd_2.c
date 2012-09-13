/*
 * File: leg_pos_pd_2.c
 *
 * Code generated for Simulink model 'leg_pos_pd_2'.
 *
 * Model version                  : 1.3
 * Simulink Coder version         : 8.2 (R2012a) 29-Dec-2011
 * TLC version                    : 8.2 (Jan 25 2012)
 * C/C++ source code generated on : Thu Sep 13 00:04:24 2012
 *
 * Target selection: ert_shrlib.tlc
 * Embedded hardware selection: 32-bit Generic
 * Emulation hardware selection:
 *    Differs from embedded hardware (MATLAB Host)
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#include "leg_pos_pd_2.h"
#include "leg_pos_pd_2_private.h"

/* Block states (auto storage) */
D_Work_leg_pos_pd_2 leg_pos_pd_2_DWork;

/* External inputs (root inport signals with auto storage) */
ExternalInputs_leg_pos_pd_2 leg_pos_pd_2_U;

/* External outputs (root outports fed by signals with auto storage) */
ExternalOutputs_leg_pos_pd_2 leg_pos_pd_2_Y;

/* Real-time model */
RT_MODEL_leg_pos_pd_2 leg_pos_pd_2_M_;
RT_MODEL_leg_pos_pd_2 *const leg_pos_pd_2_M = &leg_pos_pd_2_M_;

/* Model step function */
void leg_pos_pd_2_step(void)
{
  /* local block i/o variables */
  real_T rtb_Switch[2];
  real_T rtb_DirtyDerivative1;
  real_T rtb_GainLegAngleP;
  real_T rtb_CoordTranformation1_idx;
  real_T rtb_CoordTranformation1_idx_0;
  real_T rtb_CoordTransformation2_idx;
  real_T rtb_CoordTransformation2_idx_0;

  /* Gain: '<Root>/CoordTranformation1' incorporates:
   *  Inport: '<Root>/motorAngleA_morotAngleB'
   */
  rtb_CoordTranformation1_idx = leg_pos_pd_2_P.CoordTranformation1_Gain[0] *
    leg_pos_pd_2_U.motorAngleA_morotAngleB[0] +
    leg_pos_pd_2_P.CoordTranformation1_Gain[2] *
    leg_pos_pd_2_U.motorAngleA_morotAngleB[1];
  rtb_CoordTranformation1_idx_0 = leg_pos_pd_2_P.CoordTranformation1_Gain[1] *
    leg_pos_pd_2_U.motorAngleA_morotAngleB[0] +
    leg_pos_pd_2_P.CoordTranformation1_Gain[3] *
    leg_pos_pd_2_U.motorAngleA_morotAngleB[1];

  /* Sum: '<Root>/Sum2' incorporates:
   *  Inport: '<Root>/desiredLegAngle_desiredKneeAngle'
   */
  rtb_CoordTransformation2_idx = leg_pos_pd_2_U.desiredLegAngle_desiredKneeAngl
    [0] - rtb_CoordTranformation1_idx;
  rtb_CoordTransformation2_idx_0 =
    leg_pos_pd_2_U.desiredLegAngle_desiredKneeAngl[1] -
    rtb_CoordTranformation1_idx_0;

  /* Gain: '<Root>/GainLegAngleP' */
  rtb_GainLegAngleP = leg_pos_pd_2_P.GainLegAngleP_Gain *
    rtb_CoordTransformation2_idx;

  /* Switch: '<Root>/Switch' incorporates:
   *  Constant: '<Root>/Constant1'
   *  Gain: '<Root>/Gain1'
   */
  if (leg_pos_pd_2_P.Constant1_Value != 0.0) {
    rtb_Switch[0] = rtb_CoordTransformation2_idx;
    rtb_Switch[1] = rtb_CoordTransformation2_idx_0;
  } else {
    rtb_Switch[0] = leg_pos_pd_2_P.Gain1_Gain * rtb_CoordTranformation1_idx;
    rtb_Switch[1] = leg_pos_pd_2_P.Gain1_Gain * rtb_CoordTranformation1_idx_0;
  }

  /* End of Switch: '<Root>/Switch' */

  /* DiscreteStateSpace: '<Root>/DirtyDerivative' */
  {
    rtb_DirtyDerivative1 = (leg_pos_pd_2_P.DirtyDerivative_C[0])*
      leg_pos_pd_2_DWork.DirtyDerivative_DSTATE[0]
      + (leg_pos_pd_2_P.DirtyDerivative_C[1])*
      leg_pos_pd_2_DWork.DirtyDerivative_DSTATE[1]
      + (leg_pos_pd_2_P.DirtyDerivative_C[2])*
      leg_pos_pd_2_DWork.DirtyDerivative_DSTATE[2]
      + (leg_pos_pd_2_P.DirtyDerivative_C[3])*
      leg_pos_pd_2_DWork.DirtyDerivative_DSTATE[3];
    rtb_DirtyDerivative1 += leg_pos_pd_2_P.DirtyDerivative_D*rtb_Switch[0];
  }

  /* Sum: '<Root>/Sum' incorporates:
   *  Gain: '<Root>/GainLegAngleD'
   */
  rtb_CoordTranformation1_idx = leg_pos_pd_2_P.GainLegAngleD_Gain *
    rtb_DirtyDerivative1 + rtb_GainLegAngleP;

  /* Gain: '<Root>/GainKneeAngleP' */
  rtb_GainLegAngleP = leg_pos_pd_2_P.GainKneeAngleP_Gain *
    rtb_CoordTransformation2_idx_0;

  /* DiscreteStateSpace: '<Root>/DirtyDerivative1' */
  {
    rtb_DirtyDerivative1 = (leg_pos_pd_2_P.DirtyDerivative1_C[0])*
      leg_pos_pd_2_DWork.DirtyDerivative1_DSTATE[0]
      + (leg_pos_pd_2_P.DirtyDerivative1_C[1])*
      leg_pos_pd_2_DWork.DirtyDerivative1_DSTATE[1]
      + (leg_pos_pd_2_P.DirtyDerivative1_C[2])*
      leg_pos_pd_2_DWork.DirtyDerivative1_DSTATE[2]
      + (leg_pos_pd_2_P.DirtyDerivative1_C[3])*
      leg_pos_pd_2_DWork.DirtyDerivative1_DSTATE[3];
    rtb_DirtyDerivative1 += leg_pos_pd_2_P.DirtyDerivative1_D*rtb_Switch[1];
  }

  /* Sum: '<Root>/Sum1' incorporates:
   *  Gain: '<Root>/GainKneeAngleD'
   */
  rtb_CoordTranformation1_idx_0 = leg_pos_pd_2_P.GainKneeAngleD_Gain *
    rtb_DirtyDerivative1 + rtb_GainLegAngleP;

  /* Gain: '<Root>/CoordTransformation2' incorporates:
   *  Saturate: '<Root>/Saturation'
   */
  rtb_CoordTransformation2_idx = leg_pos_pd_2_P.CoordTransformation2_Gain[0] *
    rtb_CoordTranformation1_idx + leg_pos_pd_2_P.CoordTransformation2_Gain[2] *
    rtb_CoordTranformation1_idx_0;
  rtb_CoordTranformation1_idx_0 = leg_pos_pd_2_P.CoordTransformation2_Gain[1] *
    rtb_CoordTranformation1_idx + leg_pos_pd_2_P.CoordTransformation2_Gain[3] *
    rtb_CoordTranformation1_idx_0;

  /* Saturate: '<Root>/Saturation' */
  if (rtb_CoordTransformation2_idx >= leg_pos_pd_2_P.Saturation_UpperSat) {
    /* Outport: '<Root>/uA_uB' incorporates:
     *  Gain: '<Root>/CoordTransformation2'
     */
    leg_pos_pd_2_Y.uA_uB[0] = leg_pos_pd_2_P.Saturation_UpperSat;
  } else if (rtb_CoordTransformation2_idx <= leg_pos_pd_2_P.Saturation_LowerSat)
  {
    /* Outport: '<Root>/uA_uB' incorporates:
     *  Gain: '<Root>/CoordTransformation2'
     */
    leg_pos_pd_2_Y.uA_uB[0] = leg_pos_pd_2_P.Saturation_LowerSat;
  } else {
    /* Outport: '<Root>/uA_uB' incorporates:
     *  Gain: '<Root>/CoordTransformation2'
     */
    leg_pos_pd_2_Y.uA_uB[0] = rtb_CoordTransformation2_idx;
  }

  if (rtb_CoordTranformation1_idx_0 >= leg_pos_pd_2_P.Saturation_UpperSat) {
    /* Outport: '<Root>/uA_uB' incorporates:
     *  Gain: '<Root>/CoordTransformation2'
     */
    leg_pos_pd_2_Y.uA_uB[1] = leg_pos_pd_2_P.Saturation_UpperSat;
  } else if (rtb_CoordTranformation1_idx_0 <= leg_pos_pd_2_P.Saturation_LowerSat)
  {
    /* Outport: '<Root>/uA_uB' incorporates:
     *  Gain: '<Root>/CoordTransformation2'
     */
    leg_pos_pd_2_Y.uA_uB[1] = leg_pos_pd_2_P.Saturation_LowerSat;
  } else {
    /* Outport: '<Root>/uA_uB' incorporates:
     *  Gain: '<Root>/CoordTransformation2'
     */
    leg_pos_pd_2_Y.uA_uB[1] = rtb_CoordTranformation1_idx_0;
  }

  /* Update for DiscreteStateSpace: '<Root>/DirtyDerivative' */
  {
    real_T xnew[4];
    xnew[0] = (leg_pos_pd_2_P.DirtyDerivative_A[0])*
      leg_pos_pd_2_DWork.DirtyDerivative_DSTATE[0]
      + (leg_pos_pd_2_P.DirtyDerivative_A[1])*
      leg_pos_pd_2_DWork.DirtyDerivative_DSTATE[1]
      + (leg_pos_pd_2_P.DirtyDerivative_A[2])*
      leg_pos_pd_2_DWork.DirtyDerivative_DSTATE[2]
      + (leg_pos_pd_2_P.DirtyDerivative_A[3])*
      leg_pos_pd_2_DWork.DirtyDerivative_DSTATE[3];
    xnew[0] += (leg_pos_pd_2_P.DirtyDerivative_B[0])*rtb_Switch[0];
    xnew[1] = (leg_pos_pd_2_P.DirtyDerivative_A[4])*
      leg_pos_pd_2_DWork.DirtyDerivative_DSTATE[0]
      + (leg_pos_pd_2_P.DirtyDerivative_A[5])*
      leg_pos_pd_2_DWork.DirtyDerivative_DSTATE[1]
      + (leg_pos_pd_2_P.DirtyDerivative_A[6])*
      leg_pos_pd_2_DWork.DirtyDerivative_DSTATE[2]
      + (leg_pos_pd_2_P.DirtyDerivative_A[7])*
      leg_pos_pd_2_DWork.DirtyDerivative_DSTATE[3];
    xnew[1] += (leg_pos_pd_2_P.DirtyDerivative_B[1])*rtb_Switch[0];
    xnew[2] = (leg_pos_pd_2_P.DirtyDerivative_A[8])*
      leg_pos_pd_2_DWork.DirtyDerivative_DSTATE[0]
      + (leg_pos_pd_2_P.DirtyDerivative_A[9])*
      leg_pos_pd_2_DWork.DirtyDerivative_DSTATE[1]
      + (leg_pos_pd_2_P.DirtyDerivative_A[10])*
      leg_pos_pd_2_DWork.DirtyDerivative_DSTATE[2]
      + (leg_pos_pd_2_P.DirtyDerivative_A[11])*
      leg_pos_pd_2_DWork.DirtyDerivative_DSTATE[3];
    xnew[2] += (leg_pos_pd_2_P.DirtyDerivative_B[2])*rtb_Switch[0];
    xnew[3] = (leg_pos_pd_2_P.DirtyDerivative_A[12])*
      leg_pos_pd_2_DWork.DirtyDerivative_DSTATE[0]
      + (leg_pos_pd_2_P.DirtyDerivative_A[13])*
      leg_pos_pd_2_DWork.DirtyDerivative_DSTATE[1]
      + (leg_pos_pd_2_P.DirtyDerivative_A[14])*
      leg_pos_pd_2_DWork.DirtyDerivative_DSTATE[2]
      + (leg_pos_pd_2_P.DirtyDerivative_A[15])*
      leg_pos_pd_2_DWork.DirtyDerivative_DSTATE[3];
    xnew[3] += (leg_pos_pd_2_P.DirtyDerivative_B[3])*rtb_Switch[0];
    (void) memcpy(&leg_pos_pd_2_DWork.DirtyDerivative_DSTATE[0], xnew,
                  sizeof(real_T)*4);
  }

  /* Update for DiscreteStateSpace: '<Root>/DirtyDerivative1' */
  {
    real_T xnew[4];
    xnew[0] = (leg_pos_pd_2_P.DirtyDerivative1_A[0])*
      leg_pos_pd_2_DWork.DirtyDerivative1_DSTATE[0]
      + (leg_pos_pd_2_P.DirtyDerivative1_A[1])*
      leg_pos_pd_2_DWork.DirtyDerivative1_DSTATE[1]
      + (leg_pos_pd_2_P.DirtyDerivative1_A[2])*
      leg_pos_pd_2_DWork.DirtyDerivative1_DSTATE[2]
      + (leg_pos_pd_2_P.DirtyDerivative1_A[3])*
      leg_pos_pd_2_DWork.DirtyDerivative1_DSTATE[3];
    xnew[0] += (leg_pos_pd_2_P.DirtyDerivative1_B[0])*rtb_Switch[1];
    xnew[1] = (leg_pos_pd_2_P.DirtyDerivative1_A[4])*
      leg_pos_pd_2_DWork.DirtyDerivative1_DSTATE[0]
      + (leg_pos_pd_2_P.DirtyDerivative1_A[5])*
      leg_pos_pd_2_DWork.DirtyDerivative1_DSTATE[1]
      + (leg_pos_pd_2_P.DirtyDerivative1_A[6])*
      leg_pos_pd_2_DWork.DirtyDerivative1_DSTATE[2]
      + (leg_pos_pd_2_P.DirtyDerivative1_A[7])*
      leg_pos_pd_2_DWork.DirtyDerivative1_DSTATE[3];
    xnew[1] += (leg_pos_pd_2_P.DirtyDerivative1_B[1])*rtb_Switch[1];
    xnew[2] = (leg_pos_pd_2_P.DirtyDerivative1_A[8])*
      leg_pos_pd_2_DWork.DirtyDerivative1_DSTATE[0]
      + (leg_pos_pd_2_P.DirtyDerivative1_A[9])*
      leg_pos_pd_2_DWork.DirtyDerivative1_DSTATE[1]
      + (leg_pos_pd_2_P.DirtyDerivative1_A[10])*
      leg_pos_pd_2_DWork.DirtyDerivative1_DSTATE[2]
      + (leg_pos_pd_2_P.DirtyDerivative1_A[11])*
      leg_pos_pd_2_DWork.DirtyDerivative1_DSTATE[3];
    xnew[2] += (leg_pos_pd_2_P.DirtyDerivative1_B[2])*rtb_Switch[1];
    xnew[3] = (leg_pos_pd_2_P.DirtyDerivative1_A[12])*
      leg_pos_pd_2_DWork.DirtyDerivative1_DSTATE[0]
      + (leg_pos_pd_2_P.DirtyDerivative1_A[13])*
      leg_pos_pd_2_DWork.DirtyDerivative1_DSTATE[1]
      + (leg_pos_pd_2_P.DirtyDerivative1_A[14])*
      leg_pos_pd_2_DWork.DirtyDerivative1_DSTATE[2]
      + (leg_pos_pd_2_P.DirtyDerivative1_A[15])*
      leg_pos_pd_2_DWork.DirtyDerivative1_DSTATE[3];
    xnew[3] += (leg_pos_pd_2_P.DirtyDerivative1_B[3])*rtb_Switch[1];
    (void) memcpy(&leg_pos_pd_2_DWork.DirtyDerivative1_DSTATE[0], xnew,
                  sizeof(real_T)*4);
  }
}

/* Model initialize function */
void leg_pos_pd_2_initialize(void)
{
  /* Registration code */

  /* initialize error status */
  rtmSetErrorStatus(leg_pos_pd_2_M, (NULL));

  /* states (dwork) */
  (void) memset((void *)&leg_pos_pd_2_DWork, 0,
                sizeof(D_Work_leg_pos_pd_2));

  /* external inputs */
  (void) memset((void *)&leg_pos_pd_2_U, 0,
                sizeof(ExternalInputs_leg_pos_pd_2));

  /* external outputs */
  (void) memset(&leg_pos_pd_2_Y.uA_uB[0], 0,
                2U*sizeof(real_T));

  /* InitializeConditions for DiscreteStateSpace: '<Root>/DirtyDerivative' */
  leg_pos_pd_2_DWork.DirtyDerivative_DSTATE[0] =
    leg_pos_pd_2_P.DirtyDerivative_X0[0];
  leg_pos_pd_2_DWork.DirtyDerivative_DSTATE[1] =
    leg_pos_pd_2_P.DirtyDerivative_X0[1];
  leg_pos_pd_2_DWork.DirtyDerivative_DSTATE[2] =
    leg_pos_pd_2_P.DirtyDerivative_X0[2];
  leg_pos_pd_2_DWork.DirtyDerivative_DSTATE[3] =
    leg_pos_pd_2_P.DirtyDerivative_X0[3];

  /* InitializeConditions for DiscreteStateSpace: '<Root>/DirtyDerivative1' */
  leg_pos_pd_2_DWork.DirtyDerivative1_DSTATE[0] =
    leg_pos_pd_2_P.DirtyDerivative1_X0[0];
  leg_pos_pd_2_DWork.DirtyDerivative1_DSTATE[1] =
    leg_pos_pd_2_P.DirtyDerivative1_X0[1];
  leg_pos_pd_2_DWork.DirtyDerivative1_DSTATE[2] =
    leg_pos_pd_2_P.DirtyDerivative1_X0[2];
  leg_pos_pd_2_DWork.DirtyDerivative1_DSTATE[3] =
    leg_pos_pd_2_P.DirtyDerivative1_X0[3];
}

/* Model terminate function */
void leg_pos_pd_2_terminate(void)
{
  /* (no terminate code required) */
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
