/*
 * File: posing_controller_v2.c
 *
 * Code generated for Simulink model 'posing_controller_v2'.
 *
 * Model version                  : 1.29
 * Simulink Coder version         : 8.2 (R2012a) 29-Dec-2011
 * TLC version                    : 8.2 (Jan 25 2012)
 * C/C++ source code generated on : Mon Sep 17 21:15:46 2012
 *
 * Target selection: ert_shrlib.tlc
 * Embedded hardware selection: 32-bit Generic
 * Emulation hardware selection:
 *    Differs from embedded hardware (MATLAB Host)
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#include "posing_controller_v2.h"
#include "posing_controller_v2_private.h"

/* Block states (auto storage) */
D_Work_posing_controller_v2 posing_controller_v2_DWork;

/* External inputs (root inport signals with auto storage) */
ExternalInputs_posing_controlle posing_controller_v2_U;

/* External outputs (root outports fed by signals with auto storage) */
ExternalOutputs_posing_controll posing_controller_v2_Y;

/* Real-time model */
RT_MODEL_posing_controller_v2 posing_controller_v2_M_;
RT_MODEL_posing_controller_v2 *const posing_controller_v2_M =
  &posing_controller_v2_M_;
real_T rt_powd_snf(real_T u0, real_T u1)
{
  real_T y;
  real_T tmp;
  real_T tmp_0;
  if (rtIsNaN(u0) || rtIsNaN(u1)) {
    y = (rtNaN);
  } else {
    tmp = fabs(u0);
    tmp_0 = fabs(u1);
    if (rtIsInf(u1)) {
      if (tmp == 1.0) {
        y = (rtNaN);
      } else if (tmp > 1.0) {
        if (u1 > 0.0) {
          y = (rtInf);
        } else {
          y = 0.0;
        }
      } else if (u1 > 0.0) {
        y = 0.0;
      } else {
        y = (rtInf);
      }
    } else if (tmp_0 == 0.0) {
      y = 1.0;
    } else if (tmp_0 == 1.0) {
      if (u1 > 0.0) {
        y = u0;
      } else {
        y = 1.0 / u0;
      }
    } else if (u1 == 2.0) {
      y = u0 * u0;
    } else if ((u1 == 0.5) && (u0 >= 0.0)) {
      y = sqrt(u0);
    } else if ((u0 < 0.0) && (u1 > floor(u1))) {
      y = (rtNaN);
    } else {
      y = pow(u0, u1);
    }
  }

  return y;
}

/* Model step function */
void posing_controller_v2_step(void)
{
  /* local block i/o variables */
  real_T rtb_VectorConcatenate[26];
  real_T rtb_q_um[13];
  real_T Kp[36];
  real_T Kd[36];
  real_T rtb_u[6];
  real_T rtb_y[6];
  real_T rtb_dy[6];
  real_T rtb_DiscreteFilter;
  real_T DiscreteFilter_tmp;
  real_T DiscreteFilter1_tmp;
  real_T DiscreteFilter2_tmp;
  real_T DiscreteFilter3_tmp;
  real_T DiscreteFilter4_tmp;
  real_T DiscreteFilter5_tmp;
  int32_T i;
  real_T Kp_0[36];
  real_T Kp_1[6];
  int32_T i_0;
  real_T Kd_0[6];

  /* MATLAB Function: '<Root>/coordXfm' incorporates:
   *  Inport: '<Root>/q_osu'
   */
  /* MATLAB Function 'coordXfm': '<S3>:1' */
  /*  */
  /*  lLeg: */
  /*    hip: */
  /*      legBodyAngle */
  /*    halfA: */
  /*      legAngle */
  /*      motorAngle */
  /*    halfB: */
  /*      legAngle */
  /*      motorAngle */
  /*  */
  /*  rLeg: */
  /*    hip: */
  /*      legBodyAngle */
  /*    halfA: */
  /*      legAngle */
  /*      motorAngle */
  /*    halfB: */
  /*      legAngle */
  /*      motorAngle */
  /*  */
  /*  position: */
  /*    xPosition */
  /*    yPosition */
  /*    zPosition */
  /*    bodyPitch */
  /* '<S3>:1:29' */
  /* '<S3>:1:30' */
  /* '<S3>:1:31' */
  /* '<S3>:1:32' */
  /* '<S3>:1:33' */
  /* '<S3>:1:34' */
  /* '<S3>:1:35' */
  /* '<S3>:1:36' */
  /* '<S3>:1:37' */
  /* '<S3>:1:38' */
  /* '<S3>:1:39' */
  /* '<S3>:1:41' */
  /* '<S3>:1:42' */
  /* '<S3>:1:44' */
  /* '<S3>:1:46' */
  /* '<S3>:1:47' */
  /* '<S3>:1:48' */
  /* '<S3>:1:49' */
  /* '<S3>:1:50' */
  /* '<S3>:1:51' */
  /* '<S3>:1:53' */
  /* '<S3>:1:54' */
  /* '<S3>:1:55' */
  /* '<S3>:1:56' */
  /* '<S3>:1:57' */
  /* '<S3>:1:59' */
  rtb_q_um[0] = -posing_controller_v2_U.q_osu[10];
  rtb_q_um[1] = posing_controller_v2_U.q_osu[12];
  rtb_q_um[2] = posing_controller_v2_U.q_osu[13] + 1.5707963267948966;
  rtb_q_um[3] = posing_controller_v2_U.q_osu[1] + 1.5707963267948966;
  rtb_q_um[4] = posing_controller_v2_U.q_osu[3] + 1.5707963267948966;
  rtb_q_um[5] = posing_controller_v2_U.q_osu[2] + 1.5707963267948966;
  rtb_q_um[6] = posing_controller_v2_U.q_osu[4] + 1.5707963267948966;
  rtb_q_um[7] = -(posing_controller_v2_U.q_osu[0] - 4.71238898038469);
  rtb_q_um[8] = posing_controller_v2_U.q_osu[6] + 1.5707963267948966;
  rtb_q_um[9] = posing_controller_v2_U.q_osu[8] + 1.5707963267948966;
  rtb_q_um[10] = posing_controller_v2_U.q_osu[7] + 1.5707963267948966;
  rtb_q_um[11] = posing_controller_v2_U.q_osu[9] + 1.5707963267948966;
  rtb_q_um[12] = posing_controller_v2_U.q_osu[5] - 4.71238898038469;

  /* SignalConversion: '<Root>/ConcatBufferAtVector ConcatenateIn1' */
  memcpy(&rtb_VectorConcatenate[0], &rtb_q_um[0], 13U * sizeof(real_T));

  /* DiscreteStateSpace: '<S1>/DirtyDerivative' */
  {
    rtb_VectorConcatenate[13] = (posing_controller_v2_P.DirtyDerivative_C[0])*
      posing_controller_v2_DWork.DirtyDerivative_DSTATE[0]
      + (posing_controller_v2_P.DirtyDerivative_C[1])*
      posing_controller_v2_DWork.DirtyDerivative_DSTATE[1]
      + (posing_controller_v2_P.DirtyDerivative_C[2])*
      posing_controller_v2_DWork.DirtyDerivative_DSTATE[2]
      + (posing_controller_v2_P.DirtyDerivative_C[3])*
      posing_controller_v2_DWork.DirtyDerivative_DSTATE[3];
    rtb_VectorConcatenate[13] += posing_controller_v2_P.DirtyDerivative_D*
      rtb_q_um[0];
  }

  /* DiscreteStateSpace: '<S1>/DirtyDerivative1' */
  {
    rtb_VectorConcatenate[14] = (posing_controller_v2_P.DirtyDerivative1_C[0])*
      posing_controller_v2_DWork.DirtyDerivative1_DSTATE[0]
      + (posing_controller_v2_P.DirtyDerivative1_C[1])*
      posing_controller_v2_DWork.DirtyDerivative1_DSTATE[1]
      + (posing_controller_v2_P.DirtyDerivative1_C[2])*
      posing_controller_v2_DWork.DirtyDerivative1_DSTATE[2]
      + (posing_controller_v2_P.DirtyDerivative1_C[3])*
      posing_controller_v2_DWork.DirtyDerivative1_DSTATE[3];
    rtb_VectorConcatenate[14] += posing_controller_v2_P.DirtyDerivative1_D*
      rtb_q_um[1];
  }

  /* DiscreteStateSpace: '<S1>/DirtyDerivative2' */
  {
    rtb_VectorConcatenate[15] = (posing_controller_v2_P.DirtyDerivative2_C[0])*
      posing_controller_v2_DWork.DirtyDerivative2_DSTATE[0]
      + (posing_controller_v2_P.DirtyDerivative2_C[1])*
      posing_controller_v2_DWork.DirtyDerivative2_DSTATE[1]
      + (posing_controller_v2_P.DirtyDerivative2_C[2])*
      posing_controller_v2_DWork.DirtyDerivative2_DSTATE[2]
      + (posing_controller_v2_P.DirtyDerivative2_C[3])*
      posing_controller_v2_DWork.DirtyDerivative2_DSTATE[3];
    rtb_VectorConcatenate[15] += posing_controller_v2_P.DirtyDerivative2_D*
      rtb_q_um[2];
  }

  /* DiscreteStateSpace: '<S1>/DirtyDerivative3' */
  {
    rtb_VectorConcatenate[16] = (posing_controller_v2_P.DirtyDerivative3_C[0])*
      posing_controller_v2_DWork.DirtyDerivative3_DSTATE[0]
      + (posing_controller_v2_P.DirtyDerivative3_C[1])*
      posing_controller_v2_DWork.DirtyDerivative3_DSTATE[1]
      + (posing_controller_v2_P.DirtyDerivative3_C[2])*
      posing_controller_v2_DWork.DirtyDerivative3_DSTATE[2]
      + (posing_controller_v2_P.DirtyDerivative3_C[3])*
      posing_controller_v2_DWork.DirtyDerivative3_DSTATE[3];
    rtb_VectorConcatenate[16] += posing_controller_v2_P.DirtyDerivative3_D*
      rtb_q_um[3];
  }

  /* DiscreteStateSpace: '<S1>/DirtyDerivative4' */
  {
    rtb_VectorConcatenate[17] = (posing_controller_v2_P.DirtyDerivative4_C[0])*
      posing_controller_v2_DWork.DirtyDerivative4_DSTATE[0]
      + (posing_controller_v2_P.DirtyDerivative4_C[1])*
      posing_controller_v2_DWork.DirtyDerivative4_DSTATE[1]
      + (posing_controller_v2_P.DirtyDerivative4_C[2])*
      posing_controller_v2_DWork.DirtyDerivative4_DSTATE[2]
      + (posing_controller_v2_P.DirtyDerivative4_C[3])*
      posing_controller_v2_DWork.DirtyDerivative4_DSTATE[3];
    rtb_VectorConcatenate[17] += posing_controller_v2_P.DirtyDerivative4_D*
      rtb_q_um[4];
  }

  /* DiscreteStateSpace: '<S1>/DirtyDerivative5' */
  {
    rtb_VectorConcatenate[18] = (posing_controller_v2_P.DirtyDerivative5_C[0])*
      posing_controller_v2_DWork.DirtyDerivative5_DSTATE[0]
      + (posing_controller_v2_P.DirtyDerivative5_C[1])*
      posing_controller_v2_DWork.DirtyDerivative5_DSTATE[1]
      + (posing_controller_v2_P.DirtyDerivative5_C[2])*
      posing_controller_v2_DWork.DirtyDerivative5_DSTATE[2]
      + (posing_controller_v2_P.DirtyDerivative5_C[3])*
      posing_controller_v2_DWork.DirtyDerivative5_DSTATE[3];
    rtb_VectorConcatenate[18] += posing_controller_v2_P.DirtyDerivative5_D*
      rtb_q_um[5];
  }

  /* DiscreteStateSpace: '<S1>/DirtyDerivative6' */
  {
    rtb_VectorConcatenate[19] = (posing_controller_v2_P.DirtyDerivative6_C[0])*
      posing_controller_v2_DWork.DirtyDerivative6_DSTATE[0]
      + (posing_controller_v2_P.DirtyDerivative6_C[1])*
      posing_controller_v2_DWork.DirtyDerivative6_DSTATE[1]
      + (posing_controller_v2_P.DirtyDerivative6_C[2])*
      posing_controller_v2_DWork.DirtyDerivative6_DSTATE[2]
      + (posing_controller_v2_P.DirtyDerivative6_C[3])*
      posing_controller_v2_DWork.DirtyDerivative6_DSTATE[3];
    rtb_VectorConcatenate[19] += posing_controller_v2_P.DirtyDerivative6_D*
      rtb_q_um[6];
  }

  /* DiscreteStateSpace: '<S1>/DirtyDerivative7' */
  {
    rtb_VectorConcatenate[20] = (posing_controller_v2_P.DirtyDerivative7_C[0])*
      posing_controller_v2_DWork.DirtyDerivative7_DSTATE[0]
      + (posing_controller_v2_P.DirtyDerivative7_C[1])*
      posing_controller_v2_DWork.DirtyDerivative7_DSTATE[1]
      + (posing_controller_v2_P.DirtyDerivative7_C[2])*
      posing_controller_v2_DWork.DirtyDerivative7_DSTATE[2]
      + (posing_controller_v2_P.DirtyDerivative7_C[3])*
      posing_controller_v2_DWork.DirtyDerivative7_DSTATE[3];
    rtb_VectorConcatenate[20] += posing_controller_v2_P.DirtyDerivative7_D*
      rtb_q_um[7];
  }

  /* DiscreteStateSpace: '<S1>/DirtyDerivative8' */
  {
    rtb_VectorConcatenate[21] = (posing_controller_v2_P.DirtyDerivative8_C[0])*
      posing_controller_v2_DWork.DirtyDerivative8_DSTATE[0]
      + (posing_controller_v2_P.DirtyDerivative8_C[1])*
      posing_controller_v2_DWork.DirtyDerivative8_DSTATE[1]
      + (posing_controller_v2_P.DirtyDerivative8_C[2])*
      posing_controller_v2_DWork.DirtyDerivative8_DSTATE[2]
      + (posing_controller_v2_P.DirtyDerivative8_C[3])*
      posing_controller_v2_DWork.DirtyDerivative8_DSTATE[3];
    rtb_VectorConcatenate[21] += posing_controller_v2_P.DirtyDerivative8_D*
      rtb_q_um[8];
  }

  /* DiscreteStateSpace: '<S1>/DirtyDerivative9' */
  {
    rtb_VectorConcatenate[22] = (posing_controller_v2_P.DirtyDerivative9_C[0])*
      posing_controller_v2_DWork.DirtyDerivative9_DSTATE[0]
      + (posing_controller_v2_P.DirtyDerivative9_C[1])*
      posing_controller_v2_DWork.DirtyDerivative9_DSTATE[1]
      + (posing_controller_v2_P.DirtyDerivative9_C[2])*
      posing_controller_v2_DWork.DirtyDerivative9_DSTATE[2]
      + (posing_controller_v2_P.DirtyDerivative9_C[3])*
      posing_controller_v2_DWork.DirtyDerivative9_DSTATE[3];
    rtb_VectorConcatenate[22] += posing_controller_v2_P.DirtyDerivative9_D*
      rtb_q_um[9];
  }

  /* DiscreteStateSpace: '<S1>/DirtyDerivative10' */
  {
    rtb_VectorConcatenate[23] = (posing_controller_v2_P.DirtyDerivative10_C[0])*
      posing_controller_v2_DWork.DirtyDerivative10_DSTATE[0]
      + (posing_controller_v2_P.DirtyDerivative10_C[1])*
      posing_controller_v2_DWork.DirtyDerivative10_DSTATE[1]
      + (posing_controller_v2_P.DirtyDerivative10_C[2])*
      posing_controller_v2_DWork.DirtyDerivative10_DSTATE[2]
      + (posing_controller_v2_P.DirtyDerivative10_C[3])*
      posing_controller_v2_DWork.DirtyDerivative10_DSTATE[3];
    rtb_VectorConcatenate[23] += posing_controller_v2_P.DirtyDerivative10_D*
      rtb_q_um[10];
  }

  /* DiscreteStateSpace: '<S1>/DirtyDerivative11' */
  {
    rtb_VectorConcatenate[24] = (posing_controller_v2_P.DirtyDerivative11_C[0])*
      posing_controller_v2_DWork.DirtyDerivative11_DSTATE[0]
      + (posing_controller_v2_P.DirtyDerivative11_C[1])*
      posing_controller_v2_DWork.DirtyDerivative11_DSTATE[1]
      + (posing_controller_v2_P.DirtyDerivative11_C[2])*
      posing_controller_v2_DWork.DirtyDerivative11_DSTATE[2]
      + (posing_controller_v2_P.DirtyDerivative11_C[3])*
      posing_controller_v2_DWork.DirtyDerivative11_DSTATE[3];
    rtb_VectorConcatenate[24] += posing_controller_v2_P.DirtyDerivative11_D*
      rtb_q_um[11];
  }

  /* DiscreteStateSpace: '<S1>/DirtyDerivative12' */
  {
    rtb_VectorConcatenate[25] = (posing_controller_v2_P.DirtyDerivative12_C[0])*
      posing_controller_v2_DWork.DirtyDerivative12_DSTATE[0]
      + (posing_controller_v2_P.DirtyDerivative12_C[1])*
      posing_controller_v2_DWork.DirtyDerivative12_DSTATE[1]
      + (posing_controller_v2_P.DirtyDerivative12_C[2])*
      posing_controller_v2_DWork.DirtyDerivative12_DSTATE[2]
      + (posing_controller_v2_P.DirtyDerivative12_C[3])*
      posing_controller_v2_DWork.DirtyDerivative12_DSTATE[3];
    rtb_VectorConcatenate[25] += posing_controller_v2_P.DirtyDerivative12_D*
      rtb_q_um[12];
  }

  /* DiscreteFilter: '<Root>/Discrete Filter' incorporates:
   *  Inport: '<Root>/set_point'
   */
  DiscreteFilter_tmp = (posing_controller_v2_U.set_point[0] -
                        posing_controller_v2_P.DiscreteFilter_DenCoef[1] *
                        posing_controller_v2_DWork.DiscreteFilter_DSTATE) /
    posing_controller_v2_P.DiscreteFilter_DenCoef[0];

  /* DiscreteFilter: '<Root>/Discrete Filter1' incorporates:
   *  Inport: '<Root>/set_point'
   */
  DiscreteFilter1_tmp = (posing_controller_v2_U.set_point[1] -
    posing_controller_v2_P.DiscreteFilter1_DenCoef[1] *
    posing_controller_v2_DWork.DiscreteFilter1_DSTATE) /
    posing_controller_v2_P.DiscreteFilter1_DenCoef[0];

  /* DiscreteFilter: '<Root>/Discrete Filter2' incorporates:
   *  Inport: '<Root>/set_point'
   */
  DiscreteFilter2_tmp = (posing_controller_v2_U.set_point[2] -
    posing_controller_v2_P.DiscreteFilter2_DenCoef[1] *
    posing_controller_v2_DWork.DiscreteFilter2_DSTATE) /
    posing_controller_v2_P.DiscreteFilter2_DenCoef[0];

  /* DiscreteFilter: '<Root>/Discrete Filter3' incorporates:
   *  Inport: '<Root>/set_point'
   */
  DiscreteFilter3_tmp = (posing_controller_v2_U.set_point[3] -
    posing_controller_v2_P.DiscreteFilter3_DenCoef[1] *
    posing_controller_v2_DWork.DiscreteFilter3_DSTATE) /
    posing_controller_v2_P.DiscreteFilter3_DenCoef[0];

  /* DiscreteFilter: '<Root>/Discrete Filter4' incorporates:
   *  Inport: '<Root>/set_point'
   */
  DiscreteFilter4_tmp = (posing_controller_v2_U.set_point[4] -
    posing_controller_v2_P.DiscreteFilter4_DenCoef[1] *
    posing_controller_v2_DWork.DiscreteFilter4_DSTATE) /
    posing_controller_v2_P.DiscreteFilter4_DenCoef[0];

  /* DiscreteFilter: '<Root>/Discrete Filter5' incorporates:
   *  Inport: '<Root>/set_point'
   */
  DiscreteFilter5_tmp = (posing_controller_v2_U.set_point[5] -
    posing_controller_v2_P.DiscreteFilter5_DenCoef[1] *
    posing_controller_v2_DWork.DiscreteFilter5_DSTATE) /
    posing_controller_v2_P.DiscreteFilter5_DenCoef[0];

  /* MATLAB Function: '<Root>/controller' incorporates:
   *  DiscreteFilter: '<Root>/Discrete Filter'
   *  DiscreteFilter: '<Root>/Discrete Filter1'
   *  DiscreteFilter: '<Root>/Discrete Filter2'
   *  DiscreteFilter: '<Root>/Discrete Filter3'
   *  DiscreteFilter: '<Root>/Discrete Filter4'
   *  DiscreteFilter: '<Root>/Discrete Filter5'
   *  Inport: '<Root>/epsilon'
   *  Inport: '<Root>/kd'
   *  Inport: '<Root>/kp'
   */
  /* MATLAB Function 'controller': '<S2>:1' */
  /* %eml */
  /*  x vector 26 by 1 */
  /*  kp vector 3 by 1 */
  /*  kd vector 3 by 1 */
  /*  set point vecotr 6 by 1 */
  /* '<S2>:1:10' */
  /* '<S2>:1:11' */
  /* '<S2>:1:14' */
  /* '<S2>:1:15' */
  /* '<S2>:1:16' */
  /* '<S2>:1:17' */
  /* '<S2>:1:18' */
  /* '<S2>:1:19' */
  /* '<S2>:1:22' */
  /* '<S2>:1:23' */
  /* '<S2>:1:24' */
  /* '<S2>:1:25' */
  /* '<S2>:1:26' */
  /* '<S2>:1:27' */
  /* '<S2>:1:29' */
  /* '<S2>:1:30' */
  rtb_dy[0] = rtb_VectorConcatenate[18];
  rtb_dy[1] = rtb_VectorConcatenate[19];
  rtb_dy[2] = rtb_VectorConcatenate[20];
  rtb_dy[3] = rtb_VectorConcatenate[23];
  rtb_dy[4] = rtb_VectorConcatenate[24];
  rtb_dy[5] = rtb_VectorConcatenate[25];

  /* '<S2>:1:31' */
  rtb_y[0] = rtb_VectorConcatenate[5] -
    posing_controller_v2_P.DiscreteFilter_NumCoef * DiscreteFilter_tmp *
    3.1415926535897931 / 180.0;
  rtb_y[1] = rtb_VectorConcatenate[6] -
    posing_controller_v2_P.DiscreteFilter1_NumCoef * DiscreteFilter1_tmp *
    3.1415926535897931 / 180.0;
  rtb_y[2] = rtb_VectorConcatenate[7] -
    posing_controller_v2_P.DiscreteFilter2_NumCoef * DiscreteFilter2_tmp *
    3.1415926535897931 / 180.0;
  rtb_y[3] = rtb_VectorConcatenate[10] -
    posing_controller_v2_P.DiscreteFilter3_NumCoef * DiscreteFilter3_tmp *
    3.1415926535897931 / 180.0;
  rtb_y[4] = rtb_VectorConcatenate[11] -
    posing_controller_v2_P.DiscreteFilter4_NumCoef * DiscreteFilter4_tmp *
    3.1415926535897931 / 180.0;
  rtb_y[5] = rtb_VectorConcatenate[12] -
    posing_controller_v2_P.DiscreteFilter5_NumCoef * DiscreteFilter5_tmp *
    3.1415926535897931 / 180.0;

  /* '<S2>:1:32' */
  /* '<S2>:1:34' */
  /* '<S2>:1:35' */
  for (i = 0; i < 36; i++) {
    Kp[i] = 0.0;
    Kd[i] = 0.0;
  }

  Kp[0] = posing_controller_v2_U.kp[0];

  /* '<S2>:1:36' */
  Kp[7] = posing_controller_v2_U.kp[1];

  /* '<S2>:1:37' */
  Kp[14] = posing_controller_v2_U.kp[2];

  /* '<S2>:1:38' */
  Kp[21] = posing_controller_v2_U.kp[0];

  /* '<S2>:1:39' */
  Kp[28] = posing_controller_v2_U.kp[1];

  /* '<S2>:1:40' */
  Kp[35] = posing_controller_v2_U.kp[2];

  /* '<S2>:1:42' */
  /* '<S2>:1:43' */
  Kd[0] = posing_controller_v2_U.kd[0];

  /* '<S2>:1:44' */
  Kd[7] = posing_controller_v2_U.kd[1];

  /* '<S2>:1:45' */
  Kd[14] = posing_controller_v2_U.kd[2];

  /* '<S2>:1:46' */
  Kd[21] = posing_controller_v2_U.kd[0];

  /* '<S2>:1:47' */
  Kd[28] = posing_controller_v2_U.kd[1];

  /* '<S2>:1:48' */
  Kd[35] = posing_controller_v2_U.kd[2];

  /* '<S2>:1:51' */
  rtb_DiscreteFilter = rt_powd_snf(posing_controller_v2_U.epsilon, 2.0);
  for (i = 0; i < 6; i++) {
    for (i_0 = 0; i_0 < 6; i_0++) {
      Kp_0[i_0 + 6 * i] = -Kp[6 * i + i_0] / rtb_DiscreteFilter;
    }
  }

  for (i = 0; i < 6; i++) {
    for (i_0 = 0; i_0 < 6; i_0++) {
      Kp[i_0 + 6 * i] = Kd[6 * i + i_0] / posing_controller_v2_U.epsilon;
    }
  }

  for (i = 0; i < 6; i++) {
    Kp_1[i] = 0.0;
    for (i_0 = 0; i_0 < 6; i_0++) {
      Kp_1[i] += Kp_0[6 * i_0 + i] * rtb_y[i_0];
    }

    Kd_0[i] = 0.0;
    for (i_0 = 0; i_0 < 6; i_0++) {
      Kd_0[i] += Kp[6 * i_0 + i] * rtb_dy[i_0];
    }

    rtb_u[i] = Kp_1[i] - Kd_0[i];
  }

  /* transform to OSU coordinates */
  /* '<S2>:1:54' */
  /* '<S2>:1:55' */
  rtb_u[2] = -rtb_u[2];
  for (i = 0; i < 6; i++) {
    /* Saturate: '<Root>/Saturation' */
    if (rtb_u[i] >= posing_controller_v2_P.Saturation_UpperSat[i]) {
      /* Outport: '<Root>/u' */
      posing_controller_v2_Y.u[i] = posing_controller_v2_P.Saturation_UpperSat[i];
    } else if (rtb_u[i] <= posing_controller_v2_P.Saturation_LowerSat[i]) {
      /* Outport: '<Root>/u' */
      posing_controller_v2_Y.u[i] = posing_controller_v2_P.Saturation_LowerSat[i];
    } else {
      /* Outport: '<Root>/u' */
      posing_controller_v2_Y.u[i] = rtb_u[i];
    }

    /* End of Saturate: '<Root>/Saturation' */

    /* Outport: '<Root>/y' */
    posing_controller_v2_Y.y[i] = rtb_y[i];

    /* Outport: '<Root>/dy' */
    posing_controller_v2_Y.dy[i] = rtb_dy[i];
  }

  /* End of MATLAB Function: '<Root>/controller' */
  /* Update for DiscreteStateSpace: '<S1>/DirtyDerivative' */
  {
    real_T xnew[4];
    xnew[0] = (posing_controller_v2_P.DirtyDerivative_A[0])*
      posing_controller_v2_DWork.DirtyDerivative_DSTATE[0]
      + (posing_controller_v2_P.DirtyDerivative_A[1])*
      posing_controller_v2_DWork.DirtyDerivative_DSTATE[1]
      + (posing_controller_v2_P.DirtyDerivative_A[2])*
      posing_controller_v2_DWork.DirtyDerivative_DSTATE[2]
      + (posing_controller_v2_P.DirtyDerivative_A[3])*
      posing_controller_v2_DWork.DirtyDerivative_DSTATE[3];
    xnew[0] += (posing_controller_v2_P.DirtyDerivative_B[0])*rtb_q_um[0];
    xnew[1] = (posing_controller_v2_P.DirtyDerivative_A[4])*
      posing_controller_v2_DWork.DirtyDerivative_DSTATE[0]
      + (posing_controller_v2_P.DirtyDerivative_A[5])*
      posing_controller_v2_DWork.DirtyDerivative_DSTATE[1]
      + (posing_controller_v2_P.DirtyDerivative_A[6])*
      posing_controller_v2_DWork.DirtyDerivative_DSTATE[2]
      + (posing_controller_v2_P.DirtyDerivative_A[7])*
      posing_controller_v2_DWork.DirtyDerivative_DSTATE[3];
    xnew[1] += (posing_controller_v2_P.DirtyDerivative_B[1])*rtb_q_um[0];
    xnew[2] = (posing_controller_v2_P.DirtyDerivative_A[8])*
      posing_controller_v2_DWork.DirtyDerivative_DSTATE[0]
      + (posing_controller_v2_P.DirtyDerivative_A[9])*
      posing_controller_v2_DWork.DirtyDerivative_DSTATE[1]
      + (posing_controller_v2_P.DirtyDerivative_A[10])*
      posing_controller_v2_DWork.DirtyDerivative_DSTATE[2]
      + (posing_controller_v2_P.DirtyDerivative_A[11])*
      posing_controller_v2_DWork.DirtyDerivative_DSTATE[3];
    xnew[2] += (posing_controller_v2_P.DirtyDerivative_B[2])*rtb_q_um[0];
    xnew[3] = (posing_controller_v2_P.DirtyDerivative_A[12])*
      posing_controller_v2_DWork.DirtyDerivative_DSTATE[0]
      + (posing_controller_v2_P.DirtyDerivative_A[13])*
      posing_controller_v2_DWork.DirtyDerivative_DSTATE[1]
      + (posing_controller_v2_P.DirtyDerivative_A[14])*
      posing_controller_v2_DWork.DirtyDerivative_DSTATE[2]
      + (posing_controller_v2_P.DirtyDerivative_A[15])*
      posing_controller_v2_DWork.DirtyDerivative_DSTATE[3];
    xnew[3] += (posing_controller_v2_P.DirtyDerivative_B[3])*rtb_q_um[0];
    (void) memcpy(&posing_controller_v2_DWork.DirtyDerivative_DSTATE[0], xnew,
                  sizeof(real_T)*4);
  }

  /* Update for DiscreteStateSpace: '<S1>/DirtyDerivative1' */
  {
    real_T xnew[4];
    xnew[0] = (posing_controller_v2_P.DirtyDerivative1_A[0])*
      posing_controller_v2_DWork.DirtyDerivative1_DSTATE[0]
      + (posing_controller_v2_P.DirtyDerivative1_A[1])*
      posing_controller_v2_DWork.DirtyDerivative1_DSTATE[1]
      + (posing_controller_v2_P.DirtyDerivative1_A[2])*
      posing_controller_v2_DWork.DirtyDerivative1_DSTATE[2]
      + (posing_controller_v2_P.DirtyDerivative1_A[3])*
      posing_controller_v2_DWork.DirtyDerivative1_DSTATE[3];
    xnew[0] += (posing_controller_v2_P.DirtyDerivative1_B[0])*rtb_q_um[1];
    xnew[1] = (posing_controller_v2_P.DirtyDerivative1_A[4])*
      posing_controller_v2_DWork.DirtyDerivative1_DSTATE[0]
      + (posing_controller_v2_P.DirtyDerivative1_A[5])*
      posing_controller_v2_DWork.DirtyDerivative1_DSTATE[1]
      + (posing_controller_v2_P.DirtyDerivative1_A[6])*
      posing_controller_v2_DWork.DirtyDerivative1_DSTATE[2]
      + (posing_controller_v2_P.DirtyDerivative1_A[7])*
      posing_controller_v2_DWork.DirtyDerivative1_DSTATE[3];
    xnew[1] += (posing_controller_v2_P.DirtyDerivative1_B[1])*rtb_q_um[1];
    xnew[2] = (posing_controller_v2_P.DirtyDerivative1_A[8])*
      posing_controller_v2_DWork.DirtyDerivative1_DSTATE[0]
      + (posing_controller_v2_P.DirtyDerivative1_A[9])*
      posing_controller_v2_DWork.DirtyDerivative1_DSTATE[1]
      + (posing_controller_v2_P.DirtyDerivative1_A[10])*
      posing_controller_v2_DWork.DirtyDerivative1_DSTATE[2]
      + (posing_controller_v2_P.DirtyDerivative1_A[11])*
      posing_controller_v2_DWork.DirtyDerivative1_DSTATE[3];
    xnew[2] += (posing_controller_v2_P.DirtyDerivative1_B[2])*rtb_q_um[1];
    xnew[3] = (posing_controller_v2_P.DirtyDerivative1_A[12])*
      posing_controller_v2_DWork.DirtyDerivative1_DSTATE[0]
      + (posing_controller_v2_P.DirtyDerivative1_A[13])*
      posing_controller_v2_DWork.DirtyDerivative1_DSTATE[1]
      + (posing_controller_v2_P.DirtyDerivative1_A[14])*
      posing_controller_v2_DWork.DirtyDerivative1_DSTATE[2]
      + (posing_controller_v2_P.DirtyDerivative1_A[15])*
      posing_controller_v2_DWork.DirtyDerivative1_DSTATE[3];
    xnew[3] += (posing_controller_v2_P.DirtyDerivative1_B[3])*rtb_q_um[1];
    (void) memcpy(&posing_controller_v2_DWork.DirtyDerivative1_DSTATE[0], xnew,
                  sizeof(real_T)*4);
  }

  /* Update for DiscreteStateSpace: '<S1>/DirtyDerivative2' */
  {
    real_T xnew[4];
    xnew[0] = (posing_controller_v2_P.DirtyDerivative2_A[0])*
      posing_controller_v2_DWork.DirtyDerivative2_DSTATE[0]
      + (posing_controller_v2_P.DirtyDerivative2_A[1])*
      posing_controller_v2_DWork.DirtyDerivative2_DSTATE[1]
      + (posing_controller_v2_P.DirtyDerivative2_A[2])*
      posing_controller_v2_DWork.DirtyDerivative2_DSTATE[2]
      + (posing_controller_v2_P.DirtyDerivative2_A[3])*
      posing_controller_v2_DWork.DirtyDerivative2_DSTATE[3];
    xnew[0] += (posing_controller_v2_P.DirtyDerivative2_B[0])*rtb_q_um[2];
    xnew[1] = (posing_controller_v2_P.DirtyDerivative2_A[4])*
      posing_controller_v2_DWork.DirtyDerivative2_DSTATE[0]
      + (posing_controller_v2_P.DirtyDerivative2_A[5])*
      posing_controller_v2_DWork.DirtyDerivative2_DSTATE[1]
      + (posing_controller_v2_P.DirtyDerivative2_A[6])*
      posing_controller_v2_DWork.DirtyDerivative2_DSTATE[2]
      + (posing_controller_v2_P.DirtyDerivative2_A[7])*
      posing_controller_v2_DWork.DirtyDerivative2_DSTATE[3];
    xnew[1] += (posing_controller_v2_P.DirtyDerivative2_B[1])*rtb_q_um[2];
    xnew[2] = (posing_controller_v2_P.DirtyDerivative2_A[8])*
      posing_controller_v2_DWork.DirtyDerivative2_DSTATE[0]
      + (posing_controller_v2_P.DirtyDerivative2_A[9])*
      posing_controller_v2_DWork.DirtyDerivative2_DSTATE[1]
      + (posing_controller_v2_P.DirtyDerivative2_A[10])*
      posing_controller_v2_DWork.DirtyDerivative2_DSTATE[2]
      + (posing_controller_v2_P.DirtyDerivative2_A[11])*
      posing_controller_v2_DWork.DirtyDerivative2_DSTATE[3];
    xnew[2] += (posing_controller_v2_P.DirtyDerivative2_B[2])*rtb_q_um[2];
    xnew[3] = (posing_controller_v2_P.DirtyDerivative2_A[12])*
      posing_controller_v2_DWork.DirtyDerivative2_DSTATE[0]
      + (posing_controller_v2_P.DirtyDerivative2_A[13])*
      posing_controller_v2_DWork.DirtyDerivative2_DSTATE[1]
      + (posing_controller_v2_P.DirtyDerivative2_A[14])*
      posing_controller_v2_DWork.DirtyDerivative2_DSTATE[2]
      + (posing_controller_v2_P.DirtyDerivative2_A[15])*
      posing_controller_v2_DWork.DirtyDerivative2_DSTATE[3];
    xnew[3] += (posing_controller_v2_P.DirtyDerivative2_B[3])*rtb_q_um[2];
    (void) memcpy(&posing_controller_v2_DWork.DirtyDerivative2_DSTATE[0], xnew,
                  sizeof(real_T)*4);
  }

  /* Update for DiscreteStateSpace: '<S1>/DirtyDerivative3' */
  {
    real_T xnew[4];
    xnew[0] = (posing_controller_v2_P.DirtyDerivative3_A[0])*
      posing_controller_v2_DWork.DirtyDerivative3_DSTATE[0]
      + (posing_controller_v2_P.DirtyDerivative3_A[1])*
      posing_controller_v2_DWork.DirtyDerivative3_DSTATE[1]
      + (posing_controller_v2_P.DirtyDerivative3_A[2])*
      posing_controller_v2_DWork.DirtyDerivative3_DSTATE[2]
      + (posing_controller_v2_P.DirtyDerivative3_A[3])*
      posing_controller_v2_DWork.DirtyDerivative3_DSTATE[3];
    xnew[0] += (posing_controller_v2_P.DirtyDerivative3_B[0])*rtb_q_um[3];
    xnew[1] = (posing_controller_v2_P.DirtyDerivative3_A[4])*
      posing_controller_v2_DWork.DirtyDerivative3_DSTATE[0]
      + (posing_controller_v2_P.DirtyDerivative3_A[5])*
      posing_controller_v2_DWork.DirtyDerivative3_DSTATE[1]
      + (posing_controller_v2_P.DirtyDerivative3_A[6])*
      posing_controller_v2_DWork.DirtyDerivative3_DSTATE[2]
      + (posing_controller_v2_P.DirtyDerivative3_A[7])*
      posing_controller_v2_DWork.DirtyDerivative3_DSTATE[3];
    xnew[1] += (posing_controller_v2_P.DirtyDerivative3_B[1])*rtb_q_um[3];
    xnew[2] = (posing_controller_v2_P.DirtyDerivative3_A[8])*
      posing_controller_v2_DWork.DirtyDerivative3_DSTATE[0]
      + (posing_controller_v2_P.DirtyDerivative3_A[9])*
      posing_controller_v2_DWork.DirtyDerivative3_DSTATE[1]
      + (posing_controller_v2_P.DirtyDerivative3_A[10])*
      posing_controller_v2_DWork.DirtyDerivative3_DSTATE[2]
      + (posing_controller_v2_P.DirtyDerivative3_A[11])*
      posing_controller_v2_DWork.DirtyDerivative3_DSTATE[3];
    xnew[2] += (posing_controller_v2_P.DirtyDerivative3_B[2])*rtb_q_um[3];
    xnew[3] = (posing_controller_v2_P.DirtyDerivative3_A[12])*
      posing_controller_v2_DWork.DirtyDerivative3_DSTATE[0]
      + (posing_controller_v2_P.DirtyDerivative3_A[13])*
      posing_controller_v2_DWork.DirtyDerivative3_DSTATE[1]
      + (posing_controller_v2_P.DirtyDerivative3_A[14])*
      posing_controller_v2_DWork.DirtyDerivative3_DSTATE[2]
      + (posing_controller_v2_P.DirtyDerivative3_A[15])*
      posing_controller_v2_DWork.DirtyDerivative3_DSTATE[3];
    xnew[3] += (posing_controller_v2_P.DirtyDerivative3_B[3])*rtb_q_um[3];
    (void) memcpy(&posing_controller_v2_DWork.DirtyDerivative3_DSTATE[0], xnew,
                  sizeof(real_T)*4);
  }

  /* Update for DiscreteStateSpace: '<S1>/DirtyDerivative4' */
  {
    real_T xnew[4];
    xnew[0] = (posing_controller_v2_P.DirtyDerivative4_A[0])*
      posing_controller_v2_DWork.DirtyDerivative4_DSTATE[0]
      + (posing_controller_v2_P.DirtyDerivative4_A[1])*
      posing_controller_v2_DWork.DirtyDerivative4_DSTATE[1]
      + (posing_controller_v2_P.DirtyDerivative4_A[2])*
      posing_controller_v2_DWork.DirtyDerivative4_DSTATE[2]
      + (posing_controller_v2_P.DirtyDerivative4_A[3])*
      posing_controller_v2_DWork.DirtyDerivative4_DSTATE[3];
    xnew[0] += (posing_controller_v2_P.DirtyDerivative4_B[0])*rtb_q_um[4];
    xnew[1] = (posing_controller_v2_P.DirtyDerivative4_A[4])*
      posing_controller_v2_DWork.DirtyDerivative4_DSTATE[0]
      + (posing_controller_v2_P.DirtyDerivative4_A[5])*
      posing_controller_v2_DWork.DirtyDerivative4_DSTATE[1]
      + (posing_controller_v2_P.DirtyDerivative4_A[6])*
      posing_controller_v2_DWork.DirtyDerivative4_DSTATE[2]
      + (posing_controller_v2_P.DirtyDerivative4_A[7])*
      posing_controller_v2_DWork.DirtyDerivative4_DSTATE[3];
    xnew[1] += (posing_controller_v2_P.DirtyDerivative4_B[1])*rtb_q_um[4];
    xnew[2] = (posing_controller_v2_P.DirtyDerivative4_A[8])*
      posing_controller_v2_DWork.DirtyDerivative4_DSTATE[0]
      + (posing_controller_v2_P.DirtyDerivative4_A[9])*
      posing_controller_v2_DWork.DirtyDerivative4_DSTATE[1]
      + (posing_controller_v2_P.DirtyDerivative4_A[10])*
      posing_controller_v2_DWork.DirtyDerivative4_DSTATE[2]
      + (posing_controller_v2_P.DirtyDerivative4_A[11])*
      posing_controller_v2_DWork.DirtyDerivative4_DSTATE[3];
    xnew[2] += (posing_controller_v2_P.DirtyDerivative4_B[2])*rtb_q_um[4];
    xnew[3] = (posing_controller_v2_P.DirtyDerivative4_A[12])*
      posing_controller_v2_DWork.DirtyDerivative4_DSTATE[0]
      + (posing_controller_v2_P.DirtyDerivative4_A[13])*
      posing_controller_v2_DWork.DirtyDerivative4_DSTATE[1]
      + (posing_controller_v2_P.DirtyDerivative4_A[14])*
      posing_controller_v2_DWork.DirtyDerivative4_DSTATE[2]
      + (posing_controller_v2_P.DirtyDerivative4_A[15])*
      posing_controller_v2_DWork.DirtyDerivative4_DSTATE[3];
    xnew[3] += (posing_controller_v2_P.DirtyDerivative4_B[3])*rtb_q_um[4];
    (void) memcpy(&posing_controller_v2_DWork.DirtyDerivative4_DSTATE[0], xnew,
                  sizeof(real_T)*4);
  }

  /* Update for DiscreteStateSpace: '<S1>/DirtyDerivative5' */
  {
    real_T xnew[4];
    xnew[0] = (posing_controller_v2_P.DirtyDerivative5_A[0])*
      posing_controller_v2_DWork.DirtyDerivative5_DSTATE[0]
      + (posing_controller_v2_P.DirtyDerivative5_A[1])*
      posing_controller_v2_DWork.DirtyDerivative5_DSTATE[1]
      + (posing_controller_v2_P.DirtyDerivative5_A[2])*
      posing_controller_v2_DWork.DirtyDerivative5_DSTATE[2]
      + (posing_controller_v2_P.DirtyDerivative5_A[3])*
      posing_controller_v2_DWork.DirtyDerivative5_DSTATE[3];
    xnew[0] += (posing_controller_v2_P.DirtyDerivative5_B[0])*rtb_q_um[5];
    xnew[1] = (posing_controller_v2_P.DirtyDerivative5_A[4])*
      posing_controller_v2_DWork.DirtyDerivative5_DSTATE[0]
      + (posing_controller_v2_P.DirtyDerivative5_A[5])*
      posing_controller_v2_DWork.DirtyDerivative5_DSTATE[1]
      + (posing_controller_v2_P.DirtyDerivative5_A[6])*
      posing_controller_v2_DWork.DirtyDerivative5_DSTATE[2]
      + (posing_controller_v2_P.DirtyDerivative5_A[7])*
      posing_controller_v2_DWork.DirtyDerivative5_DSTATE[3];
    xnew[1] += (posing_controller_v2_P.DirtyDerivative5_B[1])*rtb_q_um[5];
    xnew[2] = (posing_controller_v2_P.DirtyDerivative5_A[8])*
      posing_controller_v2_DWork.DirtyDerivative5_DSTATE[0]
      + (posing_controller_v2_P.DirtyDerivative5_A[9])*
      posing_controller_v2_DWork.DirtyDerivative5_DSTATE[1]
      + (posing_controller_v2_P.DirtyDerivative5_A[10])*
      posing_controller_v2_DWork.DirtyDerivative5_DSTATE[2]
      + (posing_controller_v2_P.DirtyDerivative5_A[11])*
      posing_controller_v2_DWork.DirtyDerivative5_DSTATE[3];
    xnew[2] += (posing_controller_v2_P.DirtyDerivative5_B[2])*rtb_q_um[5];
    xnew[3] = (posing_controller_v2_P.DirtyDerivative5_A[12])*
      posing_controller_v2_DWork.DirtyDerivative5_DSTATE[0]
      + (posing_controller_v2_P.DirtyDerivative5_A[13])*
      posing_controller_v2_DWork.DirtyDerivative5_DSTATE[1]
      + (posing_controller_v2_P.DirtyDerivative5_A[14])*
      posing_controller_v2_DWork.DirtyDerivative5_DSTATE[2]
      + (posing_controller_v2_P.DirtyDerivative5_A[15])*
      posing_controller_v2_DWork.DirtyDerivative5_DSTATE[3];
    xnew[3] += (posing_controller_v2_P.DirtyDerivative5_B[3])*rtb_q_um[5];
    (void) memcpy(&posing_controller_v2_DWork.DirtyDerivative5_DSTATE[0], xnew,
                  sizeof(real_T)*4);
  }

  /* Update for DiscreteStateSpace: '<S1>/DirtyDerivative6' */
  {
    real_T xnew[4];
    xnew[0] = (posing_controller_v2_P.DirtyDerivative6_A[0])*
      posing_controller_v2_DWork.DirtyDerivative6_DSTATE[0]
      + (posing_controller_v2_P.DirtyDerivative6_A[1])*
      posing_controller_v2_DWork.DirtyDerivative6_DSTATE[1]
      + (posing_controller_v2_P.DirtyDerivative6_A[2])*
      posing_controller_v2_DWork.DirtyDerivative6_DSTATE[2]
      + (posing_controller_v2_P.DirtyDerivative6_A[3])*
      posing_controller_v2_DWork.DirtyDerivative6_DSTATE[3];
    xnew[0] += (posing_controller_v2_P.DirtyDerivative6_B[0])*rtb_q_um[6];
    xnew[1] = (posing_controller_v2_P.DirtyDerivative6_A[4])*
      posing_controller_v2_DWork.DirtyDerivative6_DSTATE[0]
      + (posing_controller_v2_P.DirtyDerivative6_A[5])*
      posing_controller_v2_DWork.DirtyDerivative6_DSTATE[1]
      + (posing_controller_v2_P.DirtyDerivative6_A[6])*
      posing_controller_v2_DWork.DirtyDerivative6_DSTATE[2]
      + (posing_controller_v2_P.DirtyDerivative6_A[7])*
      posing_controller_v2_DWork.DirtyDerivative6_DSTATE[3];
    xnew[1] += (posing_controller_v2_P.DirtyDerivative6_B[1])*rtb_q_um[6];
    xnew[2] = (posing_controller_v2_P.DirtyDerivative6_A[8])*
      posing_controller_v2_DWork.DirtyDerivative6_DSTATE[0]
      + (posing_controller_v2_P.DirtyDerivative6_A[9])*
      posing_controller_v2_DWork.DirtyDerivative6_DSTATE[1]
      + (posing_controller_v2_P.DirtyDerivative6_A[10])*
      posing_controller_v2_DWork.DirtyDerivative6_DSTATE[2]
      + (posing_controller_v2_P.DirtyDerivative6_A[11])*
      posing_controller_v2_DWork.DirtyDerivative6_DSTATE[3];
    xnew[2] += (posing_controller_v2_P.DirtyDerivative6_B[2])*rtb_q_um[6];
    xnew[3] = (posing_controller_v2_P.DirtyDerivative6_A[12])*
      posing_controller_v2_DWork.DirtyDerivative6_DSTATE[0]
      + (posing_controller_v2_P.DirtyDerivative6_A[13])*
      posing_controller_v2_DWork.DirtyDerivative6_DSTATE[1]
      + (posing_controller_v2_P.DirtyDerivative6_A[14])*
      posing_controller_v2_DWork.DirtyDerivative6_DSTATE[2]
      + (posing_controller_v2_P.DirtyDerivative6_A[15])*
      posing_controller_v2_DWork.DirtyDerivative6_DSTATE[3];
    xnew[3] += (posing_controller_v2_P.DirtyDerivative6_B[3])*rtb_q_um[6];
    (void) memcpy(&posing_controller_v2_DWork.DirtyDerivative6_DSTATE[0], xnew,
                  sizeof(real_T)*4);
  }

  /* Update for DiscreteStateSpace: '<S1>/DirtyDerivative7' */
  {
    real_T xnew[4];
    xnew[0] = (posing_controller_v2_P.DirtyDerivative7_A[0])*
      posing_controller_v2_DWork.DirtyDerivative7_DSTATE[0]
      + (posing_controller_v2_P.DirtyDerivative7_A[1])*
      posing_controller_v2_DWork.DirtyDerivative7_DSTATE[1]
      + (posing_controller_v2_P.DirtyDerivative7_A[2])*
      posing_controller_v2_DWork.DirtyDerivative7_DSTATE[2]
      + (posing_controller_v2_P.DirtyDerivative7_A[3])*
      posing_controller_v2_DWork.DirtyDerivative7_DSTATE[3];
    xnew[0] += (posing_controller_v2_P.DirtyDerivative7_B[0])*rtb_q_um[7];
    xnew[1] = (posing_controller_v2_P.DirtyDerivative7_A[4])*
      posing_controller_v2_DWork.DirtyDerivative7_DSTATE[0]
      + (posing_controller_v2_P.DirtyDerivative7_A[5])*
      posing_controller_v2_DWork.DirtyDerivative7_DSTATE[1]
      + (posing_controller_v2_P.DirtyDerivative7_A[6])*
      posing_controller_v2_DWork.DirtyDerivative7_DSTATE[2]
      + (posing_controller_v2_P.DirtyDerivative7_A[7])*
      posing_controller_v2_DWork.DirtyDerivative7_DSTATE[3];
    xnew[1] += (posing_controller_v2_P.DirtyDerivative7_B[1])*rtb_q_um[7];
    xnew[2] = (posing_controller_v2_P.DirtyDerivative7_A[8])*
      posing_controller_v2_DWork.DirtyDerivative7_DSTATE[0]
      + (posing_controller_v2_P.DirtyDerivative7_A[9])*
      posing_controller_v2_DWork.DirtyDerivative7_DSTATE[1]
      + (posing_controller_v2_P.DirtyDerivative7_A[10])*
      posing_controller_v2_DWork.DirtyDerivative7_DSTATE[2]
      + (posing_controller_v2_P.DirtyDerivative7_A[11])*
      posing_controller_v2_DWork.DirtyDerivative7_DSTATE[3];
    xnew[2] += (posing_controller_v2_P.DirtyDerivative7_B[2])*rtb_q_um[7];
    xnew[3] = (posing_controller_v2_P.DirtyDerivative7_A[12])*
      posing_controller_v2_DWork.DirtyDerivative7_DSTATE[0]
      + (posing_controller_v2_P.DirtyDerivative7_A[13])*
      posing_controller_v2_DWork.DirtyDerivative7_DSTATE[1]
      + (posing_controller_v2_P.DirtyDerivative7_A[14])*
      posing_controller_v2_DWork.DirtyDerivative7_DSTATE[2]
      + (posing_controller_v2_P.DirtyDerivative7_A[15])*
      posing_controller_v2_DWork.DirtyDerivative7_DSTATE[3];
    xnew[3] += (posing_controller_v2_P.DirtyDerivative7_B[3])*rtb_q_um[7];
    (void) memcpy(&posing_controller_v2_DWork.DirtyDerivative7_DSTATE[0], xnew,
                  sizeof(real_T)*4);
  }

  /* Update for DiscreteStateSpace: '<S1>/DirtyDerivative8' */
  {
    real_T xnew[4];
    xnew[0] = (posing_controller_v2_P.DirtyDerivative8_A[0])*
      posing_controller_v2_DWork.DirtyDerivative8_DSTATE[0]
      + (posing_controller_v2_P.DirtyDerivative8_A[1])*
      posing_controller_v2_DWork.DirtyDerivative8_DSTATE[1]
      + (posing_controller_v2_P.DirtyDerivative8_A[2])*
      posing_controller_v2_DWork.DirtyDerivative8_DSTATE[2]
      + (posing_controller_v2_P.DirtyDerivative8_A[3])*
      posing_controller_v2_DWork.DirtyDerivative8_DSTATE[3];
    xnew[0] += (posing_controller_v2_P.DirtyDerivative8_B[0])*rtb_q_um[8];
    xnew[1] = (posing_controller_v2_P.DirtyDerivative8_A[4])*
      posing_controller_v2_DWork.DirtyDerivative8_DSTATE[0]
      + (posing_controller_v2_P.DirtyDerivative8_A[5])*
      posing_controller_v2_DWork.DirtyDerivative8_DSTATE[1]
      + (posing_controller_v2_P.DirtyDerivative8_A[6])*
      posing_controller_v2_DWork.DirtyDerivative8_DSTATE[2]
      + (posing_controller_v2_P.DirtyDerivative8_A[7])*
      posing_controller_v2_DWork.DirtyDerivative8_DSTATE[3];
    xnew[1] += (posing_controller_v2_P.DirtyDerivative8_B[1])*rtb_q_um[8];
    xnew[2] = (posing_controller_v2_P.DirtyDerivative8_A[8])*
      posing_controller_v2_DWork.DirtyDerivative8_DSTATE[0]
      + (posing_controller_v2_P.DirtyDerivative8_A[9])*
      posing_controller_v2_DWork.DirtyDerivative8_DSTATE[1]
      + (posing_controller_v2_P.DirtyDerivative8_A[10])*
      posing_controller_v2_DWork.DirtyDerivative8_DSTATE[2]
      + (posing_controller_v2_P.DirtyDerivative8_A[11])*
      posing_controller_v2_DWork.DirtyDerivative8_DSTATE[3];
    xnew[2] += (posing_controller_v2_P.DirtyDerivative8_B[2])*rtb_q_um[8];
    xnew[3] = (posing_controller_v2_P.DirtyDerivative8_A[12])*
      posing_controller_v2_DWork.DirtyDerivative8_DSTATE[0]
      + (posing_controller_v2_P.DirtyDerivative8_A[13])*
      posing_controller_v2_DWork.DirtyDerivative8_DSTATE[1]
      + (posing_controller_v2_P.DirtyDerivative8_A[14])*
      posing_controller_v2_DWork.DirtyDerivative8_DSTATE[2]
      + (posing_controller_v2_P.DirtyDerivative8_A[15])*
      posing_controller_v2_DWork.DirtyDerivative8_DSTATE[3];
    xnew[3] += (posing_controller_v2_P.DirtyDerivative8_B[3])*rtb_q_um[8];
    (void) memcpy(&posing_controller_v2_DWork.DirtyDerivative8_DSTATE[0], xnew,
                  sizeof(real_T)*4);
  }

  /* Update for DiscreteStateSpace: '<S1>/DirtyDerivative9' */
  {
    real_T xnew[4];
    xnew[0] = (posing_controller_v2_P.DirtyDerivative9_A[0])*
      posing_controller_v2_DWork.DirtyDerivative9_DSTATE[0]
      + (posing_controller_v2_P.DirtyDerivative9_A[1])*
      posing_controller_v2_DWork.DirtyDerivative9_DSTATE[1]
      + (posing_controller_v2_P.DirtyDerivative9_A[2])*
      posing_controller_v2_DWork.DirtyDerivative9_DSTATE[2]
      + (posing_controller_v2_P.DirtyDerivative9_A[3])*
      posing_controller_v2_DWork.DirtyDerivative9_DSTATE[3];
    xnew[0] += (posing_controller_v2_P.DirtyDerivative9_B[0])*rtb_q_um[9];
    xnew[1] = (posing_controller_v2_P.DirtyDerivative9_A[4])*
      posing_controller_v2_DWork.DirtyDerivative9_DSTATE[0]
      + (posing_controller_v2_P.DirtyDerivative9_A[5])*
      posing_controller_v2_DWork.DirtyDerivative9_DSTATE[1]
      + (posing_controller_v2_P.DirtyDerivative9_A[6])*
      posing_controller_v2_DWork.DirtyDerivative9_DSTATE[2]
      + (posing_controller_v2_P.DirtyDerivative9_A[7])*
      posing_controller_v2_DWork.DirtyDerivative9_DSTATE[3];
    xnew[1] += (posing_controller_v2_P.DirtyDerivative9_B[1])*rtb_q_um[9];
    xnew[2] = (posing_controller_v2_P.DirtyDerivative9_A[8])*
      posing_controller_v2_DWork.DirtyDerivative9_DSTATE[0]
      + (posing_controller_v2_P.DirtyDerivative9_A[9])*
      posing_controller_v2_DWork.DirtyDerivative9_DSTATE[1]
      + (posing_controller_v2_P.DirtyDerivative9_A[10])*
      posing_controller_v2_DWork.DirtyDerivative9_DSTATE[2]
      + (posing_controller_v2_P.DirtyDerivative9_A[11])*
      posing_controller_v2_DWork.DirtyDerivative9_DSTATE[3];
    xnew[2] += (posing_controller_v2_P.DirtyDerivative9_B[2])*rtb_q_um[9];
    xnew[3] = (posing_controller_v2_P.DirtyDerivative9_A[12])*
      posing_controller_v2_DWork.DirtyDerivative9_DSTATE[0]
      + (posing_controller_v2_P.DirtyDerivative9_A[13])*
      posing_controller_v2_DWork.DirtyDerivative9_DSTATE[1]
      + (posing_controller_v2_P.DirtyDerivative9_A[14])*
      posing_controller_v2_DWork.DirtyDerivative9_DSTATE[2]
      + (posing_controller_v2_P.DirtyDerivative9_A[15])*
      posing_controller_v2_DWork.DirtyDerivative9_DSTATE[3];
    xnew[3] += (posing_controller_v2_P.DirtyDerivative9_B[3])*rtb_q_um[9];
    (void) memcpy(&posing_controller_v2_DWork.DirtyDerivative9_DSTATE[0], xnew,
                  sizeof(real_T)*4);
  }

  /* Update for DiscreteStateSpace: '<S1>/DirtyDerivative10' */
  {
    real_T xnew[4];
    xnew[0] = (posing_controller_v2_P.DirtyDerivative10_A[0])*
      posing_controller_v2_DWork.DirtyDerivative10_DSTATE[0]
      + (posing_controller_v2_P.DirtyDerivative10_A[1])*
      posing_controller_v2_DWork.DirtyDerivative10_DSTATE[1]
      + (posing_controller_v2_P.DirtyDerivative10_A[2])*
      posing_controller_v2_DWork.DirtyDerivative10_DSTATE[2]
      + (posing_controller_v2_P.DirtyDerivative10_A[3])*
      posing_controller_v2_DWork.DirtyDerivative10_DSTATE[3];
    xnew[0] += (posing_controller_v2_P.DirtyDerivative10_B[0])*rtb_q_um[10];
    xnew[1] = (posing_controller_v2_P.DirtyDerivative10_A[4])*
      posing_controller_v2_DWork.DirtyDerivative10_DSTATE[0]
      + (posing_controller_v2_P.DirtyDerivative10_A[5])*
      posing_controller_v2_DWork.DirtyDerivative10_DSTATE[1]
      + (posing_controller_v2_P.DirtyDerivative10_A[6])*
      posing_controller_v2_DWork.DirtyDerivative10_DSTATE[2]
      + (posing_controller_v2_P.DirtyDerivative10_A[7])*
      posing_controller_v2_DWork.DirtyDerivative10_DSTATE[3];
    xnew[1] += (posing_controller_v2_P.DirtyDerivative10_B[1])*rtb_q_um[10];
    xnew[2] = (posing_controller_v2_P.DirtyDerivative10_A[8])*
      posing_controller_v2_DWork.DirtyDerivative10_DSTATE[0]
      + (posing_controller_v2_P.DirtyDerivative10_A[9])*
      posing_controller_v2_DWork.DirtyDerivative10_DSTATE[1]
      + (posing_controller_v2_P.DirtyDerivative10_A[10])*
      posing_controller_v2_DWork.DirtyDerivative10_DSTATE[2]
      + (posing_controller_v2_P.DirtyDerivative10_A[11])*
      posing_controller_v2_DWork.DirtyDerivative10_DSTATE[3];
    xnew[2] += (posing_controller_v2_P.DirtyDerivative10_B[2])*rtb_q_um[10];
    xnew[3] = (posing_controller_v2_P.DirtyDerivative10_A[12])*
      posing_controller_v2_DWork.DirtyDerivative10_DSTATE[0]
      + (posing_controller_v2_P.DirtyDerivative10_A[13])*
      posing_controller_v2_DWork.DirtyDerivative10_DSTATE[1]
      + (posing_controller_v2_P.DirtyDerivative10_A[14])*
      posing_controller_v2_DWork.DirtyDerivative10_DSTATE[2]
      + (posing_controller_v2_P.DirtyDerivative10_A[15])*
      posing_controller_v2_DWork.DirtyDerivative10_DSTATE[3];
    xnew[3] += (posing_controller_v2_P.DirtyDerivative10_B[3])*rtb_q_um[10];
    (void) memcpy(&posing_controller_v2_DWork.DirtyDerivative10_DSTATE[0], xnew,
                  sizeof(real_T)*4);
  }

  /* Update for DiscreteStateSpace: '<S1>/DirtyDerivative11' */
  {
    real_T xnew[4];
    xnew[0] = (posing_controller_v2_P.DirtyDerivative11_A[0])*
      posing_controller_v2_DWork.DirtyDerivative11_DSTATE[0]
      + (posing_controller_v2_P.DirtyDerivative11_A[1])*
      posing_controller_v2_DWork.DirtyDerivative11_DSTATE[1]
      + (posing_controller_v2_P.DirtyDerivative11_A[2])*
      posing_controller_v2_DWork.DirtyDerivative11_DSTATE[2]
      + (posing_controller_v2_P.DirtyDerivative11_A[3])*
      posing_controller_v2_DWork.DirtyDerivative11_DSTATE[3];
    xnew[0] += (posing_controller_v2_P.DirtyDerivative11_B[0])*rtb_q_um[11];
    xnew[1] = (posing_controller_v2_P.DirtyDerivative11_A[4])*
      posing_controller_v2_DWork.DirtyDerivative11_DSTATE[0]
      + (posing_controller_v2_P.DirtyDerivative11_A[5])*
      posing_controller_v2_DWork.DirtyDerivative11_DSTATE[1]
      + (posing_controller_v2_P.DirtyDerivative11_A[6])*
      posing_controller_v2_DWork.DirtyDerivative11_DSTATE[2]
      + (posing_controller_v2_P.DirtyDerivative11_A[7])*
      posing_controller_v2_DWork.DirtyDerivative11_DSTATE[3];
    xnew[1] += (posing_controller_v2_P.DirtyDerivative11_B[1])*rtb_q_um[11];
    xnew[2] = (posing_controller_v2_P.DirtyDerivative11_A[8])*
      posing_controller_v2_DWork.DirtyDerivative11_DSTATE[0]
      + (posing_controller_v2_P.DirtyDerivative11_A[9])*
      posing_controller_v2_DWork.DirtyDerivative11_DSTATE[1]
      + (posing_controller_v2_P.DirtyDerivative11_A[10])*
      posing_controller_v2_DWork.DirtyDerivative11_DSTATE[2]
      + (posing_controller_v2_P.DirtyDerivative11_A[11])*
      posing_controller_v2_DWork.DirtyDerivative11_DSTATE[3];
    xnew[2] += (posing_controller_v2_P.DirtyDerivative11_B[2])*rtb_q_um[11];
    xnew[3] = (posing_controller_v2_P.DirtyDerivative11_A[12])*
      posing_controller_v2_DWork.DirtyDerivative11_DSTATE[0]
      + (posing_controller_v2_P.DirtyDerivative11_A[13])*
      posing_controller_v2_DWork.DirtyDerivative11_DSTATE[1]
      + (posing_controller_v2_P.DirtyDerivative11_A[14])*
      posing_controller_v2_DWork.DirtyDerivative11_DSTATE[2]
      + (posing_controller_v2_P.DirtyDerivative11_A[15])*
      posing_controller_v2_DWork.DirtyDerivative11_DSTATE[3];
    xnew[3] += (posing_controller_v2_P.DirtyDerivative11_B[3])*rtb_q_um[11];
    (void) memcpy(&posing_controller_v2_DWork.DirtyDerivative11_DSTATE[0], xnew,
                  sizeof(real_T)*4);
  }

  /* Update for DiscreteStateSpace: '<S1>/DirtyDerivative12' */
  {
    real_T xnew[4];
    xnew[0] = (posing_controller_v2_P.DirtyDerivative12_A[0])*
      posing_controller_v2_DWork.DirtyDerivative12_DSTATE[0]
      + (posing_controller_v2_P.DirtyDerivative12_A[1])*
      posing_controller_v2_DWork.DirtyDerivative12_DSTATE[1]
      + (posing_controller_v2_P.DirtyDerivative12_A[2])*
      posing_controller_v2_DWork.DirtyDerivative12_DSTATE[2]
      + (posing_controller_v2_P.DirtyDerivative12_A[3])*
      posing_controller_v2_DWork.DirtyDerivative12_DSTATE[3];
    xnew[0] += (posing_controller_v2_P.DirtyDerivative12_B[0])*rtb_q_um[12];
    xnew[1] = (posing_controller_v2_P.DirtyDerivative12_A[4])*
      posing_controller_v2_DWork.DirtyDerivative12_DSTATE[0]
      + (posing_controller_v2_P.DirtyDerivative12_A[5])*
      posing_controller_v2_DWork.DirtyDerivative12_DSTATE[1]
      + (posing_controller_v2_P.DirtyDerivative12_A[6])*
      posing_controller_v2_DWork.DirtyDerivative12_DSTATE[2]
      + (posing_controller_v2_P.DirtyDerivative12_A[7])*
      posing_controller_v2_DWork.DirtyDerivative12_DSTATE[3];
    xnew[1] += (posing_controller_v2_P.DirtyDerivative12_B[1])*rtb_q_um[12];
    xnew[2] = (posing_controller_v2_P.DirtyDerivative12_A[8])*
      posing_controller_v2_DWork.DirtyDerivative12_DSTATE[0]
      + (posing_controller_v2_P.DirtyDerivative12_A[9])*
      posing_controller_v2_DWork.DirtyDerivative12_DSTATE[1]
      + (posing_controller_v2_P.DirtyDerivative12_A[10])*
      posing_controller_v2_DWork.DirtyDerivative12_DSTATE[2]
      + (posing_controller_v2_P.DirtyDerivative12_A[11])*
      posing_controller_v2_DWork.DirtyDerivative12_DSTATE[3];
    xnew[2] += (posing_controller_v2_P.DirtyDerivative12_B[2])*rtb_q_um[12];
    xnew[3] = (posing_controller_v2_P.DirtyDerivative12_A[12])*
      posing_controller_v2_DWork.DirtyDerivative12_DSTATE[0]
      + (posing_controller_v2_P.DirtyDerivative12_A[13])*
      posing_controller_v2_DWork.DirtyDerivative12_DSTATE[1]
      + (posing_controller_v2_P.DirtyDerivative12_A[14])*
      posing_controller_v2_DWork.DirtyDerivative12_DSTATE[2]
      + (posing_controller_v2_P.DirtyDerivative12_A[15])*
      posing_controller_v2_DWork.DirtyDerivative12_DSTATE[3];
    xnew[3] += (posing_controller_v2_P.DirtyDerivative12_B[3])*rtb_q_um[12];
    (void) memcpy(&posing_controller_v2_DWork.DirtyDerivative12_DSTATE[0], xnew,
                  sizeof(real_T)*4);
  }

  /* Update for DiscreteFilter: '<Root>/Discrete Filter' */
  posing_controller_v2_DWork.DiscreteFilter_DSTATE = DiscreteFilter_tmp;

  /* Update for DiscreteFilter: '<Root>/Discrete Filter1' */
  posing_controller_v2_DWork.DiscreteFilter1_DSTATE = DiscreteFilter1_tmp;

  /* Update for DiscreteFilter: '<Root>/Discrete Filter2' */
  posing_controller_v2_DWork.DiscreteFilter2_DSTATE = DiscreteFilter2_tmp;

  /* Update for DiscreteFilter: '<Root>/Discrete Filter3' */
  posing_controller_v2_DWork.DiscreteFilter3_DSTATE = DiscreteFilter3_tmp;

  /* Update for DiscreteFilter: '<Root>/Discrete Filter4' */
  posing_controller_v2_DWork.DiscreteFilter4_DSTATE = DiscreteFilter4_tmp;

  /* Update for DiscreteFilter: '<Root>/Discrete Filter5' */
  posing_controller_v2_DWork.DiscreteFilter5_DSTATE = DiscreteFilter5_tmp;
}

/* Model initialize function */
void posing_controller_v2_initialize(void)
{
  /* Registration code */

  /* initialize non-finites */
  rt_InitInfAndNaN(sizeof(real_T));

  /* initialize error status */
  rtmSetErrorStatus(posing_controller_v2_M, (NULL));

  /* states (dwork) */
  (void) memset((void *)&posing_controller_v2_DWork, 0,
                sizeof(D_Work_posing_controller_v2));

  /* external inputs */
  (void) memset((void *)&posing_controller_v2_U, 0,
                sizeof(ExternalInputs_posing_controlle));

  /* external outputs */
  (void) memset((void *)&posing_controller_v2_Y, 0,
                sizeof(ExternalOutputs_posing_controll));

  /* InitializeConditions for DiscreteStateSpace: '<S1>/DirtyDerivative' */
  posing_controller_v2_DWork.DirtyDerivative_DSTATE[0] =
    posing_controller_v2_P.DirtyDerivative_X0[0];
  posing_controller_v2_DWork.DirtyDerivative_DSTATE[1] =
    posing_controller_v2_P.DirtyDerivative_X0[1];
  posing_controller_v2_DWork.DirtyDerivative_DSTATE[2] =
    posing_controller_v2_P.DirtyDerivative_X0[2];
  posing_controller_v2_DWork.DirtyDerivative_DSTATE[3] =
    posing_controller_v2_P.DirtyDerivative_X0[3];

  /* InitializeConditions for DiscreteStateSpace: '<S1>/DirtyDerivative1' */
  posing_controller_v2_DWork.DirtyDerivative1_DSTATE[0] =
    posing_controller_v2_P.DirtyDerivative1_X0[0];
  posing_controller_v2_DWork.DirtyDerivative1_DSTATE[1] =
    posing_controller_v2_P.DirtyDerivative1_X0[1];
  posing_controller_v2_DWork.DirtyDerivative1_DSTATE[2] =
    posing_controller_v2_P.DirtyDerivative1_X0[2];
  posing_controller_v2_DWork.DirtyDerivative1_DSTATE[3] =
    posing_controller_v2_P.DirtyDerivative1_X0[3];

  /* InitializeConditions for DiscreteStateSpace: '<S1>/DirtyDerivative2' */
  posing_controller_v2_DWork.DirtyDerivative2_DSTATE[0] =
    posing_controller_v2_P.DirtyDerivative2_X0[0];
  posing_controller_v2_DWork.DirtyDerivative2_DSTATE[1] =
    posing_controller_v2_P.DirtyDerivative2_X0[1];
  posing_controller_v2_DWork.DirtyDerivative2_DSTATE[2] =
    posing_controller_v2_P.DirtyDerivative2_X0[2];
  posing_controller_v2_DWork.DirtyDerivative2_DSTATE[3] =
    posing_controller_v2_P.DirtyDerivative2_X0[3];

  /* InitializeConditions for DiscreteStateSpace: '<S1>/DirtyDerivative3' */
  posing_controller_v2_DWork.DirtyDerivative3_DSTATE[0] =
    posing_controller_v2_P.DirtyDerivative3_X0[0];
  posing_controller_v2_DWork.DirtyDerivative3_DSTATE[1] =
    posing_controller_v2_P.DirtyDerivative3_X0[1];
  posing_controller_v2_DWork.DirtyDerivative3_DSTATE[2] =
    posing_controller_v2_P.DirtyDerivative3_X0[2];
  posing_controller_v2_DWork.DirtyDerivative3_DSTATE[3] =
    posing_controller_v2_P.DirtyDerivative3_X0[3];

  /* InitializeConditions for DiscreteStateSpace: '<S1>/DirtyDerivative4' */
  posing_controller_v2_DWork.DirtyDerivative4_DSTATE[0] =
    posing_controller_v2_P.DirtyDerivative4_X0[0];
  posing_controller_v2_DWork.DirtyDerivative4_DSTATE[1] =
    posing_controller_v2_P.DirtyDerivative4_X0[1];
  posing_controller_v2_DWork.DirtyDerivative4_DSTATE[2] =
    posing_controller_v2_P.DirtyDerivative4_X0[2];
  posing_controller_v2_DWork.DirtyDerivative4_DSTATE[3] =
    posing_controller_v2_P.DirtyDerivative4_X0[3];

  /* InitializeConditions for DiscreteStateSpace: '<S1>/DirtyDerivative5' */
  posing_controller_v2_DWork.DirtyDerivative5_DSTATE[0] =
    posing_controller_v2_P.DirtyDerivative5_X0[0];
  posing_controller_v2_DWork.DirtyDerivative5_DSTATE[1] =
    posing_controller_v2_P.DirtyDerivative5_X0[1];
  posing_controller_v2_DWork.DirtyDerivative5_DSTATE[2] =
    posing_controller_v2_P.DirtyDerivative5_X0[2];
  posing_controller_v2_DWork.DirtyDerivative5_DSTATE[3] =
    posing_controller_v2_P.DirtyDerivative5_X0[3];

  /* InitializeConditions for DiscreteStateSpace: '<S1>/DirtyDerivative6' */
  posing_controller_v2_DWork.DirtyDerivative6_DSTATE[0] =
    posing_controller_v2_P.DirtyDerivative6_X0[0];
  posing_controller_v2_DWork.DirtyDerivative6_DSTATE[1] =
    posing_controller_v2_P.DirtyDerivative6_X0[1];
  posing_controller_v2_DWork.DirtyDerivative6_DSTATE[2] =
    posing_controller_v2_P.DirtyDerivative6_X0[2];
  posing_controller_v2_DWork.DirtyDerivative6_DSTATE[3] =
    posing_controller_v2_P.DirtyDerivative6_X0[3];

  /* InitializeConditions for DiscreteStateSpace: '<S1>/DirtyDerivative7' */
  posing_controller_v2_DWork.DirtyDerivative7_DSTATE[0] =
    posing_controller_v2_P.DirtyDerivative7_X0[0];
  posing_controller_v2_DWork.DirtyDerivative7_DSTATE[1] =
    posing_controller_v2_P.DirtyDerivative7_X0[1];
  posing_controller_v2_DWork.DirtyDerivative7_DSTATE[2] =
    posing_controller_v2_P.DirtyDerivative7_X0[2];
  posing_controller_v2_DWork.DirtyDerivative7_DSTATE[3] =
    posing_controller_v2_P.DirtyDerivative7_X0[3];

  /* InitializeConditions for DiscreteStateSpace: '<S1>/DirtyDerivative8' */
  posing_controller_v2_DWork.DirtyDerivative8_DSTATE[0] =
    posing_controller_v2_P.DirtyDerivative8_X0[0];
  posing_controller_v2_DWork.DirtyDerivative8_DSTATE[1] =
    posing_controller_v2_P.DirtyDerivative8_X0[1];
  posing_controller_v2_DWork.DirtyDerivative8_DSTATE[2] =
    posing_controller_v2_P.DirtyDerivative8_X0[2];
  posing_controller_v2_DWork.DirtyDerivative8_DSTATE[3] =
    posing_controller_v2_P.DirtyDerivative8_X0[3];

  /* InitializeConditions for DiscreteStateSpace: '<S1>/DirtyDerivative9' */
  posing_controller_v2_DWork.DirtyDerivative9_DSTATE[0] =
    posing_controller_v2_P.DirtyDerivative9_X0[0];
  posing_controller_v2_DWork.DirtyDerivative9_DSTATE[1] =
    posing_controller_v2_P.DirtyDerivative9_X0[1];
  posing_controller_v2_DWork.DirtyDerivative9_DSTATE[2] =
    posing_controller_v2_P.DirtyDerivative9_X0[2];
  posing_controller_v2_DWork.DirtyDerivative9_DSTATE[3] =
    posing_controller_v2_P.DirtyDerivative9_X0[3];

  /* InitializeConditions for DiscreteStateSpace: '<S1>/DirtyDerivative10' */
  posing_controller_v2_DWork.DirtyDerivative10_DSTATE[0] =
    posing_controller_v2_P.DirtyDerivative10_X0[0];
  posing_controller_v2_DWork.DirtyDerivative10_DSTATE[1] =
    posing_controller_v2_P.DirtyDerivative10_X0[1];
  posing_controller_v2_DWork.DirtyDerivative10_DSTATE[2] =
    posing_controller_v2_P.DirtyDerivative10_X0[2];
  posing_controller_v2_DWork.DirtyDerivative10_DSTATE[3] =
    posing_controller_v2_P.DirtyDerivative10_X0[3];

  /* InitializeConditions for DiscreteStateSpace: '<S1>/DirtyDerivative11' */
  posing_controller_v2_DWork.DirtyDerivative11_DSTATE[0] =
    posing_controller_v2_P.DirtyDerivative11_X0[0];
  posing_controller_v2_DWork.DirtyDerivative11_DSTATE[1] =
    posing_controller_v2_P.DirtyDerivative11_X0[1];
  posing_controller_v2_DWork.DirtyDerivative11_DSTATE[2] =
    posing_controller_v2_P.DirtyDerivative11_X0[2];
  posing_controller_v2_DWork.DirtyDerivative11_DSTATE[3] =
    posing_controller_v2_P.DirtyDerivative11_X0[3];

  /* InitializeConditions for DiscreteStateSpace: '<S1>/DirtyDerivative12' */
  posing_controller_v2_DWork.DirtyDerivative12_DSTATE[0] =
    posing_controller_v2_P.DirtyDerivative12_X0[0];
  posing_controller_v2_DWork.DirtyDerivative12_DSTATE[1] =
    posing_controller_v2_P.DirtyDerivative12_X0[1];
  posing_controller_v2_DWork.DirtyDerivative12_DSTATE[2] =
    posing_controller_v2_P.DirtyDerivative12_X0[2];
  posing_controller_v2_DWork.DirtyDerivative12_DSTATE[3] =
    posing_controller_v2_P.DirtyDerivative12_X0[3];

  /* InitializeConditions for DiscreteFilter: '<Root>/Discrete Filter' */
  posing_controller_v2_DWork.DiscreteFilter_DSTATE =
    posing_controller_v2_P.DiscreteFilter_InitialStates;

  /* InitializeConditions for DiscreteFilter: '<Root>/Discrete Filter1' */
  posing_controller_v2_DWork.DiscreteFilter1_DSTATE =
    posing_controller_v2_P.DiscreteFilter1_InitialStates;

  /* InitializeConditions for DiscreteFilter: '<Root>/Discrete Filter2' */
  posing_controller_v2_DWork.DiscreteFilter2_DSTATE =
    posing_controller_v2_P.DiscreteFilter2_InitialStates;

  /* InitializeConditions for DiscreteFilter: '<Root>/Discrete Filter3' */
  posing_controller_v2_DWork.DiscreteFilter3_DSTATE =
    posing_controller_v2_P.DiscreteFilter3_InitialStates;

  /* InitializeConditions for DiscreteFilter: '<Root>/Discrete Filter4' */
  posing_controller_v2_DWork.DiscreteFilter4_DSTATE =
    posing_controller_v2_P.DiscreteFilter4_InitialStates;

  /* InitializeConditions for DiscreteFilter: '<Root>/Discrete Filter5' */
  posing_controller_v2_DWork.DiscreteFilter5_DSTATE =
    posing_controller_v2_P.DiscreteFilter5_InitialStates;
}

/* Model terminate function */
void posing_controller_v2_terminate(void)
{
  /* (no terminate code required) */
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
