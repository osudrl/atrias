/*
 * File: vc_controller_1.c
 *
 * Code generated for Simulink model 'vc_controller_1'.
 *
 * Model version                  : 1.28
 * Simulink Coder version         : 8.2 (R2012a) 29-Dec-2011
 * TLC version                    : 8.2 (Jan 25 2012)
 * C/C++ source code generated on : Tue Sep 18 19:08:43 2012
 *
 * Target selection: ert_shrlib.tlc
 * Embedded hardware selection: 32-bit Generic
 * Emulation hardware selection:
 *    Differs from embedded hardware (MATLAB Host)
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#include "vc_controller_1.h"
#include "vc_controller_1_private.h"

/* Block signals (auto storage) */
BlockIO_vc_controller_1 vc_controller_1_B;

/* Block states (auto storage) */
D_Work_vc_controller_1 vc_controller_1_DWork;

/* External inputs (root inport signals with auto storage) */
ExternalInputs_vc_controller_1 vc_controller_1_U;

/* External outputs (root outports fed by signals with auto storage) */
ExternalOutputs_vc_controller_1 vc_controller_1_Y;

/* Real-time model */
RT_MODEL_vc_controller_1 vc_controller_1_M_;
RT_MODEL_vc_controller_1 *const vc_controller_1_M = &vc_controller_1_M_;

/* Forward declaration for local functions */
static void vc__LagrangeModelAtriasFlight2D(const real_T q[11], const real_T dq
  [11], real_T D[121], real_T C[121], real_T G[11], real_T B[44], real_T
  DampingSpringTorque[11]);
static void vc_controller_1_ATRIAS2D_SS_s(const real_T q[11], const real_T dq[11],
  const real_T theta_limits[2], real_T leg, real_T *s, real_T *ds, real_T *th,
  real_T *dth, real_T *delta_theta, real_T c[11], real_T dsdq[11]);
static void vc_controller_1_invNxN(const real_T x[121], real_T y[121]);
static void vc_controller_1_bezierd(const real_T afra[24], real_T s, real_T
  value[4]);
static void vc_controller_1_mldivide(const real_T A[16], real_T B[4]);
static void ATRIAS2D_SS_feedback_control_Ro(real_T t, const real_T q[11], const
  real_T dq[11], const real_T D[121], const real_T H[11], const real_T B[44],
  const real_T theta_limits[2], real_T h_alpha[24], const real_T poly_cor[20],
  const real_T kp[2], const real_T kd[2], real_T pdCtrl, real_T pdPlusFwdCtrl,
  real_T L2fhCtrl, real_T stance_leg, real_T a3scuff, real_T a4scuff, uint8_T
  s_mode, real_T s_freq, real_T u[4], real_T y[4], real_T dy[4], real_T *s_out,
  real_T *ds);

/* Function for MATLAB Function: '<Root>/controller' */
static void vc__LagrangeModelAtriasFlight2D(const real_T q[11], const real_T dq
  [11], real_T D[121], real_T C[121], real_T G[11], real_T B[44], real_T
  DampingSpringTorque[11])
{
  int32_T i;

  /* %%%%%  LagrangeModelAtriasFlight2D.m */
  /* %%%  05-Sep-2012 17:49:43 */
  /* %%% */
  /* %%% Authors(s): Grizzle */
  /* %%% */
  /* %%% Model NOTATION: Spong and Vidyasagar, page 142, Eq. (6.3.12) */
  /* %%%                 D(q)ddq + C(q,dq)*dq + G(q) + DampingSpringTorque = B*tau + EL*GroundReactionForceLeft + ER*GroundReactionForceRight */
  /* %%% */
  /* %%%[g mTotal L1 L2 L3 L4 m1 m2 m3 m4 Jcm1 Jcm2 Jcm3 Jcm4 ellzcm1 ellzcm2 ellzcm3 ellzcm4 ellycm1 ellycm2 ellycm3 ellycm4 LT mT JcmT ellzcmT ellycmT mH JcmH K1 K2 Kd1 Kd2 Jrotor1 Jrotor2 Jgear1  Jgear2 R1  R2] = modelParametersAtrias_v05; */
  /* %%% */
  for (i = 0; i < 121; i++) {
    D[i] = 0.0;
    C[i] = 0.0;
  }

  D[0] = 64.43119999999999;
  D[22] = ((((((((((((((((((((-21.058218 * cos(q[2]) - 0.63042300000000007 * sin
    (q[2])) - cos(q[3] + q[2]) * 0.086067) - cos(q[7] + q[2]) * 0.086067) - cos
    (q[4] + q[2]) * 0.212465) - cos(q[8] + q[2]) * 0.212465) - cos(q[3] + q[2]) *
    0.094923815) - cos(q[7] + q[2]) * 0.094923815) - cos(q[4] + q[2]) *
                       0.124428024) - cos(q[8] + q[2]) * 0.124428024) - cos(q[4]
    + q[2]) * 0.021746262000000002) - cos(q[3] + q[2]) * 0.0646021079) - cos(q[8]
    + q[2]) * 0.021746262000000002) - cos(q[7] + q[2]) * 0.0646021079) - sin(q[3]
    + q[2]) * -0.016735697) - sin(q[7] + q[2]) * -0.016735697) - sin(q[4] + q[2])
               * 0.010721844) - sin(q[8] + q[2]) * 0.010721844) - sin(q[4] + q[2])
             * 0.0) - sin(q[3] + q[2]) * 0.0) - sin(q[8] + q[2]) * 0.0) - sin(q
    [7] + q[2]) * 0.0;
  D[33] = (((cos(q[3] + q[2]) * -0.086067 - cos(q[3] + q[2]) * 0.094923815) -
            cos(q[3] + q[2]) * 0.0646021079) - sin(q[3] + q[2]) * -0.016735697)
    - sin(q[3] + q[2]) * 0.0;
  D[44] = (((cos(q[4] + q[2]) * -0.212465 - cos(q[4] + q[2]) * 0.124428024) -
            cos(q[4] + q[2]) * 0.021746262000000002) - sin(q[4] + q[2]) *
           0.010721844) - sin(q[4] + q[2]) * 0.0;
  D[77] = (((cos(q[7] + q[2]) * -0.086067 - cos(q[7] + q[2]) * 0.094923815) -
            cos(q[7] + q[2]) * 0.0646021079) - sin(q[7] + q[2]) * -0.016735697)
    - sin(q[7] + q[2]) * 0.0;
  D[88] = (((cos(q[8] + q[2]) * -0.212465 - cos(q[8] + q[2]) * 0.124428024) -
            cos(q[8] + q[2]) * 0.021746262000000002) - sin(q[8] + q[2]) *
           0.010721844) - sin(q[8] + q[2]) * 0.0;
  D[12] = 64.43119999999999;
  D[23] = ((((((((((((((((((((0.63042300000000007 * cos(q[2]) - 21.058218 * sin
    (q[2])) - sin(q[3] + q[2]) * 0.086067) - sin(q[7] + q[2]) * 0.086067) - sin
    (q[4] + q[2]) * 0.212465) - sin(q[8] + q[2]) * 0.212465) + cos(q[3] + q[2]) *
    -0.016735697) + cos(q[7] + q[2]) * -0.016735697) + cos(q[4] + q[2]) *
                       0.010721844) + cos(q[8] + q[2]) * 0.010721844) + cos(q[4]
    + q[2]) * 0.0) + cos(q[3] + q[2]) * 0.0) + cos(q[8] + q[2]) * 0.0) + cos(q[7]
    + q[2]) * 0.0) - sin(q[3] + q[2]) * 0.094923815) - sin(q[7] + q[2]) *
                0.094923815) - sin(q[4] + q[2]) * 0.124428024) - sin(q[8] + q[2])
              * 0.124428024) - sin(q[4] + q[2]) * 0.021746262000000002) - sin(q
             [3] + q[2]) * 0.0646021079) - sin(q[8] + q[2]) *
           0.021746262000000002) - sin(q[7] + q[2]) * 0.0646021079;
  D[34] = (((cos(q[3] + q[2]) * -0.016735697 - sin(q[3] + q[2]) * 0.086067) +
            cos(q[3] + q[2]) * 0.0) - sin(q[3] + q[2]) * 0.094923815) - sin(q[3]
    + q[2]) * 0.0646021079;
  D[45] = (((cos(q[4] + q[2]) * 0.010721844 - sin(q[4] + q[2]) * 0.212465) + cos
            (q[4] + q[2]) * 0.0) - sin(q[4] + q[2]) * 0.124428024) - sin(q[4] +
    q[2]) * 0.021746262000000002;
  D[78] = (((cos(q[7] + q[2]) * -0.016735697 - sin(q[7] + q[2]) * 0.086067) +
            cos(q[7] + q[2]) * 0.0) - sin(q[7] + q[2]) * 0.094923815) - sin(q[7]
    + q[2]) * 0.0646021079;
  D[89] = (((cos(q[8] + q[2]) * 0.010721844 - sin(q[8] + q[2]) * 0.212465) + cos
            (q[8] + q[2]) * 0.0) - sin(q[8] + q[2]) * 0.124428024) - sin(q[8] +
    q[2]) * 0.021746262000000002;
  D[2] = ((((((((((((((((((((-21.058218 * cos(q[2]) - 0.63042300000000007 * sin
    (q[2])) - cos(q[3] + q[2]) * 0.086067) - cos(q[7] + q[2]) * 0.086067) - cos
    (q[4] + q[2]) * 0.212465) - cos(q[8] + q[2]) * 0.212465) - cos(q[3] + q[2]) *
                        0.094923815) - cos(q[7] + q[2]) * 0.094923815) - cos(q[4]
    + q[2]) * 0.124428024) - cos(q[8] + q[2]) * 0.124428024) - cos(q[4] + q[2]) *
                    0.021746262000000002) - cos(q[3] + q[2]) * 0.0646021079) -
                  cos(q[8] + q[2]) * 0.021746262000000002) - cos(q[7] + q[2]) *
                 0.0646021079) - sin(q[3] + q[2]) * -0.016735697) - sin(q[7] +
    q[2]) * -0.016735697) - sin(q[4] + q[2]) * 0.010721844) - sin(q[8] + q[2]) *
             0.010721844) - sin(q[4] + q[2]) * 0.0) - sin(q[3] + q[2]) * 0.0) -
          sin(q[8] + q[2]) * 0.0) - sin(q[7] + q[2]) * 0.0;
  D[13] = ((((((((((((((((((((0.63042300000000007 * cos(q[2]) - 21.058218 * sin
    (q[2])) - sin(q[3] + q[2]) * 0.086067) - sin(q[7] + q[2]) * 0.086067) - sin
    (q[4] + q[2]) * 0.212465) - sin(q[8] + q[2]) * 0.212465) + cos(q[3] + q[2]) *
    -0.016735697) + cos(q[7] + q[2]) * -0.016735697) + cos(q[4] + q[2]) *
                       0.010721844) + cos(q[8] + q[2]) * 0.010721844) + cos(q[4]
    + q[2]) * 0.0) + cos(q[3] + q[2]) * 0.0) + cos(q[8] + q[2]) * 0.0) + cos(q[7]
    + q[2]) * 0.0) - sin(q[3] + q[2]) * 0.094923815) - sin(q[7] + q[2]) *
                0.094923815) - sin(q[4] + q[2]) * 0.124428024) - sin(q[8] + q[2])
              * 0.124428024) - sin(q[4] + q[2]) * 0.021746262000000002) - sin(q
             [3] + q[2]) * 0.0646021079) - sin(q[8] + q[2]) *
           0.021746262000000002) - sin(q[7] + q[2]) * 0.0646021079;
  D[24] = (((((((cos(q[3] - q[4]) * 0.0195716358 + 15.259525855027274) + cos(q[7]
    - q[8]) * 0.0195716358) + cos(q[3] - q[4]) * 0.0646021079) + cos(q[7] - q[8])
              * 0.0646021079) - sin(q[3] - q[4]) * 0.0) - sin(q[7] - q[8]) * 0.0)
           + sin(q[3] - q[4]) * 0.0) + sin(q[7] - q[8]) * 0.0;
  D[35] = (((cos(q[3] - q[4]) * 0.0097858179 + 0.097996589050637) + cos(q[3] -
             q[4]) * 0.03230105395) - sin(q[3] - q[4]) * 0.0) + sin(q[3] - q[4])
    * 0.0;
  D[46] = (((cos(q[3] - q[4]) * 0.0097858179 + 0.18014416891299997) + cos(q[3] -
             q[4]) * 0.03230105395) - sin(q[3] - q[4]) * 0.0) + sin(q[3] - q[4])
    * 0.0;
  D[57] = 0.037853769999999995;
  D[68] = 0.037853769999999995;
  D[79] = (((cos(q[7] - q[8]) * 0.0097858179 + 0.097996589050637) + cos(q[7] -
             q[8]) * 0.03230105395) - sin(q[7] - q[8]) * 0.0) + sin(q[7] - q[8])
    * 0.0;
  D[90] = (((cos(q[7] - q[8]) * 0.0097858179 + 0.18014416891299997) + cos(q[7] -
             q[8]) * 0.03230105395) - sin(q[7] - q[8]) * 0.0) + sin(q[7] - q[8])
    * 0.0;
  D[101] = 0.037853769999999995;
  D[112] = 0.037853769999999995;
  D[3] = (((cos(q[3] + q[2]) * -0.086067 - cos(q[3] + q[2]) * 0.094923815) - cos
           (q[3] + q[2]) * 0.0646021079) - sin(q[3] + q[2]) * -0.016735697) -
    sin(q[3] + q[2]) * 0.0;
  D[14] = (((cos(q[3] + q[2]) * -0.016735697 - sin(q[3] + q[2]) * 0.086067) +
            cos(q[3] + q[2]) * 0.0) - sin(q[3] + q[2]) * 0.094923815) - sin(q[3]
    + q[2]) * 0.0646021079;
  D[25] = (((cos(q[3] - q[4]) * 0.0097858179 + 0.097996589050637) + cos(q[3] -
             q[4]) * 0.03230105395) - sin(q[3] - q[4]) * 0.0) + sin(q[3] - q[4])
    * 0.0;
  D[36] = 0.097996589050637;
  D[47] = ((cos(q[3] - q[4]) * 0.0097858179 + cos(q[3] - q[4]) * 0.03230105395)
           - sin(q[3] - q[4]) * 0.0) + sin(q[3] - q[4]) * 0.0;
  D[4] = (((cos(q[4] + q[2]) * -0.212465 - cos(q[4] + q[2]) * 0.124428024) - cos
           (q[4] + q[2]) * 0.021746262000000002) - sin(q[4] + q[2]) *
          0.010721844) - sin(q[4] + q[2]) * 0.0;
  D[15] = (((cos(q[4] + q[2]) * 0.010721844 - sin(q[4] + q[2]) * 0.212465) + cos
            (q[4] + q[2]) * 0.0) - sin(q[4] + q[2]) * 0.124428024) - sin(q[4] +
    q[2]) * 0.021746262000000002;
  D[26] = (((cos(q[3] - q[4]) * 0.0097858179 + 0.18014416891299997) + cos(q[3] -
             q[4]) * 0.03230105395) - sin(q[3] - q[4]) * 0.0) + sin(q[3] - q[4])
    * 0.0;
  D[37] = ((cos(q[3] - q[4]) * 0.0097858179 + cos(q[3] - q[4]) * 0.03230105395)
           - sin(q[3] - q[4]) * 0.0) + sin(q[3] - q[4]) * 0.0;
  D[48] = 0.180144168913;
  D[27] = 0.037853769999999995;
  D[60] = 1.75285377;
  D[28] = 0.037853769999999995;
  D[72] = 1.75285377;
  D[7] = (((cos(q[7] + q[2]) * -0.086067 - cos(q[7] + q[2]) * 0.094923815) - cos
           (q[7] + q[2]) * 0.0646021079) - sin(q[7] + q[2]) * -0.016735697) -
    sin(q[7] + q[2]) * 0.0;
  D[18] = (((cos(q[7] + q[2]) * -0.016735697 - sin(q[7] + q[2]) * 0.086067) +
            cos(q[7] + q[2]) * 0.0) - sin(q[7] + q[2]) * 0.094923815) - sin(q[7]
    + q[2]) * 0.0646021079;
  D[29] = (((cos(q[7] - q[8]) * 0.0097858179 + 0.097996589050637) + cos(q[7] -
             q[8]) * 0.03230105395) - sin(q[7] - q[8]) * 0.0) + sin(q[7] - q[8])
    * 0.0;
  D[84] = 0.097996589050637;
  D[95] = ((cos(q[7] - q[8]) * 0.0097858179 + cos(q[7] - q[8]) * 0.03230105395)
           - sin(q[7] - q[8]) * 0.0) + sin(q[7] - q[8]) * 0.0;
  D[8] = (((cos(q[8] + q[2]) * -0.212465 - cos(q[8] + q[2]) * 0.124428024) - cos
           (q[8] + q[2]) * 0.021746262000000002) - sin(q[8] + q[2]) *
          0.010721844) - sin(q[8] + q[2]) * 0.0;
  D[19] = (((cos(q[8] + q[2]) * 0.010721844 - sin(q[8] + q[2]) * 0.212465) + cos
            (q[8] + q[2]) * 0.0) - sin(q[8] + q[2]) * 0.124428024) - sin(q[8] +
    q[2]) * 0.021746262000000002;
  D[30] = (((cos(q[7] - q[8]) * 0.0097858179 + 0.18014416891299997) + cos(q[7] -
             q[8]) * 0.03230105395) - sin(q[7] - q[8]) * 0.0) + sin(q[7] - q[8])
    * 0.0;
  D[85] = ((cos(q[7] - q[8]) * 0.0097858179 + cos(q[7] - q[8]) * 0.03230105395)
           - sin(q[7] - q[8]) * 0.0) + sin(q[7] - q[8]) * 0.0;
  D[96] = 0.180144168913;
  D[31] = 0.037853769999999995;
  D[108] = 1.75285377;
  D[32] = 0.037853769999999995;
  D[120] = 1.75285377;

  /* %%% */
  C[22] = (((((((((((((((((((((21.058218 * sin(q[2]) - 0.63042300000000007 * cos
    (q[2])) + sin(q[3] + q[2]) * 0.086067) + sin(q[7] + q[2]) * 0.086067) + sin
    (q[4] + q[2]) * 0.212465) + sin(q[8] + q[2]) * 0.212465) - cos(q[3] + q[2]) *
    -0.016735697) - cos(q[7] + q[2]) * -0.016735697) - cos(q[4] + q[2]) *
                        0.010721844) - cos(q[8] + q[2]) * 0.010721844) - cos(q[4]
    + q[2]) * 0.0) - cos(q[3] + q[2]) * 0.0) - cos(q[8] + q[2]) * 0.0) - cos(q[7]
    + q[2]) * 0.0) + sin(q[3] + q[2]) * 0.094923815) + sin(q[7] + q[2]) *
                 0.094923815) + sin(q[4] + q[2]) * 0.124428024) + sin(q[8] + q[2])
               * 0.124428024) + sin(q[4] + q[2]) * 0.021746262000000002) + sin
             (q[3] + q[2]) * 0.0646021079) + sin(q[8] + q[2]) *
            0.021746262000000002) + sin(q[7] + q[2]) * 0.0646021079) * dq[2] +
    (((((((sin(q[3] + q[2]) * 0.086067 - cos(q[3] + q[2]) * -0.016735697) - cos
          (q[3] + q[2]) * 0.0) + sin(q[3] + q[2]) * 0.094923815) + sin(q[3] + q
         [2]) * 0.0646021079) * dq[3] + ((((sin(q[7] + q[2]) * 0.086067 - cos(q
            [7] + q[2]) * -0.016735697) - cos(q[7] + q[2]) * 0.0) + sin(q[7] +
          q[2]) * 0.094923815) + sin(q[7] + q[2]) * 0.0646021079) * dq[7]) +
      ((((sin(q[4] + q[2]) * 0.212465 - cos(q[4] + q[2]) * 0.010721844) - cos(q
          [4] + q[2]) * 0.0) + sin(q[4] + q[2]) * 0.124428024) + sin(q[4] + q[2])
       * 0.021746262000000002) * dq[4]) + ((((sin(q[8] + q[2]) * 0.212465 - cos
         (q[8] + q[2]) * 0.010721844) - cos(q[8] + q[2]) * 0.0) + sin(q[8] + q[2])
       * 0.124428024) + sin(q[8] + q[2]) * 0.021746262000000002) * dq[8]);
  C[33] = ((((sin(q[3] + q[2]) * 0.086067 - cos(q[3] + q[2]) * -0.016735697) -
             cos(q[3] + q[2]) * 0.0) + sin(q[3] + q[2]) * 0.094923815) + sin(q[3]
            + q[2]) * 0.0646021079) * (dq[3] + dq[2]);
  C[44] = ((((sin(q[4] + q[2]) * 0.212465 - cos(q[4] + q[2]) * 0.010721844) -
             cos(q[4] + q[2]) * 0.0) + sin(q[4] + q[2]) * 0.124428024) + sin(q[4]
            + q[2]) * 0.021746262000000002) * (dq[4] + dq[2]);
  C[77] = ((((sin(q[7] + q[2]) * 0.086067 - cos(q[7] + q[2]) * -0.016735697) -
             cos(q[7] + q[2]) * 0.0) + sin(q[7] + q[2]) * 0.094923815) + sin(q[7]
            + q[2]) * 0.0646021079) * (dq[7] + dq[2]);
  C[88] = ((((sin(q[8] + q[2]) * 0.212465 - cos(q[8] + q[2]) * 0.010721844) -
             cos(q[8] + q[2]) * 0.0) + sin(q[8] + q[2]) * 0.124428024) + sin(q[8]
            + q[2]) * 0.021746262000000002) * (dq[8] + dq[2]);
  C[23] = ((((((((((((((((((((((((21.058218 * cos(q[2]) + 0.63042300000000007 *
    sin(q[2])) + cos(q[3] + q[2]) * 0.086067) + cos(q[7] + q[2]) * 0.086067) +
    cos(q[4] + q[2]) * 0.212465) + cos(q[8] + q[2]) * 0.212465) + cos(q[3] + q[2])
    * 0.094923815) + cos(q[7] + q[2]) * 0.094923815) + cos(q[4] + q[2]) *
    0.124428024) + cos(q[8] + q[2]) * 0.124428024) + cos(q[4] + q[2]) *
    0.021746262000000002) + cos(q[3] + q[2]) * 0.0646021079) + cos(q[8] + q[2]) *
                       0.021746262000000002) + cos(q[7] + q[2]) * 0.0646021079)
                     + sin(q[3] + q[2]) * -0.016735697) + sin(q[7] + q[2]) *
                    -0.016735697) + sin(q[4] + q[2]) * 0.010721844) + sin(q[8] +
    q[2]) * 0.010721844) + sin(q[4] + q[2]) * 0.0) + sin(q[3] + q[2]) * 0.0) +
               sin(q[8] + q[2]) * 0.0) + sin(q[7] + q[2]) * 0.0) * -dq[2] -
             ((((cos(q[3] + q[2]) * 0.086067 + cos(q[3] + q[2]) * 0.094923815) +
                cos(q[3] + q[2]) * 0.0646021079) + sin(q[3] + q[2]) *
               -0.016735697) + sin(q[3] + q[2]) * 0.0) * dq[3]) - ((((cos(q[7] +
    q[2]) * 0.086067 + cos(q[7] + q[2]) * 0.094923815) + cos(q[7] + q[2]) *
    0.0646021079) + sin(q[7] + q[2]) * -0.016735697) + sin(q[7] + q[2]) * 0.0) *
            dq[7]) - ((((cos(q[4] + q[2]) * 0.212465 + cos(q[4] + q[2]) *
    0.124428024) + cos(q[4] + q[2]) * 0.021746262000000002) + sin(q[4] + q[2]) *
                       0.010721844) + sin(q[4] + q[2]) * 0.0) * dq[4]) - ((((cos
    (q[8] + q[2]) * 0.212465 + cos(q[8] + q[2]) * 0.124428024) + cos(q[8] + q[2])
    * 0.021746262000000002) + sin(q[8] + q[2]) * 0.010721844) + sin(q[8] + q[2])
    * 0.0) * dq[8];
  C[34] = ((((cos(q[3] + q[2]) * 0.086067 + cos(q[3] + q[2]) * 0.094923815) +
             cos(q[3] + q[2]) * 0.0646021079) + sin(q[3] + q[2]) * -0.016735697)
           + sin(q[3] + q[2]) * 0.0) * -(dq[3] + dq[2]);
  C[45] = ((((cos(q[4] + q[2]) * 0.212465 + cos(q[4] + q[2]) * 0.124428024) +
             cos(q[4] + q[2]) * 0.021746262000000002) + sin(q[4] + q[2]) *
            0.010721844) + sin(q[4] + q[2]) * 0.0) * -(dq[4] + dq[2]);
  C[78] = ((((cos(q[7] + q[2]) * 0.086067 + cos(q[7] + q[2]) * 0.094923815) +
             cos(q[7] + q[2]) * 0.0646021079) + sin(q[7] + q[2]) * -0.016735697)
           + sin(q[7] + q[2]) * 0.0) * -(dq[7] + dq[2]);
  C[89] = ((((cos(q[8] + q[2]) * 0.212465 + cos(q[8] + q[2]) * 0.124428024) +
             cos(q[8] + q[2]) * 0.021746262000000002) + sin(q[8] + q[2]) *
            0.010721844) + sin(q[8] + q[2]) * 0.0) * -(dq[8] + dq[2]);
  C[24] = (((((cos(q[3] - q[4]) * 0.0 - cos(q[3] - q[4]) * 0.0) + sin(q[3] - q[4])
              * 0.0097858179) + sin(q[3] - q[4]) * 0.03230105395) * dq[4] -
            (((cos(q[3] - q[4]) * 0.0 - cos(q[3] - q[4]) * 0.0) + sin(q[3] - q[4])
              * 0.0097858179) + sin(q[3] - q[4]) * 0.03230105395) * dq[3]) -
           (((cos(q[7] - q[8]) * 0.0 - cos(q[7] - q[8]) * 0.0) + sin(q[7] - q[8])
             * 0.0097858179) + sin(q[7] - q[8]) * 0.03230105395) * dq[7]) +
    (((cos(q[7] - q[8]) * 0.0 - cos(q[7] - q[8]) * 0.0) + sin(q[7] - q[8]) *
      0.0097858179) + sin(q[7] - q[8]) * 0.03230105395) * dq[8];
  C[35] = (((cos(q[3] - q[4]) * 0.0 - cos(q[3] - q[4]) * 0.0) + sin(q[3] - q[4])
            * 0.0097858179) + sin(q[3] - q[4]) * 0.03230105395) * -(dq[3] + dq[2]);
  C[46] = (((cos(q[3] - q[4]) * 0.0 - cos(q[3] - q[4]) * 0.0) + sin(q[3] - q[4])
            * 0.0097858179) + sin(q[3] - q[4]) * 0.03230105395) * (dq[4] + dq[2]);
  C[79] = (((cos(q[7] - q[8]) * 0.0 - cos(q[7] - q[8]) * 0.0) + sin(q[7] - q[8])
            * 0.0097858179) + sin(q[7] - q[8]) * 0.03230105395) * -(dq[7] + dq[2]);
  C[90] = (((cos(q[7] - q[8]) * 0.0 - cos(q[7] - q[8]) * 0.0) + sin(q[7] - q[8])
            * 0.0097858179) + sin(q[7] - q[8]) * 0.03230105395) * (dq[8] + dq[2]);
  C[25] = (((cos(q[3] - q[4]) * 0.0 - cos(q[3] - q[4]) * 0.0) + sin(q[3] - q[4])
            * 0.0097858179) + sin(q[3] - q[4]) * 0.03230105395) * (dq[4] + dq[2]);
  C[47] = (((cos(q[3] - q[4]) * 0.0 - cos(q[3] - q[4]) * 0.0) + sin(q[3] - q[4])
            * 0.0097858179) + sin(q[3] - q[4]) * 0.03230105395) * (dq[4] + dq[2]);
  C[26] = (((cos(q[3] - q[4]) * 0.0 - cos(q[3] - q[4]) * 0.0) + sin(q[3] - q[4])
            * 0.0097858179) + sin(q[3] - q[4]) * 0.03230105395) * -(dq[3] + dq[2]);
  C[37] = (((cos(q[3] - q[4]) * 0.0 - cos(q[3] - q[4]) * 0.0) + sin(q[3] - q[4])
            * 0.0097858179) + sin(q[3] - q[4]) * 0.03230105395) * -(dq[3] + dq[2]);
  C[29] = (((cos(q[7] - q[8]) * 0.0 - cos(q[7] - q[8]) * 0.0) + sin(q[7] - q[8])
            * 0.0097858179) + sin(q[7] - q[8]) * 0.03230105395) * (dq[8] + dq[2]);
  C[95] = (((cos(q[7] - q[8]) * 0.0 - cos(q[7] - q[8]) * 0.0) + sin(q[7] - q[8])
            * 0.0097858179) + sin(q[7] - q[8]) * 0.03230105395) * (dq[8] + dq[2]);
  C[30] = (((cos(q[7] - q[8]) * 0.0 - cos(q[7] - q[8]) * 0.0) + sin(q[7] - q[8])
            * 0.0097858179) + sin(q[7] - q[8]) * 0.03230105395) * -(dq[7] + dq[2]);
  C[85] = (((cos(q[7] - q[8]) * 0.0 - cos(q[7] - q[8]) * 0.0) + sin(q[7] - q[8])
            * 0.0097858179) + sin(q[7] - q[8]) * 0.03230105395) * -(dq[7] + dq[2]);

  /* %%% */
  memset(&G[0], 0, 11U * sizeof(real_T));
  G[0] = 0.0;
  G[1] = 632.070072;
  G[2] = ((((((((cos(q[3] + q[2]) * -0.0253 - sin(q[3] + q[2]) * 0.1435) *
                6.4892169000000006 + (cos(q[7] + q[2]) * -0.0253 - sin(q[7] + q
    [2]) * 0.1435) * 6.4892169000000006) + (cos(q[4] + q[2]) * 0.0157 - sin(q[4]
    + q[2]) * 0.1822) * 6.6994452) + (cos(q[8] + q[2]) * 0.0157 - sin(q[8] + q[2])
    * 0.1822) * 6.6994452) + (0.0181 * cos(q[2]) - 0.6046 * sin(q[2])) *
             341.6823) - ((sin(q[3] + q[2]) * 0.45 - cos(q[4] + q[2]) * 0.0) +
             sin(q[4] + q[2]) * 0.1137) * 1.8762606000000002) - ((sin(q[7] + q[2])
             * 0.45 - cos(q[8] + q[2]) * 0.0) + sin(q[8] + q[2]) * 0.1137) *
           1.8762606000000002) - ((sin(q[4] + q[2]) * 0.5 - cos(q[3] + q[2]) *
            0.0) + sin(q[3] + q[2]) * 0.15203) * 4.1685633) - ((sin(q[8] + q[2])
    * 0.5 - cos(q[7] + q[2]) * 0.0) + sin(q[7] + q[2]) * 0.15203) * 4.1685633;
  G[3] = (((cos(q[3] + q[2]) * -0.0253 - sin(q[3] + q[2]) * 0.1435) * 0.66149 +
           (cos(q[3] + q[2]) * 0.0 - sin(q[3] + q[2]) * 0.15203) * 0.42493) -
          sin(q[3] + q[2]) * 0.086067) * 9.81 + (2.0 * q[3] - 2.0 * q[5]) *
    1020.0 / 2.0;
  G[4] = (((cos(q[4] + q[2]) * 0.0157 - sin(q[4] + q[2]) * 0.1822) * 0.68292 +
           (cos(q[4] + q[2]) * 0.0 - sin(q[4] + q[2]) * 0.1137) * 0.19126) - sin
          (q[4] + q[2]) * 0.212465) * 9.81 + (2.0 * q[4] - 2.0 * q[6]) * 1320.0 /
    2.0;
  G[5] = (q[3] - q[5]) * -1020.0;
  G[6] = (q[4] - q[6]) * -1320.0;
  G[7] = (((cos(q[7] + q[2]) * -0.0253 - sin(q[7] + q[2]) * 0.1435) * 0.66149 +
           (cos(q[7] + q[2]) * 0.0 - sin(q[7] + q[2]) * 0.15203) * 0.42493) -
          sin(q[7] + q[2]) * 0.086067) * 9.81 + (2.0 * q[7] - 2.0 * q[9]) *
    1020.0 / 2.0;
  G[8] = (((cos(q[8] + q[2]) * 0.0157 - sin(q[8] + q[2]) * 0.1822) * 0.68292 +
           (cos(q[8] + q[2]) * 0.0 - sin(q[8] + q[2]) * 0.1137) * 0.19126) - sin
          (q[8] + q[2]) * 0.212465) * 9.81 + (2.0 * q[8] - 2.0 * q[10]) * 1320.0
    / 2.0;
  G[9] = (q[7] - q[9]) * -1020.0;
  G[10] = (q[8] - q[10]) * -1320.0;

  /* %%% */
  memset(&B[0], 0, 44U * sizeof(real_T));
  B[5] = 50.0;
  B[17] = 50.0;
  B[31] = 50.0;
  B[43] = 50.0;

  /* %%% */
  memset(&DampingSpringTorque[0], 0, 11U * sizeof(real_T));
  DampingSpringTorque[0] = 0.0;
  DampingSpringTorque[1] = 0.0;
  DampingSpringTorque[2] = 0.0;
  DampingSpringTorque[3] = (dq[3] - dq[5]) * 25.5499510762741;
  DampingSpringTorque[4] = (dq[4] - dq[6]) * 29.065443399335919;
  DampingSpringTorque[5] = (dq[3] - dq[5]) * -25.5499510762741;
  DampingSpringTorque[6] = (dq[4] - dq[6]) * -29.065443399335919;
  DampingSpringTorque[7] = (dq[7] - dq[9]) * 25.5499510762741;
  DampingSpringTorque[8] = (dq[8] - dq[10]) * 29.065443399335919;
  DampingSpringTorque[9] = (dq[7] - dq[9]) * -25.5499510762741;
  DampingSpringTorque[10] = (dq[8] - dq[10]) * -29.065443399335919;

  /* %%% */
}

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

/* Function for MATLAB Function: '<Root>/controller' */
static void vc_controller_1_ATRIAS2D_SS_s(const real_T q[11], const real_T dq[11],
  const real_T theta_limits[2], real_T leg, real_T *s, real_T *ds, real_T *th,
  real_T *dth, real_T *delta_theta, real_T c[11], real_T dsdq[11])
{
  real_T y;
  real_T a[11];
  int32_T k;

  /*  */
  /*  Files works with the full dynamics and the zero dynamics. Just replace q */
  /*  with z and dq with dz */
  /*  */
  /* % Compute and return s and th = theta */
  memset(&c[0], 0, 11U * sizeof(real_T));
  if (!(leg != 0.0)) {
    c[2] = 1.0;
    c[3] = 0.5;
    c[4] = 0.5;
  } else {
    c[2] = 1.0;
    c[7] = 0.5;
    c[8] = 0.5;
  }

  y = 0.0;
  for (k = 0; k < 11; k++) {
    y += c[k] * q[k];
    a[k] = -c[k];
  }

  *th = 4.71238898038469 - y;

  /* check offset */
  *dth = 0.0;
  *delta_theta = theta_limits[1] - theta_limits[0];
  *s = ((4.71238898038469 - y) - theta_limits[0]) / *delta_theta;
  for (k = 0; k < 11; k++) {
    *dth += a[k] * dq[k];
    dsdq[k] = -c[k] / *delta_theta;
  }

  *ds = *dth / *delta_theta;

  /*  */
}

/* Function for MATLAB Function: '<Root>/controller' */
static void vc_controller_1_invNxN(const real_T x[121], real_T y[121])
{
  int8_T p[11];
  real_T A[121];
  int8_T ipiv[11];
  int32_T jj;
  int32_T jp1j;
  int32_T b_j;
  int32_T ix;
  real_T smax;
  real_T s;
  int32_T b_ix;
  int32_T c_ix;
  int32_T pipk;
  int32_T e_k;
  for (b_j = 0; b_j < 121; b_j++) {
    y[b_j] = 0.0;
    A[b_j] = x[b_j];
  }

  for (b_j = 0; b_j < 11; b_j++) {
    ipiv[b_j] = (int8_T)(1 + b_j);
  }

  for (b_j = 0; b_j < 10; b_j++) {
    jj = b_j * 12;
    jp1j = jj;
    e_k = 0;
    ix = jj;
    smax = fabs(A[jj]);
    for (pipk = 1; pipk + 1 <= 11 - b_j; pipk++) {
      ix++;
      s = fabs(A[ix]);
      if (s > smax) {
        e_k = pipk;
        smax = s;
      }
    }

    if (A[jj + e_k] != 0.0) {
      if (e_k != 0) {
        ipiv[b_j] = (int8_T)((b_j + e_k) + 1);
        b_ix = b_j;
        pipk = b_j + e_k;
        for (e_k = 0; e_k < 11; e_k++) {
          smax = A[b_ix];
          A[b_ix] = A[pipk];
          A[pipk] = smax;
          b_ix += 11;
          pipk += 11;
        }
      }

      ix = jj - b_j;
      for (pipk = jj + 1; pipk + 1 <= ix + 11; pipk++) {
        A[pipk] /= A[jj];
      }
    }

    pipk = jj + 11;
    for (e_k = 1; e_k <= 10 - b_j; e_k++) {
      smax = A[pipk];
      if (A[pipk] != 0.0) {
        c_ix = jp1j + 2;
        ix = jj - b_j;
        for (b_ix = 12 + jj; b_ix + 1 <= ix + 22; b_ix++) {
          A[b_ix] += A[c_ix - 1] * -smax;
          c_ix++;
        }
      }

      pipk += 11;
      jj += 11;
    }
  }

  for (b_j = 0; b_j < 11; b_j++) {
    p[b_j] = (int8_T)(1 + b_j);
  }

  for (ix = 0; ix < 10; ix++) {
    if (ipiv[ix] > 1 + ix) {
      pipk = p[ipiv[ix] - 1];
      p[ipiv[ix] - 1] = p[ix];
      p[ix] = (int8_T)pipk;
    }
  }

  for (ix = 0; ix < 11; ix++) {
    y[ix + 11 * (p[ix] - 1)] = 1.0;
    for (pipk = ix; pipk + 1 < 12; pipk++) {
      if (y[(p[ix] - 1) * 11 + pipk] != 0.0) {
        for (e_k = pipk + 1; e_k + 1 < 12; e_k++) {
          y[e_k + 11 * (p[ix] - 1)] = y[(p[ix] - 1) * 11 + e_k] - y[(p[ix] - 1) *
            11 + pipk] * A[11 * pipk + e_k];
        }
      }
    }
  }

  for (ix = 0; ix < 11; ix++) {
    pipk = 11 * ix;
    for (e_k = 10; e_k >= 0; e_k += -1) {
      b_ix = 11 * e_k;
      if (y[e_k + pipk] != 0.0) {
        y[e_k + pipk] /= A[e_k + b_ix];
        for (c_ix = 0; c_ix + 1 <= e_k; c_ix++) {
          y[c_ix + pipk] -= y[e_k + pipk] * A[c_ix + b_ix];
        }
      }
    }
  }
}

/* Function for MATLAB Function: '<Root>/controller' */
static void vc_controller_1_bezierd(const real_T afra[24], real_T s, real_T
  value[4])
{
  real_T x[5];
  real_T y[5];
  int32_T i;
  int32_T j;
  static const int8_T b[5] = { 5, 20, 30, 20, 5 };

  value[0] = 0.0;
  value[1] = 0.0;
  value[2] = 0.0;
  value[3] = 0.0;

  /* % */
  for (i = 0; i < 5; i++) {
    x[i] = 1.0;
    y[i] = 1.0;
  }

  x[1] = s;
  y[1] = 1.0 - s;
  x[2] = s * x[1];
  y[2] = (1.0 - s) * y[1];
  x[3] = s * x[2];
  y[3] = (1.0 - s) * y[2];
  x[4] = s * x[3];
  y[4] = (1.0 - s) * y[3];
  for (i = 0; i < 4; i++) {
    value[i] = 0.0;
    for (j = 0; j < 5; j++) {
      value[i] += (afra[((1 + j) << 2) + i] - afra[(j << 2) + i]) * (real_T)b[j]
        * x[j] * y[4 - j];
    }
  }
}

/* Function for MATLAB Function: '<Root>/controller' */
static void vc_controller_1_mldivide(const real_T A[16], real_T B[4])
{
  real_T temp;
  real_T b_A[16];
  int8_T ipiv[4];
  int32_T jj;
  int32_T jp1j;
  int32_T jrow;
  int32_T j;
  int32_T ix;
  real_T s;
  int32_T b_ix;
  int32_T iy;
  int32_T ijA;
  memcpy(&b_A[0], &A[0], sizeof(real_T) << 4U);
  ipiv[0] = 1;
  ipiv[1] = 2;
  ipiv[2] = 3;
  ipiv[3] = 4;
  for (j = 0; j < 3; j++) {
    jj = j * 5;
    jp1j = jj;
    iy = 0;
    ix = jj;
    temp = fabs(b_A[jj]);
    for (jrow = 1; jrow + 1 <= 4 - j; jrow++) {
      ix++;
      s = fabs(b_A[ix]);
      if (s > temp) {
        iy = jrow;
        temp = s;
      }
    }

    if (b_A[jj + iy] != 0.0) {
      if (iy != 0) {
        ipiv[j] = (int8_T)((j + iy) + 1);
        iy += j;
        temp = b_A[j];
        b_A[j] = b_A[iy];
        b_A[iy] = temp;
        b_ix = j + 4;
        iy += 4;
        temp = b_A[b_ix];
        b_A[b_ix] = b_A[iy];
        b_A[iy] = temp;
        b_ix += 4;
        iy += 4;
        temp = b_A[b_ix];
        b_A[b_ix] = b_A[iy];
        b_A[iy] = temp;
        b_ix += 4;
        iy += 4;
        temp = b_A[b_ix];
        b_A[b_ix] = b_A[iy];
        b_A[iy] = temp;
      }

      ix = jj - j;
      for (jrow = jj + 1; jrow + 1 <= ix + 4; jrow++) {
        b_A[jrow] /= b_A[jj];
      }
    }

    ix = jj + 4;
    for (jrow = 1; jrow <= 3 - j; jrow++) {
      temp = b_A[ix];
      if (b_A[ix] != 0.0) {
        iy = jp1j + 2;
        b_ix = jj - j;
        for (ijA = 5 + jj; ijA + 1 <= b_ix + 8; ijA++) {
          b_A[ijA] += b_A[iy - 1] * -temp;
          iy++;
        }
      }

      ix += 4;
      jj += 4;
    }
  }

  if (ipiv[0] != 1) {
    temp = B[0];
    B[0] = B[ipiv[0] - 1];
    B[ipiv[0] - 1] = temp;
  }

  if (ipiv[1] != 2) {
    temp = B[1];
    B[1] = B[ipiv[1] - 1];
    B[ipiv[1] - 1] = temp;
  }

  if (ipiv[2] != 3) {
    temp = B[2];
    B[2] = B[ipiv[2] - 1];
    B[ipiv[2] - 1] = temp;
  }

  if (B[0] != 0.0) {
    for (iy = 2; iy < 5; iy++) {
      B[iy - 1] -= b_A[iy - 1] * B[0];
    }
  }

  if (B[1] != 0.0) {
    for (iy = 3; iy < 5; iy++) {
      B[iy - 1] -= b_A[iy + 3] * B[1];
    }
  }

  if (B[2] != 0.0) {
    iy = 4;
    while (iy < 5) {
      B[3] -= B[2] * b_A[11];
      iy = 5;
    }
  }

  if (B[3] != 0.0) {
    B[3] /= b_A[15];
    for (iy = 0; iy + 1 < 4; iy++) {
      B[iy] -= b_A[iy + 12] * B[3];
    }
  }

  if (B[2] != 0.0) {
    B[2] /= b_A[10];
    for (iy = 0; iy + 1 < 3; iy++) {
      B[iy] -= b_A[iy + 8] * B[2];
    }
  }

  if (B[1] != 0.0) {
    B[1] /= b_A[5];
    for (iy = 0; iy + 1 < 2; iy++) {
      B[iy] -= b_A[iy + 4] * B[1];
    }
  }

  if (B[0] != 0.0) {
    B[0] /= b_A[0];
  }
}

/* Function for MATLAB Function: '<Root>/controller' */
static void ATRIAS2D_SS_feedback_control_Ro(real_T t, const real_T q[11], const
  real_T dq[11], const real_T D[121], const real_T H[11], const real_T B[44],
  const real_T theta_limits[2], real_T h_alpha[24], const real_T poly_cor[20],
  const real_T kp[2], const real_T kd[2], real_T pdCtrl, real_T pdPlusFwdCtrl,
  real_T L2fhCtrl, real_T stance_leg, real_T a3scuff, real_T a4scuff, uint8_T
  s_mode, real_T s_freq, real_T u[4], real_T y[4], real_T dy[4], real_T *s_out,
  real_T *ds)
{
  real_T s;
  real_T D_inv[121];
  real_T d[4];
  real_T h_alpha_temp[24];
  real_T DecoupMatrix[16];
  real_T Kp[16];
  real_T Kd[16];
  real_T pd[4];
  real_T c[11];
  real_T dsdq[11];
  real_T jacob_h[44];
  real_T b_x[6];
  real_T d_y[6];
  int32_T i;
  int32_T b_j;
  static const int8_T e[6] = { 1, 5, 10, 10, 5, 1 };

  boolean_T s_0;
  real_T Kp_0[4];
  real_T Kd_0[4];
  real_T jacob_h_0[44];
  int32_T i_0;
  real_T c_x_idx;
  real_T c_x_idx_0;
  real_T e_y_idx;
  real_T e_y_idx_0;
  real_T d_0;

  /*  */
  /* % Choices */
  /*  */
  /*  [kp, kd, qTdesired] = ATRIAS2D_FeedbackParams; */
  /*  */
  /* % s */
  vc_controller_1_ATRIAS2D_SS_s(q, dq, theta_limits, stance_leg, &s, &c_x_idx,
    &e_y_idx, &c_x_idx_0, &e_y_idx_0, c, dsdq);
  *ds = c_x_idx;
  *s_out = s;
  if (s_mode == 0) {
    s = 0.0;
    *ds = 0.0;
  } else {
    if (s_mode == 1) {
      c_x_idx = sin(6.2831853071795862 * s_freq * t);
      s = 0.5 * c_x_idx + 0.5;
      *ds = 6.2831853071795862 * s_freq * 0.5 * cos(6.2831853071795862 * s_freq *
        t);
      *s_out = 0.5 * c_x_idx + 0.5;
    }
  }

  /* % Compute terms in controller */
  vc_controller_1_invNxN(D, D_inv);

  /*  */
  u[0] = 0.0;
  u[1] = 0.0;
  u[2] = 0.0;
  u[3] = 0.0;

  /*  if(auto_swap) */
  /*      [pT pHip p1 p2 p3 p4 p1L p2L p3L p4L pcm pcmT pcm1 pcm2 pcm3 pcm4 pcm1L pcm2L pcm3L pcm4L] =  PointsAtrias2DFlight(q); */
  /*  */
  /*      epsilon = 0.001; */
  /*      if(p4(2) <= epsilon) */
  /*          sw(1) = 1; */
  /*      end */
  /*  */
  /*      if(p4L(2) <= epsilon) */
  /*          sw(2) = 1; */
  /*      end */
  /*  */
  /*      temp = 0.3 */
  /*      if((sw(1)) && ~(sw(2)) && ((p4L(1)-p4(1))<temp)) */
  /*          [h0,jacob_h0,jacob_jacobh0dq]= ATRIAS2D_Fdbk_Terms(q,dq,0); % right stance */
  /*      elseif((sw(2)) && ~(sw(1)) && ((p4(1)-p4L(1))<temp)) */
  /*          [h0,jacob_h0,jacob_jacobh0dq]= ATRIAS2D_Fdbk_Terms(q,dq,1); % left stance */
  /*      else */
  /*  */
  /*      end */
  /*  else */
  /*      if(swap_cmd) */
  /*          if(prev_leg == 1) */
  /*              [h0,jacob_h0,jacob_jacobh0dq]= ATRIAS2D_Fdbk_Terms(q,dq,0); */
  /*              prev_leg = 0; */
  /*          else */
  /*              [h0,jacob_h0,jacob_jacobh0dq]= ATRIAS2D_Fdbk_Terms(q,dq,1); */
  /*              prev_leg = 1; */
  /*          end */
  /*      end */
  /*  */
  /*  end */
  memset(&h_alpha_temp[0], 0, 24U * sizeof(real_T));
  b_x[0] = 0.0;
  b_x[1] = 0.0;
  b_x[2] = a3scuff;
  b_x[3] = a4scuff;
  b_x[4] = 0.0;
  b_x[5] = 0.0;
  for (i = 0; i < 6; i++) {
    h_alpha[3 + (i << 2)] = h_alpha[(i << 2) + 3] + b_x[i];
  }

  if (!(stance_leg != 0.0)) {
    memcpy(&h_alpha_temp[0], &h_alpha[0], 24U * sizeof(real_T));
  } else {
    for (i = 0; i < 6; i++) {
      h_alpha_temp[i << 2] = h_alpha[(i << 2) + 1];
    }

    for (i = 0; i < 6; i++) {
      h_alpha_temp[1 + (i << 2)] = h_alpha[i << 2];
    }

    for (i = 0; i < 6; i++) {
      h_alpha_temp[2 + (i << 2)] = h_alpha[(i << 2) + 3];
    }

    for (i = 0; i < 6; i++) {
      h_alpha_temp[3 + (i << 2)] = h_alpha[(i << 2) + 2];
    }
  }

  /* %%%%%  ATRIAS2D_Fdbk_Terms.m */
  /* %%%  05-Sep-2012 18:17:02 */
  /* %%% */
  /* %%% Authors(s): Grizzle */
  /* %%% */
  /* %%% Model NOTATION: Spong and Vidyasagar, page 142, Eq. (6.3.12) */
  /* %%%                 D(q)ddq + C(q,dq)*dq + G(q) = B*tau */
  /* %%% */
  /* %%%[g mTotal L1 L2 L3 L4 m1 m2 m3 m4 Jcm1 Jcm2 Jcm3 Jcm4 ellzcm1 ellzcm2 ellzcm3 ellzcm4 ellycm1 ellycm2 ellycm3 ellycm4 LT mT JcmT ellzcmT ellycmT mH JcmH K1 K2 Kd1 Kd2 Jrotor1 Jrotor2 Jgear1  Jgear2 R1  R2] = modelParametersAtrias_v05; */
  /* %%% */
  y[0] = 0.0;
  y[1] = 0.0;
  y[2] = 0.0;
  y[3] = 0.0;
  y[0] = q[5] / 2.0 + q[6] / 2.0;
  y[1] = q[9] / 2.0 + q[10] / 2.0;
  y[2] = q[6] - q[5];
  y[3] = q[10] - q[9];

  /* %%% */
  memset(&jacob_h[0], 0, 44U * sizeof(real_T));
  jacob_h[20] = 0.5;
  jacob_h[24] = 0.5;
  jacob_h[37] = 0.5;
  jacob_h[41] = 0.5;
  jacob_h[22] = -1.0;
  jacob_h[26] = 1.0;
  jacob_h[39] = -1.0;
  jacob_h[43] = 1.0;

  /* %%% */
  /*  left stance */
  pd[0] = 0.0;
  pd[1] = 0.0;
  pd[2] = 0.0;
  pd[3] = 0.0;

  /* %     */
  for (i = 0; i < 6; i++) {
    b_x[i] = 1.0;
    d_y[i] = 1.0;
  }

  for (i = 0; i < 5; i++) {
    b_x[1 + i] = s * b_x[i];
    d_y[1 + i] = (1.0 - s) * d_y[i];
  }

  for (i = 0; i < 4; i++) {
    pd[i] = 0.0;
    for (b_j = 0; b_j < 6; b_j++) {
      pd[i] += h_alpha_temp[(b_j << 2) + i] * (real_T)e[b_j] * b_x[b_j] * d_y[5
        - b_j];
    }
  }

  y[0] -= pd[0];
  y[1] -= pd[1];
  y[2] -= pd[2];
  y[3] -= pd[3];
  vc_controller_1_bezierd(h_alpha_temp, s, pd);
  for (i = 0; i < 4; i++) {
    c_x_idx = 0.0;
    for (b_j = 0; b_j < 11; b_j++) {
      c_x_idx += jacob_h[(b_j << 2) + i] * dq[b_j];
    }

    dy[i] = c_x_idx - pd[i] * *ds;
  }

  d[0] = 0.0;
  d[1] = 0.0;
  d[2] = 0.0;
  d[3] = 0.0;

  /* % */
  c_x_idx = s * s;
  e_y_idx = (1.0 - s) * (1.0 - s);
  c_x_idx_0 = s * c_x_idx;
  e_y_idx_0 = (1.0 - s) * e_y_idx;
  for (i = 0; i < 4; i++) {
    d_0 = ((h_alpha_temp[i + 20] - h_alpha_temp[i + 16] * 2.0) + h_alpha_temp[i
           + 12]) * 20.0 * c_x_idx_0 + (((h_alpha_temp[i + 16] - h_alpha_temp[i
      + 12] * 2.0) + h_alpha_temp[i + 8]) * 60.0 * c_x_idx * (1.0 - s) +
      (((h_alpha_temp[i + 12] - h_alpha_temp[i + 8] * 2.0) + h_alpha_temp[i + 4])
       * 60.0 * s * e_y_idx + ((h_alpha_temp[i + 8] - h_alpha_temp[i + 4] * 2.0)
      + h_alpha_temp[i]) * 20.0 * e_y_idx_0));
    d[i] = d_0;
  }

  c_x_idx = *ds * *ds;
  d[0] *= c_x_idx;
  d[1] *= c_x_idx;
  d[2] *= c_x_idx;
  d[3] *= c_x_idx;
  vc_controller_1_bezierd(h_alpha_temp, s, pd);
  for (i = 0; i < 11; i++) {
    jacob_h_0[i << 2] = jacob_h[i << 2] - pd[0] * dsdq[i];
    jacob_h_0[1 + (i << 2)] = jacob_h[(i << 2) + 1] - pd[1] * dsdq[i];
    jacob_h_0[2 + (i << 2)] = jacob_h[(i << 2) + 2] - pd[2] * dsdq[i];
    jacob_h_0[3 + (i << 2)] = jacob_h[(i << 2) + 3] - pd[3] * dsdq[i];
  }

  if (s < 1.0) {
    s_0 = (s <= 0.5);
    if (s_0) {
      c_x_idx = (((s * poly_cor[16] + poly_cor[12]) * s + poly_cor[8]) * s +
                 poly_cor[4]) * s + poly_cor[0];
    } else {
      c_x_idx = 0.0 * poly_cor[0];
    }

    y[0] -= c_x_idx;
    if (s_0) {
      c_x_idx = (((s * poly_cor[17] + poly_cor[13]) * s + poly_cor[9]) * s +
                 poly_cor[5]) * s + poly_cor[1];
    } else {
      c_x_idx = 0.0 * poly_cor[1];
    }

    y[1] -= c_x_idx;
    if (s_0) {
      c_x_idx = (((s * poly_cor[18] + poly_cor[14]) * s + poly_cor[10]) * s +
                 poly_cor[6]) * s + poly_cor[2];
    } else {
      c_x_idx = 0.0 * poly_cor[2];
    }

    y[2] -= c_x_idx;
    if (s_0) {
      c_x_idx = (((s * poly_cor[19] + poly_cor[15]) * s + poly_cor[11]) * s +
                 poly_cor[7]) * s + poly_cor[3];
    } else {
      c_x_idx = 0.0 * poly_cor[3];
    }

    y[3] -= c_x_idx;
    if (s <= 0.5) {
      c_x_idx = s * 4.0;
      pd[0] = ((3.0 * poly_cor[12] + c_x_idx * poly_cor[16]) * s + 2.0 *
               poly_cor[8]) * s + poly_cor[4];
      pd[1] = ((3.0 * poly_cor[13] + c_x_idx * poly_cor[17]) * s + 2.0 *
               poly_cor[9]) * s + poly_cor[5];
      pd[2] = ((3.0 * poly_cor[14] + c_x_idx * poly_cor[18]) * s + 2.0 *
               poly_cor[10]) * s + poly_cor[6];
      pd[3] = ((3.0 * poly_cor[15] + c_x_idx * poly_cor[19]) * s + 2.0 *
               poly_cor[11]) * s + poly_cor[7];
    } else {
      pd[0] = 0.0 * poly_cor[4];
      pd[1] = 0.0 * poly_cor[5];
      pd[2] = 0.0 * poly_cor[6];
      pd[3] = 0.0 * poly_cor[7];
    }

    dy[0] -= pd[0] * *ds;
    dy[1] -= pd[1] * *ds;
    dy[2] -= pd[2] * *ds;
    dy[3] -= pd[3] * *ds;
    if (s <= 0.5) {
      c_x_idx = s * 12.0;
      pd[0] = (6.0 * poly_cor[12] + c_x_idx * poly_cor[16]) * s + 2.0 *
        poly_cor[8];
      pd[1] = (6.0 * poly_cor[13] + c_x_idx * poly_cor[17]) * s + 2.0 *
        poly_cor[9];
      pd[2] = (6.0 * poly_cor[14] + c_x_idx * poly_cor[18]) * s + 2.0 *
        poly_cor[10];
      pd[3] = (6.0 * poly_cor[15] + c_x_idx * poly_cor[19]) * s + 2.0 *
        poly_cor[11];
    } else {
      pd[0] = 0.0 * poly_cor[8];
      pd[1] = 0.0 * poly_cor[9];
      pd[2] = 0.0 * poly_cor[10];
      pd[3] = 0.0 * poly_cor[11];
    }

    c_x_idx = *ds * *ds;
    d[0] += pd[0] * c_x_idx;
    d[1] += pd[1] * c_x_idx;
    d[2] += pd[2] * c_x_idx;
    d[3] += pd[3] * c_x_idx;
    if (s <= 0.5) {
      c_x_idx = s * 4.0;
      pd[0] = ((3.0 * poly_cor[12] + c_x_idx * poly_cor[16]) * s + 2.0 *
               poly_cor[8]) * s + poly_cor[4];
      pd[1] = ((3.0 * poly_cor[13] + c_x_idx * poly_cor[17]) * s + 2.0 *
               poly_cor[9]) * s + poly_cor[5];
      pd[2] = ((3.0 * poly_cor[14] + c_x_idx * poly_cor[18]) * s + 2.0 *
               poly_cor[10]) * s + poly_cor[6];
      pd[3] = ((3.0 * poly_cor[15] + c_x_idx * poly_cor[19]) * s + 2.0 *
               poly_cor[11]) * s + poly_cor[7];
    } else {
      pd[0] = 0.0 * poly_cor[4];
      pd[1] = 0.0 * poly_cor[5];
      pd[2] = 0.0 * poly_cor[6];
      pd[3] = 0.0 * poly_cor[7];
    }

    for (i = 0; i < 11; i++) {
      jacob_h_0[i << 2] -= pd[0] * dsdq[i];
      jacob_h_0[1 + (i << 2)] = jacob_h_0[(i << 2) + 1] - pd[1] * dsdq[i];
      jacob_h_0[2 + (i << 2)] = jacob_h_0[(i << 2) + 2] - pd[2] * dsdq[i];
      jacob_h_0[3 + (i << 2)] = jacob_h_0[(i << 2) + 3] - pd[3] * dsdq[i];
    }
  }

  /* % Set up IO-Linearizing Controller */
  for (i = 0; i < 4; i++) {
    for (b_j = 0; b_j < 11; b_j++) {
      jacob_h[i + (b_j << 2)] = 0.0;
      for (i_0 = 0; i_0 < 11; i_0++) {
        jacob_h[i + (b_j << 2)] = jacob_h_0[(i_0 << 2) + i] * D_inv[11 * b_j +
          i_0] + jacob_h[(b_j << 2) + i];
      }
    }
  }

  for (i = 0; i < 4; i++) {
    for (b_j = 0; b_j < 4; b_j++) {
      DecoupMatrix[i + (b_j << 2)] = 0.0;
      for (i_0 = 0; i_0 < 11; i_0++) {
        DecoupMatrix[i + (b_j << 2)] = jacob_h[(i_0 << 2) + i] * B[11 * b_j +
          i_0] + DecoupMatrix[(b_j << 2) + i];
      }
    }
  }

  /* %LgLfh */
  /*  svd(DecoupMatrix) */
  /*  */
  pd[0] = kp[0];
  pd[1] = kp[1];
  pd[2] = kp[0];
  pd[3] = kp[1];
  for (i = 0; i < 16; i++) {
    Kp[i] = 0.0;
    Kd[i] = 0.0;
  }

  Kp[0] = pd[0];
  Kp[5] = pd[1];
  Kp[10] = pd[2];
  Kp[15] = pd[3];
  pd[0] = kd[0];
  pd[1] = kd[1];
  pd[2] = kd[0];
  pd[3] = kd[1];
  Kd[0] = pd[0];
  Kd[5] = pd[1];
  Kd[10] = pd[2];
  Kd[15] = pd[3];
  for (i = 0; i < 4; i++) {
    c_x_idx = Kp[i + 12] * y[3] + (Kp[i + 8] * y[2] + (Kp[i + 4] * y[1] + Kp[i] *
      y[0]));
    Kp_0[i] = c_x_idx;
  }

  for (i = 0; i < 4; i++) {
    c_x_idx = Kd[i + 12] * dy[3] + (Kd[i + 8] * dy[2] + (Kd[i + 4] * dy[1] +
      Kd[i] * dy[0]));
    Kd_0[i] = c_x_idx;
  }

  pd[0] = Kp_0[0] + Kd_0[0];
  pd[1] = Kp_0[1] + Kd_0[1];
  pd[2] = Kp_0[2] + Kd_0[2];
  pd[3] = Kp_0[3] + Kd_0[3];

  /*  */
  if (s > 1.0) {
    for (i = 0; i < 11; i++) {
      jacob_h[i << 2] = -jacob_h_0[i << 2];
      jacob_h[1 + (i << 2)] = -jacob_h_0[(i << 2) + 1];
      jacob_h[2 + (i << 2)] = -jacob_h_0[(i << 2) + 2];
      jacob_h[3 + (i << 2)] = -jacob_h_0[(i << 2) + 3];
    }

    for (i = 0; i < 4; i++) {
      for (b_j = 0; b_j < 11; b_j++) {
        jacob_h_0[i + (b_j << 2)] = 0.0;
        for (i_0 = 0; i_0 < 11; i_0++) {
          jacob_h_0[i + (b_j << 2)] = jacob_h[(i_0 << 2) + i] * D_inv[11 * b_j +
            i_0] + jacob_h_0[(b_j << 2) + i];
        }
      }
    }

    for (i = 0; i < 4; i++) {
      Kp_0[i] = 0.0;
      for (b_j = 0; b_j < 11; b_j++) {
        Kp_0[i] += jacob_h_0[(b_j << 2) + i] * H[b_j];
      }
    }

    for (i = 0; i < 4; i++) {
      Kd_0[i] = 0.0;
      for (b_j = 0; b_j < 11; b_j++) {
        Kd_0[i] += 0.0 * dq[b_j];
      }
    }

    c_x_idx = (Kp_0[0] + Kd_0[0]) - d[0];
    e_y_idx = (Kp_0[1] + Kd_0[1]) - d[1];
    c_x_idx_0 = (Kp_0[2] + Kd_0[2]) - d[2];
    e_y_idx_0 = (Kp_0[3] + Kd_0[3]) - d[3];

    /* %%L2fh */
    /* uBound=diag([3 3 7 7])*ones(4,1); */
  } else {
    for (i = 0; i < 11; i++) {
      jacob_h[i << 2] = -jacob_h_0[i << 2];
      jacob_h[1 + (i << 2)] = -jacob_h_0[(i << 2) + 1];
      jacob_h[2 + (i << 2)] = -jacob_h_0[(i << 2) + 2];
      jacob_h[3 + (i << 2)] = -jacob_h_0[(i << 2) + 3];
    }

    for (i = 0; i < 4; i++) {
      for (b_j = 0; b_j < 11; b_j++) {
        jacob_h_0[i + (b_j << 2)] = 0.0;
        for (i_0 = 0; i_0 < 11; i_0++) {
          jacob_h_0[i + (b_j << 2)] = jacob_h[(i_0 << 2) + i] * D_inv[11 * b_j +
            i_0] + jacob_h_0[(b_j << 2) + i];
        }
      }
    }

    for (i = 0; i < 4; i++) {
      Kp_0[i] = 0.0;
      for (b_j = 0; b_j < 11; b_j++) {
        Kp_0[i] += jacob_h_0[(b_j << 2) + i] * H[b_j];
      }
    }

    for (i = 0; i < 4; i++) {
      Kd_0[i] = 0.0;
      for (b_j = 0; b_j < 11; b_j++) {
        Kd_0[i] += 0.0 * dq[b_j];
      }
    }

    c_x_idx = (Kp_0[0] + Kd_0[0]) - d[0];
    e_y_idx = (Kp_0[1] + Kd_0[1]) - d[1];
    c_x_idx_0 = (Kp_0[2] + Kd_0[2]) - d[2];
    e_y_idx_0 = (Kp_0[3] + Kd_0[3]) - d[3];

    /* %%L2fh */
  }

  if (pdCtrl != 0.0) {
    u[0] = -pd[0];
    u[1] = -pd[1];
    u[2] = -pd[2];
    u[3] = -pd[3];
    vc_controller_1_mldivide(DecoupMatrix, u);
  } else if (pdPlusFwdCtrl != 0.0) {
    u[0] = -c_x_idx - pd[0];
    u[1] = -e_y_idx - pd[1];
    u[2] = -c_x_idx_0 - pd[2];
    u[3] = -e_y_idx_0 - pd[3];
    vc_controller_1_mldivide(DecoupMatrix, u);
  } else {
    if (L2fhCtrl != 0.0) {
      u[0] = -c_x_idx;
      u[1] = -e_y_idx;
      u[2] = -c_x_idx_0;
      u[3] = -e_y_idx_0;
      vc_controller_1_mldivide(DecoupMatrix, u);
    }
  }
}

/* Model step function */
void vc_controller_1_step(void)
{
  /* local block i/o variables */
  real_T rtb_Clock;
  real_T kp[3];
  real_T kd[3];
  real_T q[13];
  real_T dq[13];
  real_T sat_val[6];
  real_T D[121];
  real_T C[121];
  real_T G[11];
  real_T B[44];
  real_T DampingSpringTorque[11];
  real_T u[4];
  real_T y[4];
  real_T dy[4];
  real_T ds;
  real_T c;
  real_T x[6];
  static const int8_T b[11] = { 0, 1, 2, 3, 4, 5, 6, 8, 9, 10, 11 };

  int32_T i;
  real_T tmp[11];
  real_T tmp_0[11];
  real_T C_0[11];
  real_T tmp_1[24];
  real_T C_1[11];
  real_T B_HarmonicDrive[11];
  int32_T i_0;

  /* Clock: '<Root>/Clock' */
  rtb_Clock = vc_controller_1_M->Timing.t[0];

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
  vc_controller_1_B.q_um[0] = -vc_controller_1_U.q_osu[10];
  vc_controller_1_B.q_um[1] = vc_controller_1_U.q_osu[12];
  vc_controller_1_B.q_um[2] = vc_controller_1_U.q_osu[13] + 1.5707963267948966;
  vc_controller_1_B.q_um[3] = vc_controller_1_U.q_osu[1] + 1.5707963267948966;
  vc_controller_1_B.q_um[4] = vc_controller_1_U.q_osu[3] + 1.5707963267948966;
  vc_controller_1_B.q_um[5] = vc_controller_1_U.q_osu[2] + 1.5707963267948966;
  vc_controller_1_B.q_um[6] = vc_controller_1_U.q_osu[4] + 1.5707963267948966;
  vc_controller_1_B.q_um[7] = -(vc_controller_1_U.q_osu[0] - 4.71238898038469);
  vc_controller_1_B.q_um[8] = vc_controller_1_U.q_osu[6] + 1.5707963267948966;
  vc_controller_1_B.q_um[9] = vc_controller_1_U.q_osu[8] + 1.5707963267948966;
  vc_controller_1_B.q_um[10] = vc_controller_1_U.q_osu[7] + 1.5707963267948966;
  vc_controller_1_B.q_um[11] = vc_controller_1_U.q_osu[9] + 1.5707963267948966;
  vc_controller_1_B.q_um[12] = vc_controller_1_U.q_osu[5] - 4.71238898038469;

  /* SignalConversion: '<Root>/ConcatBufferAtVector ConcatenateIn1' */
  memcpy(&vc_controller_1_B.VectorConcatenate[0], &vc_controller_1_B.q_um[0],
         13U * sizeof(real_T));

  /* DiscreteStateSpace: '<S1>/DirtyDerivative' */
  {
    vc_controller_1_B.VectorConcatenate[13] =
      (vc_controller_1_P.DirtyDerivative_C[0])*
      vc_controller_1_DWork.DirtyDerivative_DSTATE[0]
      + (vc_controller_1_P.DirtyDerivative_C[1])*
      vc_controller_1_DWork.DirtyDerivative_DSTATE[1]
      + (vc_controller_1_P.DirtyDerivative_C[2])*
      vc_controller_1_DWork.DirtyDerivative_DSTATE[2]
      + (vc_controller_1_P.DirtyDerivative_C[3])*
      vc_controller_1_DWork.DirtyDerivative_DSTATE[3];
    vc_controller_1_B.VectorConcatenate[13] +=
      vc_controller_1_P.DirtyDerivative_D*vc_controller_1_B.q_um[0];
  }

  /* DiscreteStateSpace: '<S1>/DirtyDerivative1' */
  {
    vc_controller_1_B.VectorConcatenate[14] =
      (vc_controller_1_P.DirtyDerivative1_C[0])*
      vc_controller_1_DWork.DirtyDerivative1_DSTATE[0]
      + (vc_controller_1_P.DirtyDerivative1_C[1])*
      vc_controller_1_DWork.DirtyDerivative1_DSTATE[1]
      + (vc_controller_1_P.DirtyDerivative1_C[2])*
      vc_controller_1_DWork.DirtyDerivative1_DSTATE[2]
      + (vc_controller_1_P.DirtyDerivative1_C[3])*
      vc_controller_1_DWork.DirtyDerivative1_DSTATE[3];
    vc_controller_1_B.VectorConcatenate[14] +=
      vc_controller_1_P.DirtyDerivative1_D*vc_controller_1_B.q_um[1];
  }

  /* DiscreteStateSpace: '<S1>/DirtyDerivative2' */
  {
    vc_controller_1_B.VectorConcatenate[15] =
      (vc_controller_1_P.DirtyDerivative2_C[0])*
      vc_controller_1_DWork.DirtyDerivative2_DSTATE[0]
      + (vc_controller_1_P.DirtyDerivative2_C[1])*
      vc_controller_1_DWork.DirtyDerivative2_DSTATE[1]
      + (vc_controller_1_P.DirtyDerivative2_C[2])*
      vc_controller_1_DWork.DirtyDerivative2_DSTATE[2]
      + (vc_controller_1_P.DirtyDerivative2_C[3])*
      vc_controller_1_DWork.DirtyDerivative2_DSTATE[3];
    vc_controller_1_B.VectorConcatenate[15] +=
      vc_controller_1_P.DirtyDerivative2_D*vc_controller_1_B.q_um[2];
  }

  /* DiscreteStateSpace: '<S1>/DirtyDerivative3' */
  {
    vc_controller_1_B.VectorConcatenate[16] =
      (vc_controller_1_P.DirtyDerivative3_C[0])*
      vc_controller_1_DWork.DirtyDerivative3_DSTATE[0]
      + (vc_controller_1_P.DirtyDerivative3_C[1])*
      vc_controller_1_DWork.DirtyDerivative3_DSTATE[1]
      + (vc_controller_1_P.DirtyDerivative3_C[2])*
      vc_controller_1_DWork.DirtyDerivative3_DSTATE[2]
      + (vc_controller_1_P.DirtyDerivative3_C[3])*
      vc_controller_1_DWork.DirtyDerivative3_DSTATE[3];
    vc_controller_1_B.VectorConcatenate[16] +=
      vc_controller_1_P.DirtyDerivative3_D*vc_controller_1_B.q_um[3];
  }

  /* DiscreteStateSpace: '<S1>/DirtyDerivative4' */
  {
    vc_controller_1_B.VectorConcatenate[17] =
      (vc_controller_1_P.DirtyDerivative4_C[0])*
      vc_controller_1_DWork.DirtyDerivative4_DSTATE[0]
      + (vc_controller_1_P.DirtyDerivative4_C[1])*
      vc_controller_1_DWork.DirtyDerivative4_DSTATE[1]
      + (vc_controller_1_P.DirtyDerivative4_C[2])*
      vc_controller_1_DWork.DirtyDerivative4_DSTATE[2]
      + (vc_controller_1_P.DirtyDerivative4_C[3])*
      vc_controller_1_DWork.DirtyDerivative4_DSTATE[3];
    vc_controller_1_B.VectorConcatenate[17] +=
      vc_controller_1_P.DirtyDerivative4_D*vc_controller_1_B.q_um[4];
  }

  /* DiscreteStateSpace: '<S1>/DirtyDerivative5' */
  {
    vc_controller_1_B.VectorConcatenate[18] =
      (vc_controller_1_P.DirtyDerivative5_C[0])*
      vc_controller_1_DWork.DirtyDerivative5_DSTATE[0]
      + (vc_controller_1_P.DirtyDerivative5_C[1])*
      vc_controller_1_DWork.DirtyDerivative5_DSTATE[1]
      + (vc_controller_1_P.DirtyDerivative5_C[2])*
      vc_controller_1_DWork.DirtyDerivative5_DSTATE[2]
      + (vc_controller_1_P.DirtyDerivative5_C[3])*
      vc_controller_1_DWork.DirtyDerivative5_DSTATE[3];
    vc_controller_1_B.VectorConcatenate[18] +=
      vc_controller_1_P.DirtyDerivative5_D*vc_controller_1_B.q_um[5];
  }

  /* DiscreteStateSpace: '<S1>/DirtyDerivative6' */
  {
    vc_controller_1_B.VectorConcatenate[19] =
      (vc_controller_1_P.DirtyDerivative6_C[0])*
      vc_controller_1_DWork.DirtyDerivative6_DSTATE[0]
      + (vc_controller_1_P.DirtyDerivative6_C[1])*
      vc_controller_1_DWork.DirtyDerivative6_DSTATE[1]
      + (vc_controller_1_P.DirtyDerivative6_C[2])*
      vc_controller_1_DWork.DirtyDerivative6_DSTATE[2]
      + (vc_controller_1_P.DirtyDerivative6_C[3])*
      vc_controller_1_DWork.DirtyDerivative6_DSTATE[3];
    vc_controller_1_B.VectorConcatenate[19] +=
      vc_controller_1_P.DirtyDerivative6_D*vc_controller_1_B.q_um[6];
  }

  /* DiscreteStateSpace: '<S1>/DirtyDerivative7' */
  {
    vc_controller_1_B.VectorConcatenate[20] =
      (vc_controller_1_P.DirtyDerivative7_C[0])*
      vc_controller_1_DWork.DirtyDerivative7_DSTATE[0]
      + (vc_controller_1_P.DirtyDerivative7_C[1])*
      vc_controller_1_DWork.DirtyDerivative7_DSTATE[1]
      + (vc_controller_1_P.DirtyDerivative7_C[2])*
      vc_controller_1_DWork.DirtyDerivative7_DSTATE[2]
      + (vc_controller_1_P.DirtyDerivative7_C[3])*
      vc_controller_1_DWork.DirtyDerivative7_DSTATE[3];
    vc_controller_1_B.VectorConcatenate[20] +=
      vc_controller_1_P.DirtyDerivative7_D*vc_controller_1_B.q_um[7];
  }

  /* DiscreteStateSpace: '<S1>/DirtyDerivative8' */
  {
    vc_controller_1_B.VectorConcatenate[21] =
      (vc_controller_1_P.DirtyDerivative8_C[0])*
      vc_controller_1_DWork.DirtyDerivative8_DSTATE[0]
      + (vc_controller_1_P.DirtyDerivative8_C[1])*
      vc_controller_1_DWork.DirtyDerivative8_DSTATE[1]
      + (vc_controller_1_P.DirtyDerivative8_C[2])*
      vc_controller_1_DWork.DirtyDerivative8_DSTATE[2]
      + (vc_controller_1_P.DirtyDerivative8_C[3])*
      vc_controller_1_DWork.DirtyDerivative8_DSTATE[3];
    vc_controller_1_B.VectorConcatenate[21] +=
      vc_controller_1_P.DirtyDerivative8_D*vc_controller_1_B.q_um[8];
  }

  /* DiscreteStateSpace: '<S1>/DirtyDerivative9' */
  {
    vc_controller_1_B.VectorConcatenate[22] =
      (vc_controller_1_P.DirtyDerivative9_C[0])*
      vc_controller_1_DWork.DirtyDerivative9_DSTATE[0]
      + (vc_controller_1_P.DirtyDerivative9_C[1])*
      vc_controller_1_DWork.DirtyDerivative9_DSTATE[1]
      + (vc_controller_1_P.DirtyDerivative9_C[2])*
      vc_controller_1_DWork.DirtyDerivative9_DSTATE[2]
      + (vc_controller_1_P.DirtyDerivative9_C[3])*
      vc_controller_1_DWork.DirtyDerivative9_DSTATE[3];
    vc_controller_1_B.VectorConcatenate[22] +=
      vc_controller_1_P.DirtyDerivative9_D*vc_controller_1_B.q_um[9];
  }

  /* DiscreteStateSpace: '<S1>/DirtyDerivative10' */
  {
    vc_controller_1_B.VectorConcatenate[23] =
      (vc_controller_1_P.DirtyDerivative10_C[0])*
      vc_controller_1_DWork.DirtyDerivative10_DSTATE[0]
      + (vc_controller_1_P.DirtyDerivative10_C[1])*
      vc_controller_1_DWork.DirtyDerivative10_DSTATE[1]
      + (vc_controller_1_P.DirtyDerivative10_C[2])*
      vc_controller_1_DWork.DirtyDerivative10_DSTATE[2]
      + (vc_controller_1_P.DirtyDerivative10_C[3])*
      vc_controller_1_DWork.DirtyDerivative10_DSTATE[3];
    vc_controller_1_B.VectorConcatenate[23] +=
      vc_controller_1_P.DirtyDerivative10_D*vc_controller_1_B.q_um[10];
  }

  /* DiscreteStateSpace: '<S1>/DirtyDerivative11' */
  {
    vc_controller_1_B.VectorConcatenate[24] =
      (vc_controller_1_P.DirtyDerivative11_C[0])*
      vc_controller_1_DWork.DirtyDerivative11_DSTATE[0]
      + (vc_controller_1_P.DirtyDerivative11_C[1])*
      vc_controller_1_DWork.DirtyDerivative11_DSTATE[1]
      + (vc_controller_1_P.DirtyDerivative11_C[2])*
      vc_controller_1_DWork.DirtyDerivative11_DSTATE[2]
      + (vc_controller_1_P.DirtyDerivative11_C[3])*
      vc_controller_1_DWork.DirtyDerivative11_DSTATE[3];
    vc_controller_1_B.VectorConcatenate[24] +=
      vc_controller_1_P.DirtyDerivative11_D*vc_controller_1_B.q_um[11];
  }

  /* DiscreteStateSpace: '<S1>/DirtyDerivative12' */
  {
    vc_controller_1_B.VectorConcatenate[25] =
      (vc_controller_1_P.DirtyDerivative12_C[0])*
      vc_controller_1_DWork.DirtyDerivative12_DSTATE[0]
      + (vc_controller_1_P.DirtyDerivative12_C[1])*
      vc_controller_1_DWork.DirtyDerivative12_DSTATE[1]
      + (vc_controller_1_P.DirtyDerivative12_C[2])*
      vc_controller_1_DWork.DirtyDerivative12_DSTATE[2]
      + (vc_controller_1_P.DirtyDerivative12_C[3])*
      vc_controller_1_DWork.DirtyDerivative12_DSTATE[3];
    vc_controller_1_B.VectorConcatenate[25] +=
      vc_controller_1_P.DirtyDerivative12_D*vc_controller_1_B.q_um[12];
  }

  /* DiscreteFilter: '<Root>/Discrete Filter' incorporates:
   *  Inport: '<Root>/torso_offset'
   */
  vc_controller_1_DWork.DiscreteFilter_tmp = (vc_controller_1_U.torso_offset -
    vc_controller_1_P.DiscreteFilter_DenCoef[1] *
    vc_controller_1_DWork.DiscreteFilter_DSTATE) /
    vc_controller_1_P.DiscreteFilter_DenCoef[0];
  vc_controller_1_B.DiscreteFilter = vc_controller_1_P.DiscreteFilter_NumCoef *
    vc_controller_1_DWork.DiscreteFilter_tmp;

  /* MATLAB Function: '<Root>/controller' incorporates:
   *  Constant: '<Root>/Constant12'
   *  Constant: '<Root>/Constant13'
   *  Constant: '<Root>/Constant14'
   *  Constant: '<Root>/Constant23'
   *  Constant: '<Root>/Constant24'
   *  Constant: '<Root>/Constant25'
   *  Constant: '<Root>/Constant30'
   *  Constant: '<Root>/Constant31'
   *  Constant: '<Root>/stance_leg'
   *  Inport: '<Root>/epsilon'
   *  Inport: '<Root>/kd'
   *  Inport: '<Root>/kp'
   *  Inport: '<Root>/s_freq'
   *  Inport: '<Root>/s_mode'
   *  Inport: '<Root>/sat_val'
   */
  /* MATLAB Function 'controller': '<S2>:1' */
  /* %eml */
  /* '<S2>:1:5' */
  /* '<S2>:1:6' */
  for (i = 0; i < 13; i++) {
    q[i] = vc_controller_1_B.VectorConcatenate[i];
    dq[i] = vc_controller_1_B.VectorConcatenate[i + 13];
  }

  /* '<S2>:1:7' */
  /* '<S2>:1:8' */
  /* %%%  Model Parameters */
  for (i = 0; i < 11; i++) {
    B_HarmonicDrive[i] = vc_controller_1_B.VectorConcatenate[b[i]];
    C_0[i] = vc_controller_1_B.VectorConcatenate[13 + b[i]];
    tmp[i] = vc_controller_1_B.VectorConcatenate[b[i]];
    tmp_0[i] = vc_controller_1_B.VectorConcatenate[13 + b[i]];
  }

  vc__LagrangeModelAtriasFlight2D(B_HarmonicDrive, C_0, D, C, G, B,
    DampingSpringTorque);

  /* '<S2>:1:15' */
  /* '<S2>:1:16' */
  /* '<S2>:1:17' */
  /* '<S2>:1:18' */
  /* '<S2>:1:19' */
  /* '<S2>:1:21' */
  c = rt_powd_snf(vc_controller_1_U.epsilon, 2.0);
  kp[0] = vc_controller_1_U.kp[0] / c;
  kp[1] = vc_controller_1_U.kp[1] / c;
  kp[2] = vc_controller_1_U.kp[2] / c;

  /* '<S2>:1:22' */
  kd[0] = vc_controller_1_U.kd[0] / vc_controller_1_U.epsilon;
  kd[1] = vc_controller_1_U.kd[1] / vc_controller_1_U.epsilon;
  kd[2] = vc_controller_1_U.kd[2] / vc_controller_1_U.epsilon;

  /* % Compute Control Law */
  /* whos */
  /* '<S2>:1:26' */
  for (i = 0; i < 11; i++) {
    c = 0.0;
    for (i_0 = 0; i_0 < 11; i_0++) {
      c += C[11 * i_0 + i] * vc_controller_1_B.VectorConcatenate[13 + b[i_0]];
    }

    C_1[i] = (c + G[i]) + DampingSpringTorque[i];
  }

  for (i = 0; i < 11; i++) {
    B_HarmonicDrive[i] = 0.0;
    for (i_0 = 0; i_0 < 11; i_0++) {
      B_HarmonicDrive[i] += vc_controller_1_B.VectorConcatenate[13 + b[i_0]] *
        0.0;
    }

    C_0[i] = C_1[i] + B_HarmonicDrive[i];
  }

  memcpy(&tmp_1[0], &vc_controller_1_P.Constant13_Value[0], 24U * sizeof(real_T));
  ATRIAS2D_SS_feedback_control_Ro(rtb_Clock, tmp, tmp_0, D, C_0, B,
    vc_controller_1_P.Constant12_Value, tmp_1,
    vc_controller_1_P.Constant14_Value, *(real_T (*)[2])&kp[0], *(real_T (*)[2])
    &kd[0], vc_controller_1_P.Constant24_Value,
    vc_controller_1_P.Constant25_Value, vc_controller_1_P.Constant23_Value,
    vc_controller_1_P.stance_leg_Value, vc_controller_1_P.Constant30_Value,
    vc_controller_1_P.Constant31_Value, vc_controller_1_U.s_mode,
    vc_controller_1_U.s_freq, u, y, dy, &c, &ds);

  /* '<S2>:1:27' */
  /* %eml */
  /*  q vector 13 by 1 */
  /*  dq vector 13 by 1 */
  /*  kp vector 1 by 1 */
  /*  kd vector 1 by 1 */
  /* transform to OSU coordinates */
  /* '<S2>:1:32' */
  /* '<S2>:1:33' */
  /* '<S2>:1:34' */
  /* '<S2>:1:36' */
  sat_val[0] = vc_controller_1_U.sat_val[0];
  sat_val[1] = vc_controller_1_U.sat_val[0];
  sat_val[2] = vc_controller_1_U.sat_val[1];
  sat_val[3] = vc_controller_1_U.sat_val[0];
  sat_val[4] = vc_controller_1_U.sat_val[0];
  sat_val[5] = vc_controller_1_U.sat_val[1];

  /* '<S2>:1:37' */
  x[0] = u[0];
  x[1] = u[1];
  x[2] = -((-kp[2] * q[7] + -0.0 * q[12]) - (kd[2] * dq[7] + 0.0 * dq[12]));
  x[3] = u[2];
  x[4] = u[3];
  x[5] = (-0.0 * q[7] + -kp[2] * q[12]) - (0.0 * dq[7] + kd[2] * dq[12]);

  /* '<S2>:1:38' */
  for (i = 0; i < 6; i++) {
    /* Outport: '<Root>/u' incorporates:
     *  MATLAB Function: '<Root>/controller'
     */
    vc_controller_1_Y.u[i] = fmax(fmin(x[i], sat_val[i]), -sat_val[i]);
  }

  /* Outport: '<Root>/y' incorporates:
   *  Gain: '<Root>/rad2deg1'
   *  MATLAB Function: '<Root>/controller'
   */
  vc_controller_1_Y.y[0] = vc_controller_1_P.rad2deg1_Gain * y[0];
  vc_controller_1_Y.y[1] = vc_controller_1_P.rad2deg1_Gain * y[1];
  vc_controller_1_Y.y[2] = vc_controller_1_P.rad2deg1_Gain * q[7];
  vc_controller_1_Y.y[3] = vc_controller_1_P.rad2deg1_Gain * y[2];
  vc_controller_1_Y.y[4] = vc_controller_1_P.rad2deg1_Gain * y[3];
  vc_controller_1_Y.y[5] = vc_controller_1_P.rad2deg1_Gain * q[12];

  /* Outport: '<Root>/dy' incorporates:
   *  Gain: '<Root>/rad2deg2'
   *  MATLAB Function: '<Root>/controller'
   */
  vc_controller_1_Y.dy[0] = vc_controller_1_P.rad2deg2_Gain * dy[0];
  vc_controller_1_Y.dy[1] = vc_controller_1_P.rad2deg2_Gain * dy[1];
  vc_controller_1_Y.dy[2] = vc_controller_1_P.rad2deg2_Gain * dq[7];
  vc_controller_1_Y.dy[3] = vc_controller_1_P.rad2deg2_Gain * dy[2];
  vc_controller_1_Y.dy[4] = vc_controller_1_P.rad2deg2_Gain * dy[3];
  vc_controller_1_Y.dy[5] = vc_controller_1_P.rad2deg2_Gain * dq[12];

  /* Outport: '<Root>/s' incorporates:
   *  MATLAB Function: '<Root>/controller'
   */
  vc_controller_1_Y.s = c;

  /* Outport: '<Root>/ds' incorporates:
   *  MATLAB Function: '<Root>/controller'
   */
  vc_controller_1_Y.ds = ds;

  /* Update for DiscreteStateSpace: '<S1>/DirtyDerivative' */
  {
    real_T xnew[4];
    xnew[0] = (vc_controller_1_P.DirtyDerivative_A[0])*
      vc_controller_1_DWork.DirtyDerivative_DSTATE[0]
      + (vc_controller_1_P.DirtyDerivative_A[1])*
      vc_controller_1_DWork.DirtyDerivative_DSTATE[1]
      + (vc_controller_1_P.DirtyDerivative_A[2])*
      vc_controller_1_DWork.DirtyDerivative_DSTATE[2]
      + (vc_controller_1_P.DirtyDerivative_A[3])*
      vc_controller_1_DWork.DirtyDerivative_DSTATE[3];
    xnew[0] += (vc_controller_1_P.DirtyDerivative_B[0])*vc_controller_1_B.q_um[0];
    xnew[1] = (vc_controller_1_P.DirtyDerivative_A[4])*
      vc_controller_1_DWork.DirtyDerivative_DSTATE[0]
      + (vc_controller_1_P.DirtyDerivative_A[5])*
      vc_controller_1_DWork.DirtyDerivative_DSTATE[1]
      + (vc_controller_1_P.DirtyDerivative_A[6])*
      vc_controller_1_DWork.DirtyDerivative_DSTATE[2]
      + (vc_controller_1_P.DirtyDerivative_A[7])*
      vc_controller_1_DWork.DirtyDerivative_DSTATE[3];
    xnew[1] += (vc_controller_1_P.DirtyDerivative_B[1])*vc_controller_1_B.q_um[0];
    xnew[2] = (vc_controller_1_P.DirtyDerivative_A[8])*
      vc_controller_1_DWork.DirtyDerivative_DSTATE[0]
      + (vc_controller_1_P.DirtyDerivative_A[9])*
      vc_controller_1_DWork.DirtyDerivative_DSTATE[1]
      + (vc_controller_1_P.DirtyDerivative_A[10])*
      vc_controller_1_DWork.DirtyDerivative_DSTATE[2]
      + (vc_controller_1_P.DirtyDerivative_A[11])*
      vc_controller_1_DWork.DirtyDerivative_DSTATE[3];
    xnew[2] += (vc_controller_1_P.DirtyDerivative_B[2])*vc_controller_1_B.q_um[0];
    xnew[3] = (vc_controller_1_P.DirtyDerivative_A[12])*
      vc_controller_1_DWork.DirtyDerivative_DSTATE[0]
      + (vc_controller_1_P.DirtyDerivative_A[13])*
      vc_controller_1_DWork.DirtyDerivative_DSTATE[1]
      + (vc_controller_1_P.DirtyDerivative_A[14])*
      vc_controller_1_DWork.DirtyDerivative_DSTATE[2]
      + (vc_controller_1_P.DirtyDerivative_A[15])*
      vc_controller_1_DWork.DirtyDerivative_DSTATE[3];
    xnew[3] += (vc_controller_1_P.DirtyDerivative_B[3])*vc_controller_1_B.q_um[0];
    (void) memcpy(&vc_controller_1_DWork.DirtyDerivative_DSTATE[0], xnew,
                  sizeof(real_T)*4);
  }

  /* Update for DiscreteStateSpace: '<S1>/DirtyDerivative1' */
  {
    real_T xnew[4];
    xnew[0] = (vc_controller_1_P.DirtyDerivative1_A[0])*
      vc_controller_1_DWork.DirtyDerivative1_DSTATE[0]
      + (vc_controller_1_P.DirtyDerivative1_A[1])*
      vc_controller_1_DWork.DirtyDerivative1_DSTATE[1]
      + (vc_controller_1_P.DirtyDerivative1_A[2])*
      vc_controller_1_DWork.DirtyDerivative1_DSTATE[2]
      + (vc_controller_1_P.DirtyDerivative1_A[3])*
      vc_controller_1_DWork.DirtyDerivative1_DSTATE[3];
    xnew[0] += (vc_controller_1_P.DirtyDerivative1_B[0])*vc_controller_1_B.q_um
      [1];
    xnew[1] = (vc_controller_1_P.DirtyDerivative1_A[4])*
      vc_controller_1_DWork.DirtyDerivative1_DSTATE[0]
      + (vc_controller_1_P.DirtyDerivative1_A[5])*
      vc_controller_1_DWork.DirtyDerivative1_DSTATE[1]
      + (vc_controller_1_P.DirtyDerivative1_A[6])*
      vc_controller_1_DWork.DirtyDerivative1_DSTATE[2]
      + (vc_controller_1_P.DirtyDerivative1_A[7])*
      vc_controller_1_DWork.DirtyDerivative1_DSTATE[3];
    xnew[1] += (vc_controller_1_P.DirtyDerivative1_B[1])*vc_controller_1_B.q_um
      [1];
    xnew[2] = (vc_controller_1_P.DirtyDerivative1_A[8])*
      vc_controller_1_DWork.DirtyDerivative1_DSTATE[0]
      + (vc_controller_1_P.DirtyDerivative1_A[9])*
      vc_controller_1_DWork.DirtyDerivative1_DSTATE[1]
      + (vc_controller_1_P.DirtyDerivative1_A[10])*
      vc_controller_1_DWork.DirtyDerivative1_DSTATE[2]
      + (vc_controller_1_P.DirtyDerivative1_A[11])*
      vc_controller_1_DWork.DirtyDerivative1_DSTATE[3];
    xnew[2] += (vc_controller_1_P.DirtyDerivative1_B[2])*vc_controller_1_B.q_um
      [1];
    xnew[3] = (vc_controller_1_P.DirtyDerivative1_A[12])*
      vc_controller_1_DWork.DirtyDerivative1_DSTATE[0]
      + (vc_controller_1_P.DirtyDerivative1_A[13])*
      vc_controller_1_DWork.DirtyDerivative1_DSTATE[1]
      + (vc_controller_1_P.DirtyDerivative1_A[14])*
      vc_controller_1_DWork.DirtyDerivative1_DSTATE[2]
      + (vc_controller_1_P.DirtyDerivative1_A[15])*
      vc_controller_1_DWork.DirtyDerivative1_DSTATE[3];
    xnew[3] += (vc_controller_1_P.DirtyDerivative1_B[3])*vc_controller_1_B.q_um
      [1];
    (void) memcpy(&vc_controller_1_DWork.DirtyDerivative1_DSTATE[0], xnew,
                  sizeof(real_T)*4);
  }

  /* Update for DiscreteStateSpace: '<S1>/DirtyDerivative2' */
  {
    real_T xnew[4];
    xnew[0] = (vc_controller_1_P.DirtyDerivative2_A[0])*
      vc_controller_1_DWork.DirtyDerivative2_DSTATE[0]
      + (vc_controller_1_P.DirtyDerivative2_A[1])*
      vc_controller_1_DWork.DirtyDerivative2_DSTATE[1]
      + (vc_controller_1_P.DirtyDerivative2_A[2])*
      vc_controller_1_DWork.DirtyDerivative2_DSTATE[2]
      + (vc_controller_1_P.DirtyDerivative2_A[3])*
      vc_controller_1_DWork.DirtyDerivative2_DSTATE[3];
    xnew[0] += (vc_controller_1_P.DirtyDerivative2_B[0])*vc_controller_1_B.q_um
      [2];
    xnew[1] = (vc_controller_1_P.DirtyDerivative2_A[4])*
      vc_controller_1_DWork.DirtyDerivative2_DSTATE[0]
      + (vc_controller_1_P.DirtyDerivative2_A[5])*
      vc_controller_1_DWork.DirtyDerivative2_DSTATE[1]
      + (vc_controller_1_P.DirtyDerivative2_A[6])*
      vc_controller_1_DWork.DirtyDerivative2_DSTATE[2]
      + (vc_controller_1_P.DirtyDerivative2_A[7])*
      vc_controller_1_DWork.DirtyDerivative2_DSTATE[3];
    xnew[1] += (vc_controller_1_P.DirtyDerivative2_B[1])*vc_controller_1_B.q_um
      [2];
    xnew[2] = (vc_controller_1_P.DirtyDerivative2_A[8])*
      vc_controller_1_DWork.DirtyDerivative2_DSTATE[0]
      + (vc_controller_1_P.DirtyDerivative2_A[9])*
      vc_controller_1_DWork.DirtyDerivative2_DSTATE[1]
      + (vc_controller_1_P.DirtyDerivative2_A[10])*
      vc_controller_1_DWork.DirtyDerivative2_DSTATE[2]
      + (vc_controller_1_P.DirtyDerivative2_A[11])*
      vc_controller_1_DWork.DirtyDerivative2_DSTATE[3];
    xnew[2] += (vc_controller_1_P.DirtyDerivative2_B[2])*vc_controller_1_B.q_um
      [2];
    xnew[3] = (vc_controller_1_P.DirtyDerivative2_A[12])*
      vc_controller_1_DWork.DirtyDerivative2_DSTATE[0]
      + (vc_controller_1_P.DirtyDerivative2_A[13])*
      vc_controller_1_DWork.DirtyDerivative2_DSTATE[1]
      + (vc_controller_1_P.DirtyDerivative2_A[14])*
      vc_controller_1_DWork.DirtyDerivative2_DSTATE[2]
      + (vc_controller_1_P.DirtyDerivative2_A[15])*
      vc_controller_1_DWork.DirtyDerivative2_DSTATE[3];
    xnew[3] += (vc_controller_1_P.DirtyDerivative2_B[3])*vc_controller_1_B.q_um
      [2];
    (void) memcpy(&vc_controller_1_DWork.DirtyDerivative2_DSTATE[0], xnew,
                  sizeof(real_T)*4);
  }

  /* Update for DiscreteStateSpace: '<S1>/DirtyDerivative3' */
  {
    real_T xnew[4];
    xnew[0] = (vc_controller_1_P.DirtyDerivative3_A[0])*
      vc_controller_1_DWork.DirtyDerivative3_DSTATE[0]
      + (vc_controller_1_P.DirtyDerivative3_A[1])*
      vc_controller_1_DWork.DirtyDerivative3_DSTATE[1]
      + (vc_controller_1_P.DirtyDerivative3_A[2])*
      vc_controller_1_DWork.DirtyDerivative3_DSTATE[2]
      + (vc_controller_1_P.DirtyDerivative3_A[3])*
      vc_controller_1_DWork.DirtyDerivative3_DSTATE[3];
    xnew[0] += (vc_controller_1_P.DirtyDerivative3_B[0])*vc_controller_1_B.q_um
      [3];
    xnew[1] = (vc_controller_1_P.DirtyDerivative3_A[4])*
      vc_controller_1_DWork.DirtyDerivative3_DSTATE[0]
      + (vc_controller_1_P.DirtyDerivative3_A[5])*
      vc_controller_1_DWork.DirtyDerivative3_DSTATE[1]
      + (vc_controller_1_P.DirtyDerivative3_A[6])*
      vc_controller_1_DWork.DirtyDerivative3_DSTATE[2]
      + (vc_controller_1_P.DirtyDerivative3_A[7])*
      vc_controller_1_DWork.DirtyDerivative3_DSTATE[3];
    xnew[1] += (vc_controller_1_P.DirtyDerivative3_B[1])*vc_controller_1_B.q_um
      [3];
    xnew[2] = (vc_controller_1_P.DirtyDerivative3_A[8])*
      vc_controller_1_DWork.DirtyDerivative3_DSTATE[0]
      + (vc_controller_1_P.DirtyDerivative3_A[9])*
      vc_controller_1_DWork.DirtyDerivative3_DSTATE[1]
      + (vc_controller_1_P.DirtyDerivative3_A[10])*
      vc_controller_1_DWork.DirtyDerivative3_DSTATE[2]
      + (vc_controller_1_P.DirtyDerivative3_A[11])*
      vc_controller_1_DWork.DirtyDerivative3_DSTATE[3];
    xnew[2] += (vc_controller_1_P.DirtyDerivative3_B[2])*vc_controller_1_B.q_um
      [3];
    xnew[3] = (vc_controller_1_P.DirtyDerivative3_A[12])*
      vc_controller_1_DWork.DirtyDerivative3_DSTATE[0]
      + (vc_controller_1_P.DirtyDerivative3_A[13])*
      vc_controller_1_DWork.DirtyDerivative3_DSTATE[1]
      + (vc_controller_1_P.DirtyDerivative3_A[14])*
      vc_controller_1_DWork.DirtyDerivative3_DSTATE[2]
      + (vc_controller_1_P.DirtyDerivative3_A[15])*
      vc_controller_1_DWork.DirtyDerivative3_DSTATE[3];
    xnew[3] += (vc_controller_1_P.DirtyDerivative3_B[3])*vc_controller_1_B.q_um
      [3];
    (void) memcpy(&vc_controller_1_DWork.DirtyDerivative3_DSTATE[0], xnew,
                  sizeof(real_T)*4);
  }

  /* Update for DiscreteStateSpace: '<S1>/DirtyDerivative4' */
  {
    real_T xnew[4];
    xnew[0] = (vc_controller_1_P.DirtyDerivative4_A[0])*
      vc_controller_1_DWork.DirtyDerivative4_DSTATE[0]
      + (vc_controller_1_P.DirtyDerivative4_A[1])*
      vc_controller_1_DWork.DirtyDerivative4_DSTATE[1]
      + (vc_controller_1_P.DirtyDerivative4_A[2])*
      vc_controller_1_DWork.DirtyDerivative4_DSTATE[2]
      + (vc_controller_1_P.DirtyDerivative4_A[3])*
      vc_controller_1_DWork.DirtyDerivative4_DSTATE[3];
    xnew[0] += (vc_controller_1_P.DirtyDerivative4_B[0])*vc_controller_1_B.q_um
      [4];
    xnew[1] = (vc_controller_1_P.DirtyDerivative4_A[4])*
      vc_controller_1_DWork.DirtyDerivative4_DSTATE[0]
      + (vc_controller_1_P.DirtyDerivative4_A[5])*
      vc_controller_1_DWork.DirtyDerivative4_DSTATE[1]
      + (vc_controller_1_P.DirtyDerivative4_A[6])*
      vc_controller_1_DWork.DirtyDerivative4_DSTATE[2]
      + (vc_controller_1_P.DirtyDerivative4_A[7])*
      vc_controller_1_DWork.DirtyDerivative4_DSTATE[3];
    xnew[1] += (vc_controller_1_P.DirtyDerivative4_B[1])*vc_controller_1_B.q_um
      [4];
    xnew[2] = (vc_controller_1_P.DirtyDerivative4_A[8])*
      vc_controller_1_DWork.DirtyDerivative4_DSTATE[0]
      + (vc_controller_1_P.DirtyDerivative4_A[9])*
      vc_controller_1_DWork.DirtyDerivative4_DSTATE[1]
      + (vc_controller_1_P.DirtyDerivative4_A[10])*
      vc_controller_1_DWork.DirtyDerivative4_DSTATE[2]
      + (vc_controller_1_P.DirtyDerivative4_A[11])*
      vc_controller_1_DWork.DirtyDerivative4_DSTATE[3];
    xnew[2] += (vc_controller_1_P.DirtyDerivative4_B[2])*vc_controller_1_B.q_um
      [4];
    xnew[3] = (vc_controller_1_P.DirtyDerivative4_A[12])*
      vc_controller_1_DWork.DirtyDerivative4_DSTATE[0]
      + (vc_controller_1_P.DirtyDerivative4_A[13])*
      vc_controller_1_DWork.DirtyDerivative4_DSTATE[1]
      + (vc_controller_1_P.DirtyDerivative4_A[14])*
      vc_controller_1_DWork.DirtyDerivative4_DSTATE[2]
      + (vc_controller_1_P.DirtyDerivative4_A[15])*
      vc_controller_1_DWork.DirtyDerivative4_DSTATE[3];
    xnew[3] += (vc_controller_1_P.DirtyDerivative4_B[3])*vc_controller_1_B.q_um
      [4];
    (void) memcpy(&vc_controller_1_DWork.DirtyDerivative4_DSTATE[0], xnew,
                  sizeof(real_T)*4);
  }

  /* Update for DiscreteStateSpace: '<S1>/DirtyDerivative5' */
  {
    real_T xnew[4];
    xnew[0] = (vc_controller_1_P.DirtyDerivative5_A[0])*
      vc_controller_1_DWork.DirtyDerivative5_DSTATE[0]
      + (vc_controller_1_P.DirtyDerivative5_A[1])*
      vc_controller_1_DWork.DirtyDerivative5_DSTATE[1]
      + (vc_controller_1_P.DirtyDerivative5_A[2])*
      vc_controller_1_DWork.DirtyDerivative5_DSTATE[2]
      + (vc_controller_1_P.DirtyDerivative5_A[3])*
      vc_controller_1_DWork.DirtyDerivative5_DSTATE[3];
    xnew[0] += (vc_controller_1_P.DirtyDerivative5_B[0])*vc_controller_1_B.q_um
      [5];
    xnew[1] = (vc_controller_1_P.DirtyDerivative5_A[4])*
      vc_controller_1_DWork.DirtyDerivative5_DSTATE[0]
      + (vc_controller_1_P.DirtyDerivative5_A[5])*
      vc_controller_1_DWork.DirtyDerivative5_DSTATE[1]
      + (vc_controller_1_P.DirtyDerivative5_A[6])*
      vc_controller_1_DWork.DirtyDerivative5_DSTATE[2]
      + (vc_controller_1_P.DirtyDerivative5_A[7])*
      vc_controller_1_DWork.DirtyDerivative5_DSTATE[3];
    xnew[1] += (vc_controller_1_P.DirtyDerivative5_B[1])*vc_controller_1_B.q_um
      [5];
    xnew[2] = (vc_controller_1_P.DirtyDerivative5_A[8])*
      vc_controller_1_DWork.DirtyDerivative5_DSTATE[0]
      + (vc_controller_1_P.DirtyDerivative5_A[9])*
      vc_controller_1_DWork.DirtyDerivative5_DSTATE[1]
      + (vc_controller_1_P.DirtyDerivative5_A[10])*
      vc_controller_1_DWork.DirtyDerivative5_DSTATE[2]
      + (vc_controller_1_P.DirtyDerivative5_A[11])*
      vc_controller_1_DWork.DirtyDerivative5_DSTATE[3];
    xnew[2] += (vc_controller_1_P.DirtyDerivative5_B[2])*vc_controller_1_B.q_um
      [5];
    xnew[3] = (vc_controller_1_P.DirtyDerivative5_A[12])*
      vc_controller_1_DWork.DirtyDerivative5_DSTATE[0]
      + (vc_controller_1_P.DirtyDerivative5_A[13])*
      vc_controller_1_DWork.DirtyDerivative5_DSTATE[1]
      + (vc_controller_1_P.DirtyDerivative5_A[14])*
      vc_controller_1_DWork.DirtyDerivative5_DSTATE[2]
      + (vc_controller_1_P.DirtyDerivative5_A[15])*
      vc_controller_1_DWork.DirtyDerivative5_DSTATE[3];
    xnew[3] += (vc_controller_1_P.DirtyDerivative5_B[3])*vc_controller_1_B.q_um
      [5];
    (void) memcpy(&vc_controller_1_DWork.DirtyDerivative5_DSTATE[0], xnew,
                  sizeof(real_T)*4);
  }

  /* Update for DiscreteStateSpace: '<S1>/DirtyDerivative6' */
  {
    real_T xnew[4];
    xnew[0] = (vc_controller_1_P.DirtyDerivative6_A[0])*
      vc_controller_1_DWork.DirtyDerivative6_DSTATE[0]
      + (vc_controller_1_P.DirtyDerivative6_A[1])*
      vc_controller_1_DWork.DirtyDerivative6_DSTATE[1]
      + (vc_controller_1_P.DirtyDerivative6_A[2])*
      vc_controller_1_DWork.DirtyDerivative6_DSTATE[2]
      + (vc_controller_1_P.DirtyDerivative6_A[3])*
      vc_controller_1_DWork.DirtyDerivative6_DSTATE[3];
    xnew[0] += (vc_controller_1_P.DirtyDerivative6_B[0])*vc_controller_1_B.q_um
      [6];
    xnew[1] = (vc_controller_1_P.DirtyDerivative6_A[4])*
      vc_controller_1_DWork.DirtyDerivative6_DSTATE[0]
      + (vc_controller_1_P.DirtyDerivative6_A[5])*
      vc_controller_1_DWork.DirtyDerivative6_DSTATE[1]
      + (vc_controller_1_P.DirtyDerivative6_A[6])*
      vc_controller_1_DWork.DirtyDerivative6_DSTATE[2]
      + (vc_controller_1_P.DirtyDerivative6_A[7])*
      vc_controller_1_DWork.DirtyDerivative6_DSTATE[3];
    xnew[1] += (vc_controller_1_P.DirtyDerivative6_B[1])*vc_controller_1_B.q_um
      [6];
    xnew[2] = (vc_controller_1_P.DirtyDerivative6_A[8])*
      vc_controller_1_DWork.DirtyDerivative6_DSTATE[0]
      + (vc_controller_1_P.DirtyDerivative6_A[9])*
      vc_controller_1_DWork.DirtyDerivative6_DSTATE[1]
      + (vc_controller_1_P.DirtyDerivative6_A[10])*
      vc_controller_1_DWork.DirtyDerivative6_DSTATE[2]
      + (vc_controller_1_P.DirtyDerivative6_A[11])*
      vc_controller_1_DWork.DirtyDerivative6_DSTATE[3];
    xnew[2] += (vc_controller_1_P.DirtyDerivative6_B[2])*vc_controller_1_B.q_um
      [6];
    xnew[3] = (vc_controller_1_P.DirtyDerivative6_A[12])*
      vc_controller_1_DWork.DirtyDerivative6_DSTATE[0]
      + (vc_controller_1_P.DirtyDerivative6_A[13])*
      vc_controller_1_DWork.DirtyDerivative6_DSTATE[1]
      + (vc_controller_1_P.DirtyDerivative6_A[14])*
      vc_controller_1_DWork.DirtyDerivative6_DSTATE[2]
      + (vc_controller_1_P.DirtyDerivative6_A[15])*
      vc_controller_1_DWork.DirtyDerivative6_DSTATE[3];
    xnew[3] += (vc_controller_1_P.DirtyDerivative6_B[3])*vc_controller_1_B.q_um
      [6];
    (void) memcpy(&vc_controller_1_DWork.DirtyDerivative6_DSTATE[0], xnew,
                  sizeof(real_T)*4);
  }

  /* Update for DiscreteStateSpace: '<S1>/DirtyDerivative7' */
  {
    real_T xnew[4];
    xnew[0] = (vc_controller_1_P.DirtyDerivative7_A[0])*
      vc_controller_1_DWork.DirtyDerivative7_DSTATE[0]
      + (vc_controller_1_P.DirtyDerivative7_A[1])*
      vc_controller_1_DWork.DirtyDerivative7_DSTATE[1]
      + (vc_controller_1_P.DirtyDerivative7_A[2])*
      vc_controller_1_DWork.DirtyDerivative7_DSTATE[2]
      + (vc_controller_1_P.DirtyDerivative7_A[3])*
      vc_controller_1_DWork.DirtyDerivative7_DSTATE[3];
    xnew[0] += (vc_controller_1_P.DirtyDerivative7_B[0])*vc_controller_1_B.q_um
      [7];
    xnew[1] = (vc_controller_1_P.DirtyDerivative7_A[4])*
      vc_controller_1_DWork.DirtyDerivative7_DSTATE[0]
      + (vc_controller_1_P.DirtyDerivative7_A[5])*
      vc_controller_1_DWork.DirtyDerivative7_DSTATE[1]
      + (vc_controller_1_P.DirtyDerivative7_A[6])*
      vc_controller_1_DWork.DirtyDerivative7_DSTATE[2]
      + (vc_controller_1_P.DirtyDerivative7_A[7])*
      vc_controller_1_DWork.DirtyDerivative7_DSTATE[3];
    xnew[1] += (vc_controller_1_P.DirtyDerivative7_B[1])*vc_controller_1_B.q_um
      [7];
    xnew[2] = (vc_controller_1_P.DirtyDerivative7_A[8])*
      vc_controller_1_DWork.DirtyDerivative7_DSTATE[0]
      + (vc_controller_1_P.DirtyDerivative7_A[9])*
      vc_controller_1_DWork.DirtyDerivative7_DSTATE[1]
      + (vc_controller_1_P.DirtyDerivative7_A[10])*
      vc_controller_1_DWork.DirtyDerivative7_DSTATE[2]
      + (vc_controller_1_P.DirtyDerivative7_A[11])*
      vc_controller_1_DWork.DirtyDerivative7_DSTATE[3];
    xnew[2] += (vc_controller_1_P.DirtyDerivative7_B[2])*vc_controller_1_B.q_um
      [7];
    xnew[3] = (vc_controller_1_P.DirtyDerivative7_A[12])*
      vc_controller_1_DWork.DirtyDerivative7_DSTATE[0]
      + (vc_controller_1_P.DirtyDerivative7_A[13])*
      vc_controller_1_DWork.DirtyDerivative7_DSTATE[1]
      + (vc_controller_1_P.DirtyDerivative7_A[14])*
      vc_controller_1_DWork.DirtyDerivative7_DSTATE[2]
      + (vc_controller_1_P.DirtyDerivative7_A[15])*
      vc_controller_1_DWork.DirtyDerivative7_DSTATE[3];
    xnew[3] += (vc_controller_1_P.DirtyDerivative7_B[3])*vc_controller_1_B.q_um
      [7];
    (void) memcpy(&vc_controller_1_DWork.DirtyDerivative7_DSTATE[0], xnew,
                  sizeof(real_T)*4);
  }

  /* Update for DiscreteStateSpace: '<S1>/DirtyDerivative8' */
  {
    real_T xnew[4];
    xnew[0] = (vc_controller_1_P.DirtyDerivative8_A[0])*
      vc_controller_1_DWork.DirtyDerivative8_DSTATE[0]
      + (vc_controller_1_P.DirtyDerivative8_A[1])*
      vc_controller_1_DWork.DirtyDerivative8_DSTATE[1]
      + (vc_controller_1_P.DirtyDerivative8_A[2])*
      vc_controller_1_DWork.DirtyDerivative8_DSTATE[2]
      + (vc_controller_1_P.DirtyDerivative8_A[3])*
      vc_controller_1_DWork.DirtyDerivative8_DSTATE[3];
    xnew[0] += (vc_controller_1_P.DirtyDerivative8_B[0])*vc_controller_1_B.q_um
      [8];
    xnew[1] = (vc_controller_1_P.DirtyDerivative8_A[4])*
      vc_controller_1_DWork.DirtyDerivative8_DSTATE[0]
      + (vc_controller_1_P.DirtyDerivative8_A[5])*
      vc_controller_1_DWork.DirtyDerivative8_DSTATE[1]
      + (vc_controller_1_P.DirtyDerivative8_A[6])*
      vc_controller_1_DWork.DirtyDerivative8_DSTATE[2]
      + (vc_controller_1_P.DirtyDerivative8_A[7])*
      vc_controller_1_DWork.DirtyDerivative8_DSTATE[3];
    xnew[1] += (vc_controller_1_P.DirtyDerivative8_B[1])*vc_controller_1_B.q_um
      [8];
    xnew[2] = (vc_controller_1_P.DirtyDerivative8_A[8])*
      vc_controller_1_DWork.DirtyDerivative8_DSTATE[0]
      + (vc_controller_1_P.DirtyDerivative8_A[9])*
      vc_controller_1_DWork.DirtyDerivative8_DSTATE[1]
      + (vc_controller_1_P.DirtyDerivative8_A[10])*
      vc_controller_1_DWork.DirtyDerivative8_DSTATE[2]
      + (vc_controller_1_P.DirtyDerivative8_A[11])*
      vc_controller_1_DWork.DirtyDerivative8_DSTATE[3];
    xnew[2] += (vc_controller_1_P.DirtyDerivative8_B[2])*vc_controller_1_B.q_um
      [8];
    xnew[3] = (vc_controller_1_P.DirtyDerivative8_A[12])*
      vc_controller_1_DWork.DirtyDerivative8_DSTATE[0]
      + (vc_controller_1_P.DirtyDerivative8_A[13])*
      vc_controller_1_DWork.DirtyDerivative8_DSTATE[1]
      + (vc_controller_1_P.DirtyDerivative8_A[14])*
      vc_controller_1_DWork.DirtyDerivative8_DSTATE[2]
      + (vc_controller_1_P.DirtyDerivative8_A[15])*
      vc_controller_1_DWork.DirtyDerivative8_DSTATE[3];
    xnew[3] += (vc_controller_1_P.DirtyDerivative8_B[3])*vc_controller_1_B.q_um
      [8];
    (void) memcpy(&vc_controller_1_DWork.DirtyDerivative8_DSTATE[0], xnew,
                  sizeof(real_T)*4);
  }

  /* Update for DiscreteStateSpace: '<S1>/DirtyDerivative9' */
  {
    real_T xnew[4];
    xnew[0] = (vc_controller_1_P.DirtyDerivative9_A[0])*
      vc_controller_1_DWork.DirtyDerivative9_DSTATE[0]
      + (vc_controller_1_P.DirtyDerivative9_A[1])*
      vc_controller_1_DWork.DirtyDerivative9_DSTATE[1]
      + (vc_controller_1_P.DirtyDerivative9_A[2])*
      vc_controller_1_DWork.DirtyDerivative9_DSTATE[2]
      + (vc_controller_1_P.DirtyDerivative9_A[3])*
      vc_controller_1_DWork.DirtyDerivative9_DSTATE[3];
    xnew[0] += (vc_controller_1_P.DirtyDerivative9_B[0])*vc_controller_1_B.q_um
      [9];
    xnew[1] = (vc_controller_1_P.DirtyDerivative9_A[4])*
      vc_controller_1_DWork.DirtyDerivative9_DSTATE[0]
      + (vc_controller_1_P.DirtyDerivative9_A[5])*
      vc_controller_1_DWork.DirtyDerivative9_DSTATE[1]
      + (vc_controller_1_P.DirtyDerivative9_A[6])*
      vc_controller_1_DWork.DirtyDerivative9_DSTATE[2]
      + (vc_controller_1_P.DirtyDerivative9_A[7])*
      vc_controller_1_DWork.DirtyDerivative9_DSTATE[3];
    xnew[1] += (vc_controller_1_P.DirtyDerivative9_B[1])*vc_controller_1_B.q_um
      [9];
    xnew[2] = (vc_controller_1_P.DirtyDerivative9_A[8])*
      vc_controller_1_DWork.DirtyDerivative9_DSTATE[0]
      + (vc_controller_1_P.DirtyDerivative9_A[9])*
      vc_controller_1_DWork.DirtyDerivative9_DSTATE[1]
      + (vc_controller_1_P.DirtyDerivative9_A[10])*
      vc_controller_1_DWork.DirtyDerivative9_DSTATE[2]
      + (vc_controller_1_P.DirtyDerivative9_A[11])*
      vc_controller_1_DWork.DirtyDerivative9_DSTATE[3];
    xnew[2] += (vc_controller_1_P.DirtyDerivative9_B[2])*vc_controller_1_B.q_um
      [9];
    xnew[3] = (vc_controller_1_P.DirtyDerivative9_A[12])*
      vc_controller_1_DWork.DirtyDerivative9_DSTATE[0]
      + (vc_controller_1_P.DirtyDerivative9_A[13])*
      vc_controller_1_DWork.DirtyDerivative9_DSTATE[1]
      + (vc_controller_1_P.DirtyDerivative9_A[14])*
      vc_controller_1_DWork.DirtyDerivative9_DSTATE[2]
      + (vc_controller_1_P.DirtyDerivative9_A[15])*
      vc_controller_1_DWork.DirtyDerivative9_DSTATE[3];
    xnew[3] += (vc_controller_1_P.DirtyDerivative9_B[3])*vc_controller_1_B.q_um
      [9];
    (void) memcpy(&vc_controller_1_DWork.DirtyDerivative9_DSTATE[0], xnew,
                  sizeof(real_T)*4);
  }

  /* Update for DiscreteStateSpace: '<S1>/DirtyDerivative10' */
  {
    real_T xnew[4];
    xnew[0] = (vc_controller_1_P.DirtyDerivative10_A[0])*
      vc_controller_1_DWork.DirtyDerivative10_DSTATE[0]
      + (vc_controller_1_P.DirtyDerivative10_A[1])*
      vc_controller_1_DWork.DirtyDerivative10_DSTATE[1]
      + (vc_controller_1_P.DirtyDerivative10_A[2])*
      vc_controller_1_DWork.DirtyDerivative10_DSTATE[2]
      + (vc_controller_1_P.DirtyDerivative10_A[3])*
      vc_controller_1_DWork.DirtyDerivative10_DSTATE[3];
    xnew[0] += (vc_controller_1_P.DirtyDerivative10_B[0])*
      vc_controller_1_B.q_um[10];
    xnew[1] = (vc_controller_1_P.DirtyDerivative10_A[4])*
      vc_controller_1_DWork.DirtyDerivative10_DSTATE[0]
      + (vc_controller_1_P.DirtyDerivative10_A[5])*
      vc_controller_1_DWork.DirtyDerivative10_DSTATE[1]
      + (vc_controller_1_P.DirtyDerivative10_A[6])*
      vc_controller_1_DWork.DirtyDerivative10_DSTATE[2]
      + (vc_controller_1_P.DirtyDerivative10_A[7])*
      vc_controller_1_DWork.DirtyDerivative10_DSTATE[3];
    xnew[1] += (vc_controller_1_P.DirtyDerivative10_B[1])*
      vc_controller_1_B.q_um[10];
    xnew[2] = (vc_controller_1_P.DirtyDerivative10_A[8])*
      vc_controller_1_DWork.DirtyDerivative10_DSTATE[0]
      + (vc_controller_1_P.DirtyDerivative10_A[9])*
      vc_controller_1_DWork.DirtyDerivative10_DSTATE[1]
      + (vc_controller_1_P.DirtyDerivative10_A[10])*
      vc_controller_1_DWork.DirtyDerivative10_DSTATE[2]
      + (vc_controller_1_P.DirtyDerivative10_A[11])*
      vc_controller_1_DWork.DirtyDerivative10_DSTATE[3];
    xnew[2] += (vc_controller_1_P.DirtyDerivative10_B[2])*
      vc_controller_1_B.q_um[10];
    xnew[3] = (vc_controller_1_P.DirtyDerivative10_A[12])*
      vc_controller_1_DWork.DirtyDerivative10_DSTATE[0]
      + (vc_controller_1_P.DirtyDerivative10_A[13])*
      vc_controller_1_DWork.DirtyDerivative10_DSTATE[1]
      + (vc_controller_1_P.DirtyDerivative10_A[14])*
      vc_controller_1_DWork.DirtyDerivative10_DSTATE[2]
      + (vc_controller_1_P.DirtyDerivative10_A[15])*
      vc_controller_1_DWork.DirtyDerivative10_DSTATE[3];
    xnew[3] += (vc_controller_1_P.DirtyDerivative10_B[3])*
      vc_controller_1_B.q_um[10];
    (void) memcpy(&vc_controller_1_DWork.DirtyDerivative10_DSTATE[0], xnew,
                  sizeof(real_T)*4);
  }

  /* Update for DiscreteStateSpace: '<S1>/DirtyDerivative11' */
  {
    real_T xnew[4];
    xnew[0] = (vc_controller_1_P.DirtyDerivative11_A[0])*
      vc_controller_1_DWork.DirtyDerivative11_DSTATE[0]
      + (vc_controller_1_P.DirtyDerivative11_A[1])*
      vc_controller_1_DWork.DirtyDerivative11_DSTATE[1]
      + (vc_controller_1_P.DirtyDerivative11_A[2])*
      vc_controller_1_DWork.DirtyDerivative11_DSTATE[2]
      + (vc_controller_1_P.DirtyDerivative11_A[3])*
      vc_controller_1_DWork.DirtyDerivative11_DSTATE[3];
    xnew[0] += (vc_controller_1_P.DirtyDerivative11_B[0])*
      vc_controller_1_B.q_um[11];
    xnew[1] = (vc_controller_1_P.DirtyDerivative11_A[4])*
      vc_controller_1_DWork.DirtyDerivative11_DSTATE[0]
      + (vc_controller_1_P.DirtyDerivative11_A[5])*
      vc_controller_1_DWork.DirtyDerivative11_DSTATE[1]
      + (vc_controller_1_P.DirtyDerivative11_A[6])*
      vc_controller_1_DWork.DirtyDerivative11_DSTATE[2]
      + (vc_controller_1_P.DirtyDerivative11_A[7])*
      vc_controller_1_DWork.DirtyDerivative11_DSTATE[3];
    xnew[1] += (vc_controller_1_P.DirtyDerivative11_B[1])*
      vc_controller_1_B.q_um[11];
    xnew[2] = (vc_controller_1_P.DirtyDerivative11_A[8])*
      vc_controller_1_DWork.DirtyDerivative11_DSTATE[0]
      + (vc_controller_1_P.DirtyDerivative11_A[9])*
      vc_controller_1_DWork.DirtyDerivative11_DSTATE[1]
      + (vc_controller_1_P.DirtyDerivative11_A[10])*
      vc_controller_1_DWork.DirtyDerivative11_DSTATE[2]
      + (vc_controller_1_P.DirtyDerivative11_A[11])*
      vc_controller_1_DWork.DirtyDerivative11_DSTATE[3];
    xnew[2] += (vc_controller_1_P.DirtyDerivative11_B[2])*
      vc_controller_1_B.q_um[11];
    xnew[3] = (vc_controller_1_P.DirtyDerivative11_A[12])*
      vc_controller_1_DWork.DirtyDerivative11_DSTATE[0]
      + (vc_controller_1_P.DirtyDerivative11_A[13])*
      vc_controller_1_DWork.DirtyDerivative11_DSTATE[1]
      + (vc_controller_1_P.DirtyDerivative11_A[14])*
      vc_controller_1_DWork.DirtyDerivative11_DSTATE[2]
      + (vc_controller_1_P.DirtyDerivative11_A[15])*
      vc_controller_1_DWork.DirtyDerivative11_DSTATE[3];
    xnew[3] += (vc_controller_1_P.DirtyDerivative11_B[3])*
      vc_controller_1_B.q_um[11];
    (void) memcpy(&vc_controller_1_DWork.DirtyDerivative11_DSTATE[0], xnew,
                  sizeof(real_T)*4);
  }

  /* Update for DiscreteStateSpace: '<S1>/DirtyDerivative12' */
  {
    real_T xnew[4];
    xnew[0] = (vc_controller_1_P.DirtyDerivative12_A[0])*
      vc_controller_1_DWork.DirtyDerivative12_DSTATE[0]
      + (vc_controller_1_P.DirtyDerivative12_A[1])*
      vc_controller_1_DWork.DirtyDerivative12_DSTATE[1]
      + (vc_controller_1_P.DirtyDerivative12_A[2])*
      vc_controller_1_DWork.DirtyDerivative12_DSTATE[2]
      + (vc_controller_1_P.DirtyDerivative12_A[3])*
      vc_controller_1_DWork.DirtyDerivative12_DSTATE[3];
    xnew[0] += (vc_controller_1_P.DirtyDerivative12_B[0])*
      vc_controller_1_B.q_um[12];
    xnew[1] = (vc_controller_1_P.DirtyDerivative12_A[4])*
      vc_controller_1_DWork.DirtyDerivative12_DSTATE[0]
      + (vc_controller_1_P.DirtyDerivative12_A[5])*
      vc_controller_1_DWork.DirtyDerivative12_DSTATE[1]
      + (vc_controller_1_P.DirtyDerivative12_A[6])*
      vc_controller_1_DWork.DirtyDerivative12_DSTATE[2]
      + (vc_controller_1_P.DirtyDerivative12_A[7])*
      vc_controller_1_DWork.DirtyDerivative12_DSTATE[3];
    xnew[1] += (vc_controller_1_P.DirtyDerivative12_B[1])*
      vc_controller_1_B.q_um[12];
    xnew[2] = (vc_controller_1_P.DirtyDerivative12_A[8])*
      vc_controller_1_DWork.DirtyDerivative12_DSTATE[0]
      + (vc_controller_1_P.DirtyDerivative12_A[9])*
      vc_controller_1_DWork.DirtyDerivative12_DSTATE[1]
      + (vc_controller_1_P.DirtyDerivative12_A[10])*
      vc_controller_1_DWork.DirtyDerivative12_DSTATE[2]
      + (vc_controller_1_P.DirtyDerivative12_A[11])*
      vc_controller_1_DWork.DirtyDerivative12_DSTATE[3];
    xnew[2] += (vc_controller_1_P.DirtyDerivative12_B[2])*
      vc_controller_1_B.q_um[12];
    xnew[3] = (vc_controller_1_P.DirtyDerivative12_A[12])*
      vc_controller_1_DWork.DirtyDerivative12_DSTATE[0]
      + (vc_controller_1_P.DirtyDerivative12_A[13])*
      vc_controller_1_DWork.DirtyDerivative12_DSTATE[1]
      + (vc_controller_1_P.DirtyDerivative12_A[14])*
      vc_controller_1_DWork.DirtyDerivative12_DSTATE[2]
      + (vc_controller_1_P.DirtyDerivative12_A[15])*
      vc_controller_1_DWork.DirtyDerivative12_DSTATE[3];
    xnew[3] += (vc_controller_1_P.DirtyDerivative12_B[3])*
      vc_controller_1_B.q_um[12];
    (void) memcpy(&vc_controller_1_DWork.DirtyDerivative12_DSTATE[0], xnew,
                  sizeof(real_T)*4);
  }

  /* Update for DiscreteFilter: '<Root>/Discrete Filter' */
  vc_controller_1_DWork.DiscreteFilter_DSTATE =
    vc_controller_1_DWork.DiscreteFilter_tmp;

  /* Update absolute time for base rate */
  /* The "clockTick0" counts the number of times the code of this task has
   * been executed. The absolute time is the multiplication of "clockTick0"
   * and "Timing.stepSize0". Size of "clockTick0" ensures timer will not
   * overflow during the application lifespan selected.
   */
  vc_controller_1_M->Timing.t[0] =
    (++vc_controller_1_M->Timing.clockTick0) *
    vc_controller_1_M->Timing.stepSize0;

  {
    /* Update absolute timer for sample time: [0.001s, 0.0s] */
    /* The "clockTick1" counts the number of times the code of this task has
     * been executed. The resolution of this integer timer is 0.001, which is the step size
     * of the task. Size of "clockTick1" ensures timer will not overflow during the
     * application lifespan selected.
     */
    vc_controller_1_M->Timing.clockTick1++;
  }
}

/* Model initialize function */
void vc_controller_1_initialize(void)
{
  /* Registration code */

  /* initialize non-finites */
  rt_InitInfAndNaN(sizeof(real_T));

  /* initialize real-time model */
  (void) memset((void *)vc_controller_1_M, 0,
                sizeof(RT_MODEL_vc_controller_1));

  {
    /* Setup solver object */
    rtsiSetSimTimeStepPtr(&vc_controller_1_M->solverInfo,
                          &vc_controller_1_M->Timing.simTimeStep);
    rtsiSetTPtr(&vc_controller_1_M->solverInfo, &rtmGetTPtr(vc_controller_1_M));
    rtsiSetStepSizePtr(&vc_controller_1_M->solverInfo,
                       &vc_controller_1_M->Timing.stepSize0);
    rtsiSetErrorStatusPtr(&vc_controller_1_M->solverInfo, ((const char_T **)
      (&rtmGetErrorStatus(vc_controller_1_M))));
    rtsiSetRTModelPtr(&vc_controller_1_M->solverInfo, vc_controller_1_M);
  }

  rtsiSetSimTimeStep(&vc_controller_1_M->solverInfo, MAJOR_TIME_STEP);
  rtsiSetSolverName(&vc_controller_1_M->solverInfo,"FixedStepDiscrete");
  rtmSetTPtr(vc_controller_1_M, &vc_controller_1_M->Timing.tArray[0]);
  vc_controller_1_M->Timing.stepSize0 = 0.001;

  /* block I/O */
  (void) memset(((void *) &vc_controller_1_B), 0,
                sizeof(BlockIO_vc_controller_1));

  /* states (dwork) */
  (void) memset((void *)&vc_controller_1_DWork, 0,
                sizeof(D_Work_vc_controller_1));

  /* external inputs */
  (void) memset((void *)&vc_controller_1_U, 0,
                sizeof(ExternalInputs_vc_controller_1));

  /* external outputs */
  (void) memset((void *)&vc_controller_1_Y, 0,
                sizeof(ExternalOutputs_vc_controller_1));

  /* InitializeConditions for DiscreteStateSpace: '<S1>/DirtyDerivative' */
  vc_controller_1_DWork.DirtyDerivative_DSTATE[0] =
    vc_controller_1_P.DirtyDerivative_X0[0];
  vc_controller_1_DWork.DirtyDerivative_DSTATE[1] =
    vc_controller_1_P.DirtyDerivative_X0[1];
  vc_controller_1_DWork.DirtyDerivative_DSTATE[2] =
    vc_controller_1_P.DirtyDerivative_X0[2];
  vc_controller_1_DWork.DirtyDerivative_DSTATE[3] =
    vc_controller_1_P.DirtyDerivative_X0[3];

  /* InitializeConditions for DiscreteStateSpace: '<S1>/DirtyDerivative1' */
  vc_controller_1_DWork.DirtyDerivative1_DSTATE[0] =
    vc_controller_1_P.DirtyDerivative1_X0[0];
  vc_controller_1_DWork.DirtyDerivative1_DSTATE[1] =
    vc_controller_1_P.DirtyDerivative1_X0[1];
  vc_controller_1_DWork.DirtyDerivative1_DSTATE[2] =
    vc_controller_1_P.DirtyDerivative1_X0[2];
  vc_controller_1_DWork.DirtyDerivative1_DSTATE[3] =
    vc_controller_1_P.DirtyDerivative1_X0[3];

  /* InitializeConditions for DiscreteStateSpace: '<S1>/DirtyDerivative2' */
  vc_controller_1_DWork.DirtyDerivative2_DSTATE[0] =
    vc_controller_1_P.DirtyDerivative2_X0[0];
  vc_controller_1_DWork.DirtyDerivative2_DSTATE[1] =
    vc_controller_1_P.DirtyDerivative2_X0[1];
  vc_controller_1_DWork.DirtyDerivative2_DSTATE[2] =
    vc_controller_1_P.DirtyDerivative2_X0[2];
  vc_controller_1_DWork.DirtyDerivative2_DSTATE[3] =
    vc_controller_1_P.DirtyDerivative2_X0[3];

  /* InitializeConditions for DiscreteStateSpace: '<S1>/DirtyDerivative3' */
  vc_controller_1_DWork.DirtyDerivative3_DSTATE[0] =
    vc_controller_1_P.DirtyDerivative3_X0[0];
  vc_controller_1_DWork.DirtyDerivative3_DSTATE[1] =
    vc_controller_1_P.DirtyDerivative3_X0[1];
  vc_controller_1_DWork.DirtyDerivative3_DSTATE[2] =
    vc_controller_1_P.DirtyDerivative3_X0[2];
  vc_controller_1_DWork.DirtyDerivative3_DSTATE[3] =
    vc_controller_1_P.DirtyDerivative3_X0[3];

  /* InitializeConditions for DiscreteStateSpace: '<S1>/DirtyDerivative4' */
  vc_controller_1_DWork.DirtyDerivative4_DSTATE[0] =
    vc_controller_1_P.DirtyDerivative4_X0[0];
  vc_controller_1_DWork.DirtyDerivative4_DSTATE[1] =
    vc_controller_1_P.DirtyDerivative4_X0[1];
  vc_controller_1_DWork.DirtyDerivative4_DSTATE[2] =
    vc_controller_1_P.DirtyDerivative4_X0[2];
  vc_controller_1_DWork.DirtyDerivative4_DSTATE[3] =
    vc_controller_1_P.DirtyDerivative4_X0[3];

  /* InitializeConditions for DiscreteStateSpace: '<S1>/DirtyDerivative5' */
  vc_controller_1_DWork.DirtyDerivative5_DSTATE[0] =
    vc_controller_1_P.DirtyDerivative5_X0[0];
  vc_controller_1_DWork.DirtyDerivative5_DSTATE[1] =
    vc_controller_1_P.DirtyDerivative5_X0[1];
  vc_controller_1_DWork.DirtyDerivative5_DSTATE[2] =
    vc_controller_1_P.DirtyDerivative5_X0[2];
  vc_controller_1_DWork.DirtyDerivative5_DSTATE[3] =
    vc_controller_1_P.DirtyDerivative5_X0[3];

  /* InitializeConditions for DiscreteStateSpace: '<S1>/DirtyDerivative6' */
  vc_controller_1_DWork.DirtyDerivative6_DSTATE[0] =
    vc_controller_1_P.DirtyDerivative6_X0[0];
  vc_controller_1_DWork.DirtyDerivative6_DSTATE[1] =
    vc_controller_1_P.DirtyDerivative6_X0[1];
  vc_controller_1_DWork.DirtyDerivative6_DSTATE[2] =
    vc_controller_1_P.DirtyDerivative6_X0[2];
  vc_controller_1_DWork.DirtyDerivative6_DSTATE[3] =
    vc_controller_1_P.DirtyDerivative6_X0[3];

  /* InitializeConditions for DiscreteStateSpace: '<S1>/DirtyDerivative7' */
  vc_controller_1_DWork.DirtyDerivative7_DSTATE[0] =
    vc_controller_1_P.DirtyDerivative7_X0[0];
  vc_controller_1_DWork.DirtyDerivative7_DSTATE[1] =
    vc_controller_1_P.DirtyDerivative7_X0[1];
  vc_controller_1_DWork.DirtyDerivative7_DSTATE[2] =
    vc_controller_1_P.DirtyDerivative7_X0[2];
  vc_controller_1_DWork.DirtyDerivative7_DSTATE[3] =
    vc_controller_1_P.DirtyDerivative7_X0[3];

  /* InitializeConditions for DiscreteStateSpace: '<S1>/DirtyDerivative8' */
  vc_controller_1_DWork.DirtyDerivative8_DSTATE[0] =
    vc_controller_1_P.DirtyDerivative8_X0[0];
  vc_controller_1_DWork.DirtyDerivative8_DSTATE[1] =
    vc_controller_1_P.DirtyDerivative8_X0[1];
  vc_controller_1_DWork.DirtyDerivative8_DSTATE[2] =
    vc_controller_1_P.DirtyDerivative8_X0[2];
  vc_controller_1_DWork.DirtyDerivative8_DSTATE[3] =
    vc_controller_1_P.DirtyDerivative8_X0[3];

  /* InitializeConditions for DiscreteStateSpace: '<S1>/DirtyDerivative9' */
  vc_controller_1_DWork.DirtyDerivative9_DSTATE[0] =
    vc_controller_1_P.DirtyDerivative9_X0[0];
  vc_controller_1_DWork.DirtyDerivative9_DSTATE[1] =
    vc_controller_1_P.DirtyDerivative9_X0[1];
  vc_controller_1_DWork.DirtyDerivative9_DSTATE[2] =
    vc_controller_1_P.DirtyDerivative9_X0[2];
  vc_controller_1_DWork.DirtyDerivative9_DSTATE[3] =
    vc_controller_1_P.DirtyDerivative9_X0[3];

  /* InitializeConditions for DiscreteStateSpace: '<S1>/DirtyDerivative10' */
  vc_controller_1_DWork.DirtyDerivative10_DSTATE[0] =
    vc_controller_1_P.DirtyDerivative10_X0[0];
  vc_controller_1_DWork.DirtyDerivative10_DSTATE[1] =
    vc_controller_1_P.DirtyDerivative10_X0[1];
  vc_controller_1_DWork.DirtyDerivative10_DSTATE[2] =
    vc_controller_1_P.DirtyDerivative10_X0[2];
  vc_controller_1_DWork.DirtyDerivative10_DSTATE[3] =
    vc_controller_1_P.DirtyDerivative10_X0[3];

  /* InitializeConditions for DiscreteStateSpace: '<S1>/DirtyDerivative11' */
  vc_controller_1_DWork.DirtyDerivative11_DSTATE[0] =
    vc_controller_1_P.DirtyDerivative11_X0[0];
  vc_controller_1_DWork.DirtyDerivative11_DSTATE[1] =
    vc_controller_1_P.DirtyDerivative11_X0[1];
  vc_controller_1_DWork.DirtyDerivative11_DSTATE[2] =
    vc_controller_1_P.DirtyDerivative11_X0[2];
  vc_controller_1_DWork.DirtyDerivative11_DSTATE[3] =
    vc_controller_1_P.DirtyDerivative11_X0[3];

  /* InitializeConditions for DiscreteStateSpace: '<S1>/DirtyDerivative12' */
  vc_controller_1_DWork.DirtyDerivative12_DSTATE[0] =
    vc_controller_1_P.DirtyDerivative12_X0[0];
  vc_controller_1_DWork.DirtyDerivative12_DSTATE[1] =
    vc_controller_1_P.DirtyDerivative12_X0[1];
  vc_controller_1_DWork.DirtyDerivative12_DSTATE[2] =
    vc_controller_1_P.DirtyDerivative12_X0[2];
  vc_controller_1_DWork.DirtyDerivative12_DSTATE[3] =
    vc_controller_1_P.DirtyDerivative12_X0[3];

  /* InitializeConditions for DiscreteFilter: '<Root>/Discrete Filter' */
  vc_controller_1_DWork.DiscreteFilter_DSTATE =
    vc_controller_1_P.DiscreteFilter_InitialStates;
}

/* Model terminate function */
void vc_controller_1_terminate(void)
{
  /* (no terminate code required) */
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
