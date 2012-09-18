/*
 * File: vc_controller_1_data.c
 *
 * Code generated for Simulink model 'vc_controller_1'.
 *
 * Model version                  : 1.28
 * Simulink Coder version         : 8.2 (R2012a) 29-Dec-2011
 * TLC version                    : 8.2 (Jan 25 2012)
 * C/C++ source code generated on : Tue Sep 18 16:35:06 2012
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

/* Block parameters (auto storage) */
Parameters_vc_controller_1 vc_controller_1_P = {
  1.0,                                 /* Expression: prev_leg
                                        * Referenced by: '<Root>/controller'
                                        */

  /*  Computed Parameter: DirtyDerivative_A
   * Referenced by: '<S1>/DirtyDerivative'
   */
  { 0.26941640375703091, 0.55523437264240449, -0.40763138378494,
    0.096578338543328646, -0.55523437264240516, 0.75449863027385256,
    0.22630738856557703, -0.061558493187215735, 0.40763138378494052,
    0.22630738856557686, -0.22935555597794208, 0.49517215830633282,
    0.096578338543328618, 0.061558493187215541, -0.49517215830633321,
    -0.7945594780529408 },

  /*  Computed Parameter: DirtyDerivative_B
   * Referenced by: '<S1>/DirtyDerivative'
   */
  { -12.606428657752771, 4.4889080376483328, 6.8848303100480734,
    2.9375398321742061 },

  /*  Computed Parameter: DirtyDerivative_C
   * Referenced by: '<S1>/DirtyDerivative'
   */
  { -12.606428657752776, -4.4889080376483284, -6.8848303100480726,
    2.9375398321742123 },
  200.00000000000003,                  /* Computed Parameter: DirtyDerivative_D
                                        * Referenced by: '<S1>/DirtyDerivative'
                                        */

  /*  Expression: x0
   * Referenced by: '<S1>/DirtyDerivative'
   */
  { 0.0, 0.0, 0.0, 0.0 },

  /*  Computed Parameter: DirtyDerivative1_A
   * Referenced by: '<S1>/DirtyDerivative1'
   */
  { 0.26941640375703091, 0.55523437264240449, -0.40763138378494,
    0.096578338543328646, -0.55523437264240516, 0.75449863027385256,
    0.22630738856557703, -0.061558493187215735, 0.40763138378494052,
    0.22630738856557686, -0.22935555597794208, 0.49517215830633282,
    0.096578338543328618, 0.061558493187215541, -0.49517215830633321,
    -0.7945594780529408 },

  /*  Computed Parameter: DirtyDerivative1_B
   * Referenced by: '<S1>/DirtyDerivative1'
   */
  { -12.606428657752771, 4.4889080376483328, 6.8848303100480734,
    2.9375398321742061 },

  /*  Computed Parameter: DirtyDerivative1_C
   * Referenced by: '<S1>/DirtyDerivative1'
   */
  { -12.606428657752776, -4.4889080376483284, -6.8848303100480726,
    2.9375398321742123 },
  200.00000000000003,                  /* Computed Parameter: DirtyDerivative1_D
                                        * Referenced by: '<S1>/DirtyDerivative1'
                                        */

  /*  Expression: x0
   * Referenced by: '<S1>/DirtyDerivative1'
   */
  { 0.0, 0.0, 0.0, 0.0 },

  /*  Computed Parameter: DirtyDerivative2_A
   * Referenced by: '<S1>/DirtyDerivative2'
   */
  { 0.26941640375703091, 0.55523437264240449, -0.40763138378494,
    0.096578338543328646, -0.55523437264240516, 0.75449863027385256,
    0.22630738856557703, -0.061558493187215735, 0.40763138378494052,
    0.22630738856557686, -0.22935555597794208, 0.49517215830633282,
    0.096578338543328618, 0.061558493187215541, -0.49517215830633321,
    -0.7945594780529408 },

  /*  Computed Parameter: DirtyDerivative2_B
   * Referenced by: '<S1>/DirtyDerivative2'
   */
  { -12.606428657752771, 4.4889080376483328, 6.8848303100480734,
    2.9375398321742061 },

  /*  Computed Parameter: DirtyDerivative2_C
   * Referenced by: '<S1>/DirtyDerivative2'
   */
  { -12.606428657752776, -4.4889080376483284, -6.8848303100480726,
    2.9375398321742123 },
  200.00000000000003,                  /* Computed Parameter: DirtyDerivative2_D
                                        * Referenced by: '<S1>/DirtyDerivative2'
                                        */

  /*  Expression: x0
   * Referenced by: '<S1>/DirtyDerivative2'
   */
  { 0.0, 0.0, 0.0, 0.0 },

  /*  Computed Parameter: DirtyDerivative3_A
   * Referenced by: '<S1>/DirtyDerivative3'
   */
  { 0.26941640375703091, 0.55523437264240449, -0.40763138378494,
    0.096578338543328646, -0.55523437264240516, 0.75449863027385256,
    0.22630738856557703, -0.061558493187215735, 0.40763138378494052,
    0.22630738856557686, -0.22935555597794208, 0.49517215830633282,
    0.096578338543328618, 0.061558493187215541, -0.49517215830633321,
    -0.7945594780529408 },

  /*  Computed Parameter: DirtyDerivative3_B
   * Referenced by: '<S1>/DirtyDerivative3'
   */
  { -12.606428657752771, 4.4889080376483328, 6.8848303100480734,
    2.9375398321742061 },

  /*  Computed Parameter: DirtyDerivative3_C
   * Referenced by: '<S1>/DirtyDerivative3'
   */
  { -12.606428657752776, -4.4889080376483284, -6.8848303100480726,
    2.9375398321742123 },
  200.00000000000003,                  /* Computed Parameter: DirtyDerivative3_D
                                        * Referenced by: '<S1>/DirtyDerivative3'
                                        */

  /*  Expression: x0
   * Referenced by: '<S1>/DirtyDerivative3'
   */
  { 0.0, 0.0, 0.0, 0.0 },

  /*  Computed Parameter: DirtyDerivative4_A
   * Referenced by: '<S1>/DirtyDerivative4'
   */
  { 0.26941640375703091, 0.55523437264240449, -0.40763138378494,
    0.096578338543328646, -0.55523437264240516, 0.75449863027385256,
    0.22630738856557703, -0.061558493187215735, 0.40763138378494052,
    0.22630738856557686, -0.22935555597794208, 0.49517215830633282,
    0.096578338543328618, 0.061558493187215541, -0.49517215830633321,
    -0.7945594780529408 },

  /*  Computed Parameter: DirtyDerivative4_B
   * Referenced by: '<S1>/DirtyDerivative4'
   */
  { -12.606428657752771, 4.4889080376483328, 6.8848303100480734,
    2.9375398321742061 },

  /*  Computed Parameter: DirtyDerivative4_C
   * Referenced by: '<S1>/DirtyDerivative4'
   */
  { -12.606428657752776, -4.4889080376483284, -6.8848303100480726,
    2.9375398321742123 },
  200.00000000000003,                  /* Computed Parameter: DirtyDerivative4_D
                                        * Referenced by: '<S1>/DirtyDerivative4'
                                        */

  /*  Expression: x0
   * Referenced by: '<S1>/DirtyDerivative4'
   */
  { 0.0, 0.0, 0.0, 0.0 },

  /*  Computed Parameter: DirtyDerivative5_A
   * Referenced by: '<S1>/DirtyDerivative5'
   */
  { 0.26941640375703091, 0.55523437264240449, -0.40763138378494,
    0.096578338543328646, -0.55523437264240516, 0.75449863027385256,
    0.22630738856557703, -0.061558493187215735, 0.40763138378494052,
    0.22630738856557686, -0.22935555597794208, 0.49517215830633282,
    0.096578338543328618, 0.061558493187215541, -0.49517215830633321,
    -0.7945594780529408 },

  /*  Computed Parameter: DirtyDerivative5_B
   * Referenced by: '<S1>/DirtyDerivative5'
   */
  { -12.606428657752771, 4.4889080376483328, 6.8848303100480734,
    2.9375398321742061 },

  /*  Computed Parameter: DirtyDerivative5_C
   * Referenced by: '<S1>/DirtyDerivative5'
   */
  { -12.606428657752776, -4.4889080376483284, -6.8848303100480726,
    2.9375398321742123 },
  200.00000000000003,                  /* Computed Parameter: DirtyDerivative5_D
                                        * Referenced by: '<S1>/DirtyDerivative5'
                                        */

  /*  Expression: x0
   * Referenced by: '<S1>/DirtyDerivative5'
   */
  { 0.0, 0.0, 0.0, 0.0 },

  /*  Computed Parameter: DirtyDerivative6_A
   * Referenced by: '<S1>/DirtyDerivative6'
   */
  { 0.26941640375703091, 0.55523437264240449, -0.40763138378494,
    0.096578338543328646, -0.55523437264240516, 0.75449863027385256,
    0.22630738856557703, -0.061558493187215735, 0.40763138378494052,
    0.22630738856557686, -0.22935555597794208, 0.49517215830633282,
    0.096578338543328618, 0.061558493187215541, -0.49517215830633321,
    -0.7945594780529408 },

  /*  Computed Parameter: DirtyDerivative6_B
   * Referenced by: '<S1>/DirtyDerivative6'
   */
  { -12.606428657752771, 4.4889080376483328, 6.8848303100480734,
    2.9375398321742061 },

  /*  Computed Parameter: DirtyDerivative6_C
   * Referenced by: '<S1>/DirtyDerivative6'
   */
  { -12.606428657752776, -4.4889080376483284, -6.8848303100480726,
    2.9375398321742123 },
  200.00000000000003,                  /* Computed Parameter: DirtyDerivative6_D
                                        * Referenced by: '<S1>/DirtyDerivative6'
                                        */

  /*  Expression: x0
   * Referenced by: '<S1>/DirtyDerivative6'
   */
  { 0.0, 0.0, 0.0, 0.0 },

  /*  Computed Parameter: DirtyDerivative7_A
   * Referenced by: '<S1>/DirtyDerivative7'
   */
  { 0.26941640375703091, 0.55523437264240449, -0.40763138378494,
    0.096578338543328646, -0.55523437264240516, 0.75449863027385256,
    0.22630738856557703, -0.061558493187215735, 0.40763138378494052,
    0.22630738856557686, -0.22935555597794208, 0.49517215830633282,
    0.096578338543328618, 0.061558493187215541, -0.49517215830633321,
    -0.7945594780529408 },

  /*  Computed Parameter: DirtyDerivative7_B
   * Referenced by: '<S1>/DirtyDerivative7'
   */
  { -12.606428657752771, 4.4889080376483328, 6.8848303100480734,
    2.9375398321742061 },

  /*  Computed Parameter: DirtyDerivative7_C
   * Referenced by: '<S1>/DirtyDerivative7'
   */
  { -12.606428657752776, -4.4889080376483284, -6.8848303100480726,
    2.9375398321742123 },
  200.00000000000003,                  /* Computed Parameter: DirtyDerivative7_D
                                        * Referenced by: '<S1>/DirtyDerivative7'
                                        */

  /*  Expression: x0
   * Referenced by: '<S1>/DirtyDerivative7'
   */
  { 0.0, 0.0, 0.0, 0.0 },

  /*  Computed Parameter: DirtyDerivative8_A
   * Referenced by: '<S1>/DirtyDerivative8'
   */
  { 0.26941640375703091, 0.55523437264240449, -0.40763138378494,
    0.096578338543328646, -0.55523437264240516, 0.75449863027385256,
    0.22630738856557703, -0.061558493187215735, 0.40763138378494052,
    0.22630738856557686, -0.22935555597794208, 0.49517215830633282,
    0.096578338543328618, 0.061558493187215541, -0.49517215830633321,
    -0.7945594780529408 },

  /*  Computed Parameter: DirtyDerivative8_B
   * Referenced by: '<S1>/DirtyDerivative8'
   */
  { -12.606428657752771, 4.4889080376483328, 6.8848303100480734,
    2.9375398321742061 },

  /*  Computed Parameter: DirtyDerivative8_C
   * Referenced by: '<S1>/DirtyDerivative8'
   */
  { -12.606428657752776, -4.4889080376483284, -6.8848303100480726,
    2.9375398321742123 },
  200.00000000000003,                  /* Computed Parameter: DirtyDerivative8_D
                                        * Referenced by: '<S1>/DirtyDerivative8'
                                        */

  /*  Expression: x0
   * Referenced by: '<S1>/DirtyDerivative8'
   */
  { 0.0, 0.0, 0.0, 0.0 },

  /*  Computed Parameter: DirtyDerivative9_A
   * Referenced by: '<S1>/DirtyDerivative9'
   */
  { 0.26941640375703091, 0.55523437264240449, -0.40763138378494,
    0.096578338543328646, -0.55523437264240516, 0.75449863027385256,
    0.22630738856557703, -0.061558493187215735, 0.40763138378494052,
    0.22630738856557686, -0.22935555597794208, 0.49517215830633282,
    0.096578338543328618, 0.061558493187215541, -0.49517215830633321,
    -0.7945594780529408 },

  /*  Computed Parameter: DirtyDerivative9_B
   * Referenced by: '<S1>/DirtyDerivative9'
   */
  { -12.606428657752771, 4.4889080376483328, 6.8848303100480734,
    2.9375398321742061 },

  /*  Computed Parameter: DirtyDerivative9_C
   * Referenced by: '<S1>/DirtyDerivative9'
   */
  { -12.606428657752776, -4.4889080376483284, -6.8848303100480726,
    2.9375398321742123 },
  200.00000000000003,                  /* Computed Parameter: DirtyDerivative9_D
                                        * Referenced by: '<S1>/DirtyDerivative9'
                                        */

  /*  Expression: x0
   * Referenced by: '<S1>/DirtyDerivative9'
   */
  { 0.0, 0.0, 0.0, 0.0 },

  /*  Computed Parameter: DirtyDerivative10_A
   * Referenced by: '<S1>/DirtyDerivative10'
   */
  { 0.26941640375703091, 0.55523437264240449, -0.40763138378494,
    0.096578338543328646, -0.55523437264240516, 0.75449863027385256,
    0.22630738856557703, -0.061558493187215735, 0.40763138378494052,
    0.22630738856557686, -0.22935555597794208, 0.49517215830633282,
    0.096578338543328618, 0.061558493187215541, -0.49517215830633321,
    -0.7945594780529408 },

  /*  Computed Parameter: DirtyDerivative10_B
   * Referenced by: '<S1>/DirtyDerivative10'
   */
  { -12.606428657752771, 4.4889080376483328, 6.8848303100480734,
    2.9375398321742061 },

  /*  Computed Parameter: DirtyDerivative10_C
   * Referenced by: '<S1>/DirtyDerivative10'
   */
  { -12.606428657752776, -4.4889080376483284, -6.8848303100480726,
    2.9375398321742123 },
  200.00000000000003,                  /* Computed Parameter: DirtyDerivative10_D
                                        * Referenced by: '<S1>/DirtyDerivative10'
                                        */

  /*  Expression: x0
   * Referenced by: '<S1>/DirtyDerivative10'
   */
  { 0.0, 0.0, 0.0, 0.0 },

  /*  Computed Parameter: DirtyDerivative11_A
   * Referenced by: '<S1>/DirtyDerivative11'
   */
  { 0.26941640375703091, 0.55523437264240449, -0.40763138378494,
    0.096578338543328646, -0.55523437264240516, 0.75449863027385256,
    0.22630738856557703, -0.061558493187215735, 0.40763138378494052,
    0.22630738856557686, -0.22935555597794208, 0.49517215830633282,
    0.096578338543328618, 0.061558493187215541, -0.49517215830633321,
    -0.7945594780529408 },

  /*  Computed Parameter: DirtyDerivative11_B
   * Referenced by: '<S1>/DirtyDerivative11'
   */
  { -12.606428657752771, 4.4889080376483328, 6.8848303100480734,
    2.9375398321742061 },

  /*  Computed Parameter: DirtyDerivative11_C
   * Referenced by: '<S1>/DirtyDerivative11'
   */
  { -12.606428657752776, -4.4889080376483284, -6.8848303100480726,
    2.9375398321742123 },
  200.00000000000003,                  /* Computed Parameter: DirtyDerivative11_D
                                        * Referenced by: '<S1>/DirtyDerivative11'
                                        */

  /*  Expression: x0
   * Referenced by: '<S1>/DirtyDerivative11'
   */
  { 0.0, 0.0, 0.0, 0.0 },

  /*  Computed Parameter: DirtyDerivative12_A
   * Referenced by: '<S1>/DirtyDerivative12'
   */
  { 0.26941640375703091, 0.55523437264240449, -0.40763138378494,
    0.096578338543328646, -0.55523437264240516, 0.75449863027385256,
    0.22630738856557703, -0.061558493187215735, 0.40763138378494052,
    0.22630738856557686, -0.22935555597794208, 0.49517215830633282,
    0.096578338543328618, 0.061558493187215541, -0.49517215830633321,
    -0.7945594780529408 },

  /*  Computed Parameter: DirtyDerivative12_B
   * Referenced by: '<S1>/DirtyDerivative12'
   */
  { -12.606428657752771, 4.4889080376483328, 6.8848303100480734,
    2.9375398321742061 },

  /*  Computed Parameter: DirtyDerivative12_C
   * Referenced by: '<S1>/DirtyDerivative12'
   */
  { -12.606428657752776, -4.4889080376483284, -6.8848303100480726,
    2.9375398321742123 },
  200.00000000000003,                  /* Computed Parameter: DirtyDerivative12_D
                                        * Referenced by: '<S1>/DirtyDerivative12'
                                        */

  /*  Expression: x0
   * Referenced by: '<S1>/DirtyDerivative12'
   */
  { 0.0, 0.0, 0.0, 0.0 },

  /*  Expression: theta_limits
   * Referenced by: '<Root>/Constant12'
   */
  { 1.3037043188119144, 1.8399159373619143 },

  /*  Expression: h_alpha
   * Referenced by: '<Root>/Constant13'
   */
  { 3.47785489105, 2.96408543145, 0.45591308829886573, 0.40360632049886597,
    3.3428846892731432, 2.8591673871481187, 0.20869753758338738,
    0.23641888418436985, 3.2994682544, 3.0267797154, 0.79404335698886586,
    0.9441703602688658, 3.1300228441, 3.3815914165, 0.014124085328865887,
    0.651160812858866, 3.0469029498985085, 3.5842157290087377,
    0.53458351355151268, 0.64958552939526681, 2.96408543145, 3.47785489105,
    0.40360632049886597, 0.45591308829886573 },

  /*  Expression: poly_cor
   * Referenced by: '<Root>/Constant14'
   */
  { 0.0, 0.0, 0.0, 0.0, -9.9920072216264089E-16, 0.0, 0.0, 0.0,
    5.9952043329758437E-15, 0.0, 0.0, 0.0, -1.1990408665951686E-14, 0.0, 0.0,
    0.0, 7.9936057773011239E-15, -0.0, -0.0, -0.0 },
  0.0,                                 /* Expression: 0
                                        * Referenced by: '<Root>/Discrete Filter'
                                        */
  0.0020000000000000018,               /* Expression: [1-alpha]
                                        * Referenced by: '<Root>/Discrete Filter'
                                        */

  /*  Expression: [1 -alpha]
   * Referenced by: '<Root>/Discrete Filter'
   */
  { 1.0, -0.998 },
  1.0,                                 /* Expression: get(gd.radiobutton6,'Value')
                                        * Referenced by: '<Root>/Constant24'
                                        */
  0.0,                                 /* Expression: get(gd.radiobutton7,'Value')
                                        * Referenced by: '<Root>/Constant25'
                                        */
  0.0,                                 /* Expression: get(gd.radiobutton8,'Value')
                                        * Referenced by: '<Root>/Constant23'
                                        */
  1.0,                                 /* Expression: get(gd.radiobutton9,'Value')
                                        * Referenced by: '<Root>/Constant27'
                                        */
  1.0,                                 /* Expression: get(gd.radiobutton9,'Value')
                                        * Referenced by: '<Root>/Constant26'
                                        */
  0.0,                                 /* Expression: 0
                                        * Referenced by: '<Root>/stance_leg'
                                        */
  0.0,                                 /* Expression: str2num(get(gd.edit105,'string'))
                                        * Referenced by: '<Root>/Constant30'
                                        */
  0.0,                                 /* Expression: str2num(get(gd.edit106,'string'))
                                        * Referenced by: '<Root>/Constant31'
                                        */
  57.295779513082323,                  /* Expression: 180/pi
                                        * Referenced by: '<Root>/rad2deg1'
                                        */
  57.295779513082323                   /* Expression: 180/pi
                                        * Referenced by: '<Root>/rad2deg2'
                                        */
};

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
