/*
 * File: posing_controller_v2_data.c
 *
 * Code generated for Simulink model 'posing_controller_v2'.
 *
 * Model version                  : 1.28
 * Simulink Coder version         : 8.2 (R2012a) 29-Dec-2011
 * TLC version                    : 8.2 (Jan 25 2012)
 * C/C++ source code generated on : Mon Sep 17 20:00:26 2012
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

/* Block parameters (auto storage) */
Parameters_posing_controller_v2 posing_controller_v2_P = {
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
  0.0,                                 /* Expression: 0
                                        * Referenced by: '<Root>/Discrete Filter'
                                        */
  0.0020000000000000018,               /* Expression: [1-0.998]
                                        * Referenced by: '<Root>/Discrete Filter'
                                        */

  /*  Expression: [1 -0.998]
   * Referenced by: '<Root>/Discrete Filter'
   */
  { 1.0, -0.998 },
  0.0,                                 /* Expression: 0
                                        * Referenced by: '<Root>/Discrete Filter1'
                                        */
  0.0020000000000000018,               /* Expression: [1-0.998]
                                        * Referenced by: '<Root>/Discrete Filter1'
                                        */

  /*  Expression: [1 -0.998]
   * Referenced by: '<Root>/Discrete Filter1'
   */
  { 1.0, -0.998 },
  0.0,                                 /* Expression: 0
                                        * Referenced by: '<Root>/Discrete Filter2'
                                        */
  0.0020000000000000018,               /* Expression: [1-0.998]
                                        * Referenced by: '<Root>/Discrete Filter2'
                                        */

  /*  Expression: [1 -0.998]
   * Referenced by: '<Root>/Discrete Filter2'
   */
  { 1.0, -0.998 },
  0.0,                                 /* Expression: 0
                                        * Referenced by: '<Root>/Discrete Filter3'
                                        */
  0.0020000000000000018,               /* Expression: [1-0.998]
                                        * Referenced by: '<Root>/Discrete Filter3'
                                        */

  /*  Expression: [1 -0.998]
   * Referenced by: '<Root>/Discrete Filter3'
   */
  { 1.0, -0.998 },
  0.0,                                 /* Expression: 0
                                        * Referenced by: '<Root>/Discrete Filter4'
                                        */
  0.0020000000000000018,               /* Expression: [1-0.998]
                                        * Referenced by: '<Root>/Discrete Filter4'
                                        */

  /*  Expression: [1 -0.998]
   * Referenced by: '<Root>/Discrete Filter4'
   */
  { 1.0, -0.998 },
  0.0,                                 /* Expression: 0
                                        * Referenced by: '<Root>/Discrete Filter5'
                                        */
  0.0020000000000000018,               /* Expression: [1-0.998]
                                        * Referenced by: '<Root>/Discrete Filter5'
                                        */

  /*  Expression: [1 -0.998]
   * Referenced by: '<Root>/Discrete Filter5'
   */
  { 1.0, -0.998 }
};

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
