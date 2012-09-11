/*
 * File: leg_position_pd_test_data.c
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

/* Block parameters (auto storage) */
Parameters_leg_position_pd_test leg_position_pd_test_P = {
  /*  Expression: [0.5 0.5; -1 1]
   * Referenced by: '<Root>/CoordTranformation1'
   */
  { 0.5, -1.0, 0.5, 1.0 },
  30.0,                                /* Expression: 30
                                        * Referenced by: '<Root>/LegAngleP'
                                        */

  /*  Computed Parameter: DirtyDerivativeLA_A
   * Referenced by: '<Root>/DirtyDerivativeLA'
   */
  { 0.26941640375703091, 0.55523437264240449, -0.40763138378494,
    0.096578338543328646, -0.55523437264240516, 0.75449863027385256,
    0.22630738856557703, -0.061558493187215735, 0.40763138378494052,
    0.22630738856557686, -0.22935555597794208, 0.49517215830633282,
    0.096578338543328618, 0.061558493187215541, -0.49517215830633321,
    -0.7945594780529408 },

  /*  Computed Parameter: DirtyDerivativeLA_B
   * Referenced by: '<Root>/DirtyDerivativeLA'
   */
  { -12.606428657752771, 4.4889080376483328, 6.8848303100480734,
    2.9375398321742061 },

  /*  Computed Parameter: DirtyDerivativeLA_C
   * Referenced by: '<Root>/DirtyDerivativeLA'
   */
  { -12.606428657752776, -4.4889080376483284, -6.8848303100480726,
    2.9375398321742123 },
  200.00000000000003,                  /* Computed Parameter: DirtyDerivativeLA_D
                                        * Referenced by: '<Root>/DirtyDerivativeLA'
                                        */

  /*  Expression: x0
   * Referenced by: '<Root>/DirtyDerivativeLA'
   */
  { 0.0, 0.0, 0.0, 0.0 },
  15.0,                                /* Expression: 15
                                        * Referenced by: '<Root>/LegAngleD'
                                        */
  30.0,                                /* Expression: 30
                                        * Referenced by: '<Root>/KneeAngleP'
                                        */

  /*  Computed Parameter: DirtyDerivativeKA_A
   * Referenced by: '<Root>/DirtyDerivativeKA'
   */
  { 0.26941640375703091, 0.55523437264240449, -0.40763138378494,
    0.096578338543328646, -0.55523437264240516, 0.75449863027385256,
    0.22630738856557703, -0.061558493187215735, 0.40763138378494052,
    0.22630738856557686, -0.22935555597794208, 0.49517215830633282,
    0.096578338543328618, 0.061558493187215541, -0.49517215830633321,
    -0.7945594780529408 },

  /*  Computed Parameter: DirtyDerivativeKA_B
   * Referenced by: '<Root>/DirtyDerivativeKA'
   */
  { -12.606428657752771, 4.4889080376483328, 6.8848303100480734,
    2.9375398321742061 },

  /*  Computed Parameter: DirtyDerivativeKA_C
   * Referenced by: '<Root>/DirtyDerivativeKA'
   */
  { -12.606428657752776, -4.4889080376483284, -6.8848303100480726,
    2.9375398321742123 },
  200.00000000000003,                  /* Computed Parameter: DirtyDerivativeKA_D
                                        * Referenced by: '<Root>/DirtyDerivativeKA'
                                        */

  /*  Expression: x0
   * Referenced by: '<Root>/DirtyDerivativeKA'
   */
  { 0.0, 0.0, 0.0, 0.0 },
  15.0,                                /* Expression: 15
                                        * Referenced by: '<Root>/KneeAngleD'
                                        */

  /*  Expression: [1 -0.5; 1 0.5]
   * Referenced by: '<Root>/CoordTransformation2'
   */
  { 1.0, 1.0, -0.5, 0.5 },
  8.0,                                 /* Expression: 8
                                        * Referenced by: '<Root>/Saturation'
                                        */
  -8.0                                 /* Expression: -8
                                        * Referenced by: '<Root>/Saturation'
                                        */
};

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
