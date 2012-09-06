/*
 * File: open_loop_sin_deriv_data.c
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

/* Block parameters (auto storage) */
Parameters_open_loop_sin_deriv open_loop_sin_deriv_P = {
  6.0,                                 /* Expression: 6
                                        * Referenced by: '<Root>/Sine Wave'
                                        */
  0.0,                                 /* Expression: 0
                                        * Referenced by: '<Root>/Sine Wave'
                                        */
  6.2831853071795862,                  /* Expression: 2*pi
                                        * Referenced by: '<Root>/Sine Wave'
                                        */
  0.0,                                 /* Expression: 0
                                        * Referenced by: '<Root>/Sine Wave'
                                        */

  /*  Computed Parameter: DirtyDerivative_A
   * Referenced by: '<Root>/DirtyDerivative'
   */
  { 0.26941640375703091, 0.55523437264240449, -0.40763138378494,
    0.096578338543328646, -0.55523437264240516, 0.75449863027385256,
    0.22630738856557703, -0.061558493187215735, 0.40763138378494052,
    0.22630738856557686, -0.22935555597794208, 0.49517215830633282,
    0.096578338543328618, 0.061558493187215541, -0.49517215830633321,
    -0.7945594780529408 },

  /*  Computed Parameter: DirtyDerivative_B
   * Referenced by: '<Root>/DirtyDerivative'
   */
  { -12.606428657752771, 4.4889080376483328, 6.8848303100480734,
    2.9375398321742061 },

  /*  Computed Parameter: DirtyDerivative_C
   * Referenced by: '<Root>/DirtyDerivative'
   */
  { -12.606428657752776, -4.4889080376483284, -6.8848303100480726,
    2.9375398321742123 },
  200.00000000000003,                  /* Computed Parameter: DirtyDerivative_D
                                        * Referenced by: '<Root>/DirtyDerivative'
                                        */

  /*  Expression: x0
   * Referenced by: '<Root>/DirtyDerivative'
   */
  { 0.0, 0.0, 0.0, 0.0 },
  0.15915494309189535                  /* Expression: 1/(2*pi)
                                        * Referenced by: '<Root>/Gain'
                                        */
};

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
