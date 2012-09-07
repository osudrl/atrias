/*
 * File: leg_position_pd_test.h
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

#ifndef RTW_HEADER_leg_position_pd_test_h_
#define RTW_HEADER_leg_position_pd_test_h_
#ifndef leg_position_pd_test_COMMON_INCLUDES_
# define leg_position_pd_test_COMMON_INCLUDES_
#include <stddef.h>
#include <string.h>
#include "rtwtypes.h"
#include "rtw_continuous.h"
#include "rtw_solver.h"
#endif                                 /* leg_position_pd_test_COMMON_INCLUDES_ */

#include "leg_position_pd_test_types.h"

/* Macros for accessing real-time model data structure */
#ifndef rtmGetErrorStatus
# define rtmGetErrorStatus(rtm)        ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
# define rtmSetErrorStatus(rtm, val)   ((rtm)->errorStatus = (val))
#endif

/* Block states (auto storage) for system '<Root>' */
typedef struct {
  real_T DirtyDerivativeLA_DSTATE[4];  /* '<Root>/DirtyDerivativeLA' */
  real_T DirtyDerivativeKA_DSTATE[4];  /* '<Root>/DirtyDerivativeKA' */
} D_Work_leg_position_pd_test;

/* External inputs (root inport signals with auto storage) */
typedef struct {
  real_T motorAnglesAB[2];             /* '<Root>/motorAnglesAB' */
  real_T desiredAnglesLA_KA[2];        /* '<Root>/desiredAnglesLA_KA' */
} ExternalInputs_leg_position_pd_;

/* External outputs (root outports fed by signals with auto storage) */
typedef struct {
  real_T uA_uB[2];                     /* '<Root>/uA_uB' */
} ExternalOutputs_leg_position_pd;

/* Parameters (auto storage) */
struct Parameters_leg_position_pd_test_ {
  real_T CoordTranformation1_Gain[4];  /* Expression: [0.5 0.5; -1 1]
                                        * Referenced by: '<Root>/CoordTranformation1'
                                        */
  real_T LegAngleP_Gain;               /* Expression: 30
                                        * Referenced by: '<Root>/LegAngleP'
                                        */
  real_T DirtyDerivativeLA_A[16];      /* Computed Parameter: DirtyDerivativeLA_A
                                        * Referenced by: '<Root>/DirtyDerivativeLA'
                                        */
  real_T DirtyDerivativeLA_B[4];       /* Computed Parameter: DirtyDerivativeLA_B
                                        * Referenced by: '<Root>/DirtyDerivativeLA'
                                        */
  real_T DirtyDerivativeLA_C[4];       /* Computed Parameter: DirtyDerivativeLA_C
                                        * Referenced by: '<Root>/DirtyDerivativeLA'
                                        */
  real_T DirtyDerivativeLA_D;          /* Computed Parameter: DirtyDerivativeLA_D
                                        * Referenced by: '<Root>/DirtyDerivativeLA'
                                        */
  real_T DirtyDerivativeLA_X0[4];      /* Expression: x0
                                        * Referenced by: '<Root>/DirtyDerivativeLA'
                                        */
  real_T LegAngleD_Gain;               /* Expression: 15
                                        * Referenced by: '<Root>/LegAngleD'
                                        */
  real_T KneeAngleP_Gain;              /* Expression: 30
                                        * Referenced by: '<Root>/KneeAngleP'
                                        */
  real_T DirtyDerivativeKA_A[16];      /* Computed Parameter: DirtyDerivativeKA_A
                                        * Referenced by: '<Root>/DirtyDerivativeKA'
                                        */
  real_T DirtyDerivativeKA_B[4];       /* Computed Parameter: DirtyDerivativeKA_B
                                        * Referenced by: '<Root>/DirtyDerivativeKA'
                                        */
  real_T DirtyDerivativeKA_C[4];       /* Computed Parameter: DirtyDerivativeKA_C
                                        * Referenced by: '<Root>/DirtyDerivativeKA'
                                        */
  real_T DirtyDerivativeKA_D;          /* Computed Parameter: DirtyDerivativeKA_D
                                        * Referenced by: '<Root>/DirtyDerivativeKA'
                                        */
  real_T DirtyDerivativeKA_X0[4];      /* Expression: x0
                                        * Referenced by: '<Root>/DirtyDerivativeKA'
                                        */
  real_T KneeAngleD_Gain;              /* Expression: 15
                                        * Referenced by: '<Root>/KneeAngleD'
                                        */
  real_T CoordTransformation2_Gain[4]; /* Expression: [1 -0.5; 1 0.5]
                                        * Referenced by: '<Root>/CoordTransformation2'
                                        */
  real_T Saturation_UpperSat;          /* Expression: 8
                                        * Referenced by: '<Root>/Saturation'
                                        */
  real_T Saturation_LowerSat;          /* Expression: -8
                                        * Referenced by: '<Root>/Saturation'
                                        */
};

/* Real-time Model Data Structure */
struct RT_MODEL_leg_position_pd_test {
  const char_T * volatile errorStatus;
};

/* Block parameters (auto storage) */
extern Parameters_leg_position_pd_test leg_position_pd_test_P;

/* Block states (auto storage) */
extern D_Work_leg_position_pd_test leg_position_pd_test_DWork;

/* External inputs (root inport signals with auto storage) */
extern ExternalInputs_leg_position_pd_ leg_position_pd_test_U;

/* External outputs (root outports fed by signals with auto storage) */
extern ExternalOutputs_leg_position_pd leg_position_pd_test_Y;

/* Model entry point functions */
extern void leg_position_pd_test_initialize(void);
extern void leg_position_pd_test_step(void);
extern void leg_position_pd_test_terminate(void);

/* Real-time Model object */
extern struct RT_MODEL_leg_position_pd_test *const leg_position_pd_test_M;

/*-
 * The generated code includes comments that allow you to trace directly
 * back to the appropriate location in the model.  The basic format
 * is <system>/block_name, where system is the system number (uniquely
 * assigned by Simulink) and block_name is the name of the block.
 *
 * Use the MATLAB hilite_system command to trace the generated code back
 * to the model.  For example,
 *
 * hilite_system('<S3>')    - opens system 3
 * hilite_system('<S3>/Kp') - opens and selects block Kp which resides in S3
 *
 * Here is the system hierarchy for this model
 *
 * '<Root>' : 'leg_position_pd_test'
 */
#endif                                 /* RTW_HEADER_leg_position_pd_test_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
