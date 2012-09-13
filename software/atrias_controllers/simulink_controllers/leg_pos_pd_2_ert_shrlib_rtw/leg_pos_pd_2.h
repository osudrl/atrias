/*
 * File: leg_pos_pd_2.h
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

#ifndef RTW_HEADER_leg_pos_pd_2_h_
#define RTW_HEADER_leg_pos_pd_2_h_
#ifndef leg_pos_pd_2_COMMON_INCLUDES_
# define leg_pos_pd_2_COMMON_INCLUDES_
#include <stddef.h>
#include <string.h>
#include "rtwtypes.h"
#include "rtw_continuous.h"
#include "rtw_solver.h"
#endif                                 /* leg_pos_pd_2_COMMON_INCLUDES_ */

#include "leg_pos_pd_2_types.h"

/* Macros for accessing real-time model data structure */
#ifndef rtmGetErrorStatus
# define rtmGetErrorStatus(rtm)        ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
# define rtmSetErrorStatus(rtm, val)   ((rtm)->errorStatus = (val))
#endif

/* Block states (auto storage) for system '<Root>' */
typedef struct {
  real_T DirtyDerivative_DSTATE[4];    /* '<Root>/DirtyDerivative' */
  real_T DirtyDerivative1_DSTATE[4];   /* '<Root>/DirtyDerivative1' */
} D_Work_leg_pos_pd_2;

/* External inputs (root inport signals with auto storage) */
typedef struct {
  real_T motorAngleA_morotAngleB[2];   /* '<Root>/motorAngleA_morotAngleB' */
  real_T desiredLegAngle_desiredKneeAngl[2];/* '<Root>/desiredLegAngle_desiredKneeAngle' */
} ExternalInputs_leg_pos_pd_2;

/* External outputs (root outports fed by signals with auto storage) */
typedef struct {
  real_T uA_uB[2];                     /* '<Root>/uA_uB' */
} ExternalOutputs_leg_pos_pd_2;

/* Parameters (auto storage) */
struct Parameters_leg_pos_pd_2_ {
  real_T Gain1_Gain;                   /* Expression: -1
                                        * Referenced by: '<Root>/Gain1'
                                        */
  real_T CoordTranformation1_Gain[4];  /* Expression: [0.5 0.5; -1 1]
                                        * Referenced by: '<Root>/CoordTranformation1'
                                        */
  real_T GainLegAngleP_Gain;           /* Expression: Kp
                                        * Referenced by: '<Root>/GainLegAngleP'
                                        */
  real_T Constant1_Value;              /* Expression: useErrorDeriv
                                        * Referenced by: '<Root>/Constant1'
                                        */
  real_T DirtyDerivative_A[16];        /* Computed Parameter: DirtyDerivative_A
                                        * Referenced by: '<Root>/DirtyDerivative'
                                        */
  real_T DirtyDerivative_B[4];         /* Computed Parameter: DirtyDerivative_B
                                        * Referenced by: '<Root>/DirtyDerivative'
                                        */
  real_T DirtyDerivative_C[4];         /* Computed Parameter: DirtyDerivative_C
                                        * Referenced by: '<Root>/DirtyDerivative'
                                        */
  real_T DirtyDerivative_D;            /* Computed Parameter: DirtyDerivative_D
                                        * Referenced by: '<Root>/DirtyDerivative'
                                        */
  real_T DirtyDerivative_X0[4];        /* Expression: x0
                                        * Referenced by: '<Root>/DirtyDerivative'
                                        */
  real_T GainLegAngleD_Gain;           /* Expression: Kd
                                        * Referenced by: '<Root>/GainLegAngleD'
                                        */
  real_T GainKneeAngleP_Gain;          /* Expression: Kp
                                        * Referenced by: '<Root>/GainKneeAngleP'
                                        */
  real_T DirtyDerivative1_A[16];       /* Computed Parameter: DirtyDerivative1_A
                                        * Referenced by: '<Root>/DirtyDerivative1'
                                        */
  real_T DirtyDerivative1_B[4];        /* Computed Parameter: DirtyDerivative1_B
                                        * Referenced by: '<Root>/DirtyDerivative1'
                                        */
  real_T DirtyDerivative1_C[4];        /* Computed Parameter: DirtyDerivative1_C
                                        * Referenced by: '<Root>/DirtyDerivative1'
                                        */
  real_T DirtyDerivative1_D;           /* Computed Parameter: DirtyDerivative1_D
                                        * Referenced by: '<Root>/DirtyDerivative1'
                                        */
  real_T DirtyDerivative1_X0[4];       /* Expression: x0
                                        * Referenced by: '<Root>/DirtyDerivative1'
                                        */
  real_T GainKneeAngleD_Gain;          /* Expression: Kd
                                        * Referenced by: '<Root>/GainKneeAngleD'
                                        */
  real_T CoordTransformation2_Gain[4]; /* Expression: [1 -0.5; 1 0.5]
                                        * Referenced by: '<Root>/CoordTransformation2'
                                        */
  real_T Saturation_UpperSat;          /* Expression: SATVAL
                                        * Referenced by: '<Root>/Saturation'
                                        */
  real_T Saturation_LowerSat;          /* Expression: -SATVAL
                                        * Referenced by: '<Root>/Saturation'
                                        */
};

/* Real-time Model Data Structure */
struct RT_MODEL_leg_pos_pd_2 {
  const char_T * volatile errorStatus;
};

/* Block parameters (auto storage) */
extern Parameters_leg_pos_pd_2 leg_pos_pd_2_P;

/* Block states (auto storage) */
extern D_Work_leg_pos_pd_2 leg_pos_pd_2_DWork;

/* External inputs (root inport signals with auto storage) */
extern ExternalInputs_leg_pos_pd_2 leg_pos_pd_2_U;

/* External outputs (root outports fed by signals with auto storage) */
extern ExternalOutputs_leg_pos_pd_2 leg_pos_pd_2_Y;

/* Model entry point functions */
extern void leg_pos_pd_2_initialize(void);
extern void leg_pos_pd_2_step(void);
extern void leg_pos_pd_2_terminate(void);

/* Real-time Model object */
extern struct RT_MODEL_leg_pos_pd_2 *const leg_pos_pd_2_M;

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
 * '<Root>' : 'leg_pos_pd_2'
 */
#endif                                 /* RTW_HEADER_leg_pos_pd_2_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
