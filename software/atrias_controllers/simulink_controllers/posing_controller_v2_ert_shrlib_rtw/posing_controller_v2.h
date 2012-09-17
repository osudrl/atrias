/*
 * File: posing_controller_v2.h
 *
 * Code generated for Simulink model 'posing_controller_v2'.
 *
 * Model version                  : 1.27
 * Simulink Coder version         : 8.2 (R2012a) 29-Dec-2011
 * TLC version                    : 8.2 (Jan 25 2012)
 * C/C++ source code generated on : Mon Sep 17 19:14:31 2012
 *
 * Target selection: ert_shrlib.tlc
 * Embedded hardware selection: 32-bit Generic
 * Emulation hardware selection:
 *    Differs from embedded hardware (MATLAB Host)
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#ifndef RTW_HEADER_posing_controller_v2_h_
#define RTW_HEADER_posing_controller_v2_h_
#ifndef posing_controller_v2_COMMON_INCLUDES_
# define posing_controller_v2_COMMON_INCLUDES_
#include <math.h>
#include <stddef.h>
#include <string.h>
#include "rtwtypes.h"
#include "rtw_continuous.h"
#include "rtw_solver.h"
#include "rt_nonfinite.h"
#include "rtGetInf.h"
#include "rtGetNaN.h"
#endif                                 /* posing_controller_v2_COMMON_INCLUDES_ */

#include "posing_controller_v2_types.h"

/* Macros for accessing real-time model data structure */
#ifndef rtmGetErrorStatus
# define rtmGetErrorStatus(rtm)        ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
# define rtmSetErrorStatus(rtm, val)   ((rtm)->errorStatus = (val))
#endif

/* Block states (auto storage) for system '<Root>' */
typedef struct {
  real_T DirtyDerivative_DSTATE[4];    /* '<S1>/DirtyDerivative' */
  real_T DirtyDerivative1_DSTATE[4];   /* '<S1>/DirtyDerivative1' */
  real_T DirtyDerivative2_DSTATE[4];   /* '<S1>/DirtyDerivative2' */
  real_T DirtyDerivative3_DSTATE[4];   /* '<S1>/DirtyDerivative3' */
  real_T DirtyDerivative4_DSTATE[4];   /* '<S1>/DirtyDerivative4' */
  real_T DirtyDerivative5_DSTATE[4];   /* '<S1>/DirtyDerivative5' */
  real_T DirtyDerivative6_DSTATE[4];   /* '<S1>/DirtyDerivative6' */
  real_T DirtyDerivative7_DSTATE[4];   /* '<S1>/DirtyDerivative7' */
  real_T DirtyDerivative8_DSTATE[4];   /* '<S1>/DirtyDerivative8' */
  real_T DirtyDerivative9_DSTATE[4];   /* '<S1>/DirtyDerivative9' */
  real_T DirtyDerivative10_DSTATE[4];  /* '<S1>/DirtyDerivative10' */
  real_T DirtyDerivative11_DSTATE[4];  /* '<S1>/DirtyDerivative11' */
  real_T DirtyDerivative12_DSTATE[4];  /* '<S1>/DirtyDerivative12' */
  real_T DiscreteFilter_DSTATE;        /* '<Root>/Discrete Filter' */
  real_T DiscreteFilter1_DSTATE;       /* '<Root>/Discrete Filter1' */
  real_T DiscreteFilter2_DSTATE;       /* '<Root>/Discrete Filter2' */
  real_T DiscreteFilter3_DSTATE;       /* '<Root>/Discrete Filter3' */
  real_T DiscreteFilter4_DSTATE;       /* '<Root>/Discrete Filter4' */
  real_T DiscreteFilter5_DSTATE;       /* '<Root>/Discrete Filter5' */
} D_Work_posing_controller_v2;

/* External inputs (root inport signals with auto storage) */
typedef struct {
  real_T q_osu[14];                    /* '<Root>/q_osu' */
  real_T kp[3];                        /* '<Root>/kp' */
  real_T kd[3];                        /* '<Root>/kd' */
  real_T epsilon;                      /* '<Root>/epsilon' */
  real_T set_point[6];                 /* '<Root>/set_point' */
} ExternalInputs_posing_controlle;

/* External outputs (root outports fed by signals with auto storage) */
typedef struct {
  real_T u[6];                         /* '<Root>/u' */
  real_T y[6];                         /* '<Root>/y' */
  real_T dy[6];                        /* '<Root>/dy' */
} ExternalOutputs_posing_controll;

/* Parameters (auto storage) */
struct Parameters_posing_controller_v2_ {
  real_T DirtyDerivative_A[16];        /* Computed Parameter: DirtyDerivative_A
                                        * Referenced by: '<S1>/DirtyDerivative'
                                        */
  real_T DirtyDerivative_B[4];         /* Computed Parameter: DirtyDerivative_B
                                        * Referenced by: '<S1>/DirtyDerivative'
                                        */
  real_T DirtyDerivative_C[4];         /* Computed Parameter: DirtyDerivative_C
                                        * Referenced by: '<S1>/DirtyDerivative'
                                        */
  real_T DirtyDerivative_D;            /* Computed Parameter: DirtyDerivative_D
                                        * Referenced by: '<S1>/DirtyDerivative'
                                        */
  real_T DirtyDerivative_X0[4];        /* Expression: x0
                                        * Referenced by: '<S1>/DirtyDerivative'
                                        */
  real_T DirtyDerivative1_A[16];       /* Computed Parameter: DirtyDerivative1_A
                                        * Referenced by: '<S1>/DirtyDerivative1'
                                        */
  real_T DirtyDerivative1_B[4];        /* Computed Parameter: DirtyDerivative1_B
                                        * Referenced by: '<S1>/DirtyDerivative1'
                                        */
  real_T DirtyDerivative1_C[4];        /* Computed Parameter: DirtyDerivative1_C
                                        * Referenced by: '<S1>/DirtyDerivative1'
                                        */
  real_T DirtyDerivative1_D;           /* Computed Parameter: DirtyDerivative1_D
                                        * Referenced by: '<S1>/DirtyDerivative1'
                                        */
  real_T DirtyDerivative1_X0[4];       /* Expression: x0
                                        * Referenced by: '<S1>/DirtyDerivative1'
                                        */
  real_T DirtyDerivative2_A[16];       /* Computed Parameter: DirtyDerivative2_A
                                        * Referenced by: '<S1>/DirtyDerivative2'
                                        */
  real_T DirtyDerivative2_B[4];        /* Computed Parameter: DirtyDerivative2_B
                                        * Referenced by: '<S1>/DirtyDerivative2'
                                        */
  real_T DirtyDerivative2_C[4];        /* Computed Parameter: DirtyDerivative2_C
                                        * Referenced by: '<S1>/DirtyDerivative2'
                                        */
  real_T DirtyDerivative2_D;           /* Computed Parameter: DirtyDerivative2_D
                                        * Referenced by: '<S1>/DirtyDerivative2'
                                        */
  real_T DirtyDerivative2_X0[4];       /* Expression: x0
                                        * Referenced by: '<S1>/DirtyDerivative2'
                                        */
  real_T DirtyDerivative3_A[16];       /* Computed Parameter: DirtyDerivative3_A
                                        * Referenced by: '<S1>/DirtyDerivative3'
                                        */
  real_T DirtyDerivative3_B[4];        /* Computed Parameter: DirtyDerivative3_B
                                        * Referenced by: '<S1>/DirtyDerivative3'
                                        */
  real_T DirtyDerivative3_C[4];        /* Computed Parameter: DirtyDerivative3_C
                                        * Referenced by: '<S1>/DirtyDerivative3'
                                        */
  real_T DirtyDerivative3_D;           /* Computed Parameter: DirtyDerivative3_D
                                        * Referenced by: '<S1>/DirtyDerivative3'
                                        */
  real_T DirtyDerivative3_X0[4];       /* Expression: x0
                                        * Referenced by: '<S1>/DirtyDerivative3'
                                        */
  real_T DirtyDerivative4_A[16];       /* Computed Parameter: DirtyDerivative4_A
                                        * Referenced by: '<S1>/DirtyDerivative4'
                                        */
  real_T DirtyDerivative4_B[4];        /* Computed Parameter: DirtyDerivative4_B
                                        * Referenced by: '<S1>/DirtyDerivative4'
                                        */
  real_T DirtyDerivative4_C[4];        /* Computed Parameter: DirtyDerivative4_C
                                        * Referenced by: '<S1>/DirtyDerivative4'
                                        */
  real_T DirtyDerivative4_D;           /* Computed Parameter: DirtyDerivative4_D
                                        * Referenced by: '<S1>/DirtyDerivative4'
                                        */
  real_T DirtyDerivative4_X0[4];       /* Expression: x0
                                        * Referenced by: '<S1>/DirtyDerivative4'
                                        */
  real_T DirtyDerivative5_A[16];       /* Computed Parameter: DirtyDerivative5_A
                                        * Referenced by: '<S1>/DirtyDerivative5'
                                        */
  real_T DirtyDerivative5_B[4];        /* Computed Parameter: DirtyDerivative5_B
                                        * Referenced by: '<S1>/DirtyDerivative5'
                                        */
  real_T DirtyDerivative5_C[4];        /* Computed Parameter: DirtyDerivative5_C
                                        * Referenced by: '<S1>/DirtyDerivative5'
                                        */
  real_T DirtyDerivative5_D;           /* Computed Parameter: DirtyDerivative5_D
                                        * Referenced by: '<S1>/DirtyDerivative5'
                                        */
  real_T DirtyDerivative5_X0[4];       /* Expression: x0
                                        * Referenced by: '<S1>/DirtyDerivative5'
                                        */
  real_T DirtyDerivative6_A[16];       /* Computed Parameter: DirtyDerivative6_A
                                        * Referenced by: '<S1>/DirtyDerivative6'
                                        */
  real_T DirtyDerivative6_B[4];        /* Computed Parameter: DirtyDerivative6_B
                                        * Referenced by: '<S1>/DirtyDerivative6'
                                        */
  real_T DirtyDerivative6_C[4];        /* Computed Parameter: DirtyDerivative6_C
                                        * Referenced by: '<S1>/DirtyDerivative6'
                                        */
  real_T DirtyDerivative6_D;           /* Computed Parameter: DirtyDerivative6_D
                                        * Referenced by: '<S1>/DirtyDerivative6'
                                        */
  real_T DirtyDerivative6_X0[4];       /* Expression: x0
                                        * Referenced by: '<S1>/DirtyDerivative6'
                                        */
  real_T DirtyDerivative7_A[16];       /* Computed Parameter: DirtyDerivative7_A
                                        * Referenced by: '<S1>/DirtyDerivative7'
                                        */
  real_T DirtyDerivative7_B[4];        /* Computed Parameter: DirtyDerivative7_B
                                        * Referenced by: '<S1>/DirtyDerivative7'
                                        */
  real_T DirtyDerivative7_C[4];        /* Computed Parameter: DirtyDerivative7_C
                                        * Referenced by: '<S1>/DirtyDerivative7'
                                        */
  real_T DirtyDerivative7_D;           /* Computed Parameter: DirtyDerivative7_D
                                        * Referenced by: '<S1>/DirtyDerivative7'
                                        */
  real_T DirtyDerivative7_X0[4];       /* Expression: x0
                                        * Referenced by: '<S1>/DirtyDerivative7'
                                        */
  real_T DirtyDerivative8_A[16];       /* Computed Parameter: DirtyDerivative8_A
                                        * Referenced by: '<S1>/DirtyDerivative8'
                                        */
  real_T DirtyDerivative8_B[4];        /* Computed Parameter: DirtyDerivative8_B
                                        * Referenced by: '<S1>/DirtyDerivative8'
                                        */
  real_T DirtyDerivative8_C[4];        /* Computed Parameter: DirtyDerivative8_C
                                        * Referenced by: '<S1>/DirtyDerivative8'
                                        */
  real_T DirtyDerivative8_D;           /* Computed Parameter: DirtyDerivative8_D
                                        * Referenced by: '<S1>/DirtyDerivative8'
                                        */
  real_T DirtyDerivative8_X0[4];       /* Expression: x0
                                        * Referenced by: '<S1>/DirtyDerivative8'
                                        */
  real_T DirtyDerivative9_A[16];       /* Computed Parameter: DirtyDerivative9_A
                                        * Referenced by: '<S1>/DirtyDerivative9'
                                        */
  real_T DirtyDerivative9_B[4];        /* Computed Parameter: DirtyDerivative9_B
                                        * Referenced by: '<S1>/DirtyDerivative9'
                                        */
  real_T DirtyDerivative9_C[4];        /* Computed Parameter: DirtyDerivative9_C
                                        * Referenced by: '<S1>/DirtyDerivative9'
                                        */
  real_T DirtyDerivative9_D;           /* Computed Parameter: DirtyDerivative9_D
                                        * Referenced by: '<S1>/DirtyDerivative9'
                                        */
  real_T DirtyDerivative9_X0[4];       /* Expression: x0
                                        * Referenced by: '<S1>/DirtyDerivative9'
                                        */
  real_T DirtyDerivative10_A[16];      /* Computed Parameter: DirtyDerivative10_A
                                        * Referenced by: '<S1>/DirtyDerivative10'
                                        */
  real_T DirtyDerivative10_B[4];       /* Computed Parameter: DirtyDerivative10_B
                                        * Referenced by: '<S1>/DirtyDerivative10'
                                        */
  real_T DirtyDerivative10_C[4];       /* Computed Parameter: DirtyDerivative10_C
                                        * Referenced by: '<S1>/DirtyDerivative10'
                                        */
  real_T DirtyDerivative10_D;          /* Computed Parameter: DirtyDerivative10_D
                                        * Referenced by: '<S1>/DirtyDerivative10'
                                        */
  real_T DirtyDerivative10_X0[4];      /* Expression: x0
                                        * Referenced by: '<S1>/DirtyDerivative10'
                                        */
  real_T DirtyDerivative11_A[16];      /* Computed Parameter: DirtyDerivative11_A
                                        * Referenced by: '<S1>/DirtyDerivative11'
                                        */
  real_T DirtyDerivative11_B[4];       /* Computed Parameter: DirtyDerivative11_B
                                        * Referenced by: '<S1>/DirtyDerivative11'
                                        */
  real_T DirtyDerivative11_C[4];       /* Computed Parameter: DirtyDerivative11_C
                                        * Referenced by: '<S1>/DirtyDerivative11'
                                        */
  real_T DirtyDerivative11_D;          /* Computed Parameter: DirtyDerivative11_D
                                        * Referenced by: '<S1>/DirtyDerivative11'
                                        */
  real_T DirtyDerivative11_X0[4];      /* Expression: x0
                                        * Referenced by: '<S1>/DirtyDerivative11'
                                        */
  real_T DirtyDerivative12_A[16];      /* Computed Parameter: DirtyDerivative12_A
                                        * Referenced by: '<S1>/DirtyDerivative12'
                                        */
  real_T DirtyDerivative12_B[4];       /* Computed Parameter: DirtyDerivative12_B
                                        * Referenced by: '<S1>/DirtyDerivative12'
                                        */
  real_T DirtyDerivative12_C[4];       /* Computed Parameter: DirtyDerivative12_C
                                        * Referenced by: '<S1>/DirtyDerivative12'
                                        */
  real_T DirtyDerivative12_D;          /* Computed Parameter: DirtyDerivative12_D
                                        * Referenced by: '<S1>/DirtyDerivative12'
                                        */
  real_T DirtyDerivative12_X0[4];      /* Expression: x0
                                        * Referenced by: '<S1>/DirtyDerivative12'
                                        */
  real_T DiscreteFilter_InitialStates; /* Expression: 0
                                        * Referenced by: '<Root>/Discrete Filter'
                                        */
  real_T DiscreteFilter_NumCoef;       /* Expression: [1-0.998]
                                        * Referenced by: '<Root>/Discrete Filter'
                                        */
  real_T DiscreteFilter_DenCoef[2];    /* Expression: [1 -0.998]
                                        * Referenced by: '<Root>/Discrete Filter'
                                        */
  real_T DiscreteFilter1_InitialStates;/* Expression: 0
                                        * Referenced by: '<Root>/Discrete Filter1'
                                        */
  real_T DiscreteFilter1_NumCoef;      /* Expression: [1-0.998]
                                        * Referenced by: '<Root>/Discrete Filter1'
                                        */
  real_T DiscreteFilter1_DenCoef[2];   /* Expression: [1 -0.998]
                                        * Referenced by: '<Root>/Discrete Filter1'
                                        */
  real_T DiscreteFilter2_InitialStates;/* Expression: 0
                                        * Referenced by: '<Root>/Discrete Filter2'
                                        */
  real_T DiscreteFilter2_NumCoef;      /* Expression: [1-0.998]
                                        * Referenced by: '<Root>/Discrete Filter2'
                                        */
  real_T DiscreteFilter2_DenCoef[2];   /* Expression: [1 -0.998]
                                        * Referenced by: '<Root>/Discrete Filter2'
                                        */
  real_T DiscreteFilter3_InitialStates;/* Expression: 0
                                        * Referenced by: '<Root>/Discrete Filter3'
                                        */
  real_T DiscreteFilter3_NumCoef;      /* Expression: [1-0.998]
                                        * Referenced by: '<Root>/Discrete Filter3'
                                        */
  real_T DiscreteFilter3_DenCoef[2];   /* Expression: [1 -0.998]
                                        * Referenced by: '<Root>/Discrete Filter3'
                                        */
  real_T DiscreteFilter4_InitialStates;/* Expression: 0
                                        * Referenced by: '<Root>/Discrete Filter4'
                                        */
  real_T DiscreteFilter4_NumCoef;      /* Expression: [1-0.998]
                                        * Referenced by: '<Root>/Discrete Filter4'
                                        */
  real_T DiscreteFilter4_DenCoef[2];   /* Expression: [1 -0.998]
                                        * Referenced by: '<Root>/Discrete Filter4'
                                        */
  real_T DiscreteFilter5_InitialStates;/* Expression: 0
                                        * Referenced by: '<Root>/Discrete Filter5'
                                        */
  real_T DiscreteFilter5_NumCoef;      /* Expression: [1-0.998]
                                        * Referenced by: '<Root>/Discrete Filter5'
                                        */
  real_T DiscreteFilter5_DenCoef[2];   /* Expression: [1 -0.998]
                                        * Referenced by: '<Root>/Discrete Filter5'
                                        */
};

/* Real-time Model Data Structure */
struct RT_MODEL_posing_controller_v2 {
  const char_T * volatile errorStatus;
};

/* Block parameters (auto storage) */
extern Parameters_posing_controller_v2 posing_controller_v2_P;

/* Block states (auto storage) */
extern D_Work_posing_controller_v2 posing_controller_v2_DWork;

/* External inputs (root inport signals with auto storage) */
extern ExternalInputs_posing_controlle posing_controller_v2_U;

/* External outputs (root outports fed by signals with auto storage) */
extern ExternalOutputs_posing_controll posing_controller_v2_Y;

/* Model entry point functions */
extern void posing_controller_v2_initialize(void);
extern void posing_controller_v2_step(void);
extern void posing_controller_v2_terminate(void);

/* Real-time Model object */
extern struct RT_MODEL_posing_controller_v2 *const posing_controller_v2_M;

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
 * '<Root>' : 'posing_controller_v2'
 * '<S1>'   : 'posing_controller_v2/DerivFilters'
 * '<S2>'   : 'posing_controller_v2/controller'
 * '<S3>'   : 'posing_controller_v2/coordXfm'
 */
#endif                                 /* RTW_HEADER_posing_controller_v2_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
