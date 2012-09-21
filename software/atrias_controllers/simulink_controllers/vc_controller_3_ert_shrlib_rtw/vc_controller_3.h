/*
 * File: vc_controller_3.h
 *
 * Code generated for Simulink model 'vc_controller_3'.
 *
 * Model version                  : 1.51
 * Simulink Coder version         : 8.2 (R2012a) 29-Dec-2011
 * TLC version                    : 8.2 (Jan 25 2012)
 * C/C++ source code generated on : Thu Sep 20 16:41:51 2012
 *
 * Target selection: ert_shrlib.tlc
 * Embedded hardware selection: 32-bit Generic
 * Emulation hardware selection:
 *    Differs from embedded hardware (MATLAB Host)
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#ifndef RTW_HEADER_vc_controller_3_h_
#define RTW_HEADER_vc_controller_3_h_
#ifndef vc_controller_3_COMMON_INCLUDES_
# define vc_controller_3_COMMON_INCLUDES_
#include <math.h>
#include <string.h>
#include "rtwtypes.h"
#include "rtw_continuous.h"
#include "rtw_solver.h"
#include "rt_nonfinite.h"
#include "rtGetInf.h"
#include "rtGetNaN.h"
#endif                                 /* vc_controller_3_COMMON_INCLUDES_ */

#include "vc_controller_3_types.h"

/* Macros for accessing real-time model data structure */
#ifndef rtmGetErrorStatus
# define rtmGetErrorStatus(rtm)        ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
# define rtmSetErrorStatus(rtm, val)   ((rtm)->errorStatus = (val))
#endif

#ifndef rtmGetT
# define rtmGetT(rtm)                  (rtmGetTPtr((rtm))[0])
#endif

/* Block signals (auto storage) */
typedef struct {
  real_T VectorConcatenate[26];        /* '<Root>/Vector Concatenate' */
  real_T DiscreteFilter;               /* '<Root>/Discrete Filter' */
  real_T stance_leg_state;             /* '<Root>/stance_leg_state' */
  real_T q_um[13];                     /* '<Root>/coordXfm' */
  real_T stance_leg_next;              /* '<Root>/controller' */
} BlockIO_vc_controller_3;

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
  real_T stance_leg_state_DSTATE;      /* '<Root>/stance_leg_state' */
  real_T DiscreteFilter_tmp;           /* '<Root>/Discrete Filter' */
  uint32_T counter_DSTATE;             /* '<Root>/counter' */
} D_Work_vc_controller_3;

/* External inputs (root inport signals with auto storage) */
typedef struct {
  real_T q_osu[14];                    /* '<Root>/q_osu' */
  real_T kp[3];                        /* '<Root>/kp' */
  real_T kd[3];                        /* '<Root>/kd' */
  real_T epsilon;                      /* '<Root>/epsilon' */
  real_T torso_offset;                 /* '<Root>/torso_offset' */
  real_T sat_val[2];                   /* '<Root>/sat_val' */
  uint8_T s_mode;                      /* '<Root>/s_mode' */
  real_T s_freq;                       /* '<Root>/s_freq' */
  boolean_T stance_leg;                /* '<Root>/stance_leg' */
  real_T q3_des[2];                    /* '<Root>/q3_des' */
  uint8_T swap;                        /* '<Root>/swap' */
  real_T swap_threshold[3];            /* '<Root>/swap_threshold' */
  real_T scuff[2];                     /* '<Root>/scuff' */
} ExternalInputs_vc_controller_3;

/* External outputs (root outports fed by signals with auto storage) */
typedef struct {
  real_T u[6];                         /* '<Root>/u' */
  real_T y[6];                         /* '<Root>/y' */
  real_T dy[6];                        /* '<Root>/dy' */
  real_T s;                            /* '<Root>/s' */
  real_T ds;                           /* '<Root>/ds' */
  uint32_T count;                      /* '<Root>/count' */
} ExternalOutputs_vc_controller_3;

/* Parameters (auto storage) */
struct Parameters_vc_controller_3_ {
  real_T SFunction_p1;                 /* Expression: prev_leg
                                        * Referenced by: '<Root>/controller'
                                        */
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
  real_T Constant12_Value[2];          /* Expression: theta_limits
                                        * Referenced by: '<Root>/Constant12'
                                        */
  real_T Constant13_Value[24];         /* Expression: h_alpha
                                        * Referenced by: '<Root>/Constant13'
                                        */
  real_T Constant14_Value[20];         /* Expression: poly_cor
                                        * Referenced by: '<Root>/Constant14'
                                        */
  real_T DiscreteFilter_InitialStates; /* Expression: 0
                                        * Referenced by: '<Root>/Discrete Filter'
                                        */
  real_T DiscreteFilter_NumCoef;       /* Expression: [1-alpha]
                                        * Referenced by: '<Root>/Discrete Filter'
                                        */
  real_T DiscreteFilter_DenCoef[2];    /* Expression: [1 -alpha]
                                        * Referenced by: '<Root>/Discrete Filter'
                                        */
  real_T Constant24_Value;             /* Expression: get(gd.radiobutton6,'Value')
                                        * Referenced by: '<Root>/Constant24'
                                        */
  real_T Constant25_Value;             /* Expression: get(gd.radiobutton7,'Value')
                                        * Referenced by: '<Root>/Constant25'
                                        */
  real_T Constant23_Value;             /* Expression: get(gd.radiobutton8,'Value')
                                        * Referenced by: '<Root>/Constant23'
                                        */
  real_T Constant27_Value;             /* Expression: get(gd.radiobutton9,'Value')
                                        * Referenced by: '<Root>/Constant27'
                                        */
  real_T Constant26_Value;             /* Expression: get(gd.radiobutton9,'Value')
                                        * Referenced by: '<Root>/Constant26'
                                        */
  real_T stance_leg_state_X0;          /* Expression: 0
                                        * Referenced by: '<Root>/stance_leg_state'
                                        */
  real_T rad2deg1_Gain;                /* Expression: 180/pi
                                        * Referenced by: '<Root>/rad2deg1'
                                        */
  real_T rad2deg2_Gain;                /* Expression: 180/pi
                                        * Referenced by: '<Root>/rad2deg2'
                                        */
  uint32_T counter_X0;                 /* Computed Parameter: counter_X0
                                        * Referenced by: '<Root>/counter'
                                        */
  uint32_T Constant_Value;             /* Computed Parameter: Constant_Value
                                        * Referenced by: '<Root>/Constant'
                                        */
};

/* Real-time Model Data Structure */
struct RT_MODEL_vc_controller_3 {
  const char_T * volatile errorStatus;
  RTWSolverInfo solverInfo;

  /*
   * Timing:
   * The following substructure contains information regarding
   * the timing information for the model.
   */
  struct {
    uint32_T clockTick0;
    time_T stepSize0;
    uint32_T clockTick1;
    SimTimeStep simTimeStep;
    time_T *t;
    time_T tArray[2];
  } Timing;
};

/* Block parameters (auto storage) */
extern Parameters_vc_controller_3 vc_controller_3_P;

/* Block signals (auto storage) */
extern BlockIO_vc_controller_3 vc_controller_3_B;

/* Block states (auto storage) */
extern D_Work_vc_controller_3 vc_controller_3_DWork;

/* External inputs (root inport signals with auto storage) */
extern ExternalInputs_vc_controller_3 vc_controller_3_U;

/* External outputs (root outports fed by signals with auto storage) */
extern ExternalOutputs_vc_controller_3 vc_controller_3_Y;

/* Model entry point functions */
extern void vc_controller_3_initialize(void);
extern void vc_controller_3_step(void);
extern void vc_controller_3_terminate(void);

/* Real-time Model object */
extern struct RT_MODEL_vc_controller_3 *const vc_controller_3_M;

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
 * '<Root>' : 'vc_controller_3'
 * '<S1>'   : 'vc_controller_3/DerivFilters'
 * '<S2>'   : 'vc_controller_3/controller'
 * '<S3>'   : 'vc_controller_3/coordXfm'
 */
#endif                                 /* RTW_HEADER_vc_controller_3_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
