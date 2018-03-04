//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: ros_image_filters_0706_private.h
//
// Code generated for Simulink model 'ros_image_filters_0706'.
//
// Model version                  : 1.446
// Simulink Coder version         : 8.12 (R2017a) 16-Feb-2017
// C/C++ source code generated on : Mon Jul 10 14:54:12 2017
//
// Target selection: ert.tlc
// Embedded hardware selection: Generic->Unspecified (assume 32-bit Generic)
// Code generation objectives: Unspecified
// Validation result: Not run
//
#ifndef RTW_HEADER_ros_image_filters_0706_private_h_
#define RTW_HEADER_ros_image_filters_0706_private_h_
#include "rtwtypes.h"
#include "ros_image_filters_0706.h"

// Private macros used by the generated code to access rtModel
#ifndef rtmIsMajorTimeStep
# define rtmIsMajorTimeStep(rtm)       (((rtm)->Timing.simTimeStep) == MAJOR_TIME_STEP)
#endif

#ifndef rtmIsMinorTimeStep
# define rtmIsMinorTimeStep(rtm)       (((rtm)->Timing.simTimeStep) == MINOR_TIME_STEP)
#endif

#ifndef rtmGetTPtr
# define rtmGetTPtr(rtm)               ((rtm)->Timing.t)
#endif

#ifndef rtmSetTPtr
# define rtmSetTPtr(rtm, val)          ((rtm)->Timing.t = (val))
#endif

extern real_T rt_roundd_snf(real_T u);
extern int32_T div_nzp_s32(int32_T numerator, int32_T denominator);
extern int32_T div_nzp_s32_floor(int32_T numerator, int32_T denominator);
extern void A1(const real_T rtu_img[518400], real_T rty_int_img[519901], B_A1
               *localB);

#endif                                 // RTW_HEADER_ros_image_filters_0706_private_h_ 

//
// File trailer for generated code.
//
// [EOF]
//
