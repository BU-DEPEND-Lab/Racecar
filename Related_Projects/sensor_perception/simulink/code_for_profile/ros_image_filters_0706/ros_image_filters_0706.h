//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: ros_image_filters_0706.h
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
#ifndef RTW_HEADER_ros_image_filters_0706_h_
#define RTW_HEADER_ros_image_filters_0706_h_
#include <math.h>
#include <string.h>
#include <stddef.h>
#ifndef ros_image_filters_0706_COMMON_INCLUDES_
# define ros_image_filters_0706_COMMON_INCLUDES_
#include "rtwtypes.h"
#include "rtw_continuous.h"
#include "rtw_solver.h"
#include "slros_initialize.h"
#endif                                 // ros_image_filters_0706_COMMON_INCLUDES_ 

#include "ros_image_filters_0706_types.h"
#include "rtGetNaN.h"
#include "rt_nonfinite.h"
#include "rtGetInf.h"

// Macros for accessing real-time model data structure
#ifndef rtmGetErrorStatus
# define rtmGetErrorStatus(rtm)        ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
# define rtmSetErrorStatus(rtm, val)   ((rtm)->errorStatus = (val))
#endif

#ifndef rtmGetT
# define rtmGetT(rtm)                  (rtmGetTPtr((rtm))[0])
#endif

#define ros_image_filters_0706_M       (rtM)

// Block signals for system '<Root>/A1'
typedef struct {
  real_T x[518400];
} B_A1;

// Block signals (auto storage)
typedef struct {
  cell_wrap_0 b;
  cell_wrap_0 c;
  real_T int_img[519901];
  real_T int_img_m[519901];
  real_T ImageDataTypeConversion2[518400];// '<Root>/Image Data Type Conversion2' 
  real_T Erosion1[518400];             // '<Root>/Erosion1'
  real_T Erosion[518400];              // '<Root>/Erosion'
  real_T Abs[518400];                  // '<Root>/Abs'
  SL_Bus_ros_image_filters_0706_sensor_msgs_Image In1;// '<S46>/In1'
  SL_Bus_ros_image_filters_0706_sensor_msgs_Image y;// '<S28>/ROS_Img_Data_to_ROS_Img1' 
  uint8_T RGB[1555200];                // '<S10>/MATLAB Function1'
  uint8_T img[1555200];                // '<S28>/3D_img_to_1D_img'
  uint8_T ImageDataTypeConversion[518400];
  real_T S_new[1200];
  real_T dv0[961];
  real_T edges[201];
  real_T unusedU0[201];
  real_T s_k_pre[200];
  real_T b_m[200];
  real_T x_bar[6];                     // '<S13>/MATLAB Function2'
  uint8_T VectorConcatenate[1555200];  // '<S17>/Vector Concatenate'
  real_T x[6];
  SL_Bus_ros_image_filters_0706_geometry_msgs_Point BusAssignment;// '<S11>/Bus Assignment' 
  char_T cv0[17];
  int32_T iv0[4];
  real_T p_top_left[2];
  real_T s_v[2];
  real_T x_width;
  real_T weight_sum;
  real_T Clock;
  real_T filter_weights;
  real_T filter_weights_idx_0;
  real_T sumw;
  real_T b_c;
  real_T t;
  real_T c_r;
  real_T x_k;
  real_T c_u;
  real_T x_c;
  real_T y_b;
  uint32_T u[2];
  uint32_T u32[2];
  uint32_T u_p[2];
  uint32_T u_c[2];
  int32_T centerRow;
  int32_T outIdx;
  int32_T col;
  int32_T idx;
  int32_T hOffset;
  int32_T gOffset;
  int32_T iter;
  int32_T numIter;
  int32_T numEleTmp;
  int32_T lastBlockCol;
  int32_T lineOff;
  int32_T ky;
  int32_T ku;
  int32_T firstRow;
  int32_T firstCol;
  int32_T lastRow;
  int32_T lastCol;
  int32_T halfLineWidth;
  int32_T in;
  int32_T i;
  int32_T d_idxFillColor;
  int32_T h_idxPix;
  int32_T b_line_idx_3;
  int32_T hi;
  uint32_T qY;
  boolean_T bv0[4];
  boolean_T SourceBlock_o1;            // '<S16>/SourceBlock'
  B_A1 sf_A2;                          // '<Root>/A2'
  B_A1 sf_A1;                          // '<Root>/A1'
} B;

// Block states (auto storage) for system '<Root>'
typedef struct {
  real_T Erosion1_ONE_PAD_IMG_DW[538069];// '<Root>/Erosion1'
  real_T Erosion_TWO_PAD_IMG_DW[538069];// '<Root>/Erosion'
  real_T Erosion_ONE_PAD_IMG_DW[538069];// '<Root>/Erosion'
  real_T UnitDelay_DSTATE[518400];     // '<Root>/Unit Delay'
  real_T Erosion1_HBUF_DW[987];        // '<Root>/Erosion1'
  real_T Erosion1_GBUF_DW[987];        // '<Root>/Erosion1'
  real_T Erosion_HBUF_DW[987];         // '<Root>/Erosion'
  real_T Erosion_GBUF_DW[987];         // '<Root>/Erosion'
  visioncodegen_ShapeInserter h3111;   // '<S10>/MATLAB Function1'
  real_T FrameRateDisplay1_PrevTime;   // '<Root>/Frame Rate Display1'
  real_T FrameRateDisplay1_TotalTime;  // '<Root>/Frame Rate Display1'
  real_T FrameRateDisplay1_Count;      // '<Root>/Frame Rate Display1'
  real_T F_W[8];                       // '<Root>/Filter Weights'
  real_T X_P[1200];                    // '<Root>/Particle States'
  void *SourceBlock_PWORK;             // '<S16>/SourceBlock'
  void *SinkBlock_PWORK;               // '<S43>/SinkBlock'
  void *SinkBlock_PWORK_j;             // '<S30>/SinkBlock'
  int32_T Erosion_NUMNONZ_DW[2];       // '<Root>/Erosion'
  int32_T Erosion_STREL_DW[2];         // '<Root>/Erosion'
  int32_T Erosion_ERODE_OFF_DW[14];    // '<Root>/Erosion'
  int32_T Erosion1_NUMNONZ_DW[2];      // '<Root>/Erosion1'
  int32_T Erosion1_STREL_DW[2];        // '<Root>/Erosion1'
  int32_T Erosion1_ERODE_OFF_DW[14];   // '<Root>/Erosion1'
  uint32_T method;                     // '<S13>/MATLAB Function2'
  uint32_T state;                      // '<S13>/MATLAB Function2'
  uint32_T state_a[2];                 // '<S13>/MATLAB Function2'
  uint32_T state_i[625];               // '<S13>/MATLAB Function2'
  uint32_T method_c;                   // '<S13>/MATLAB Function2'
  uint32_T state_p[2];                 // '<S13>/MATLAB Function2'
  uint32_T method_k;                   // '<S12>/MATLAB Function2'
  uint32_T state_l;                    // '<S12>/MATLAB Function2'
  uint32_T state_j[2];                 // '<S12>/MATLAB Function2'
  uint32_T state_b[625];               // '<S12>/MATLAB Function2'
  robotics_slros_internal_block_P obj; // '<S43>/SinkBlock'
  robotics_slros_internal_block_P obj_i;// '<S30>/SinkBlock'
  robotics_slros_internal_block_S obj_j;// '<S16>/SourceBlock'
  uint8_T ColorSpaceConversion_DWORK1[1555200];// '<Root>/Color Space  Conversion' 
  boolean_T state_not_empty;           // '<S13>/MATLAB Function2'
  boolean_T method_not_empty;          // '<S13>/MATLAB Function2'
  boolean_T state_not_empty_o;         // '<S12>/MATLAB Function2'
  boolean_T h3111_not_empty;           // '<S10>/MATLAB Function1'
  boolean_T ParticleUpdate_MODE;       // '<Root>/Particle Update'
  boolean_T ParticleInitialization_MODE;// '<Root>/Particle Initialization'
} DW;

// Parameters (auto storage)
struct P_ {
  SL_Bus_ros_image_filters_0706_sensor_msgs_Image Out1_Y0;// Computed Parameter: Out1_Y0
                                                          //  Referenced by: '<S46>/Out1'

  SL_Bus_ros_image_filters_0706_sensor_msgs_Image Constant_Value;// Computed Parameter: Constant_Value
                                                                 //  Referenced by: '<S16>/Constant'

  SL_Bus_ros_image_filters_0706_sensor_msgs_Image Constant_Value_b;// Computed Parameter: Constant_Value_b
                                                                   //  Referenced by: '<S40>/Constant'

  SL_Bus_ros_image_filters_0706_geometry_msgs_Point Constant_Value_e;// Computed Parameter: Constant_Value_e
                                                                     //  Referenced by: '<S42>/Constant'

  real_T Result_Y0;                    // Computed Parameter: Result_Y0
                                       //  Referenced by: '<S13>/Result'

  real_T UnitDelay_InitialCondition;   // Expression: 0
                                       //  Referenced by: '<Root>/Unit Delay'

  real_T Constant_Value_ek;            // Expression: 0
                                       //  Referenced by: '<S7>/Constant'

  real_T Constant_Value_g;             // Expression: 0
                                       //  Referenced by: '<S6>/Constant'

  real_T FilterWeights_InitialValue[8];// Expression: [0,0,0,0,0,0,0,0]
                                       //  Referenced by: '<Root>/Filter Weights'

  real_T ParticleStates_InitialValue[1200];// Expression: zeros(200,3,2)
                                           //  Referenced by: '<Root>/Particle States'

};

// Real-time Model Data Structure
struct tag_RTM {
  const char_T *errorStatus;
  RTWSolverInfo solverInfo;

  //
  //  Timing:
  //  The following substructure contains information regarding
  //  the timing information for the model.

  struct {
    uint32_T clockTick0;
    time_T stepSize0;
    uint32_T clockTick1;
    SimTimeStep simTimeStep;
    time_T *t;
    time_T tArray[2];
  } Timing;
};

// Block parameters (auto storage)
#ifdef __cplusplus

extern "C" {

#endif

  extern P rtP;

#ifdef __cplusplus

}
#endif

// Block signals (auto storage)
extern B rtB;

// Block states (auto storage)
extern DW rtDW;

#ifdef __cplusplus

extern "C" {

#endif

#ifdef __cplusplus

}
#endif

#ifdef __cplusplus

extern "C" {

#endif

  // Model entry point functions
  extern void ros_image_filters_0706_initialize(void);
  extern void ros_image_filters_0706_step(void);
  extern void ros_image_filters_0706_terminate(void);

#ifdef __cplusplus

}
#endif

// Real-time Model object
#ifdef __cplusplus

extern "C" {

#endif

  extern RT_MODEL *const rtM;

#ifdef __cplusplus

}
#endif

//-
//  The generated code includes comments that allow you to trace directly
//  back to the appropriate location in the model.  The basic format
//  is <system>/block_name, where system is the system number (uniquely
//  assigned by Simulink) and block_name is the name of the block.
//
//  Use the MATLAB hilite_system command to trace the generated code back
//  to the model.  For example,
//
//  hilite_system('<S3>')    - opens system 3
//  hilite_system('<S3>/Kp') - opens and selects block Kp which resides in S3
//
//  Here is the system hierarchy for this model
//
//  '<Root>' : 'ros_image_filters_0706'
//  '<S1>'   : 'ros_image_filters_0706/2-d-image2ros'
//  '<S2>'   : 'ros_image_filters_0706/3-d-image2ros'
//  '<S3>'   : 'ros_image_filters_0706/A1'
//  '<S4>'   : 'ros_image_filters_0706/A2'
//  '<S5>'   : 'ros_image_filters_0706/Color Classifier'
//  '<S6>'   : 'ros_image_filters_0706/Compare To Zero'
//  '<S7>'   : 'ros_image_filters_0706/Compare To Zero1'
//  '<S8>'   : 'ros_image_filters_0706/MATLAB Function'
//  '<S9>'   : 'ros_image_filters_0706/MATLAB Function1'
//  '<S10>'  : 'ros_image_filters_0706/Monitor'
//  '<S11>'  : 'ros_image_filters_0706/Parameter Publisher'
//  '<S12>'  : 'ros_image_filters_0706/Particle Initialization'
//  '<S13>'  : 'ros_image_filters_0706/Particle Update'
//  '<S14>'  : 'ros_image_filters_0706/Publish'
//  '<S15>'  : 'ros_image_filters_0706/Publish1'
//  '<S16>'  : 'ros_image_filters_0706/Subscribe'
//  '<S17>'  : 'ros_image_filters_0706/ros2image'
//  '<S18>'  : 'ros_image_filters_0706/2-d-image2ros/2D_img_to_1D_img1'
//  '<S19>'  : 'ros_image_filters_0706/2-d-image2ros/Blank Message'
//  '<S20>'  : 'ros_image_filters_0706/2-d-image2ros/ROS_Img_Data_to_ROS_Img1'
//  '<S21>'  : 'ros_image_filters_0706/3-d-image2ros/3D_img_to_1D_img'
//  '<S22>'  : 'ros_image_filters_0706/3-d-image2ros/Blank Message'
//  '<S23>'  : 'ros_image_filters_0706/3-d-image2ros/ROS_Img_Data_to_ROS_Img1'
//  '<S24>'  : 'ros_image_filters_0706/Color Classifier/Compare To Constant1'
//  '<S25>'  : 'ros_image_filters_0706/Color Classifier/Mahalanobis  Distance '
//  '<S26>'  : 'ros_image_filters_0706/Monitor/2-d-image2ros'
//  '<S27>'  : 'ros_image_filters_0706/Monitor/2-d-image2ros1'
//  '<S28>'  : 'ros_image_filters_0706/Monitor/3-d-image2ros'
//  '<S29>'  : 'ros_image_filters_0706/Monitor/MATLAB Function1'
//  '<S30>'  : 'ros_image_filters_0706/Monitor/Publish'
//  '<S31>'  : 'ros_image_filters_0706/Monitor/Publish1'
//  '<S32>'  : 'ros_image_filters_0706/Monitor/Publish2'
//  '<S33>'  : 'ros_image_filters_0706/Monitor/2-d-image2ros/2D_img_to_1D_img1'
//  '<S34>'  : 'ros_image_filters_0706/Monitor/2-d-image2ros/Blank Message'
//  '<S35>'  : 'ros_image_filters_0706/Monitor/2-d-image2ros/ROS_Img_Data_to_ROS_Img1'
//  '<S36>'  : 'ros_image_filters_0706/Monitor/2-d-image2ros1/2D_img_to_1D_img1'
//  '<S37>'  : 'ros_image_filters_0706/Monitor/2-d-image2ros1/Blank Message'
//  '<S38>'  : 'ros_image_filters_0706/Monitor/2-d-image2ros1/ROS_Img_Data_to_ROS_Img1'
//  '<S39>'  : 'ros_image_filters_0706/Monitor/3-d-image2ros/3D_img_to_1D_img'
//  '<S40>'  : 'ros_image_filters_0706/Monitor/3-d-image2ros/Blank Message'
//  '<S41>'  : 'ros_image_filters_0706/Monitor/3-d-image2ros/ROS_Img_Data_to_ROS_Img1'
//  '<S42>'  : 'ros_image_filters_0706/Parameter Publisher/Blank Message'
//  '<S43>'  : 'ros_image_filters_0706/Parameter Publisher/Publish2'
//  '<S44>'  : 'ros_image_filters_0706/Particle Initialization/MATLAB Function2'
//  '<S45>'  : 'ros_image_filters_0706/Particle Update/MATLAB Function2'
//  '<S46>'  : 'ros_image_filters_0706/Subscribe/Enabled Subsystem'
//  '<S47>'  : 'ros_image_filters_0706/ros2image/ROSraw2RGB'

#endif                                 // RTW_HEADER_ros_image_filters_0706_h_

//
// File trailer for generated code.
//
// [EOF]
//
