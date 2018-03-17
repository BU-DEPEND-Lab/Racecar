//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: ros_image_filters_0706_types.h
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
#ifndef RTW_HEADER_ros_image_filters_0706_types_h_
#define RTW_HEADER_ros_image_filters_0706_types_h_
#include "rtwtypes.h"
#ifndef DEFINED_TYPEDEF_FOR_SL_Bus_ROSVariableLengthArrayInfo_
#define DEFINED_TYPEDEF_FOR_SL_Bus_ROSVariableLengthArrayInfo_

typedef struct {
  uint32_T CurrentLength;
  uint32_T ReceivedLength;
} SL_Bus_ROSVariableLengthArrayInfo;

#endif

#ifndef DEFINED_TYPEDEF_FOR_SL_Bus_ros_image_filters_0706_ros_time_Time_
#define DEFINED_TYPEDEF_FOR_SL_Bus_ros_image_filters_0706_ros_time_Time_

// MsgType=ros_time/Time
typedef struct {
  real_T Sec;
  real_T Nsec;
} SL_Bus_ros_image_filters_0706_ros_time_Time;

#endif

#ifndef DEFINED_TYPEDEF_FOR_SL_Bus_ros_image_filters_0706_std_msgs_Header_
#define DEFINED_TYPEDEF_FOR_SL_Bus_ros_image_filters_0706_std_msgs_Header_

// MsgType=std_msgs/Header
typedef struct {
  uint32_T Seq;

  // PrimitiveROSType=string:IsVarLen=1:VarLenCategory=data:VarLenElem=FrameId_SL_Info:TruncateAction=warn 
  uint8_T FrameId[128];

  // IsVarLen=1:VarLenCategory=length:VarLenElem=FrameId
  SL_Bus_ROSVariableLengthArrayInfo FrameId_SL_Info;

  // MsgType=ros_time/Time
  SL_Bus_ros_image_filters_0706_ros_time_Time Stamp;
} SL_Bus_ros_image_filters_0706_std_msgs_Header;

#endif

#ifndef DEFINED_TYPEDEF_FOR_SL_Bus_ros_image_filters_0706_sensor_msgs_Image_
#define DEFINED_TYPEDEF_FOR_SL_Bus_ros_image_filters_0706_sensor_msgs_Image_

// MsgType=sensor_msgs/Image
typedef struct {
  uint32_T Height;
  uint32_T Width;

  // PrimitiveROSType=string:IsVarLen=1:VarLenCategory=data:VarLenElem=Encoding_SL_Info:TruncateAction=warn 
  uint8_T Encoding[4];

  // IsVarLen=1:VarLenCategory=length:VarLenElem=Encoding
  SL_Bus_ROSVariableLengthArrayInfo Encoding_SL_Info;
  uint8_T IsBigendian;
  uint32_T Step;

  // IsVarLen=1:VarLenCategory=data:VarLenElem=Data_SL_Info:TruncateAction=warn
  uint8_T Data[1555200];

  // IsVarLen=1:VarLenCategory=length:VarLenElem=Data
  SL_Bus_ROSVariableLengthArrayInfo Data_SL_Info;

  // MsgType=std_msgs/Header
  SL_Bus_ros_image_filters_0706_std_msgs_Header Header;
} SL_Bus_ros_image_filters_0706_sensor_msgs_Image;

#endif

#ifndef DEFINED_TYPEDEF_FOR_SL_Bus_ros_image_filters_0706_geometry_msgs_Point_
#define DEFINED_TYPEDEF_FOR_SL_Bus_ros_image_filters_0706_geometry_msgs_Point_

// MsgType=geometry_msgs/Point
typedef struct {
  real_T X;
  real_T Y;
  real_T Z;
} SL_Bus_ros_image_filters_0706_geometry_msgs_Point;

#endif

// Custom Type definition for MATLAB Function: '<Root>/A1'
#ifndef typedef_struct_T
#define typedef_struct_T

typedef struct {
  char_T f1[7];
  char_T f2[7];
} struct_T;

#endif                                 //typedef_struct_T

#ifndef typedef_b_struct_T
#define typedef_b_struct_T

typedef struct {
  char_T f1[9];
  char_T f2[4];
} b_struct_T;

#endif                                 //typedef_b_struct_T

// Custom Type definition for MATLAB Function: '<S10>/MATLAB Function1'
#ifndef struct_tag_ss1ABwxnargyhTC3tzoCWF
#define struct_tag_ss1ABwxnargyhTC3tzoCWF

struct tag_ss1ABwxnargyhTC3tzoCWF
{
  uint32_T LineWidth;
  uint32_T Color;
  uint32_T Opacity;
  uint32_T smoothEdges;
};

#endif                                 //struct_tag_ss1ABwxnargyhTC3tzoCWF

#ifndef typedef_ss1ABwxnargyhTC3tzoCWF
#define typedef_ss1ABwxnargyhTC3tzoCWF

typedef struct tag_ss1ABwxnargyhTC3tzoCWF ss1ABwxnargyhTC3tzoCWF;

#endif                                 //typedef_ss1ABwxnargyhTC3tzoCWF

#ifndef struct_tag_s9s8BC13iTohZXRbLMSIDHE
#define struct_tag_s9s8BC13iTohZXRbLMSIDHE

struct tag_s9s8BC13iTohZXRbLMSIDHE
{
  boolean_T CaseSensitivity;
  boolean_T StructExpand;
  boolean_T PartialMatching;
};

#endif                                 //struct_tag_s9s8BC13iTohZXRbLMSIDHE

#ifndef typedef_s9s8BC13iTohZXRbLMSIDHE
#define typedef_s9s8BC13iTohZXRbLMSIDHE

typedef struct tag_s9s8BC13iTohZXRbLMSIDHE s9s8BC13iTohZXRbLMSIDHE;

#endif                                 //typedef_s9s8BC13iTohZXRbLMSIDHE

#ifndef struct_md9618f2de20624d3dc0c1b05078441be7
#define struct_md9618f2de20624d3dc0c1b05078441be7

struct md9618f2de20624d3dc0c1b05078441be7
{
  int32_T S0_isInitialized;
  int32_T P0_RTP_LINEWIDTH;
};

#endif                                 //struct_md9618f2de20624d3dc0c1b05078441be7

#ifndef typedef_vision_ShapeInserter_7
#define typedef_vision_ShapeInserter_7

typedef struct md9618f2de20624d3dc0c1b05078441be7 vision_ShapeInserter_7;

#endif                                 //typedef_vision_ShapeInserter_7

#ifndef struct_md8RE5KOmaDDLogI2QQA3WVH
#define struct_md8RE5KOmaDDLogI2QQA3WVH

struct md8RE5KOmaDDLogI2QQA3WVH
{
  int32_T isInitialized;
  vision_ShapeInserter_7 cSFunObject;
  real_T LineWidth;
};

#endif                                 //struct_md8RE5KOmaDDLogI2QQA3WVH

#ifndef typedef_visioncodegen_ShapeInserter
#define typedef_visioncodegen_ShapeInserter

typedef struct md8RE5KOmaDDLogI2QQA3WVH visioncodegen_ShapeInserter;

#endif                                 //typedef_visioncodegen_ShapeInserter

#ifndef typedef_robotics_slros_internal_block_P
#define typedef_robotics_slros_internal_block_P

typedef struct {
  int32_T isInitialized;
} robotics_slros_internal_block_P;

#endif                                 //typedef_robotics_slros_internal_block_P

// Custom Type definition for MATLAB Function: '<S17>/ROSraw2RGB'
#ifndef typedef_struct_T_p
#define typedef_struct_T_p

typedef struct {
  int32_T f1;
  int32_T f2;
} struct_T_p;

#endif                                 //typedef_struct_T_p

#ifndef typedef_robotics_slros_internal_block_S
#define typedef_robotics_slros_internal_block_S

typedef struct {
  int32_T isInitialized;
} robotics_slros_internal_block_S;

#endif                                 //typedef_robotics_slros_internal_block_S

// Custom Type definition for MATLAB Function: '<S10>/MATLAB Function1'
#ifndef typedef_struct_T_n
#define typedef_struct_T_n

typedef struct {
  char_T f1[5];
  char_T f2[6];
  char_T f3[5];
  char_T f4[6];
  char_T f5[6];
} struct_T_n;

#endif                                 //typedef_struct_T_n

#ifndef typedef_b_struct_T_e
#define typedef_b_struct_T_e

typedef struct {
  char_T f1[4];
  char_T f2[9];
} b_struct_T_e;

#endif                                 //typedef_b_struct_T_e

#ifndef typedef_c_struct_T
#define typedef_c_struct_T

typedef struct {
  char_T f1[9];
  char_T f2[15];
  char_T f3[4];
  char_T f4[7];
  char_T f5[13];
  char_T f6[6];
  char_T f7[12];
} c_struct_T;

#endif                                 //typedef_c_struct_T

#ifndef typedef_d_struct_T
#define typedef_d_struct_T

typedef struct {
  char_T f1[7];
} d_struct_T;

#endif                                 //typedef_d_struct_T

#ifndef typedef_e_struct_T
#define typedef_e_struct_T

typedef struct {
  char_T f1[4];
  char_T f2[9];
  char_T f3[2];
  char_T f4[6];
} e_struct_T;

#endif                                 //typedef_e_struct_T

#ifndef struct_tag_sG0VVOg4rfK7hVMB5PGAhgB
#define struct_tag_sG0VVOg4rfK7hVMB5PGAhgB

struct tag_sG0VVOg4rfK7hVMB5PGAhgB
{
  real_T LineWidth;
  real_T Color[3];
  real_T Opacity;
  boolean_T smoothEdges;
};

#endif                                 //struct_tag_sG0VVOg4rfK7hVMB5PGAhgB

#ifndef typedef_sG0VVOg4rfK7hVMB5PGAhgB
#define typedef_sG0VVOg4rfK7hVMB5PGAhgB

typedef struct tag_sG0VVOg4rfK7hVMB5PGAhgB sG0VVOg4rfK7hVMB5PGAhgB;

#endif                                 //typedef_sG0VVOg4rfK7hVMB5PGAhgB

#ifndef typedef_f_struct_T
#define typedef_f_struct_T

typedef struct {
  char_T f1[9];
  char_T f2[7];
  char_T f3[6];
  char_T f4[4];
  char_T f5[8];
} f_struct_T;

#endif                                 //typedef_f_struct_T

#ifndef typedef_g_struct_T
#define typedef_g_struct_T

typedef struct {
  char_T f1[4];
  char_T f2[9];
  char_T f3[6];
  char_T f4[6];
  char_T f5[2];
  char_T f6[4];
  real_T f7[2];
} g_struct_T;

#endif                                 //typedef_g_struct_T

#ifndef typedef_h_struct_T
#define typedef_h_struct_T

typedef struct {
  char_T f1[8];
  char_T f2[6];
  char_T f3[6];
  char_T f4[9];
  char_T f5[4];
  char_T f6[6];
  char_T f7[2];
  real_T f8;
  char_T f9[2];
  real_T f10;
} h_struct_T;

#endif                                 //typedef_h_struct_T

#ifndef typedef_i_struct_T
#define typedef_i_struct_T

typedef struct {
  char_T f1[6];
} i_struct_T;

#endif                                 //typedef_i_struct_T

// Custom Type definition for MATLAB Function: '<S13>/MATLAB Function2'
#ifndef typedef_cell_wrap_0
#define typedef_cell_wrap_0

typedef struct {
  real_T f1[519901];
} cell_wrap_0;

#endif                                 //typedef_cell_wrap_0

#ifndef typedef_struct_T_l
#define typedef_struct_T_l

typedef struct {
  real_T f1[2];
} struct_T_l;

#endif                                 //typedef_struct_T_l

#ifndef typedef_struct_T_ls
#define typedef_struct_T_ls

typedef struct {
  char_T f1[4];
} struct_T_ls;

#endif                                 //typedef_struct_T_ls

#ifndef typedef_struct_T_ls1
#define typedef_struct_T_ls1

typedef struct {
  char_T f1[8];
} struct_T_ls1;

#endif                                 //typedef_struct_T_ls1

#ifndef typedef_struct_T_ls1q
#define typedef_struct_T_ls1q

typedef struct {
  char_T f1[7];
} struct_T_ls1q;

#endif                                 //typedef_struct_T_ls1q

#ifndef typedef_struct_T_ls1qn
#define typedef_struct_T_ls1qn

typedef struct {
  char_T f1[8];
  char_T f2[7];
  char_T f3[6];
} struct_T_ls1qn;

#endif                                 //typedef_struct_T_ls1qn

// Parameters (auto storage)
typedef struct P_ P;

// Forward declaration for rtModel
typedef struct tag_RTM RT_MODEL;

#endif                                 // RTW_HEADER_ros_image_filters_0706_types_h_ 

//
// File trailer for generated code.
//
// [EOF]
//
