//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: ros_image_filters_0706.cpp
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
#include "ros_image_filters_0706.h"
#include "ros_image_filters_0706_private.h"
#define MessageQueueLen                (1)

// Block signals (auto storage)
B rtB;

// Block states (auto storage)
DW rtDW;

// Real-time model
RT_MODEL rtM_;
RT_MODEL *const rtM = &rtM_;

// Forward declaration for local functions
static real_T sum(const real_T x[200]);
static void twister_state_vector_g(uint32_T mt[625], uint32_T seed);
static void genrand_uint32_vector(uint32_T mt[625], uint32_T u[2]);
static boolean_T is_valid_state(const uint32_T mt[625]);
static real_T eml_rand_mt19937ar_g(uint32_T state[625]);
static real_T eml_rand_mcg16807(uint32_T *state);
static void rand_o(real_T r[200]);
static void histc(const real_T X_0[200], const real_T edges[201], real_T N[201],
                  real_T BIN[200]);
static void randsample(const real_T varargin_4[200], real_T y[200]);
static void mean(const real_T x[519901], real_T y[961]);
static real_T eval_at_p(const real_T A[519901], const real_T point[2], real_T
  width);
static real_T mean_c(const real_T x[961]);
static void SystemCore_step(visioncodegen_ShapeInserter *obj, uint8_T
  varargin_1[1555200], const int32_T varargin_2[4]);
static void twister_state_vector(uint32_T mt[625], uint32_T seed);
static real_T eml_rand_mt19937ar(uint32_T state[625]);
static real_T rand_e(void);
static real_T sum_j(const boolean_T x[4]);
static real_T rand_oi(void);
static real_T eval_at_p_b(const real_T point[2], real_T width, const real_T A
  [519901]);
static real_T eml_rand_shr3cong(uint32_T state[2]);
static void genrandu(uint32_T s, uint32_T *state, real_T *r);
static real_T genrandu_h(uint32_T mt[625]);
static real_T eml_rand_mt19937ar_g2(uint32_T state[625]);
static real_T randn(void);
int32_T div_nzp_s32(int32_T numerator, int32_T denominator)
{
  uint32_T tempAbsQuotient;
  tempAbsQuotient = (numerator < 0 ? ~(uint32_T)numerator + 1U : (uint32_T)
                     numerator) / (denominator < 0 ? ~(uint32_T)denominator + 1U
    : (uint32_T)denominator);
  return (numerator < 0) != (denominator < 0) ? -(int32_T)tempAbsQuotient :
    (int32_T)tempAbsQuotient;
}

int32_T div_nzp_s32_floor(int32_T numerator, int32_T denominator)
{
  uint32_T absNumerator;
  uint32_T absDenominator;
  uint32_T tempAbsQuotient;
  boolean_T quotientNeedsNegation;
  absNumerator = numerator < 0 ? ~(uint32_T)numerator + 1U : (uint32_T)numerator;
  absDenominator = denominator < 0 ? ~(uint32_T)denominator + 1U : (uint32_T)
    denominator;
  quotientNeedsNegation = ((numerator < 0) != (denominator < 0));
  tempAbsQuotient = absNumerator / absDenominator;
  if (quotientNeedsNegation) {
    absNumerator %= absDenominator;
    if (absNumerator > 0U) {
      tempAbsQuotient++;
    }
  }

  return quotientNeedsNegation ? -(int32_T)tempAbsQuotient : (int32_T)
    tempAbsQuotient;
}

//
// Output and update for atomic system:
//    '<Root>/A1'
//    '<Root>/A2'
//
void A1(const real_T rtu_img[518400], real_T rty_int_img[519901], B_A1 *localB)
{
  int32_T pageoffset;
  int32_T i;
  int32_T k;

  // MATLAB Function 'A1': '<S3>:1'
  // '<S3>:1:3' int_img = integralImage(img);
  memset(&rty_int_img[0], 0, 519901U * sizeof(real_T));
  memcpy(&localB->x[0], &rtu_img[0], 518400U * sizeof(real_T));
  for (i = 0; i < 960; i++) {
    pageoffset = i * 540 + 1;
    for (k = 0; k < 539; k++) {
      localB->x[pageoffset + k] += localB->x[(pageoffset + k) - 1];
    }
  }

  for (i = 0; i < 540; i++) {
    for (pageoffset = 0; pageoffset < 959; pageoffset++) {
      localB->x[i + (pageoffset + 1) * 540] += localB->x[pageoffset * 540 + i];
    }
  }

  for (i = 0; i < 960; i++) {
    memcpy(&rty_int_img[i * 541 + 542], &localB->x[i * 540], 540U * sizeof
           (real_T));
  }
}

// Function for MATLAB Function: '<S13>/MATLAB Function2'
static real_T sum(const real_T x[200])
{
  real_T y;
  int32_T k;
  y = x[0];
  for (k = 0; k < 199; k++) {
    y += x[k + 1];
  }

  return y;
}

// Function for MATLAB Function: '<S13>/MATLAB Function2'
static void twister_state_vector_g(uint32_T mt[625], uint32_T seed)
{
  uint32_T r;
  int32_T mti;
  r = seed;
  mt[0] = seed;
  for (mti = 0; mti < 623; mti++) {
    r = ((r >> 30U ^ r) * 1812433253U + mti) + 1U;
    mt[mti + 1] = r;
  }

  mt[624] = 624U;
}

// Function for MATLAB Function: '<S13>/MATLAB Function2'
static void genrand_uint32_vector(uint32_T mt[625], uint32_T u[2])
{
  uint32_T mti;
  uint32_T y;
  int32_T j;
  int32_T kk;
  for (j = 0; j < 2; j++) {
    mti = mt[624] + 1U;
    if (mti >= 625U) {
      for (kk = 0; kk < 227; kk++) {
        y = (mt[kk + 1] & 2147483647U) | (mt[kk] & 2147483648U);
        if ((int32_T)(y & 1U) == 0) {
          y >>= 1U;
        } else {
          y = y >> 1U ^ 2567483615U;
        }

        mt[kk] = mt[kk + 397] ^ y;
      }

      for (kk = 0; kk < 396; kk++) {
        y = (mt[kk + 227] & 2147483648U) | (mt[kk + 228] & 2147483647U);
        if ((int32_T)(y & 1U) == 0) {
          y >>= 1U;
        } else {
          y = y >> 1U ^ 2567483615U;
        }

        mt[kk + 227] = mt[kk] ^ y;
      }

      y = (mt[623] & 2147483648U) | (mt[0] & 2147483647U);
      if ((int32_T)(y & 1U) == 0) {
        y >>= 1U;
      } else {
        y = y >> 1U ^ 2567483615U;
      }

      mt[623] = mt[396] ^ y;
      mti = 1U;
    }

    y = mt[(int32_T)mti - 1];
    mt[624] = mti;
    y ^= y >> 11U;
    y ^= y << 7U & 2636928640U;
    y ^= y << 15U & 4022730752U;
    y ^= y >> 18U;
    u[j] = y;
  }
}

// Function for MATLAB Function: '<S13>/MATLAB Function2'
static boolean_T is_valid_state(const uint32_T mt[625])
{
  boolean_T isvalid;
  int32_T k;
  boolean_T exitg1;
  isvalid = ((mt[624] >= 1U) && (mt[624] < 625U));
  if (isvalid) {
    isvalid = false;
    k = 1;
    exitg1 = false;
    while ((!exitg1) && (k < 625)) {
      if (mt[k - 1] == 0U) {
        k++;
      } else {
        isvalid = true;
        exitg1 = true;
      }
    }
  }

  return isvalid;
}

// Function for MATLAB Function: '<S13>/MATLAB Function2'
static real_T eml_rand_mt19937ar_g(uint32_T state[625])
{
  real_T r;
  int32_T exitg1;

  // ========================= COPYRIGHT NOTICE ============================
  //  This is a uniform (0,1) pseudorandom number generator based on:
  //
  //  A C-program for MT19937, with initialization improved 2002/1/26.
  //  Coded by Takuji Nishimura and Makoto Matsumoto.
  //
  //  Copyright (C) 1997 - 2002, Makoto Matsumoto and Takuji Nishimura,
  //  All rights reserved.
  //
  //  Redistribution and use in source and binary forms, with or without
  //  modification, are permitted provided that the following conditions
  //  are met:
  //
  //    1. Redistributions of source code must retain the above copyright
  //       notice, this list of conditions and the following disclaimer.
  //
  //    2. Redistributions in binary form must reproduce the above copyright
  //       notice, this list of conditions and the following disclaimer
  //       in the documentation and/or other materials provided with the
  //       distribution.
  //
  //    3. The names of its contributors may not be used to endorse or
  //       promote products derived from this software without specific
  //       prior written permission.
  //
  //  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
  //  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
  //  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
  //  A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT
  //  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
  //  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  //  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
  //  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
  //  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
  //  (INCLUDING  NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  //  OF THIS  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  //
  // =============================   END   =================================
  do {
    exitg1 = 0;
    genrand_uint32_vector(state, rtB.u);
    r = ((real_T)(rtB.u[0] >> 5U) * 6.7108864E+7 + (real_T)(rtB.u[1] >> 6U)) *
      1.1102230246251565E-16;
    if (r == 0.0) {
      if (!is_valid_state(state)) {
        twister_state_vector_g(state, 5489U);
      }
    } else {
      exitg1 = 1;
    }
  } while (exitg1 == 0);

  return r;
}

// Function for MATLAB Function: '<S13>/MATLAB Function2'
static real_T eml_rand_mcg16807(uint32_T *state)
{
  int32_T hi;
  uint32_T test1;
  uint32_T test2;
  hi = (int32_T)(*state / 127773U);
  test1 = (*state - hi * 127773U) * 16807U;
  test2 = 2836U * hi;
  if (test1 < test2) {
    test1 = (test1 - test2) + 2147483647U;
  } else {
    test1 -= test2;
  }

  *state = test1;
  return (real_T)test1 * 4.6566128752457969E-10;
}

// Function for MATLAB Function: '<S13>/MATLAB Function2'
static void rand_o(real_T r[200])
{
  int32_T k;
  uint32_T d;
  uint32_T state;
  switch (rtDW.method) {
   case 4U:
    for (k = 0; k < 200; k++) {
      state = rtDW.state;
      rtB.b_c = eml_rand_mcg16807(&state);
      rtDW.state = state;
      r[k] = rtB.b_c;
    }
    break;

   case 5U:
    for (k = 0; k < 200; k++) {
      state = 69069U * rtDW.state_a[0] + 1234567U;
      d = rtDW.state_a[1] << 13 ^ rtDW.state_a[1];
      d ^= d >> 17;
      d ^= d << 5;
      rtDW.state_a[0] = state;
      rtDW.state_a[1] = d;
      r[k] = (real_T)(state + d) * 2.328306436538696E-10;
    }
    break;

   default:
    if (!rtDW.state_not_empty) {
      memset(&rtDW.state_i[0], 0, 625U * sizeof(uint32_T));
      twister_state_vector_g(rtDW.state_i, 5489U);
      rtDW.state_not_empty = true;
    }

    for (k = 0; k < 200; k++) {
      rtB.b_c = eml_rand_mt19937ar_g(rtDW.state_i);
      r[k] = rtB.b_c;
    }
    break;
  }
}

// Function for MATLAB Function: '<S13>/MATLAB Function2'
static void histc(const real_T X_0[200], const real_T edges[201], real_T N[201],
                  real_T BIN[200])
{
  boolean_T eok;
  int32_T xind;
  int32_T k;
  int32_T low_i;
  int32_T low_ip1;
  int32_T high_i;
  int32_T mid_i;
  int32_T exitg1;
  memset(&N[0], 0, 201U * sizeof(real_T));
  memset(&BIN[0], 0, 200U * sizeof(real_T));
  xind = 1;
  do {
    exitg1 = 0;
    if (xind + 1 < 202) {
      if (!(edges[xind] >= edges[xind - 1])) {
        eok = false;
        exitg1 = 1;
      } else {
        xind++;
      }
    } else {
      eok = true;
      exitg1 = 1;
    }
  } while (exitg1 == 0);

  if (!eok) {
    for (xind = 0; xind < 201; xind++) {
      N[xind] = (rtNaN);
    }

    for (xind = 0; xind < 200; xind++) {
      BIN[xind] = (rtNaN);
    }
  } else {
    xind = 0;
    for (k = 0; k < 200; k++) {
      low_i = 0;
      if (!rtIsNaN(X_0[xind])) {
        if ((X_0[xind] >= edges[0]) && (X_0[xind] < edges[200])) {
          low_i = 1;
          low_ip1 = 2;
          high_i = 201;
          while (high_i > low_ip1) {
            mid_i = (low_i + high_i) >> 1;
            if (X_0[xind] >= edges[mid_i - 1]) {
              low_i = mid_i;
              low_ip1 = mid_i + 1;
            } else {
              high_i = mid_i;
            }
          }
        }

        if (X_0[xind] == edges[200]) {
          low_i = 201;
        }
      }

      if (low_i > 0) {
        N[low_i - 1]++;
      }

      BIN[xind] = low_i;
      xind++;
    }
  }
}

// Function for MATLAB Function: '<S13>/MATLAB Function2'
static void randsample(const real_T varargin_4[200], real_T y[200])
{
  int32_T j;
  rtB.sumw = sum(varargin_4);
  rtB.edges[0] = 0.0;
  rtB.edges[200] = 1.0;
  for (j = 0; j < 199; j++) {
    rtB.edges[j + 1] = fmin(varargin_4[j] / rtB.sumw + rtB.edges[j], 1.0);
  }

  rand_o(rtB.b_m);
  histc(rtB.b_m, rtB.edges, rtB.unusedU0, y);
}

// Function for MATLAB Function: '<S10>/MATLAB Function1'
static void mean(const real_T x[519901], real_T y[961])
{
  real_T s;
  int32_T xoffset;
  int32_T i;
  int32_T k;
  for (i = 0; i < 961; i++) {
    xoffset = i * 541;
    s = x[xoffset];
    for (k = 0; k < 540; k++) {
      s += x[(xoffset + k) + 1];
    }

    y[i] = s / 541.0;
  }
}

//
// Function for MATLAB Function: '<S10>/MATLAB Function1'
// function value = eval_at_p(A, point, width)
//
static real_T eval_at_p(const real_T A[519901], const real_T point[2], real_T
  width)
{
  //  the eval() function
  // '<S29>:1:62' p_tl = ceil([min(max(point(1),1), 541), min(max(point(2), 1), 961)]); 
  // '<S29>:1:63' p_tr = ceil([min(max(point(1),1), 541), max(min(point(2), 961 - width) + width, 1)]); 
  // '<S29>:1:64' p_bl = ceil([max(min(point(1),541 - width) + width, 1), min(max(point(2), 1), 961)]); 
  // '<S29>:1:65' p_br = ceil([max(min(point(1),541 - width) + width, 1), max(min(point(2),961 - width) + width, 1)]); 
  // '<S29>:1:67' value = A(p_tl(1), p_tl(2)) + A(p_br(1),p_br(2))...
  // '<S29>:1:68'     - A(p_tr(1), p_tr(2)) - A(p_bl(1),p_bl(2));
  return ((A[(((int32_T)ceil(fmax(fmin(point[1], 961.0 - width) + width, 1.0)) -
               1) * 541 + (int32_T)ceil(fmax(fmin(point[0], 541.0 - width) +
              width, 1.0))) - 1] + A[(int32_T)((ceil(fmin(fmax(point[1], 1.0),
    961.0)) - 1.0) * 541.0 + ceil(fmin(fmax(point[0], 1.0), 541.0))) - 1]) - A
          [(((int32_T)ceil(fmax(fmin(point[1], 961.0 - width) + width, 1.0)) - 1)
            * 541 + (int32_T)ceil(fmin(fmax(point[0], 1.0), 541.0))) - 1]) - A
    [((int32_T)ceil(fmax(fmin(point[0], 541.0 - width) + width, 1.0)) +
      ((int32_T)ceil(fmin(fmax(point[1], 1.0), 961.0)) - 1) * 541) - 1];
}

// Function for MATLAB Function: '<S10>/MATLAB Function1'
static real_T mean_c(const real_T x[961])
{
  real_T b_y;
  int32_T k;
  b_y = x[0];
  for (k = 0; k < 960; k++) {
    b_y += x[k + 1];
  }

  return b_y / 961.0;
}

real_T rt_roundd_snf(real_T u)
{
  real_T y;
  if (fabs(u) < 4.503599627370496E+15) {
    if (u >= 0.5) {
      y = floor(u + 0.5);
    } else if (u > -0.5) {
      y = u * 0.0;
    } else {
      y = ceil(u - 0.5);
    }
  } else {
    y = u;
  }

  return y;
}

// Function for MATLAB Function: '<S10>/MATLAB Function1'
static void SystemCore_step(visioncodegen_ShapeInserter *obj, uint8_T
  varargin_1[1555200], const int32_T varargin_2[4])
{
  visioncodegen_ShapeInserter *b_obj;
  boolean_T isInBound;
  boolean_T visited1;
  boolean_T visited2;
  boolean_T done;
  int32_T b_line_idx_2;
  int32_T b_line_idx_1;
  int32_T b_line_idx_0;
  int32_T line_idx_2;
  int32_T line_idx_0;
  if (obj->isInitialized != 1) {
    b_obj = obj;
    b_obj->isInitialized = 1;
  }

  b_obj = obj;

  // System object Outputs function: vision.ShapeInserter
  // Copy the image from input to output.
  // Update view port.
  // Draw all rectangles.
  rtB.firstRow = varargin_2[1] - 1;
  rtB.firstCol = varargin_2[0] - 1;
  rtB.lastRow = (varargin_2[1] + varargin_2[3]) - 2;
  rtB.lastCol = (varargin_2[0] + varargin_2[2]) - 2;
  if (b_obj->cSFunObject.P0_RTP_LINEWIDTH > 1) {
    rtB.halfLineWidth = b_obj->cSFunObject.P0_RTP_LINEWIDTH >> 1;
    rtB.firstRow = (varargin_2[1] - rtB.halfLineWidth) - 1;
    rtB.lastRow += rtB.halfLineWidth;
    rtB.firstCol = (varargin_2[0] - rtB.halfLineWidth) - 1;
    rtB.lastCol += rtB.halfLineWidth;
  }

  if ((rtB.firstRow <= rtB.lastRow) && (rtB.firstCol <= rtB.lastCol)) {
    rtB.halfLineWidth = 0;
    while (rtB.halfLineWidth < b_obj->cSFunObject.P0_RTP_LINEWIDTH) {
      line_idx_0 = rtB.firstRow + rtB.halfLineWidth;
      line_idx_2 = rtB.firstRow + rtB.halfLineWidth;
      isInBound = false;

      // Find the visible portion of a line.
      visited1 = false;
      visited2 = false;
      done = false;
      b_line_idx_0 = line_idx_0;
      b_line_idx_1 = rtB.firstCol;
      b_line_idx_2 = line_idx_2;
      rtB.b_line_idx_3 = rtB.lastCol;
      while (!done) {
        rtB.d_idxFillColor = 0;
        rtB.h_idxPix = 0;

        // Determine viewport violations.
        if (b_line_idx_0 < 0) {
          rtB.d_idxFillColor = 4;
        } else {
          if (b_line_idx_0 > 539) {
            rtB.d_idxFillColor = 8;
          }
        }

        if (b_line_idx_2 < 0) {
          rtB.h_idxPix = 4;
        } else {
          if (b_line_idx_2 > 539) {
            rtB.h_idxPix = 8;
          }
        }

        if (b_line_idx_1 < 0) {
          rtB.d_idxFillColor |= 1U;
        } else {
          if (b_line_idx_1 > 959) {
            rtB.d_idxFillColor |= 2U;
          }
        }

        if (rtB.b_line_idx_3 < 0) {
          rtB.h_idxPix |= 1U;
        } else {
          if (rtB.b_line_idx_3 > 959) {
            rtB.h_idxPix |= 2U;
          }
        }

        if (!(((uint32_T)rtB.d_idxFillColor | rtB.h_idxPix) != 0U)) {
          // Line falls completely within bounds.
          done = true;
          isInBound = true;
        } else if (((uint32_T)rtB.d_idxFillColor & rtB.h_idxPix) != 0U) {
          // Line falls completely out of bounds.
          done = true;
          isInBound = false;
        } else if ((uint32_T)rtB.d_idxFillColor != 0U) {
          // Clip 1st point; if it's in-bounds, clip 2nd point.
          if (visited1) {
            b_line_idx_0 = line_idx_0;
            b_line_idx_1 = rtB.firstCol;
          }

          rtB.i = b_line_idx_2 - b_line_idx_0;
          rtB.in = rtB.b_line_idx_3 - b_line_idx_1;
          if ((rtB.i > 1073741824) || (rtB.i < -1073741824) || ((rtB.in >
                1073741824) || (rtB.in < -1073741824))) {
            // Possible Inf or Nan.
            done = true;
            isInBound = false;
            visited1 = true;
          } else if ((rtB.d_idxFillColor & 4U) != 0U) {
            // Violated RMin.
            rtB.d_idxFillColor = -b_line_idx_0 * rtB.in;
            if ((rtB.d_idxFillColor > 1073741824) || (rtB.d_idxFillColor <
                 -1073741824)) {
              // Check for Inf or Nan.
              done = true;
              isInBound = false;
            } else if (((rtB.d_idxFillColor >= 0) && (rtB.i >= 0)) ||
                       ((rtB.d_idxFillColor < 0) && (rtB.i < 0))) {
              b_line_idx_1 += (div_nzp_s32_floor(rtB.d_idxFillColor << 1, rtB.i)
                               + 1) >> 1;
            } else {
              b_line_idx_1 -= (div_nzp_s32_floor(-rtB.d_idxFillColor << 1, rtB.i)
                               + 1) >> 1;
            }

            b_line_idx_0 = 0;
            visited1 = true;
          } else if ((rtB.d_idxFillColor & 8U) != 0U) {
            // Violated RMax.
            rtB.d_idxFillColor = (539 - b_line_idx_0) * rtB.in;
            if ((rtB.d_idxFillColor > 1073741824) || (rtB.d_idxFillColor <
                 -1073741824)) {
              // Check for Inf or Nan.
              done = true;
              isInBound = false;
            } else if (((rtB.d_idxFillColor >= 0) && (rtB.i >= 0)) ||
                       ((rtB.d_idxFillColor < 0) && (rtB.i < 0))) {
              b_line_idx_1 += (div_nzp_s32_floor(rtB.d_idxFillColor << 1, rtB.i)
                               + 1) >> 1;
            } else {
              b_line_idx_1 -= (div_nzp_s32_floor(-rtB.d_idxFillColor << 1, rtB.i)
                               + 1) >> 1;
            }

            b_line_idx_0 = 539;
            visited1 = true;
          } else if ((rtB.d_idxFillColor & 1U) != 0U) {
            // Violated CMin.
            rtB.d_idxFillColor = -b_line_idx_1 * rtB.i;
            if ((rtB.d_idxFillColor > 1073741824) || (rtB.d_idxFillColor <
                 -1073741824)) {
              // Check for Inf or Nan.
              done = true;
              isInBound = false;
            } else if (((rtB.d_idxFillColor >= 0) && (rtB.in >= 0)) ||
                       ((rtB.d_idxFillColor < 0) && (rtB.in < 0))) {
              b_line_idx_0 += (div_nzp_s32_floor(rtB.d_idxFillColor << 1, rtB.in)
                               + 1) >> 1;
            } else {
              b_line_idx_0 -= (div_nzp_s32_floor(-rtB.d_idxFillColor << 1,
                rtB.in) + 1) >> 1;
            }

            b_line_idx_1 = 0;
            visited1 = true;
          } else {
            // Violated CMax.
            rtB.d_idxFillColor = (959 - b_line_idx_1) * rtB.i;
            if ((rtB.d_idxFillColor > 1073741824) || (rtB.d_idxFillColor <
                 -1073741824)) {
              // Check for Inf or Nan.
              done = true;
              isInBound = false;
            } else if (((rtB.d_idxFillColor >= 0) && (rtB.in >= 0)) ||
                       ((rtB.d_idxFillColor < 0) && (rtB.in < 0))) {
              b_line_idx_0 += (div_nzp_s32_floor(rtB.d_idxFillColor << 1, rtB.in)
                               + 1) >> 1;
            } else {
              b_line_idx_0 -= (div_nzp_s32_floor(-rtB.d_idxFillColor << 1,
                rtB.in) + 1) >> 1;
            }

            b_line_idx_1 = 959;
            visited1 = true;
          }
        } else {
          // Clip the 2nd point.
          if (visited2) {
            b_line_idx_2 = line_idx_2;
            rtB.b_line_idx_3 = rtB.lastCol;
          }

          rtB.i = b_line_idx_2 - b_line_idx_0;
          rtB.in = rtB.b_line_idx_3 - b_line_idx_1;
          if ((rtB.i > 1073741824) || (rtB.i < -1073741824) || ((rtB.in >
                1073741824) || (rtB.in < -1073741824))) {
            // Possible Inf or Nan.
            done = true;
            isInBound = false;
            visited2 = true;
          } else if ((rtB.h_idxPix & 4U) != 0U) {
            // Violated RMin.
            rtB.d_idxFillColor = -b_line_idx_2 * rtB.in;
            if ((rtB.d_idxFillColor > 1073741824) || (rtB.d_idxFillColor <
                 -1073741824)) {
              // Check for Inf or Nan.
              done = true;
              isInBound = false;
            } else if (((rtB.d_idxFillColor >= 0) && (rtB.i >= 0)) ||
                       ((rtB.d_idxFillColor < 0) && (rtB.i < 0))) {
              rtB.b_line_idx_3 += (div_nzp_s32_floor(rtB.d_idxFillColor << 1,
                rtB.i) + 1) >> 1;
            } else {
              rtB.b_line_idx_3 -= (div_nzp_s32_floor(-rtB.d_idxFillColor << 1,
                rtB.i) + 1) >> 1;
            }

            b_line_idx_2 = 0;
            visited2 = true;
          } else if ((rtB.h_idxPix & 8U) != 0U) {
            // Violated RMax.
            rtB.d_idxFillColor = (539 - b_line_idx_2) * rtB.in;
            if ((rtB.d_idxFillColor > 1073741824) || (rtB.d_idxFillColor <
                 -1073741824)) {
              // Check for Inf or Nan.
              done = true;
              isInBound = false;
            } else if (((rtB.d_idxFillColor >= 0) && (rtB.i >= 0)) ||
                       ((rtB.d_idxFillColor < 0) && (rtB.i < 0))) {
              rtB.b_line_idx_3 += (div_nzp_s32_floor(rtB.d_idxFillColor << 1,
                rtB.i) + 1) >> 1;
            } else {
              rtB.b_line_idx_3 -= (div_nzp_s32_floor(-rtB.d_idxFillColor << 1,
                rtB.i) + 1) >> 1;
            }

            b_line_idx_2 = 539;
            visited2 = true;
          } else if ((rtB.h_idxPix & 1U) != 0U) {
            // Violated CMin.
            rtB.d_idxFillColor = -rtB.b_line_idx_3 * rtB.i;
            if ((rtB.d_idxFillColor > 1073741824) || (rtB.d_idxFillColor <
                 -1073741824)) {
              // Check for Inf or Nan.
              done = true;
              isInBound = false;
            } else if (((rtB.d_idxFillColor >= 0) && (rtB.in >= 0)) ||
                       ((rtB.d_idxFillColor < 0) && (rtB.in < 0))) {
              b_line_idx_2 += (div_nzp_s32_floor(rtB.d_idxFillColor << 1, rtB.in)
                               + 1) >> 1;
            } else {
              b_line_idx_2 -= (div_nzp_s32_floor(-rtB.d_idxFillColor << 1,
                rtB.in) + 1) >> 1;
            }

            rtB.b_line_idx_3 = 0;
            visited2 = true;
          } else {
            // Violated CMax.
            rtB.d_idxFillColor = (959 - rtB.b_line_idx_3) * rtB.i;
            if ((rtB.d_idxFillColor > 1073741824) || (rtB.d_idxFillColor <
                 -1073741824)) {
              // Check for Inf or Nan.
              done = true;
              isInBound = false;
            } else if (((rtB.d_idxFillColor >= 0) && (rtB.in >= 0)) ||
                       ((rtB.d_idxFillColor < 0) && (rtB.in < 0))) {
              b_line_idx_2 += (div_nzp_s32_floor(rtB.d_idxFillColor << 1, rtB.in)
                               + 1) >> 1;
            } else {
              b_line_idx_2 -= (div_nzp_s32_floor(-rtB.d_idxFillColor << 1,
                rtB.in) + 1) >> 1;
            }

            rtB.b_line_idx_3 = 959;
            visited2 = true;
          }
        }
      }

      if (isInBound) {
        rtB.i = b_line_idx_1 * 540 + b_line_idx_0;
        rtB.in = b_line_idx_1;
        while (rtB.in <= rtB.b_line_idx_3) {
          varargin_1[rtB.i] = MAX_uint8_T;
          rtB.h_idxPix = rtB.i + 518400;
          varargin_1[rtB.h_idxPix] = MAX_uint8_T;
          rtB.h_idxPix += 518400;
          varargin_1[rtB.h_idxPix] = 0U;
          rtB.i += 540;
          rtB.in++;
        }
      }

      line_idx_0 = rtB.firstCol + rtB.halfLineWidth;
      line_idx_2 = rtB.firstCol + rtB.halfLineWidth;
      isInBound = false;

      // Find the visible portion of a line.
      visited1 = false;
      visited2 = false;
      done = false;
      b_line_idx_0 = rtB.firstRow;
      b_line_idx_1 = line_idx_0;
      b_line_idx_2 = rtB.lastRow;
      rtB.b_line_idx_3 = line_idx_2;
      while (!done) {
        rtB.d_idxFillColor = 0;
        rtB.h_idxPix = 0;

        // Determine viewport violations.
        if (b_line_idx_0 < 0) {
          rtB.d_idxFillColor = 4;
        } else {
          if (b_line_idx_0 > 539) {
            rtB.d_idxFillColor = 8;
          }
        }

        if (b_line_idx_2 < 0) {
          rtB.h_idxPix = 4;
        } else {
          if (b_line_idx_2 > 539) {
            rtB.h_idxPix = 8;
          }
        }

        if (b_line_idx_1 < 0) {
          rtB.d_idxFillColor |= 1U;
        } else {
          if (b_line_idx_1 > 959) {
            rtB.d_idxFillColor |= 2U;
          }
        }

        if (rtB.b_line_idx_3 < 0) {
          rtB.h_idxPix |= 1U;
        } else {
          if (rtB.b_line_idx_3 > 959) {
            rtB.h_idxPix |= 2U;
          }
        }

        if (!(((uint32_T)rtB.d_idxFillColor | rtB.h_idxPix) != 0U)) {
          // Line falls completely within bounds.
          done = true;
          isInBound = true;
        } else if (((uint32_T)rtB.d_idxFillColor & rtB.h_idxPix) != 0U) {
          // Line falls completely out of bounds.
          done = true;
          isInBound = false;
        } else if ((uint32_T)rtB.d_idxFillColor != 0U) {
          // Clip 1st point; if it's in-bounds, clip 2nd point.
          if (visited1) {
            b_line_idx_0 = rtB.firstRow;
            b_line_idx_1 = line_idx_0;
          }

          rtB.i = b_line_idx_2 - b_line_idx_0;
          rtB.in = rtB.b_line_idx_3 - b_line_idx_1;
          if ((rtB.i > 1073741824) || (rtB.i < -1073741824) || ((rtB.in >
                1073741824) || (rtB.in < -1073741824))) {
            // Possible Inf or Nan.
            done = true;
            isInBound = false;
            visited1 = true;
          } else if ((rtB.d_idxFillColor & 4U) != 0U) {
            // Violated RMin.
            rtB.d_idxFillColor = -b_line_idx_0 * rtB.in;
            if ((rtB.d_idxFillColor > 1073741824) || (rtB.d_idxFillColor <
                 -1073741824)) {
              // Check for Inf or Nan.
              done = true;
              isInBound = false;
            } else if (((rtB.d_idxFillColor >= 0) && (rtB.i >= 0)) ||
                       ((rtB.d_idxFillColor < 0) && (rtB.i < 0))) {
              b_line_idx_1 += (div_nzp_s32_floor(rtB.d_idxFillColor << 1, rtB.i)
                               + 1) >> 1;
            } else {
              b_line_idx_1 -= (div_nzp_s32_floor(-rtB.d_idxFillColor << 1, rtB.i)
                               + 1) >> 1;
            }

            b_line_idx_0 = 0;
            visited1 = true;
          } else if ((rtB.d_idxFillColor & 8U) != 0U) {
            // Violated RMax.
            rtB.d_idxFillColor = (539 - b_line_idx_0) * rtB.in;
            if ((rtB.d_idxFillColor > 1073741824) || (rtB.d_idxFillColor <
                 -1073741824)) {
              // Check for Inf or Nan.
              done = true;
              isInBound = false;
            } else if (((rtB.d_idxFillColor >= 0) && (rtB.i >= 0)) ||
                       ((rtB.d_idxFillColor < 0) && (rtB.i < 0))) {
              b_line_idx_1 += (div_nzp_s32_floor(rtB.d_idxFillColor << 1, rtB.i)
                               + 1) >> 1;
            } else {
              b_line_idx_1 -= (div_nzp_s32_floor(-rtB.d_idxFillColor << 1, rtB.i)
                               + 1) >> 1;
            }

            b_line_idx_0 = 539;
            visited1 = true;
          } else if ((rtB.d_idxFillColor & 1U) != 0U) {
            // Violated CMin.
            rtB.d_idxFillColor = -b_line_idx_1 * rtB.i;
            if ((rtB.d_idxFillColor > 1073741824) || (rtB.d_idxFillColor <
                 -1073741824)) {
              // Check for Inf or Nan.
              done = true;
              isInBound = false;
            } else if (((rtB.d_idxFillColor >= 0) && (rtB.in >= 0)) ||
                       ((rtB.d_idxFillColor < 0) && (rtB.in < 0))) {
              b_line_idx_0 += (div_nzp_s32_floor(rtB.d_idxFillColor << 1, rtB.in)
                               + 1) >> 1;
            } else {
              b_line_idx_0 -= (div_nzp_s32_floor(-rtB.d_idxFillColor << 1,
                rtB.in) + 1) >> 1;
            }

            b_line_idx_1 = 0;
            visited1 = true;
          } else {
            // Violated CMax.
            rtB.d_idxFillColor = (959 - b_line_idx_1) * rtB.i;
            if ((rtB.d_idxFillColor > 1073741824) || (rtB.d_idxFillColor <
                 -1073741824)) {
              // Check for Inf or Nan.
              done = true;
              isInBound = false;
            } else if (((rtB.d_idxFillColor >= 0) && (rtB.in >= 0)) ||
                       ((rtB.d_idxFillColor < 0) && (rtB.in < 0))) {
              b_line_idx_0 += (div_nzp_s32_floor(rtB.d_idxFillColor << 1, rtB.in)
                               + 1) >> 1;
            } else {
              b_line_idx_0 -= (div_nzp_s32_floor(-rtB.d_idxFillColor << 1,
                rtB.in) + 1) >> 1;
            }

            b_line_idx_1 = 959;
            visited1 = true;
          }
        } else {
          // Clip the 2nd point.
          if (visited2) {
            b_line_idx_2 = rtB.lastRow;
            rtB.b_line_idx_3 = line_idx_2;
          }

          rtB.i = b_line_idx_2 - b_line_idx_0;
          rtB.in = rtB.b_line_idx_3 - b_line_idx_1;
          if ((rtB.i > 1073741824) || (rtB.i < -1073741824) || ((rtB.in >
                1073741824) || (rtB.in < -1073741824))) {
            // Possible Inf or Nan.
            done = true;
            isInBound = false;
            visited2 = true;
          } else if ((rtB.h_idxPix & 4U) != 0U) {
            // Violated RMin.
            rtB.d_idxFillColor = -b_line_idx_2 * rtB.in;
            if ((rtB.d_idxFillColor > 1073741824) || (rtB.d_idxFillColor <
                 -1073741824)) {
              // Check for Inf or Nan.
              done = true;
              isInBound = false;
            } else if (((rtB.d_idxFillColor >= 0) && (rtB.i >= 0)) ||
                       ((rtB.d_idxFillColor < 0) && (rtB.i < 0))) {
              rtB.b_line_idx_3 += (div_nzp_s32_floor(rtB.d_idxFillColor << 1,
                rtB.i) + 1) >> 1;
            } else {
              rtB.b_line_idx_3 -= (div_nzp_s32_floor(-rtB.d_idxFillColor << 1,
                rtB.i) + 1) >> 1;
            }

            b_line_idx_2 = 0;
            visited2 = true;
          } else if ((rtB.h_idxPix & 8U) != 0U) {
            // Violated RMax.
            rtB.d_idxFillColor = (539 - b_line_idx_2) * rtB.in;
            if ((rtB.d_idxFillColor > 1073741824) || (rtB.d_idxFillColor <
                 -1073741824)) {
              // Check for Inf or Nan.
              done = true;
              isInBound = false;
            } else if (((rtB.d_idxFillColor >= 0) && (rtB.i >= 0)) ||
                       ((rtB.d_idxFillColor < 0) && (rtB.i < 0))) {
              rtB.b_line_idx_3 += (div_nzp_s32_floor(rtB.d_idxFillColor << 1,
                rtB.i) + 1) >> 1;
            } else {
              rtB.b_line_idx_3 -= (div_nzp_s32_floor(-rtB.d_idxFillColor << 1,
                rtB.i) + 1) >> 1;
            }

            b_line_idx_2 = 539;
            visited2 = true;
          } else if ((rtB.h_idxPix & 1U) != 0U) {
            // Violated CMin.
            rtB.d_idxFillColor = -rtB.b_line_idx_3 * rtB.i;
            if ((rtB.d_idxFillColor > 1073741824) || (rtB.d_idxFillColor <
                 -1073741824)) {
              // Check for Inf or Nan.
              done = true;
              isInBound = false;
            } else if (((rtB.d_idxFillColor >= 0) && (rtB.in >= 0)) ||
                       ((rtB.d_idxFillColor < 0) && (rtB.in < 0))) {
              b_line_idx_2 += (div_nzp_s32_floor(rtB.d_idxFillColor << 1, rtB.in)
                               + 1) >> 1;
            } else {
              b_line_idx_2 -= (div_nzp_s32_floor(-rtB.d_idxFillColor << 1,
                rtB.in) + 1) >> 1;
            }

            rtB.b_line_idx_3 = 0;
            visited2 = true;
          } else {
            // Violated CMax.
            rtB.d_idxFillColor = (959 - rtB.b_line_idx_3) * rtB.i;
            if ((rtB.d_idxFillColor > 1073741824) || (rtB.d_idxFillColor <
                 -1073741824)) {
              // Check for Inf or Nan.
              done = true;
              isInBound = false;
            } else if (((rtB.d_idxFillColor >= 0) && (rtB.in >= 0)) ||
                       ((rtB.d_idxFillColor < 0) && (rtB.in < 0))) {
              b_line_idx_2 += (div_nzp_s32_floor(rtB.d_idxFillColor << 1, rtB.in)
                               + 1) >> 1;
            } else {
              b_line_idx_2 -= (div_nzp_s32_floor(-rtB.d_idxFillColor << 1,
                rtB.in) + 1) >> 1;
            }

            rtB.b_line_idx_3 = 959;
            visited2 = true;
          }
        }
      }

      if (isInBound) {
        rtB.i = b_line_idx_1 * 540 + b_line_idx_0;
        rtB.in = b_line_idx_0;
        while (rtB.in <= b_line_idx_2) {
          varargin_1[rtB.i] = MAX_uint8_T;
          rtB.h_idxPix = rtB.i + 518400;
          varargin_1[rtB.h_idxPix] = MAX_uint8_T;
          rtB.h_idxPix += 518400;
          varargin_1[rtB.h_idxPix] = 0U;
          rtB.i++;
          rtB.in++;
        }
      }

      line_idx_0 = rtB.lastRow - rtB.halfLineWidth;
      line_idx_2 = rtB.lastRow - rtB.halfLineWidth;
      isInBound = false;

      // Find the visible portion of a line.
      visited1 = false;
      visited2 = false;
      done = false;
      b_line_idx_0 = line_idx_0;
      b_line_idx_1 = rtB.firstCol;
      b_line_idx_2 = line_idx_2;
      rtB.b_line_idx_3 = rtB.lastCol;
      while (!done) {
        rtB.d_idxFillColor = 0;
        rtB.h_idxPix = 0;

        // Determine viewport violations.
        if (b_line_idx_0 < 0) {
          rtB.d_idxFillColor = 4;
        } else {
          if (b_line_idx_0 > 539) {
            rtB.d_idxFillColor = 8;
          }
        }

        if (b_line_idx_2 < 0) {
          rtB.h_idxPix = 4;
        } else {
          if (b_line_idx_2 > 539) {
            rtB.h_idxPix = 8;
          }
        }

        if (b_line_idx_1 < 0) {
          rtB.d_idxFillColor |= 1U;
        } else {
          if (b_line_idx_1 > 959) {
            rtB.d_idxFillColor |= 2U;
          }
        }

        if (rtB.b_line_idx_3 < 0) {
          rtB.h_idxPix |= 1U;
        } else {
          if (rtB.b_line_idx_3 > 959) {
            rtB.h_idxPix |= 2U;
          }
        }

        if (!(((uint32_T)rtB.d_idxFillColor | rtB.h_idxPix) != 0U)) {
          // Line falls completely within bounds.
          done = true;
          isInBound = true;
        } else if (((uint32_T)rtB.d_idxFillColor & rtB.h_idxPix) != 0U) {
          // Line falls completely out of bounds.
          done = true;
          isInBound = false;
        } else if ((uint32_T)rtB.d_idxFillColor != 0U) {
          // Clip 1st point; if it's in-bounds, clip 2nd point.
          if (visited1) {
            b_line_idx_0 = line_idx_0;
            b_line_idx_1 = rtB.firstCol;
          }

          rtB.i = b_line_idx_2 - b_line_idx_0;
          rtB.in = rtB.b_line_idx_3 - b_line_idx_1;
          if ((rtB.i > 1073741824) || (rtB.i < -1073741824) || ((rtB.in >
                1073741824) || (rtB.in < -1073741824))) {
            // Possible Inf or Nan.
            done = true;
            isInBound = false;
            visited1 = true;
          } else if ((rtB.d_idxFillColor & 4U) != 0U) {
            // Violated RMin.
            rtB.d_idxFillColor = -b_line_idx_0 * rtB.in;
            if ((rtB.d_idxFillColor > 1073741824) || (rtB.d_idxFillColor <
                 -1073741824)) {
              // Check for Inf or Nan.
              done = true;
              isInBound = false;
            } else if (((rtB.d_idxFillColor >= 0) && (rtB.i >= 0)) ||
                       ((rtB.d_idxFillColor < 0) && (rtB.i < 0))) {
              b_line_idx_1 += (div_nzp_s32_floor(rtB.d_idxFillColor << 1, rtB.i)
                               + 1) >> 1;
            } else {
              b_line_idx_1 -= (div_nzp_s32_floor(-rtB.d_idxFillColor << 1, rtB.i)
                               + 1) >> 1;
            }

            b_line_idx_0 = 0;
            visited1 = true;
          } else if ((rtB.d_idxFillColor & 8U) != 0U) {
            // Violated RMax.
            rtB.d_idxFillColor = (539 - b_line_idx_0) * rtB.in;
            if ((rtB.d_idxFillColor > 1073741824) || (rtB.d_idxFillColor <
                 -1073741824)) {
              // Check for Inf or Nan.
              done = true;
              isInBound = false;
            } else if (((rtB.d_idxFillColor >= 0) && (rtB.i >= 0)) ||
                       ((rtB.d_idxFillColor < 0) && (rtB.i < 0))) {
              b_line_idx_1 += (div_nzp_s32_floor(rtB.d_idxFillColor << 1, rtB.i)
                               + 1) >> 1;
            } else {
              b_line_idx_1 -= (div_nzp_s32_floor(-rtB.d_idxFillColor << 1, rtB.i)
                               + 1) >> 1;
            }

            b_line_idx_0 = 539;
            visited1 = true;
          } else if ((rtB.d_idxFillColor & 1U) != 0U) {
            // Violated CMin.
            rtB.d_idxFillColor = -b_line_idx_1 * rtB.i;
            if ((rtB.d_idxFillColor > 1073741824) || (rtB.d_idxFillColor <
                 -1073741824)) {
              // Check for Inf or Nan.
              done = true;
              isInBound = false;
            } else if (((rtB.d_idxFillColor >= 0) && (rtB.in >= 0)) ||
                       ((rtB.d_idxFillColor < 0) && (rtB.in < 0))) {
              b_line_idx_0 += (div_nzp_s32_floor(rtB.d_idxFillColor << 1, rtB.in)
                               + 1) >> 1;
            } else {
              b_line_idx_0 -= (div_nzp_s32_floor(-rtB.d_idxFillColor << 1,
                rtB.in) + 1) >> 1;
            }

            b_line_idx_1 = 0;
            visited1 = true;
          } else {
            // Violated CMax.
            rtB.d_idxFillColor = (959 - b_line_idx_1) * rtB.i;
            if ((rtB.d_idxFillColor > 1073741824) || (rtB.d_idxFillColor <
                 -1073741824)) {
              // Check for Inf or Nan.
              done = true;
              isInBound = false;
            } else if (((rtB.d_idxFillColor >= 0) && (rtB.in >= 0)) ||
                       ((rtB.d_idxFillColor < 0) && (rtB.in < 0))) {
              b_line_idx_0 += (div_nzp_s32_floor(rtB.d_idxFillColor << 1, rtB.in)
                               + 1) >> 1;
            } else {
              b_line_idx_0 -= (div_nzp_s32_floor(-rtB.d_idxFillColor << 1,
                rtB.in) + 1) >> 1;
            }

            b_line_idx_1 = 959;
            visited1 = true;
          }
        } else {
          // Clip the 2nd point.
          if (visited2) {
            b_line_idx_2 = line_idx_2;
            rtB.b_line_idx_3 = rtB.lastCol;
          }

          rtB.i = b_line_idx_2 - b_line_idx_0;
          rtB.in = rtB.b_line_idx_3 - b_line_idx_1;
          if ((rtB.i > 1073741824) || (rtB.i < -1073741824) || ((rtB.in >
                1073741824) || (rtB.in < -1073741824))) {
            // Possible Inf or Nan.
            done = true;
            isInBound = false;
            visited2 = true;
          } else if ((rtB.h_idxPix & 4U) != 0U) {
            // Violated RMin.
            rtB.d_idxFillColor = -b_line_idx_2 * rtB.in;
            if ((rtB.d_idxFillColor > 1073741824) || (rtB.d_idxFillColor <
                 -1073741824)) {
              // Check for Inf or Nan.
              done = true;
              isInBound = false;
            } else if (((rtB.d_idxFillColor >= 0) && (rtB.i >= 0)) ||
                       ((rtB.d_idxFillColor < 0) && (rtB.i < 0))) {
              rtB.b_line_idx_3 += (div_nzp_s32_floor(rtB.d_idxFillColor << 1,
                rtB.i) + 1) >> 1;
            } else {
              rtB.b_line_idx_3 -= (div_nzp_s32_floor(-rtB.d_idxFillColor << 1,
                rtB.i) + 1) >> 1;
            }

            b_line_idx_2 = 0;
            visited2 = true;
          } else if ((rtB.h_idxPix & 8U) != 0U) {
            // Violated RMax.
            rtB.d_idxFillColor = (539 - b_line_idx_2) * rtB.in;
            if ((rtB.d_idxFillColor > 1073741824) || (rtB.d_idxFillColor <
                 -1073741824)) {
              // Check for Inf or Nan.
              done = true;
              isInBound = false;
            } else if (((rtB.d_idxFillColor >= 0) && (rtB.i >= 0)) ||
                       ((rtB.d_idxFillColor < 0) && (rtB.i < 0))) {
              rtB.b_line_idx_3 += (div_nzp_s32_floor(rtB.d_idxFillColor << 1,
                rtB.i) + 1) >> 1;
            } else {
              rtB.b_line_idx_3 -= (div_nzp_s32_floor(-rtB.d_idxFillColor << 1,
                rtB.i) + 1) >> 1;
            }

            b_line_idx_2 = 539;
            visited2 = true;
          } else if ((rtB.h_idxPix & 1U) != 0U) {
            // Violated CMin.
            rtB.d_idxFillColor = -rtB.b_line_idx_3 * rtB.i;
            if ((rtB.d_idxFillColor > 1073741824) || (rtB.d_idxFillColor <
                 -1073741824)) {
              // Check for Inf or Nan.
              done = true;
              isInBound = false;
            } else if (((rtB.d_idxFillColor >= 0) && (rtB.in >= 0)) ||
                       ((rtB.d_idxFillColor < 0) && (rtB.in < 0))) {
              b_line_idx_2 += (div_nzp_s32_floor(rtB.d_idxFillColor << 1, rtB.in)
                               + 1) >> 1;
            } else {
              b_line_idx_2 -= (div_nzp_s32_floor(-rtB.d_idxFillColor << 1,
                rtB.in) + 1) >> 1;
            }

            rtB.b_line_idx_3 = 0;
            visited2 = true;
          } else {
            // Violated CMax.
            rtB.d_idxFillColor = (959 - rtB.b_line_idx_3) * rtB.i;
            if ((rtB.d_idxFillColor > 1073741824) || (rtB.d_idxFillColor <
                 -1073741824)) {
              // Check for Inf or Nan.
              done = true;
              isInBound = false;
            } else if (((rtB.d_idxFillColor >= 0) && (rtB.in >= 0)) ||
                       ((rtB.d_idxFillColor < 0) && (rtB.in < 0))) {
              b_line_idx_2 += (div_nzp_s32_floor(rtB.d_idxFillColor << 1, rtB.in)
                               + 1) >> 1;
            } else {
              b_line_idx_2 -= (div_nzp_s32_floor(-rtB.d_idxFillColor << 1,
                rtB.in) + 1) >> 1;
            }

            rtB.b_line_idx_3 = 959;
            visited2 = true;
          }
        }
      }

      if (isInBound) {
        rtB.i = b_line_idx_1 * 540 + b_line_idx_0;
        rtB.in = b_line_idx_1;
        while (rtB.in <= rtB.b_line_idx_3) {
          varargin_1[rtB.i] = MAX_uint8_T;
          rtB.h_idxPix = rtB.i + 518400;
          varargin_1[rtB.h_idxPix] = MAX_uint8_T;
          rtB.h_idxPix += 518400;
          varargin_1[rtB.h_idxPix] = 0U;
          rtB.i += 540;
          rtB.in++;
        }
      }

      line_idx_0 = rtB.lastCol - rtB.halfLineWidth;
      line_idx_2 = rtB.lastCol - rtB.halfLineWidth;
      isInBound = false;

      // Find the visible portion of a line.
      visited1 = false;
      visited2 = false;
      done = false;
      b_line_idx_0 = rtB.firstRow;
      b_line_idx_1 = line_idx_0;
      b_line_idx_2 = rtB.lastRow;
      rtB.b_line_idx_3 = line_idx_2;
      while (!done) {
        rtB.d_idxFillColor = 0;
        rtB.h_idxPix = 0;

        // Determine viewport violations.
        if (b_line_idx_0 < 0) {
          rtB.d_idxFillColor = 4;
        } else {
          if (b_line_idx_0 > 539) {
            rtB.d_idxFillColor = 8;
          }
        }

        if (b_line_idx_2 < 0) {
          rtB.h_idxPix = 4;
        } else {
          if (b_line_idx_2 > 539) {
            rtB.h_idxPix = 8;
          }
        }

        if (b_line_idx_1 < 0) {
          rtB.d_idxFillColor |= 1U;
        } else {
          if (b_line_idx_1 > 959) {
            rtB.d_idxFillColor |= 2U;
          }
        }

        if (rtB.b_line_idx_3 < 0) {
          rtB.h_idxPix |= 1U;
        } else {
          if (rtB.b_line_idx_3 > 959) {
            rtB.h_idxPix |= 2U;
          }
        }

        if (!(((uint32_T)rtB.d_idxFillColor | rtB.h_idxPix) != 0U)) {
          // Line falls completely within bounds.
          done = true;
          isInBound = true;
        } else if (((uint32_T)rtB.d_idxFillColor & rtB.h_idxPix) != 0U) {
          // Line falls completely out of bounds.
          done = true;
          isInBound = false;
        } else if ((uint32_T)rtB.d_idxFillColor != 0U) {
          // Clip 1st point; if it's in-bounds, clip 2nd point.
          if (visited1) {
            b_line_idx_0 = rtB.firstRow;
            b_line_idx_1 = line_idx_0;
          }

          rtB.i = b_line_idx_2 - b_line_idx_0;
          rtB.in = rtB.b_line_idx_3 - b_line_idx_1;
          if ((rtB.i > 1073741824) || (rtB.i < -1073741824) || ((rtB.in >
                1073741824) || (rtB.in < -1073741824))) {
            // Possible Inf or Nan.
            done = true;
            isInBound = false;
            visited1 = true;
          } else if ((rtB.d_idxFillColor & 4U) != 0U) {
            // Violated RMin.
            rtB.d_idxFillColor = -b_line_idx_0 * rtB.in;
            if ((rtB.d_idxFillColor > 1073741824) || (rtB.d_idxFillColor <
                 -1073741824)) {
              // Check for Inf or Nan.
              done = true;
              isInBound = false;
            } else if (((rtB.d_idxFillColor >= 0) && (rtB.i >= 0)) ||
                       ((rtB.d_idxFillColor < 0) && (rtB.i < 0))) {
              b_line_idx_1 += (div_nzp_s32_floor(rtB.d_idxFillColor << 1, rtB.i)
                               + 1) >> 1;
            } else {
              b_line_idx_1 -= (div_nzp_s32_floor(-rtB.d_idxFillColor << 1, rtB.i)
                               + 1) >> 1;
            }

            b_line_idx_0 = 0;
            visited1 = true;
          } else if ((rtB.d_idxFillColor & 8U) != 0U) {
            // Violated RMax.
            rtB.d_idxFillColor = (539 - b_line_idx_0) * rtB.in;
            if ((rtB.d_idxFillColor > 1073741824) || (rtB.d_idxFillColor <
                 -1073741824)) {
              // Check for Inf or Nan.
              done = true;
              isInBound = false;
            } else if (((rtB.d_idxFillColor >= 0) && (rtB.i >= 0)) ||
                       ((rtB.d_idxFillColor < 0) && (rtB.i < 0))) {
              b_line_idx_1 += (div_nzp_s32_floor(rtB.d_idxFillColor << 1, rtB.i)
                               + 1) >> 1;
            } else {
              b_line_idx_1 -= (div_nzp_s32_floor(-rtB.d_idxFillColor << 1, rtB.i)
                               + 1) >> 1;
            }

            b_line_idx_0 = 539;
            visited1 = true;
          } else if ((rtB.d_idxFillColor & 1U) != 0U) {
            // Violated CMin.
            rtB.d_idxFillColor = -b_line_idx_1 * rtB.i;
            if ((rtB.d_idxFillColor > 1073741824) || (rtB.d_idxFillColor <
                 -1073741824)) {
              // Check for Inf or Nan.
              done = true;
              isInBound = false;
            } else if (((rtB.d_idxFillColor >= 0) && (rtB.in >= 0)) ||
                       ((rtB.d_idxFillColor < 0) && (rtB.in < 0))) {
              b_line_idx_0 += (div_nzp_s32_floor(rtB.d_idxFillColor << 1, rtB.in)
                               + 1) >> 1;
            } else {
              b_line_idx_0 -= (div_nzp_s32_floor(-rtB.d_idxFillColor << 1,
                rtB.in) + 1) >> 1;
            }

            b_line_idx_1 = 0;
            visited1 = true;
          } else {
            // Violated CMax.
            rtB.d_idxFillColor = (959 - b_line_idx_1) * rtB.i;
            if ((rtB.d_idxFillColor > 1073741824) || (rtB.d_idxFillColor <
                 -1073741824)) {
              // Check for Inf or Nan.
              done = true;
              isInBound = false;
            } else if (((rtB.d_idxFillColor >= 0) && (rtB.in >= 0)) ||
                       ((rtB.d_idxFillColor < 0) && (rtB.in < 0))) {
              b_line_idx_0 += (div_nzp_s32_floor(rtB.d_idxFillColor << 1, rtB.in)
                               + 1) >> 1;
            } else {
              b_line_idx_0 -= (div_nzp_s32_floor(-rtB.d_idxFillColor << 1,
                rtB.in) + 1) >> 1;
            }

            b_line_idx_1 = 959;
            visited1 = true;
          }
        } else {
          // Clip the 2nd point.
          if (visited2) {
            b_line_idx_2 = rtB.lastRow;
            rtB.b_line_idx_3 = line_idx_2;
          }

          rtB.i = b_line_idx_2 - b_line_idx_0;
          rtB.in = rtB.b_line_idx_3 - b_line_idx_1;
          if ((rtB.i > 1073741824) || (rtB.i < -1073741824) || ((rtB.in >
                1073741824) || (rtB.in < -1073741824))) {
            // Possible Inf or Nan.
            done = true;
            isInBound = false;
            visited2 = true;
          } else if ((rtB.h_idxPix & 4U) != 0U) {
            // Violated RMin.
            rtB.d_idxFillColor = -b_line_idx_2 * rtB.in;
            if ((rtB.d_idxFillColor > 1073741824) || (rtB.d_idxFillColor <
                 -1073741824)) {
              // Check for Inf or Nan.
              done = true;
              isInBound = false;
            } else if (((rtB.d_idxFillColor >= 0) && (rtB.i >= 0)) ||
                       ((rtB.d_idxFillColor < 0) && (rtB.i < 0))) {
              rtB.b_line_idx_3 += (div_nzp_s32_floor(rtB.d_idxFillColor << 1,
                rtB.i) + 1) >> 1;
            } else {
              rtB.b_line_idx_3 -= (div_nzp_s32_floor(-rtB.d_idxFillColor << 1,
                rtB.i) + 1) >> 1;
            }

            b_line_idx_2 = 0;
            visited2 = true;
          } else if ((rtB.h_idxPix & 8U) != 0U) {
            // Violated RMax.
            rtB.d_idxFillColor = (539 - b_line_idx_2) * rtB.in;
            if ((rtB.d_idxFillColor > 1073741824) || (rtB.d_idxFillColor <
                 -1073741824)) {
              // Check for Inf or Nan.
              done = true;
              isInBound = false;
            } else if (((rtB.d_idxFillColor >= 0) && (rtB.i >= 0)) ||
                       ((rtB.d_idxFillColor < 0) && (rtB.i < 0))) {
              rtB.b_line_idx_3 += (div_nzp_s32_floor(rtB.d_idxFillColor << 1,
                rtB.i) + 1) >> 1;
            } else {
              rtB.b_line_idx_3 -= (div_nzp_s32_floor(-rtB.d_idxFillColor << 1,
                rtB.i) + 1) >> 1;
            }

            b_line_idx_2 = 539;
            visited2 = true;
          } else if ((rtB.h_idxPix & 1U) != 0U) {
            // Violated CMin.
            rtB.d_idxFillColor = -rtB.b_line_idx_3 * rtB.i;
            if ((rtB.d_idxFillColor > 1073741824) || (rtB.d_idxFillColor <
                 -1073741824)) {
              // Check for Inf or Nan.
              done = true;
              isInBound = false;
            } else if (((rtB.d_idxFillColor >= 0) && (rtB.in >= 0)) ||
                       ((rtB.d_idxFillColor < 0) && (rtB.in < 0))) {
              b_line_idx_2 += (div_nzp_s32_floor(rtB.d_idxFillColor << 1, rtB.in)
                               + 1) >> 1;
            } else {
              b_line_idx_2 -= (div_nzp_s32_floor(-rtB.d_idxFillColor << 1,
                rtB.in) + 1) >> 1;
            }

            rtB.b_line_idx_3 = 0;
            visited2 = true;
          } else {
            // Violated CMax.
            rtB.d_idxFillColor = (959 - rtB.b_line_idx_3) * rtB.i;
            if ((rtB.d_idxFillColor > 1073741824) || (rtB.d_idxFillColor <
                 -1073741824)) {
              // Check for Inf or Nan.
              done = true;
              isInBound = false;
            } else if (((rtB.d_idxFillColor >= 0) && (rtB.in >= 0)) ||
                       ((rtB.d_idxFillColor < 0) && (rtB.in < 0))) {
              b_line_idx_2 += (div_nzp_s32_floor(rtB.d_idxFillColor << 1, rtB.in)
                               + 1) >> 1;
            } else {
              b_line_idx_2 -= (div_nzp_s32_floor(-rtB.d_idxFillColor << 1,
                rtB.in) + 1) >> 1;
            }

            rtB.b_line_idx_3 = 959;
            visited2 = true;
          }
        }
      }

      if (isInBound) {
        rtB.i = b_line_idx_1 * 540 + b_line_idx_0;
        rtB.in = b_line_idx_0;
        while (rtB.in <= b_line_idx_2) {
          varargin_1[rtB.i] = MAX_uint8_T;
          rtB.h_idxPix = rtB.i + 518400;
          varargin_1[rtB.h_idxPix] = MAX_uint8_T;
          rtB.h_idxPix += 518400;
          varargin_1[rtB.h_idxPix] = 0U;
          rtB.i++;
          rtB.in++;
        }
      }

      rtB.halfLineWidth++;
    }
  }
}

// Function for MATLAB Function: '<S12>/MATLAB Function2'
static void twister_state_vector(uint32_T mt[625], uint32_T seed)
{
  uint32_T r;
  int32_T mti;
  r = seed;
  mt[0] = seed;
  for (mti = 0; mti < 623; mti++) {
    r = ((r >> 30U ^ r) * 1812433253U + mti) + 1U;
    mt[mti + 1] = r;
  }

  mt[624] = 624U;
}

// Function for MATLAB Function: '<S12>/MATLAB Function2'
static real_T eml_rand_mt19937ar(uint32_T state[625])
{
  real_T r;
  uint32_T mti;
  uint32_T y;
  int32_T kk;
  int32_T k;
  boolean_T b_isvalid;
  int32_T exitg1;
  boolean_T exitg2;

  // ========================= COPYRIGHT NOTICE ============================
  //  This is a uniform (0,1) pseudorandom number generator based on:
  //
  //  A C-program for MT19937, with initialization improved 2002/1/26.
  //  Coded by Takuji Nishimura and Makoto Matsumoto.
  //
  //  Copyright (C) 1997 - 2002, Makoto Matsumoto and Takuji Nishimura,
  //  All rights reserved.
  //
  //  Redistribution and use in source and binary forms, with or without
  //  modification, are permitted provided that the following conditions
  //  are met:
  //
  //    1. Redistributions of source code must retain the above copyright
  //       notice, this list of conditions and the following disclaimer.
  //
  //    2. Redistributions in binary form must reproduce the above copyright
  //       notice, this list of conditions and the following disclaimer
  //       in the documentation and/or other materials provided with the
  //       distribution.
  //
  //    3. The names of its contributors may not be used to endorse or
  //       promote products derived from this software without specific
  //       prior written permission.
  //
  //  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
  //  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
  //  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
  //  A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT
  //  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
  //  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  //  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
  //  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
  //  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
  //  (INCLUDING  NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  //  OF THIS  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  //
  // =============================   END   =================================
  do {
    exitg1 = 0;
    for (k = 0; k < 2; k++) {
      mti = state[624] + 1U;
      if (mti >= 625U) {
        for (kk = 0; kk < 227; kk++) {
          y = (state[kk + 1] & 2147483647U) | (state[kk] & 2147483648U);
          if ((int32_T)(y & 1U) == 0) {
            y >>= 1U;
          } else {
            y = y >> 1U ^ 2567483615U;
          }

          state[kk] = state[kk + 397] ^ y;
        }

        for (kk = 0; kk < 396; kk++) {
          y = (state[kk + 227] & 2147483648U) | (state[kk + 228] & 2147483647U);
          if ((int32_T)(y & 1U) == 0) {
            y >>= 1U;
          } else {
            y = y >> 1U ^ 2567483615U;
          }

          state[kk + 227] = state[kk] ^ y;
        }

        y = (state[623] & 2147483648U) | (state[0] & 2147483647U);
        if ((int32_T)(y & 1U) == 0) {
          y >>= 1U;
        } else {
          y = y >> 1U ^ 2567483615U;
        }

        state[623] = state[396] ^ y;
        mti = 1U;
      }

      y = state[(int32_T)mti - 1];
      state[624] = mti;
      y ^= y >> 11U;
      y ^= y << 7U & 2636928640U;
      y ^= y << 15U & 4022730752U;
      y ^= y >> 18U;
      rtB.u_c[k] = y;
    }

    r = ((real_T)(rtB.u_c[0] >> 5U) * 6.7108864E+7 + (real_T)(rtB.u_c[1] >> 6U))
      * 1.1102230246251565E-16;
    if (r == 0.0) {
      b_isvalid = ((state[624] >= 1U) && (state[624] < 625U));
      if (b_isvalid) {
        b_isvalid = false;
        k = 1;
        exitg2 = false;
        while ((!exitg2) && (k < 625)) {
          if (state[k - 1] == 0U) {
            k++;
          } else {
            b_isvalid = true;
            exitg2 = true;
          }
        }
      }

      if (!b_isvalid) {
        twister_state_vector(state, 5489U);
      }
    } else {
      exitg1 = 1;
    }
  } while (exitg1 == 0);

  return r;
}

// Function for MATLAB Function: '<S12>/MATLAB Function2'
static real_T rand_e(void)
{
  real_T r;
  uint32_T test1;
  uint32_T test2;
  switch (rtDW.method_k) {
   case 4U:
    rtB.hi = (int32_T)(rtDW.state_l / 127773U);
    test1 = (rtDW.state_l - rtB.hi * 127773U) * 16807U;
    test2 = 2836U * rtB.hi;
    if (test1 < test2) {
      test1 = (test1 - test2) + 2147483647U;
    } else {
      test1 -= test2;
    }

    rtDW.state_l = test1;
    r = (real_T)test1 * 4.6566128752457969E-10;
    break;

   case 5U:
    test1 = 69069U * rtDW.state_j[0] + 1234567U;
    test2 = rtDW.state_j[1] << 13 ^ rtDW.state_j[1];
    test2 ^= test2 >> 17;
    test2 ^= test2 << 5;
    rtDW.state_j[0] = test1;
    rtDW.state_j[1] = test2;
    r = (real_T)(test1 + test2) * 2.328306436538696E-10;
    break;

   default:
    if (!rtDW.state_not_empty_o) {
      memset(&rtDW.state_b[0], 0, 625U * sizeof(uint32_T));
      twister_state_vector(rtDW.state_b, 5489U);
      rtDW.state_not_empty_o = true;
    }

    r = eml_rand_mt19937ar(rtDW.state_b);
    break;
  }

  return r;
}

// Function for MATLAB Function: '<S13>/MATLAB Function2'
static real_T sum_j(const boolean_T x[4])
{
  return (((real_T)x[0] + (real_T)x[1]) + (real_T)x[2]) + (real_T)x[3];
}

// Function for MATLAB Function: '<S13>/MATLAB Function2'
static real_T rand_oi(void)
{
  real_T r;
  uint32_T d;
  uint32_T state;
  switch (rtDW.method) {
   case 4U:
    state = rtDW.state;
    r = eml_rand_mcg16807(&state);
    rtDW.state = state;
    break;

   case 5U:
    state = 69069U * rtDW.state_a[0] + 1234567U;
    d = rtDW.state_a[1] << 13 ^ rtDW.state_a[1];
    d ^= d >> 17;
    d ^= d << 5;
    rtDW.state_a[0] = state;
    rtDW.state_a[1] = d;
    r = (real_T)(state + d) * 2.328306436538696E-10;
    break;

   default:
    if (!rtDW.state_not_empty) {
      memset(&rtDW.state_i[0], 0, 625U * sizeof(uint32_T));
      twister_state_vector_g(rtDW.state_i, 5489U);
      rtDW.state_not_empty = true;
    }

    r = eml_rand_mt19937ar_g(rtDW.state_i);
    break;
  }

  return r;
}

//
// Function for MATLAB Function: '<S13>/MATLAB Function2'
// function value = eval_at_p(point, width, A)
//
static real_T eval_at_p_b(const real_T point[2], real_T width, const real_T A
  [519901])
{
  //  the eval() function
  // '<S45>:1:67' p_tl = ceil(point);
  // '<S45>:1:68' p_tr = ceil(point + [0, width]);
  // '<S45>:1:69' p_bl = ceil(point + [width, 0]);
  // '<S45>:1:70' p_br = ceil(point + [width, width]);
  // '<S45>:1:72' value = A(p_tl(1), p_tl(2)) + A(p_br(1),p_br(2))...
  // '<S45>:1:73'     - A(p_tr(1), p_tr(2)) - A(p_bl(1),p_bl(2));
  return ((A[(((int32_T)ceil(point[1] + width) - 1) * 541 + (int32_T)ceil(point
             [0] + width)) - 1] + A[(((int32_T)ceil(point[1]) - 1) * 541 +
            (int32_T)ceil(point[0])) - 1]) - A[(((int32_T)ceil(point[1] + width)
            - 1) * 541 + (int32_T)ceil(point[0])) - 1]) - A[(((int32_T)ceil
    (point[1]) - 1) * 541 + (int32_T)ceil(point[0] + width)) - 1];
}

// Function for MATLAB Function: '<S13>/MATLAB Function2'
static real_T eml_rand_shr3cong(uint32_T state[2])
{
  real_T r;
  uint32_T icng;
  uint32_T jsr;
  int32_T j;
  real_T s;
  uint32_T ui;
  static const real_T b[65] = { 0.340945, 0.4573146, 0.5397793, 0.6062427,
    0.6631691, 0.7136975, 0.7596125, 0.8020356, 0.8417227, 0.8792102, 0.9148948,
    0.9490791, 0.9820005, 1.0138492, 1.044781, 1.0749254, 1.1043917, 1.1332738,
    1.161653, 1.189601, 1.2171815, 1.2444516, 1.2714635, 1.298265, 1.3249008,
    1.3514125, 1.3778399, 1.4042211, 1.4305929, 1.4569915, 1.4834527, 1.5100122,
    1.5367061, 1.5635712, 1.5906454, 1.617968, 1.6455802, 1.6735255, 1.7018503,
    1.7306045, 1.7598422, 1.7896223, 1.8200099, 1.851077, 1.8829044, 1.9155831,
    1.9492166, 1.9839239, 2.0198431, 2.0571356, 2.095993, 2.136645, 2.1793713,
    2.2245175, 2.2725186, 2.3239338, 2.3795008, 2.4402218, 2.5075117, 2.5834658,
    2.6713916, 2.7769942, 2.7769942, 2.7769942, 2.7769942 };

  icng = 69069U * state[0] + 1234567U;
  jsr = state[1] << 13 ^ state[1];
  jsr ^= jsr >> 17;
  jsr ^= jsr << 5;
  ui = icng + jsr;
  j = (int32_T)((ui & 63U) + 1U);
  r = (real_T)(int32_T)ui * 4.6566128730773926E-10 * b[j];
  if (!(fabs(r) <= b[j - 1])) {
    rtB.x_c = (fabs(r) - b[j - 1]) / (b[j] - b[j - 1]);
    icng = 69069U * icng + 1234567U;
    jsr ^= jsr << 13;
    jsr ^= jsr >> 17;
    jsr ^= jsr << 5;
    rtB.y_b = (real_T)(int32_T)(icng + jsr) * 2.328306436538696E-10 + 0.5;
    s = rtB.x_c + rtB.y_b;
    if (s > 1.301198) {
      if (r < 0.0) {
        r = 0.4878992 * rtB.x_c - 0.4878992;
      } else {
        r = 0.4878992 - 0.4878992 * rtB.x_c;
      }
    } else {
      if (!(s <= 0.9689279)) {
        rtB.x_c = 0.4878992 - 0.4878992 * rtB.x_c;
        if (rtB.y_b > 12.67706 - exp(-0.5 * rtB.x_c * rtB.x_c) * 12.37586) {
          if (r < 0.0) {
            r = -rtB.x_c;
          } else {
            r = rtB.x_c;
          }
        } else {
          if (!(exp(-0.5 * b[j] * b[j]) + rtB.y_b * 0.01958303 / b[j] <= exp
                (-0.5 * r * r))) {
            do {
              icng = 69069U * icng + 1234567U;
              jsr ^= jsr << 13;
              jsr ^= jsr >> 17;
              jsr ^= jsr << 5;
              rtB.x_c = log((real_T)(int32_T)(icng + jsr) *
                            2.328306436538696E-10 + 0.5) / 2.776994;
              icng = 69069U * icng + 1234567U;
              jsr ^= jsr << 13;
              jsr ^= jsr >> 17;
              jsr ^= jsr << 5;
            } while (!(log((real_T)(int32_T)(icng + jsr) * 2.328306436538696E-10
                           + 0.5) * -2.0 > rtB.x_c * rtB.x_c));

            if (r < 0.0) {
              r = rtB.x_c - 2.776994;
            } else {
              r = 2.776994 - rtB.x_c;
            }
          }
        }
      }
    }
  }

  state[0] = icng;
  state[1] = jsr;
  return r;
}

// Function for MATLAB Function: '<S13>/MATLAB Function2'
static void genrandu(uint32_T s, uint32_T *state, real_T *r)
{
  int32_T hi;
  uint32_T test1;
  uint32_T test2;
  hi = (int32_T)(s / 127773U);
  test1 = (s - hi * 127773U) * 16807U;
  test2 = 2836U * hi;
  if (test1 < test2) {
    *state = (test1 - test2) + 2147483647U;
  } else {
    *state = test1 - test2;
  }

  *r = (real_T)*state * 4.6566128752457969E-10;
}

// Function for MATLAB Function: '<S13>/MATLAB Function2'
static real_T genrandu_h(uint32_T mt[625])
{
  real_T r;
  int32_T exitg1;

  // ========================= COPYRIGHT NOTICE ============================
  //  This is a uniform (0,1) pseudorandom number generator based on:
  //
  //  A C-program for MT19937, with initialization improved 2002/1/26.
  //  Coded by Takuji Nishimura and Makoto Matsumoto.
  //
  //  Copyright (C) 1997 - 2002, Makoto Matsumoto and Takuji Nishimura,
  //  All rights reserved.
  //
  //  Redistribution and use in source and binary forms, with or without
  //  modification, are permitted provided that the following conditions
  //  are met:
  //
  //    1. Redistributions of source code must retain the above copyright
  //       notice, this list of conditions and the following disclaimer.
  //
  //    2. Redistributions in binary form must reproduce the above copyright
  //       notice, this list of conditions and the following disclaimer
  //       in the documentation and/or other materials provided with the
  //       distribution.
  //
  //    3. The names of its contributors may not be used to endorse or
  //       promote products derived from this software without specific
  //       prior written permission.
  //
  //  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
  //  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
  //  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
  //  A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT
  //  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
  //  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  //  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
  //  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
  //  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
  //  (INCLUDING  NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  //  OF THIS  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  //
  // =============================   END   =================================
  do {
    exitg1 = 0;
    genrand_uint32_vector(mt, rtB.u_p);
    r = ((real_T)(rtB.u_p[0] >> 5U) * 6.7108864E+7 + (real_T)(rtB.u_p[1] >> 6U))
      * 1.1102230246251565E-16;
    if (r == 0.0) {
      if (!is_valid_state(mt)) {
        twister_state_vector_g(mt, 5489U);
      }
    } else {
      exitg1 = 1;
    }
  } while (exitg1 == 0);

  return r;
}

// Function for MATLAB Function: '<S13>/MATLAB Function2'
static real_T eml_rand_mt19937ar_g2(uint32_T state[625])
{
  real_T r;
  int32_T i;
  static const real_T b[257] = { 0.0, 0.215241895984875, 0.286174591792068,
    0.335737519214422, 0.375121332878378, 0.408389134611989, 0.43751840220787,
    0.46363433679088, 0.487443966139235, 0.50942332960209, 0.529909720661557,
    0.549151702327164, 0.567338257053817, 0.584616766106378, 0.601104617755991,
    0.61689699000775, 0.63207223638606, 0.646695714894993, 0.660822574244419,
    0.674499822837293, 0.687767892795788, 0.700661841106814, 0.713212285190975,
    0.725446140909999, 0.737387211434295, 0.749056662017815, 0.760473406430107,
    0.771654424224568, 0.782615023307232, 0.793369058840623, 0.80392911698997,
    0.814306670135215, 0.824512208752291, 0.834555354086381, 0.844444954909153,
    0.854189171008163, 0.863795545553308, 0.87327106808886, 0.882622229585165,
    0.891855070732941, 0.900975224461221, 0.909987953496718, 0.91889818364959,
    0.927710533401999, 0.936429340286575, 0.945058684468165, 0.953602409881086,
    0.96206414322304, 0.970447311064224, 0.978755155294224, 0.986990747099062,
    0.99515699963509, 1.00325667954467, 1.01129241744, 1.01926671746548,
    1.02718196603564, 1.03504043983344, 1.04284431314415, 1.05059566459093,
    1.05829648333067, 1.06594867476212, 1.07355406579244, 1.0811144097034,
    1.08863139065398, 1.09610662785202, 1.10354167942464, 1.11093804601357,
    1.11829717411934, 1.12562045921553, 1.13290924865253, 1.14016484436815,
    1.14738850542085, 1.15458145035993, 1.16174485944561, 1.16887987673083,
    1.17598761201545, 1.18306914268269, 1.19012551542669, 1.19715774787944,
    1.20416683014438, 1.2111537262437, 1.21811937548548, 1.22506469375653,
    1.23199057474614, 1.23889789110569, 1.24578749554863, 1.2526602218949,
    1.25951688606371, 1.26635828701823, 1.27318520766536, 1.27999841571382,
    1.28679866449324, 1.29358669373695, 1.30036323033084, 1.30712898903073,
    1.31388467315022, 1.32063097522106, 1.32736857762793, 1.33409815321936,
    1.3408203658964, 1.34753587118059, 1.35424531676263, 1.36094934303328,
    1.36764858359748, 1.37434366577317, 1.38103521107586, 1.38772383568998,
    1.39441015092814, 1.40109476367925, 1.4077782768464, 1.41446128977547,
    1.42114439867531, 1.42782819703026, 1.43451327600589, 1.44120022484872,
    1.44788963128058, 1.45458208188841, 1.46127816251028, 1.46797845861808,
    1.47468355569786, 1.48139403962819, 1.48811049705745, 1.49483351578049,
    1.50156368511546, 1.50830159628131, 1.51504784277671, 1.521803020761,
    1.52856772943771, 1.53534257144151, 1.542128153229, 1.54892508547417,
    1.55573398346918, 1.56255546753104, 1.56939016341512, 1.57623870273591,
    1.58310172339603, 1.58997987002419, 1.59687379442279, 1.60378415602609,
    1.61071162236983, 1.61765686957301, 1.62462058283303, 1.63160345693487,
    1.63860619677555, 1.64562951790478, 1.65267414708306, 1.65974082285818,
    1.66683029616166, 1.67394333092612, 1.68108070472517, 1.68824320943719,
    1.69543165193456, 1.70264685479992, 1.7098896570713, 1.71716091501782,
    1.72446150294804, 1.73179231405296, 1.73915426128591, 1.74654827828172,
    1.75397532031767, 1.76143636531891, 1.76893241491127, 1.77646449552452,
    1.78403365954944, 1.79164098655216, 1.79928758454972, 1.80697459135082,
    1.81470317596628, 1.82247454009388, 1.83028991968276, 1.83815058658281,
    1.84605785028518, 1.8540130597602, 1.86201760539967, 1.87007292107127,
    1.878180486293, 1.88634182853678, 1.8945585256707, 1.90283220855043,
    1.91116456377125, 1.91955733659319, 1.92801233405266, 1.93653142827569,
    1.94511656000868, 1.95376974238465, 1.96249306494436, 1.97128869793366,
    1.98015889690048, 1.98910600761744, 1.99813247135842, 2.00724083056053,
    2.0164337349062, 2.02571394786385, 2.03508435372962, 2.04454796521753,
    2.05410793165065, 2.06376754781173, 2.07353026351874, 2.0833996939983,
    2.09337963113879, 2.10347405571488, 2.11368715068665, 2.12402331568952,
    2.13448718284602, 2.14508363404789, 2.15581781987674, 2.16669518035431,
    2.17772146774029, 2.18890277162636, 2.20024554661128, 2.21175664288416,
    2.22344334009251, 2.23531338492992, 2.24737503294739, 2.25963709517379,
    2.27210899022838, 2.28480080272449, 2.29772334890286, 2.31088825060137,
    2.32430801887113, 2.33799614879653, 2.35196722737914, 2.36623705671729,
    2.38082279517208, 2.39574311978193, 2.41101841390112, 2.42667098493715,
    2.44272531820036, 2.4592083743347, 2.47614993967052, 2.49358304127105,
    2.51154444162669, 2.53007523215985, 2.54922155032478, 2.56903545268184,
    2.58957598670829, 2.61091051848882, 2.63311639363158, 2.65628303757674,
    2.68051464328574, 2.70593365612306, 2.73268535904401, 2.76094400527999,
    2.79092117400193, 2.82287739682644, 2.85713873087322, 2.89412105361341,
    2.93436686720889, 2.97860327988184, 3.02783779176959, 3.08352613200214,
    3.147889289518, 3.2245750520478, 3.32024473383983, 3.44927829856143,
    3.65415288536101, 3.91075795952492 };

  static const real_T c[257] = { 1.0, 0.977101701267673, 0.959879091800108,
    0.9451989534423, 0.932060075959231, 0.919991505039348, 0.908726440052131,
    0.898095921898344, 0.887984660755834, 0.878309655808918, 0.869008688036857,
    0.860033621196332, 0.851346258458678, 0.842915653112205, 0.834716292986884,
    0.826726833946222, 0.818929191603703, 0.811307874312656, 0.803849483170964,
    0.796542330422959, 0.789376143566025, 0.782341832654803, 0.775431304981187,
    0.768637315798486, 0.761953346836795, 0.755373506507096, 0.748892447219157,
    0.742505296340151, 0.736207598126863, 0.729995264561476, 0.72386453346863,
    0.717811932630722, 0.711834248878248, 0.705928501332754, 0.700091918136512,
    0.694321916126117, 0.688616083004672, 0.682972161644995, 0.677388036218774,
    0.671861719897082, 0.66639134390875, 0.660975147776663, 0.655611470579697,
    0.650298743110817, 0.645035480820822, 0.639820277453057, 0.634651799287624,
    0.629528779924837, 0.624450015547027, 0.619414360605834, 0.614420723888914,
    0.609468064925773, 0.604555390697468, 0.599681752619125, 0.594846243767987,
    0.590047996332826, 0.585286179263371, 0.580559996100791, 0.575868682972354,
    0.571211506735253, 0.566587763256165, 0.561996775814525, 0.557437893618766,
    0.552910490425833, 0.548413963255266, 0.543947731190026, 0.539511234256952,
    0.535103932380458, 0.530725304403662, 0.526374847171684, 0.522052074672322,
    0.517756517229756, 0.513487720747327, 0.509245245995748, 0.505028667943468,
    0.500837575126149, 0.49667156905249, 0.492530263643869, 0.488413284705458,
    0.484320269426683, 0.480250865909047, 0.476204732719506, 0.47218153846773,
    0.468180961405694, 0.464202689048174, 0.460246417812843, 0.456311852678716,
    0.452398706861849, 0.448506701507203, 0.444635565395739, 0.440785034665804,
    0.436954852547985, 0.433144769112652, 0.429354541029442, 0.425583931338022,
    0.421832709229496, 0.418100649837848, 0.414387534040891, 0.410693148270188,
    0.407017284329473, 0.403359739221114, 0.399720314980197, 0.396098818515832,
    0.392495061459315, 0.388908860018789, 0.385340034840077, 0.381788410873393,
    0.378253817245619, 0.374736087137891, 0.371235057668239, 0.367750569779032,
    0.364282468129004, 0.360830600989648, 0.357394820145781, 0.353974980800077,
    0.350570941481406, 0.347182563956794, 0.343809713146851, 0.340452257044522,
    0.337110066637006, 0.333783015830718, 0.330470981379163, 0.327173842813601,
    0.323891482376391, 0.320623784956905, 0.317370638029914, 0.314131931596337,
    0.310907558126286, 0.307697412504292, 0.30450139197665, 0.301319396100803,
    0.298151326696685, 0.294997087799962, 0.291856585617095, 0.288729728482183,
    0.285616426815502, 0.282516593083708, 0.279430141761638, 0.276356989295668,
    0.273297054068577, 0.270250256365875, 0.267216518343561, 0.264195763997261,
    0.261187919132721, 0.258192911337619, 0.255210669954662, 0.252241126055942,
    0.249284212418529, 0.246339863501264, 0.24340801542275, 0.240488605940501,
    0.237581574431238, 0.23468686187233, 0.231804410824339, 0.228934165414681,
    0.226076071322381, 0.223230075763918, 0.220396127480152, 0.217574176724331,
    0.214764175251174, 0.211966076307031, 0.209179834621125, 0.206405406397881,
    0.203642749310335, 0.200891822494657, 0.198152586545776, 0.195425003514135,
    0.192709036903589, 0.190004651670465, 0.187311814223801, 0.1846304924268,
    0.181960655599523, 0.179302274522848, 0.176655321443735, 0.174019770081839,
    0.171395595637506, 0.168782774801212, 0.166181285764482, 0.163591108232366,
    0.161012223437511, 0.158444614155925, 0.15588826472448, 0.153343161060263,
    0.150809290681846, 0.148286642732575, 0.145775208005994, 0.143274978973514,
    0.140785949814445, 0.138308116448551, 0.135841476571254, 0.133386029691669,
    0.130941777173644, 0.12850872228, 0.126086870220186, 0.123676228201597,
    0.12127680548479, 0.11888861344291, 0.116511665625611, 0.114145977827839,
    0.111791568163838, 0.109448457146812, 0.107116667774684, 0.104796225622487,
    0.102487158941935, 0.10018949876881, 0.0979032790388625, 0.095628536713009,
    0.093365311912691, 0.0911136480663738, 0.0888735920682759,
    0.0866451944505581, 0.0844285095703535, 0.082223595813203,
    0.0800305158146631, 0.0778493367020961, 0.0756801303589272,
    0.0735229737139814, 0.0713779490588905, 0.0692451443970068,
    0.0671246538277886, 0.065016577971243, 0.0629210244377582, 0.06083810834954,
    0.0587679529209339, 0.0567106901062031, 0.0546664613248891,
    0.0526354182767924, 0.0506177238609479, 0.0486135532158687,
    0.0466230949019305, 0.0446465522512946, 0.0426841449164746,
    0.0407361106559411, 0.0388027074045262, 0.0368842156885674,
    0.0349809414617162, 0.0330932194585786, 0.0312214171919203,
    0.0293659397581334, 0.0275272356696031, 0.0257058040085489,
    0.0239022033057959, 0.0221170627073089, 0.0203510962300445,
    0.0186051212757247, 0.0168800831525432, 0.0151770883079353,
    0.0134974506017399, 0.0118427578579079, 0.0102149714397015,
    0.00861658276939875, 0.00705087547137324, 0.00552240329925101,
    0.00403797259336304, 0.00260907274610216, 0.0012602859304986,
    0.000477467764609386 };

  int32_T exitg1;
  do {
    exitg1 = 0;
    genrand_uint32_vector(state, rtB.u32);
    i = (int32_T)((rtB.u32[1] >> 24U) + 1U);
    r = (((real_T)(rtB.u32[0] >> 3U) * 1.6777216E+7 + (real_T)((int32_T)rtB.u32
           [1] & 16777215)) * 2.2204460492503131E-16 - 1.0) * b[i];
    if (fabs(r) <= b[i - 1]) {
      exitg1 = 1;
    } else if (i < 256) {
      rtB.x_k = genrandu_h(state);
      if ((c[i - 1] - c[i]) * rtB.x_k + c[i] < exp(-0.5 * r * r)) {
        exitg1 = 1;
      }
    } else {
      do {
        rtB.x_k = genrandu_h(state);
        rtB.x_k = log(rtB.x_k) * 0.273661237329758;
        rtB.c_u = genrandu_h(state);
      } while (!(-2.0 * log(rtB.c_u) > rtB.x_k * rtB.x_k));

      if (r < 0.0) {
        r = rtB.x_k - 3.65415288536101;
      } else {
        r = 3.65415288536101 - rtB.x_k;
      }

      exitg1 = 1;
    }
  } while (exitg1 == 0);

  return r;
}

// Function for MATLAB Function: '<S13>/MATLAB Function2'
static real_T randn(void)
{
  real_T r;
  uint32_T b_state;
  uint32_T c_state;
  if (!rtDW.method_not_empty) {
    rtDW.method_c = 0U;
    rtDW.method_not_empty = true;
    rtDW.state_p[0] = 362436069U;
    rtDW.state_p[1] = 0U;
    if (rtDW.state_p[1] == 0U) {
      rtDW.state_p[1] = 521288629U;
    }
  }

  if (rtDW.method_c == 0U) {
    switch (rtDW.method) {
     case 4U:
      c_state = rtDW.state;
      do {
        genrandu(c_state, &b_state, &rtB.c_r);
        genrandu(b_state, &c_state, &rtB.t);
        rtB.c_r = 2.0 * rtB.c_r - 1.0;
        rtB.t = 2.0 * rtB.t - 1.0;
        rtB.t = rtB.t * rtB.t + rtB.c_r * rtB.c_r;
      } while (!(rtB.t <= 1.0));

      r = sqrt(-2.0 * log(rtB.t) / rtB.t) * rtB.c_r;
      rtDW.state = c_state;
      break;

     case 5U:
      r = eml_rand_shr3cong(rtDW.state_a);
      break;

     default:
      if (!rtDW.state_not_empty) {
        memset(&rtDW.state_i[0], 0, 625U * sizeof(uint32_T));
        twister_state_vector_g(rtDW.state_i, 5489U);
        rtDW.state_not_empty = true;
      }

      r = eml_rand_mt19937ar_g2(rtDW.state_i);
      break;
    }
  } else if (rtDW.method_c == 4U) {
    c_state = rtDW.state_p[0];
    do {
      genrandu(c_state, &b_state, &rtB.c_r);
      genrandu(b_state, &c_state, &rtB.t);
      rtB.c_r = 2.0 * rtB.c_r - 1.0;
      rtB.t = 2.0 * rtB.t - 1.0;
      rtB.t = rtB.t * rtB.t + rtB.c_r * rtB.c_r;
    } while (!(rtB.t <= 1.0));

    r = sqrt(-2.0 * log(rtB.t) / rtB.t) * rtB.c_r;
    rtDW.state_p[0] = c_state;
  } else {
    r = eml_rand_shr3cong(rtDW.state_p);
  }

  return r;
}

// Model step function
void ros_image_filters_0706_step(void)
{
  uint8_T mtmp;
  uint8_T b_mtmp;
  visioncodegen_ShapeInserter *iobj_0;
  uint8_T *ImageDataTypeConversion;
  boolean_T exitg1;

  // Outputs for Atomic SubSystem: '<Root>/Subscribe'
  // MATLABSystem: '<S16>/SourceBlock' incorporates:
  //   Start for MATLABSystem: '<S16>/SourceBlock'

  rtB.SourceBlock_o1 = Sub_ros_image_filters_0706_335.getLatestMessage(&rtB.y);

  // Outputs for Enabled SubSystem: '<S16>/Enabled Subsystem' incorporates:
  //   EnablePort: '<S46>/Enable'

  if (rtB.SourceBlock_o1) {
    // Inport: '<S46>/In1' incorporates:
    //   MATLABSystem: '<S16>/SourceBlock'
    //   Start for MATLABSystem: '<S16>/SourceBlock'

    rtB.In1 = rtB.y;
  }

  // End of Outputs for SubSystem: '<S16>/Enabled Subsystem'
  // End of Outputs for SubSystem: '<Root>/Subscribe'
  // MATLAB Function: '<S17>/ROSraw2RGB'
  // MATLAB Function 'ros2image/ROSraw2RGB': '<S47>:1'
  // '<S47>:1:3' Rs = u(1:3:end);
  // '<S47>:1:4' Gs = u(2:3:end);
  // '<S47>:1:5' Bs = u(3:3:end);
  // '<S47>:1:6' B = reshape(Rs,960,540)';
  for (rtB.ku = 0; rtB.ku < 518400; rtB.ku++) {
    rtB.ImageDataTypeConversion[rtB.ku] = rtB.In1.Data[3 * rtB.ku];
  }

  for (rtB.ku = 0; rtB.ku < 960; rtB.ku++) {
    for (rtB.lineOff = 0; rtB.lineOff < 540; rtB.lineOff++) {
      rtB.VectorConcatenate[(rtB.lineOff + rtB.ku * 540) + 1036800] =
        rtB.ImageDataTypeConversion[960 * rtB.lineOff + rtB.ku];
    }
  }

  // '<S47>:1:7' G = reshape(Gs,960,540)';
  for (rtB.ku = 0; rtB.ku < 518400; rtB.ku++) {
    rtB.ImageDataTypeConversion[rtB.ku] = rtB.In1.Data[3 * rtB.ku + 1];
  }

  for (rtB.ku = 0; rtB.ku < 960; rtB.ku++) {
    for (rtB.lineOff = 0; rtB.lineOff < 540; rtB.lineOff++) {
      rtB.VectorConcatenate[(rtB.lineOff + rtB.ku * 540) + 518400] =
        rtB.ImageDataTypeConversion[960 * rtB.lineOff + rtB.ku];
    }
  }

  // '<S47>:1:8' R = reshape(Bs,960,540)';
  for (rtB.ku = 0; rtB.ku < 518400; rtB.ku++) {
    rtB.ImageDataTypeConversion[rtB.ku] = rtB.In1.Data[3 * rtB.ku + 2];
  }

  for (rtB.ku = 0; rtB.ku < 960; rtB.ku++) {
    for (rtB.lineOff = 0; rtB.lineOff < 540; rtB.lineOff++) {
      rtB.VectorConcatenate[rtB.lineOff + rtB.ku * 540] =
        rtB.ImageDataTypeConversion[960 * rtB.lineOff + rtB.ku];
    }
  }

  // End of MATLAB Function: '<S17>/ROSraw2RGB'
  for (rtB.lineOff = 0; rtB.lineOff < 518400; rtB.lineOff++) {
    // S-Function (svipscalenconvert): '<Root>/Image Data Type Conversion2' incorporates:
    //   S-Function (svipcolorconv): '<Root>/Color Space  Conversion'

    rtB.ImageDataTypeConversion2[rtB.lineOff] = (real_T)(uint8_T)
      ((((rtB.VectorConcatenate[518400 + rtB.lineOff] * 38470U +
          rtB.VectorConcatenate[rtB.lineOff] * 19595U) + rtB.VectorConcatenate
         [1036800 + rtB.lineOff] * 7471U) + 32768U) >> 16) / 255.0;

    // Abs: '<Root>/Abs' incorporates:
    //   Sum: '<Root>/Subtract'
    //   UnitDelay: '<Root>/Unit Delay'

    rtB.Abs[rtB.lineOff] = fabs(rtB.ImageDataTypeConversion2[rtB.lineOff] -
      rtDW.UnitDelay_DSTATE[rtB.lineOff]);
  }

  // S-Function (svipmorphop): '<Root>/Erosion'
  rtB.ky = 0;
  rtB.ku = 0;
  for (rtB.lineOff = 0; rtB.lineOff < 3; rtB.lineOff++) {
    rtDW.Erosion_ONE_PAD_IMG_DW[rtB.ky] = (rtInf);
    rtB.ky++;
    for (rtB.centerRow = 0; rtB.centerRow < 552; rtB.centerRow++) {
      rtDW.Erosion_ONE_PAD_IMG_DW[rtB.ky] = (rtInf);
      rtB.ky++;
    }
  }

  for (rtB.lineOff = 0; rtB.lineOff < 960; rtB.lineOff++) {
    rtDW.Erosion_ONE_PAD_IMG_DW[rtB.ky] = (rtInf);
    rtB.ky++;
    rtDW.Erosion_ONE_PAD_IMG_DW[rtB.ky] = (rtInf);
    rtB.ky++;
    rtDW.Erosion_ONE_PAD_IMG_DW[rtB.ky] = (rtInf);
    rtB.ky++;
    memcpy(&rtDW.Erosion_ONE_PAD_IMG_DW[rtB.ky], &rtB.Abs[rtB.ku], 540U * sizeof
           (real_T));
    rtB.ky += 540;
    rtB.ku += 540;
    rtDW.Erosion_ONE_PAD_IMG_DW[rtB.ky] = (rtInf);
    rtB.ky++;
    for (rtB.centerRow = 0; rtB.centerRow < 9; rtB.centerRow++) {
      rtDW.Erosion_ONE_PAD_IMG_DW[rtB.ky] = (rtInf);
      rtB.ky++;
    }
  }

  for (rtB.lineOff = 0; rtB.lineOff < 10; rtB.lineOff++) {
    rtDW.Erosion_ONE_PAD_IMG_DW[rtB.ky] = (rtInf);
    rtB.ky++;
    for (rtB.centerRow = 0; rtB.centerRow < 552; rtB.centerRow++) {
      rtDW.Erosion_ONE_PAD_IMG_DW[rtB.ky] = (rtInf);
      rtB.ky++;
    }
  }

  for (rtB.lineOff = 0; rtB.lineOff < 538069; rtB.lineOff++) {
    rtDW.Erosion_TWO_PAD_IMG_DW[rtB.lineOff] = (rtInf);
  }

  rtB.ku = rtDW.Erosion_NUMNONZ_DW[0];
  rtB.ky = 0;
  rtB.outIdx = 3;
  if (rtDW.Erosion_STREL_DW[0] == 0) {
    rtB.col = 0;
    while (rtB.col < 972) {
      rtB.numEleTmp = 0;
      while (rtB.numEleTmp < 546) {
        rtB.Clock = rtDW.Erosion_ONE_PAD_IMG_DW[rtB.ky +
          rtDW.Erosion_ERODE_OFF_DW[0]];
        rtB.lineOff = 1;
        while (rtB.lineOff < rtB.ku) {
          if (rtDW.Erosion_ONE_PAD_IMG_DW[rtB.ky +
              rtDW.Erosion_ERODE_OFF_DW[rtB.lineOff]] < rtB.Clock) {
            rtB.Clock = rtDW.Erosion_ONE_PAD_IMG_DW[rtB.ky +
              rtDW.Erosion_ERODE_OFF_DW[rtB.lineOff]];
          }

          rtB.lineOff++;
        }

        rtDW.Erosion_TWO_PAD_IMG_DW[rtB.outIdx] = rtB.Clock;
        rtB.ky++;
        rtB.outIdx++;
        rtB.numEleTmp++;
      }

      rtB.ky += 7;
      rtB.outIdx += 7;
      rtB.col++;
    }
  } else if (rtDW.Erosion_STREL_DW[0] == 1) {
    rtB.numIter = div_nzp_s32(549, rtDW.Erosion_NUMNONZ_DW[0]);
    rtB.ky = div_nzp_s32(rtDW.Erosion_ERODE_OFF_DW[0], 553) * 553;
    rtB.lineOff = rtDW.Erosion_ERODE_OFF_DW[0] - rtB.ky;
    rtB.gOffset = (rtDW.Erosion_NUMNONZ_DW[0] + rtB.lineOff) - 3;
    rtB.hOffset = rtB.lineOff - 3;
    rtB.ky += 3;
    rtB.lastBlockCol = rtB.numIter * rtDW.Erosion_NUMNONZ_DW[0];
    rtB.lineOff = 0;
    while (rtB.lineOff < rtB.ku) {
      rtDW.Erosion_GBUF_DW[rtB.lineOff] = (rtInf);
      rtB.lineOff++;
    }

    rtB.numEleTmp = (rtB.numIter + 1) * rtDW.Erosion_NUMNONZ_DW[0];
    rtB.lineOff = rtB.numEleTmp;
    while (rtB.lineOff < rtB.numEleTmp + rtB.ku) {
      rtDW.Erosion_GBUF_DW[rtB.lineOff] = (rtInf);
      rtB.lineOff++;
    }

    rtB.lineOff = 0;
    while (rtB.lineOff < rtB.ku) {
      rtDW.Erosion_HBUF_DW[rtB.lineOff] = (rtInf);
      rtB.lineOff++;
    }

    rtB.lineOff = rtB.numEleTmp;
    while (rtB.lineOff < rtB.numEleTmp + rtB.ku) {
      rtDW.Erosion_HBUF_DW[rtB.lineOff] = (rtInf);
      rtB.lineOff++;
    }

    rtB.col = 0;
    while (rtB.col < 972) {
      rtB.idx = rtB.ku;
      rtB.iter = 0;
      while (rtB.iter < rtB.numIter) {
        rtDW.Erosion_GBUF_DW[rtB.idx] = rtDW.Erosion_ONE_PAD_IMG_DW[rtB.ky];
        rtB.idx++;
        rtB.ky++;
        rtB.lineOff = 1;
        while (rtB.lineOff < rtB.ku) {
          rtDW.Erosion_GBUF_DW[rtB.idx] = fmin(rtDW.Erosion_GBUF_DW[rtB.idx - 1],
            rtDW.Erosion_ONE_PAD_IMG_DW[rtB.ky]);
          rtB.idx++;
          rtB.ky++;
          rtB.lineOff++;
        }

        rtB.iter++;
      }

      if (rtB.lastBlockCol <= 546) {
        rtDW.Erosion_GBUF_DW[rtB.idx] = rtDW.Erosion_ONE_PAD_IMG_DW[rtB.ky];
        rtB.idx++;
        rtB.ky++;
        rtB.iter = rtB.lastBlockCol + rtB.ku;
        rtB.lineOff = rtB.lastBlockCol + 1;
        while (rtB.lineOff < rtB.iter) {
          if (rtB.lineOff < 546) {
            rtDW.Erosion_GBUF_DW[rtB.idx] = fmin(rtDW.Erosion_GBUF_DW[rtB.idx -
              1], rtDW.Erosion_ONE_PAD_IMG_DW[rtB.ky]);
          } else {
            rtDW.Erosion_GBUF_DW[rtB.idx] = rtDW.Erosion_GBUF_DW[rtB.idx - 1];
          }

          rtB.idx++;
          rtB.ky++;
          rtB.lineOff++;
        }
      }

      rtB.ky--;
      if (rtB.lastBlockCol <= 546) {
        rtB.lineOff = 1;
        while (rtB.lineOff - 1 < rtB.ku) {
          if ((rtB.ku - rtB.lineOff) + rtB.lastBlockCol < 546) {
            rtDW.Erosion_HBUF_DW[rtB.idx - 1] = fmin
              (rtDW.Erosion_HBUF_DW[rtB.idx], rtDW.Erosion_ONE_PAD_IMG_DW[rtB.ky]);
          } else {
            rtDW.Erosion_HBUF_DW[rtB.idx - 1] = rtDW.Erosion_HBUF_DW[rtB.idx];
          }

          rtB.idx--;
          rtB.ky--;
          rtB.lineOff++;
        }
      }

      rtB.iter = 0;
      while (rtB.iter < rtB.numIter) {
        rtDW.Erosion_HBUF_DW[rtB.idx - 1] = rtDW.Erosion_ONE_PAD_IMG_DW[rtB.ky];
        rtB.idx--;
        rtB.ky--;
        rtB.lineOff = 1;
        while (rtB.lineOff < rtB.ku) {
          rtDW.Erosion_HBUF_DW[rtB.idx - 1] = fmin(rtDW.Erosion_HBUF_DW[rtB.idx],
            rtDW.Erosion_ONE_PAD_IMG_DW[rtB.ky]);
          rtB.idx--;
          rtB.ky--;
          rtB.lineOff++;
        }

        rtB.iter++;
      }

      rtB.numEleTmp = 0;
      while (rtB.numEleTmp < 546) {
        rtDW.Erosion_TWO_PAD_IMG_DW[rtB.outIdx] = fmin(rtDW.Erosion_GBUF_DW
          [(rtB.idx + rtB.gOffset) - 1], rtDW.Erosion_HBUF_DW[rtB.idx +
          rtB.hOffset]);
        rtB.idx++;
        rtB.outIdx++;
        rtB.numEleTmp++;
      }

      rtB.ky += 554;
      rtB.outIdx += 7;
      rtB.col++;
    }
  } else {
    rtB.numIter = div_nzp_s32(972, rtDW.Erosion_NUMNONZ_DW[0]);
    rtB.lineOff = div_nzp_s32(rtDW.Erosion_ERODE_OFF_DW[0], 553);
    rtB.gOffset = rtDW.Erosion_NUMNONZ_DW[0] + rtB.lineOff;
    rtB.hOffset = rtB.lineOff;
    rtB.ky = rtDW.Erosion_ERODE_OFF_DW[0] - rtB.lineOff * 553;
    rtB.lastBlockCol = rtB.numIter * rtDW.Erosion_NUMNONZ_DW[0];
    rtB.lineOff = 0;
    while (rtB.lineOff < rtB.ku) {
      rtDW.Erosion_GBUF_DW[rtB.lineOff] = (rtInf);
      rtB.lineOff++;
    }

    rtB.numEleTmp = (rtB.numIter + 1) * rtDW.Erosion_NUMNONZ_DW[0];
    rtB.lineOff = rtB.numEleTmp;
    while (rtB.lineOff < rtB.numEleTmp + rtB.ku) {
      rtDW.Erosion_GBUF_DW[rtB.lineOff] = (rtInf);
      rtB.lineOff++;
    }

    rtB.lineOff = 0;
    while (rtB.lineOff < rtB.ku) {
      rtDW.Erosion_HBUF_DW[rtB.lineOff] = (rtInf);
      rtB.lineOff++;
    }

    rtB.lineOff = rtB.numEleTmp;
    while (rtB.lineOff < rtB.numEleTmp + rtB.ku) {
      rtDW.Erosion_HBUF_DW[rtB.lineOff] = (rtInf);
      rtB.lineOff++;
    }

    rtB.numEleTmp = 0;
    while (rtB.numEleTmp < 546) {
      rtB.idx = rtB.ku;
      rtB.iter = 0;
      while (rtB.iter < rtB.numIter) {
        rtDW.Erosion_GBUF_DW[rtB.idx] = rtDW.Erosion_ONE_PAD_IMG_DW[rtB.ky];
        rtB.idx++;
        rtB.ky += 553;
        rtB.lineOff = 1;
        while (rtB.lineOff < rtB.ku) {
          rtDW.Erosion_GBUF_DW[rtB.idx] = fmin(rtDW.Erosion_GBUF_DW[rtB.idx - 1],
            rtDW.Erosion_ONE_PAD_IMG_DW[rtB.ky]);
          rtB.idx++;
          rtB.ky += 553;
          rtB.lineOff++;
        }

        rtB.iter++;
      }

      if (rtB.lastBlockCol <= 972) {
        rtDW.Erosion_GBUF_DW[rtB.idx] = rtDW.Erosion_ONE_PAD_IMG_DW[rtB.ky];
        rtB.idx++;
        rtB.ky += 553;
        rtB.iter = rtB.lastBlockCol + rtB.ku;
        rtB.lineOff = rtB.lastBlockCol + 1;
        while (rtB.lineOff < rtB.iter) {
          if (rtB.lineOff < 972) {
            rtDW.Erosion_GBUF_DW[rtB.idx] = fmin(rtDW.Erosion_GBUF_DW[rtB.idx -
              1], rtDW.Erosion_ONE_PAD_IMG_DW[rtB.ky]);
          } else {
            rtDW.Erosion_GBUF_DW[rtB.idx] = rtDW.Erosion_GBUF_DW[rtB.idx - 1];
          }

          rtB.idx++;
          rtB.ky += 553;
          rtB.lineOff++;
        }
      }

      rtB.ky -= 553;
      if (rtB.lastBlockCol <= 972) {
        rtB.lineOff = 1;
        while (rtB.lineOff - 1 < rtB.ku) {
          if ((rtB.ku - rtB.lineOff) + rtB.lastBlockCol < 972) {
            rtDW.Erosion_HBUF_DW[rtB.idx - 1] = fmin
              (rtDW.Erosion_HBUF_DW[rtB.idx], rtDW.Erosion_ONE_PAD_IMG_DW[rtB.ky]);
          } else {
            rtDW.Erosion_HBUF_DW[rtB.idx - 1] = rtDW.Erosion_HBUF_DW[rtB.idx];
          }

          rtB.idx--;
          rtB.ky -= 553;
          rtB.lineOff++;
        }
      }

      rtB.iter = 0;
      while (rtB.iter < rtB.numIter) {
        rtDW.Erosion_HBUF_DW[rtB.idx - 1] = rtDW.Erosion_ONE_PAD_IMG_DW[rtB.ky];
        rtB.idx--;
        rtB.ky -= 553;
        rtB.lineOff = 1;
        while (rtB.lineOff < rtB.ku) {
          rtDW.Erosion_HBUF_DW[rtB.idx - 1] = fmin(rtDW.Erosion_HBUF_DW[rtB.idx],
            rtDW.Erosion_ONE_PAD_IMG_DW[rtB.ky]);
          rtB.idx--;
          rtB.ky -= 553;
          rtB.lineOff++;
        }

        rtB.iter++;
      }

      rtB.col = 0;
      while (rtB.col < 972) {
        rtDW.Erosion_TWO_PAD_IMG_DW[rtB.outIdx] = fmin(rtDW.Erosion_GBUF_DW
          [(rtB.idx + rtB.gOffset) - 1], rtDW.Erosion_HBUF_DW[rtB.idx +
          rtB.hOffset]);
        rtB.idx++;
        rtB.outIdx += 553;
        rtB.col++;
      }

      rtB.ky += 554;
      rtB.outIdx += -537515;
      rtB.numEleTmp++;
    }
  }

  rtB.numIter = rtDW.Erosion_NUMNONZ_DW[0];
  rtB.ku = rtDW.Erosion_NUMNONZ_DW[1];
  rtB.ky = 3;
  rtB.outIdx = 0;
  if (rtDW.Erosion_STREL_DW[1] == 0) {
    rtB.col = 0;
    while (rtB.col < 960) {
      rtB.numEleTmp = 3;
      while (rtB.numEleTmp < 543) {
        rtB.Clock = rtDW.Erosion_TWO_PAD_IMG_DW[rtB.ky +
          rtDW.Erosion_ERODE_OFF_DW[rtB.numIter]];
        rtB.lineOff = 1;
        while (rtB.lineOff < rtB.ku) {
          if (rtDW.Erosion_TWO_PAD_IMG_DW[rtDW.Erosion_ERODE_OFF_DW[rtB.lineOff
              + rtB.numIter] + rtB.ky] < rtB.Clock) {
            rtB.Clock =
              rtDW.Erosion_TWO_PAD_IMG_DW[rtDW.Erosion_ERODE_OFF_DW[rtB.lineOff
              + rtB.numIter] + rtB.ky];
          }

          rtB.lineOff++;
        }

        rtB.Erosion[rtB.outIdx] = rtB.Clock;
        rtB.ky++;
        rtB.outIdx++;
        rtB.numEleTmp++;
      }

      rtB.ky += 13;
      rtB.col++;
    }
  } else if (rtDW.Erosion_STREL_DW[1] == 1) {
    rtB.numIter = div_nzp_s32(540, rtDW.Erosion_NUMNONZ_DW[1]);
    rtB.ky = div_nzp_s32(rtDW.Erosion_ERODE_OFF_DW[rtDW.Erosion_NUMNONZ_DW[0]],
                         553) * 553;
    rtB.lineOff = rtDW.Erosion_ERODE_OFF_DW[rtDW.Erosion_NUMNONZ_DW[0]] - rtB.ky;
    rtB.gOffset = rtDW.Erosion_NUMNONZ_DW[1] + rtB.lineOff;
    rtB.hOffset = rtB.lineOff;
    rtB.ky += 3;
    rtB.lastBlockCol = rtB.numIter * rtDW.Erosion_NUMNONZ_DW[1] + 3;
    rtB.lineOff = 0;
    while (rtB.lineOff < rtB.ku) {
      rtDW.Erosion_GBUF_DW[rtB.lineOff] = (rtInf);
      rtB.lineOff++;
    }

    rtB.numEleTmp = (rtB.numIter + 1) * rtDW.Erosion_NUMNONZ_DW[1];
    rtB.lineOff = rtB.numEleTmp;
    while (rtB.lineOff < rtB.numEleTmp + rtB.ku) {
      rtDW.Erosion_GBUF_DW[rtB.lineOff] = (rtInf);
      rtB.lineOff++;
    }

    rtB.lineOff = 0;
    while (rtB.lineOff < rtB.ku) {
      rtDW.Erosion_HBUF_DW[rtB.lineOff] = (rtInf);
      rtB.lineOff++;
    }

    rtB.lineOff = rtB.numEleTmp;
    while (rtB.lineOff < rtB.numEleTmp + rtB.ku) {
      rtDW.Erosion_HBUF_DW[rtB.lineOff] = (rtInf);
      rtB.lineOff++;
    }

    rtB.col = 0;
    while (rtB.col < 960) {
      rtB.idx = rtB.ku;
      rtB.iter = 0;
      while (rtB.iter < rtB.numIter) {
        rtDW.Erosion_GBUF_DW[rtB.idx] = rtDW.Erosion_TWO_PAD_IMG_DW[rtB.ky];
        rtB.idx++;
        rtB.ky++;
        rtB.lineOff = 1;
        while (rtB.lineOff < rtB.ku) {
          rtDW.Erosion_GBUF_DW[rtB.idx] = fmin(rtDW.Erosion_GBUF_DW[rtB.idx - 1],
            rtDW.Erosion_TWO_PAD_IMG_DW[rtB.ky]);
          rtB.idx++;
          rtB.ky++;
          rtB.lineOff++;
        }

        rtB.iter++;
      }

      if (rtB.lastBlockCol <= 543) {
        rtDW.Erosion_GBUF_DW[rtB.idx] = rtDW.Erosion_TWO_PAD_IMG_DW[rtB.ky];
        rtB.idx++;
        rtB.ky++;
        rtB.iter = rtB.lastBlockCol + rtB.ku;
        rtB.lineOff = rtB.lastBlockCol + 1;
        while (rtB.lineOff < rtB.iter) {
          if (rtB.lineOff < 543) {
            rtDW.Erosion_GBUF_DW[rtB.idx] = fmin(rtDW.Erosion_GBUF_DW[rtB.idx -
              1], rtDW.Erosion_TWO_PAD_IMG_DW[rtB.ky]);
          } else {
            rtDW.Erosion_GBUF_DW[rtB.idx] = rtDW.Erosion_GBUF_DW[rtB.idx - 1];
          }

          rtB.idx++;
          rtB.ky++;
          rtB.lineOff++;
        }
      }

      rtB.ky--;
      if (rtB.lastBlockCol <= 543) {
        rtB.lineOff = 1;
        while (rtB.lineOff - 1 < rtB.ku) {
          if ((rtB.ku - rtB.lineOff) + rtB.lastBlockCol < 543) {
            rtDW.Erosion_HBUF_DW[rtB.idx - 1] = fmin
              (rtDW.Erosion_HBUF_DW[rtB.idx], rtDW.Erosion_TWO_PAD_IMG_DW[rtB.ky]);
          } else {
            rtDW.Erosion_HBUF_DW[rtB.idx - 1] = rtDW.Erosion_HBUF_DW[rtB.idx];
          }

          rtB.idx--;
          rtB.ky--;
          rtB.lineOff++;
        }
      }

      rtB.iter = 0;
      while (rtB.iter < rtB.numIter) {
        rtDW.Erosion_HBUF_DW[rtB.idx - 1] = rtDW.Erosion_TWO_PAD_IMG_DW[rtB.ky];
        rtB.idx--;
        rtB.ky--;
        rtB.lineOff = 1;
        while (rtB.lineOff < rtB.ku) {
          rtDW.Erosion_HBUF_DW[rtB.idx - 1] = fmin(rtDW.Erosion_HBUF_DW[rtB.idx],
            rtDW.Erosion_TWO_PAD_IMG_DW[rtB.ky]);
          rtB.idx--;
          rtB.ky--;
          rtB.lineOff++;
        }

        rtB.iter++;
      }

      rtB.numEleTmp = 3;
      while (rtB.numEleTmp < 543) {
        rtB.Erosion[rtB.outIdx] = fmin(rtDW.Erosion_GBUF_DW[(rtB.idx +
          rtB.gOffset) - 1], rtDW.Erosion_HBUF_DW[rtB.idx + rtB.hOffset]);
        rtB.idx++;
        rtB.outIdx++;
        rtB.numEleTmp++;
      }

      rtB.ky += 554;
      rtB.col++;
    }
  } else {
    rtB.numIter = div_nzp_s32(963, rtDW.Erosion_NUMNONZ_DW[1]);
    rtB.lineOff = div_nzp_s32(rtDW.Erosion_ERODE_OFF_DW[rtDW.Erosion_NUMNONZ_DW
      [0]], 553);
    rtB.gOffset = (rtDW.Erosion_NUMNONZ_DW[1] + rtB.lineOff) - 3;
    rtB.hOffset = rtB.lineOff - 3;
    rtB.ky = (rtDW.Erosion_ERODE_OFF_DW[rtDW.Erosion_NUMNONZ_DW[0]] -
              rtB.lineOff * 553) + 1662;
    rtB.lastBlockCol = rtB.numIter * rtDW.Erosion_NUMNONZ_DW[1];
    rtB.lineOff = 0;
    while (rtB.lineOff < rtB.ku) {
      rtDW.Erosion_GBUF_DW[rtB.lineOff] = (rtInf);
      rtB.lineOff++;
    }

    rtB.numEleTmp = (rtB.numIter + 1) * rtDW.Erosion_NUMNONZ_DW[1];
    rtB.lineOff = rtB.numEleTmp;
    while (rtB.lineOff < rtB.numEleTmp + rtB.ku) {
      rtDW.Erosion_GBUF_DW[rtB.lineOff] = (rtInf);
      rtB.lineOff++;
    }

    rtB.lineOff = 0;
    while (rtB.lineOff < rtB.ku) {
      rtDW.Erosion_HBUF_DW[rtB.lineOff] = (rtInf);
      rtB.lineOff++;
    }

    rtB.lineOff = rtB.numEleTmp;
    while (rtB.lineOff < rtB.numEleTmp + rtB.ku) {
      rtDW.Erosion_HBUF_DW[rtB.lineOff] = (rtInf);
      rtB.lineOff++;
    }

    rtB.numEleTmp = 3;
    while (rtB.numEleTmp < 543) {
      rtB.idx = rtB.ku;
      rtB.iter = 0;
      while (rtB.iter < rtB.numIter) {
        rtDW.Erosion_GBUF_DW[rtB.idx] = rtDW.Erosion_TWO_PAD_IMG_DW[rtB.ky];
        rtB.idx++;
        rtB.ky += 553;
        rtB.lineOff = 1;
        while (rtB.lineOff < rtB.ku) {
          rtDW.Erosion_GBUF_DW[rtB.idx] = fmin(rtDW.Erosion_GBUF_DW[rtB.idx - 1],
            rtDW.Erosion_TWO_PAD_IMG_DW[rtB.ky]);
          rtB.idx++;
          rtB.ky += 553;
          rtB.lineOff++;
        }

        rtB.iter++;
      }

      if (rtB.lastBlockCol <= 960) {
        rtDW.Erosion_GBUF_DW[rtB.idx] = rtDW.Erosion_TWO_PAD_IMG_DW[rtB.ky];
        rtB.idx++;
        rtB.ky += 553;
        rtB.iter = rtB.lastBlockCol + rtB.ku;
        rtB.lineOff = rtB.lastBlockCol + 1;
        while (rtB.lineOff < rtB.iter) {
          if (rtB.lineOff < 960) {
            rtDW.Erosion_GBUF_DW[rtB.idx] = fmin(rtDW.Erosion_GBUF_DW[rtB.idx -
              1], rtDW.Erosion_TWO_PAD_IMG_DW[rtB.ky]);
          } else {
            rtDW.Erosion_GBUF_DW[rtB.idx] = rtDW.Erosion_GBUF_DW[rtB.idx - 1];
          }

          rtB.idx++;
          rtB.ky += 553;
          rtB.lineOff++;
        }
      }

      rtB.ky -= 553;
      if (rtB.lastBlockCol <= 960) {
        rtB.lineOff = 1;
        while (rtB.lineOff - 1 < rtB.ku) {
          if ((rtB.ku - rtB.lineOff) + rtB.lastBlockCol < 960) {
            rtDW.Erosion_HBUF_DW[rtB.idx - 1] = fmin
              (rtDW.Erosion_HBUF_DW[rtB.idx], rtDW.Erosion_TWO_PAD_IMG_DW[rtB.ky]);
          } else {
            rtDW.Erosion_HBUF_DW[rtB.idx - 1] = rtDW.Erosion_HBUF_DW[rtB.idx];
          }

          rtB.idx--;
          rtB.ky -= 553;
          rtB.lineOff++;
        }
      }

      rtB.iter = 0;
      while (rtB.iter < rtB.numIter) {
        rtDW.Erosion_HBUF_DW[rtB.idx - 1] = rtDW.Erosion_TWO_PAD_IMG_DW[rtB.ky];
        rtB.idx--;
        rtB.ky -= 553;
        rtB.lineOff = 1;
        while (rtB.lineOff < rtB.ku) {
          rtDW.Erosion_HBUF_DW[rtB.idx - 1] = fmin(rtDW.Erosion_HBUF_DW[rtB.idx],
            rtDW.Erosion_TWO_PAD_IMG_DW[rtB.ky]);
          rtB.idx--;
          rtB.ky -= 553;
          rtB.lineOff++;
        }

        rtB.iter++;
      }

      rtB.col = 0;
      while (rtB.col < 960) {
        rtB.Erosion[rtB.outIdx] = fmin(rtDW.Erosion_GBUF_DW[(rtB.idx +
          rtB.gOffset) - 1], rtDW.Erosion_HBUF_DW[rtB.idx + rtB.hOffset]);
        rtB.idx++;
        rtB.outIdx += 540;
        rtB.col++;
      }

      rtB.ky += 554;
      rtB.outIdx += -518399;
      rtB.numEleTmp++;
    }
  }

  // End of S-Function (svipmorphop): '<Root>/Erosion'

  // MATLAB Function: '<Root>/A1'
  A1(rtB.Erosion, rtB.int_img_m, &rtB.sf_A1);

  // MATLAB Function: '<Root>/MATLAB Function'
  // MATLAB Function 'MATLAB Function': '<S8>:1'
  // '<S8>:1:3' R = u(:, :, 1);
  // '<S8>:1:4' G = u(:, :, 2);
  // '<S8>:1:5' B = u(:, :, 3);
  // '<S8>:1:8' result = double(zeros(540, 960, 1));
  memset(&rtB.Abs[0], 0, 518400U * sizeof(real_T));

  // result(:, :, 1) = R./(R + G + B);
  // result(:, :, 2) = G./(R + G + B);
  // result(:, :, 3) = B./(R + G + B);
  // '<S8>:1:15' for i = 1:540
  for (rtB.lineOff = 0; rtB.lineOff < 540; rtB.lineOff++) {
    // '<S8>:1:16' for j = 1:960
    for (rtB.centerRow = 0; rtB.centerRow < 960; rtB.centerRow++) {
      // '<S8>:1:17' rgb = u(i, j, :);
      // '<S8>:1:18' if rgb(1) > 95 && rgb(2) > 40 && rgb(3) > 20 && max(rgb) - min(rgb) > 15 ... 
      // '<S8>:1:19'             && abs(rgb(1) - rgb(2)) > 15 && rgb(1) > rgb(2) && rgb(1) > rgb(3) 
      if ((rtB.VectorConcatenate[540 * rtB.centerRow + rtB.lineOff] > 95) &&
          (rtB.VectorConcatenate[(540 * rtB.centerRow + rtB.lineOff) + 518400] >
           40) && (rtB.VectorConcatenate[(540 * rtB.centerRow + rtB.lineOff) +
                   1036800] > 20)) {
        mtmp = rtB.VectorConcatenate[540 * rtB.centerRow + rtB.lineOff];
        b_mtmp = rtB.VectorConcatenate[540 * rtB.centerRow + rtB.lineOff];
        if (rtB.VectorConcatenate[(540 * rtB.centerRow + rtB.lineOff) + 518400] >
            rtB.VectorConcatenate[540 * rtB.centerRow + rtB.lineOff]) {
          mtmp = rtB.VectorConcatenate[(540 * rtB.centerRow + rtB.lineOff) +
            518400];
        }

        if (rtB.VectorConcatenate[(540 * rtB.centerRow + rtB.lineOff) + 518400] <
            rtB.VectorConcatenate[540 * rtB.centerRow + rtB.lineOff]) {
          b_mtmp = rtB.VectorConcatenate[(540 * rtB.centerRow + rtB.lineOff) +
            518400];
        }

        if (rtB.VectorConcatenate[(540 * rtB.centerRow + rtB.lineOff) + 1036800]
            > mtmp) {
          mtmp = rtB.VectorConcatenate[(540 * rtB.centerRow + rtB.lineOff) +
            1036800];
        }

        if (rtB.VectorConcatenate[(540 * rtB.centerRow + rtB.lineOff) + 1036800]
            < b_mtmp) {
          b_mtmp = rtB.VectorConcatenate[(540 * rtB.centerRow + rtB.lineOff) +
            1036800];
        }

        rtB.qY = (uint32_T)mtmp - /*MW:OvSatOk*/ b_mtmp;
        if (rtB.qY > mtmp) {
          rtB.qY = 0U;
        }

        if ((int32_T)rtB.qY > 15) {
          rtB.qY = (uint32_T)rtB.VectorConcatenate[540 * rtB.centerRow +
            rtB.lineOff] - /*MW:OvSatOk*/ rtB.VectorConcatenate[(540 *
            rtB.centerRow + rtB.lineOff) + 518400];
          if (rtB.qY > rtB.VectorConcatenate[540 * rtB.centerRow + rtB.lineOff])
          {
            rtB.qY = 0U;
          }

          if (((int32_T)rtB.qY > 15) && (rtB.VectorConcatenate[540 *
               rtB.centerRow + rtB.lineOff] > rtB.VectorConcatenate[(540 *
                rtB.centerRow + rtB.lineOff) + 518400]) &&
              (rtB.VectorConcatenate[540 * rtB.centerRow + rtB.lineOff] >
               rtB.VectorConcatenate[(540 * rtB.centerRow + rtB.lineOff) +
               1036800])) {
            // '<S8>:1:20' result(i, j) = 1;
            rtB.Abs[rtB.lineOff + 540 * rtB.centerRow] = 1.0;
          }
        }
      }
    }
  }

  // End of MATLAB Function: '<Root>/MATLAB Function'

  // S-Function (svipmorphop): '<Root>/Erosion1'
  rtB.ky = 0;
  rtB.ku = 0;
  for (rtB.lineOff = 0; rtB.lineOff < 3; rtB.lineOff++) {
    rtDW.Erosion1_ONE_PAD_IMG_DW[rtB.ky] = (rtInf);
    rtB.ky++;
    for (rtB.centerRow = 0; rtB.centerRow < 552; rtB.centerRow++) {
      rtDW.Erosion1_ONE_PAD_IMG_DW[rtB.ky] = (rtInf);
      rtB.ky++;
    }
  }

  for (rtB.lineOff = 0; rtB.lineOff < 960; rtB.lineOff++) {
    rtDW.Erosion1_ONE_PAD_IMG_DW[rtB.ky] = (rtInf);
    rtB.ky++;
    rtDW.Erosion1_ONE_PAD_IMG_DW[rtB.ky] = (rtInf);
    rtB.ky++;
    rtDW.Erosion1_ONE_PAD_IMG_DW[rtB.ky] = (rtInf);
    rtB.ky++;
    memcpy(&rtDW.Erosion1_ONE_PAD_IMG_DW[rtB.ky], &rtB.Abs[rtB.ku], 540U *
           sizeof(real_T));
    rtB.ky += 540;
    rtB.ku += 540;
    rtDW.Erosion1_ONE_PAD_IMG_DW[rtB.ky] = (rtInf);
    rtB.ky++;
    for (rtB.centerRow = 0; rtB.centerRow < 9; rtB.centerRow++) {
      rtDW.Erosion1_ONE_PAD_IMG_DW[rtB.ky] = (rtInf);
      rtB.ky++;
    }
  }

  for (rtB.lineOff = 0; rtB.lineOff < 10; rtB.lineOff++) {
    rtDW.Erosion1_ONE_PAD_IMG_DW[rtB.ky] = (rtInf);
    rtB.ky++;
    for (rtB.centerRow = 0; rtB.centerRow < 552; rtB.centerRow++) {
      rtDW.Erosion1_ONE_PAD_IMG_DW[rtB.ky] = (rtInf);
      rtB.ky++;
    }
  }

  for (rtB.lineOff = 0; rtB.lineOff < 538069; rtB.lineOff++) {
    rtDW.Erosion_TWO_PAD_IMG_DW[rtB.lineOff] = (rtInf);
  }

  rtB.ku = rtDW.Erosion1_NUMNONZ_DW[0];
  rtB.ky = 0;
  rtB.outIdx = 3;
  if (rtDW.Erosion1_STREL_DW[0] == 0) {
    rtB.col = 0;
    while (rtB.col < 972) {
      rtB.numEleTmp = 0;
      while (rtB.numEleTmp < 546) {
        rtB.Clock = rtDW.Erosion1_ONE_PAD_IMG_DW[rtB.ky +
          rtDW.Erosion1_ERODE_OFF_DW[0]];
        rtB.lineOff = 1;
        while (rtB.lineOff < rtB.ku) {
          if (rtDW.Erosion1_ONE_PAD_IMG_DW[rtB.ky +
              rtDW.Erosion1_ERODE_OFF_DW[rtB.lineOff]] < rtB.Clock) {
            rtB.Clock = rtDW.Erosion1_ONE_PAD_IMG_DW[rtB.ky +
              rtDW.Erosion1_ERODE_OFF_DW[rtB.lineOff]];
          }

          rtB.lineOff++;
        }

        rtDW.Erosion_TWO_PAD_IMG_DW[rtB.outIdx] = rtB.Clock;
        rtB.ky++;
        rtB.outIdx++;
        rtB.numEleTmp++;
      }

      rtB.ky += 7;
      rtB.outIdx += 7;
      rtB.col++;
    }
  } else if (rtDW.Erosion1_STREL_DW[0] == 1) {
    rtB.numIter = div_nzp_s32(549, rtDW.Erosion1_NUMNONZ_DW[0]);
    rtB.ky = div_nzp_s32(rtDW.Erosion1_ERODE_OFF_DW[0], 553) * 553;
    rtB.lineOff = rtDW.Erosion1_ERODE_OFF_DW[0] - rtB.ky;
    rtB.gOffset = (rtDW.Erosion1_NUMNONZ_DW[0] + rtB.lineOff) - 3;
    rtB.hOffset = rtB.lineOff - 3;
    rtB.ky += 3;
    rtB.lastBlockCol = rtB.numIter * rtDW.Erosion1_NUMNONZ_DW[0];
    rtB.lineOff = 0;
    while (rtB.lineOff < rtB.ku) {
      rtDW.Erosion1_GBUF_DW[rtB.lineOff] = (rtInf);
      rtB.lineOff++;
    }

    rtB.numEleTmp = (rtB.numIter + 1) * rtDW.Erosion1_NUMNONZ_DW[0];
    rtB.lineOff = rtB.numEleTmp;
    while (rtB.lineOff < rtB.numEleTmp + rtB.ku) {
      rtDW.Erosion1_GBUF_DW[rtB.lineOff] = (rtInf);
      rtB.lineOff++;
    }

    rtB.lineOff = 0;
    while (rtB.lineOff < rtB.ku) {
      rtDW.Erosion1_HBUF_DW[rtB.lineOff] = (rtInf);
      rtB.lineOff++;
    }

    rtB.lineOff = rtB.numEleTmp;
    while (rtB.lineOff < rtB.numEleTmp + rtB.ku) {
      rtDW.Erosion1_HBUF_DW[rtB.lineOff] = (rtInf);
      rtB.lineOff++;
    }

    rtB.col = 0;
    while (rtB.col < 972) {
      rtB.idx = rtB.ku;
      rtB.iter = 0;
      while (rtB.iter < rtB.numIter) {
        rtDW.Erosion1_GBUF_DW[rtB.idx] = rtDW.Erosion1_ONE_PAD_IMG_DW[rtB.ky];
        rtB.idx++;
        rtB.ky++;
        rtB.lineOff = 1;
        while (rtB.lineOff < rtB.ku) {
          rtDW.Erosion1_GBUF_DW[rtB.idx] = fmin(rtDW.Erosion1_GBUF_DW[rtB.idx -
            1], rtDW.Erosion1_ONE_PAD_IMG_DW[rtB.ky]);
          rtB.idx++;
          rtB.ky++;
          rtB.lineOff++;
        }

        rtB.iter++;
      }

      if (rtB.lastBlockCol <= 546) {
        rtDW.Erosion1_GBUF_DW[rtB.idx] = rtDW.Erosion1_ONE_PAD_IMG_DW[rtB.ky];
        rtB.idx++;
        rtB.ky++;
        rtB.iter = rtB.lastBlockCol + rtB.ku;
        rtB.lineOff = rtB.lastBlockCol + 1;
        while (rtB.lineOff < rtB.iter) {
          if (rtB.lineOff < 546) {
            rtDW.Erosion1_GBUF_DW[rtB.idx] = fmin(rtDW.Erosion1_GBUF_DW[rtB.idx
              - 1], rtDW.Erosion1_ONE_PAD_IMG_DW[rtB.ky]);
          } else {
            rtDW.Erosion1_GBUF_DW[rtB.idx] = rtDW.Erosion1_GBUF_DW[rtB.idx - 1];
          }

          rtB.idx++;
          rtB.ky++;
          rtB.lineOff++;
        }
      }

      rtB.ky--;
      if (rtB.lastBlockCol <= 546) {
        rtB.lineOff = 1;
        while (rtB.lineOff - 1 < rtB.ku) {
          if ((rtB.ku - rtB.lineOff) + rtB.lastBlockCol < 546) {
            rtDW.Erosion1_HBUF_DW[rtB.idx - 1] = fmin
              (rtDW.Erosion1_HBUF_DW[rtB.idx],
               rtDW.Erosion1_ONE_PAD_IMG_DW[rtB.ky]);
          } else {
            rtDW.Erosion1_HBUF_DW[rtB.idx - 1] = rtDW.Erosion1_HBUF_DW[rtB.idx];
          }

          rtB.idx--;
          rtB.ky--;
          rtB.lineOff++;
        }
      }

      rtB.iter = 0;
      while (rtB.iter < rtB.numIter) {
        rtDW.Erosion1_HBUF_DW[rtB.idx - 1] = rtDW.Erosion1_ONE_PAD_IMG_DW[rtB.ky];
        rtB.idx--;
        rtB.ky--;
        rtB.lineOff = 1;
        while (rtB.lineOff < rtB.ku) {
          rtDW.Erosion1_HBUF_DW[rtB.idx - 1] = fmin
            (rtDW.Erosion1_HBUF_DW[rtB.idx], rtDW.Erosion1_ONE_PAD_IMG_DW[rtB.ky]);
          rtB.idx--;
          rtB.ky--;
          rtB.lineOff++;
        }

        rtB.iter++;
      }

      rtB.numEleTmp = 0;
      while (rtB.numEleTmp < 546) {
        rtDW.Erosion_TWO_PAD_IMG_DW[rtB.outIdx] = fmin(rtDW.Erosion1_GBUF_DW
          [(rtB.idx + rtB.gOffset) - 1], rtDW.Erosion1_HBUF_DW[rtB.idx +
          rtB.hOffset]);
        rtB.idx++;
        rtB.outIdx++;
        rtB.numEleTmp++;
      }

      rtB.ky += 554;
      rtB.outIdx += 7;
      rtB.col++;
    }
  } else {
    rtB.numIter = div_nzp_s32(972, rtDW.Erosion1_NUMNONZ_DW[0]);
    rtB.lineOff = div_nzp_s32(rtDW.Erosion1_ERODE_OFF_DW[0], 553);
    rtB.gOffset = rtDW.Erosion1_NUMNONZ_DW[0] + rtB.lineOff;
    rtB.hOffset = rtB.lineOff;
    rtB.ky = rtDW.Erosion1_ERODE_OFF_DW[0] - rtB.lineOff * 553;
    rtB.lastBlockCol = rtB.numIter * rtDW.Erosion1_NUMNONZ_DW[0];
    rtB.lineOff = 0;
    while (rtB.lineOff < rtB.ku) {
      rtDW.Erosion1_GBUF_DW[rtB.lineOff] = (rtInf);
      rtB.lineOff++;
    }

    rtB.numEleTmp = (rtB.numIter + 1) * rtDW.Erosion1_NUMNONZ_DW[0];
    rtB.lineOff = rtB.numEleTmp;
    while (rtB.lineOff < rtB.numEleTmp + rtB.ku) {
      rtDW.Erosion1_GBUF_DW[rtB.lineOff] = (rtInf);
      rtB.lineOff++;
    }

    rtB.lineOff = 0;
    while (rtB.lineOff < rtB.ku) {
      rtDW.Erosion1_HBUF_DW[rtB.lineOff] = (rtInf);
      rtB.lineOff++;
    }

    rtB.lineOff = rtB.numEleTmp;
    while (rtB.lineOff < rtB.numEleTmp + rtB.ku) {
      rtDW.Erosion1_HBUF_DW[rtB.lineOff] = (rtInf);
      rtB.lineOff++;
    }

    rtB.numEleTmp = 0;
    while (rtB.numEleTmp < 546) {
      rtB.idx = rtB.ku;
      rtB.iter = 0;
      while (rtB.iter < rtB.numIter) {
        rtDW.Erosion1_GBUF_DW[rtB.idx] = rtDW.Erosion1_ONE_PAD_IMG_DW[rtB.ky];
        rtB.idx++;
        rtB.ky += 553;
        rtB.lineOff = 1;
        while (rtB.lineOff < rtB.ku) {
          rtDW.Erosion1_GBUF_DW[rtB.idx] = fmin(rtDW.Erosion1_GBUF_DW[rtB.idx -
            1], rtDW.Erosion1_ONE_PAD_IMG_DW[rtB.ky]);
          rtB.idx++;
          rtB.ky += 553;
          rtB.lineOff++;
        }

        rtB.iter++;
      }

      if (rtB.lastBlockCol <= 972) {
        rtDW.Erosion1_GBUF_DW[rtB.idx] = rtDW.Erosion1_ONE_PAD_IMG_DW[rtB.ky];
        rtB.idx++;
        rtB.ky += 553;
        rtB.iter = rtB.lastBlockCol + rtB.ku;
        rtB.lineOff = rtB.lastBlockCol + 1;
        while (rtB.lineOff < rtB.iter) {
          if (rtB.lineOff < 972) {
            rtDW.Erosion1_GBUF_DW[rtB.idx] = fmin(rtDW.Erosion1_GBUF_DW[rtB.idx
              - 1], rtDW.Erosion1_ONE_PAD_IMG_DW[rtB.ky]);
          } else {
            rtDW.Erosion1_GBUF_DW[rtB.idx] = rtDW.Erosion1_GBUF_DW[rtB.idx - 1];
          }

          rtB.idx++;
          rtB.ky += 553;
          rtB.lineOff++;
        }
      }

      rtB.ky -= 553;
      if (rtB.lastBlockCol <= 972) {
        rtB.lineOff = 1;
        while (rtB.lineOff - 1 < rtB.ku) {
          if ((rtB.ku - rtB.lineOff) + rtB.lastBlockCol < 972) {
            rtDW.Erosion1_HBUF_DW[rtB.idx - 1] = fmin
              (rtDW.Erosion1_HBUF_DW[rtB.idx],
               rtDW.Erosion1_ONE_PAD_IMG_DW[rtB.ky]);
          } else {
            rtDW.Erosion1_HBUF_DW[rtB.idx - 1] = rtDW.Erosion1_HBUF_DW[rtB.idx];
          }

          rtB.idx--;
          rtB.ky -= 553;
          rtB.lineOff++;
        }
      }

      rtB.iter = 0;
      while (rtB.iter < rtB.numIter) {
        rtDW.Erosion1_HBUF_DW[rtB.idx - 1] = rtDW.Erosion1_ONE_PAD_IMG_DW[rtB.ky];
        rtB.idx--;
        rtB.ky -= 553;
        rtB.lineOff = 1;
        while (rtB.lineOff < rtB.ku) {
          rtDW.Erosion1_HBUF_DW[rtB.idx - 1] = fmin
            (rtDW.Erosion1_HBUF_DW[rtB.idx], rtDW.Erosion1_ONE_PAD_IMG_DW[rtB.ky]);
          rtB.idx--;
          rtB.ky -= 553;
          rtB.lineOff++;
        }

        rtB.iter++;
      }

      rtB.col = 0;
      while (rtB.col < 972) {
        rtDW.Erosion_TWO_PAD_IMG_DW[rtB.outIdx] = fmin(rtDW.Erosion1_GBUF_DW
          [(rtB.idx + rtB.gOffset) - 1], rtDW.Erosion1_HBUF_DW[rtB.idx +
          rtB.hOffset]);
        rtB.idx++;
        rtB.outIdx += 553;
        rtB.col++;
      }

      rtB.ky += 554;
      rtB.outIdx += -537515;
      rtB.numEleTmp++;
    }
  }

  rtB.numIter = rtDW.Erosion1_NUMNONZ_DW[0];
  rtB.centerRow = 0;
  rtB.ku = rtDW.Erosion1_NUMNONZ_DW[1];
  rtB.ky = 3;
  rtB.outIdx = 0;
  if (rtDW.Erosion1_STREL_DW[1] == 0) {
    rtB.col = 0;
    while (rtB.col < 960) {
      rtB.numEleTmp = 3;
      while (rtB.numEleTmp < 543) {
        rtB.Clock = rtDW.Erosion_TWO_PAD_IMG_DW[rtB.ky +
          rtDW.Erosion1_ERODE_OFF_DW[rtB.numIter]];
        rtB.lineOff = 1;
        while (rtB.lineOff < rtB.ku) {
          if (rtDW.Erosion_TWO_PAD_IMG_DW[rtDW.Erosion1_ERODE_OFF_DW[rtB.lineOff
              + rtB.numIter] + rtB.ky] < rtB.Clock) {
            rtB.Clock =
              rtDW.Erosion_TWO_PAD_IMG_DW[rtDW.Erosion1_ERODE_OFF_DW[rtB.lineOff
              + rtB.numIter] + rtB.ky];
          }

          rtB.lineOff++;
        }

        rtB.Erosion1[rtB.outIdx] = rtB.Clock;
        rtB.ky++;
        rtB.outIdx++;
        rtB.numEleTmp++;
      }

      rtB.ky += 13;
      rtB.col++;
    }
  } else if (rtDW.Erosion1_STREL_DW[1] == 1) {
    rtB.numIter = div_nzp_s32(540, rtDW.Erosion1_NUMNONZ_DW[1]);
    rtB.ky = div_nzp_s32(rtDW.Erosion1_ERODE_OFF_DW[rtDW.Erosion1_NUMNONZ_DW[0]],
                         553) * 553;
    rtB.lineOff = rtDW.Erosion1_ERODE_OFF_DW[rtDW.Erosion1_NUMNONZ_DW[0]] -
      rtB.ky;
    rtB.gOffset = rtDW.Erosion1_NUMNONZ_DW[1] + rtB.lineOff;
    rtB.hOffset = rtB.lineOff;
    rtB.ky += 3;
    rtB.lastBlockCol = rtB.numIter * rtDW.Erosion1_NUMNONZ_DW[1] + 3;
    rtB.lineOff = 0;
    while (rtB.lineOff < rtB.ku) {
      rtDW.Erosion1_GBUF_DW[rtB.lineOff] = (rtInf);
      rtB.lineOff++;
    }

    rtB.numEleTmp = (rtB.numIter + 1) * rtDW.Erosion1_NUMNONZ_DW[1];
    rtB.lineOff = rtB.numEleTmp;
    while (rtB.lineOff < rtB.numEleTmp + rtB.ku) {
      rtDW.Erosion1_GBUF_DW[rtB.lineOff] = (rtInf);
      rtB.lineOff++;
    }

    rtB.lineOff = 0;
    while (rtB.lineOff < rtB.ku) {
      rtDW.Erosion1_HBUF_DW[rtB.lineOff] = (rtInf);
      rtB.lineOff++;
    }

    rtB.lineOff = rtB.numEleTmp;
    while (rtB.lineOff < rtB.numEleTmp + rtB.ku) {
      rtDW.Erosion1_HBUF_DW[rtB.lineOff] = (rtInf);
      rtB.lineOff++;
    }

    rtB.col = 0;
    while (rtB.col < 960) {
      rtB.idx = rtB.ku;
      rtB.iter = 0;
      while (rtB.iter < rtB.numIter) {
        rtDW.Erosion1_GBUF_DW[rtB.idx] = rtDW.Erosion_TWO_PAD_IMG_DW[rtB.ky];
        rtB.idx++;
        rtB.ky++;
        rtB.lineOff = 1;
        while (rtB.lineOff < rtB.ku) {
          rtDW.Erosion1_GBUF_DW[rtB.idx] = fmin(rtDW.Erosion1_GBUF_DW[rtB.idx -
            1], rtDW.Erosion_TWO_PAD_IMG_DW[rtB.ky]);
          rtB.idx++;
          rtB.ky++;
          rtB.lineOff++;
        }

        rtB.iter++;
      }

      if (rtB.lastBlockCol <= 543) {
        rtDW.Erosion1_GBUF_DW[rtB.idx] = rtDW.Erosion_TWO_PAD_IMG_DW[rtB.ky];
        rtB.idx++;
        rtB.ky++;
        rtB.iter = rtB.lastBlockCol + rtB.ku;
        rtB.lineOff = rtB.lastBlockCol + 1;
        while (rtB.lineOff < rtB.iter) {
          if (rtB.lineOff < 543) {
            rtDW.Erosion1_GBUF_DW[rtB.idx] = fmin(rtDW.Erosion1_GBUF_DW[rtB.idx
              - 1], rtDW.Erosion_TWO_PAD_IMG_DW[rtB.ky]);
          } else {
            rtDW.Erosion1_GBUF_DW[rtB.idx] = rtDW.Erosion1_GBUF_DW[rtB.idx - 1];
          }

          rtB.idx++;
          rtB.ky++;
          rtB.lineOff++;
        }
      }

      rtB.ky--;
      if (rtB.lastBlockCol <= 543) {
        rtB.lineOff = 1;
        while (rtB.lineOff - 1 < rtB.ku) {
          if ((rtB.ku - rtB.lineOff) + rtB.lastBlockCol < 543) {
            rtDW.Erosion1_HBUF_DW[rtB.idx - 1] = fmin
              (rtDW.Erosion1_HBUF_DW[rtB.idx],
               rtDW.Erosion_TWO_PAD_IMG_DW[rtB.ky]);
          } else {
            rtDW.Erosion1_HBUF_DW[rtB.idx - 1] = rtDW.Erosion1_HBUF_DW[rtB.idx];
          }

          rtB.idx--;
          rtB.ky--;
          rtB.lineOff++;
        }
      }

      rtB.iter = 0;
      while (rtB.iter < rtB.numIter) {
        rtDW.Erosion1_HBUF_DW[rtB.idx - 1] = rtDW.Erosion_TWO_PAD_IMG_DW[rtB.ky];
        rtB.idx--;
        rtB.ky--;
        rtB.lineOff = 1;
        while (rtB.lineOff < rtB.ku) {
          rtDW.Erosion1_HBUF_DW[rtB.idx - 1] = fmin
            (rtDW.Erosion1_HBUF_DW[rtB.idx], rtDW.Erosion_TWO_PAD_IMG_DW[rtB.ky]);
          rtB.idx--;
          rtB.ky--;
          rtB.lineOff++;
        }

        rtB.iter++;
      }

      rtB.numEleTmp = 3;
      while (rtB.numEleTmp < 543) {
        rtB.Erosion1[rtB.outIdx] = fmin(rtDW.Erosion1_GBUF_DW[(rtB.idx +
          rtB.gOffset) - 1], rtDW.Erosion1_HBUF_DW[rtB.idx + rtB.hOffset]);
        rtB.idx++;
        rtB.outIdx++;
        rtB.numEleTmp++;
      }

      rtB.ky += 554;
      rtB.col++;
    }
  } else {
    rtB.numIter = div_nzp_s32(963, rtDW.Erosion1_NUMNONZ_DW[1]);
    rtB.lineOff = div_nzp_s32
      (rtDW.Erosion1_ERODE_OFF_DW[rtDW.Erosion1_NUMNONZ_DW[0]], 553);
    rtB.gOffset = (rtDW.Erosion1_NUMNONZ_DW[1] + rtB.lineOff) - 3;
    rtB.hOffset = rtB.lineOff - 3;
    rtB.ky = (rtDW.Erosion1_ERODE_OFF_DW[rtDW.Erosion1_NUMNONZ_DW[0]] -
              rtB.lineOff * 553) + 1662;
    rtB.lastBlockCol = rtB.numIter * rtDW.Erosion1_NUMNONZ_DW[1];
    rtB.lineOff = 0;
    while (rtB.lineOff < rtB.ku) {
      rtDW.Erosion1_GBUF_DW[rtB.lineOff] = (rtInf);
      rtB.lineOff++;
    }

    rtB.numEleTmp = (rtB.numIter + 1) * rtDW.Erosion1_NUMNONZ_DW[1];
    rtB.lineOff = rtB.numEleTmp;
    while (rtB.lineOff < rtB.numEleTmp + rtB.ku) {
      rtDW.Erosion1_GBUF_DW[rtB.lineOff] = (rtInf);
      rtB.lineOff++;
    }

    rtB.lineOff = 0;
    while (rtB.lineOff < rtB.ku) {
      rtDW.Erosion1_HBUF_DW[rtB.lineOff] = (rtInf);
      rtB.lineOff++;
    }

    rtB.lineOff = rtB.numEleTmp;
    while (rtB.lineOff < rtB.numEleTmp + rtB.ku) {
      rtDW.Erosion1_HBUF_DW[rtB.lineOff] = (rtInf);
      rtB.lineOff++;
    }

    rtB.numEleTmp = 3;
    while (rtB.numEleTmp < 543) {
      rtB.idx = rtB.ku;
      rtB.iter = 0;
      while (rtB.iter < rtB.numIter) {
        rtDW.Erosion1_GBUF_DW[rtB.idx] = rtDW.Erosion_TWO_PAD_IMG_DW[rtB.ky];
        rtB.idx++;
        rtB.ky += 553;
        rtB.lineOff = 1;
        while (rtB.lineOff < rtB.ku) {
          rtDW.Erosion1_GBUF_DW[rtB.idx] = fmin(rtDW.Erosion1_GBUF_DW[rtB.idx -
            1], rtDW.Erosion_TWO_PAD_IMG_DW[rtB.ky]);
          rtB.idx++;
          rtB.ky += 553;
          rtB.lineOff++;
        }

        rtB.iter++;
      }

      if (rtB.lastBlockCol <= 960) {
        rtDW.Erosion1_GBUF_DW[rtB.idx] = rtDW.Erosion_TWO_PAD_IMG_DW[rtB.ky];
        rtB.idx++;
        rtB.ky += 553;
        rtB.iter = rtB.lastBlockCol + rtB.ku;
        rtB.lineOff = rtB.lastBlockCol + 1;
        while (rtB.lineOff < rtB.iter) {
          if (rtB.lineOff < 960) {
            rtDW.Erosion1_GBUF_DW[rtB.idx] = fmin(rtDW.Erosion1_GBUF_DW[rtB.idx
              - 1], rtDW.Erosion_TWO_PAD_IMG_DW[rtB.ky]);
          } else {
            rtDW.Erosion1_GBUF_DW[rtB.idx] = rtDW.Erosion1_GBUF_DW[rtB.idx - 1];
          }

          rtB.idx++;
          rtB.ky += 553;
          rtB.lineOff++;
        }
      }

      rtB.ky -= 553;
      if (rtB.lastBlockCol <= 960) {
        rtB.lineOff = 1;
        while (rtB.lineOff - 1 < rtB.ku) {
          if ((rtB.ku - rtB.lineOff) + rtB.lastBlockCol < 960) {
            rtDW.Erosion1_HBUF_DW[rtB.idx - 1] = fmin
              (rtDW.Erosion1_HBUF_DW[rtB.idx],
               rtDW.Erosion_TWO_PAD_IMG_DW[rtB.ky]);
          } else {
            rtDW.Erosion1_HBUF_DW[rtB.idx - 1] = rtDW.Erosion1_HBUF_DW[rtB.idx];
          }

          rtB.idx--;
          rtB.ky -= 553;
          rtB.lineOff++;
        }
      }

      rtB.iter = 0;
      while (rtB.iter < rtB.numIter) {
        rtDW.Erosion1_HBUF_DW[rtB.idx - 1] = rtDW.Erosion_TWO_PAD_IMG_DW[rtB.ky];
        rtB.idx--;
        rtB.ky -= 553;
        rtB.lineOff = 1;
        while (rtB.lineOff < rtB.ku) {
          rtDW.Erosion1_HBUF_DW[rtB.idx - 1] = fmin
            (rtDW.Erosion1_HBUF_DW[rtB.idx], rtDW.Erosion_TWO_PAD_IMG_DW[rtB.ky]);
          rtB.idx--;
          rtB.ky -= 553;
          rtB.lineOff++;
        }

        rtB.iter++;
      }

      rtB.col = 0;
      while (rtB.col < 960) {
        rtB.Erosion1[rtB.outIdx] = fmin(rtDW.Erosion1_GBUF_DW[(rtB.idx +
          rtB.gOffset) - 1], rtDW.Erosion1_HBUF_DW[rtB.idx + rtB.hOffset]);
        rtB.idx++;
        rtB.outIdx += 540;
        rtB.col++;
      }

      rtB.ky += 554;
      rtB.outIdx += -518399;
      rtB.numEleTmp++;
    }
  }

  // End of S-Function (svipmorphop): '<Root>/Erosion1'

  // MATLAB Function: '<Root>/A2'
  A1(rtB.Erosion1, rtB.int_img, &rtB.sf_A2);

  // Clock: '<Root>/Clock'
  rtB.Clock = rtM->Timing.t[0];

  // Outputs for Enabled SubSystem: '<Root>/Particle Update' incorporates:
  //   EnablePort: '<S13>/Enable'

  // RelationalOperator: '<S7>/Compare' incorporates:
  //   Constant: '<S7>/Constant'

  if ((int32_T)(rtB.Clock > rtP.Constant_Value_ek) > 0) {
    if (!rtDW.ParticleUpdate_MODE) {
      rtDW.ParticleUpdate_MODE = true;
    }
  } else {
    if (rtDW.ParticleUpdate_MODE) {
      rtDW.ParticleUpdate_MODE = false;
    }
  }

  // End of RelationalOperator: '<S7>/Compare'
  if (rtDW.ParticleUpdate_MODE) {
    // MATLAB Function: '<S13>/MATLAB Function2'
    // coder.extrinsic('imshow');
    // MATLAB Function 'Particle Update/MATLAB Function2': '<S45>:1'
    // '<S45>:1:8' N = 200;
    //  sample population
    // '<S45>:1:9' filter_size = 2;
    // '<S45>:1:10' saliency_map = {A1, A2};
    memcpy(&rtB.b.f1[0], &rtB.int_img_m[0], 519901U * sizeof(real_T));
    memcpy(&rtB.c.f1[0], &rtB.int_img[0], 519901U * sizeof(real_T));

    // '<S45>:1:11' Eta = 1;
    // '<S45>:1:15' W_pre = X_P(:, 3, 2);
    //  weights of old sample
    // '<S45>:1:16' S_new = zeros(N, 3, 2);
    //  initial new sample container
    // '<S45>:1:18' s_k_pre = randsample(N,N,true,W_pre);
    randsample(&rtDW.X_P[1000], rtB.s_k_pre);

    //  Sampling from previous state
    // disp(s_k_pre);
    // '<S45>:1:22' for i = 1:N
    for (rtB.lineOff = 0; rtB.lineOff < 200; rtB.lineOff++) {
      // '<S45>:1:23' index = s_k_pre(i);
      // '<S45>:1:24' p_top_left = reshape(X_P(index, 1, :), [1 2]);
      rtB.ku = (int32_T)rtB.s_k_pre[rtB.lineOff];

      //  get sample position
      // '<S45>:1:26' s_v = reshape(X_P(index, 2, :),[1 2]);
      rtB.ky = (int32_T)rtB.s_k_pre[rtB.lineOff];

      //  get sample velocity
      // '<S45>:1:27' s_width = X_P(index, 3, 1);
      rtB.weight_sum = rtDW.X_P[(int32_T)rtB.s_k_pre[rtB.lineOff] + 399];

      //  get sample width
      // '<S45>:1:29' p_inrange_1 = [1, 1] < p_top_left;
      // '<S45>:1:30' p_inrange_2 = p_top_left <= [540 - s_width, 960 - s_width]; 
      // '<S45>:1:31' if sum([p_inrange_1 p_inrange_2]) ~= 4
      rtB.outIdx = (int32_T)rtB.s_k_pre[rtB.lineOff];
      rtB.numEleTmp = (int32_T)rtB.s_k_pre[rtB.lineOff];
      rtB.p_top_left[0] = rtDW.X_P[rtB.ku - 1];
      rtB.s_v[0] = rtDW.X_P[rtB.ky + 199];
      rtB.bv0[0] = (1.0 < rtDW.X_P[rtB.outIdx - 1]);
      rtB.bv0[2] = (rtDW.X_P[rtB.numEleTmp - 1] <= 540.0 - rtDW.X_P[(int32_T)
                    rtB.s_k_pre[rtB.lineOff] + 399]);
      rtB.p_top_left[1] = rtDW.X_P[rtB.ku + 599];
      rtB.s_v[1] = rtDW.X_P[rtB.ky + 799];
      rtB.bv0[1] = (1.0 < rtDW.X_P[rtB.outIdx + 599]);
      rtB.bv0[3] = (rtDW.X_P[rtB.numEleTmp + 599] <= 960.0 - rtDW.X_P[(int32_T)
                    rtB.s_k_pre[rtB.lineOff] + 399]);
      if (sum_j(rtB.bv0) != 4.0) {
        // '<S45>:1:32' p_top_left = [rand * (540 - s_width), rand * (960 - s_width)]; 
        rtB.filter_weights = rand_oi();
        rtB.filter_weights_idx_0 = rand_oi();
        rtB.p_top_left[0] = (540.0 - rtDW.X_P[(int32_T)rtB.s_k_pre[rtB.lineOff]
                             + 399]) * rtB.filter_weights;
        rtB.p_top_left[1] = (960.0 - rtDW.X_P[(int32_T)rtB.s_k_pre[rtB.lineOff]
                             + 399]) * rtB.filter_weights_idx_0;
      }

      // '<S45>:1:35' w_new = 0;
      // '<S45>:1:36' for j = 1:filter_size
      // '<S45>:1:37' w_new = w_new + Eta * F_W(j) * eval_at_p(p_top_left, s_width, saliency_map{j}); 
      rtB.x_width = eval_at_p_b(rtB.p_top_left, rtDW.X_P[(int32_T)
        rtB.s_k_pre[rtB.lineOff] + 399], rtB.b.f1) * rtDW.F_W[0] + eval_at_p_b
        (rtB.p_top_left, rtDW.X_P[(int32_T)rtB.s_k_pre[rtB.lineOff] + 399],
         rtB.c.f1) * rtDW.F_W[1];

      // '<S45>:1:40' if w_new < 1
      if (rtB.x_width < 1.0) {
        // '<S45>:1:41' p_top_left = [rand * (540 - s_width), rand * (960 - s_width)]; 
        rtB.filter_weights = rand_oi();
        rtB.filter_weights_idx_0 = rand_oi();
        rtB.p_top_left[0] = (540.0 - rtDW.X_P[(int32_T)rtB.s_k_pre[rtB.lineOff]
                             + 399]) * rtB.filter_weights;
        rtB.p_top_left[1] = (960.0 - rtDW.X_P[(int32_T)rtB.s_k_pre[rtB.lineOff]
                             + 399]) * rtB.filter_weights_idx_0;

        // '<S45>:1:42' s_v = [0 0];
        rtB.s_v[0] = 0.0;
        rtB.s_v[1] = 0.0;

        // '<S45>:1:43' w_new = w_new + 0.01;
        rtB.x_width += 0.01;

        // '<S45>:1:44' s_width = 50;
        rtB.weight_sum = 50.0;
      }

      // '<S45>:1:47' s_v_new = s_v + [randn, randn] * 5;
      rtB.filter_weights = randn();
      rtB.filter_weights_idx_0 = randn();
      rtB.s_v[0] += rtB.filter_weights * 5.0;
      rtB.s_v[1] += rtB.filter_weights_idx_0 * 5.0;

      //  update velocity
      // '<S45>:1:48' s_width_new = min(max(s_width + randn * 5, 30), 150);
      rtB.weight_sum += randn() * 5.0;

      // s_width_new = s_width;
      // '<S45>:1:51' p_new = p_top_left + s_v_new + randn;
      rtB.filter_weights = randn();

      // '<S45>:1:53' S_new(i, :, :) = [p_new; s_v_new; s_width_new, w_new];
      rtB.x[2] = fmin(fmax(rtB.weight_sum, 30.0), 150.0);
      rtB.x[5] = rtB.x_width;
      for (rtB.ku = 0; rtB.ku < 2; rtB.ku++) {
        rtB.x_width = (rtB.p_top_left[rtB.ku] + rtB.s_v[rtB.ku]) +
          rtB.filter_weights;
        rtB.x[3 * rtB.ku] = rtB.x_width;
        rtB.x[1 + 3 * rtB.ku] = rtB.s_v[rtB.ku];
        rtB.S_new[rtB.lineOff + 600 * rtB.ku] = rtB.x[3 * rtB.ku];
        rtB.S_new[(rtB.lineOff + 600 * rtB.ku) + 200] = rtB.x[3 * rtB.ku + 1];
        rtB.S_new[(rtB.lineOff + 600 * rtB.ku) + 400] = rtB.x[3 * rtB.ku + 2];
        rtB.p_top_left[rtB.ku] = rtB.x_width;
      }
    }

    // '<S45>:1:58' [~, max_index] = max(X_P(:, 3, 2));
    rtB.lineOff = 1;
    rtB.x_width = rtDW.X_P[1000];
    if (rtIsNaN(rtDW.X_P[1000])) {
      rtB.ky = 2;
      exitg1 = false;
      while ((!exitg1) && (rtB.ky < 201)) {
        rtB.lineOff = rtB.ky;
        if (!rtIsNaN(rtDW.X_P[rtB.ky + 999])) {
          rtB.x_width = rtDW.X_P[rtB.ky + 999];
          rtB.centerRow = rtB.ky - 1;
          exitg1 = true;
        } else {
          rtB.ky++;
        }
      }
    }

    if (rtB.lineOff < 200) {
      while (rtB.lineOff + 1 < 201) {
        if (rtDW.X_P[1000 + rtB.lineOff] > rtB.x_width) {
          rtB.x_width = rtDW.X_P[1000 + rtB.lineOff];
          rtB.centerRow = rtB.lineOff;
        }

        rtB.lineOff++;
      }
    }

    // '<S45>:1:58' ~
    // '<S45>:1:60' x_bar = reshape(X_P(max_index, :, :), [3 2]);
    for (rtB.ku = 0; rtB.ku < 2; rtB.ku++) {
      rtB.x[3 * rtB.ku] = rtDW.X_P[600 * rtB.ku + rtB.centerRow];
      rtB.x[1 + 3 * rtB.ku] = rtDW.X_P[(600 * rtB.ku + rtB.centerRow) + 200];
      rtB.x[2 + 3 * rtB.ku] = rtDW.X_P[(600 * rtB.ku + rtB.centerRow) + 400];
    }

    // '<S45>:1:61' X_P = S_new;
    memcpy(&rtDW.X_P[0], &rtB.S_new[0], 1200U * sizeof(real_T));
    for (rtB.ku = 0; rtB.ku < 6; rtB.ku++) {
      rtB.x_bar[rtB.ku] = rtB.x[rtB.ku];
    }

    // End of MATLAB Function: '<S13>/MATLAB Function2'
  }

  // End of Outputs for SubSystem: '<Root>/Particle Update'

  // MATLAB Function: '<S10>/MATLAB Function1'
  // MATLAB Function 'Monitor/MATLAB Function1': '<S29>:1'
  //  img_inputs = {img1, img2};
  // '<S29>:1:7' A_inputs = {A1, A2};
  memcpy(&rtB.b.f1[0], &rtB.int_img_m[0], 519901U * sizeof(real_T));
  memcpy(&rtB.c.f1[0], &rtB.int_img[0], 519901U * sizeof(real_T));

  // '<S29>:1:10' filter_size = 2;
  // '<S29>:1:11' filter_weights = zeros(filter_size, 1);
  // '<S29>:1:13' theta_min = 60;
  // '<S29>:1:15' x_position = result(1,:);
  // '<S29>:1:16' x_width = result(3,1);
  rtB.x_width = rtB.x_bar[2];

  // '<S29>:1:17' x_weight = result(3,2);
  // '<S29>:1:20' if x_weight > theta_min
  if (rtB.x_bar[5] > 60.0) {
    // '<S29>:1:21' weight_sum = 0;
    // '<S29>:1:22' for i = 1:filter_size
    // disp('filter____weights:');
    // disp(filter_weights);
    // '<S29>:1:32' for i = 1:filter_size
    // '<S29>:1:23' avg_val = mean(mean(A_inputs{i}));
    // disp('avg_val of filter');
    // disp(avg_val);
    // disp('eval:');
    // disp(eval_at_p(A_inputs{i}, x_position, x_width));
    // '<S29>:1:28' filter_weights(i) = max(0.01, eval_at_p(A_inputs{i}, x_position, x_width)-avg_val); 
    rtB.s_v[0] = rtB.x_bar[0];
    rtB.s_v[1] = rtB.x_bar[3];
    mean(rtB.b.f1, rtB.dv0);
    rtB.filter_weights = fmax(0.01, eval_at_p(rtB.b.f1, rtB.s_v, rtB.x_bar[2]) -
      mean_c(rtB.dv0));

    // '<S29>:1:33' weight_sum = weight_sum + filter_weights(i);
    rtB.weight_sum = rtB.filter_weights;
    rtB.filter_weights_idx_0 = rtB.filter_weights;

    // '<S29>:1:23' avg_val = mean(mean(A_inputs{i}));
    // disp('avg_val of filter');
    // disp(avg_val);
    // disp('eval:');
    // disp(eval_at_p(A_inputs{i}, x_position, x_width));
    // '<S29>:1:28' filter_weights(i) = max(0.01, eval_at_p(A_inputs{i}, x_position, x_width)-avg_val); 
    rtB.s_v[0] = rtB.x_bar[0];
    rtB.s_v[1] = rtB.x_bar[3];
    mean(rtB.c.f1, rtB.dv0);
    rtB.filter_weights = fmax(0.01, eval_at_p(rtB.c.f1, rtB.s_v, rtB.x_bar[2]) -
      mean_c(rtB.dv0));

    // '<S29>:1:33' weight_sum = weight_sum + filter_weights(i);
    rtB.weight_sum += rtB.filter_weights;

    // '<S29>:1:35' for i = 1:filter_size
    // '<S29>:1:36' F_W(i) = filter_weights(i)/weight_sum;
    rtDW.F_W[0] = rtB.filter_weights_idx_0 / rtB.weight_sum;

    // '<S29>:1:36' F_W(i) = filter_weights(i)/weight_sum;
    rtDW.F_W[1] = rtB.filter_weights / rtB.weight_sum;

    // disp('filter weights:');
    // disp([F_W(1), F_W(2)]);
  } else {
    // '<S29>:1:40' else
    // disp('filter weight reset');
    // '<S29>:1:42' for i = 1:filter_size
    // '<S29>:1:43' F_W(i) = 1/filter_size;
    rtDW.F_W[0] = 0.5;

    // '<S29>:1:43' F_W(i) = 1/filter_size;
    rtDW.F_W[1] = 0.5;

    // '<S29>:1:45' x_width = 0;
    rtB.x_width = 0.0;
  }

  // RGB = insertShape(img, 'FilledCircle', [int16([result(1,2), result(1,1)]),10]); 
  // disp(x_width);
  // RGB = img;
  // '<S29>:1:53' RGB = insertShape(img, 'Rectangle', int16([result(1,2), result(1,1), x_width, x_width]),'LineWidth',5); 
  iobj_0 = &rtDW.h3111;
  if (!rtDW.h3111_not_empty) {
    rtDW.h3111.isInitialized = 0;

    // System object Constructor function: vision.ShapeInserter
    iobj_0->cSFunObject.P0_RTP_LINEWIDTH = 1;
    rtDW.h3111.LineWidth = 1.0;
    rtDW.h3111_not_empty = true;
    rtDW.h3111.isInitialized = 1;
  }

  if (5.0 != rtDW.h3111.LineWidth) {
    rtDW.h3111.cSFunObject.P0_RTP_LINEWIDTH = 5;
    rtDW.h3111.LineWidth = 5.0;
  }

  memcpy(&rtB.RGB[0], &rtB.VectorConcatenate[0], 1555200U * sizeof(uint8_T));
  rtB.filter_weights = rt_roundd_snf(rtB.x_bar[3]);
  if (rtB.filter_weights < 32768.0) {
    if (rtB.filter_weights >= -32768.0) {
      rtB.iv0[0] = (int16_T)rtB.filter_weights;
    } else {
      rtB.iv0[0] = -32768;
    }
  } else {
    rtB.iv0[0] = 32767;
  }

  rtB.filter_weights = rt_roundd_snf(rtB.x_bar[0]);
  if (rtB.filter_weights < 32768.0) {
    if (rtB.filter_weights >= -32768.0) {
      rtB.iv0[1] = (int16_T)rtB.filter_weights;
    } else {
      rtB.iv0[1] = -32768;
    }
  } else {
    rtB.iv0[1] = 32767;
  }

  rtB.filter_weights = rt_roundd_snf(rtB.x_width);
  if (rtB.filter_weights < 32768.0) {
    if (rtB.filter_weights >= -32768.0) {
      rtB.iv0[2] = (int16_T)rtB.filter_weights;
    } else {
      rtB.iv0[2] = -32768;
    }
  } else {
    rtB.iv0[2] = 32767;
  }

  rtB.filter_weights = rt_roundd_snf(rtB.x_width);
  if (rtB.filter_weights < 32768.0) {
    if (rtB.filter_weights >= -32768.0) {
      rtB.iv0[3] = (int16_T)rtB.filter_weights;
    } else {
      rtB.iv0[3] = -32768;
    }
  } else {
    rtB.iv0[3] = 32767;
  }

  SystemCore_step(&rtDW.h3111, rtB.RGB, rtB.iv0);

  // BusAssignment: '<S11>/Bus Assignment' incorporates:
  //   Constant: '<S42>/Constant'
  //   MATLAB Function: '<S10>/MATLAB Function1'

  // '<S29>:1:56' W1 = double(F_W(1));
  // '<S29>:1:57' W2 = double(F_W(2));
  rtB.BusAssignment = rtP.Constant_Value_e;
  rtB.BusAssignment.X = rtDW.F_W[0];
  rtB.BusAssignment.Y = rtDW.F_W[1];

  // Outputs for Atomic SubSystem: '<S11>/Publish2'
  // Start for MATLABSystem: '<S43>/SinkBlock' incorporates:
  //   MATLABSystem: '<S43>/SinkBlock'

  Pub_ros_image_filters_0706_588.publish(&rtB.BusAssignment);

  // End of Outputs for SubSystem: '<S11>/Publish2'

  // Outputs for Enabled SubSystem: '<Root>/Particle Initialization' incorporates:
  //   EnablePort: '<S12>/Enable'

  // RelationalOperator: '<S6>/Compare' incorporates:
  //   Constant: '<S6>/Constant'

  if (rtB.Clock <= rtP.Constant_Value_g) {
    if (!rtDW.ParticleInitialization_MODE) {
      rtDW.ParticleInitialization_MODE = true;
    }

    // MATLAB Function: '<S12>/MATLAB Function2'
    // MATLAB Function 'Particle Initialization/MATLAB Function2': '<S44>:1'
    // '<S44>:1:5' N = 200;
    // '<S44>:1:8' for i = 1:N
    for (rtB.lineOff = 0; rtB.lineOff < 200; rtB.lineOff++) {
      // '<S44>:1:9' weight = 1 / N;
      // '<S44>:1:10' width = 50;
      // '<S44>:1:12' X_P(i,2,:) = [0, 0];
      // '<S44>:1:13' X_P(i,3,:) = [width, weight];
      rtDW.X_P[200 + rtB.lineOff] = 0.0;
      rtDW.X_P[400 + rtB.lineOff] = 50.0;
      rtDW.X_P[rtB.lineOff + 800] = 0.0;
      rtDW.X_P[rtB.lineOff + 1000] = 0.005;

      // '<S44>:1:14' x = rand * 540;
      rtB.Clock = rand_e() * 540.0;

      // '<S44>:1:15' y = rand * 960;
      rtB.x_width = rand_e() * 960.0;

      // '<S44>:1:16' X_P(i,1,:) = [x, y];
      rtDW.X_P[rtB.lineOff] = rtB.Clock;
      rtDW.X_P[rtB.lineOff + 600] = rtB.x_width;
    }

    // End of MATLAB Function: '<S12>/MATLAB Function2'
  } else {
    if (rtDW.ParticleInitialization_MODE) {
      rtDW.ParticleInitialization_MODE = false;
    }
  }

  // End of RelationalOperator: '<S6>/Compare'
  // End of Outputs for SubSystem: '<Root>/Particle Initialization'

  // S-Function (svipscalenconvert): '<S10>/Image Data Type Conversion'
  ImageDataTypeConversion = &rtB.RGB[0];

  // MATLAB Function: '<S28>/3D_img_to_1D_img'
  //  assignString - Assign a string in a ROS message
  //
  //  MSG = assignString(blankMsg) assigns a known string value to a
  //   string field in a ROS message, and outputs the updated ROS message.
  //
  //   blankMsg (input): A bus for a 'std_msgs/String' message. This is
  //       used to initialize the output.
  //   msg (output): A bus for a 'std_msgs/String' message, with the Data
  //       field set to a string.
  // MATLAB Function 'Monitor/3-d-image2ros/3D_img_to_1D_img': '<S39>:1'
  //    Copyright 2015 The MathWorks, Inc.
  //  Note: The datatype of 'msg' has been explicitly set to the appropriate
  //  Simulink bus type ("Bus: SL_Bus_robotROSMessageUsageExample_std_msgs_String").  
  //
  //  If the model name is changed, the datatype needs to be changed as well:
  //    1) Click "Edit Data" in the Editor toolstrip to open "Ports and Data Manager" 
  //    2) Select "msg". From the "Type" dropdown, select "--- Refresh data types ---" 
  //    3) Set datatype to "Bus: SL_Bus_<modelname>_std_msgs_String"
  //  msg (std_msgs/String) consists of
  //            Data : array of type uint8
  //    Data_SL_Info : struct with fields CurrentLength, ReceivedLength
  // '<S39>:1:27' img = uint8(zeros(1, 1555200));
  memset(&rtB.img[0], 0, 1555200U * sizeof(uint8_T));

  // '<S39>:1:28' img(1:3:end) = data(:,:,1)';
  for (rtB.ku = 0; rtB.ku < 540; rtB.ku++) {
    for (rtB.lineOff = 0; rtB.lineOff < 960; rtB.lineOff++) {
      rtB.ImageDataTypeConversion[rtB.lineOff + 960 * rtB.ku] =
        ImageDataTypeConversion[540 * rtB.lineOff + rtB.ku];
    }
  }

  for (rtB.ku = 0; rtB.ku < 518400; rtB.ku++) {
    rtB.img[3 * rtB.ku] = rtB.ImageDataTypeConversion[rtB.ku];
  }

  // '<S39>:1:29' img(2:3:end) = data(:,:,2)';
  for (rtB.ku = 0; rtB.ku < 540; rtB.ku++) {
    for (rtB.lineOff = 0; rtB.lineOff < 960; rtB.lineOff++) {
      rtB.ImageDataTypeConversion[rtB.lineOff + 960 * rtB.ku] =
        ImageDataTypeConversion[(540 * rtB.lineOff + rtB.ku) + 518400];
    }
  }

  for (rtB.ku = 0; rtB.ku < 518400; rtB.ku++) {
    rtB.img[1 + 3 * rtB.ku] = rtB.ImageDataTypeConversion[rtB.ku];
  }

  // '<S39>:1:30' img(3:3:end) = data(:,:,3)';
  for (rtB.ku = 0; rtB.ku < 540; rtB.ku++) {
    for (rtB.lineOff = 0; rtB.lineOff < 960; rtB.lineOff++) {
      rtB.ImageDataTypeConversion[rtB.lineOff + 960 * rtB.ku] =
        ImageDataTypeConversion[(540 * rtB.lineOff + rtB.ku) + 1036800];
    }
  }

  for (rtB.ku = 0; rtB.ku < 518400; rtB.ku++) {
    rtB.img[2 + 3 * rtB.ku] = rtB.ImageDataTypeConversion[rtB.ku];
  }

  // End of MATLAB Function: '<S28>/3D_img_to_1D_img'

  // MATLAB Function: '<S28>/ROS_Img_Data_to_ROS_Img1' incorporates:
  //   Constant: '<S40>/Constant'

  rtB.y = rtP.Constant_Value_b;

  //  assignString - Assign a string in a ROS message
  //
  //  MSG = assignString(blankMsg) assigns a known string value to a
  //   string field in a ROS message, and outputs the updated ROS message.
  //
  //   blankMsg (input): A bus for a 'std_msgs/String' message. This is
  //       used to initialize the output.
  //   msg (output): A bus for a 'std_msgs/String' message, with the Data
  //       field set to a string.
  // MATLAB Function 'Monitor/3-d-image2ros/ROS_Img_Data_to_ROS_Img1': '<S41>:1' 
  //    Copyright 2015 The MathWorks, Inc.
  //  Note: The datatype of 'msg' has been explicitly set to the appropriate
  //  Simulink bus type ("Bus: SL_Bus_robotROSMessageUsageExample_std_msgs_String").  
  //
  //  If the model name is changed, the datatype needs to be changed as well:
  //    1) Click "Edit Data" in the Editor toolstrip to open "Ports and Data Manager" 
  //    2) Select "msg". From the "Type" dropdown, select "--- Refresh data types ---" 
  //    3) Set datatype to "Bus: SL_Bus_<modelname>_std_msgs_String"
  // '<S41>:1:23' stringToAssign = 'rgb8';
  // '<S41>:1:24' height = 540;
  // '<S41>:1:25' width = 960;
  //  msg (std_msgs/String) consists of
  //            Data : array of type uint8
  //    Data_SL_Info : struct with fields CurrentLength, ReceivedLength
  // '<S41>:1:30' msg.Encoding = uint8(stringToAssign)';
  rtB.y.Encoding[0] = 114U;
  rtB.y.Encoding[1] = 103U;
  rtB.y.Encoding[2] = 98U;
  rtB.y.Encoding[3] = 56U;

  // '<S41>:1:31' msg.Encoding_SL_Info.CurrentLength = uint32(length(stringToAssign)); 
  rtB.y.Encoding_SL_Info.CurrentLength = 4U;

  // '<S41>:1:33' msg.Data = uint8(data);
  memcpy(&rtB.y.Data[0], &rtB.img[0], 1555200U * sizeof(uint8_T));

  // '<S41>:1:34' msg.Data_SL_Info.CurrentLength = uint32(length(data));
  rtB.y.Data_SL_Info.CurrentLength = 1555200U;

  // '<S41>:1:36' msg.Height = uint32(height);
  rtB.y.Height = 540U;

  // '<S41>:1:37' msg.Width = uint32(width);
  rtB.y.Width = 960U;

  // '<S41>:1:38' msg.Step = uint32(width * 3);
  rtB.y.Step = 2880U;

  // Outputs for Atomic SubSystem: '<S10>/Publish'
  // Start for MATLABSystem: '<S30>/SinkBlock' incorporates:
  //   MATLABSystem: '<S30>/SinkBlock'

  // '<S41>:1:40' y = msg;
  Pub_ros_image_filters_0706_490.publish(&rtB.y);

  // End of Outputs for SubSystem: '<S10>/Publish'

  // Update for UnitDelay: '<Root>/Unit Delay'
  memcpy(&rtDW.UnitDelay_DSTATE[0], &rtB.ImageDataTypeConversion2[0], 518400U *
         sizeof(real_T));

  // Update absolute time for base rate
  // The "clockTick0" counts the number of times the code of this task has
  //  been executed. The absolute time is the multiplication of "clockTick0"
  //  and "Timing.stepSize0". Size of "clockTick0" ensures timer will not
  //  overflow during the application lifespan selected.

  rtM->Timing.t[0] =
    (++rtM->Timing.clockTick0) * rtM->Timing.stepSize0;

  {
    // Update absolute timer for sample time: [0.01s, 0.0s]
    // The "clockTick1" counts the number of times the code of this task has
    //  been executed. The resolution of this integer timer is 0.01, which is the step size
    //  of the task. Size of "clockTick1" ensures timer will not overflow during the
    //  application lifespan selected.

    rtM->Timing.clockTick1++;
  }
}

// Model initialize function
void ros_image_filters_0706_initialize(void)
{
  // Registration code

  // initialize non-finites
  rt_InitInfAndNaN(sizeof(real_T));

  {
    // Setup solver object
    rtsiSetSimTimeStepPtr(&rtM->solverInfo, &rtM->Timing.simTimeStep);
    rtsiSetTPtr(&rtM->solverInfo, &rtmGetTPtr(rtM));
    rtsiSetStepSizePtr(&rtM->solverInfo, &rtM->Timing.stepSize0);
    rtsiSetErrorStatusPtr(&rtM->solverInfo, (&rtmGetErrorStatus(rtM)));
    rtsiSetRTModelPtr(&rtM->solverInfo, rtM);
  }

  rtsiSetSimTimeStep(&rtM->solverInfo, MAJOR_TIME_STEP);
  rtsiSetSolverName(&rtM->solverInfo,"FixedStepDiscrete");
  rtmSetTPtr(rtM, &rtM->Timing.tArray[0]);
  rtM->Timing.stepSize0 = 0.01;

  {
    int32_T curNumNonZ;
    int32_T n;
    int32_T m;
    boolean_T isValid;
    int32_T idxOut;
    int32_T step;
    int32_T previous;
    static const char_T tmp[16] = { '/', 't', 'r', 'a', 'c', 'k', 'i', 'n', 'g',
      '_', 'r', 'e', 's', 'u', 'l', 't' };

    static const char_T tmp_0[15] = { '/', 'f', 'i', 'l', 't', 'e', 'r', '_',
      'w', 'e', 'i', 'g', 't', 'h', 's' };

    static const char_T tmp_1[13] = { '/', 'i', 'm', 'a', 'g', 'e', '_', 'b',
      'r', 'i', 'd', 'g', 'e' };

    char_T tmp_2[16];
    char_T tmp_3[14];

    // Start for Atomic SubSystem: '<Root>/Subscribe'
    // Start for MATLABSystem: '<S16>/SourceBlock'
    rtDW.obj_j.isInitialized = 0;
    rtDW.obj_j.isInitialized = 1;
    for (previous = 0; previous < 13; previous++) {
      tmp_3[previous] = tmp_1[previous];
    }

    tmp_3[13] = '\x00';
    Sub_ros_image_filters_0706_335.createSubscriber(tmp_3, MessageQueueLen);

    // End of Start for MATLABSystem: '<S16>/SourceBlock'
    // End of Start for SubSystem: '<Root>/Subscribe'
    // Start for S-Function (svipmorphop): '<Root>/Erosion'
    previous = 0;
    step = 0;
    curNumNonZ = 0;
    isValid = true;
    idxOut = 0;
    n = 0;
    while (n < 1) {
      m = 0;
      while (m < 7) {
        if (curNumNonZ == 0) {
          curNumNonZ = 1;
        } else if (curNumNonZ == 1) {
          step = idxOut - previous;
          curNumNonZ = 2;
        } else if (idxOut - previous == step) {
          curNumNonZ++;
        } else {
          isValid = false;
          m = 7;
          n = 1;
        }

        previous = idxOut;
        idxOut++;
        m++;
      }

      idxOut += 546;
      n++;
    }

    if (isValid && (curNumNonZ >= 4)) {
      if (step == 553) {
        rtDW.Erosion_STREL_DW[0] = 2;
      } else {
        rtDW.Erosion_STREL_DW[0] = (step == 1);
      }
    } else {
      rtDW.Erosion_STREL_DW[0] = 0;
    }

    curNumNonZ = 0;
    isValid = true;
    idxOut = 0;
    n = 0;
    while (n < 7) {
      m = 0;
      while (m < 1) {
        if (curNumNonZ == 0) {
          curNumNonZ = 1;
        } else if (curNumNonZ == 1) {
          step = idxOut - previous;
          curNumNonZ = 2;
        } else if (idxOut - previous == step) {
          curNumNonZ++;
        } else {
          isValid = false;
          m = 1;
          n = 7;
        }

        previous = idxOut;
        idxOut++;
        m++;
      }

      idxOut += 552;
      n++;
    }

    if (isValid && (curNumNonZ >= 4)) {
      if (step == 553) {
        rtDW.Erosion_STREL_DW[1] = 2;
      } else {
        rtDW.Erosion_STREL_DW[1] = (step == 1);
      }
    } else {
      rtDW.Erosion_STREL_DW[1] = 0;
    }

    previous = 0;
    curNumNonZ = 0;
    n = 0;
    while (n < 1) {
      for (m = 0; m < 7; m++) {
        rtDW.Erosion_ERODE_OFF_DW[previous] = m;
        curNumNonZ++;
        previous++;
      }

      n = 1;
    }

    rtDW.Erosion_NUMNONZ_DW[0] = curNumNonZ;
    curNumNonZ = 0;
    for (n = 0; n < 7; n++) {
      m = 0;
      while (m < 1) {
        rtDW.Erosion_ERODE_OFF_DW[previous] = n * 553;
        curNumNonZ++;
        previous++;
        m = 1;
      }
    }

    rtDW.Erosion_NUMNONZ_DW[1] = curNumNonZ;

    // End of Start for S-Function (svipmorphop): '<Root>/Erosion'

    // Start for S-Function (svipmorphop): '<Root>/Erosion1'
    previous = 0;
    step = 0;
    curNumNonZ = 0;
    isValid = true;
    idxOut = 0;
    n = 0;
    while (n < 1) {
      m = 0;
      while (m < 7) {
        if (curNumNonZ == 0) {
          curNumNonZ = 1;
        } else if (curNumNonZ == 1) {
          step = idxOut - previous;
          curNumNonZ = 2;
        } else if (idxOut - previous == step) {
          curNumNonZ++;
        } else {
          isValid = false;
          m = 7;
          n = 1;
        }

        previous = idxOut;
        idxOut++;
        m++;
      }

      idxOut += 546;
      n++;
    }

    if (isValid && (curNumNonZ >= 4)) {
      if (step == 553) {
        rtDW.Erosion1_STREL_DW[0] = 2;
      } else {
        rtDW.Erosion1_STREL_DW[0] = (step == 1);
      }
    } else {
      rtDW.Erosion1_STREL_DW[0] = 0;
    }

    curNumNonZ = 0;
    isValid = true;
    idxOut = 0;
    n = 0;
    while (n < 7) {
      m = 0;
      while (m < 1) {
        if (curNumNonZ == 0) {
          curNumNonZ = 1;
        } else if (curNumNonZ == 1) {
          step = idxOut - previous;
          curNumNonZ = 2;
        } else if (idxOut - previous == step) {
          curNumNonZ++;
        } else {
          isValid = false;
          m = 1;
          n = 7;
        }

        previous = idxOut;
        idxOut++;
        m++;
      }

      idxOut += 552;
      n++;
    }

    if (isValid && (curNumNonZ >= 4)) {
      if (step == 553) {
        rtDW.Erosion1_STREL_DW[1] = 2;
      } else {
        rtDW.Erosion1_STREL_DW[1] = (step == 1);
      }
    } else {
      rtDW.Erosion1_STREL_DW[1] = 0;
    }

    previous = 0;
    curNumNonZ = 0;
    n = 0;
    while (n < 1) {
      for (m = 0; m < 7; m++) {
        rtDW.Erosion1_ERODE_OFF_DW[previous] = m;
        curNumNonZ++;
        previous++;
      }

      n = 1;
    }

    rtDW.Erosion1_NUMNONZ_DW[0] = curNumNonZ;
    curNumNonZ = 0;
    for (n = 0; n < 7; n++) {
      m = 0;
      while (m < 1) {
        rtDW.Erosion1_ERODE_OFF_DW[previous] = n * 553;
        curNumNonZ++;
        previous++;
        m = 1;
      }
    }

    rtDW.Erosion1_NUMNONZ_DW[1] = curNumNonZ;

    // End of Start for S-Function (svipmorphop): '<Root>/Erosion1'

    // Start for Atomic SubSystem: '<S11>/Publish2'
    // Start for MATLABSystem: '<S43>/SinkBlock'
    rtDW.obj.isInitialized = 0;
    rtDW.obj.isInitialized = 1;
    for (previous = 0; previous < 15; previous++) {
      tmp_2[previous] = tmp_0[previous];
    }

    tmp_2[15] = '\x00';
    Pub_ros_image_filters_0706_588.createPublisher(tmp_2, MessageQueueLen);

    // End of Start for MATLABSystem: '<S43>/SinkBlock'
    // End of Start for SubSystem: '<S11>/Publish2'

    // Start for Atomic SubSystem: '<S10>/Publish'
    // Start for MATLABSystem: '<S30>/SinkBlock'
    rtDW.obj_i.isInitialized = 0;
    rtDW.obj_i.isInitialized = 1;
    for (previous = 0; previous < 16; previous++) {
      rtB.cv0[previous] = tmp[previous];
    }

    rtB.cv0[16] = '\x00';
    Pub_ros_image_filters_0706_490.createPublisher(rtB.cv0, MessageQueueLen);

    // End of Start for MATLABSystem: '<S30>/SinkBlock'
    // End of Start for SubSystem: '<S10>/Publish'

    // Start for DataStoreMemory: '<Root>/Filter Weights'
    memcpy(&rtDW.F_W[0], &rtP.FilterWeights_InitialValue[0], sizeof(real_T) <<
           3U);

    // Start for DataStoreMemory: '<Root>/Particle States'
    memcpy(&rtDW.X_P[0], &rtP.ParticleStates_InitialValue[0], 1200U * sizeof
           (real_T));

    // InitializeConditions for UnitDelay: '<Root>/Unit Delay'
    for (previous = 0; previous < 518400; previous++) {
      rtDW.UnitDelay_DSTATE[previous] = rtP.UnitDelay_InitialCondition;
    }

    // End of InitializeConditions for UnitDelay: '<Root>/Unit Delay'

    // SystemInitialize for Atomic SubSystem: '<Root>/Subscribe'
    // SystemInitialize for Enabled SubSystem: '<S16>/Enabled Subsystem'
    // SystemInitialize for Outport: '<S46>/Out1'
    rtB.In1 = rtP.Out1_Y0;

    // End of SystemInitialize for SubSystem: '<S16>/Enabled Subsystem'
    // End of SystemInitialize for SubSystem: '<Root>/Subscribe'

    // SystemInitialize for Enabled SubSystem: '<Root>/Particle Update'
    // SystemInitialize for MATLAB Function: '<S13>/MATLAB Function2'
    rtDW.method = 7U;
    rtDW.state = 1144108930U;
    rtDW.state_a[0] = 362436069U;
    rtDW.state_a[1] = 521288629U;

    // SystemInitialize for Outport: '<S13>/Result'
    for (previous = 0; previous < 6; previous++) {
      rtB.x_bar[previous] = rtP.Result_Y0;
    }

    // End of SystemInitialize for Outport: '<S13>/Result'
    // End of SystemInitialize for SubSystem: '<Root>/Particle Update'

    // SystemInitialize for Enabled SubSystem: '<Root>/Particle Initialization'
    // SystemInitialize for MATLAB Function: '<S12>/MATLAB Function2'
    rtDW.method_k = 7U;
    rtDW.state_l = 1144108930U;
    rtDW.state_j[0] = 362436069U;
    rtDW.state_j[1] = 521288629U;

    // End of SystemInitialize for SubSystem: '<Root>/Particle Initialization'
  }
}

// Model terminate function
void ros_image_filters_0706_terminate(void)
{
  // Terminate for Atomic SubSystem: '<Root>/Subscribe'
  // Start for MATLABSystem: '<S16>/SourceBlock' incorporates:
  //   Terminate for MATLABSystem: '<S16>/SourceBlock'

  if (rtDW.obj_j.isInitialized == 1) {
    rtDW.obj_j.isInitialized = 2;
  }

  // End of Start for MATLABSystem: '<S16>/SourceBlock'
  // End of Terminate for SubSystem: '<Root>/Subscribe'
  // Terminate for Atomic SubSystem: '<S11>/Publish2'
  // Start for MATLABSystem: '<S43>/SinkBlock' incorporates:
  //   Terminate for MATLABSystem: '<S43>/SinkBlock'

  if (rtDW.obj.isInitialized == 1) {
    rtDW.obj.isInitialized = 2;
  }

  // End of Start for MATLABSystem: '<S43>/SinkBlock'
  // End of Terminate for SubSystem: '<S11>/Publish2'

  // Terminate for Atomic SubSystem: '<S10>/Publish'
  // Start for MATLABSystem: '<S30>/SinkBlock' incorporates:
  //   Terminate for MATLABSystem: '<S30>/SinkBlock'

  if (rtDW.obj_i.isInitialized == 1) {
    rtDW.obj_i.isInitialized = 2;
  }

  // End of Start for MATLABSystem: '<S30>/SinkBlock'
  // End of Terminate for SubSystem: '<S10>/Publish'
}

//
// File trailer for generated code.
//
// [EOF]
//
