//
//  math_types.h
//
//  Created by Jonathan Tompson on 4/23/12.
//
//  Vectors and Matrix types are templates.
//

#pragma once

#include <stdint.h>  // For uint8_t, uint16_t, etc
#include <float.h>
#include <cmath>

#if defined(WIN32) || defined(_WIN32)
  #include <random>  // For std::tr1::mt19937, and others
#endif
#if defined(__APPLE__)
  #include <tr1/random> 
#endif

#ifndef EPSILON
  #define EPSILON (2 * FLT_EPSILON)  // 2 times machine precision for float
#endif

#ifndef DEPSILON
  #define DEPSILON (2 * DBL_EPSILON)  // 2 times machine precision for float
#endif


#ifndef NULL
  #define NULL 0
#endif

#ifndef LOOSE_EPSILON
  #define LOOSE_EPSILON 0.000001f
#endif

#ifndef LOOSE_DEPSILON
  #define LOOSE_DEPSILON 0.00000000001
#endif

// Define either ROW_MAJOR (DirectX) or COLUMN_MAJOR (openGL)
// #define ROW_MAJOR
#define COLUMN_MAJOR

// This library uses the convention of column vectors (so vectors on the RHS
// of matricies).  This is true regardless of whether ROW_MAJOR or COLUMN_MAJOR
// is defined.

#include "math/vec2.h"
#include "math/vec3.h"
#include "math/vec4.h"
#include "math/mat2x2.h"
#include "math/mat3x3.h"
#include "math/mat4x4.h"
#include "math/quat.h"
#include "math/plane.h"

#ifndef M_E
  #define M_E     2.71828182845904523536
#endif
#ifndef M_PI
  #define M_PI    3.14159265358979323846
#endif
#ifndef M_PI_2
  #define M_PI_2  1.57079632679489661923
#endif
#ifndef M_PI_4
  #define M_PI_4  0.785398163397448309616
#endif
#ifndef PI_OVER_180
  #define PI_OVER_180 0.017453292519943295769236907684886  // 2*PI / 360
#endif
#ifndef PI_OVER_360
  #define PI_OVER_360 0.0087266462599716478846184538424431
#endif

#ifndef MAX_INT16
  #define MAX_INT16 32767  // 2^15 - 1
#endif

#ifndef MAX_UINT32
  #define MAX_UINT32 0xffffffff
#endif
#ifndef MAX_UINT64
  #define MAX_UINT64 0xffffffffffffffff
#endif
#ifndef MAX_INT64
  #define MAX_INT64 0x7fffffffffffffff
#endif

namespace jtil {
namespace math {

  typedef Vec2<int32_t> Int2;
  typedef Vec2<uint32_t> Uint2;
  typedef Vec2<float> Float2;
  typedef Vec2<double> Double2;

  typedef Vec3<int32_t> Int3;
  typedef Vec3<uint32_t> Uint3;
  typedef Vec3<float> Float3;
  typedef Vec3<double> Double3;

  typedef Vec4<int32_t> Int4;
  typedef Vec4<uint32_t> Uint4;
  typedef Vec4<float> Float4;
  typedef Vec4<double> Double4;

  typedef Mat2x2<float> Float2x2;
  typedef Mat2x2<double> Double2x2;

  typedef Mat3x3<float> Float3x3;
  typedef Mat3x3<double> Double3x3;

  typedef Mat4x4<float> Float4x4;
  typedef Mat4x4<double> Double4x4;

  typedef Quat<float> FloatQuat;
  typedef Quat<double> DoubleQuat;
};  // namespace math
};  // namespace jtil

#if defined(WIN32) || defined(_WIN32)
  #define RAND_ENGINE std::tr1::mt19937
  #define NORM_DIST std::tr1::normal_distribution  // Usgae: NORM_DIST<float> dist;
  #define UNIF_DIST std::tr1::uniform_real_distribution
#endif
#ifdef __APPLE__
  #define RAND_ENGINE std::mt19937
  #define NORM_DIST std::normal_distribution
  #define UNIF_DIST std::uniform_real_distribution
#endif

#if defined(ROW_MAJOR) && defined(COLUMN_MAJOR)
  #error define either ROW_MAJOR or COLUMN_MAJOR but not both
#endif

#if !defined(ROW_MAJOR) && !defined(COLUMN_MAJOR)
  #error define either ROW_MAJOR or COLUMN_MAJOR but not both
#endif
