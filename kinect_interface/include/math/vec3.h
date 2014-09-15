//
//  vec3.h
//
//  Created by Jonathan Tompson on 3/14/12.
//

#pragma once

#include <float.h>
#include <math.h>
#ifdef __APPLE__
#include <algorithm>  // for std::min/max 
#endif
#include "math/data_align.h"

#ifndef EPSILON
#define EPSILON (2 * FLT_EPSILON)  // 2 times machine precision for float
#endif

namespace jtil {
namespace math {
  template <class T>
  class Mat3x3;
  
  template <class T>
  class Mat4x4;

  // Note Vectors are all column vectors (OpenGL uses column vectors):
  //          | m(0,0) |   | 0 |
  //   Vec3 = | m(1,0) | = | 1 |
  //          | m(2,0) |   | 2 |
  template <class T>
  class DATA_ALIGN(ALIGNMENT, Vec3 {
  public:
    Vec3();
    explicit Vec3(const T* data);
    Vec3(const Vec3& data);
    Vec3(T x, T y, T z);

    // Getter methods
    void print() const;  // Print to std::cout
    T operator[](const int i) const { 
      return m[i]; 
    } 
    T & operator[](const int i) { 
      return m[i]; 
    }  

    // Setter methods
    void zeros();
    void ones();
    void set(const T x, const T y, const T z);
    void set(const T* data);
    inline void set(const Vec3& data) { this->set(data.m); }

    // Math operations
    void accum(const T* a);  // this += accum
    void normalize();
    T length() const;

    // Static Math operations
    static void normalize(Vec3& ret);
    static void normalize(Vec3& ret, const Vec3& a);
    static T length(const Vec3& a);
    static void scale(Vec3& ret, const T s);
    static T dot(const Vec3& a, const Vec3& b);
    static void outerProd(Mat3x3<T>& ret, const Vec3& a, const Vec3& b);
    static void cross(Vec3& ret, const Vec3& a, const Vec3& b);
    static void sub(Vec3& ret, const Vec3& a, const Vec3& b);
    static void add(Vec3& ret, const Vec3& a, const Vec3& b);
    static void pairwiseMult(Vec3& ret, const Vec3& a, const Vec3& b);
    static void mult(Vec3& ret, const Mat3x3<T>& a, const Vec3& b);
    static bool equal(const Vec3& a, const Vec3& b);
    static bool approxEqual(const Vec3& a, const Vec3& b);
    static void affineTransformVec(Vec3& ret, const Mat4x4<T>& a, 
      const Vec3& b);
    static void affineTransformPos(Vec3& ret, const Mat4x4<T>& a, 
      const Vec3& b);
    static void (min)(Vec3& ret, const Vec3& a, const Vec3& b);
    static void (max)(Vec3& ret, const Vec3& a, const Vec3& b);

    T m[3];  // Not private --> Avoid some overhead with getter setter methods
    char pad[ALIGNMENT - ((3*sizeof(T)) % ALIGNMENT)];
  }  // end class Vec3
  );  // end DATA_ALIGN

  // Vec3 Constructors
  template <class T>
  Vec3<T>::Vec3() {
    m[0] = (T)0;
    m[1] = (T)0;
    m[2] = (T)0;
  };

  template <class T>
  Vec3<T>::Vec3(const T* data) {
    m[0] = data[0];
    m[1] = data[1];
    m[2] = data[2];
  };

  template <class T>
  Vec3<T>::Vec3(const Vec3& data) {
    m[0] = data.m[0];
    m[1] = data.m[1];
    m[2] = data.m[2];
  };

  template <class T>
  Vec3<T>::Vec3(const T x, const T y, const T z) {
    m[0] = x;
    m[1] = y;
    m[2] = z;
  };

  // Setter methods
  template <class T>
  void Vec3<T>::zeros() {
    m[0] = (T)0;
    m[1] = (T)0;
    m[2] = (T)0;
  };

  template <class T>
  void Vec3<T>::ones() {
    m[0] = (T)1;
    m[1] = (T)1;
    m[2] = (T)1;
  };

  template <class T>
  void Vec3<T>::set(const T x, const T y, const T z) {
    m[0] = x;
    m[1] = y;
    m[2] = z;
  };

  template <class T>
  void Vec3<T>::set(const T* data) {
    m[0] = data[0];
    m[1] = data[1];
    m[2] = data[2];
  };

  // Getter Methods
  template <class T>
  void Vec3<T>::print() const {
    printf("| %+.4e |\n| %+.4e |\n| %+.4e |\n", m[0], m[1], m[2]);
  };

  // Math operations
  template <class T>
  void Vec3<T>::accum(const T* a) {
    m[0] += a[0];
    m[1] += a[1];
    m[2] += a[2];
  };

  template <class T>
  void Vec3<T>::normalize() {
    T one_over_length = (T)1.0 / (this->length());
    m[0] *= one_over_length;
    m[1] *= one_over_length;
    m[2] *= one_over_length;
  }

  template <class T>
  T Vec3<T>::length() const {
    return (T)sqrt(m[0] * m[0] + m[1] * m[1] + m[2] * m[2]);
  };

  // Static Math operations
  template <class T>
  void Vec3<T>::normalize(Vec3& ret) {
    normalize(ret, ret);
  };

  template <class T>
  void Vec3<T>::normalize(Vec3& ret, const Vec3& a) {
    T oneOverLength = (T)1.0 / sqrt(a.m[0] * a.m[0] + 
      a.m[1] * a.m[1] + 
      a.m[2] * a.m[2]);
    ret.m[0] = a.m[0] * oneOverLength;
    ret.m[1] = a.m[1] * oneOverLength;
    ret.m[2] = a.m[2] * oneOverLength;
  };

  template <class T>
  T Vec3<T>::length(const Vec3& a) {
    return sqrt(a.m[0] * a.m[0] + 
      a.m[1] * a.m[1] + 
      a.m[2] * a.m[2]);
  };

  template <class T>
  void Vec3<T>::scale(Vec3& ret, const T s) {
    ret.m[0] *= s;
    ret.m[1] *= s;
    ret.m[2] *= s;
  };

  template <class T>
  T Vec3<T>::dot(const Vec3& a, const Vec3& b) {
    return a.m[0] * b.m[0] + 
      a.m[1] * b.m[1] +
      a.m[2] * b.m[2];
  };

  template <class T>
  void Vec3<T>::cross(Vec3& ret, const Vec3& a, const Vec3& b) {
    ret.m[0] = a.m[1] * b.m[2] - b.m[1] * a.m[2];
    ret.m[1] = a.m[2] * b.m[0] - b.m[2] * a.m[0];
    ret.m[2] = a.m[0] * b.m[1] - b.m[0] * a.m[1];
  };

  template <class T>
  void Vec3<T>::outerProd(Mat3x3<T>& ret, const Vec3& a, const Vec3& b) {
#ifdef COLUMN_MAJOR
    ret.m[0] = a[0]*b[0];
    ret.m[1] = a[1]*b[0];
    ret.m[2] = a[2]*b[0];
    ret.m[3] = a[0]*b[1];
    ret.m[4] = a[1]*b[1];
    ret.m[5] = a[2]*b[1];
    ret.m[6] = a[0]*b[2];
    ret.m[7] = a[1]*b[2];
    ret.m[8] = a[2]*b[2];
#endif
#ifdef ROW_MAJOR
    ret.m[0] = a[0]*b[0];
    ret.m[1] = a[0]*b[1];
    ret.m[2] = a[0]*b[2];
    ret.m[3] = a[1]*b[0];
    ret.m[4] = a[1]*b[1];
    ret.m[5] = a[1]*b[2];
    ret.m[6] = a[2]*b[0];
    ret.m[7] = a[2]*b[1];
    ret.m[8] = a[2]*b[2];
#endif
  }

  template <class T>
  void Vec3<T>::sub(Vec3& ret, const Vec3& a, const Vec3& b) {
    ret.m[0] = a.m[0] - b.m[0];
    ret.m[1] = a.m[1] - b.m[1];
    ret.m[2] = a.m[2] - b.m[2];
  };

  template <class T>
  void Vec3<T>::add(Vec3& ret, const Vec3& a, const Vec3& b) {
    ret.m[0] = a.m[0] + b.m[0];
    ret.m[1] = a.m[1] + b.m[1];
    ret.m[2] = a.m[2] + b.m[2];
  };

  template <class T>
  void Vec3<T>::pairwiseMult(Vec3& ret, const Vec3& a, const Vec3& b) {
    ret.m[0] = a.m[0] * b.m[0];
    ret.m[1] = a.m[1] * b.m[1];
    ret.m[2] = a.m[2] * b.m[2];
  };

  template <class T>
  void Vec3<T>::mult(Vec3& ret, const Mat3x3<T>& a, const Vec3& b) {
#ifdef ROW_MAJOR
    ret.m[0] = a.m[0]*b.m[0] + a.m[1]*b.m[1] + a.m[2]*b.m[2];
    ret.m[1] = a.m[3]*b.m[0] + a.m[4]*b.m[1] + a.m[5]*b.m[2];
    ret.m[2] = a.m[6]*b.m[0] + a.m[7]*b.m[1] + a.m[8]*b.m[2];
#endif
#ifdef COLUMN_MAJOR
    ret.m[0] = a.m[0]*b.m[0] + a.m[3]*b.m[1] + a.m[6]*b.m[2];
    ret.m[1] = a.m[1]*b.m[0] + a.m[4]*b.m[1] + a.m[7]*b.m[2];
    ret.m[2] = a.m[2]*b.m[0] + a.m[5]*b.m[1] + a.m[8]*b.m[2];
#endif
  };

  template <class T>
  bool Vec3<T>::equal(const Vec3& a, const Vec3& b) {
    if (abs(a.m[0] - b.m[0]) > EPSILON)
      return false;
    if (abs(a.m[1] - b.m[1]) > EPSILON)
      return false;
    if (abs(a.m[2] - b.m[2]) > EPSILON)
      return false;    
    return true;
  };

  template <class T>
  bool Vec3<T>::approxEqual(const Vec3& a, const Vec3& b) {
    if (abs(a.m[0] - b.m[0]) > LOOSE_EPSILON)
      return false;
    if (abs(a.m[1] - b.m[1]) > LOOSE_EPSILON)
      return false;
    if (abs(a.m[2] - b.m[2]) > LOOSE_EPSILON)
      return false;    
    return true;
  };

  template <class T>
  void Vec3<T>::affineTransformVec(Vec3& ret, const Mat4x4<T>& a, 
    const Vec3& b) {
    // For vectors, homogeneous coord is 0 --> Ignore translation data
#ifdef ROW_MAJOR
    ret.m[0] = a.m[0]*b.m[0] + a.m[1]*b.m[1] + a.m[2]*b.m[2];
    ret.m[1] = a.m[4]*b.m[0] + a.m[5]*b.m[1] + a.m[6]*b.m[2];
    ret.m[2] = a.m[8]*b.m[0] + a.m[9]*b.m[1] + a.m[10]*b.m[2];
#endif
#ifdef COLUMN_MAJOR
    ret.m[0] = a.m[0]*b.m[0] + a.m[4]*b.m[1] + a.m[8]*b.m[2];
    ret.m[1] = a.m[1]*b.m[0] + a.m[5]*b.m[1] + a.m[9]*b.m[2];
    ret.m[2] = a.m[2]*b.m[0] + a.m[6]*b.m[1] + a.m[10]*b.m[2];
#endif
  };  

  template <class T>
  void Vec3<T>::affineTransformPos(Vec3& ret, const Mat4x4<T>& a, 
    const Vec3& b) {
    // For positions, homogeneous coord is 1 --> Add translation data
#ifdef ROW_MAJOR
    ret.m[0] = a.m[0]*b.m[0] + a.m[1]*b.m[1] + a.m[2]*b.m[2] + a.m[3];
    ret.m[1] = a.m[4]*b.m[0] + a.m[5]*b.m[1] + a.m[6]*b.m[2] + a.m[7];
    ret.m[2] = a.m[8]*b.m[0] + a.m[9]*b.m[1] + a.m[10]*b.m[2] + a.m[11];
#endif
#ifdef COLUMN_MAJOR
    ret.m[0] = a.m[0]*b.m[0] + a.m[4]*b.m[1] + a.m[8]*b.m[2] + a.m[12];
    ret.m[1] = a.m[1]*b.m[0] + a.m[5]*b.m[1] + a.m[9]*b.m[2] + a.m[13];
    ret.m[2] = a.m[2]*b.m[0] + a.m[6]*b.m[1] + a.m[10]*b.m[2] + a.m[14];
#endif
  };    

  template <class T>
  void (Vec3<T>::min)(Vec3& ret, const Vec3& a, const Vec3& b) {
    ret.m[0] = (std::min<T>)(a.m[0], b.m[0]);
    ret.m[1] = (std::min<T>)(a.m[1], b.m[1]);
    ret.m[2] = (std::min<T>)(a.m[2], b.m[2]);
  };

  template <class T>
  void (Vec3<T>::max)(Vec3& ret, const Vec3& a, const Vec3& b) {
    ret.m[0] = (std::max<T>)(a.m[0], b.m[0]);
    ret.m[1] = (std::max<T>)(a.m[1], b.m[1]);
    ret.m[2] = (std::max<T>)(a.m[2], b.m[2]);
  };

};  // namespace math
};  // namespace jtil

