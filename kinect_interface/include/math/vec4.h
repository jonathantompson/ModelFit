//
//  vec4.h
//
//  Created by Jonathan Tompson on 3/14/12.
//

#pragma once

#include <float.h>
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
  class Mat4x4;

  // Note Vectors are all column vectors (OpenGL uses column vectors):
  //          | m(0,0) |   | 0 |
  //   Vec4 = | m(1,0) | = | 1 |
  //          | m(2,0) |   | 2 |
  //          | m(3,0) |   | 3 |
  template <class T>
  class DATA_ALIGN(ALIGNMENT, Vec4 {
   public:
    Vec4();
    explicit Vec4(const T* data);
    Vec4(const Vec4& data);
    Vec4(const T x, const T y, const T z, const T w);

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
    void set(const T x, const T y, const T z, const T w);
    void set(const T* data);
    inline void set(const Vec4& data) { this->set(data.m); }

    // Math operations
    void accum(const T* a);  // this += accum
    void normalize();
    T length() const;

    // Static Math operations
    static void normalize(Vec4& ret);
    static T length(const Vec4& a);
    static void scale(Vec4& ret, const T s);
    static T dot(const Vec4& a, const Vec4& b);  // a.b
    static void outerProd(Mat4x4<T>& ret, const Vec4& a, const Vec4& b);
    static void sub(Vec4& ret, const Vec4& a, const Vec4& b);  // ret = a-b
    static void add(Vec4& ret, const Vec4& a, const Vec4& b);  // ret = a+b
    static void pairwiseMult(Vec4& ret, const Vec4& a, const Vec4& b);
    static void mult(Vec4& ret, const Mat4x4<T>& a, const Vec4& b);
    static bool equal(const Vec4& a, const Vec4& b);
    static bool approxEqual(const Vec4& a, const Vec4& b);
    static void (min)(Vec4& ret, const Vec4& a, const Vec4& b);
    static void (max)(Vec4& ret, const Vec4& a, const Vec4& b);

    T m[4];  // Not private -> Avoid some overhead with getter setter methods
    char pad[ALIGNMENT - ((4*sizeof(T)) % ALIGNMENT)];
  }  // end class Vec4
  );  // end DATA_ALIGN

  // Vec4 Constructors
  template <class T>
  Vec4<T>::Vec4() {
    m[0] = (T)0;
    m[1] = (T)0;
    m[2] = (T)0;
    m[3] = (T)0;
  };

  template <class T>
  Vec4<T>::Vec4(const T* data) {
    m[0] = data[0];
    m[1] = data[1];
    m[2] = data[2];
    m[3] = data[3];
  };

  template <class T>
  Vec4<T>::Vec4(const Vec4& data) {
    m[0] = data.m[0];
    m[1] = data.m[1];
    m[2] = data.m[2];
    m[3] = data.m[3];
  };

  template <class T>
  Vec4<T>::Vec4(const T x, const T y, const T z, const T w) {
    m[0] = x;
    m[1] = y;
    m[2] = z;
    m[3] = w;
  };

  // Setter methods
  template <class T>
  void Vec4<T>::zeros() {
    m[0] = (T)0;
    m[1] = (T)0;
    m[2] = (T)0;
    m[3] = (T)0;
  };

  template <class T>
  void Vec4<T>::ones() {
    m[0] = (T)1;
    m[1] = (T)1;
    m[2] = (T)1;
    m[3] = (T)1;
  };

  template <class T>
  void Vec4<T>::set(const T x, const T y, const T z, const T w) {
    m[0] = x;
    m[1] = y;
    m[2] = z;
    m[3] = w;
  };

  template <class T>
  void Vec4<T>::set(const T* data) {
    m[0] = data[0];
    m[1] = data[1];
    m[2] = data[2];
    m[3] = data[3];
  };

  // Getter Methods
  template <class T>
  void Vec4<T>::print() const {
    printf("| %+.4e |\n| %+.4e |\n| %+.4e |\n| %+.4e |\n", 
      m[0], m[1], m[2], m[3]);
  };

  // Math operations
  template <class T>
  void Vec4<T>::accum(const T* a) {
    m[0] += a[0];
    m[1] += a[1];
    m[2] += a[2];
    m[3] += a[3];
  };

  template <class T>
  void Vec4<T>::normalize() {
    T one_over_length = (T)1.0 / (this->length());
    m[0] *= one_over_length;
    m[1] *= one_over_length;
    m[2] *= one_over_length;
    m[3] *= one_over_length;
  }

  template <class T>
  T Vec4<T>::length() const {
    return (T)sqrt(m[0] * m[0] + m[1] * m[1] + m[2] * m[2] + m[3] * m[3]);
  };

  // Static Math operations
  template <class T>
  void Vec4<T>::normalize(Vec4& ret) {
    T oneOverLength = (T)1.0 / sqrt(ret.m[0] * ret.m[0] + 
      ret.m[1] * ret.m[1] + 
      ret.m[2] * ret.m[2] + 
      ret.m[3] * ret.m[3]);
    ret.m[0] *= oneOverLength;
    ret.m[1] *= oneOverLength;
    ret.m[2] *= oneOverLength;
    ret.m[3] *= oneOverLength;
  };

  template <class T>
  T Vec4<T>::length(const Vec4& a) {
    return sqrt(a.m[0] * a.m[0] + 
      a.m[1] * a.m[1] + 
      a.m[2] * a.m[2] + 
      a.m[3] * a.m[3]);
  };

  template <class T>
  void Vec4<T>::scale(Vec4& ret, const T s) {
    ret.m[0] *= s;
    ret.m[1] *= s;
    ret.m[2] *= s;
    ret.m[3] *= s;
  };

  template <class T>
  T Vec4<T>::dot(const Vec4& a, const Vec4& b) {
    return a.m[0] * b.m[0] + 
      a.m[1] * b.m[1] +
      a.m[2] * b.m[2] +
      a.m[3] * b.m[3];
  };

  template <class T>
  void Vec4<T>::outerProd(Mat4x4<T>& ret, const Vec4& a, const Vec4& b) {
#ifdef COLUMN_MAJOR
    ret.m[0] = a[0]*b[0];
    ret.m[1] = a[1]*b[0];
    ret.m[2] = a[2]*b[0];
    ret.m[3] = a[3]*b[0];
    ret.m[4] = a[0]*b[1];
    ret.m[5] = a[1]*b[1];
    ret.m[6] = a[2]*b[1];
    ret.m[7] = a[3]*b[1];
    ret.m[8] = a[0]*b[2];
    ret.m[9] = a[1]*b[2];
    ret.m[10] = a[2]*b[2];
    ret.m[11] = a[3]*b[2];
    ret.m[12] = a[0]*b[3];
    ret.m[13] = a[1]*b[3];
    ret.m[14] = a[2]*b[3];
    ret.m[15] = a[3]*b[3];

#endif
#ifdef ROW_MAJOR
    ret.m[0] = a[0]*b[0];
    ret.m[1] = a[0]*b[1];
    ret.m[2] = a[0]*b[2];
    ret.m[3] = a[0]*b[3];
    ret.m[4] = a[1]*b[0];
    ret.m[5] = a[1]*b[1];
    ret.m[6] = a[1]*b[2];
    ret.m[7] = a[1]*b[3];
    ret.m[8] = a[2]*b[0];
    ret.m[9] = a[2]*b[1];
    ret.m[10] = a[2]*b[2];
    ret.m[11] = a[2]*b[3];
    ret.m[12] = a[3]*b[0];
    ret.m[13] = a[3]*b[1];
    ret.m[14] = a[3]*b[2];
    ret.m[15] = a[3]*b[3];
#endif
  }

  template <class T>
  void Vec4<T>::sub(Vec4& ret, const Vec4& a, const Vec4& b) {
    ret.m[0] = a.m[0] - b.m[0];
    ret.m[1] = a.m[1] - b.m[1];
    ret.m[2] = a.m[2] - b.m[2];
    ret.m[3] = a.m[3] - b.m[3];
  };

  template <class T>
  void Vec4<T>::add(Vec4& ret, const Vec4& a, const Vec4& b) {
    ret.m[0] = a.m[0] + b.m[0];
    ret.m[1] = a.m[1] + b.m[1];
    ret.m[2] = a.m[2] + b.m[2];
    ret.m[3] = a.m[3] + b.m[3];
  };

  template <class T>
  void Vec4<T>::pairwiseMult(Vec4& ret, const Vec4& a, const Vec4& b) {
    ret.m[0] = a.m[0] * b.m[0];
    ret.m[1] = a.m[1] * b.m[1];
    ret.m[2] = a.m[2] * b.m[2];
    ret.m[3] = a.m[3] * b.m[3];
  };

  template <class T>
  void Vec4<T>::mult(Vec4& ret, const Mat4x4<T>& a, const Vec4& b) {
#ifdef ROW_MAJOR
    ret.m[0] = a.m[0]*b.m[0]  + a.m[1]*b.m[1]  + a.m[2]*b.m[2]  
                + a.m[3]*b.m[3];
    ret.m[1] = a.m[4]*b.m[0]  + a.m[5]*b.m[1]  + a.m[6]*b.m[2]  
                + a.m[7]*b.m[3];
    ret.m[2] = a.m[8]*b.m[0]  + a.m[9]*b.m[1]  + a.m[10]*b.m[2] 
                + a.m[11]*b.m[3];
    ret.m[3] = a.m[12]*b.m[0] + a.m[13]*b.m[1] + a.m[14]*b.m[2] 
                + a.m[15]*b.m[3];
#endif
#ifdef COLUMN_MAJOR
    ret.m[0] = a.m[0]*b.m[0] + a.m[4]*b.m[1] + a.m[8]*b.m[2]  
                + a.m[12]*b.m[3];
    ret.m[1] = a.m[1]*b.m[0] + a.m[5]*b.m[1] + a.m[9]*b.m[2]  
                + a.m[13]*b.m[3];
    ret.m[2] = a.m[2]*b.m[0] + a.m[6]*b.m[1] + a.m[10]*b.m[2] 
                + a.m[14]*b.m[3];
    ret.m[3] = a.m[3]*b.m[0] + a.m[7]*b.m[1] + a.m[11]*b.m[2] 
                + a.m[15]*b.m[3];
#endif
  };

  template <class T>
  bool Vec4<T>::equal(const Vec4& a, const Vec4& b) {
    for (int i = 0; i < 4; i ++)
      if (abs(a.m[i] - b.m[i]) > EPSILON)
        return false;
    return true;
  };

  template <class T>
  bool Vec4<T>::approxEqual(const Vec4& a, const Vec4& b) {
    for (int i = 0; i < 4; i ++)
      if (abs(a.m[i] - b.m[i]) > LOOSE_EPSILON)
        return false;
    return true;
  };

  template <class T>
  void (Vec4<T>::min)(Vec4& ret, const Vec4& a, const Vec4& b) {
    ret.m[0] = (std::min<T>)(a.m[0], b.m[0]);
    ret.m[1] = (std::min<T>)(a.m[1], b.m[1]);
    ret.m[2] = (std::min<T>)(a.m[2], b.m[2]);
    ret.m[3] = (std::min<T>)(a.m[3], b.m[3]);
  };

  template <class T>
  void (Vec4<T>::max)(Vec4& ret, const Vec4& a, const Vec4& b) {
    ret.m[0] = (std::max<T>)(a.m[0], b.m[0]);
    ret.m[1] = (std::max<T>)(a.m[1], b.m[1]);
    ret.m[2] = (std::max<T>)(a.m[2], b.m[2]);
    ret.m[3] = (std::max<T>)(a.m[3], b.m[3]);
  };

};  // namespace math
};  // namespace jtil

