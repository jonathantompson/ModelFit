//
//  vec2.h
//
//  Created by Jonathan Tompson on 3/14/12.
//

#pragma once

#include <float.h>
#if defined(__APPLE__) || defined(__GNUC__)
  #include <algorithm>  // for std::min/max 
#endif
#include "math/data_align.h"

#ifndef EPSILON
#define EPSILON (2 * FLT_EPSILON)  // 2 times machine precision for float
#endif

namespace jtil {
namespace math {
  template <class T>
  class Mat2x2;

  // Note Vectors are all column vectors (OpenGL uses column vectors):
  //          | m(0,0) |   | 0 |
  //   Vec2 = | m(1,0) | = | 1 |
  template <class T>
  class DATA_ALIGN(ALIGNMENT, Vec2 {
  public:
    Vec2();
    explicit Vec2(const T* data);
    Vec2(const Vec2& data);
    Vec2(const T x, const T y);

    // Getter methods
    void print() const;  // Print to std::cout
    T operator[](const int i) const { 
      return m[i]; 
    } 
    T& operator[](const int i) { 
      return m[i]; 
    }  

    // Setter methods
    void zeros();
    void ones();
    void set(const T x, const T y);
    void set(const T* data);
    inline void set(const Vec2& data) { this->set(data.m); }

    // Math operations
    inline void accum(const T* a);  // this += accum
    void normalize();
    T length() const;

    // Static Math operations
    static void normalize(Vec2& a);
    static T length(const Vec2& a);
    static void scale(Vec2& a, const T s);
    static T dot(const Vec2& a, const Vec2& b);
    static void outerProd(Mat2x2<T>& ret, const Vec2& a, const Vec2& b);
    static void sub(Vec2& ret, const Vec2& a, const Vec2& b);  // ret = a - b
    static void add(Vec2& ret, const Vec2& a, const Vec2& b);
    static void pairwiseMult(Vec2& ret, const Vec2& a, const Vec2& b);
    static void mult(Vec2& ret, const Mat2x2<T>& a, const Vec2& b);
    static bool equal(const Vec2& a, const Vec2& b);
    static bool approxEqual(const Vec2& a, const Vec2& b);
    static void (min)(Vec2& ret, const Vec2& a, const Vec2& b); 
    static void (max)(Vec2& ret, const Vec2& a, const Vec2& b); 

    T m[2];  // Not private --> Avoid some overhead with getter setter methods
    char pad[ALIGNMENT - ((2*sizeof(T)) % ALIGNMENT)];
  }  // end class Vec2
  );  // end DATA_ALIGN

  // Vec2 Constructors
  template <class T>
  Vec2<T>::Vec2() {
    m[0] = (T)0;
    m[1] = (T)0;
  };

  template <class T>
  Vec2<T>::Vec2(const T* data) {
    m[0] = data[0];
    m[1] = data[1];
  };

  template <class T>
  Vec2<T>::Vec2(const Vec2& data) {
    m[0] = data.m[0];
    m[1] = data.m[1];
  };

  template <class T>
  Vec2<T>::Vec2(const T x, const T y) {
    m[0] = x;
    m[1] = y;
  };

  // Setter methods
  template <class T>
  void Vec2<T>::zeros() {
    m[0] = (T)0;
    m[1] = (T)0;
  };

  template <class T>
  void Vec2<T>::ones() {
    m[0] = (T)1;
    m[1] = (T)1;
  };

  template <class T>
  void Vec2<T>::set(const T x, const T y) {
    m[0] = x;
    m[1] = y;
  };

  template <class T>
  void Vec2<T>::set(const T* data) {
    m[0] = data[0];
    m[1] = data[1];
  };

  // Getter Methods
  template <class T>
  void Vec2<T>::print() const {
    printf("| %+.4e |\n| %+.4e |\n", m[0], m[1]);
  };

  // Math operations
  template <class T>
  void Vec2<T>::accum(const T* a) {
    m[0] += a[0];
    m[1] += a[1];
  };

  template <class T>
  void Vec2<T>::normalize() {
    T one_over_length = (T)1.0 / (this->length());
    m[0] *= one_over_length;
    m[1] *= one_over_length;
  }

  template <class T>
  T Vec2<T>::length() const {
    return (T)sqrt(m[0] * m[0] + m[1] * m[1]);
  };

  // Static Math operations
  template <class T>
  void Vec2<T>::normalize(Vec2& a) {
    T oneOverLength = (T)1.0 / std::sqrt(a.m[0] * a.m[0] + a.m[1] * a.m[1]);
    a.m[0] *= oneOverLength;
    a.m[1] *= oneOverLength;
  };

  template <class T>
  T Vec2<T>::length(const Vec2& a) {
    return std::sqrt(a.m[0] * a.m[0] + a.m[1] * a.m[1]);
  };

  template <class T>
  void Vec2<T>::scale(Vec2& a, const T s) {
    a.m[0] *= s;
    a.m[1] *= s;
  };

  template <class T>
  T Vec2<T>::dot(const Vec2& a, const Vec2& b) {
    return a.m[0] * b.m[0] + a.m[1] * b.m[1];
  };

  template <class T>
  void Vec2<T>::outerProd(Mat2x2<T>& ret, const Vec2& a, const Vec2& b) {
#ifdef COLUMN_MAJOR
    ret.m[0] = a[0]*b[0];
    ret.m[1] = a[1]*b[0];
    ret.m[2] = a[0]*b[1];
    ret.m[3] = a[1]*b[1];
#endif
#ifdef ROW_MAJOR
    ret.m[0] = a[0]*b[0];
    ret.m[1] = a[0]*b[1];
    ret.m[2] = a[1]*b[0];
    ret.m[3] = a[1]*b[1];
#endif
  }

  template <class T>
  void Vec2<T>::sub(Vec2& ret, const Vec2& a, const Vec2& b) {
    ret.m[0] = a.m[0] - b.m[0];
    ret.m[1] = a.m[1] - b.m[1];
  };

  template <class T>
  void Vec2<T>::add(Vec2& ret, const Vec2& a, const Vec2& b) {
    ret.m[0] = a.m[0] + b.m[0];
    ret.m[1] = a.m[1] + b.m[1];
  };

  template <class T>
  void Vec2<T>::pairwiseMult(Vec2& ret, const Vec2& a, const Vec2& b) {
    ret.m[0] = a.m[0] * b.m[0];
    ret.m[1] = a.m[1] * b.m[1];
  };

  template <class T>
  void Vec2<T>::mult(Vec2& ret, const Mat2x2<T>& a, const Vec2& b) {
#ifdef ROW_MAJOR
    ret.m[0] = a.m[0]*b.m[0]  + a.m[1]*b.m[1];
    ret.m[1] = a.m[2]*b.m[0]  + a.m[3]*b.m[1];
#endif
#ifdef COLUMN_MAJOR
    ret.m[0] = a.m[0]*b.m[0]  + a.m[2]*b.m[1];
    ret.m[1] = a.m[1]*b.m[0]  + a.m[3]*b.m[1];
#endif
  };

  template <class T>
  bool Vec2<T>::equal(const Vec2& a, const Vec2& b) {
    for (int i = 0; i < 2; i ++)
      if (abs(a.m[i] - b.m[i]) > EPSILON)
        return false;
    return true;
  };

  template <class T>
  bool Vec2<T>::approxEqual(const Vec2& a, const Vec2& b) {
    for (int i = 0; i < 2; i ++)
      if (abs(a.m[i] - b.m[i]) > LOOSE_EPSILON)
        return false;
    return true;
  };

  template <class T>
  void (Vec2<T>::min)(Vec2& ret, const Vec2& a, const Vec2& b) {
    ret.m[0] = (std::min<T>)(a.m[0], b.m[0]);
    ret.m[1] = (std::min<T>)(a.m[1], b.m[1]);
  };

  template <class T>
  void (Vec2<T>::max)(Vec2& ret, const Vec2& a, const Vec2& b) {
    ret.m[0] = (std::max<T>)(a.m[0], b.m[0]);
    ret.m[1] = (std::max<T>)(a.m[1], b.m[1]);
  };

};  // namespace math
};  // namespace jtil

