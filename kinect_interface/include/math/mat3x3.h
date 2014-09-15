//
//  mat3x3.h
//
//  Created by Jonathan Tompson on 3/14/12.
//

#pragma once

#include "math/data_align.h"

namespace jtil {
namespace math {
  // Note Matricies are all row-major:
  //            | m(0,0) m(0,1) m(0,2) |   | 0 1 2 |
  //   Mat3x3 = | m(1,0) m(1,1) m(1,2) | = | 3 4 5 |
  //            | m(2,0) m(2,1) m(2,2) |   | 6 7 8 |
  //      or column-major (COLUMN_MAJOR is defined):
  //            | m(0,0) m(1,0) m(2,0) |   | 0 3 6 |
  //   Mat2x2 = | m(0,1) m(1,1) m(2,1) | = | 1 4 7 |
  //            | m(0,2) m(1,2) m(2,2) |   | 2 5 8 |
  template <class T>
  class DATA_ALIGN(ALIGNMENT, Mat3x3 {
  public:
    Mat3x3();  // Default is identity
    explicit Mat3x3(const T* data);
    explicit Mat3x3(const Mat3x3& data);
    explicit Mat3x3(const Mat4x4<T>& data);  // Drop off the outside elements
    Mat3x3(const T _00, const T _01, const T _02, const T _10, const T _11, 
      const T _12, const T _20, const T _21, const T _22);

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
    void identity();
    void set(const T* data);
    inline void set(const T data, const int i, const int j); // m[i][j] == data (NOTE: i could be ROW OR COLUMN!)
    void set(const Mat3x3& data);
    void set(const T s);
    T& operator()(const int i_row, const int j_col);
    T operator()(const int i_row, const int j_col) const;

    // Math operations
    void transpose();  // this = this^t

    // Static Math operations
    static T det(const Mat3x3& a);
    static T trace(const Mat3x3& a);
    static void scale(Mat3x3& ret, const Mat3x3& a, const T s);  // ret = a * s
    static void scale(Mat3x3& ret, const T s);  // ret *= s
    static void sub(Mat3x3& ret, const Mat3x3& a, const Mat3x3& b);
    static void add(Mat3x3& ret, const Mat3x3& a, const Mat3x3& b);
    static void pairwiseMult(Mat3x3& ret, const Mat3x3& a, const Mat3x3& b);
    static void inverse(Mat3x3& ret, const Mat3x3& a);
    static void mult(Mat3x3& ret, const Mat3x3& a, const Mat3x3& b);
    static void transpose(Mat3x3& ret, const Mat3x3& a);
    static bool equal(const Mat3x3& a, const Mat3x3& b);
    static bool approxEqual(const Mat3x3& a, const Mat3x3& b);

    T m[9];  // Not private --> Avoid some overhead with getter setter methods
    char pad[ALIGNMENT - ((9*sizeof(T)) % ALIGNMENT)];
  }  // end class Mat3x3
  );  // end DATA_ALIGN

  // Mat3x3 Constructors
  template <class T>
  Mat3x3<T>::Mat3x3() {
    m[0] = (T)1;
    m[1] = (T)0;
    m[2] = (T)0;
    m[3] = (T)0;
    m[4] = (T)1;
    m[5] = (T)0;
    m[6] = (T)0;
    m[7] = (T)0;
    m[8] = (T)1;
  };

  template <class T>
  Mat3x3<T>::Mat3x3(const T* data) {
    m[0] = data[0];
    m[1] = data[1];
    m[2] = data[2];
    m[3] = data[3];
    m[4] = data[4];
    m[5] = data[5];
    m[6] = data[6];
    m[7] = data[7];
    m[8] = data[8];
  };

  template <class T>
  Mat3x3<T>::Mat3x3(const Mat3x3& data) {
    m[0] = data.m[0];
    m[1] = data.m[1];
    m[2] = data.m[2];
    m[3] = data.m[3];
    m[4] = data.m[4];
    m[5] = data.m[5];
    m[6] = data.m[6];
    m[7] = data.m[7];
    m[8] = data.m[8];
  };

  template <class T>
  Mat3x3<T>::Mat3x3(const Mat4x4<T>& data) {
    m[0] = data.m[0];
    m[1] = data.m[1];
    m[2] = data.m[2];
    m[3] = data.m[4];
    m[4] = data.m[5];
    m[5] = data.m[6];
    m[6] = data.m[8];
    m[7] = data.m[9];
    m[8] = data.m[10];
  };

  template <class T>
  Mat3x3<T>::Mat3x3(const T _00, const T _01, const T _02, const T _10, 
    const T _11, const T _12, const T _20, const T _21, const T _22) {
      m[0] = _00;
      m[1] = _01;
      m[2] = _02;
      m[3] = _10;
      m[4] = _11;
      m[5] = _12;
      m[6] = _20;
      m[7] = _21;
      m[8] = _22;
  };

  // Getter methods
  template <class T>
  void Mat3x3<T>::print() const {
#ifdef ROW_MAJOR
    printf("| %+.4e  %+.4e  %+.4e |\n", m[0], m[1], m[2]);
    printf("| %+.4e  %+.4e  %+.4e |\n", m[3], m[4], m[5]);
    printf("| %+.4e  %+.4e  %+.4e |\n", m[6], m[7], m[8]);
#endif
#ifdef COLUMN_MAJOR
    printf("| %+.4e  %+.4e  %+.4e |\n", m[0], m[3], m[6]);
    printf("| %+.4e  %+.4e  %+.4e |\n", m[1], m[4], m[7]);
    printf("| %+.4e  %+.4e  %+.4e |\n", m[2], m[5], m[8]);
#endif
  };

  // Setter methods
  template <class T>
  void Mat3x3<T>::zeros() {
    m[0] = (T)0;
    m[1] = (T)0;
    m[2] = (T)0;
    m[3] = (T)0;
    m[4] = (T)0;
    m[5] = (T)0;
    m[6] = (T)0;
    m[7] = (T)0;
    m[8] = (T)0;
  };

  template <class T>
  void Mat3x3<T>::ones() {
    m[0] = (T)1;
    m[1] = (T)1;
    m[2] = (T)1;
    m[3] = (T)1;
    m[4] = (T)1;
    m[5] = (T)1;
    m[6] = (T)1;
    m[7] = (T)1;
    m[8] = (T)1;
  };

  template <class T>
  void Mat3x3<T>::identity() {
    m[0] = (T)1;
    m[1] = (T)0;
    m[2] = (T)0;
    m[3] = (T)0;
    m[4] = (T)1;
    m[5] = (T)0;
    m[6] = (T)0;
    m[7] = (T)0;
    m[8] = (T)1;
  };

  template <class T>
  void Mat3x3<T>::set(const T* data) {
    m[0] = data[0];
    m[1] = data[1];
    m[2] = data[2];
    m[3] = data[3];
    m[4] = data[4];
    m[5] = data[5];
    m[6] = data[6];
    m[7] = data[7];
    m[8] = data[8];
  };

  template <class T>
  void Mat3x3<T>::set(const Mat3x3& data) {
    set(data->m);
  };

  template <class T>
  void Mat3x3<T>::set(const T data, const int i, const int j) {
    m[i*3+j] = data;
  };

  template <class T>
  void Mat3x3<T>::set(const T s) {
    m[0] = s;
    m[1] = s;
    m[2] = s;
    m[3] = s;
    m[4] = s;
    m[5] = s;
    m[6] = s;
    m[7] = s;
    m[8] = s;
  };

  template <class T>
  T& Mat3x3<T>::operator()(const int i_row, const int j_col) {
#ifdef ROW_MAJOR
    return m[i_row * 3 + j_col];
#endif
#ifdef COLUMN_MAJOR
    return m[i_row + j_col * 3];
#endif
  }

  template <class T>
  T Mat3x3<T>::operator()(const int i_row, const int j_col) const {
#ifdef ROW_MAJOR
    return m[i_row * 3 + j_col];
#endif
#ifdef COLUMN_MAJOR
    return m[i_row + j_col * 3];
#endif
  }

  // Math operations
  template <class T>
  void Mat3x3<T>::transpose() {
    T temp;
    temp = m[1];
    m[1] = m[3];
    m[3] = temp;
    temp = m[2];
    m[2] = m[6];
    m[6] = temp;
    temp = m[5];
    m[5] = m[7];
    m[7] = temp;
  };

  // Static Math operations
  // det(A) = det(A^t) (so row and column major have the same determinent)
  template <class T>
  T Mat3x3<T>::det(const Mat3x3& a) {
    return a.m[0] * (a.m[8] * a.m[4] - a.m[7] * a.m[5]) 
         - a.m[3] * (a.m[8] * a.m[1] - a.m[7] * a.m[2]) 
         + a.m[6] * (a.m[5] * a.m[1] - a.m[4] * a.m[2]);
  };

  template <class T>
  T Mat3x3<T>::trace(const Mat3x3& a) {  // Sum of elements on main diagonal
    return a.m[0] + a.m[4] + a.m[8];
  };

  template <class T>
  void Mat3x3<T>::scale(Mat3x3& ret, const Mat3x3& a, const T s) {
    ret.m[0] = a.m[0] * s;
    ret.m[1] = a.m[1] * s;
    ret.m[2] = a.m[2] * s;
    ret.m[3] = a.m[3] * s;
    ret.m[4] = a.m[4] * s;
    ret.m[5] = a.m[5] * s;
    ret.m[6] = a.m[6] * s;
    ret.m[7] = a.m[7] * s;
    ret.m[8] = a.m[8] * s;
  };

  template <class T>
  void Mat3x3<T>::scale(Mat3x3& ret, const T s) {
    ret.m[0] *= s;
    ret.m[1] *= s;
    ret.m[2] *= s;
    ret.m[3] *= s;
    ret.m[4] *= s;
    ret.m[5] *= s;
    ret.m[6] *= s;
    ret.m[7] *= s;
    ret.m[8] *= s;
  };

  template <class T>
  void Mat3x3<T>::sub(Mat3x3& ret, const Mat3x3& a, const Mat3x3& b) {
    ret.m[0] = a.m[0] - b.m[0];
    ret.m[1] = a.m[1] - b.m[1];
    ret.m[2] = a.m[2] - b.m[2];
    ret.m[3] = a.m[3] - b.m[3];
    ret.m[4] = a.m[4] - b.m[4];
    ret.m[5] = a.m[5] - b.m[5];
    ret.m[6] = a.m[6] - b.m[6];
    ret.m[7] = a.m[7] - b.m[7];
    ret.m[8] = a.m[8] - b.m[8];
  };

  template <class T>
  void Mat3x3<T>::add(Mat3x3& ret, const Mat3x3& a, const Mat3x3& b) {
    ret.m[0] = a.m[0] + b.m[0];
    ret.m[1] = a.m[1] + b.m[1];
    ret.m[2] = a.m[2] + b.m[2];
    ret.m[3] = a.m[3] + b.m[3];
    ret.m[4] = a.m[4] + b.m[4];
    ret.m[5] = a.m[5] + b.m[5];
    ret.m[6] = a.m[6] + b.m[6];
    ret.m[7] = a.m[7] + b.m[7];
    ret.m[8] = a.m[8] + b.m[8];
  };

  template <class T>
  void Mat3x3<T>::pairwiseMult(Mat3x3& ret, const Mat3x3& a, const Mat3x3& b) {
    ret.m[0] = a.m[0] * b.m[0];
    ret.m[1] = a.m[1] * b.m[1];
    ret.m[2] = a.m[2] * b.m[2];
    ret.m[3] = a.m[3] * b.m[3];
    ret.m[4] = a.m[4] * b.m[4];
    ret.m[5] = a.m[5] * b.m[5];
    ret.m[6] = a.m[6] * b.m[6];
    ret.m[7] = a.m[7] * b.m[7];
    ret.m[8] = a.m[8] * b.m[8];
  };

  template <class T>
  void Mat3x3<T>::inverse(Mat3x3& ret, const Mat3x3& a) {
    T det = Mat3x3::det(a);
#ifdef ROW_MAJOR
    ret.m[0] = a.m[8]*a.m[4]-a.m[7]*a.m[5];
    ret.m[1] = -(a.m[8]*a.m[1]-a.m[7]*a.m[2]);
    ret.m[2] = a.m[5]*a.m[1]-a.m[4]*a.m[2];
    ret.m[3] = -(a.m[8]*a.m[3]-a.m[6]*a.m[5]);
    ret.m[4] = a.m[8]*a.m[0]-a.m[6]*a.m[2];
    ret.m[5] = -(a.m[5]*a.m[0]-a.m[3]*a.m[2]);
    ret.m[6] = a.m[7]*a.m[3]-a.m[6]*a.m[4];
    ret.m[7] = -(a.m[7]*a.m[0]-a.m[6]*a.m[1]);
    ret.m[8] = a.m[4]*a.m[0]-a.m[3]*a.m[1];
#endif
#ifdef COLUMN_MAJOR
    ret.m[0] = a.m[8]*a.m[4]-a.m[5]*a.m[7];
    ret.m[3] = -(a.m[8]*a.m[3]-a.m[5]*a.m[6]);
    ret.m[6] = a.m[7]*a.m[3]-a.m[4]*a.m[6];
    ret.m[1] = -(a.m[8]*a.m[1]-a.m[2]*a.m[7]);
    ret.m[4] = a.m[8]*a.m[0]-a.m[2]*a.m[6];
    ret.m[7] = -(a.m[7]*a.m[0]-a.m[1]*a.m[6]);
    ret.m[2] = a.m[5]*a.m[1]-a.m[2]*a.m[4];
    ret.m[5] = -(a.m[5]*a.m[0]-a.m[2]*a.m[3]);
    ret.m[8] = a.m[4]*a.m[0]-a.m[1]*a.m[3];
#endif
    Mat3x3::scale(ret, ret, (T)1.0/det);
  };

  template <class T>
  void Mat3x3<T>::mult(Mat3x3& ret, const Mat3x3& a, const Mat3x3& b) {
#ifdef ROW_MAJOR
    ret.m[0] = a.m[0]*b.m[0] + a.m[1]*b.m[3] + a.m[2]*b.m[6];
    ret.m[1] = a.m[0]*b.m[1] + a.m[1]*b.m[4] + a.m[2]*b.m[7];
    ret.m[2] = a.m[0]*b.m[2] + a.m[1]*b.m[5] + a.m[2]*b.m[8];
    ret.m[3] = a.m[3]*b.m[0] + a.m[4]*b.m[3] + a.m[5]*b.m[6];
    ret.m[4] = a.m[3]*b.m[1] + a.m[4]*b.m[4] + a.m[5]*b.m[7];
    ret.m[5] = a.m[3]*b.m[2] + a.m[4]*b.m[5] + a.m[5]*b.m[8];
    ret.m[6] = a.m[6]*b.m[0] + a.m[7]*b.m[3] + a.m[8]*b.m[6];
    ret.m[7] = a.m[6]*b.m[1] + a.m[7]*b.m[4] + a.m[8]*b.m[7];
    ret.m[8] = a.m[6]*b.m[2] + a.m[7]*b.m[5] + a.m[8]*b.m[8];
#endif
#ifdef COLUMN_MAJOR
    ret.m[0] = a.m[0]*b.m[0] + a.m[3]*b.m[1] + a.m[6]*b.m[2];
    ret.m[3] = a.m[0]*b.m[3] + a.m[3]*b.m[4] + a.m[6]*b.m[5];
    ret.m[6] = a.m[0]*b.m[6] + a.m[3]*b.m[7] + a.m[6]*b.m[8];
    ret.m[1] = a.m[1]*b.m[0] + a.m[4]*b.m[1] + a.m[7]*b.m[2];
    ret.m[4] = a.m[1]*b.m[3] + a.m[4]*b.m[4] + a.m[7]*b.m[5];
    ret.m[7] = a.m[1]*b.m[6] + a.m[4]*b.m[7] + a.m[7]*b.m[8];
    ret.m[2] = a.m[2]*b.m[0] + a.m[5]*b.m[1] + a.m[8]*b.m[2];
    ret.m[5] = a.m[2]*b.m[3] + a.m[5]*b.m[4] + a.m[8]*b.m[5];
    ret.m[8] = a.m[2]*b.m[6] + a.m[5]*b.m[7] + a.m[8]*b.m[8];
#endif
  };

  template <class T>
  void Mat3x3<T>::transpose(Mat3x3& ret, const Mat3x3& a) {
    ret.m[0] = a.m[0];
    ret.m[1] = a.m[3];
    ret.m[2] = a.m[6];
    ret.m[3] = a.m[1];
    ret.m[4] = a.m[4];
    ret.m[5] = a.m[7];
    ret.m[6] = a.m[2];
    ret.m[7] = a.m[5];
    ret.m[8] = a.m[8];
  };

  template <class T>
  bool Mat3x3<T>::equal(const Mat3x3& a, const Mat3x3& b) {
    for (int i = 0; i < 9; i ++)
      if (abs(a.m[i] - b.m[i]) > EPSILON)
        return false;
    return true;
  };

  template <class T>
  bool Mat3x3<T>::approxEqual(const Mat3x3& a, const Mat3x3& b) {
    for (int i = 0; i < 9; i ++)
      if (abs(a.m[i] - b.m[i]) > LOOSE_EPSILON)
        return false;
    return true;
  };

};  // namespace math
};  // namespace jtil
