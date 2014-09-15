//
//  mat4x4.h
//
//  Created by Jonathan Tompson on 3/14/12.
//

#pragma once

#include <xmmintrin.h>

#include "math/data_align.h"
extern "C" { 
  #include "math/decompose.h"
}

#ifndef PI_OVER_360
  #define PI_OVER_360 0.0087266462599716478846184538424431
#endif

#ifndef M_PI_2
  #define M_PI_2  1.57079632679489661923
#endif

namespace jtil {
namespace math {
  // Note Matricies are all row-major:
  //            | m(0,0) m(0,1) m(0,2) m(0,3) |   |  0  1  2  3 |
  //   Mat4x4 = | m(1,0) m(1,1) m(1,2) m(1,3) | = |  4  5  6  7 |
  //            | m(2,0) m(2,1) m(2,2) m(2,3) |   |  8  9 10 11 |
  //            | m(3,0) m(3,1) m(3,2) m(3,3) |   | 12 13 14 15 |
  //      or column-major (COLUMN_MAJOR is defined):
  //            | m(0,0) m(1,0) m(2,0) m(3,0) |   | 0  4  8  12 |
  //   Mat4x4 = | m(0,1) m(1,1) m(2,1) m(3,1) | = | 1  5  9  13 |
  //            | m(0,2) m(1,2) m(2,2) m(3,2) |   | 2  6  10 14 |
  //            | m(0,3) m(1,3) m(2,3) m(3,3) |   | 3  7  11 15 |
  template <class T>
  class DATA_ALIGN(ALIGNMENT, Mat4x4 {
  public:
    Mat4x4();  // Default is identity
    explicit Mat4x4(const T* data);
    explicit Mat4x4(const Mat4x4& data);
    explicit Mat4x4(const Mat3x3<T>& data);
    Mat4x4(const T _00, const T _01, const T _02, const T _03, const T _10, 
      const T _11, const T _12, const T _13, const T _20, const T _21, 
      const T _22, const T _23, const T _30, const T _31, const T _32, 
      const T _33);
    
    // Getter methods
    void print() const;  // Print to std::cout
    void printPrecise() const;  // Print to std::cout with 8 bits precision
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
    inline void set(const T data, const int i, const int j);  // m[i][j] == data (NOTE: i could be ROW OR COLUMN!)
    void set(const Mat4x4& data);
    void set(const T s);
    void set(const T _00, const T _01, const T _02, const T _03, const T _10, 
      const T _11, const T _12, const T _13, const T _20, const T _21, 
      const T _22, const T _23, const T _30, const T _31, const T _32, 
      const T _33);
    T& operator()(const int i_row, const int j_col);
    T operator()(const int i_row, const int j_col) const;
    
    // Math operations
    void transpose();  // this = this^t
    void leftMultTranslation(const T x, const T y, const T z);  // A = M_trans * A
    void leftMultTranslation(const Vec3<T>& trans);  // A = M_trans * A
    void leftMultScale(const T x, const T y, const T z);  // A = M_scale * A
    void leftMultRotateXAxis(const T angle);  // A = M_scale * A
    void leftMultRotateYAxis(const T angle);  // A = M_scale * A
    void leftMultRotateZAxis(const T angle);  // A = M_scale * A
    void rightMultTranslation(const T x, const T y, const T z);  // A = A * M_trans
    void rightMultTranslation(const Vec3<T>& trans);  // A = A * M_trans
    void rightMultScale(const T x, const T y, const T z);  // A = A * M_scale
    void rightMultRotateXAxis(const T angle);  // A = A * M_scale
    void rightMultRotateYAxis(const T angle);  // A = A * M_scale
    void rightMultRotateZAxis(const T angle);  // A = A * M_scale
    void rotateMatAxisAngle(const Vec3<T>& axis, const T angle);
    void scaleMat(const T x, const T y, const T z);
    void translationMat(const T x, const T y, const T z);
    void glProjection(const T znear, const T zfar, const T fov_deg, 
      const T screen_width, const T screen_height);
    void glOrthoProjection(const T znear, const T zfar, const T left, 
      const T right, const T bottom, const T top);
    void glLookAt(const Vec3<T>& up, const Vec3<T>& forward,
      const Vec3<T>& pos);
    T frobeniusNorm();
    
    // Static Math operations
    static T det(const Mat4x4& a);
    static T trace(const Mat4x4& a);
    static void scale(Mat4x4& ret, const Mat4x4& a, const T s);  // ret = a*s
    static void sub(Mat4x4& ret, const Mat4x4& a, const Mat4x4& b);  // ret = a-b
    static void add(Mat4x4& ret, const Mat4x4& a, const Mat4x4& b);  // ret = a+b
    static void pairwiseMult(Mat4x4& ret, const Mat4x4& a, const Mat4x4& b);  // ret = a.*b
    static void inverse(Mat4x4& ret, const Mat4x4& a);  // ret = a^-1
    static void mult(Mat4x4& ret, const Mat4x4& a, const Mat4x4& b);  // ret = a*b
    // multSIMD - 2.5x faster on release build --> Only works for float!
    static void multSIMD(Mat4x4& ret, const Mat4x4& a, const Mat4x4& b);  // ret = a*b
    static void transpose(Mat4x4& ret, const Mat4x4& a);  // ret = a^t
    static bool equal(const Mat4x4& a, const Mat4x4& b);
    static bool approxEqual(const Mat4x4& a, const Mat4x4& b);
    static void rotateMatXAxis(Mat4x4& ret, const T angle);
    static void rotateMatYAxis(Mat4x4& ret, const T angle);
    static void rotateMatZAxis(Mat4x4& ret, const T angle);
    static void rotateMatBasis(Mat4x4& ret, const Vec3<T>& x, 
      const Vec3<T>& y, const Vec3<T>& z);
    static void rotateMatAxisAngle(Mat4x4& ret, const Vec3<T>& axis, const T angle);
    static void scaleMat(Mat4x4& ret, const T x, const T y, const T z);
    static void scaleMat(Mat4x4& ret, const Vec3<T>& scale);
    static void translationMat(Mat4x4& ret, const T x, const T y, const T z);
    static void glProjection(Mat4x4& a, const T znear, const T zfar, 
      const T fov_deg, const T screen_width, const T screen_height);
    static void glOrthoProjection(Mat4x4& a, const T znear, const T zfar, 
      const T left, const T right, const T bottom, const T top);
    static void glLookAt(Mat4x4& view, const Vec3<T>& up,
      const Vec3<T>& forward, const Vec3<T>& pos);
    static void affineInverse(Mat4x4& ret, const Mat4x4& a);
    static void affineRotationTranslationInverse(Mat4x4& ret, const Mat4x4& a);
    static void affineRotationInverse(Mat4x4& ret, const Mat4x4& a);
    // decompose into: Rot * Scale * Translation (IN THAT ORDER!), a is affine 
    // and invertible
    static void decomposeRST(Mat4x4& rotation, Vec3<T>& scale, 
      Vec3<T>& translation, const Mat4x4& a);  // Matrix must be affine and invertible!
    static void extractRotation(Mat4x4& rotation, const Mat4x4& a);
    static void getTranslation(Vec3<T>& translation, const Mat4x4& a); 
    static void euler2RotMat(Mat4x4& a, const T x_angle, 
      const T y_angle, const T z_angle);
    static void rotMat2Euler(T& x_angle, T& y_angle, T& z_angle, 
      const Mat4x4& a);
    static T frobeniusNorm(const Mat4x4& a);
    
    T m[16];  // Not private --> Avoid some overhead with getter setter methods
    char pad[ALIGNMENT - ((16*sizeof(T)) % ALIGNMENT)];
  }  // end class Mat4x4
  );  // end DATA_ALIGN
  
  // Mat4x4 Constructors
  template <class T>
  Mat4x4<T>::Mat4x4() {
    m[0] = (T)1;
    m[1] = (T)0;
    m[2] = (T)0;
    m[3] = (T)0;
    m[4] = (T)0;
    m[5] = (T)1;
    m[6] = (T)0;
    m[7] = (T)0;
    m[8] = (T)0;
    m[9] = (T)0;
    m[10] = (T)1;
    m[11] = (T)0;
    m[12] = (T)0;
    m[13] = (T)0;
    m[14] = (T)0;
    m[15] = (T)1;
  };
  
  template <class T>
  Mat4x4<T>::Mat4x4(const T* data) {
    m[0] = data[0];
    m[1] = data[1];
    m[2] = data[2];
    m[3] = data[3];
    m[4] = data[4];
    m[5] = data[5];
    m[6] = data[6];
    m[7] = data[7];
    m[8] = data[8];
    m[9] = data[9];
    m[10] = data[10];
    m[11] = data[11];
    m[12] = data[12];
    m[13] = data[13];
    m[14] = data[14];
    m[15] = data[15];
  };
  
  template <class T>
  Mat4x4<T>::Mat4x4(const Mat4x4& data) {
    m[0] = data.m[0];
    m[1] = data.m[1];
    m[2] = data.m[2];
    m[3] = data.m[3];
    m[4] = data.m[4];
    m[5] = data.m[5];
    m[6] = data.m[6];
    m[7] = data.m[7];
    m[8] = data.m[8];
    m[9] = data.m[9];
    m[10] = data.m[10];
    m[11] = data.m[11];
    m[12] = data.m[12];
    m[13] = data.m[13];
    m[14] = data.m[14];
    m[15] = data.m[15];
  };
  
  template <class T>
  Mat4x4<T>::Mat4x4(const Mat3x3<T>& data) {
    m[0] = data.m[0];
    m[1] = data.m[1];
    m[2] = data.m[2];
    m[3] = (T)0;
    m[4] = data.m[3];
    m[5] = data.m[4];
    m[6] = data.m[5];
    m[7] = (T)0;
    m[8] = data.m[6];
    m[9] = data.m[7];
    m[10] = data.m[8];
    m[11] = (T)0;
    m[12] = (T)0;
    m[13] = (T)0;
    m[14] = (T)0;
    m[15] = (T)1;
  };
  
  template <class T>
  Mat4x4<T>::Mat4x4(const T _00, const T _01, const T _02, const T _03, const T _10, 
      const T _11, const T _12, const T _13, const T _20, const T _21, 
      const T _22, const T _23, const T _30, const T _31, const T _32, 
      const T _33) {
    m[0] = _00;
    m[1] = _01;
    m[2] = _02;
    m[3] = _03;
    m[4] = _10;
    m[5] = _11;
    m[6] = _12;
    m[7] = _13;
    m[8] = _20;
    m[9] = _21;
    m[10] = _22;
    m[11] = _23;
    m[12] = _30;
    m[13] = _31;
    m[14] = _32;
    m[15] = _33;
  };
  
  // Getter methods
  template <class T>
  void Mat4x4<T>::print() const {
#ifdef ROW_MAJOR
    printf("| %+.4e  %+.4e  %+.4e  %+.4e |\n", m[0],  m[1],  m[2],  m[3]);
    printf("| %+.4e  %+.4e  %+.4e  %+.4e |\n", m[4],  m[5],  m[6],  m[7]);
    printf("| %+.4e  %+.4e  %+.4e  %+.4e |\n", m[8],  m[9],  m[10], m[11]);
    printf("| %+.4e  %+.4e  %+.4e  %+.4e |\n", m[12], m[13], m[14], m[15]);
#endif
#ifdef COLUMN_MAJOR
    printf("| %+.4e  %+.4e  %+.4e  %+.4e |\n", m[0],  m[4],  m[8],  m[12]);
    printf("| %+.4e  %+.4e  %+.4e  %+.4e |\n", m[1],  m[5],  m[9],  m[13]);
    printf("| %+.4e  %+.4e  %+.4e  %+.4e |\n", m[2],  m[6],  m[10], m[14]);
    printf("| %+.4e  %+.4e  %+.4e  %+.4e |\n", m[3],  m[7],  m[11], m[15]);
#endif
  };

  // Getter methods
  template <class T>
  void Mat4x4<T>::printPrecise() const {
#ifdef ROW_MAJOR
    printf("| %+.8e  %+.8e  %+.8e  %+.8e |\n", m[0],  m[1],  m[2],  m[3]);
    printf("| %+.8e  %+.8e  %+.8e  %+.8e |\n", m[4],  m[5],  m[6],  m[7]);
    printf("| %+.8e  %+.8e  %+.8e  %+.8e |\n", m[8],  m[9],  m[10], m[11]);
    printf("| %+.8e  %+.8e  %+.8e  %+.8e |\n", m[12], m[13], m[14], m[15]);
#endif
#ifdef COLUMN_MAJOR
    printf("| %+.8e  %+.8e  %+.8e  %+.8e |\n", m[0],  m[4],  m[8],  m[12]);
    printf("| %+.8e  %+.8e  %+.8e  %+.8e |\n", m[1],  m[5],  m[9],  m[13]);
    printf("| %+.8e  %+.8e  %+.8e  %+.8e |\n", m[2],  m[6],  m[10], m[14]);
    printf("| %+.8e  %+.8e  %+.8e  %+.8e |\n", m[3],  m[7],  m[11], m[15]);
#endif
  };
  
  // Setter methods
  template <class T>
  void Mat4x4<T>::zeros() {
    m[0] = (T)0;
    m[1] = (T)0;
    m[2] = (T)0;
    m[3] = (T)0;
    m[4] = (T)0;
    m[5] = (T)0;
    m[6] = (T)0;
    m[7] = (T)0;
    m[8] = (T)0;
    m[9] = (T)0;
    m[10] = (T)0;
    m[11] = (T)0;
    m[12] = (T)0;
    m[13] = (T)0;
    m[14] = (T)0;
    m[15] = (T)0;
  };
  
  template <class T>
  void Mat4x4<T>::ones() {
    m[0] = (T)1;
    m[1] = (T)1;
    m[2] = (T)1;
    m[3] = (T)1;
    m[4] = (T)1;
    m[5] = (T)1;
    m[6] = (T)1;
    m[7] = (T)1;
    m[8] = (T)1;
    m[9] = (T)1;
    m[10] = (T)1;
    m[11] = (T)1;
    m[12] = (T)1;
    m[13] = (T)1;
    m[14] = (T)1;
    m[15] = (T)1;
  };
  
  template <class T>
  void Mat4x4<T>::identity() {
    m[0] = (T)1;
    m[1] = (T)0;
    m[2] = (T)0;
    m[3] = (T)0;
    m[4] = (T)0;
    m[5] = (T)1;
    m[6] = (T)0;
    m[7] = (T)0;
    m[8] = (T)0;
    m[9] = (T)0;
    m[10] = (T)1;
    m[11] = (T)0;
    m[12] = (T)0;
    m[13] = (T)0;
    m[14] = (T)0;
    m[15] = (T)1;
  };

  template <class T>
  void Mat4x4<T>::set(const T _00, const T _01, const T _02, const T _03, 
    const T _10, const T _11, const T _12, const T _13, const T _20, 
    const T _21, const T _22, const T _23, const T _30, const T _31, 
    const T _32, const T _33) {
    m[0] = _00;
    m[1] = _01;
    m[2] = _02;
    m[3] = _03;
    m[4] = _10;
    m[5] = _11;
    m[6] = _12;
    m[7] = _13;
    m[8] = _20;
    m[9] = _21;
    m[10] = _22;
    m[11] = _23;
    m[12] = _30;
    m[13] = _31;
    m[14] = _32;
    m[15] = _33;
  };
  
  template <class T>
  void Mat4x4<T>::set(const T* data) {
    m[0] = data[0];
    m[1] = data[1];
    m[2] = data[2];
    m[3] = data[3];
    m[4] = data[4];
    m[5] = data[5];
    m[6] = data[6];
    m[7] = data[7];
    m[8] = data[8];
    m[9] = data[9];
    m[10] = data[10];
    m[11] = data[11];
    m[12] = data[12];
    m[13] = data[13];
    m[14] = data[14];
    m[15] = data[15];
  };
  
  template <class T>
  void Mat4x4<T>::set(const Mat4x4& data) {
    set(data.m);
  };
  
  template <class T>
  void Mat4x4<T>::set(const T data, const int i, const int j) {
    m[i*3+j] = data;
  };
  
  template <class T>
  void Mat4x4<T>::set(const T s) {
    m[0] = s;
    m[1] = s;
    m[2] = s;
    m[3] = s;
    m[4] = s;
    m[5] = s;
    m[6] = s;
    m[7] = s;
    m[8] = s;
    m[9] = s;
    m[10] = s;
    m[11] = s;
    m[12] = s;
    m[13] = s;
    m[14] = s;
    m[15] = s;
  };

  template <class T>
  T& Mat4x4<T>::operator()(const int i_row, const int j_col) {
#ifdef ROW_MAJOR
    return m[i_row * 4 + j_col];
#endif
#ifdef COLUMN_MAJOR
    return m[i_row + j_col * 4];
#endif
  }

  template <class T>
  T Mat4x4<T>::operator()(const int i_row, const int j_col) const {
#ifdef ROW_MAJOR
    return m[i_row * 4 + j_col];
#endif
#ifdef COLUMN_MAJOR
    return m[i_row + j_col * 4];
#endif
  }
  
  template <class T>
  void Mat4x4<T>::rightMultTranslation(const T x, const T y, const T z) {
#ifdef ROW_MAJOR
    m[3] = m[3] + x*m[0] + y*m[1] + z*m[2];
    m[7] = m[7] + x*m[4] + y*m[5] + z*m[6];
    m[11] = m[11] + x*m[8] + y*m[9] + z*m[10];
#endif
#ifdef COLUMN_MAJOR
    m[12] = m[12] + x*m[0] + y*m[4] + z*m[8];
    m[13] = m[13] + x*m[1] + y*m[5] + z*m[9];
    m[14] = m[14] + x*m[2] + y*m[6] + z*m[10];
#endif
  };

  template <class T>
  void Mat4x4<T>::rightMultTranslation(const Vec3<T>& trans) {
    this->rightMultTranslation(trans[0], trans[1], trans[2]);
  };
  
  template <class T>
  void Mat4x4<T>::leftMultTranslation(const T x, const T y, const T z) {
#ifdef ROW_MAJOR
    m[3] = m[3] + x;
    m[7] = m[7] + y;
    m[11] = m[11] + z;
#endif
#ifdef COLUMN_MAJOR
    m[12] = m[12] + x;
    m[13] = m[13] + y;
    m[14] = m[14] + z;
#endif
  };

  template <class T>
  void Mat4x4<T>::leftMultTranslation(const Vec3<T>& trans) {
    this->leftMultTranslation(trans[0], trans[1], trans[2]);
  };
  
  template <class T>
  void Mat4x4<T>::rightMultScale(const T x, const T y, const T z) {
#ifdef ROW_MAJOR
    m[0] *= x;
    m[1] *= y;
    m[2] *= z;
    m[4] *= x;
    m[5] *= y;
    m[6] *= z;
    m[8] *= x;
    m[9] *= y;
    m[10] *= z;
    m[12] *= x;
    m[13] *= y;
    m[14] *= z;
#endif
#ifdef COLUMN_MAJOR
    m[0] *= x;
    m[1] *= x;
    m[2] *= x;
    m[3] *= x;
    m[4] *= y;
    m[5] *= y;
    m[6] *= y;
    m[7] *= y;
    m[8] *= z;
    m[9] *= z;
    m[10] *= z;
    m[11] *= z;
#endif
  };
  
  template <class T>
  void Mat4x4<T>::leftMultScale(const T x, const T y, const T z) {
#ifdef ROW_MAJOR
    m[0] *= x;
    m[1] *= x;
    m[2] *= x;
    m[3] *= x;
    m[4] *= y;
    m[5] *= y;
    m[6] *= y;
    m[7] *= y;
    m[8] *= z;
    m[9] *= z;
    m[10] *= z;
    m[11] *= z;
#endif
#ifdef COLUMN_MAJOR
    m[0] *= x;
    m[1] *= y;
    m[2] *= z;
    m[4] *= x;
    m[5] *= y;
    m[6] *= z;
    m[8] *= x;
    m[9] *= y;
    m[10] *= z;
    m[12] *= x;
    m[13] *= y;
    m[14] *= z;
#endif
  };

  template <class T>
  void Mat4x4<T>::leftMultRotateXAxis(const T angle) {
    // See utilities/rot_mat.m for symbolic toolbox source code
    T c = cos(angle);
    T s = sin(angle);
#ifdef ROW_MAJOR
    T m4 = m[4];
    T m5 = m[5];
    T m6 = m[6];
    T m7 = m[7];
    m[4] = c * m[4] - s * m[8];
    m[5] = c * m[5] - s * m[9];
    m[6] = c * m[6] - s * m[10];
    m[7] = c * m[7] - s * m[11];
    m[8] = c * m[8] + s * m4;
    m[9] = c * m[9] + s * m5;
    m[10] = c * m[10] + s * m6;
    m[11] = c * m[11] + s * m7;
#endif
#ifdef COLUMN_MAJOR
    T m1 = m[1];
    T m5 = m[5];
    T m9 = m[9];
    T m13 = m[13];
    m[1] = c * m[1] - s * m[2];
    m[5] = c * m[5] - s * m[6];
    m[9] = c * m[9] - s * m[10];
    m[13] = c * m[13] - s * m[14];
    m[2] = c * m[2] + s * m1;
    m[6] = c * m[6] + s * m5;
    m[10] = c * m[10] + s * m9;
    m[14] = c * m[14] + s * m13;
#endif
  };
 
  template <class T>
  void Mat4x4<T>::leftMultRotateYAxis(const T angle) {
    // See utilities/rot_mat.m for symbolic toolbox source code
    T c = cos(angle);
    T s = sin(angle);
#ifdef ROW_MAJOR
    T m0 = m[0];
    T m1 = m[1];
    T m2 = m[2];
    T m3 = m[3];
    m[0] = c * m[0] + s * m[8];
    m[1] = c * m[1] + s * m[9];
    m[2] = c * m[2] + s * m[10];
    m[3] = c * m[3] + s * m[11];
    m[8] = c * m[8] - s * m0;
    m[9] = c * m[9] - s * m1;
    m[10] = c * m[10] - s * m2;
    m[11] = c * m[11] - s * m3;
#endif
#ifdef COLUMN_MAJOR
    T m0 = m[0];
    T m4 = m[4];
    T m8 = m[8];
    T m12 = m[12];
    m[0] = c * m[0] + s * m[2];
    m[4] = c * m[4] + s * m[6];
    m[8] = c * m[8] + s * m[10];
    m[12] = c * m[12] + s * m[14];
    m[2] = c * m[2] - s * m0;
    m[6] = c * m[6] - s * m4;
    m[10] = c * m[10] - s * m8;
    m[14] = c * m[14] - s * m12;
#endif
  };

  template <class T>
  void Mat4x4<T>::leftMultRotateZAxis(const T angle) {
    // See utilities/rot_mat.m for symbolic toolbox source code
    T c = cos(angle);
    T s = sin(angle);
#ifdef ROW_MAJOR
    T m0 = m[0];
    T m1 = m[1];
    T m2 = m[2];
    T m3 = m[3];
    m[0] = c * m[0] - s * m[4];
    m[1] = c * m[1] - s * m[5];
    m[2] = c * m[2] - s * m[6];
    m[3] = c * m[3] - s * m[7];
    m[4] = c * m[4] + s * m0;
    m[5] = c * m[5] + s * m1;
    m[6] = c * m[6] + s * m2;
    m[7] = c * m[7] + s * m3;
#endif
#ifdef COLUMN_MAJOR
    T m0 = m[0];
    T m4 = m[4];
    T m8 = m[8];
    T m12 = m[12];
    m[0] = c * m[0] - s * m[1];
    m[4] = c * m[4] - s * m[5];
    m[8] = c * m[8] - s * m[9];
    m[12] = c * m[12] - s * m[13];
    m[1] = c * m[1] + s * m0;
    m[5] = c * m[5] + s * m4;
    m[9] = c * m[9] + s * m8;
    m[13] = c * m[13] + s * m12;
#endif
  };

  template <class T>
  void Mat4x4<T>::rightMultRotateXAxis(const T angle) {
    // See utilities/rot_mat.m for symbolic toolbox source code
    T c = cos(angle);
    T s = sin(angle);
#ifdef ROW_MAJOR
    T m1 = m[1];
    T m5 = m[5];
    T m9 = m[9];
    T m13 = m[13];
    m[1] = c * m[1] + s * m[2];
    m[5] = c * m[5] + s * m[6];
    m[9] = c * m[9] + s * m[10];
    m[13] = c * m[13] + s * m[14];
    m[2] = c * m[2] - s * m1;
    m[6] = c * m[6] - s * m5;
    m[10] = c * m[10] - s * m9;
    m[14] = c * m[14] - s * m13;
#endif
#ifdef COLUMN_MAJOR
    T m4 = m[4];
    T m5 = m[5];
    T m6 = m[6];
    T m7 = m[7];
    m[4] = c * m[4] + s * m[8];
    m[5] = c * m[5] + s * m[9];
    m[6] = c * m[6] + s * m[10];
    m[7] = c * m[7] + s * m[11];
    m[8] = c * m[8] - s * m4;
    m[9] = c * m[9] - s * m5;
    m[10] = c * m[10] - s * m6;
    m[11] = c * m[11] - s * m7;
#endif
  };

  template <class T>
  void Mat4x4<T>::rightMultRotateYAxis(const T angle) {
    // See utilities/rot_mat.m for symbolic toolbox source code
    T c = cos(angle);
    T s = sin(angle);
#ifdef ROW_MAJOR
    T m0 = m[0];
    T m4 = m[4];
    T m8 = m[8];
    T m12 = m[12];
    m[0] = c * m[0] - s * m[2];
    m[4] = c * m[4] - s * m[6];
    m[8] = c * m[8] - s * m[10];
    m[12] = c * m[12] - s * m[14];
    m[2] = c * m[2] + s * m0;
    m[6] = c * m[6] + s * m4;
    m[10] = c * m[10] + s * m8;
    m[14] = c * m[14] + s * m12;
#endif
#ifdef COLUMN_MAJOR
    T m0 = m[0];
    T m1 = m[1];
    T m2 = m[2];
    T m3 = m[3];
    m[0] = c * m[0] - s * m[8];
    m[1] = c * m[1] - s * m[9];
    m[2] = c * m[2] - s * m[10];
    m[3] = c * m[3] - s * m[11];
    m[8] = c * m[8] + s * m0;
    m[9] = c * m[9] + s * m1;
    m[10] = c * m[10] + s * m2;
    m[11] = c * m[11] + s * m3;
#endif
  };

  template <class T>
  void Mat4x4<T>::rightMultRotateZAxis(const T angle) {
    // See utilities/rot_mat.m for symbolic toolbox source code
    T c = cos(angle);
    T s = sin(angle);
#ifdef ROW_MAJOR
    T m0 = m[0];
    T m4 = m[4];
    T m8 = m[8];
    T m12 = m[12];
    m[0] = c * m[0] + s * m[1];
    m[4] = c * m[4] + s * m[5];
    m[8] = c * m[8] + s * m[9];
    m[12] = c * m[12] + s * m[13];
    m[1] = c * m[1] - s * m0;
    m[5] = c * m[5] - s * m4;
    m[9] = c * m[9] - s * m8;
    m[13] = c * m[13] - s * m12;
#endif
#ifdef COLUMN_MAJOR
    T m0 = m[0];
    T m1 = m[1];
    T m2 = m[2];
    T m3 = m[3];
    m[0] = c * m[0] + s * m[4];
    m[1] = c * m[1] + s * m[5];
    m[2] = c * m[2] + s * m[6];
    m[3] = c * m[3] + s * m[7];
    m[4] = c * m[4] - s * m0;
    m[5] = c * m[5] - s * m1;
    m[6] = c * m[6] - s * m2;
    m[7] = c * m[7] - s * m3;
#endif
  };

  template <class T>
  void Mat4x4<T>::rotateMatAxisAngle(const Vec3<T>& axis, const T angle) {
    rotateMatAxisAngle(*this, axis, angle);
  };
  
  template <class T>
  void Mat4x4<T>::scaleMat(const T x, const T y, const T z) {
    scaleMat(*this, x, y, z);
  };
  
  template <class T>
  void Mat4x4<T>::translationMat(const T x, const T y, const T z) {
    translationMat(*this, x, y, z);
  };
  
  template <class T>
  void Mat4x4<T>::glProjection(const T znear, const T zfar, const T fov_deg,
    const T screen_width, const T screen_height) {
    glProjection(*this, znear, zfar, fov_deg, screen_width, screen_height);
  };
  
  template <class T>
  void Mat4x4<T>::glOrthoProjection(const T znear, const T zfar,
    const T left, const T right, const T bottom, const T top) {
    glOrthoProjection(*this, znear, zfar, left, right, bottom, top);
  };
  
  template <class T>
  void Mat4x4<T>::glLookAt(const Vec3<T>& up, const Vec3<T>& forward,
                           const Vec3<T>& pos) {
    glLookAt(*this, up, forward, pos);
  };
  
  // Math operations
  template <class T>
  void Mat4x4<T>::transpose() {
    T temp;
    temp = m[1];  // Swap 1 and 4
    m[1] = m[4];
    m[4] = temp;
    temp = m[2];  // Swap 2 and 8
    m[2] = m[8];
    m[8] = temp;
    temp = m[3];  // Swap 3 and 12
    m[3] = m[12];
    m[12] = temp;
    temp = m[6];  // Swap 6 and 9
    m[6] = m[9];
    m[9] = temp;
    temp = m[7];  // Swap 7 and 13
    m[7] = m[13];
    m[13] = temp;
    temp = m[11];  // Swap 11 and 14
    m[11] = m[14];
    m[14] = temp;
  };
  
  // Static Math operations
  // 4x4 matrix determinant taken from here:
  // http://www.geometrictools.com/LibMathematics/Algebra/Wm5Matrix4.inl
  // det(A) = det(A^t) (so row and column major have the same determinent)
  template <class T>
  T Mat4x4<T>::det(const Mat4x4& a) {
    T a0 = a.m[ 0]*a.m[ 5] - a.m[ 1]*a.m[ 4];
    T a1 = a.m[ 0]*a.m[ 6] - a.m[ 2]*a.m[ 4];
    T a2 = a.m[ 0]*a.m[ 7] - a.m[ 3]*a.m[ 4];
    T a3 = a.m[ 1]*a.m[ 6] - a.m[ 2]*a.m[ 5];
    T a4 = a.m[ 1]*a.m[ 7] - a.m[ 3]*a.m[ 5];
    T a5 = a.m[ 2]*a.m[ 7] - a.m[ 3]*a.m[ 6];
    T b0 = a.m[ 8]*a.m[13] - a.m[ 9]*a.m[12];
    T b1 = a.m[ 8]*a.m[14] - a.m[10]*a.m[12];
    T b2 = a.m[ 8]*a.m[15] - a.m[11]*a.m[12];
    T b3 = a.m[ 9]*a.m[14] - a.m[10]*a.m[13];
    T b4 = a.m[ 9]*a.m[15] - a.m[11]*a.m[13];
    T b5 = a.m[10]*a.m[15] - a.m[11]*a.m[14];
    return a0*b5 - a1*b4 + a2*b3 + a3*b2 - a4*b1 + a5*b0;
  };
  
  template <class T>
  T Mat4x4<T>::trace(const Mat4x4& a) {  // Sum of elements on main diagonal
    return a.m[0] + a.m[5] + a.m[10] + a.m[15];
  };
  
  template <class T>
  void Mat4x4<T>::scale(Mat4x4& ret, const Mat4x4& a, const T s) {
    ret.m[0] = a.m[0] * s;
    ret.m[1] = a.m[1] * s;
    ret.m[2] = a.m[2] * s;
    ret.m[3] = a.m[3] * s;
    ret.m[4] = a.m[4] * s;
    ret.m[5] = a.m[5] * s;
    ret.m[6] = a.m[6] * s;
    ret.m[7] = a.m[7] * s;
    ret.m[8] = a.m[8] * s;
    ret.m[9] = a.m[9] * s;
    ret.m[10] = a.m[10] * s;
    ret.m[11] = a.m[11] * s;
    ret.m[12] = a.m[12] * s;
    ret.m[13] = a.m[13] * s;
    ret.m[14] = a.m[14] * s;
    ret.m[15] = a.m[15] * s;
  };
  
  template <class T>
  void Mat4x4<T>::sub(Mat4x4& ret, const Mat4x4& a, const Mat4x4& b) {
    ret.m[0] = a.m[0] - b.m[0];
    ret.m[1] = a.m[1] - b.m[1];
    ret.m[2] = a.m[2] - b.m[2];
    ret.m[3] = a.m[3] - b.m[3];
    ret.m[4] = a.m[4] - b.m[4];
    ret.m[5] = a.m[5] - b.m[5];
    ret.m[6] = a.m[6] - b.m[6];
    ret.m[7] = a.m[7] - b.m[7];
    ret.m[8] = a.m[8] - b.m[8];
    ret.m[9] = a.m[9] - b.m[9];
    ret.m[10] = a.m[10] - b.m[10];
    ret.m[11] = a.m[11] - b.m[11];
    ret.m[12] = a.m[12] - b.m[12];
    ret.m[13] = a.m[13] - b.m[13];
    ret.m[14] = a.m[14] - b.m[14];
    ret.m[15] = a.m[15] - b.m[15];
  };
  
  template <class T>
  void Mat4x4<T>::add(Mat4x4& ret, const Mat4x4& a, const Mat4x4& b) {
    ret.m[0] = a.m[0] + b.m[0];
    ret.m[1] = a.m[1] + b.m[1];
    ret.m[2] = a.m[2] + b.m[2];
    ret.m[3] = a.m[3] + b.m[3];
    ret.m[4] = a.m[4] + b.m[4];
    ret.m[5] = a.m[5] + b.m[5];
    ret.m[6] = a.m[6] + b.m[6];
    ret.m[7] = a.m[7] + b.m[7];
    ret.m[8] = a.m[8] + b.m[8];
    ret.m[9] = a.m[9] + b.m[9];
    ret.m[10] = a.m[10] + b.m[10];
    ret.m[11] = a.m[11] + b.m[11];
    ret.m[12] = a.m[12] + b.m[12];
    ret.m[13] = a.m[13] + b.m[13];
    ret.m[14] = a.m[14] + b.m[14];
    ret.m[15] = a.m[15] + b.m[15];
  };
  
  template <class T>
  void Mat4x4<T>::pairwiseMult(Mat4x4& ret, const Mat4x4& a, const Mat4x4& b) {
    ret.m[0] = a.m[0] * b.m[0];
    ret.m[1] = a.m[1] * b.m[1];
    ret.m[2] = a.m[2] * b.m[2];
    ret.m[3] = a.m[3] * b.m[3];
    ret.m[4] = a.m[4] * b.m[4];
    ret.m[5] = a.m[5] * b.m[5];
    ret.m[6] = a.m[6] * b.m[6];
    ret.m[7] = a.m[7] * b.m[7];
    ret.m[8] = a.m[8] * b.m[8];
    ret.m[9] = a.m[9] * b.m[9];
    ret.m[10] = a.m[10] * b.m[10];
    ret.m[11] = a.m[11] * b.m[11];
    ret.m[12] = a.m[12] * b.m[12];
    ret.m[13] = a.m[13] * b.m[13];
    ret.m[14] = a.m[14] * b.m[14];
    ret.m[15] = a.m[15] * b.m[15];
  };
  
  // 4x4 matrix invert taken from here:
  // http://www.geometrictools.com/LibMathematics/Algebra/Wm5Matrix4.inl
  template <class T>
  void Mat4x4<T>::inverse(Mat4x4& ret, const Mat4x4& a) {
#ifdef ROW_MAJOR
    T a0 = a.m[ 0]*a.m[ 5] - a.m[ 1]*a.m[ 4];
    T a1 = a.m[ 0]*a.m[ 6] - a.m[ 2]*a.m[ 4];
    T a2 = a.m[ 0]*a.m[ 7] - a.m[ 3]*a.m[ 4];
    T a3 = a.m[ 1]*a.m[ 6] - a.m[ 2]*a.m[ 5];
    T a4 = a.m[ 1]*a.m[ 7] - a.m[ 3]*a.m[ 5];
    T a5 = a.m[ 2]*a.m[ 7] - a.m[ 3]*a.m[ 6];
    T b0 = a.m[ 8]*a.m[13] - a.m[ 9]*a.m[12];
    T b1 = a.m[ 8]*a.m[14] - a.m[10]*a.m[12];
    T b2 = a.m[ 8]*a.m[15] - a.m[11]*a.m[12];
    T b3 = a.m[ 9]*a.m[14] - a.m[10]*a.m[13];
    T b4 = a.m[ 9]*a.m[15] - a.m[11]*a.m[13];
    T b5 = a.m[10]*a.m[15] - a.m[11]*a.m[14];
    
    T det = a0*b5 - a1*b4 + a2*b3 + a3*b2 - a4*b1 + a5*b0;
    if (fabs(det) > EPSILON) {
      ret.m[ 0] = + a.m[ 5]*b5 - a.m[ 6]*b4 + a.m[ 7]*b3;
      ret.m[ 4] = - a.m[ 4]*b5 + a.m[ 6]*b2 - a.m[ 7]*b1;
      ret.m[ 8] = + a.m[ 4]*b4 - a.m[ 5]*b2 + a.m[ 7]*b0;
      ret.m[12] = - a.m[ 4]*b3 + a.m[ 5]*b1 - a.m[ 6]*b0;
      ret.m[ 1] = - a.m[ 1]*b5 + a.m[ 2]*b4 - a.m[ 3]*b3;
      ret.m[ 5] = + a.m[ 0]*b5 - a.m[ 2]*b2 + a.m[ 3]*b1;
      ret.m[ 9] = - a.m[ 0]*b4 + a.m[ 1]*b2 - a.m[ 3]*b0;
      ret.m[13] = + a.m[ 0]*b3 - a.m[ 1]*b1 + a.m[ 2]*b0;
      ret.m[ 2] = + a.m[13]*a5 - a.m[14]*a4 + a.m[15]*a3;
      ret.m[ 6] = - a.m[12]*a5 + a.m[14]*a2 - a.m[15]*a1;
      ret.m[10] = + a.m[12]*a4 - a.m[13]*a2 + a.m[15]*a0;
      ret.m[14] = - a.m[12]*a3 + a.m[13]*a1 - a.m[14]*a0;
      ret.m[ 3] = - a.m[ 9]*a5 + a.m[10]*a4 - a.m[11]*a3;
      ret.m[ 7] = + a.m[ 8]*a5 - a.m[10]*a2 + a.m[11]*a1;
      ret.m[11] = - a.m[ 8]*a4 + a.m[ 9]*a2 - a.m[11]*a0;
      ret.m[15] = + a.m[ 8]*a3 - a.m[ 9]*a1 + a.m[10]*a0;
      
      T invDet = (T)1/det;
      Mat4x4<T>::scale(ret, ret, invDet);
    } else {
      ret.zeros();
    }
#endif
#ifdef COLUMN_MAJOR
    T a0 = a.m[0]*a.m[5] - a.m[4]*a.m[1];
    T a1 = a.m[0]*a.m[9] - a.m[8]*a.m[1];
    T a2 = a.m[0]*a.m[13] - a.m[12]*a.m[1];
    T a3 = a.m[4]*a.m[9] - a.m[8]*a.m[5];
    T a4 = a.m[4]*a.m[13] - a.m[12]*a.m[5];
    T a5 = a.m[8]*a.m[13] - a.m[12]*a.m[9];
    T b0 = a.m[2]*a.m[7] - a.m[6]*a.m[3];
    T b1 = a.m[2]*a.m[11] - a.m[10]*a.m[3];
    T b2 = a.m[2]*a.m[15] - a.m[14]*a.m[3];
    T b3 = a.m[6]*a.m[11] - a.m[10]*a.m[7];
    T b4 = a.m[6]*a.m[15] - a.m[14]*a.m[7];
    T b5 = a.m[10]*a.m[15] - a.m[14]*a.m[11];
    
    T det = a0*b5 - a1*b4 + a2*b3 + a3*b2 - a4*b1 + a5*b0;
    if (fabs(det) > EPSILON) {
      ret.m[0] = + a.m[5]*b5 - a.m[9]*b4 + a.m[13]*b3;
      ret.m[1] = - a.m[1]*b5 + a.m[9]*b2 - a.m[13]*b1;
      ret.m[2] = + a.m[1]*b4 - a.m[5]*b2 + a.m[13]*b0;
      ret.m[3] = - a.m[1]*b3 + a.m[5]*b1 - a.m[9]*b0;
      ret.m[4] = - a.m[4]*b5 + a.m[8]*b4 - a.m[12]*b3;
      ret.m[5] = + a.m[0]*b5 - a.m[8]*b2 + a.m[12]*b1;
      ret.m[6] = - a.m[0]*b4 + a.m[4]*b2 - a.m[12]*b0;
      ret.m[7] = + a.m[0]*b3 - a.m[4]*b1 + a.m[8]*b0;
      ret.m[8] = + a.m[7]*a5 - a.m[11]*a4 + a.m[15]*a3;
      ret.m[9] = - a.m[3]*a5 + a.m[11]*a2 - a.m[15]*a1;
      ret.m[10] = + a.m[3]*a4 - a.m[7]*a2 + a.m[15]*a0;
      ret.m[11] = - a.m[3]*a3 + a.m[7]*a1 - a.m[11]*a0;
      ret.m[12] = - a.m[6]*a5 + a.m[10]*a4 - a.m[14]*a3;
      ret.m[13] = + a.m[2]*a5 - a.m[10]*a2 + a.m[14]*a1;
      ret.m[14] = - a.m[2]*a4 + a.m[6]*a2 - a.m[14]*a0;
      ret.m[15] = + a.m[2]*a3 - a.m[6]*a1 + a.m[10]*a0;
      
      T invDet = (T)1/det;
      Mat4x4<T>::scale(ret, ret, invDet);
    } else {
      ret.zeros();
    }
#endif
  };
  
  template <class T>
  void Mat4x4<T>::mult(Mat4x4& ret, const Mat4x4& a, const Mat4x4& b) {
#ifdef ROW_MAJOR
    ret.m[0] = a.m[0]*b.m[0] + a.m[1]*b.m[4] + a.m[2]*b.m[8] +
    a.m[3]*b.m[12];
    ret.m[1] = a.m[0]*b.m[1] + a.m[1]*b.m[5] + a.m[2]*b.m[9] +
    a.m[3]*b.m[13];
    ret.m[2] = a.m[0]*b.m[2] + a.m[1]*b.m[6] + a.m[2]*b.m[10] +
    a.m[3]*b.m[14];
    ret.m[3] = a.m[0]*b.m[3] + a.m[1]*b.m[7] + a.m[2]*b.m[11] +
    a.m[3]*b.m[15];
    ret.m[4] = a.m[4]*b.m[0] + a.m[5]*b.m[4] + a.m[6]*b.m[8] +
    a.m[7]*b.m[12];
    ret.m[5] = a.m[4]*b.m[1] + a.m[5]*b.m[5] + a.m[6]*b.m[9] +
    a.m[7]*b.m[13];
    ret.m[6] = a.m[4]*b.m[2] + a.m[5]*b.m[6] + a.m[6]*b.m[10] +
    a.m[7]*b.m[14];
    ret.m[7] = a.m[4]*b.m[3] + a.m[5]*b.m[7] + a.m[6]*b.m[11] +
    a.m[7]*b.m[15];
    ret.m[8] = a.m[8]*b.m[0] + a.m[9]*b.m[4] + a.m[10]*b.m[8] +
    a.m[11]*b.m[12];
    ret.m[9] = a.m[8]*b.m[1] + a.m[9]*b.m[5] + a.m[10]*b.m[9] +
    a.m[11]*b.m[13];
    ret.m[10] = a.m[8]*b.m[2] + a.m[9]*b.m[6] + a.m[10]*b.m[10] +
    a.m[11]*b.m[14];
    ret.m[11] = a.m[8]*b.m[3] + a.m[9]*b.m[7] + a.m[10]*b.m[11] +
    a.m[11]*b.m[15];
    ret.m[12] = a.m[12]*b.m[0] + a.m[13]*b.m[4] + a.m[14]*b.m[8] +
    a.m[15]*b.m[12];
    ret.m[13] = a.m[12]*b.m[1] + a.m[13]*b.m[5] + a.m[14]*b.m[9] +
    a.m[15]*b.m[13];
    ret.m[14] = a.m[12]*b.m[2] + a.m[13]*b.m[6] + a.m[14]*b.m[10] +
    a.m[15]*b.m[14];
    ret.m[15] = a.m[12]*b.m[3] + a.m[13]*b.m[7] + a.m[14]*b.m[11] +
    a.m[15]*b.m[15];
#endif
#ifdef COLUMN_MAJOR
    ret.m[0] = a.m[0]*b.m[0] + a.m[4]*b.m[1] + a.m[8]*b.m[2] +
    a.m[12]*b.m[3];
    ret.m[4] = a.m[0]*b.m[4] + a.m[4]*b.m[5] + a.m[8]*b.m[6] +
    a.m[12]*b.m[7];
    ret.m[8] = a.m[0]*b.m[8] + a.m[4]*b.m[9] + a.m[8]*b.m[10] +
    a.m[12]*b.m[11];
    ret.m[12] = a.m[0]*b.m[12] + a.m[4]*b.m[13] + a.m[8]*b.m[14] +
    a.m[12]*b.m[15];
    ret.m[1] = a.m[1]*b.m[0] + a.m[5]*b.m[1] + a.m[9]*b.m[2] +
    a.m[13]*b.m[3];
    ret.m[5] = a.m[1]*b.m[4] + a.m[5]*b.m[5] + a.m[9]*b.m[6] +
    a.m[13]*b.m[7];
    ret.m[9] = a.m[1]*b.m[8] + a.m[5]*b.m[9] + a.m[9]*b.m[10] +
    a.m[13]*b.m[11];
    ret.m[13] = a.m[1]*b.m[12] + a.m[5]*b.m[13] + a.m[9]*b.m[14] +
    a.m[13]*b.m[15];
    ret.m[2] = a.m[2]*b.m[0] + a.m[6]*b.m[1] + a.m[10]*b.m[2] +
    a.m[14]*b.m[3];
    ret.m[6] = a.m[2]*b.m[4] + a.m[6]*b.m[5] + a.m[10]*b.m[6] +
    a.m[14]*b.m[7];
    ret.m[10] = a.m[2]*b.m[8] + a.m[6]*b.m[9] + a.m[10]*b.m[10] +
    a.m[14]*b.m[11];
    ret.m[14] = a.m[2]*b.m[12] + a.m[6]*b.m[13] + a.m[10]*b.m[14] +
    a.m[14]*b.m[15];
    ret.m[3] = a.m[3]*b.m[0] + a.m[7]*b.m[1] + a.m[11]*b.m[2] +
    a.m[15]*b.m[3];
    ret.m[7] = a.m[3]*b.m[4] + a.m[7]*b.m[5] + a.m[11]*b.m[6] +
    a.m[15]*b.m[7];
    ret.m[11] = a.m[3]*b.m[8] + a.m[7]*b.m[9] + a.m[11]*b.m[10] +
    a.m[15]*b.m[11];
    ret.m[15] = a.m[3]*b.m[12] + a.m[7]*b.m[13] + a.m[11]*b.m[14] +
    a.m[15]*b.m[15];
#endif
  };

  template <class T>
  void Mat4x4<T>::multSIMD(Mat4x4& ret, const Mat4x4& m1, const Mat4x4& m2) {
#ifdef ROW_MAJOR
#error "not yet supported"
#endif
#ifdef COLUMN_MAJOR
  const float* a = m1.m;
  const float* b = m2.m;
  float* r = ret.m;

  __m128 a_line, b_line, r_line;
  for (int i=0; i<16; i+=4) {
    // unroll the first step of the loop to avoid having to initialize r_line to zero
    a_line = _mm_load_ps(a);         // a_line = vec4(column(a, 0))
    b_line = _mm_set1_ps(b[i]);      // b_line = vec4(b[i][0])
    r_line = _mm_mul_ps(a_line, b_line); // r_line = a_line * b_line
    for (int j=1; j<4; j++) {
      a_line = _mm_load_ps(&a[j*4]); // a_line = vec4(column(a, j))
      b_line = _mm_set1_ps(b[i+j]);  // b_line = vec4(b[i][j])
                                     // r_line += a_line * b_line
      r_line = _mm_add_ps(_mm_mul_ps(a_line, b_line), r_line);
    }
    _mm_store_ps(&r[i], r_line);     // r[i] = r_line
  }
#endif
  };
  
  template <class T>
  void Mat4x4<T>::transpose(Mat4x4& ret, const Mat4x4& a) {
    ret.m[0] = a.m[0];
    ret.m[1] = a.m[4];
    ret.m[2] = a.m[8];
    ret.m[3] = a.m[12];
    ret.m[4] = a.m[1];
    ret.m[5] = a.m[5];
    ret.m[6] = a.m[9];
    ret.m[7] = a.m[13];
    ret.m[8] = a.m[2];
    ret.m[9] = a.m[6];
    ret.m[10] = a.m[10];
    ret.m[11] = a.m[14];
    ret.m[12] = a.m[3];
    ret.m[13] = a.m[7];
    ret.m[14] = a.m[11];
    ret.m[15] = a.m[15];
  };
  
  template <class T>
  bool Mat4x4<T>::equal(const Mat4x4& a, const Mat4x4& b) {
    for (int i = 0; i < 16; i ++)
      if (abs(a.m[i] - b.m[i]) > EPSILON)
        return false;
    return true;
  };

  template <class T>
  bool Mat4x4<T>::approxEqual(const Mat4x4& a, const Mat4x4& b) {
    for (int i = 0; i < 16; i ++)
      if (abs(a.m[i] - b.m[i]) > LOOSE_EPSILON)
        return false;
    return true;
  };
  
  template <class T>
  void Mat4x4<T>::rotateMatXAxis(Mat4x4& ret, const T angle) {
    T cos_angle = cos(angle);
    T sin_angle = sin(angle);
#ifdef ROW_MAJOR
    ret.m[0] = (T)1;
    ret.m[1] = (T)0;
    ret.m[2] = (T)0;
    ret.m[3] = (T)0;
    ret.m[4] = (T)0;
    ret.m[5] = cos_angle;
    ret.m[6] = -sin_angle;
    ret.m[7] = (T)0;
    ret.m[8] = (T)0;
    ret.m[9] = sin_angle;
    ret.m[10] = cos_angle;
    ret.m[11] = (T)0;
    ret.m[12] = (T)0;
    ret.m[13] = (T)0;
    ret.m[14] = (T)0;
    ret.m[15] = (T)1;
#endif
#ifdef COLUMN_MAJOR
    ret.m[0] = (T)1;
    ret.m[1] = (T)0;
    ret.m[2] = (T)0;
    ret.m[3] = (T)0;
    ret.m[4] = (T)0;
    ret.m[5] = cos_angle;
    ret.m[6] = sin_angle;
    ret.m[7] = (T)0;
    ret.m[8] = (T)0;
    ret.m[9] = -sin_angle;
    ret.m[10] = cos_angle;
    ret.m[11] = (T)0;
    ret.m[12] = (T)0;
    ret.m[13] = (T)0;
    ret.m[14] = (T)0;
    ret.m[15] = (T)1;
#endif
  };
  
  template <class T>
  void Mat4x4<T>::rotateMatYAxis(Mat4x4& ret, const T angle) {
    T cos_angle = cos(angle);
    T sin_angle = sin(angle);
#ifdef ROW_MAJOR
    ret.m[0] = cos_angle;
    ret.m[1] = (T)0;
    ret.m[2] = sin_angle;
    ret.m[3] = (T)0;
    ret.m[4] = (T)0;
    ret.m[5] = (T)1;
    ret.m[6] = (T)0;
    ret.m[7] = (T)0;
    ret.m[8] = -sin_angle;
    ret.m[9] = (T)0;
    ret.m[10] = cos_angle;
    ret.m[11] = (T)0;
    ret.m[12] = (T)0;
    ret.m[13] = (T)0;
    ret.m[14] = (T)0;
    ret.m[15] = (T)1;
#endif
#ifdef COLUMN_MAJOR
    ret.m[0] = cos_angle;
    ret.m[1] = (T)0;
    ret.m[2] = -sin_angle;
    ret.m[3] = (T)0;
    ret.m[4] = (T)0;
    ret.m[5] = (T)1;
    ret.m[6] = (T)0;
    ret.m[7] = (T)0;
    ret.m[8] = sin_angle;
    ret.m[9] = (T)0;
    ret.m[10] = cos_angle;
    ret.m[11] = (T)0;
    ret.m[12] = (T)0;
    ret.m[13] = (T)0;
    ret.m[14] = (T)0;
    ret.m[15] = (T)1;
#endif
  };
  
  template <class T>
  void Mat4x4<T>::rotateMatZAxis(Mat4x4& ret, const T angle) {
    T cos_angle = cos(angle);
    T sin_angle = sin(angle);
#ifdef ROW_MAJOR
    ret.m[0] = cos_angle;
    ret.m[1] = -sin_angle;
    ret.m[2] = (T)0;
    ret.m[3] = (T)0;
    ret.m[4] = sin_angle;
    ret.m[5] = cos_angle;
    ret.m[6] = (T)0;
    ret.m[7] = (T)0;
    ret.m[8] = (T)0;
    ret.m[9] = (T)0;
    ret.m[10] = (T)1;
    ret.m[11] = (T)0;
    ret.m[12] = (T)0;
    ret.m[13] = (T)0;
    ret.m[14] = (T)0;
    ret.m[15] = (T)1;
#endif
#ifdef COLUMN_MAJOR
    ret.m[0] = cos_angle;
    ret.m[1] = sin_angle;
    ret.m[2] = (T)0;
    ret.m[3] = (T)0;
    ret.m[4] = -sin_angle;
    ret.m[5] = cos_angle;
    ret.m[6] = (T)0;
    ret.m[7] = (T)0;
    ret.m[8] = (T)0;
    ret.m[9] = (T)0;
    ret.m[10] = (T)1;
    ret.m[11] = (T)0;
    ret.m[12] = (T)0;
    ret.m[13] = (T)0;
    ret.m[14] = (T)0;
    ret.m[15] = (T)1;
#endif
  };
  
  template <class T>
  void Mat4x4<T>::rotateMatAxisAngle(Mat4x4& ret, const Vec3<T>& axis,
    const T angle) {
    // http://en.wikipedia.org/wiki/Rotation_matrix
    Vec3<T> axis_(const_cast<Vec3<T>&>(axis));
    Vec3<T>::normalize(axis_);  // just in case
    T xy = axis_.m[0] * axis_.m[1];
    T xz = axis_.m[0] * axis_.m[2];
    T yz = axis_.m[1] * axis_.m[2];
    T cos_angle = cos(angle);
    T one_minus_cos_angle = 1 - cos(angle);
    T sin_angle = sin(angle);
#ifdef ROW_MAJOR
    ret.m[0] = cos_angle + axis_.m[0]*axis_.m[0]*one_minus_cos_angle;
    ret.m[4] = xy*one_minus_cos_angle + axis_.m[2]*sin_angle;
    ret.m[8] = xz*one_minus_cos_angle - axis_.m[1]*sin_angle;
    ret.m[12] = (T)0;
    ret.m[1] = xy*one_minus_cos_angle - axis_.m[2]*sin_angle;
    ret.m[5] = cos_angle + axis_.m[1]*axis_.m[1]*one_minus_cos_angle;
    ret.m[9] = yz*one_minus_cos_angle + axis_.m[0]*sin_angle;
    ret.m[13] = (T)0;
    ret.m[2] = xz*one_minus_cos_angle + axis_.m[1]*sin_angle;
    ret.m[6] = yz*one_minus_cos_angle - axis_.m[0]*sin_angle;
    ret.m[10] = cos_angle + axis_.m[2]*axis_.m[2]*one_minus_cos_angle;
    ret.m[14] = (T)0;
    ret.m[3] = (T)0;
    ret.m[7] = (T)0;
    ret.m[11] = (T)0;
    ret.m[15] = (T)1;
#endif
#ifdef COLUMN_MAJOR
    ret.m[0] = cos_angle + axis_.m[0]*axis_.m[0]*one_minus_cos_angle;
    ret.m[1] = xy*one_minus_cos_angle + axis_.m[2]*sin_angle;
    ret.m[2] = xz*one_minus_cos_angle - axis_.m[1]*sin_angle;
    ret.m[3] = (T)0;
    ret.m[4] = xy*one_minus_cos_angle - axis_.m[2]*sin_angle;
    ret.m[5] = cos_angle + axis_.m[1]*axis_.m[1]*one_minus_cos_angle;
    ret.m[6] = yz*one_minus_cos_angle + axis_.m[0]*sin_angle;
    ret.m[7] = (T)0;
    ret.m[8] = xz*one_minus_cos_angle + axis_.m[1]*sin_angle;
    ret.m[9] = yz*one_minus_cos_angle - axis_.m[0]*sin_angle;
    ret.m[10] = cos_angle + axis_.m[2]*axis_.m[2]*one_minus_cos_angle;
    ret.m[11] = (T)0;
    ret.m[12] = (T)0;
    ret.m[13] = (T)0;
    ret.m[14] = (T)0;
    ret.m[15] = (T)1;
#endif
  }

  template <class T>
  void Mat4x4<T>::rotateMatBasis(Mat4x4& ret, const Vec3<T>& x, 
    const Vec3<T>& y, const Vec3<T>& z) {
#ifdef ROW_MAJOR
    ret.m[0] = x.m[0];
    ret.m[4] = x.m[1];
    ret.m[8] = x.m[2];
    ret.m[12] = 0;
    
    ret.m[1] = y.m[0];
    ret.m[5] = y.m[1];
    ret.m[9] = y.m[2];
    ret.m[13] = 0;
    
    ret.m[2] = z.m[0];
    ret.m[6] = z.m[1];
    ret.m[10] = z.m[2];
    ret.m[14] = 0;
    
    ret.m[3] = 0.0f;
    ret.m[7] = 0.0f;
    ret.m[11] = 0.0f;
    ret.m[15] = 1.0f;
#endif
#ifdef COLUMN_MAJOR
    ret.m[0] = x.m[0];
    ret.m[1] = x.m[1];
    ret.m[2] = x.m[2];
    ret.m[3] = 0;
    
    ret.m[4] = y.m[0];
    ret.m[5] = y.m[1];
    ret.m[6] = y.m[2];
    ret.m[7] = 0;
    
    ret.m[8] = z.m[0];
    ret.m[9] = z.m[1];
    ret.m[10] = z.m[2];
    ret.m[11] = 0;
    
    ret.m[12] = 0.0f;
    ret.m[13] = 0.0f;
    ret.m[14] = 0.0f;
    ret.m[15] = 1.0f;
#endif
  }
  
  
  template <class T>
  void Mat4x4<T>::glProjection(Mat4x4& a, const T znear, const T zfar, 
    const T fov_deg, const T screen_width, const T screen_height) {
    T aspect = screen_width / screen_height;
    
    // Calculate the projection matrix, from:
    // http://www.geeks3d.com/20090729/howto-perspective-projection-matrix-in-opengl/
    T xymax = znear * tanf(fov_deg * static_cast<T>(PI_OVER_360));
    T ymin = -xymax;
    T xmin = -xymax;
    
    T width = xymax - xmin;
    T height = xymax - ymin;
    
    T depth = zfar - znear;
    T q = -(zfar + znear) / depth;
    T qn = -2 * (zfar * znear) / depth;
    
    T w = 2 * znear / width;
    w = w / aspect;
    T h = 2 * znear / height;
    
    a.m[0]  = w;
    a.m[1]  = 0;
    a.m[2]  = 0;
    a.m[3]  = 0;
    
    a.m[4]  = 0;
    a.m[5]  = h;
    a.m[6]  = 0;
    a.m[7]  = 0;
    
    a.m[8]  = 0;
    a.m[9]  = 0;
    a.m[10] = q;
    a.m[11] = -1;
    
    a.m[12] = 0;
    a.m[13] = 0;
    a.m[14] = qn;
    a.m[15] = 0;
  };
  
  template <class T>
  void Mat4x4<T>::glOrthoProjection(Mat4x4& a, const T znear, const T zfar,
    const T left, const T right, const T bottom, const T top) {
    T tx = -(right + left) / (right - left);
    T ty = -(top + bottom) / (top - bottom);
    T tz = -(zfar + znear) / (zfar - znear);
    
    a.m[0] = (T) (2 / (right - left));
    a.m[1]  = 0;
    a.m[2]  = 0;
    a.m[3]  = 0;
    
    a.m[4]  = 0;
    a.m[5] = (T) (2 / (top - bottom));
    a.m[6]  = 0;
    a.m[7]  = 0;
    
    a.m[8]  = 0;
    a.m[9]  = 0;
    a.m[10] = (T) (-2 / (zfar - znear));
    a.m[11] = 0;
    
    a.m[12] = (T) tx;
    a.m[13] = (T) ty;
    a.m[14] = (T) tz;
    a.m[15] = (T) 1;
  };
  
  template <class T>
  void Mat4x4<T>::glLookAt(Mat4x4& view, const Vec3<T>& up, 
    const Vec3<T>& forward, const Vec3<T>& pos) {
    Vec3<T> side_;
    Vec3<T> up_(const_cast<Vec3<T>&>(up));
    Vec3<T> forward_(const_cast<Vec3<T>&>(forward));
    up_.normalize();  // Just in case
    forward_.normalize();  // Just in case
    
    // Create orthonormal basis
    Vec3<T>::cross(side_, forward_, up_);
    side_.normalize();
    Vec3<T>::cross(up_, side_, forward_);  // Cross again to ensure orthonorm
    up_.normalize();
    
    // View matrix is an inverse rotation matrix (therefore axis vectors are
    // the rows).  OpenGL is column major, so (0, 4, 8) is the first row.
    // Recal inverse of an orthonormal matrix is just the transpose.
#ifdef COLUMN_MAJOR
    view.m[0] = side_.m[0];
    view.m[4] = side_.m[1];
    view.m[8] = side_.m[2];
    view.m[12] = 0;
    
    view.m[1] = up_.m[0];
    view.m[5] = up_.m[1];
    view.m[9] = up_.m[2];
    view.m[13] = 0;
    
    view.m[2] = -forward_.m[0];
    view.m[6] = -forward_.m[1];
    view.m[10] = -forward_.m[2];
    view.m[14] = 0;
    
    view.m[3] = 0.0f;
    view.m[7] = 0.0f;
    view.m[11] = 0.0f;
    view.m[15] = 1.0f;
#endif
#ifdef ROW_MAJOR
    view.m[0] = side_.m[0];
    view.m[1] = side_.m[1];
    view.m[2] = side_.m[2];
    view.m[3] = 0;
    
    view.m[4] = up_.m[0];
    view.m[5] = up_.m[1];
    view.m[6] = up_.m[2];
    view.m[7] = 0;
    
    view.m[8] = -forward_.m[0];
    view.m[9] = -forward_.m[1];
    view.m[10] = -forward_.m[2];
    view.m[11] = 0;
    
    view.m[12] = 0.0f;
    view.m[13] = 0.0f;
    view.m[14] = 0.0f;
    view.m[15] = 1.0f;
#endif

    view.rightMultTranslation(-pos.m[0], -pos.m[1], -pos.m[2]);
  };
  
  template <class T>
  void Mat4x4<T>::affineInverse(Mat4x4& ret, const Mat4x4& a) {
    T det = a.m[0] * (a.m[10] * a.m[5] - a.m[9] * a.m[6])
    - a.m[4] * (a.m[10] * a.m[1] - a.m[9] * a.m[2])
    + a.m[8] * (a.m[6] * a.m[1] - a.m[5] * a.m[2]);
    T one_over_det = T(1) / det;
    // First calculate inverse of upper left
#ifdef ROW_MAJOR
    ret.m[0] = (a.m[10]*a.m[5]-a.m[9]*a.m[6]) * one_over_det;
    ret.m[1] = (-(a.m[10]*a.m[1]-a.m[9]*a.m[2])) * one_over_det;
    ret.m[2] = (a.m[6]*a.m[1]-a.m[5]*a.m[2]) * one_over_det;
    ret.m[4] = (-(a.m[10]*a.m[4]-a.m[8]*a.m[6])) * one_over_det;
    ret.m[5] = (a.m[10]*a.m[0]-a.m[8]*a.m[2]) * one_over_det;
    ret.m[6] = (-(a.m[6]*a.m[0]-a.m[4]*a.m[2])) * one_over_det;
    ret.m[8] = (a.m[9]*a.m[4]-a.m[8]*a.m[5]) * one_over_det;
    ret.m[9] = (-(a.m[9]*a.m[0]-a.m[8]*a.m[1])) * one_over_det;
    ret.m[10] = (a.m[5]*a.m[0]-a.m[4]*a.m[1]) * one_over_det;
#endif
#ifdef COLUMN_MAJOR
    ret.m[0] = (a.m[10]*a.m[5]-a.m[6]*a.m[9]) * one_over_det;
    ret.m[1] = (-(a.m[10]*a.m[1]-a.m[2]*a.m[9])) * one_over_det;
    ret.m[2] = (a.m[6]*a.m[1]-a.m[2]*a.m[5]) * one_over_det;
    ret.m[4] = (-(a.m[10]*a.m[4]-a.m[6]*a.m[8])) * one_over_det;
    ret.m[5] = (a.m[10]*a.m[0]-a.m[2]*a.m[8]) * one_over_det;
    ret.m[6] = (-(a.m[6]*a.m[0]-a.m[2]*a.m[4])) * one_over_det;
    ret.m[8] = (a.m[9]*a.m[4]-a.m[5]*a.m[8]) * one_over_det;
    ret.m[9] = (-(a.m[9]*a.m[0]-a.m[1]*a.m[8])) * one_over_det;
    ret.m[10] = (a.m[5]*a.m[0]-a.m[1]*a.m[4]) * one_over_det;
#endif
    // Now calculate the -M^inv * T
#ifdef ROW_MAJOR
    ret.m[3] = -(ret.m[0]*a.m[3] + ret.m[1]*a.m[7] + ret.m[2]*a.m[11]);
    ret.m[7] = -(ret.m[4]*a.m[3] + ret.m[5]*a.m[7] + ret.m[6]*a.m[11]);
    ret.m[11] = -(ret.m[8]*a.m[3] + ret.m[9]*a.m[7] + ret.m[10]*a.m[11]);
    ret.m[12] = (T)0;
    ret.m[13] = (T)0;
    ret.m[14] = (T)0;
#endif
#ifdef COLUMN_MAJOR
    ret.m[3] = (T)0;
    ret.m[7] = (T)0;
    ret.m[11] = (T)0;
    ret.m[12] = -(ret.m[0]*a.m[12] + ret.m[4]*a.m[13] +
                   ret.m[8]*a.m[14]);
    ret.m[13] = -(ret.m[1]*a.m[12] + ret.m[5]*a.m[13] +
                   ret.m[9]*a.m[14]);
    ret.m[14] = -(ret.m[2]*a.m[12] + ret.m[6]*a.m[13] +
                   ret.m[10]*a.m[14]);
#endif
    ret.m[15] = (T)1;
  };
  
  template <class T>
  void Mat4x4<T>::scaleMat(Mat4x4& ret, const T x, const T y, const T z) {
    ret.m[0] = x;
    ret.m[1] = (T)0;
    ret.m[2] = (T)0;
    ret.m[3] = (T)0;
    ret.m[4] = (T)0;
    ret.m[5] = y;
    ret.m[6] = (T)0;
    ret.m[7] = (T)0;
    ret.m[8] = (T)0;
    ret.m[9] = (T)0;
    ret.m[10] = z;
    ret.m[11] = (T)0;
    ret.m[12] = (T)0;
    ret.m[13] = (T)0;
    ret.m[14] = (T)0;
    ret.m[15] = (T)1;
  };

  template <class T>
  void Mat4x4<T>::scaleMat(Mat4x4& ret, const Vec3<T>& scale) {
    scaleMat(ret, scale[0], scale[1], scale[2]);
  };
  
  template <class T>
  void Mat4x4<T>::translationMat(Mat4x4& ret, const T x, const T y, 
    const T z) {
    ret.m[0] = (T)1;
    ret.m[1] = (T)0;
    ret.m[2] = (T)0;
    ret.m[4] = (T)0;
    ret.m[5] = (T)1;
    ret.m[6] = (T)0;
    ret.m[8] = (T)0;
    ret.m[9] = (T)0;
    ret.m[10] = (T)1;
#ifdef ROW_MAJOR
    ret.m[3] = x;
    ret.m[7] = y;
    ret.m[11] = z;
    ret.m[12] = (T)0;
    ret.m[13] = (T)0;
    ret.m[14] = (T)0;
#endif
#ifdef COLUMN_MAJOR
    ret.m[3] = (T)0;
    ret.m[7] = (T)0;
    ret.m[11] = (T)0;
    ret.m[12] = x;
    ret.m[13] = y;
    ret.m[14] = z;
#endif
    ret.m[15] = (T)1;
  };
  
  // Faster than affineInverse, but only for composite rotation + translation
  // matrices
  template <class T>
  void Mat4x4<T>::affineRotationTranslationInverse(Mat4x4& ret, 
    const Mat4x4& a) {
    // Inverse of the upper left rotation matrix is just it's transpose since it
    // is orthonormal
    ret.m[0] = a.m[0];
    ret.m[1] = a.m[4];
    ret.m[2] = a.m[8];
    ret.m[4] = a.m[1];
    ret.m[5] = a.m[5];
    ret.m[6] = a.m[9];
    ret.m[8] = a.m[2];
    ret.m[9] = a.m[6];
    ret.m[10] = a.m[10];
    // Now calculate the -M^inv * T
#ifdef ROW_MAJOR
    ret.m[3] = -(ret.m[0]*a.m[3] + ret.m[1]*a.m[7] + ret.m[2]*a.m[11]);
    ret.m[7] = -(ret.m[4]*a.m[3] + ret.m[5]*a.m[7] + ret.m[6]*a.m[11]);
    ret.m[11] = -(ret.m[8]*a.m[3] + ret.m[9]*a.m[7] + ret.m[10]*a.m[11]);
    ret.m[12] = (T)0;
    ret.m[13] = (T)0;
    ret.m[14] = (T)0;
#endif
#ifdef COLUMN_MAJOR
    ret.m[3] = (T)0;
    ret.m[7] = (T)0;
    ret.m[11] = (T)0;
    ret.m[12] = -(ret.m[0]*a.m[12] + ret.m[4]*a.m[13] +
                   ret.m[8]*a.m[14]);
    ret.m[13] = -(ret.m[1]*a.m[12] + ret.m[5]*a.m[13] +
                   ret.m[9]*a.m[14]);
    ret.m[14] = -(ret.m[2]*a.m[12] + ret.m[6]*a.m[13] +
                   ret.m[10]*a.m[14]);
#endif
    ret.m[15] = (T)1;
  };

  template <class T>
  void Mat4x4<T>::getTranslation(Vec3<T>& translation, const Mat4x4& a) {
#ifdef ROW_MAJOR
    translation.m[0] = a.m[3];
    translation.m[1] = a.m[7];
    translation.m[2] = a.m[11];
#endif
#ifdef COLUMN_MAJOR
    translation.m[0] = a.m[12];
    translation.m[1] = a.m[13];
    translation.m[2] = a.m[14];
#endif
  };
  
  // Faster than affineInverse, but only for composite rotation matrices
  template <class T>
  void Mat4x4<T>::affineRotationInverse(Mat4x4& ret, const Mat4x4& a) {
    // Inverse of the upper left rotation matrix is just it's transpose since it
    // is orthonormal
    ret.m[0] = a.m[0];
    ret.m[1] = a.m[4];
    ret.m[2] = a.m[8];
    ret.m[3] = (T)0;
    ret.m[4] = a.m[1];
    ret.m[5] = a.m[5];
    ret.m[6] = a.m[9];
    ret.m[7] = (T)0;
    ret.m[8] = a.m[2];
    ret.m[9] = a.m[6];
    ret.m[10] = a.m[10];
    ret.m[11] = (T)0;
    ret.m[12] = (T)0;
    ret.m[13] = (T)0;
    ret.m[14] = (T)0;
    ret.m[15] = (T)1;
  };
  
  // In math_base.h
  template <class T>
  void PolarDecomposition(const Mat4x4<T>& M, Mat4x4<T>& Q, Mat4x4<T>& S);

  template <class T>
  void Mat4x4<T>::extractRotation(Mat4x4& rotation, const Mat4x4& a) {
    // Note "a" must be affine and invertible!
    // Uses polar decomposition to find an approximate rotation matrix:
    // http://callumhay.blogspot.com/2010/10/decomposing-affine-transforms.html
    // Also see: "Matrix Animation and Polar Decomposition", by Ken Shoemake 
    // and Tom Duff
    Mat4x4<T> Q;
    Mat4x4<T> S;
    PolarDecomposition<T>(a, Q, S);

    // Rotation matrix is the Q matrix:
    rotation.set(Q);
  }

  template <class T>
  void Mat4x4<T>::decomposeRST(Mat4x4& rotation, Vec3<T>& scale, 
    Vec3<T>& translation, const Mat4x4& a) {
    AffineParts parts;
    HMatrix A;
    for (uint32_t r = 0; r < 4; r++) { 
      for (uint32_t c = 0; c < 4; c++) {
        A[r][c] = (float)(a(r,c));
      }
    }
    decomp_affine(A, &parts);
    translation.set(parts.t.x, parts.t.y, parts.t.z);
    scale.set(parts.k.x, parts.k.y, parts.k.z);
    for (uint32_t r = 0; r < 3; r++) { 
      for (uint32_t c = 0; c < 3; c++) {
        (rotation)(r, c) = parts.q_mat[r][c];
      }
    }
    rotation.m[3] = (T)0;
    rotation.m[7] = (T)0;
    rotation.m[11] = (T)0;
    rotation.m[12] = (T)0;
    rotation.m[13] = (T)0;
    rotation.m[14] = (T)0;
    rotation.m[15] = (T)1;
  }

  template <class T>
  void Mat4x4<T>::euler2RotMat(Mat4x4& a, const T x_angle, 
    const T y_angle, const T z_angle) {
    // http://www.euclideanspace.com/maths/geometry/rotations/conversions/eulerToMatrix/index.htm
    T c1 = cos(x_angle);  // Heading
    T s1 = sin(x_angle);
    T c2 = cos(y_angle);  // Attitude
    T s2 = sin(y_angle);
    T c3 = cos(z_angle);  // bank
    T s3 = sin(z_angle);
#ifdef COLUMN_MAJOR
    a.m[0] = c1*c2;
    a.m[4] = -c1*s2*c3 + s1*s3;
    a.m[8] = c1*s2*s3 + s1*c3;
    a.m[12] = (T)0;
    a.m[1] = s2;
    a.m[5] = c2*c3;
    a.m[9] = -c2*s3;
    a.m[13] = (T)0;
    a.m[2] = -s1*c2;
    a.m[6] = s1*s2*c3 + c1*s3;
    a.m[10] = -s1*s2*s3 + c1*c3;
    a.m[14] = (T)0;
    a.m[3] = (T)0;
    a.m[7] = (T)0;
    a.m[11] = (T)0;
    a.m[15] = (T)1;
#endif
#ifdef ROW_MAJOR
    a.m[0] = c1*c2;
    a.m[1] = -c1*s2*c3 + s1*s3;
    a.m[2] = c1*s2*s3 + s1*c3;
    a.m[3] = (T)0;
    a.m[4] = s2;
    a.m[5] = c2*c3;
    a.m[6] = -c2*s3;
    a.m[7] = (T)0;
    a.m[8] = -s1*c2;
    a.m[9] = s1*s2*c3 + c1*s3;
    a.m[10] = -s1*s2*s3 + c1*c3;
    a.m[11] = (T)0;
    a.m[12] = (T)0;
    a.m[13] = (T)0;
    a.m[14] = (T)0;
    a.m[15] = (T)1;
#endif
  }

  template <class T>
  void Mat4x4<T>::rotMat2Euler(T& x_angle, T& y_angle, T& z_angle, 
    const Mat4x4& a) {
    // http://www.euclideanspace.com/maths/geometry/rotations/conversions/matrixToEuler/index.htm
#ifdef COLUMN_MAJOR
    // Assuming the angles are in radians.
	  if (a.m[1] > (T)0.998) { // singularity at north pole
		  x_angle = atan2(a.m[8],a.m[10]);
		  y_angle = (T)M_PI_2;
		  z_angle = 0;
		  return;
	  }
	  if (a.m[1] < (T)-0.998) { // singularity at south pole
		  x_angle = atan2(a.m[8],a.m[10]);
		  y_angle = -(T)M_PI_2;
		  z_angle = 0;
		  return;
	  }
	  x_angle = atan2(-a.m[2],a.m[0]);
	  z_angle = atan2(-a.m[9],a.m[5]);
	  y_angle = asin(a.m[1]);
#endif
#ifdef ROW_MAJOR
    // Assuming the angles are in radians.
    if (a.m[4] > (T)0.998) { // singularity at north pole
      x_angle = atan2(a.m[2],a.m[10]);
      y_angle = (T)M_PI_2;
      z_angle = 0;
      return;
    }
    if (m.m10 < (T)-0.998) { // singularity at south pole
      heading = atan2(a.m[2],a.m[10]);
      y_angle = -(T)M_PI_2;
      z_angle = 0;
      return;
    }
    x_angle = atan2(-a.m[8],a.m[0]);
    z_angle = atan2(-a.m[6],a.m[5]);
    y_angle = asin(a.m[4]);
#endif
  }

  template <class T>
  T Mat4x4<T>::frobeniusNorm(const Mat4x4<T>& a) {
    T accum = 0;
    for (uint32_t i = 0; i < 16; i++) {
      accum += a.m[i] * a.m[i];
    }
    return sqrt(accum);
  }

  template <class T>
  T Mat4x4<T>::frobeniusNorm() {
    return Mat4x4<T>::frobeniusNorm(*this);
  }
  
};  // namespace math
};  // namespace jtil
