//
//  math_base.h
//
//  Created by Jonathan Tompson on 3/14/12.
//
//  This is where all the miscilaneous math functions go.  It's a little messy.
//

#pragma once

#ifdef __APPLE__
  #include <algorithm>
#endif
#include <stdexcept>
#include <string>
#include "math/math_types.h"

namespace jtil {
namespace math {
  
  std::size_t NextPrime(std::size_t x);  // Return the next prime number:
  bool IsPrime(std::size_t x);  // Brute-force primality test

  // Return a random value with gaussian PDF
  // Alternatively use "RAND_ENGINE eng; NORM_DIST<float> norm_dist;"
  float CalcGaussianNoise(float mean, float stdDev);

  // Eigen decomposition of a real symmetric nxn matrix
  // --> source is in eig3.cpp
  // --> Code uses QL method. It is based on the NETLIB algol/fortran procedure 
  // --> tql1 by Bowdler, Martin, Reinsch and Wilkinson.
  // http://beige.ucs.indiana.edu/B673/node30.html
  // To find eigenvalues QL method is O(20N^2)
  // To find both eigen vectors and values, method becomes O(N^3)
  // Use doubles if A is illconditioned!!
  template <class T> void SymEigen(T* A, T* vecs, T* vals, unsigned int n);
  template <class T> void SymMatEigenVecs(Vec3<T>* eigVecs, Vec3<T>* eigVals, 
    Mat3x3<T>* A);
  template <class T> void SymMatEigenVals(Vec3<T>* eigVals, Mat3x3<T>* A);  

  // Statistics of 3D points clouds
  template <class T> void Mean3DPoints(Vec3<T>* com, T* pts, unsigned int num);
  template <class T> void Cov3DPoints(Mat3x3<T>* cov, Vec3<T>* com, T* pts,
    unsigned int num);
  template <class T> void PCA3DPoints(Vec3<T>* axes, Mat3x3<T>* cov, 
    Vec3<T>* com, Vec3<T>* eigVals, T* pts, unsigned int num);

  // Misc. math functions
  void calcOpenGLAffine(float* ret, Float3* axes, Float3* trans);
  void calcOpenGLAffine(float* ret, Float3* axes, float* trans);
  double Round(double a, double precision);
  double Interpolate(const double &f0, const double &f1, double alpha);
  inline float fabs_manual1(float x);  // These might be faster... profile!
  inline float fabs_manual2(float g);
  void SphericalToCartesean(Float3* xyz, float r, float phi, float theta );
  void WrapTwo2PI(float& angle);  // wrap [rad] angle to [0...2PI)
  void WrapTwo2PI(double& angle);
  void WrapTwoPI(float& angle);  // wrap [rad] angle to [-PI...PI)
  void WrapTwoPI(double& angle);
  // Mod() - similar to matlab's mod() --> result is always possitive
  float Mod(float x, float y);
  double Mod(double x, double y);

  template <class T>
  T round(const T num) {
    return (num > (T)0.0) ? floor(num + (T)0.5) : ceil(num - (T)0.5);
  }

  // Convolve - Basic 2D convolution
  // Kernel is NxN (square)
  // Output dimensions are: 
  //   out_width = in_width - kernel_size + 1
  //   out_height = in_height - kernel_size + 1
  void Convolve(const float* input, const float* kernel, float* output,
    const int32_t in_width, const int32_t in_height,
    const int32_t out_width, const int32_t out_height, 
    const int32_t kernel_size, const int32_t n_threads);

  // next PO2 will return the next higher power of 2, if val is a power of 2
  // already, then it will return val.
  int32_t nextPO2(int32_t val);

  // **************************************************************************
  // "export" keyword not yet supported by VS 10 so template functions have to 
  // go here:
  
  /* Eigen decomposition code for symmetric 3x3 matrices, copied from public
   domain Java Matrix library JAMA. */
  
#define MAX_VAL(a, b) ((a)>(b)?(a):(b))
  
  template <class T>
  static T hypot2(T x, T y) {
    return std::sqrt(x*x+y*y);
  };
  
  // Symmetric Householder reduction to tridiagonal form.
  // Origionally double V[][] notation, this was converted to T *
  // using row-major storage.  Ie for n=3:
  //          | m_(0,0) m_(0,1) m_(0,2) |   | 0 1 2 |
  //   Mat3 = | m_(1,0) m_(1,1) m_(1,2) | = | 3 4 5 | -> m[i][j] = m[i*n+j]
  //          | m_(2,0) m_(2,1) m_(2,2) |   | 6 7 8 |
  template <class T>
  static void tred2(T* V, T* d, T* e, int n) {
    //  This is derived from the Algol procedures tred2 by
    //  Bowdler, Martin, Reinsch, and Wilkinson, Handbook for
    //  Auto. Comp., Vol.ii-Linear Algebra, and the corresponding
    //  Fortran subroutine in EISPACK.
    for (int j = 0; j < n; j++) {
      d[j] = V[(n-1)*n+j];
    }
    
    // Householder reduction to tridiagonal form.
    
    for (int i = n-1; i > 0; i--) {
      // Scale to avoid under/overflow.
      T scale = 0.0;
      T h = 0.0;
      for (int k = 0; k < i; k++) {
        scale = scale + std::fabs(d[k]);
      }
      if (scale == 0.0) {
        e[i] = d[i-1];
        for (int j = 0; j < i; j++) {
          d[j] = V[(i-1)*n+j]; 
          V[i*n+j] = 0.0;
          V[j*n+i] = 0.0; 
        }
      } else {
        // Generate Householder vector.
        for (int k = 0; k < i; k++) {
          d[k] /= scale;
          h += d[k] * d[k];
        }
        T f = d[i-1];
        T g = std::sqrt(h);
        if (f > 0) {
          g = -g;
        }
        e[i] = scale * g;
        h = h - f * g;
        d[i-1] = f - g;
        for (int j = 0; j < i; j++) {
          e[j] = 0.0;
        }
        
        // Apply similarity transformation to remaining columns.
        
        for (int j = 0; j < i; j++) {
          f = d[j];
          V[j*n+i] = f;  
          g = e[j] + V[j*n+j] * f;  
          for (int k = j+1; k <= i-1; k++) {
            g += V[k*n+j] * d[k];  
            e[k] += V[k*n+j] * f;  
          }
          e[j] = g;
        }
        f = 0.0;
        for (int j = 0; j < i; j++) {
          e[j] /= h;
          f += e[j] * d[j];
        }
        T hh = f / (h + h);
        for (int j = 0; j < i; j++) {
          e[j] -= hh * d[j];
        }
        for (int j = 0; j < i; j++) {
          f = d[j];
          g = e[j];
          for (int k = j; k <= i-1; k++) {
            V[k*n+j] -= (f * e[k] + g * d[k]); 
          }
          d[j] = V[(i-1)*n+j]; 
          V[i*n+j] = 0.0;
        }
      }
      d[i] = h;
    }
    
    // Accumulate transformations.
    
    for (int i = 0; i < n-1; i++) {
      V[(n-1)*n+i] = V[i*n+i]; 
      V[i*n+i] = 1.0;
      T h = d[i+1];
      if (h != 0.0) {
        for (int k = 0; k <= i; k++) {
          d[k] = V[k*n+i+1] / h; 
        }
        for (int j = 0; j <= i; j++) {
          T g = 0.0;
          for (int k = 0; k <= i; k++) {
            g += V[k*n+i+1] * V[k*n+j];  
          }
          for (int k = 0; k <= i; k++) {
            V[k*n+j] -= g * d[k]; 
          }
        }
      }
      for (int k = 0; k <= i; k++) {
        V[k*n+i+1] = 0.0; 
      }
    }
    for (int j = 0; j < n; j++) {
      d[j] = V[(n-1)*n+j];  
      V[(n-1)*n+j] = 0.0;
    }
    V[(n-1)*n+n-1] = 1.0; 
    e[0] = 0.0;
  };
  
  // Symmetric tridiagonal QL algorithm.
  template <class T>
  static void tql2(T* V, T* d, T* e, int n) {
    //  This is derived from the Algol procedures tql2, by
    //  Bowdler, Martin, Reinsch, and Wilkinson, Handbook for
    //  Auto. Comp., Vol.ii-Linear Algebra, and the corresponding
    //  Fortran subroutine in EISPACK.
    
    for (int i = 1; i < n; i++) {
      e[i-1] = e[i];
    }
    e[n-1] = 0.0;
    
    T f = 0.0;
    T tst1 = 0.0;
    T eps = pow(2.0, -52.0);
    for (int l = 0; l < n; l++) {
      // Find small subdiagonal element
      tst1 = MAX_VAL(tst1, std::fabs(d[l]) + std::fabs(e[l]));
      int m = l;
      while (m < n) {
        if (std::fabs(e[m]) <= eps*tst1) {
          break;
        }
        m++;
      }
      
      // If m == l, d[l] is an eigenvalue,
      // otherwise, iterate.
      if (m > l) {
        int iter = 0;
        do {
          iter = iter + 1;  // (Could check iteration count here.)
          
          // Compute implicit shift
          
          T g = d[l];
          T p = (d[l+1] - g) / (2.0 * e[l]);
          T r = hypot2(p, (T)1.0);
          if (p < 0) {
            r = -r;
          }
          d[l] = e[l] / (p + r);
          d[l+1] = e[l] * (p + r);
          T dl1 = d[l+1];
          T h = g - d[l];
          for (int i = l+2; i < n; i++) {
            d[i] -= h;
          }
          f = f + h;
          
          // Implicit QL transformation.
          
          p = d[m];
          T c = 1.0;
          T c2 = c;
          T c3 = c;
          T el1 = e[l+1];
          T s = 0.0;
          T s2 = 0.0;
          for (int i = m-1; i >= l; i--) {
            c3 = c2;
            c2 = c;
            s2 = s;
            g = c * e[i];
            h = c * p;
            r = hypot2(p, e[i]);
            e[i+1] = s * r;
            s = e[i] / r;
            c = p / r;
            p = c * d[i] - s * g;
            d[i+1] = h + s * (c * g + s * d[i]);
            
            // Accumulate transformation.
            
            for (int k = 0; k < n; k++) {
              h = V[k*n+i+1]; 
              V[k*n+i+1] = s * V[k*n+i] + c * h;
              V[k*n+i] = c * V[k*n+i] - s * h;
            }
          }
          p = -s * s2 * c3 * el1 * e[l] / dl1;
          e[l] = s * p;
          d[l] = c * p;
          
          // Check for convergence.
        } while (std::fabs(e[l]) > eps*tst1);
      }
      d[l] = d[l] + f;
      e[l] = 0.0;
    }
    
    // Sort eigenvalues and corresponding vectors.
    
    for (int i = 0; i < n-1; i++) {
      int k = i;
      T p = d[i];
      for (int j = i+1; j < n; j++) {
        if (d[j] < p) {
          k = j;
          p = d[j];
        }
      }
      if (k != i) {
        d[k] = d[i];
        d[i] = p;
        for (int j = 0; j < n; j++) {
          p = V[j*n+i];  
          V[j*n+i] = V[j*n+k];  
          V[j*n+k] = p; 
        }
      }
    }
  };
  
  template <class T>
  void SymEigen(T* A, T* vecs, T* vals, int n) {
    T* e = new T[n];
    for (int i = 0; i < n; i++) {
      for (int j = 0; j < n; j++) {
        vecs[i*n+j] = A[i*n+j];
      }
    }
    tred2(vecs, vals, e, n);
    tql2(vecs, vals, e, n);
    delete[] e;
  };

  template <class T>
  void SymMatEigenVecs(Vec3<T>* eigVecs, Vec3<T>* eigVals, Mat3x3<T>* A) {
    T outVec[3*3];

    math::SymEigen(A->m, outVec, eigVals->m, 3);

    // The eigen vectors are colonm-wise in the return vector --> Verified with
    // matlab.  Transforming <1,0,0> with the return matrix gives us the column
    // of the origional matrix (which suggests I am correct)

    eigVecs[0].m[0] = outVec[0]; 
    eigVecs[0].m[1] = outVec[3]; 
    eigVecs[0].m[2] = outVec[6];

    eigVecs[1].m[0] = outVec[1]; 
    eigVecs[1].m[1] = outVec[4]; 
    eigVecs[1].m[2] = outVec[7];

    eigVecs[2].m[0] = outVec[2]; 
    eigVecs[2].m[1] = outVec[5]; 
    eigVecs[2].m[2] = outVec[8];
  };

  template <class T>
  void SymMatEigenVals(Vec3<T>* eigVals, Mat3x3<T>* A) {
    // Copy to temp array.  Use double if A is illconditioned.
    Mat3x3<T> inA;

    inA.m[0] = A->m[0]; 
    inA.m[1] = A->m[1]; 
    inA.m[2] = A->m[2];
    inA.m[3] = A->m[3]; 
    inA.m[4] = A->m[4]; 
    inA.m[5] = A->m[5];
    inA.m[6] = A->m[6]; 
    inA.m[7] = A->m[7];
    inA.m[8] = A->m[8];

    // Closed form solution http://dl.acm.org/citation.cfm?id=366316
    T m = (T)(inA.m[0]+inA.m[4]+inA.m[8])/(T)3.0;  // m = trace(M)/3;
    inA.m[0] -= m;  // K = inA-m*eye(3);
    inA.m[4] -= m;
    inA.m[8] -= m; 
    T q = Mat3x3<T>::det(inA)/(T)2.0;

    // Calculate the sum of squares of the elements
    T p = 0;
    for (unsigned int i = 0; i < 3; i ++) {
      for (unsigned int j = 0; j < 3; j ++) {
        p = p + (inA.m[i*3+j]*inA.m[i*3+j]);  // p = p + K(i,j)^2;
      }
    }
    p = p/(T)6.0;

    // phi as written in paper
    T p_3 = p*p*p;
    T q_2 = q*q;
    T phi = (1.0/3.0)*std::atan(std::sqrt(p_3 - q_2)/q);
    if (q_2 >= p_3) {
      phi = 0.0;
    }

    if (phi < 0.0) {
      phi = phi+M_PI / 3.0;
    }

    eigVals->set((T)(m + 2.0*sqrt(p)*std::cos(phi)),
      (T)(m - std::sqrt(p)*(std::cos(phi) + std::sqrt(3.0)*std::sin(phi))),
      (T)(m - std::sqrt(p)*(std::cos(phi) - std::sqrt(3.0)*std::sin(phi))));
  };

  template <class T>
  void Mean3DPoints(Vec3<T>* com, T* pts, unsigned int num) {
    com->zeros();
    T* com_m = com->m;  // Get array to avoid function-call overhead
    for (unsigned int i = 0; i < num; i ++) {
      unsigned int ind = i*3;
      com_m[0] += pts[ind];
      com_m[1] += pts[ind + 1];
      com_m[2] += pts[ind + 2];
    }
    com->scale((T)1.0/(T)num);
  };

  // http://www.mechcore.net/files/docs/alg/gottschalk00collision.pdf
  // Page 48
  // Note Matricies are all row-major:
  //            | m(0,0) m(0,1) m(0,2) |   | 0 1 2 |
  //   Mat3x3 = | m(1,0) m(1,1) m(1,2) | = | 3 4 5 |
  //            | m(2,0) m(2,1) m(2,2) |   | 6 7 8 |
  template <class T>
  void Cov3DPoints(Mat3x3<T>* cov, Vec3<T>* com, T* pts, unsigned int num) {
    Mean3DPoints(com, pts, num);  // Calculate the center of mass (geom. mean)
    cov->zeros();
    T* ret_m = cov->m;  // Get array to avoid function-call overhead
    T* com_m = com->m; 
    // Perform the accumulation (only do it for upper values, since ret is sym)
    for (unsigned int i = 0; i < num; i ++) {
      unsigned int x = i*3;
      unsigned int y = x+1;
      unsigned int z = x+2;
      ret_m[0] += pts[x]*pts[x];  // m[0][0] += pi_x * pi_x
      ret_m[1] += pts[x]*pts[y];  // m[0][1] += pi_x * pi_y
      ret_m[2] += pts[x]*pts[z];  // m[0][2] += pi_x * pi_z
      ret_m[4] += pts[y]*pts[y];  // m[1][1] += pi_y * pi_y
      ret_m[5] += pts[y]*pts[z];  // m[1][2] += pi_y * pi_z
      ret_m[8] += pts[z]*pts[z];  // m[1][2] += pi_y * pi_z
    }
    // Now finish calculating Upper Right values
    T one_over_num = (T)1.0 / (T)num;
    ret_m[0] = (ret_m[0] * one_over_num) - (com_m[0] * com_m[0]);
    ret_m[1] = (ret_m[1] * one_over_num) - (com_m[0] * com_m[1]);
    ret_m[2] = (ret_m[2] * one_over_num) - (com_m[0] * com_m[2]);
    ret_m[4] = (ret_m[4] * one_over_num) - (com_m[1] * com_m[1]);
    ret_m[5] = (ret_m[5] * one_over_num) - (com_m[1] * com_m[2]);
    ret_m[8] = (ret_m[8] * one_over_num) - (com_m[2] * com_m[2]);

    // Now set the lower left values
    ret_m[3] = ret_m[1];
    ret_m[6] = ret_m[2];
    ret_m[7] = ret_m[5];
  };

  template <class T> 
  void PCA3DPoints(Vec3<T>* axes, Mat3x3<T>* cov, Vec3<T>* com, 
    Vec3<T>* eigVals, T* pts, unsigned int num) {
    // First calculate the cov matrix
    Cov3DPoints(cov, com, pts, num);  // O(6*num)

    // Now calculate eigen vectors of the sym cov matrix
    SymMatEigenVecs(axes, eigVals, cov);  // O(2n^3)
    axes[0].normalize();
    axes[1].normalize();
    //    axes[2].normalize();
    Vec3<T>::cross(&axes[2], &axes[0], &axes[1]);  // Make sure it's RH coord
  };

  // M = Q * S, where Q is unitary (orthonormal) and R is positive semi-definite
  template <class T>
  void PolarDecomposition(const Mat4x4<T>& M, Mat4x4<T>& Q, Mat4x4<T>& S) {
    // Copy the matrix first - we'll use this to break down each component
    Mat4x4<T> M_copy(M);

    // Zero the translation components
    M_copy.m[3] = (T)0;
    M_copy.m[7] = (T)0;
    M_copy.m[11] = (T)0;
    M_copy.m[12] = (T)0;
    M_copy.m[13] = (T)0;
    M_copy.m[14] = (T)0;
    M_copy.m[15] = (T)1;

    // Extract the Q component - this is done using polar decompostion, 
    // where we successively average the matrix with its inverse transpose 
    // until there is a very small difference between successive averages
    T norm;
    uint32_t count = 0;
    Q.set(M_copy);
    Mat4x4<T> nextQ;
    Mat4x4<T> currInvTranspose;
    Mat4x4<T> currTranspose;
    do {
      Mat4x4<T>::transpose(currTranspose, Q);
      Mat4x4<T>::inverse(currInvTranspose, currTranspose);

      // Go through every component in the matrices and find the next matrix
      for (uint32_t i = 0; i < 16; i++) {
        nextQ[i] = (T)0.5 * (Q.m[i] + currInvTranspose.m[i]);
      }

      norm = (T)0;
      for (int i = 0; i < 3; i++) {
        T n = static_cast<T>(
          fabs(Q(i, 0) - nextQ(i, 0)) +
          fabs(Q(i, 1) - nextQ(i, 1)) +
          fabs(Q(i, 2) - nextQ(i, 2)));
        norm = std::max<T>(norm, n);
      }
      Q.set(nextQ);
      count++;
    } while (count < 100 && norm > EPSILON);

    if (norm > EPSILON) {
      printf("PolarDecomposition() - WARNING: norm is high");
    }

    Mat4x4<T> QInverse;
    Mat4x4<T>::transpose(QInverse, Q);

#if defined(DEBUG) || defined(_DEBUG)
    // Check that Q is orthonormal
    Mat4x4<T> QQInverse;
    Mat4x4<T>::mult(QQInverse, QInverse, Q);
    bool orthonormal = true;
    for (uint32_t r = 0; r < 4; r++) {
      for (uint32_t c = 0; c < 4; c++) {
        if ((r == c && fabs(QQInverse(r, c) - (T)1) > LOOSE_EPSILON) ||
            (r != c && fabs(QQInverse(r, c)) > LOOSE_EPSILON)) {
          orthonormal = false;
        }
      }
    }
    if (!orthonormal) {
      throw std::runtime_error(std::string("PolarDecomposition() - ERROR: ") +
        std::string("Q is not orthonormal."));
    }
#endif

    Mat4x4<T>::mult(S, QInverse, M);
  };

};  // namespace math
};  // namespace jtil
