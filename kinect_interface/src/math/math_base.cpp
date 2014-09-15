#include <stdlib.h>
#include <cmath>
#include <cstddef>
#include "math/math_base.h"

#define rand_r rand

namespace jtil {
namespace math {
  bool seed = false;

  // next PO2 will return the next higher power of 2, if val is a power of 2
  // already, then it will return val.
  int32_t nextPO2(int32_t val) {
    if (val == 0) {
      throw std::runtime_error("invalid input");
    }
    val--;
    val = (val >> 1) | val;
    val = (val >> 2) | val;
    val = (val >> 4) | val;
    val = (val >> 8) | val;
    val = (val >> 16) | val;
    val++;
    return val;
  }

  float CalcGaussianNoise(float mean, float stdDev) {
    if (seed == false) {
      srand(0);
      seed = true;
    }
#ifdef _WIN32
    float u1 = static_cast<float>(rand_r()) / 
      (static_cast<float>(RAND_MAX) + 1.0f);
    float u2 = static_cast<float>(rand_r()) / 
      (static_cast<float>(RAND_MAX) + 1.0f);
    float randStdNormal = std::sqrtf(-2.0f * std::logf(u1)) * 
      std::sinf(2.0f * static_cast<float>(M_PI) * u2);
    float randNormal = mean + stdDev * randStdNormal;
    return randNormal;
#else
    float u1 = static_cast<float>(rand_r()) / 
               (static_cast<float>(RAND_MAX) + 1.0f);
    float u2 = static_cast<float>(rand_r()) / 
               (static_cast<float>(RAND_MAX) + 1.0f);
    float randStdNormal = sqrtf(-2.0f * logf(u1)) * 
                          sinf(2.0f * static_cast<float>(M_PI) * u2);
    float randNormal = mean + stdDev * randStdNormal;
    return randNormal;
#endif
  }

  void calcOpenGLAffine(float* ret, Float3* axes, float* trans) {
    ret[0]  = axes[0][0];
    ret[1]  = axes[0][1];
    ret[2]  = axes[0][2];
    ret[3]  = 0.0f;
    ret[4]  = axes[1][0];
    ret[5]  = axes[1][1];
    ret[6]  = axes[1][2];
    ret[7]  = 0.0f;
    ret[8]  = axes[2][0];
    ret[9]  = axes[2][1];
    ret[10] = axes[2][2];
    ret[11] = 0.0f;
    ret[12] = trans[0];
    ret[13] = trans[1];
    ret[14] = trans[2];
    ret[15] = 1.0f;
  }

  void calcOpenGLAffine(float* ret, Float3* axes, Float3* trans) {
    calcOpenGLAffine(ret, axes, trans->m);
  }

  double Round(double a, double precision) {
    return std::floor(0.5+a/precision)*precision;
  }

  double Interpolate(const double &f0, const double &f1, double alpha) {
    return f0*(1-alpha) + f1*alpha;
  }

  inline float fabs_manual1(float x) {
    int y = (int&)x & 0x7FFFFFFF;
    return (float&)y;
  }

  inline float fabs_manual2(float g) {
    unsigned int *gg;
    gg=(unsigned int*)&g;
    *(gg)&=2147483647u;
    return g;
  }

  // theta is angle from top [0, pi], phi is angle along slice [0, 2pi]
  // http://en.wikipedia.org/wiki/Spherical_coordinate_system
  void SphericalToCartesean(Float3* xyz, float r, float phi, float theta ) {
    xyz->m[0] = r * sinf(theta) * cosf(phi);
    xyz->m[1] = r * sinf(theta) * sinf(phi);
    xyz->m[2] = r * cosf(theta);
  }

  // Brute force primality test, from: 
  // http://stackoverflow.com/questions/4475996/given-prime-number-n-compute-the-next-prime
  // Answer 7 by Howard Hinnant --> Implementation 3
  bool IsPrime(std::size_t x) {
    if (x != 2 && x % 2 == 0) {
      return false;
    }
    for (std::size_t i = 3; true; i += 2) {
      std::size_t q = x / i;
      if (q < i) {
        return true;
      }
      if (x == q * i) {
        return false;
      }
    }
    return true;
  }

  // Brute force next prime integer (reasonably quick I think)
  // http://stackoverflow.com/questions/4475996/given-prime-number-n-compute-the-next-prime
  // Answer 7 by Howard Hinnant --> Implementation 4
  std::size_t NextPrime(std::size_t x) {
    if (x <= 2)
      return 2;
    if (!(x & 1))
      ++x;
    for (; !IsPrime(x); x += 2)
      ;
    return x;
  }

  // Forward declare some of these to force compilation
  template void PolarDecomposition<float>(const Mat4x4<float>& M, 
    Mat4x4<float>& Q, Mat4x4<float>& S);
  template void PolarDecomposition<double>(const Mat4x4<double>& M, 
    Mat4x4<double>& Q, Mat4x4<double>& S);

  float Mod(float x, float y) {
    if (0 == y) {
      return x;
    }

    return x - y * floor(x / y);
  }


  void WrapTwo2PI(float& angle) {
    angle = Mod(angle, static_cast<float>(2.0 * M_PI));
  }


  void WrapTwoPI(float& angle) {
    angle = Mod(angle + static_cast<float>(M_PI), 
      static_cast<float>(2.0 * M_PI)) - static_cast<float>(M_PI);
  }

  double Mod(double x, double y) {
    if (0 == y) {
      return x;
    }

    return x - y * floor(x / y);
  }


  void WrapTwo2PI(double& angle) {
    angle = Mod(angle, 2.0 * M_PI);
  }


  void WrapTwoPI(double& angle) {
    angle = Mod(angle + M_PI, 2.0 * M_PI) - M_PI;
  }

  void Convolve(const float* input, const float* kernel, float* output,
    const int32_t in_width, const int32_t in_height,
    const int32_t out_width, const int32_t out_height, 
    const int32_t kernel_size, const int32_t n_threads) {
    // Check the sizes (in case the user messed up)
    if (out_width != in_width - kernel_size + 1 ||
      out_height != in_height - kernel_size + 1) {
      throw std::runtime_error("jtil::math::Convolve() - ERROR: "
        "Input/Output size mismatch!");
    }
    // From:
    // http://developer.amd.com/resources/heterogeneous-computing/opencl-zone/programming-in-opencl/image-convolution-using-opencl/
    #pragma omp parallel for num_threads(n_threads)
    for (int32_t yOut = 0; yOut < out_height; yOut++) {
      const int32_t yInTopLeft = yOut;
      for (int32_t xOut = 0; xOut < out_width; xOut++) {
        const int32_t xInTopLeft = xOut;
        float sum = 0;
        for (int32_t r = 0; r < kernel_size; r++) {
          const int32_t idxFtmp = r * kernel_size;
          const int32_t yIn = yInTopLeft + r;
          const int32_t idxIntmp = yIn * in_width + xInTopLeft;
          for (int32_t c = 0; c < kernel_size; c++) {
            const int32_t idxF  = idxFtmp  + c;
            const int32_t idxIn = idxIntmp + c;    
            sum += kernel[idxF] * input[idxIn];
          }
        } //for (int r = 0...

        const int idxOut = yOut * out_width + xOut;
        output[idxOut] = sum;

      } //for (int xOut = 0...
    } //for (int yOut = 0...
  }

}  // namespace math
}  // namespace jtil
