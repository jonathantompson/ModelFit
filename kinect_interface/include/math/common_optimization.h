//
//  common_optimization.h
//
//  Created by Jonathan Tompson on 8/23/12.
//
//  Just some common structures used for non-linear optimiation algorithms
//

#pragma once

#include "math/math_types.h"
#include "data_str/vector.h"

// TODO: Move these to math_base (not common_optimization)
#if defined(WIN32) || defined(_WIN32)
  #define UNIFORM_INT_DISTRIBUTION std::tr1::uniform_int_distribution<int>
  #define UNIFORM_REAL_DISTRIBUTION std::tr1::uniform_real_distribution<float>
  #define MERSINE_TWISTER_ENG std::tr1::mt19937
  #define NORMAL_REAL_DISTRIBUTION std::tr1::normal_distribution<float>
#else
  #define UNIFORM_INT_DISTRIBUTION std::uniform_int_distribution<int>
  #define UNIFORM_REAL_DISTRIBUTION std::uniform_real_distribution<float>
  #define MERSINE_TWISTER_ENG std::mt19937
  #define NORMAL_REAL_DISTRIBUTION std::normal_distribution<float>
#endif

namespace jtil {
namespace math {

  typedef void (*CoeffUpdateFuncPtr)(float* coeff);
  typedef void (*ObjectFuncParallelPtr)(jtil::data_str::Vector<float>& residues, 
      jtil::data_str::Vector<float*>& coeffs);
  typedef float (*ObjectiveFuncPtr)(const float* coeff);
  typedef void (*JacobianFuncPtr)(float* jacob, const float* coeff);


  // For non-linear least squares fitting routines (such as LMFit)
  typedef float (*FitFuncPtr)(const float* x, const float* c);
  typedef void (*FitJacobFuncPtr)(float* jacob, const float* x, const float* c);
  typedef float (*ResidueFuncPtr)(const float* y, const float* y_fit);

  typedef enum {
    ARMIJO,  // Less conservative, BFGS performance is not gaurenteed!
    STRONG_WOLFE,  // Expensive but more conservative (includes ARMIJO)
  } SufficientDescentCondition;
  
  struct OptNode {
    float* coeff;
    float residue;
    OptNode& operator=(const OptNode &rhs) {
    if (this != &rhs) {
      this->coeff = rhs.coeff;
      this->residue = rhs.residue;
    }
    return *this;
  }
  };

  struct SwarmNode {
  public:
    SwarmNode();
    ~SwarmNode();
    void resize(const uint32_t size);

    float* vel;  // num_coeffs
    float* pos;  // num_coeffs
    float* best_pos;  // num_coeffs
    float residue;
    float best_residue;
  };

};  // namespace math
};  // namespace jtil
