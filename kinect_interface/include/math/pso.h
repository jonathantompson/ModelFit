//
//  pso.h
//
//  Created by Jonathan Tompson on 8/23/12.
//
//  A black box implementation of the Particle-swarm non-linear optimization
//  algorithm.  Does not require Hessian or Jacobian evaluations.
//
//  This is an implementation of the algorithm discribed in "An Off-The-Shelf
//  PSO" (with details filled out from the wikipedia article)
//
//  This class minimizes f(c) by modifying c.  It is unconstrained optimiation.
//

#pragma once

#include <random>
#include "math/math_types.h"
#include "math/common_optimization.h"

#if defined(WIN32) || defined(_WIN32)
  #define constexpr 
#endif

// #define PSO_VERBOSE_SOLVER  // Print out per-iteration information

namespace jtil {
namespace math {

  // Particle Swarm Optimiation
  class PSO {
  public:
    // Set population size (num_agents) to -1 to let the optimizer choose a
    // population size for you.
    PSO(const uint32_t num_coeffs, const int swarm_size_ = -1);
    ~PSO();

    // minimize():
    // - end_c: The output value.
    // - start_c: The starting inital guess and the center of the intial 
    //   search radius.  Should be chosen very carefully to ensure convergence.
    // - radius_c: The initial search radius.  Also, should be chosen
    //   very carefully to ensure proper convergence.
    // - angle_coeff: Since the mid-point of two angles needs to be calc'ed
    //   differently, the user can supply an array of boolians indicating that 
    //   the i-th coefficient is an angle.
    // - obj_func: Function to minimize.
    // - coeff_norm_func: After coefficient update, some coefficients may need
    //   normalization.  Set to NULL if not needed.
    void minimize(float* end_c,
                  const float* start_c,
                  const float* radius_c,
                  const bool* angle_coeff,  // can be NULL
                  const ObjectiveFuncPtr obj_func,
                  const CoeffUpdateFuncPtr coeff_update_func);  // can be NULL

    // Termination and Optimization settings:
    float delta_coeff_termination;  // The spread of the agent coefficients
    uint64_t max_iterations;
    float phi_p;
    float phi_g;
    bool verbose;  // Set to true for detailed output
  
  private:
    uint32_t num_coeffs_;
    uint32_t swarm_size_;
    float* c_lo_;  // lower bound of search space
    float* c_hi_;  // upper bound of search space
    float* cur_c_min_;  // Current lower bound of the swarm's postions
    float* cur_c_max_;  // Current upper bound of the swarm's postions
    float* vel_max_;
    float* delta_c_;
    float* best_pos_global_;
    float best_residue_global_;
    float kappa_;  // Formula from paper

    SwarmNode** swarm_;

    static MERSINE_TWISTER_ENG eng;
    static UNIFORM_REAL_DISTRIBUTION dist_real;
    
    float interpolateCoeff(const float a, const float interp_val, 
      const float b, const float c, bool angle);

    inline void copyVec(float* dst, const float* src) {
      memcpy(dst, src, sizeof(dst[0]) * num_coeffs_); }
  };

};  // namespace math
};  // namespace jtil

#pragma once