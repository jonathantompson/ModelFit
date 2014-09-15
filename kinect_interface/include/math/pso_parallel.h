//
//  pso_parallel.h
//
//  Created by Jonathan Tompson on 11/23/12.
//
//  Same implementation as the regular pso but takes advantage of parallel 
//  objective function evaluations.
//

#pragma once

// NTILES is the number of evaluations that we want to perform in parallel
#define NTILES_DIM 8  // Number of tiles in the x and y dimensions
#define NTILES (NTILES_DIM * NTILES_DIM)  // Total number of tiles

#include <random>
#include "math/math_types.h"
#include "math/common_optimization.h"
#include "data_str/vector.h"
#include "data_str/vector_managed.h"

#if defined(WIN32) || defined(_WIN32)
  #define constexpr 
#endif

// #define USE_LPRPSO_UPDATE  // otherwise just a PSO update

namespace jtil {
namespace math {

  // PSO Optimization
  class PSOParallel {
  public:
    // Set population size (num_agents) to -1 to let the optimizer choose a
    // population size for you.
    PSOParallel(uint32_t num_coeffs, int swarm_size = -1, 
      int ntiles = NTILES);
    ~PSOParallel();

    // minimize():
    // - end_c: The output value.
    // - start_c: The starting inital guess and the center of the intial 
    //   search radius.  Should be chosen very carefully to ensure convergence.
    // - radius_c: The initial search radius.  Also, should be chosen
    //   very carefully to ensure proper convergence.
    void minimize(float* end_c,
                  const float* start_c,
                  const float* radius_c,
                  const bool* angle_coeff,  // can be NULL
                  const ObjectFuncParallelPtr obj_func,
                  const CoeffUpdateFuncPtr coeff_update_func);

    // Termination and Optimization settings:
    float delta_coeff_termination;  // The spread of the agent coefficients
    uint64_t max_iterations;
    // LPRPSO
    float C;
    float w;
    float Pr;
    // PSO
    float c_p;
    float c_g;
    float kappa;
    bool verbose;  // Set to true for detailed output

  private:
    ObjectFuncParallelPtr obj_func_parallel_;
    const bool* angle_coeffs_;
    const uint32_t num_coeffs_;
    uint32_t swarm_size_;
    const uint32_t ntiles_;
    float* c_lo_;  // lower bound of search space
    float* c_hi_;  // upper bound of search space
    float* cur_c_min_;  // Current lower bound of the swarm's postions
    float* cur_c_max_;  // Current upper bound of the swarm's postions
    float* vel_max_;
    float* delta_c_;
    float best_residue_global_;
    float* best_pos_global_;
    CoeffUpdateFuncPtr coeff_update_func_;

    data_str::Vector<float*> tiled_coeffs;  // size = 8 x 8 (default)
    data_str::Vector<float> tiled_residues;  // size = 8 x 8 (default)

    SwarmNode** swarm_;
    SwarmNode** ordered_swarm_;

    static MERSINE_TWISTER_ENG eng;
    static UNIFORM_REAL_DISTRIBUTION dist_real;
    
    // ret = a + interp_val * (b - c)
    // float interpolateCoeff(const float a, const float interp_val, 
    //   const float b, const float c, const bool angle);

    // ret = a - b  // Chooses the smaller of the angles
    float calcDisplacement(const float a, const float b, const bool angle);
    void InsertionSortSwarmPts();

    inline void copyVec(float* dst, const float* src) {
      memcpy(dst, src, sizeof(dst[0]) * num_coeffs_); }
  };

};  // namespace math
};  // namespace jtil

