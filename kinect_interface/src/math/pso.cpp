#include <cstring>
#include <random>
#include <stdexcept>
#include <iostream>
#include "math/pso.h"

using std::cout;
using std::endl;
using std::runtime_error;

#define SAFE_DELETE(x) if (x != NULL) { delete x; x = NULL; }
#define SAFE_DELETE_ARR(x) if (x != NULL) { delete[] x; x = NULL; }

namespace jtil {
namespace math {

  MERSINE_TWISTER_ENG PSO::eng;
  UNIFORM_REAL_DISTRIBUTION PSO::dist_real(0, 1);

  PSO::PSO(const uint32_t num_coeffs, const int swarm_size) {
    num_coeffs_ = num_coeffs;
    if (swarm_size > 0) {
      swarm_size_ = static_cast<uint32_t>(swarm_size);
    } else {
      swarm_size_ = 30;  // Recommended by "An Off-The-Shelf PSO"
    }

    best_pos_global_ = new float[num_coeffs_];
    c_lo_ = new float[num_coeffs_];
    c_hi_ = new float[num_coeffs_];
    cur_c_min_ = new float[num_coeffs_];
    cur_c_max_ = new float[num_coeffs_];
    delta_c_ = new float[num_coeffs_];
    vel_max_ = new float[num_coeffs_];

    // Allocate space for the N+1 probe points
    swarm_ = new SwarmNode*[swarm_size_];
    for (uint32_t i = 0; i < swarm_size_; i++) {
      swarm_[i] = new SwarmNode();
      swarm_[i]->resize(num_coeffs_);
    }

    // Some default parameters
    max_iterations = 1000;
    delta_coeff_termination = 1e-8f;
    phi_p = 2.8f;  // Recommended by "An Off-The-Shelf PSO"
    phi_g = 1.3f;  // Recommended by "An Off-The-Shelf PSO"

    verbose = false;
  }

  PSO::~PSO() {
    if (swarm_) {
      for (uint32_t i = 0; i < swarm_size_; i++) {
        SAFE_DELETE(swarm_[i]);
      }
    }
    SAFE_DELETE_ARR(best_pos_global_);
    SAFE_DELETE_ARR(c_lo_);
    SAFE_DELETE_ARR(c_hi_);
    SAFE_DELETE_ARR(cur_c_min_);
    SAFE_DELETE_ARR(cur_c_max_);
    SAFE_DELETE_ARR(delta_c_);
    SAFE_DELETE_ARR(vel_max_);
    SAFE_DELETE_ARR(swarm_);
  }

  void PSO::minimize(float* end_c, const float* start_c,
    const float* radius_c, const bool* angle_coeff, 
    const ObjectiveFuncPtr obj_func, 
    const CoeffUpdateFuncPtr coeff_update_func) {

    eng.seed();
    if (verbose) {
      cout << "Starting PSO optimization..." << endl;
    }

    // Initialize all random swarm particles to Uniform(c_lo_, c_hi_)
    for (uint32_t i = 0; i < num_coeffs_; i++) {
      float rad = fabsf(radius_c[i]);
      c_lo_[i] = start_c[i] - rad;
      c_hi_[i] = start_c[i] + rad;
      // According to paper (forgot title), 0.5x search space
      // is best for multi-modal distributions.
      vel_max_[i] = rad;
    }

    // Make the 0th particle the same as the start  --> Important for systems
    // that use the last frame as a start guess
    copyVec(swarm_[0]->pos, start_c); 

    // Stochasticly sample for the other particles
    for (uint32_t j = 0; j < num_coeffs_; j++) {
      UNIFORM_REAL_DISTRIBUTION c_dist(c_lo_[j], c_hi_[j]);
      for (uint32_t i = 1; i < swarm_size_; i++) {
        float uniform_rand_num = c_dist(eng);  // [c_lo_, c_hi_)
        swarm_[i]->pos[j] = uniform_rand_num;
      }
    }

    // evaluate the agent's function values, calculate residue and set the best
    // position and residue for the particle x_i as it's starting position
    best_residue_global_ = std::numeric_limits<float>::infinity();
    for (uint32_t i = 0; i < swarm_size_; i++) {
      if (coeff_update_func) {
        coeff_update_func(swarm_[i]->pos);
      }
      swarm_[i]->residue = obj_func(swarm_[i]->pos);
      swarm_[i]->best_residue = swarm_[i]->residue;
      copyVec(swarm_[i]->best_pos, swarm_[i]->pos);
      if (swarm_[i]->residue < best_residue_global_) {
        best_residue_global_ = swarm_[i]->residue;
        copyVec(best_pos_global_, swarm_[i]->pos);
      }
    }

    // Initialize random velocity to Uniform(-2*radius_c, 2*radius_c)
    for (uint32_t j = 0; j < num_coeffs_; j++) {
      UNIFORM_REAL_DISTRIBUTION c_dist(-2 * fabsf(radius_c[j]),
        2 * fabsf(radius_c[j]));
      for (uint32_t i = 0; i < swarm_size_; i++) {
        float uniform_rand_num = c_dist(eng);  // [-2*radius_c, 2*radius_c)
        swarm_[i]->vel[j] = uniform_rand_num;
      }
    }

    float phi = phi_p + phi_g;
    if (phi <= 4) {
      throw std::runtime_error("ERROR: kappa_ = phi_p + phi_g <= 4!");
    }
    kappa_ = 2.0f / fabsf(2.0f - phi - sqrtf(phi * phi - 4 * phi));

    if (verbose) {
      cout << "Iteration 0:" << endl;
      cout << "  --> min residue of swarm = " << best_residue_global_ << endl;
      cout << "  --> Agent residues: <";
      for (uint32_t i = 0; i < swarm_size_; i++) {
        cout << swarm_[i]->residue;
        if (i != swarm_size_ - 1) { cout << ", "; }
      }
      cout << ">" << endl;
    }

    uint64_t num_iterations = 0;
    float delta_coeff = std::numeric_limits<float>::infinity();
    do {
      // For each particle, i in the swarm:
      for (uint32_t i = 0; i < swarm_size_; i++) {
        SwarmNode* cur_node = swarm_[i];
        // For each dimension d:
        for (uint32_t d = 0; d < num_coeffs_; d++) {
          float r_p = dist_real(eng);  // [0,1)
          float r_g = dist_real(eng);  // [0,1)
          // Update the velocity
          cur_node->vel[d] = kappa_ * (cur_node->vel[d] + 
            (phi_p * r_p * (cur_node->best_pos[d] - cur_node->pos[d])) + 
            (phi_g * r_g * (best_pos_global_[d] - cur_node->pos[d])));
          // Limit the velocity as discussed in the paper "An Off-The-Shelf PSO"
          if (cur_node->vel[d] > vel_max_[d]) {
            cur_node->vel[d] = vel_max_[d];
          }
          if (cur_node->vel[d] < -vel_max_[d]) {
            cur_node->vel[d] = -vel_max_[d];
          }
        }  // for each dimension

        // Update the particle's postion
        for (uint32_t d = 0; d < num_coeffs_; d++) {
          cur_node->pos[d] = cur_node->pos[d] + cur_node->vel[d];
        }

        if (coeff_update_func) {
          coeff_update_func(cur_node->pos);
        }

        // Evaluate the function at the new position
        cur_node->residue = obj_func(cur_node->pos);

        if (cur_node->residue < cur_node->best_residue) {
          cur_node->best_residue = cur_node->residue;
          copyVec(cur_node->best_pos, cur_node->pos);
        }

        if (cur_node->residue < best_residue_global_) {
          best_residue_global_ = cur_node->residue;
          copyVec(best_pos_global_, cur_node->pos);
        }
      }  // for each agent

      num_iterations ++;

      // Calculate the spread in coefficients
      if (delta_coeff_termination > 0) {
        float l2_norm = 0.0f;
        for (uint32_t j = 0; j < num_coeffs_; j++) {
          cur_c_min_[j] = std::numeric_limits<float>::infinity();
          cur_c_max_[j] = -std::numeric_limits<float>::infinity();
          for (uint32_t i = 0; i < swarm_size_; i++) {
            cur_c_min_[j] = std::min<float>(cur_c_min_[j], swarm_[i]->pos[j]);
            cur_c_max_[j] = std::max<float>(cur_c_max_[j], swarm_[i]->pos[j]);
          }
          delta_c_[j] = cur_c_max_[j] - cur_c_min_[j];
          l2_norm += delta_c_[j] * delta_c_[j];
        }
        l2_norm = sqrtf(l2_norm);
        delta_coeff = l2_norm;
      } else {
        delta_coeff = std::numeric_limits<float>::infinity();
      }

      if (verbose) {
        cout << "Iteration " << num_iterations << ":" << endl;
        cout << "  --> min residue of swarm = " << best_residue_global_ << endl;
        cout << "  --> delta_coeff = " << delta_coeff << endl;
      }
    } while (num_iterations <= max_iterations && 
             delta_coeff >= delta_coeff_termination);
    
    if (verbose) {
      cout << endl << "Finished PSO optimization with ";
      cout << "residue " << best_residue_global_ << endl;
    }

    copyVec(end_c, best_pos_global_);
  }
  
  // interpolateCoeff performs the following:
  // ret = a + interp_val * (b - c)
  float PSO::interpolateCoeff(const float a, const float interp_val, 
    const float b, const float c, bool angle) {
    float interp;
    if (angle) {
      float real_a = cos(a);
      float imag_a = sin(a);
      float real_b = cos(b);
      float imag_b = sin(b);
      float real_c = cos(c);
      float imag_c = sin(c);
      float real_interp = real_a + interp_val * (real_b - real_c);
      float imag_interp = imag_a + interp_val * (imag_b - imag_c);
      interp = atan2(imag_interp, real_interp);
    } else {
      interp = a + interp_val * (b - c);
    }
    return interp;
  }

};  // namespace math
};  // namespace jtil
