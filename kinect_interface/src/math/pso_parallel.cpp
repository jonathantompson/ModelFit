#include <cstring>
#include <random>
#include <stdexcept>
#include <iostream>
#include "math/pso_parallel.h"

using std::cout;
using std::endl;
using std::runtime_error;

#define SAFE_DELETE(x) if (x != NULL) { delete x; x = NULL; }
#define SAFE_DELETE_ARR(x) if (x != NULL) { delete[] x; x = NULL; }

namespace jtil {
namespace math {

  MERSINE_TWISTER_ENG PSOParallel::eng;
  UNIFORM_REAL_DISTRIBUTION PSOParallel::dist_real(0, 1);

  PSOParallel::PSOParallel(uint32_t num_coeffs, int swarm_size, int ntiles) :
    num_coeffs_(num_coeffs), ntiles_(ntiles) {
    if (swarm_size > 0) {
      swarm_size_ = static_cast<uint32_t>(swarm_size);
    } else {
      swarm_size_ = 30;  // Recommended by "An Off-The-Shelf PSO"
    }

    if (swarm_size % ntiles_ != 0) {
      throw std::runtime_error("swarm_size must be a multiple of ntiles_!");
    }

    best_pos_global_ = new float[num_coeffs_];
    c_lo_ = new float[num_coeffs_];
    c_hi_ = new float[num_coeffs_];
    cur_c_min_ = new float[num_coeffs_];
    cur_c_max_ = new float[num_coeffs_];
    delta_c_ = new float[num_coeffs_];
    vel_max_ = new float[num_coeffs_];

    // Allocate space for the N probe points
    swarm_ = new SwarmNode*[swarm_size_];
    ordered_swarm_ = new SwarmNode*[swarm_size_];
    for (uint32_t i = 0; i < swarm_size_; i++) {
      swarm_[i] = new SwarmNode();
      swarm_[i]->resize(num_coeffs_);
      ordered_swarm_[i] = swarm_[i];
    }

    // Some default parameters
    max_iterations = 1000;
    delta_coeff_termination = 1e-4f;
    // Pr = (1.0f / static_cast<float>(num_coeffs)) * expf(-2.0f);  // Paper value
    Pr = (1.0f / static_cast<float>(num_coeffs));  // My value

    // Values recommended by "Extended PSO with Partial Randomization..."
    // C = 3.0f;  
    // w = 0.729f;
    // Values consistent with other papers
    C = 3.0f;  
    w = 0.7f;

    c_p = 2.8f;  // Recommended by "An Off-The-Shelf PSO"
    c_g = 1.3f;  // Recommended by "An Off-The-Shelf PSO"

    tiled_coeffs.capacity(ntiles_);
    for (uint32_t i = 0; i < ntiles_; i++) {
      tiled_coeffs.pushBack(new float[num_coeffs_]);
    }
    tiled_residues.capacity(ntiles_);
    tiled_residues.resize(ntiles_);

    verbose = false;
  }

  PSOParallel::~PSOParallel() {
    if (swarm_) {
      for (uint32_t i = 0; i < swarm_size_; i++) {
        SAFE_DELETE(swarm_[i]);
      }
    }
    for (uint32_t i = 0; i < tiled_coeffs.size(); i++) {
      SAFE_DELETE_ARR(tiled_coeffs[i]);
    }
    SAFE_DELETE_ARR(swarm_);
    SAFE_DELETE_ARR(ordered_swarm_);
    SAFE_DELETE_ARR(best_pos_global_);
    SAFE_DELETE_ARR(c_lo_);
    SAFE_DELETE_ARR(c_hi_);
    SAFE_DELETE_ARR(cur_c_min_);
    SAFE_DELETE_ARR(cur_c_max_);
    SAFE_DELETE_ARR(delta_c_);
    SAFE_DELETE_ARR(vel_max_);
  }

  void PSOParallel::minimize(float* end_c, const float* start_c,
    const float* radius_c, const bool* angle_coeff, 
    const ObjectFuncParallelPtr obj_func, 
    const CoeffUpdateFuncPtr coeff_update_func) {
    angle_coeffs_ = angle_coeff;
    obj_func_parallel_ = obj_func;
    coeff_update_func_ = coeff_update_func;

    eng.seed();
    float phi = c_p + c_g;
    if (phi <= 4) {
      throw std::runtime_error("ERROR: kappa_ = phi_p + phi_g <= 4!");
    }
    kappa = 2.0f / fabsf(2.0f - phi - sqrtf(phi * phi - 4 * phi));

    if (verbose) {
      cout << "Starting LPRPSO optimization..." << endl;
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

    // Make the 0th particle the same as the start
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
      coeff_update_func_(swarm_[i]->pos);
    }
    for (uint32_t i = 0; i < (swarm_size_ / ntiles_); i++) {
      for (uint32_t j = 0; j < ntiles_; j++) {
        copyVec(tiled_coeffs[j], swarm_[i * ntiles_ + j]->pos);
      }
      obj_func_parallel_(tiled_residues, tiled_coeffs);
      for (uint32_t j = 0; j < ntiles_; j++) {
        swarm_[i * ntiles_ + j]->residue = tiled_residues[j];
        swarm_[i * ntiles_ + j]->best_residue = swarm_[i * ntiles_ + j]->residue;
        copyVec(swarm_[i * ntiles_ + j]->best_pos, swarm_[i * ntiles_ + j]->pos);
        if (swarm_[i * ntiles_ + j]->residue < best_residue_global_) {
          best_residue_global_ = swarm_[i * ntiles_ + j]->residue;
          copyVec(best_pos_global_, swarm_[i * ntiles_ + j]->pos);
        }
      }
    }


    // Initialize random velocity to Uniform(-vel_max_, vel_max_)
    for (uint32_t j = 0; j < num_coeffs_; j++) {
      UNIFORM_REAL_DISTRIBUTION c_dist(-vel_max_[j], vel_max_[j]);
      for (uint32_t i = 0; i < swarm_size_; i++) {
        float uniform_rand_num = c_dist(eng);  // [-2*radius_c, 2*radius_c)
        swarm_[i]->vel[j] = uniform_rand_num;
      }
    }

    if (verbose) {
      // Calculate the spread in coefficients
      for (uint32_t j = 0; j < num_coeffs_; j++) {
        cur_c_min_[j] = std::numeric_limits<float>::infinity();
        cur_c_max_[j] = -std::numeric_limits<float>::infinity();
        for (uint32_t i = 0; i < swarm_size_; i++) {
          if (cur_c_min_[j] > swarm_[i]->pos[j]) {
            cur_c_min_[j] = swarm_[i]->pos[j];
          }
          if (cur_c_max_[j] < swarm_[i]->pos[j]) {
            cur_c_max_[j] = swarm_[i]->pos[j];
          }
        }
        delta_c_[j] = cur_c_max_[j] - cur_c_min_[j];
      }
      cout << "Iteration 0:" << endl;
      cout << "  --> min residue of swarm = " << best_residue_global_ << endl;
      cout << "  --> Agent residues: <";
      for (uint32_t i = 0; i < swarm_size_; i++) {
        cout << swarm_[i]->residue;
        if (i != swarm_size_ - 1) { cout << ", "; }
      }
      cout << ">" << endl;
      cout << "  --> delta_c_ = "; 
      for (uint32_t i = 0; i < num_coeffs_; i++) { cout << delta_c_[i] << " "; }
      float bounding_area = delta_c_[0];
      for (uint32_t i = 1; i < num_coeffs_; i++) {
        bounding_area *= delta_c_[i];
      }
      cout << endl << "  --> bounding_area = " << bounding_area << endl;
    }

    uint64_t num_iterations = 0;
    float delta_coeff = std::numeric_limits<float>::infinity();
    float preturb_rad = 1.0f;
    // Exponential decay, want preturb_rad = 1e-4 on max_iteration
    const float nth_rad = 1e-4f;
    const float preturb_rad_decay =
      powf(10.0f, log10f(nth_rad) / static_cast<float>(max_iterations));
    do {
#ifdef USE_LPRPSO_UPDATE
      // Particles need to be ranked by their residue:
      InsertionSortSwarmPts();  // O(N^2) worst case, O(N) likely best case
#endif

      // For each particle, i in the swarm:
      for (uint32_t i = 0; i < swarm_size_; i++) {
        SwarmNode* cur_node = ordered_swarm_[i];

#ifdef USE_LPRPSO_UPDATE
        // LPRPSO UPDATE
        float rank = static_cast<float>(i + 1);  // 0 --> Best, swarm_size_-1 --> Worst
        rank = std::min<float>((float)swarm_size_, rank + (float)swarm_size_ / 2.0f);
        float tmp = static_cast<float>(swarm_size_ - 1);
        float tmp2 = rank - static_cast<float>(swarm_size_);
        float c_p = (C / (tmp * tmp)) * (tmp2 * tmp2);
        float c_g = C - c_p;
        // For each dimension d:
        for (uint32_t d = 0; d < num_coeffs_; d++) {
          float r_p = dist_real(eng);  // [0,1)
          float r_g = dist_real(eng);  // [0,1)
          float delta_p = calcDisplacement(cur_node->best_pos[d], 
            cur_node->pos[d], angle_coeffs_[d]);
          float delta_g = calcDisplacement(best_pos_global_[d], 
            cur_node->pos[d], angle_coeffs_[d]);
          // Update the velocity
          cur_node->vel[d] = w * cur_node->vel[d] + (c_p * r_p * delta_p) + 
            (c_g * r_g * delta_g);
          // Limit the velocity as discussed in the paper "An Off-The-Shelf PSO"
          if (cur_node->vel[d] > vel_max_[d]) {
            cur_node->vel[d] = vel_max_[d];
          }
          if (cur_node->vel[d] < -vel_max_[d]) {
            cur_node->vel[d] = -vel_max_[d];
          }
        }  // for each dimension
#else
        // PSO UPDATE
        // For each dimension d:
        for (uint32_t d = 0; d < num_coeffs_; d++) {
          float r_p = dist_real(eng);  // [0,1)
          float r_g = dist_real(eng);  // [0,1)
          float delta_p = calcDisplacement(cur_node->best_pos[d], 
            cur_node->pos[d], angle_coeffs_[d]);
          float delta_g = calcDisplacement(best_pos_global_[d], 
            cur_node->pos[d], angle_coeffs_[d]);
          cur_node->vel[d] = kappa * (cur_node->vel[d] + (c_p * r_p * delta_p) + 
            (c_g * r_g * delta_g));
          // Limit the velocity as discussed in the paper "An Off-The-Shelf PSO"
          if (cur_node->vel[d] > vel_max_[d]) {
            cur_node->vel[d] = vel_max_[d];
          }
          if (cur_node->vel[d] < -vel_max_[d]) {
            cur_node->vel[d] = -vel_max_[d];
          }
        }  // for each dimension
#endif

        // Update the particle's postion
        for (uint32_t d = 0; d < num_coeffs_; d++) {
          cur_node->pos[d] = cur_node->pos[d] + cur_node->vel[d];
        }

#ifdef USE_LPRPSO_UPDATE
        // As per the PrPSO part of the paper, randomly preturb particles on
        // a restart.
        if (num_iterations < max_iterations - 1) {
          // For each dimension d:
          for (uint32_t d = 0; d < num_coeffs_; d++) {
            float rand = dist_real(eng);  // [0,1)
            // With some probability, preterb the position of the d-th coeff
            // using Uniform(c_lo_, c_hi_)
            if (rand <= Pr) {
              // The paper version --> Completely new position (kinda silly)
              // cur_node->pos(d) =
              //   (preturb_rad * (c_hi_(d) - c_lo_(d))) * dist_real(eng) + c_lo_(d);  

              // My version --> Preturb away from where the particle is now
              rand = rand * 2.0f - 1.0f;  // [-1, 1]
              cur_node->pos[d] =
                (preturb_rad * (c_hi_[d] - c_lo_[d])) * rand + cur_node->pos[d];  
            }
          }  // for each dimension
        }
#endif

        coeff_update_func_(cur_node->pos);
      }  // for each agent

      // Calculate the tiled residues
      for (uint32_t i = 0; i < (swarm_size_ / ntiles_); i++) {
        for (uint32_t j = 0; j < ntiles_; j++) {
          copyVec(tiled_coeffs[j], swarm_[i * ntiles_ + j]->pos);
        }
        obj_func_parallel_(tiled_residues, tiled_coeffs);
        for (uint32_t j = 0; j < ntiles_; j++) {
          SwarmNode* cur_node = swarm_[i * ntiles_ + j];
          cur_node->residue = tiled_residues[j];
          if (cur_node->residue < cur_node->best_residue) {
            cur_node->best_residue = cur_node->residue;
            copyVec(cur_node->best_pos, cur_node->pos);
          }
          if (cur_node->residue < best_residue_global_) {
            best_residue_global_ = cur_node->residue;
            copyVec(best_pos_global_, cur_node->pos);
          }
        }
      }

      num_iterations ++;

      // Calculate the spread in coefficients
      if (delta_coeff_termination > 0) {
        float l2_norm = 0.0f;
        for (uint32_t j = 0; j < num_coeffs_; j++) {
          cur_c_min_[j] = std::numeric_limits<float>::infinity();
          cur_c_max_[j] = -std::numeric_limits<float>::infinity();
          for (uint32_t i = 0; i < swarm_size_; i++) {
            if (cur_c_min_[j] > swarm_[i]->pos[j]) {
              cur_c_min_[j] = swarm_[i]->pos[j];
            }
            if (cur_c_max_[j] < swarm_[i]->pos[j]) {
              cur_c_max_[j] = swarm_[i]->pos[j];
            }
          }
          delta_c_[j] = cur_c_max_[j] - cur_c_min_[j];
          l2_norm += delta_c_[j] * delta_c_[j];
        }
        l2_norm = sqrtf(l2_norm);
        delta_coeff = l2_norm;
      } else {
        delta_coeff = std::numeric_limits<float>::infinity();
      }
      preturb_rad *= preturb_rad_decay;

      if (verbose) {
        cout << "Iteration " << num_iterations << ":" << endl;
        cout << "  --> min residue of swarm = " << best_residue_global_ << endl;
        cout << "  --> delta_coeff = " << delta_coeff << endl;
        cout << "  --> delta_c_ = "; 
        for (uint32_t i = 0; i < num_coeffs_; i++) { cout << delta_c_[i] << " "; }
        float bounding_area = delta_c_[0];
        for (uint32_t i = 1; i < num_coeffs_; i++) {
          bounding_area *= delta_c_[i];
        }
        cout << endl << "  --> bounding_area = " << bounding_area << endl;
      }
    } while (num_iterations <= max_iterations && 
             delta_coeff >= delta_coeff_termination);
    
    if (verbose) {
      cout << "Finished PSO optimization with residue ";
      cout << best_residue_global_ << endl;
    }

    copyVec(end_c, best_pos_global_);
  }

  float PSOParallel::calcDisplacement(const float a, const float b, 
    const bool angle) {
    float disp;
    if (angle) {
      // Always choose the smaller angle difference
      float disp1 = a - b;
      float disp2;
      if (disp1 > 0) {
        disp2 = disp1 - static_cast<float>(2.0 * M_PI);
      } else {
        disp2 = disp1 + static_cast<float>(2.0 * M_PI);
      }
      if (fabsf(disp1) < fabsf(disp2)) {
        disp = disp1;
      } else {
        disp = disp2;
      }
    } else {
      disp = a - b;
    }
    return disp;
  }

  // http://en.wikipedia.org/wiki/Insertion_sort
  void PSOParallel::InsertionSortSwarmPts() {
    for (uint32_t i = 1; i < swarm_size_; i++) {
      SwarmNode* item = ordered_swarm_[i];
      uint32_t i_hole = i;
      while (i_hole > 0 && ordered_swarm_[i_hole - 1]->residue > item->residue) {
        // move hole to next smaller index
        ordered_swarm_[i_hole] = ordered_swarm_[i_hole - 1];
        i_hole--;
      }
      // put item in the hole
      ordered_swarm_[i_hole] = item;
    }
#if (defined(DEBUG) || defined(_DEBUG))
    if (verbose) {
      cout << endl << "ordered_swarm = ";
      for (uint32_t i = 0; i < swarm_size_; i++) {
        cout << ordered_swarm_[i]->residue;
        if (i != swarm_size_ - 1) {
          cout << ", ";
        }
      }
      cout << endl << endl;
    }
#endif
  }

}  // namespace math
}  // namespace jtil
