//
//  model_fit.h
//
//  Created by Jonathan Tompson on 8/17/12.
//
//  Code to perform PSO to fit a known graphical model to input depth data.  
//  It does this by rendering using openGL.
//
//  Obviously, the region of convergence is finite so starting values matter.
//  pso_radius_c sets the radius in each dimension that the PSO will sample
//  from.  This should be as tight as possible.
//
//  For better results, you should set the coeff_min_limit, coeff_max_limit and
//  coeff_penalty scale parameters to provide a prior on your coeff limits.
//  This can drasticly reduce the space of possible poses.
//
//  Finally, you must set the angle_coeffs boolean array.  This simply marks
//  those coefficients that represent angles so that the optimizer handles them
//  properly.
//

#pragma once

#include "math/math_types.h"
#include "data_str/vector.h"
#include "math/common_optimization.h"  // For CoeffUpdateFuncPtr
#include "math/pso_parallel.h"

#define NTILES_DEFAULT NTILES
#define NTILES_X NTILES_DIM
#define NTILES_Y NTILES_DIM

// Particle-Swarm optimizer settings
#define PSO_MAX_ITERATIONS 300  // Default 501
#define PSO_DELTA_C_TERMINATION 1e-3f
#define PSO_SWARM_SIZE 64  // Default 128 or 64
#define PSO_REPEATS 1  // Default 2
 
// #define PREV_FRAME_DIST_PENALTY  // Distance from last frame penalty
#define PREV_FRAME_DIST_PENALTY_SCALE  0.001f
#define PREV_FRAME_DIST_PENALTY_THRESHOLD  0.3f  // % of total range
#define MAX_DEPTH_IN_RESIDUE 30.0f  // default 40 (from paper) but 30 works better
#define DATA_TERM_LAMBDA 0.2f  // default 0.0025f  (higher values = depth difference is more important)
#define INTERPENETRATION_CONSTANT 0.1f

namespace jtil { namespace math { class PSOParallel; } }
 
namespace model_fit {
  class ModelRenderer;
  class PoseModel;

  class ModelFit {
  public:
    friend class jtil::math::PSOParallel;
    // Constructor / Destructor
    ModelFit(const uint32_t num_models, const uint32_t coeff_dim_per_model, 
      const uint32_t num_cameras);
    ~ModelFit();

    // fitModel - Top function:
    // Requirements: depth and label array for each num_cameras
    //               models, coeffs and prev_coeffs for each num_models
    void fitModel(int16_t** depth, uint8_t** label, PoseModel** models, 
      float** coeffs, float** prev_coeffs, 
      jtil::math::CoeffUpdateFuncPtr coeff_update_func);

    float queryObjFunc(int16_t** depth, uint8_t** label, PoseModel** models, 
      float** coeffs);

    inline void resetFuncEvalCount() { func_eval_count_ = 0; }
    inline uint64_t func_eval_count() { return func_eval_count_; }

    ModelRenderer* model_renderer() { return model_renderer_; }

    void setCameraView(const uint32_t i_camera, 
      const jtil::math::Float4x4& view);
    void getCameraView(const uint32_t i_camera, 
      jtil::math::Float4x4& view_ret);

  private:
    uint32_t coeff_dim_; 
    uint32_t coeff_dim_per_model_;
    uint32_t num_models_;
    uint32_t num_cameras_;
    PoseModel** models_;  // Not owned here
    static ModelFit* cur_fit_;
    static uint64_t func_eval_count_;
    ModelRenderer* model_renderer_;

    float* coeff_optim_prev_;  // Flattened coeff array
    float* coeff_optim_;  // Flattened coeff array
    float* coeff_tmp_;  // Flattened coeff array

    jtil::math::PSOParallel* pso_;
    float* cur_obj_func_coeff_;
    int16_t* kinect_depth_masked_;
    static const float finger_crossover_penalty_threshold;

    // save_next_image_set_ --> The next synthetic, depth and residue textures
    // will be saved (so that we can plot them nicely in Matlab)
    bool save_next_image_set_;

    // static functions for non-linear fitting
    static float objectiveFunc(const float* coeff);
    static void objectiveFuncTiled(jtil::data_str::Vector<float>& residues, 
      jtil::data_str::Vector<float*>& coeffs);
    // If any of the pointers are NULL then the term isn't calculated.
    // All terms are accumulated to the current value.
    static void calculateResidual(const float* coeff, const uint32_t i_camera,
      float* depth_term, float* penalty_term, float* interpen_term);
    static void calculateResidualTiled(jtil::data_str::Vector<float>* depth_term, 
      jtil::data_str::Vector<float>* penalty_term,
      jtil::data_str::Vector<float*>& coeffs, const uint32_t i_camera);
    static float calcPenalty(const float* coeff);
    static float calcDistPenalty(const float* coeff0, const float* coeff1);

    void prepareOptimization(int16_t** depth, uint8_t** label, 
      PoseModel** models, float** coeffs, float** prev_coeffs,
      jtil::data_str::Vector<bool>& old_attachement_vals);
    void prepareKinectData(int16_t** depth, uint8_t** label);

    // Non-copyable, non-assignable.
    ModelFit(ModelFit&);
    ModelFit& operator=(const ModelFit&);
  };
};  // namespace model_fit
