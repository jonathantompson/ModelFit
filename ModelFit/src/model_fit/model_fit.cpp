#include <random>
#include <thread>
#include <iostream>
#include <stdexcept>
#include "model_fit/model_fit.h"
#include "model_fit/model_renderer.h"
#include "model_fit/pose_model.h"
#include "data_str/vector_managed.h"
#include "string_util/string_util.h"
#include "math/pso_parallel.h"
#include "file_io/file_io.h"
#include "renderer/gl_state.h"
#include "renderer/camera/camera.h"
#include "renderer/texture/texture_renderable.h"
#include "kinect_interface_primesense/kinect_interface_primesense.h"  // for src_dim

#define SAFE_DELETE(x) if (x != NULL) { delete x; x = NULL; }
#define SAFE_DELETE_ARR(x) if (x != NULL) { delete[] x; x = NULL; }

using namespace jtil::math;
using namespace jtil::data_str;
using std::string;
using std::runtime_error;
using std::cout;
using std::endl;
using jtil::data_str::VectorManaged;
using renderer::GLState;

namespace model_fit {
  
  // Static variables
  ModelFit* ModelFit::cur_fit_ = NULL;
  uint64_t ModelFit::func_eval_count_ = 0;
 
  ModelFit::ModelFit(const uint32_t num_models, 
    const uint32_t coeff_dim_per_model, const uint32_t num_cameras) {
    if (num_models == 0 || coeff_dim_per_model == 0) {
      throw std::runtime_error("ModelFit::ModelFit() - ERROR: check inputs!");
    }

    save_next_image_set_ = false;
    num_cameras_ = num_cameras;
    num_models_ = num_models;
    coeff_dim_per_model_ = coeff_dim_per_model;
    coeff_dim_ = coeff_dim_per_model_ * num_models_;

    coeff_optim_prev_ = new float[coeff_dim_];
    coeff_optim_ = new float[coeff_dim_];
    coeff_tmp_ = new float[coeff_dim_];

    pso_ = new PSOParallel(coeff_dim_, PSO_SWARM_SIZE);
    pso_->max_iterations = PSO_MAX_ITERATIONS;
    pso_->delta_coeff_termination = PSO_DELTA_C_TERMINATION;

    kinect_depth_masked_ = new int16_t[src_dim];

    model_renderer_ = new ModelRenderer(num_cameras_);
  }

  ModelFit::~ModelFit() {
    SAFE_DELETE(pso_);
    SAFE_DELETE_ARR(kinect_depth_masked_);
    SAFE_DELETE_ARR(coeff_optim_);
    SAFE_DELETE_ARR(coeff_optim_prev_);
    SAFE_DELETE_ARR(coeff_tmp_);
    SAFE_DELETE(model_renderer_);
  }

  void ModelFit::prepareKinectData(int16_t** depth, uint8_t** label) {
    for (uint32_t i_camera = 0; i_camera < num_cameras_; i_camera++) {
      int16_t* cur_depth = depth[i_camera];
      memcpy(kinect_depth_masked_, cur_depth, sizeof(kinect_depth_masked_[0]) * 
        src_dim);
  #ifdef DEPTH_ONLY_RESIDUE_FUNC
      // Do nothing
  #else
      for (uint32_t i = 0; i < src_dim; i++) {
        if (label[i_camera][i] == 0) {
          kinect_depth_masked_[i] = 0;
        }
      }
  #endif
      model_renderer_->uploadDepth(i_camera, kinect_depth_masked_);
    }
  }

  void ModelFit::setCameraView(const uint32_t i_camera, 
    const jtil::math::Float4x4& view) {
    model_renderer_->camera(i_camera)->set_view_mat_directly = true;
    model_renderer_->camera(i_camera)->view()->set(view);
  }

  void ModelFit::getCameraView(const uint32_t i_camera, 
    jtil::math::Float4x4& view) {
    view.set(*model_renderer_->camera(i_camera)->view());
  }

  void ModelFit::fitModel(int16_t** depth, uint8_t** label, PoseModel** models, 
    float** coeffs, float** prev_coeffs,
    CoeffUpdateFuncPtr coeff_update_func) {
    Vector<bool> old_attachement_vals(num_models_);
    prepareOptimization(depth, label, models, coeffs, prev_coeffs,
      old_attachement_vals);

    // This is a hack: There's something wrong with the first iteration using
    // tiled rendering --> Doing one regular render pass helps.
    float start_func_val = objectiveFunc(coeff_optim_);
    cout << "Starting objective function value = " << start_func_val << endl;
    
    // PSO fitting
    for (uint32_t i = 0; i < PSO_REPEATS; i++) {
      pso_->minimize(coeff_tmp_, coeff_optim_, models_[0]->pso_radius_c(),
         models_[0]->angle_coeffs(), objectiveFuncTiled, coeff_update_func);
      // Now copy back the new coeff values
      memcpy(coeff_optim_, coeff_tmp_, sizeof(coeff_optim_[0]) * coeff_dim_);
    }

    // Copy the local coeff values back into the return value
    for (uint32_t i = 0; i < num_models_; i++) {
      memcpy(coeffs[i], &coeff_optim_[i * coeff_dim_per_model_], 
        sizeof(coeffs[i][0]) * coeff_dim_per_model_);
    }

    for (uint32_t i = 0; i < num_models_; i++) {
      models_[i]->setRendererAttachement(old_attachement_vals[i]);
    }

    float end_func_val = objectiveFunc(coeff_optim_);
    cout << "Final objective function value = " << end_func_val << endl;
  }


  void ModelFit::prepareOptimization(int16_t** depth, uint8_t** label, 
    PoseModel** models, float** coeffs, float** prev_coeffs,
    Vector<bool>& old_attachement_vals) {
    models_ = models;
    
    // Detach the models from the global renderer so that there aren't issues
    // with scene graph parent transformation inheritance.
    for (uint32_t i = 0; i < num_models_; i++) {
      old_attachement_vals.pushBack(models_[i]->getRendererAttachement());
      models_[i]->setRendererAttachement(false);
    }

    for (uint32_t i = 0; i < num_models_; i++) {
      memcpy(&coeff_optim_[i * coeff_dim_per_model_], coeffs[i], 
        sizeof(coeff_optim_[0]) * coeff_dim_per_model_);
    }
    if (prev_coeffs != NULL) {
      for (uint32_t i = 0; i < num_models_; i++) {
        memcpy(&coeff_optim_prev_[i * coeff_dim_per_model_], prev_coeffs[i], 
          sizeof(coeff_optim_prev_[0]) * coeff_dim_per_model_);
      }
    } else {
      memcpy(coeff_optim_prev_, coeff_optim_, 
        sizeof(coeff_optim_prev_[0]) * coeff_dim_);
    }

    cur_fit_ = this;
    prepareKinectData(depth, label);
  }

  float ModelFit::queryObjFunc(int16_t** depth, uint8_t** label, 
    PoseModel** models, float** coeffs) {
    Vector<bool> old_attachement_vals(num_models_);
    prepareOptimization(depth, label, models, coeffs, NULL,
      old_attachement_vals);

    jtil::file_io::SaveArrayToFile<int16_t>(depth[0], 640*480, 
      "./kinect_texture.bin");

    // This is a hack: There's something wrong with the first iteration using
    // tiled rendering --> Doing one regular render pass helps.
    save_next_image_set_ = true;
    float start_func_val = objectiveFunc(coeff_optim_);
    cout << "objective function value = " << start_func_val << endl;

    for (uint32_t i = 0; i < num_models_; i++) {
      models_[i]->setRendererAttachement(old_attachement_vals[i]);
    }

    return start_func_val;
  }

  char* float2CStr(float val) {
    string str = jtil::string_util::Num2Str<float>(val);
    char* c_str = new char[str.length()+1];
    std::copy(str.begin(), str.end(), c_str);
    c_str[str.size()] = '\0';
    return c_str;
  }
  
  // modulu - similar to matlab's mod()
  // result is always possitive. not similar to fmod()
  // Mod(-3,4)= 1   fmod(-3,4)= -3
#if defined(__APPLE__) || defined(_WIN32)
  float inline __fastcall Mod(float x, float y) {
    if (0 == y) {
      return x;
    }
    
    return x - y * floor(x / y);
  }
#else
  float inline Mod(float x, float y) {
    if (0 == y) {
      return x;
    }

    return x - y * floor(x / y);
  }
#endif
  
#if defined(WIN32) || defined(_WIN32)
  std::tr1::mt19937 eng;  // a core engine class
  std::tr1::normal_distribution<float> dist;
#else
  std::mt19937 eng;  // a core engine class
  std::normal_distribution<float> dist;
#endif

  void ModelFit::calculateResidual(const float* coeff, const uint32_t i_camera,
    float* depth_term, float* penalty_term, float* interpen_term) {
    // Residue is calculated on the GPU
    if (depth_term) {
      *depth_term += 
        cur_fit_->model_renderer_->calculateResidualDataTerm(i_camera);
    }
    
    if (interpen_term) {
      const uint32_t max_groups = cur_fit_->models_[0]->max_bsphere_groups();
      *interpen_term += 
        cur_fit_->model_renderer_->calcInterpenetrationTerm(max_groups);
    }

    if (penalty_term) {
      *penalty_term += calcPenalty(coeff);  // formally scaled by 0.1
    }
  }

  void ModelFit::calculateResidualTiled(Vector<float>* depth_term, 
    Vector<float>* penalty_term, Vector<float*>& coeffs, 
    const uint32_t i_camera) {
    // Note: at this point interpenetration term is already calculated and is
    // sitting in residues[i]
    if (depth_term) {
      cur_fit_->model_renderer_->calculateResidualDataTermTiled(*depth_term, 
        i_camera);
    }

    // Now calculate the penalty terms:
    if (penalty_term) {
      for (uint32_t i = 0; i < coeffs.size(); i++) {
        (*penalty_term)[i] += calcPenalty(coeffs[i]);
      }
    }
  }

  float ModelFit::calcPenalty(const float* coeff) {
    float penalty = 1.0f;
    const uint32_t coeff_dim = cur_fit_->coeff_dim_;
    const uint32_t coeff_dim_per_model = cur_fit_->coeff_dim_per_model_;
    const float* penalty_scale = cur_fit_->models_[0]->coeff_penalty_scale();
    const float* max_limit = cur_fit_->models_[0]->coeff_max_limit();
    const float* min_limit = cur_fit_->models_[0]->coeff_min_limit();

    for (uint32_t i = 0; i < coeff_dim; i++) {
      if (penalty_scale[i % coeff_dim_per_model] > EPSILON) {
        float cur_coeff_val = coeff[i];

#ifdef LINEAR_PENALTY
        // Linear penalty
        if (cur_coeff_val > max_limit[i % coeff_dim_per_model]) {
          penalty += penalty_scale[i % coeff_dim_per_model] * 
            fabsf(cur_coeff_val - max_limit[i % coeff_dim_per_model]) / 10.0f;
        }
        if (cur_coeff_val < min_limit[i % coeff_dim_per_model]) { 
          penalty += penalty_scale[i % coeff_dim_per_model] * 
            fabsf(min_limit[i % coeff_dim_per_model] - cur_coeff_val) / 10.0f;
        }
#else
        // Quadratic penalty
        if (cur_coeff_val > max_limit[i % coeff_dim_per_model]) {
          float cur_penalty = penalty_scale[i % coeff_dim_per_model] * 
            fabsf(cur_coeff_val - max_limit[i % coeff_dim_per_model]) / 2.0f;
          penalty += (cur_penalty * cur_penalty);
        }
        if (cur_coeff_val < min_limit[i % coeff_dim_per_model]) { 
          float cur_penalty = penalty_scale[i % coeff_dim_per_model] * 
            fabsf(min_limit[i % coeff_dim_per_model] - cur_coeff_val) / 2.0f;
          penalty += (cur_penalty * cur_penalty);
        }
#endif
      }
    }

#ifdef PREV_FRAME_DIST_PENALTY
      penalty += calcDistPenalty(cur_fit_->coeff_optim_prev_, coeff);
#endif

    return penalty;
  }

  float ModelFit::calcDistPenalty(const float* coeff0, const float* coeff1) {
    float dist = 0;  // Prevent NANs
    const uint32_t coeff_dim_per_model = cur_fit_->coeff_dim_per_model_;
    const float* penalty_scale = cur_fit_->models_[0]->coeff_penalty_scale();
    const float* max_limit = cur_fit_->models_[0]->coeff_max_limit();
    const float* min_limit = cur_fit_->models_[0]->coeff_min_limit();
    for (uint32_t i = 0; i < cur_fit_->coeff_dim_; i++) {
      if (penalty_scale[i % coeff_dim_per_model] > EPSILON) {
        float delta = fabsf(coeff0[i] - coeff1[i]);
        float err = delta - PREV_FRAME_DIST_PENALTY_THRESHOLD * 
          (max_limit[i] - min_limit[i]);
        if (err > 0) {
          dist += err * err;
        }
      }
    }
    return PREV_FRAME_DIST_PENALTY_SCALE * dist;
  }

  float data_temp[640*480*3];
  float ModelFit::objectiveFunc(const float* coeff) {
    float depth_term = 0.0f;
    float penalty_term = 0.0f;
    float interpenetration_term = 0.0f;
    for (uint32_t i_camera = 0; i_camera < cur_fit_->num_cameras_; i_camera++) {
      cur_fit_->model_renderer_->drawDepthMap(coeff, 
        cur_fit_->coeff_dim_per_model_, cur_fit_->models_, 
        cur_fit_->num_models_, i_camera, false);
      // Only calculate interpenetration and penalty terms on the first camera 
      // angle to save computation time
      if (i_camera == 0) {
        calculateResidual(coeff, i_camera, &depth_term, &penalty_term, 
          &interpenetration_term);
      } else {
        calculateResidual(coeff, i_camera, &depth_term, NULL, NULL);
      }

      if (cur_fit_->save_next_image_set_) {
        cur_fit_->model_renderer_->depth_texture()->getTexture0Data<float>(data_temp);
        jtil::file_io::SaveArrayToFile<float>(data_temp, 640*480, 
          "./synth_texture.bin");
        cur_fit_->model_renderer_->residue_texture_1()->getTexture0Data<float>(data_temp);
        jtil::file_io::SaveArrayToFile<float>(data_temp, 640*480, 
          "./residue_texture.bin");
        cur_fit_->save_next_image_set_ = false;

      }
    }
    return depth_term * penalty_term * interpenetration_term;
  }

  void ModelFit::objectiveFuncTiled(Vector<float>& residues, 
    Vector<float*>& coeffs) {
    if (coeffs.size() > NTILES_DEFAULT) {
      throw runtime_error("objectiveFuncTiled() - coeffs.size() > NTILES");
    }
    if (residues.capacity() < coeffs.size()) {
      residues.capacity(coeffs.size());
    }

    // Zero out some accumulators
    Vector<float> depth_term(coeffs.size());
    Vector<float> penalty_term(coeffs.size());
    Vector<float> interpenetration_term(coeffs.size());
    for (uint32_t i = 0; i < coeffs.size(); i++) {
      depth_term.pushBack(0);
      penalty_term.pushBack(0);
      interpenetration_term.pushBack(0);
    }
    
    for (uint32_t i_camera = 0; i_camera < cur_fit_->num_cameras_; i_camera++) {
      // Only calculate interpenetration and penalty terms on the first camera 
      // angle to save computation time
      if (i_camera == 0) {
        cur_fit_->model_renderer_->drawDepthMapTiled(coeffs, 
          cur_fit_->coeff_dim_per_model_, cur_fit_->models_, cur_fit_->num_models_, 
          i_camera, &interpenetration_term, 
          cur_fit_->models_[0]->max_bsphere_groups());
      } else {
        cur_fit_->model_renderer_->drawDepthMapTiled(coeffs, 
          cur_fit_->coeff_dim_per_model_, cur_fit_->models_, cur_fit_->num_models_, 
          i_camera, NULL, cur_fit_->models_[0]->max_bsphere_groups());
      }
      if (i_camera == 0) {
        calculateResidualTiled(&depth_term, &penalty_term, coeffs, i_camera);
      } else {
        calculateResidualTiled(&depth_term, NULL, coeffs, i_camera);
      }
    }
    func_eval_count_ += NTILES_DEFAULT;

    // Now calculate the final residue term
    residues.resize(0);
    for (uint32_t i = 0; i < coeffs.size(); i++) {
      residues.pushBack(depth_term[i] * penalty_term[i] * 
        interpenetration_term[i]);
    }
  }

}  // namespace hand_fit
