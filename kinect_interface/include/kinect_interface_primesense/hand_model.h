//
//  hand_model.h
//
//  Created by Jonathan Tompson on 5/22/13.
//
//  A dense modeled hand mesh (linear blend skinning) --> this version is for
//  my deferred rendering engine, and doesn't include any of the PSO related 
//  stuff from HandGeometryMesh.  It's a simple wrapper in order to update
//  from a coeff input.
//

#pragma once

#include "kinect_interface_primesense/hand_model_coeff.h"
#include "math/math_types.h"
#include "data_str/vector.h"
#include "data_str/vector_managed.h"
#include "data_str/pair.h"

#define HN_SRC_IM_SIZE 384  // U, V size (before downsampling)
#define HN_IM_SIZE 96  // Size after downsampling
#define HN_NOM_DIST 500  // Downsample is exactly HN_SRC_IM_SIZE:HN_IM_SIZE at this depth
#define HN_HAND_SIZE 300.0f
#define HN_DEFAULT_NUM_CONV_BANKS 3
#define HN_HPF_GAIN 2.0f 
#define HN_RECT_KERNEL_SIZE 9  // Clemont recommends 5x5 (aggressive), must be odd --> Was 11
#define HN_CONTRAST_NORM_THRESHOLD 5e-2f  // Was 2e-2f
#define HN_LOCAL_CONTRAST_NORM  // Otherwise subtractive local, divisive global --> Was undefined
#define HN_NUM_WORKER_THREADS 6
#define FEATURE_SIZE 3  // UV = 2, UVD = 3
#define NUM_FEATS_PER_FINGER 2
#define NUM_FEATS_PER_THUMB 3
#define NUM_FEATS_PER_PALM 3
#define NUM_COEFFS_PER_GAUSSIAN 5  // (mean_u, mean_v, std_u, std_v)
#define X_DIM_LM_FIT 2
#define BFGS_FINGER_NUM_COEFF 3
#define RAD_UVD_SEARCH 2
#define HN_PSO_RAD_FINGERS 0.30f  // Search radius in frac of min - max coeff
#define HN_PSO_RAD_THUMB 0.20f
#define HN_PSO_RAD_EULER 0.20f
#define HN_PSO_RAD_POSITION 2.0f * (float)M_PI * (5.0f / 100.0f)
#define HN_PSO_SWARM_SIZE 32
// #define HN_PSO_NUM_ITERATIONS 50 --> This now an input parameter 

namespace jtil { namespace renderer { class GeometryInstance; } }
namespace jtil { namespace renderer { class Geometry; } }
namespace jtil { namespace renderer { namespace objects { class BSphere; } } }

namespace kinect_interface_primesense {
namespace hand_net {

  const uint32_t num_convnet_feats = 14;
  const uint32_t convnet_sphere_indices[num_convnet_feats] =  {PALM_3, 
    PALM_1, PALM_2, TH_KNU3_A, TH_KNU3_B, TH_KNU2_B, F1_KNU3_A, F1_KNU2_B, 
    F2_KNU3_A, F2_KNU2_B, F3_KNU3_A, F3_KNU2_B, F4_KNU3_A, F4_KNU2_B};

  typedef enum {
    // HAND_POS1: Base hand position --> (0,0,0) in the palm coordinate system
    HAND_POS1_U = 0,   HAND_POS1_V = 1,  HAND_POS1_D = 2,   // PALM_3 (bounding sph)
    HAND_POS2_U = 3,   HAND_POS2_V = 4,  HAND_POS2_D = 5,   // PALM_1
    HAND_POS3_U = 6,   HAND_POS3_V = 7,  HAND_POS3_D = 8,   // PALM_2
    // Thumb
    THUMB_TIP_U = 9,   THUMB_TIP_V = 10, THUMB_TIP_D = 11,  // TH_KNU3_A
    THUMB_MID_U = 12,  THUMB_MID_V = 13, THUMB_MID_D = 14,  // TH_KNU3_B
    THUMB_KNU_U = 15,  THUMB_KNU_V = 16, THUMB_KNU_D = 17,  // TH_KNU2_B
    // F0
    F0_TIP_U = 18,    F0_TIP_V = 19,   F0_TIP_D = 20,     // F1_KNU3_A
    F0_KNU_U = 21,    F0_KNU_V = 22,   F0_KNU_D = 23,     // F1_KNU2_B
    // F1
    F1_TIP_U = 24,    F1_TIP_V = 25,   F1_TIP_D = 26,     // F2_KNU3_A
    F1_KNU_U = 27,    F1_KNU_V = 28,   F1_KNU_D = 29,     // F2_KNU2_B
    // F2
    F2_TIP_U = 30,    F2_TIP_V = 31,   F2_TIP_D = 32,     // F3_KNU3_A
    F2_KNU_U = 33,    F2_KNU_V = 34,   F2_KNU_D = 35,     // F3_KNU2_B
    // F3
    F3_TIP_U = 36,    F3_TIP_V = 37,   F3_TIP_D = 38,     // F4_KNU3_A
    F3_KNU_U = 39,    F3_KNU_V = 40,   F3_KNU_D = 41,     // F4_KNU2_B
    HAND_NUM_COEFF_CONVNET = 42, 
  } HandCoeffConvnet;

  const uint32_t num_uvd_feats = NUM_BOUNDING_SPHERES;
  const uint32_t HAND_NUM_COEFF_UVD = NUM_BOUNDING_SPHERES * 3;

  class HandModel {
  public:
    static const bool* angle_coeffs() { return angle_coeffs_; }
    static const float* coeff_min_limit() { return coeff_min_limit_; }
    static const float* coeff_min_limit_conservative() { return coeff_min_limit_conservative_; }
    static const float* coeff_max_limit() { return coeff_max_limit_; }
    static const float* coeff_max_limit_conservative() { return coeff_max_limit_conservative_; }
    static const float* coeff_penalty_scale() { return coeff_penalty_scale_; }
    static const uint32_t max_bsphere_groups() { return 6; }
    static const uint32_t num_bspheres_per_group() { return 6; }

  private:

    static const float coeff_min_limit_[HAND_NUM_COEFF];
    static const float coeff_max_limit_[HAND_NUM_COEFF];
    static const float coeff_max_limit_conservative_[HAND_NUM_COEFF];
    static const float coeff_min_limit_conservative_[HAND_NUM_COEFF];
    static const float coeff_penalty_scale_[HAND_NUM_COEFF];
    static const bool angle_coeffs_[HAND_NUM_COEFF];
  };
};  // namespace hand_net
};  // namespace kinect_interface_primesense
