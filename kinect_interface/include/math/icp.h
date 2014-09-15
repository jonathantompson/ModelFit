//
//  icp.h
//
//  Created by Jonathan Tompson on 4/24/13.  With help from Murphy Stein.
//
//  A vanilla implementation of the ICP algorithm.  Correspondance 
//  search is done using the Flann header library: 
//  http://www.cs.ubc.ca/~mariusm/index.php/FLANN/FLANN
//
//  Takes in two lists of point clouds plus an intial guess for the orientation
//  of pc1 onto pc2, and returns the final rigid transformation matrix.
//
//  If the point cloud normals are avaliable then you should include them.
//
//  The Umeyama method will solve for a constant scale (using
//  Eigen's built in Umeyama method), rather than the SVD ICP method:
//  http://eigen.tuxfamily.org/dox/group__Geometry__Module.html
//  However, weighting per correspondance is effectively 1, so it might not 
//  work well in instances where the initial alignment disparity is high. 
//
//  BFGS is probably your best best if you don't know what you're doing, 
//  however it will fit to noise, so try and keep the number of correspondances
//  high.
//
//  THIS ICP CLASS IS NOT THREAD SAFE!
//

#pragma once

#include <random>
#include "math/math_types.h"
#include "data_str/vector.h"

#if defined(WIN32) || defined(_WIN32)
  #define constexpr 
#endif

#define ICP_DEFAULT_VERBOSE true
#define ICP_DEFAULT_ITERATIONS 30
#define ICP_DEFAULT_COS_NORMAL_THRESHOLD 0.5f  // 60 deg
#define ICP_NUM_CORES 8
#define ICP_DEFAULT_MIN_DISTANCE_SQ 0.000001f  // 1e-6
#define ICP_DEFAULT_MAX_DISTANCE_SQ 1000000000.0f  // 1e9
#define ICP_DEFAULT_METHOD BFGS_ICP

#define ICP_BFGS_INCLUDE_SCALE
#define ICP_LINEAR_WEIGHT_FUNCTION  // 1 / (1 + d), otherwise 1 / (1 + d^2)
namespace jtil {
namespace math {

  struct ICPEigenData;
  template <class T>
  class BFGS;

  typedef enum {
    SVD_ICP,  // Cannot adjust scale
    BFGS_ICP,  // Can adjust scale independantly in all directions
    UMEYAMA_ICP,  // Cannot 
  } ICPMethod;

  typedef enum {
    ICP_POS_X = 0,
    ICP_POS_Y = 1,
    ICP_POS_Z = 2,
    ICP_ORIENT_X = 3,
    ICP_ORIENT_Y = 4,
    ICP_ORIENT_Z = 5,
#ifdef ICP_BFGS_INCLUDE_SCALE
    ICP_SCALE_X = 6, 
    ICP_SCALE_Y = 7,
    ICP_SCALE_Z = 8,
    ICP_BFGS_NUM_COEFFS = 9
#else
    ICP_BFGS_NUM_COEFFS = 6
#endif
  } ICPBFGSCoeffs;

  // BFGS with backtracking optimization
  class ICP {
  public:
    ICP();
    ~ICP();

    // pc1 will remain static and ICP will match pc2 ONTO pc1
    void match(Float4x4& ret_pc1_pc2, const float* pc1, const uint32_t len_pc1, 
      const float* pc2, const uint32_t len_pc2, const Float4x4& guess_pc1_pc2,
      const float* norm_pc1 = NULL, const float* norm_pc2 = NULL);

    // Adjustment variables
    bool verbose;
    uint32_t num_iterations;
    float cos_normal_threshold;
    float min_distance_sq;  // Avoids correspondances with 0 distance causing 
                            // numerical issues
    float max_distance_sq;  // Beyond this distance, correspondance weights are
                            // forced to zero
    ICPMethod icp_method; 

    // Some functions for getting at correspondance data (after match())
    float* getLastPC2Transformed() { return &pc2_transformed_[0]; }
    int* getLastCorrespondances() { return &matches_[0]; }
    float* getLastWeights() { return &weights_[0]; }
    jtil::data_str::Vector<jtil::math::Float4x4>& getTransforms() { return transforms_; }
    
  private:
    ICPEigenData* edata_;  // To avoid exposing Eigen to the outside world!
    BFGS<double>* bfgs_;
    // Static variables for running ICP:
    // ICP class is not thread safe because of these!
    static jtil::data_str::Vector<double> cur_Q_;
    static jtil::data_str::Vector<double> cur_D_;
    static jtil::data_str::Vector<double> cur_weights_;
    static ICP* cur_icp_;
    static Double4x4 cur_mat_;

    void calcICPMat(Float4x4& ret, const float* pc1, const float* norm_pc1,
      const uint32_t len_pc1, const float* pc2, const float* norm_pc2, 
      const uint32_t len_pc2);
    void transformPC(float* pc_dst, float* norm_pc_dst, const Float4x4& mat, 
      const float* pc_src, const float* norm_pc_src, const uint32_t len_pc);

    static double bfgsObjFunc(const double* coeff);
    static void bfgsJacobFunc(double* jacob, const double* coeff);
    static void bfgsUpdateFunc(double* coeff);
    static void bfgsCoeffsToMat(Double4x4& mat, const double* coeff);
    static bool bfgs_angle_coeffs_[ICP_BFGS_NUM_COEFFS];

    jtil::data_str::Vector<jtil::math::Float4x4> transforms_;
    jtil::data_str::Vector<int> matches_;
    jtil::data_str::Vector<float> weights_;
    jtil::data_str::Vector<float> pc2_transformed_;
    jtil::data_str::Vector<float> norm_pc2_transformed_;
  };

};  // namespace math
};  // namespace jtil
