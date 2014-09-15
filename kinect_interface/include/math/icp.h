//
//  icp.h
//
//  Created by Jonathan Tompson on 4/24/13.  With help from Murphy Stein.
//
//  A modified implementation of the ICP algorithm.  Correspondance 
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
//  THIS ICP CLASS IS MOSTLY THREAD SAFE, but is serializes all match calls
//

#pragma once

#include <random>
#include <mutex>
#include "math/math_types.h"
#include "data_str/vector.h"

#if defined(WIN32) || defined(_WIN32)
  #define constexpr 
#endif

#define ICP_DEFAULT_VERBOSE true
#define ICP_DEFAULT_ITERATIONS 30
#define ICP_DEFAULT_COS_NORMAL_THRESHOLD 0.5f  // 60 deg
#define ICP_DEFAULT_MIN_DISTANCE_SQ 1e-6f
#define ICP_DEFAULT_MAX_DISTANCE_SQ 1e+9f
#define ICP_DEFAULT_METHOD BFGS_ICP
#define ICP_DEFAULT_CUR_MATCH_SCALE true

#define ICP_BFGS_MAX_ITERATIONS 20
#define ICP_BFGS_DESCENT_CONDITIONS jtil::math::SufficientDescentCondition::ARMIJO
#define ICP_BFGS_JAC_2NORM_TERM 1e-11
#define ICP_BFGS_DELTA_F_TERM 1e-11
#define ICP_BFGS_DELTA_X_2NORM_TERM 1e-11
#define ICP_BFGS_OBJ_FUNC_SCALE 1e3
#define ICP_BFGS_C1 1e-4
#define ICP_BFGS_JAC_STEP_SIZE 1e-5
#define ICP_PSO_MAX_ITERATIONS 100
#define ICP_PSO_DELTA_X_TERM 1e-3
#define ICP_DEFAULT_FROBENIUS_NORM_TERM 1e-6

// #define ICP_LINEAR_WEIGHT_FUNCTION  // 1 / (1 + d), otherwise 1 / (1 + d^2)
namespace jtil {
namespace math {

  template <class T>
  struct ICPEigenData;
  template <class T>
  class BFGS;
  template <class T>
  class PSO;

  typedef enum {
    SVD_ICP = 0,  // Cannot adjust scale
    BFGS_ICP,     // Can adjust scale independantly in all directions
    PSO_ICP,      // Can adjust scale independantly in all directions, DEFAULT
    UMEYAMA_ICP,  // Cannot adjust scale
    NUM_METHODS,
  } ICPMethod;

  typedef enum {
    ICP_POS_X = 0,
    ICP_POS_Y = 1,
    ICP_POS_Z = 2,
    ICP_ORIENT_X = 3,
    ICP_ORIENT_Y = 4,
    ICP_ORIENT_Z = 5,
    ICP_SCALE_X = 6, 
    ICP_SCALE_Y = 7,
    ICP_SCALE_Z = 8,
    ICP_OPT_NUM_COEFFS = 9
  } ICPOPTCoeffs;

  template <typename T>
  class ICP {
  public:
    ICP();
    ~ICP();

    // pc1 will remain static and ICP will match pc2 ONTO pc1
    // return is the correspondance fit error.
    T match(Mat4x4<T>& ret_pc1_pc2, const T* pc1, const uint32_t len_pc1, 
      const T* pc2, const uint32_t len_pc2, const Mat4x4<T>& guess_pc1_pc2,
      const T* norm_pc1 = NULL, const T* norm_pc2 = NULL);

    // Adjustment variables
    bool verbose;
    uint32_t num_iterations;
    T cos_normal_threshold;
    T min_distance_sq;  // Avoids correspondances with 0 distance causing 
                        // numerical issues
    T max_distance_sq;  // Beyond this distance, correspondance weights are
                        // forced to zero
    ICPMethod icp_method; 
    T frobenius_norm_termination;  // ICP termination condition.  Terminate 
                                   // when the affine transform stops changing.
    bool match_scale;  // PSO and BFGS will adjust scale. DEFAULT = TRUE

    // Some functions for getting at correspondance data (after match())
    // NONE of these functions are thread safe.
    T* getLastPC2Transformed() { return &pc2_transformed_[0]; }
    int* getLastCorrespondances() { return &matches_[0]; }
    T* getLastWeights() { return &weights_[0]; }
    jtil::data_str::Vector<jtil::math::Mat4x4<T>>& getTransforms() { return transforms_; }
    jtil::data_str::Vector<T>& getPC2Transformed() { return pc2_transformed_; }
    jtil::data_str::Vector<T>& getNormPC2Transformed() { return norm_pc2_transformed_; }

    static double testJacobFunc();
    
  private:
    std::mutex lck_;
    ICPEigenData<T>* edata_;  // To avoid exposing Eigen to the outside world!
    BFGS<double>* bfgs_;
    PSO<double>* pso_;
    // Static variables for running ICP:
    // ICP class is not thread safe because of these!
    static jtil::data_str::Vector<double> cur_Q_;
    static jtil::data_str::Vector<double> cur_D_;
    static jtil::data_str::Vector<double> cur_weights_;
    // cur_translation_scale_ is the largest aabbox dimension
    static double cur_translation_scale_;
    static bool cur_match_scale_;
    static Mat4x4<double> cur_mat_;

    T calcICPMat(Mat4x4<T>& ret, const T* pc1, const T* norm_pc1,
      const uint32_t len_pc1, const T* pc2, const T* norm_pc2, 
      const uint32_t len_pc2);
    void transformPC(T* pc_dst, T* norm_pc_dst, const Mat4x4<T>& mat, 
      const T* pc_src, const T* norm_pc_src, const uint32_t len_pc);

    static double optObjFunc(const double* coeff);
    static void optUpdateFunc(double* coeff);
    static void optCoeffsToMat(Mat4x4<double>& mat, const double* coeff);
    static bool opt_angle_coeffs_[ICP_OPT_NUM_COEFFS];
    static void optJacobFunc(double* jacob, const double* coeff);
    static void optJacobFuncFiniteDiff(double* jacob, const double* coeff);
    static void optJacobFuncPnt(double* j, const double* c, const double* pc1,
    const double* pc2, const double weight1, const double scale);

    jtil::data_str::Vector<jtil::math::Mat4x4<T>> transforms_;
    jtil::data_str::Vector<int> matches_;
    jtil::data_str::Vector<T> weights_;
    jtil::data_str::Vector<T> pc2_transformed_;
    jtil::data_str::Vector<T> norm_pc2_transformed_;
  };

};  // namespace math
};  // namespace jtil
