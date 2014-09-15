#include <random>
#include <stdexcept>
#include <iostream>
#include "math/icp.h"
#include <Eigen/Eigen>
#include <Eigen/Geometry>
#include "math/icp_eigen_data.h"
#include <nanoflann/nanoflann.hpp>
#include "math/bfgs.h"
#include "math/pso.h"
#include "math/math_base.h"

#define SAFE_DELETE(x) if (x != NULL) { delete x; x = NULL; }
#define SAFE_DELETE_ARR(x) if (x != NULL) { delete[] x; x = NULL; }

using Eigen::MatrixXf;
using std::cout;
using std::endl;
using std::runtime_error;
using namespace nanoflann;

// nanoflann needs a DataSet Adaptor class to calculate distances
template <typename T> 
struct PointCloud {
  PointCloud(T* points, const uint32_t npoints) : points(points), 
    npoints(npoints) { }
  T* points;
  uint32_t npoints;

	// Must return the number of data points
	inline size_t kdtree_get_point_count() const { return npoints; }

	// Returns the distance between the vector "p1[0:size-1]" and the data point
  // with index "idx_p2" stored in the class:
  inline T kdtree_distance(const T* p1, const size_t idx_p2, size_t size) const {
    const T d0 = p1[0] - points[idx_p2 * 3 + 0];
    const T d1 = p1[1] - points[idx_p2 * 3 + 1];
    const T d2 = p1[2] - points[idx_p2 * 3 + 2];
    return d0 * d0 + d1 * d1 + d2 * d2;  // L2 squared
  }

	// Returns the dim'th component of the idx'th point in the class
	inline T kdtree_get_pt(const size_t idx, int dim) const {
    return points[idx * 3 + dim];
	}

	// Optional bounding-box computation: return false to default to a standard
  // bbox computation loop.
	template <class BBOX>
	bool kdtree_get_bbox(BBOX &bb) const { return false; }
};

namespace jtil {
namespace math {

  template <typename T> jtil::data_str::Vector<double> ICP<T>::cur_Q_;
  template <typename T> jtil::data_str::Vector<double> ICP<T>::cur_D_;
  template <typename T> jtil::data_str::Vector<double> ICP<T>::cur_weights_;
  template <typename T> double ICP<T>::cur_translation_scale_;
  template <typename T> Mat4x4<double> ICP<T>::cur_mat_;
  template <typename T> bool ICP<T>::cur_match_scale_;

  template <typename T>
  ICP<T>::ICP() {
    bfgs_ = NULL;
    edata_ = NULL;
    pso_ = NULL;

    match_scale = ICP_DEFAULT_CUR_MATCH_SCALE;
    edata_ = new ICPEigenData<T>();
    bfgs_ = new BFGS<double>(ICP_OPT_NUM_COEFFS);
    pso_ = new PSO<double>(ICP_OPT_NUM_COEFFS);
    verbose = ICP_DEFAULT_VERBOSE;
    num_iterations = ICP_DEFAULT_ITERATIONS;
    cos_normal_threshold = (T)ICP_DEFAULT_COS_NORMAL_THRESHOLD;
    min_distance_sq = (T)ICP_DEFAULT_MIN_DISTANCE_SQ;
    max_distance_sq = (T)ICP_DEFAULT_MAX_DISTANCE_SQ;
    icp_method = ICP_DEFAULT_METHOD;
    frobenius_norm_termination = (T)ICP_DEFAULT_FROBENIUS_NORM_TERM;
  }

  template <typename T>
  T ICP<T>::match(Mat4x4<T>& ret_pc1_pc2, const T* pc1, 
    const uint32_t len_pc1, const T* pc2, const uint32_t len_pc2, 
    const Mat4x4<T>& guess_pc1_pc2, const T* norm_pc1, 
    const T* norm_pc2) {
    std::lock_guard<std::mutex> lock(lck_);

    // Resize data structures if necessary
    if (transforms_.capacity() < num_iterations) {
      transforms_.capacity(num_iterations);
    }
    transforms_.resize(0);

    if (matches_.capacity() < len_pc2) {
      matches_.capacity(len_pc2);
    }
    matches_.resize(len_pc2);

    if (weights_.capacity() < len_pc2) {
      weights_.capacity(len_pc2);
    }
    weights_.resize(len_pc2);

    if (pc2_transformed_.capacity() / 3 < len_pc2) {
      pc2_transformed_.capacity(len_pc2 * 3);
    }
    pc2_transformed_.resize(len_pc2 * 3);

    if (norm_pc2 != NULL) {
      if (norm_pc2_transformed_.capacity() / 3 < len_pc2) {
        norm_pc2_transformed_.capacity(len_pc2 * 3);
      }
      norm_pc2_transformed_.resize(len_pc2 * 3);
    } else {
      norm_pc2_transformed_.resize(0);
    }
    
    transforms_.pushBack(guess_pc1_pc2);
    Mat4x4<T> cur_icp_mat;
    Mat4x4<T> new_composite_mat;
    Mat4x4<T> I;
    I.identity();
    T fit_error = std::numeric_limits<T>::infinity();
    for (uint32_t i = 0; i < num_iterations; i++) {
      if (verbose) {
        std::cout << "ICP iteration " << (i + 1) << " of " << num_iterations;
        std::cout << std::endl;
      }
      transformPC(&pc2_transformed_[0], &norm_pc2_transformed_[0], 
        transforms_[i], pc2, norm_pc2, len_pc2);
      if (norm_pc1 != NULL) {
        fit_error = calcICPMat(cur_icp_mat, pc1, norm_pc1, len_pc1, 
          &pc2_transformed_[0], &norm_pc2_transformed_[0], len_pc2);
      } else {
        fit_error = calcICPMat(cur_icp_mat, pc1, NULL, len_pc1, 
          &pc2_transformed_[0], NULL, len_pc2);
      }

      Mat4x4<T> delta_M;
      Mat4x4<T>::sub(delta_M, cur_icp_mat, I);
      T frobenius_norm = delta_M.frobeniusNorm();  // ||M - I||_F
      if (frobenius_norm < frobenius_norm_termination) {
        break;  // Early termination
      }

      Mat4x4<T>::mult(new_composite_mat, cur_icp_mat, transforms_[i]);
      transforms_.pushBack(new_composite_mat);

    }
    ret_pc1_pc2.set(transforms_[transforms_.size() - 1]);
    // Finally, transform the points in case someone wants to use them
    transformPC(&pc2_transformed_[0], &norm_pc2_transformed_[0], 
        ret_pc1_pc2, pc2, norm_pc2, len_pc2);
    return fit_error;
  }

  template <typename T>
  T ICP<T>::calcICPMat(Mat4x4<T>& ret, const T* D, const T* norm_D,
    const uint32_t len_D, const T* Q, const T* norm_Q, 
    const uint32_t len_Q) {
    // Note D is pc1 in the top level code, and Q is pc2

    // Zero out the matches and weights
    for (uint32_t i = 0; i < len_Q; i++) {
      matches_[i] = -1;
      weights_[i] = 1.0f;
    }

    // Construct a KD tree using pc2
    // Flann library expects features to be row-major (one point on each row)
    // luckily this is exactly how we have it.
    if (verbose) {
      std::cout << "    Finding correspondances..." << std::endl;
    }
    const int dim = 3;
    const int nn = 1;  // Num nearest neighbours to search for

    PointCloud<T> pc1(const_cast<T*>(D), len_D);
    PointCloud<T> pc2(const_cast<T*>(Q), len_Q);

    // construct a kd-tree index:
	  typedef KDTreeSingleIndexAdaptor<
		  L2_Simple_Adaptor<T, PointCloud<T> > ,
		  PointCloud<T>,
		  3 /* dim */
		  > my_kd_tree_t;

    // TODO: Leaf size should scale with PC1 size 
    my_kd_tree_t index(3 /*dim*/, pc1, 
      KDTreeSingleIndexAdaptorParams(10 /* max leaf */) );  
	  index.buildIndex();

    const size_t num_results = 1;
    T fit_error = 0;
    for (uint32_t i = 0; i < len_Q; i++) {
      size_t closest_index;
      T closest_distance_sq;
      index.knnSearch(&Q[i*3], num_results, &closest_index, 
        &closest_distance_sq);
      matches_[i] = static_cast<int>(closest_index);
      weights_[i] = static_cast<T>(closest_distance_sq);
      fit_error += sqrt(closest_distance_sq);
    }
    fit_error = fit_error / len_Q;

    /*
    // This is the old FLANN code.  I've since switched to nanoflann to make 
    // the code more portable (nanoflann is just a header library).
    flann::Matrix<T> dataset(const_cast<T*>(D), len_D, dim);
    flann::Matrix<T> query(const_cast<T*>(Q), len_Q, dim);

    flann::Matrix<int> indices(&matches_[0], len_Q, nn);
    flann::Matrix<T> dists(&weights_[0], len_Q, nn);

    // construct an randomized kd-tree index using 4 kd-trees
    // Note: L2 is the squared distances
    flann::Index<flann::L2<T>> index(dataset, flann::KDTreeIndexParams(4));
    index.buildIndex();

    // do a knn search, using 128 checks
    // For each point in the query, what is its closest neightbour in the dataset
    //flann::SearchParams sp = flann::SearchParams(128);
    flann::SearchParams sp = flann::SearchParams(32);
    sp.cores = ICP_NUM_CORES;  // Must be const in order for OpenMP to compile
                               // Multithreaded code!
    index.knnSearch(query, indices, dists, nn, sp);
    */

    // Compute the weights per match
    if (verbose) {
      std::cout << "    Calculating correspondance weights..." << std::endl;
    }

#if defined(DEBUG) || defined(_DEBUG)
    // Validate the FLANN min match
    uint32_t i_min = 0;
    T w_min = weights_[0];
    for (uint32_t i = 0; i < len_Q; i++) {
      if (weights_[i] < w_min) {
        w_min = weights_[i];
        i_min = i;
      }
    }
    Vec3<T> Q_min(&Q[i_min * 3]);
    Vec3<T> D_min(&D[matches_[i_min] * 3]);
    Vec3<T> diff;
    Vec3<T>::sub(diff, Q_min, D_min);
    T weight = Vec3<T>::dot(diff, diff);
    if (fabs(weight - w_min) > LOOSE_EPSILON) {
      std::cout << "WARNING! manually calculated weight doesn't match Flann's" << std::endl;
      std::cout << weight << " vs " << w_min << std::endl;
    }
    for (uint32_t i = 0; i < len_D; i++) {
      Vec3<T> D_val(&D[i * 3]);
      Vec3<T> diff;
      Vec3<T>::sub(diff, Q_min, D_val);
      weight = Vec3<T>::dot(diff, diff);
      if (weight < w_min) {
        std::cout << "WARNING! manually calculated weight is lower than Flann's" << std::endl;
        std::cout << weight << " vs " << w_min << std::endl;
      }
    }
#endif

    // weights_[i] is the squared distance of the correspondance
    T min_distance = sqrt(min_distance_sq);
    for (uint32_t i = 0; i < weights_.size(); i++) {
      if (weights_[i] >= max_distance_sq) {
        weights_[i] = 0.0f;
      } else {
        T angle_adjustment = 1.0f;
        if (norm_Q != NULL) {
          Vec3<T> cur_norm_D(&(norm_D[matches_[i] * 3]));
          Vec3<T> cur_norm_Q(&(norm_Q[i * 3]));
          T cos_angle = Vec3<T>::dot(cur_norm_D, cur_norm_Q);
          angle_adjustment = std::max<T>(0, 
            (cos_angle - cos_normal_threshold) / ((T)1.0 - cos_normal_threshold));
        }
#ifdef ICP_LINEAR_WEIGHT_FUNCTION
        weights_[i] = std::max<T>(sqrt(weights_[i]), min_distance);
#else
        weights_[i] = std::max<T>(weights_[i], min_distance_sq);
#endif
        weights_[i] = angle_adjustment / ((T)1.0 + weights_[i]);
      }
    }

    // Compute total weight and normalize the weights
    T total_weight = 0.0f;
    for (uint32_t i = 0; i < weights_.size(); i++) {
      total_weight += weights_[i];
    }
    for (uint32_t i = 0; i < weights_.size(); i++) {
      weights_[i] /= total_weight;
    }

    switch (icp_method) {
    case SVD_ICP:
      {
        // Compute Mean of both sets
        if (verbose) {
          std::cout << "    Computing point cloud means..." << std::endl;
        }
        Vec3<T> D_mean, Q_mean;
        D_mean.zeros();
        Q_mean.zeros();
        uint32_t n_pts = 0;
        for (uint32_t i = 0; i < weights_.size(); i++) {
          if (weights_[i] > std::numeric_limits<T>::epsilon()) {
            // Always true, but lets be explicit, just in case this changes
            n_pts++;
            Q_mean[0] += Q[i * 3] * weights_[i];
            Q_mean[1] += Q[i * 3 + 1] * weights_[i];
            Q_mean[2] += Q[i * 3 + 2] * weights_[i];
            uint32_t j = matches_[i];
            D_mean[0] += D[j * 3] * weights_[i];
            D_mean[1] += D[j * 3 + 1] * weights_[i];
            D_mean[2] += D[j * 3 + 2] * weights_[i];
          }
        }
        // No need to normalize since the weights have already been scaled

        // Calculate Cross-Covariance matrix
        if (verbose) {
          std::cout << "    Calculating Cross-Covariance Matrix..." << std::endl;
        }
        Vec3<T> D_pt, Q_pt;
        Mat3x3<T> cross_cov, outer_prod;
        for (uint32_t i = 0; i < weights_.size(); i++) {
          if (weights_[i] > std::numeric_limits<T>::epsilon()) {
            Q_pt.set(&Q[i * 3]);
            D_pt.set(&D[matches_[i] * 3]);
            Vec3<T>::outerProd(outer_prod, Q_pt, D_pt);
            Mat3x3<T>::scale(outer_prod, weights_[i]);
            Mat3x3<T>::add(cross_cov, cross_cov, outer_prod);
          }
        }
        Vec3<T>::outerProd(outer_prod, Q_mean, D_mean);
        Mat3x3<T>::sub(cross_cov, cross_cov, outer_prod);

        // Compute SVD using Eigen
        if (verbose) {
          std::cout << "    Performing SVD..." << std::endl;
        }
        for (uint32_t i = 0; i < 3; i++) {
          for (uint32_t j = 0; j < 3; j++) {
            edata_->cross_cov_mat_(i, j) = cross_cov(i, j);
          }
        }
        Eigen::JacobiSVD<Eigen::Matrix<T,3,3>> svd(edata_->cross_cov_mat_, 
          Eigen::ComputeFullU | Eigen::ComputeFullV);
        edata_->rot_e_mat_ = svd.matrixU() * svd.matrixV().transpose();
        if (edata_->rot_e_mat_.determinant() < 0) {
          Eigen::Matrix<T,3,3> D = Eigen::Matrix<T,3,3>::Identity();
          D(2,2) = -1;
          edata_->rot_e_mat_ = svd.matrixU() * D * svd.matrixV().transpose();
          cout << "    fixing reflection" << endl;
        }
        Mat4x4<T> new_rot;
        new_rot.identity();
        for (int i = 0; i < 3; i++) {
          for (int j = 0; j < 3; j++) {
            new_rot(i, j) = edata_->rot_e_mat_(i,j);
          }
        }

        Mat4x4<T> T1;
        Mat4x4<T>::translationMat(T1, -Q_mean[0], -Q_mean[1], -Q_mean[2]);
        Mat4x4<T> T2;
        Mat4x4<T>::inverse(T2, new_rot);
        Mat4x4<T> T3;
        Mat4x4<T>::translationMat(T3, D_mean[0], D_mean[1], D_mean[2]);

        Mat4x4<T> temp;
        Mat4x4<T>::mult(temp, T2, T1);
        Mat4x4<T>::mult(ret, T3, temp);
      }
      break;
    case UMEYAMA_ICP:
      {
        if (verbose) {
          std::cout << "    Calculating Umeyama method matrix..." << std::endl;
        }
        // Perform Umeyama method
        uint32_t n_pts = 0;
        for (uint32_t i = 0; i < weights_.size(); i++) {
          if (weights_[i] > std::numeric_limits<T>::epsilon()) {
            n_pts++;
          }
        }
        if (n_pts <= 3) {
          if (verbose) {
            std::cout << "WARNING: less than 3 correspondance points found. "
              "Terminating early... (try changing mix/max distance or cos "
              "normal threshold)";
          }
          ret.identity();
          return fit_error;
        }

        Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> X;  // Point set X is brought onto Y
        Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> Y;
        X.resize(3, n_pts);  // Each column is a point
        Y.resize(3, n_pts);

        // Fill up the Eigen structure
        uint32_t dst_ind = 0;
        for (uint32_t i = 0; i < matches_.size(); i++) {
          if (weights_[i] > std::numeric_limits<T>::epsilon()) {
            X.col(dst_ind) <<  Q[i * 3], Q[i * 3 + 1], Q[i * 3 + 2];
            uint32_t j = matches_[i];
            Y.col(dst_ind) << D[j * 3], D[j * 3 + 1], D[j * 3 + 2];
            dst_ind++;
          }
        }
        Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> mat = 
          Eigen::umeyama(X, Y, true);

        for (int i = 0; i < 4; i++) {
          for (int j = 0; j < 4; j++) {
            ret(i, j) = mat(i,j);
          }
        }
        break;
      }
    case PSO_ICP:
    case BFGS_ICP:
      {
        if (verbose) {
          std::cout << "    Calculating BFGS method matrix..." << std::endl;
        }

        // Collect the non-zero weighted correspondances and convert them to
        // double for numerical stability:
        uint32_t n_pts = 0;
        for (uint32_t i = 0; i < matches_.size(); i++) {
          if (weights_[i] > std::numeric_limits<T>::epsilon()) {
            n_pts++;
          }
        }
        if (n_pts <= 3) {
          if (verbose) {
            std::cout << "WARNING: less than 3 correspondance points found. "
              "Terminating early... (try changing mix/max distance or cos "
              "normal threshold)";
          }
          ret.identity();
          return fit_error;
        }

        if (cur_Q_.capacity() < n_pts * 3) {
          cur_Q_.capacity(n_pts * 3);
        }
        cur_Q_.resize(n_pts * 3);
        if (cur_D_.capacity() < n_pts * 3) {
          cur_D_.capacity(n_pts * 3);
        }
        cur_D_.resize(n_pts * 3);
        if (cur_weights_.capacity() < n_pts) {
          cur_weights_.capacity(n_pts);
        }
        cur_weights_.resize(n_pts);
        uint32_t ind = 0;
        Vec3<double> max_pc1(0, 0, 0);
        Vec3<double> min_pc1(0, 0, 0);
        for (uint32_t i = 0; i < matches_.size(); i++) {
          if (weights_[i] > std::numeric_limits<T>::epsilon()) {
            uint32_t j = matches_[i];
            cur_D_[ind * 3] = D[j * 3];
            cur_D_[ind * 3 + 1] = D[j * 3 + 1];
            cur_D_[ind * 3 + 2] = D[j * 3 + 2];
            Vec3<double>::max(max_pc1, max_pc1, Vec3<double>(D[j*3], D[j*3+1], 
              D[j*3+2]));
            Vec3<double>::min(min_pc1, min_pc1, Vec3<double>(D[j*3], D[j*3+1], 
              D[j*3+2]));
            cur_Q_[ind * 3] = Q[i * 3];
            cur_Q_[ind * 3 + 1] = Q[i * 3 + 1];
            cur_Q_[ind * 3 + 2] = Q[i * 3 + 2];
            cur_weights_[ind] = weights_[i];
            ind++;
          } 
        }
        // cur_translation_scale_ is the largest aabbox dimension
        cur_translation_scale_ = std::max<double>(max_pc1[0]-min_pc1[0],
          std::max<double>(max_pc1[1]-min_pc1[1], max_pc1[2]-min_pc1[2]));
        cur_translation_scale_ = std::max<double>(cur_translation_scale_, 1e-4);
        cur_match_scale_ = match_scale;

        // The starting coeffs should result in the identity matrix
        double start_coeff[ICPOPTCoeffs::ICP_OPT_NUM_COEFFS];
        for (uint32_t i = ICP_POS_X; i <= ICP_ORIENT_Z; i++) {
          start_coeff[i] = 0.0;
        }
        for (uint32_t i = ICP_SCALE_X; i <= ICP_SCALE_Z; i++) {
          start_coeff[i] = 1.0;
        }

        double end_coeff[ICPOPTCoeffs::ICP_OPT_NUM_COEFFS];
        if (icp_method == BFGS_ICP) {
          // Solve the problem using BFGS
          bfgs_->verbose = verbose;
          bfgs_->c1 = (T)ICP_BFGS_C1;
          bfgs_->descent_cond = ICP_BFGS_DESCENT_CONDITIONS;
          bfgs_->max_iterations = ICP_BFGS_MAX_ITERATIONS;
          bfgs_->jac_2norm_term = (T)ICP_BFGS_JAC_2NORM_TERM;
          bfgs_->delta_f_term = (T)ICP_BFGS_DELTA_F_TERM;
          bfgs_->delta_x_2norm_term = (T)ICP_BFGS_DELTA_X_2NORM_TERM;
          bfgs_->minimize(end_coeff, start_coeff, opt_angle_coeffs_, 
            optObjFunc, optJacobFunc, optUpdateFunc);
        } else {
          // Solve the problem using PSO
          double coeff_rad[ICPOPTCoeffs::ICP_OPT_NUM_COEFFS];
          for (uint32_t i = ICP_POS_X; i <= ICP_POS_Z; i++) {
            coeff_rad[i] = cur_translation_scale_;
          }
          for (uint32_t i = ICP_ORIENT_X; i <= ICP_ORIENT_Z; i++) {
            coeff_rad[i] = M_PI_2;
          }
          for (uint32_t i = ICP_SCALE_X; i <= ICP_SCALE_Z; i++) {
            coeff_rad[i] = 0.2;
          }

          pso_->verbose = verbose;
          pso_->max_iterations = ICP_PSO_MAX_ITERATIONS;
          pso_->delta_coeff_termination = ICP_PSO_DELTA_X_TERM;
          pso_->minimize(end_coeff, start_coeff, coeff_rad, opt_angle_coeffs_, 
            optObjFunc, optUpdateFunc);
        }
        Mat4x4<double> opt_ret;
        optCoeffsToMat(opt_ret, end_coeff);
        for (uint32_t i = 0; i < 16; i++) {
          ret[i] = (float)opt_ret[i];
        }
        if (verbose) {
          std::cout << "    Optimization complete using pos, euler, and scale"
            " coeffs:";
          std::cout << std::endl;
          for (uint32_t i = 0; i < ICP_OPT_NUM_COEFFS; i++) {
            std::cout << end_coeff[i] << ", ";
          }
          std::cout << std::endl;
        }
      }
      break;
    default:
      throw std::runtime_error("ICP::match() - ERROR: ICPMethod is not "
        "recognized!");
    }
    return fit_error;
  }

  template <typename T>
  void ICP<T>::transformPC(T* pc_dst, T* norm_pc_dst, const Mat4x4<T>& mat, 
    const T* pc_src, const T* norm_pc_src, const uint32_t len_pc) {
    if (verbose) {
      std::cout << "    Transforming point cloud..." << std::endl;
    }
    Vec3<T> pt;
    Vec3<T> pt_transformed;
    for (uint32_t i = 0; i < len_pc; i++) {
      pt.set(&pc_src[i*3]);
      Vec3<T>::affineTransformPos(pt_transformed, mat, pt);
      pc_dst[i * 3] = pt_transformed[0];
      pc_dst[i * 3 + 1] = pt_transformed[1];
      pc_dst[i * 3 + 2] = pt_transformed[2];
    }

    // Now transform the normals
    if (norm_pc_src != NULL) {
      // In general ICP deals with only translation and rotation matricies,
      // so a full normal matrix is not really required.  However, since the
      // current matrix is seeded by an user-supplied transform we cannot 
      // assume that it is even affine, let alone only trans + rotation
      Mat4x4<T> normal_mat;
      Mat4x4<T>::inverse(normal_mat, mat);
      normal_mat.transpose();
      Vec3<T> norm;
      Vec3<T> norm_transformed;
      for (uint32_t i = 0; i < len_pc; i++) {
        norm.set(&norm_pc_src[i*3]);
        Vec3<T>::affineTransformVec(norm_transformed, normal_mat, norm);
        norm_transformed.normalize();
        norm_pc_dst[i * 3] = norm_transformed[0];
        norm_pc_dst[i * 3 + 1] = norm_transformed[1];
        norm_pc_dst[i * 3 + 2] = norm_transformed[2];
      }
    }
  }

  template <typename T>
  bool ICP<T>::opt_angle_coeffs_[ICP_OPT_NUM_COEFFS] = {
    false,  // ICP_POSX
    false,  // ICP_POSY
    false,  // ICP_POSZ
    true,  // ICP_ORIENT_X
    true,  // ICP_ORIENT_Y
    true,  // ICP_ORIENT_Z
    false,  // ICP_SCALE_X
    false,  // ICP_SCALE_Y
    false,  // ICP_SCALE_Z
  };

  template <typename T>
  void ICP<T>::optCoeffsToMat(Mat4x4<double>& mat, const double* coeff) {
    Mat4x4<double>::euler2RotMat(mat, coeff[ICP_ORIENT_X] * (2.0 * M_PI), 
      coeff[ICP_ORIENT_Y] * (2.0 * M_PI), coeff[ICP_ORIENT_Z] * (2.0 * M_PI));
    if (cur_match_scale_) {
      mat.rightMultScale(coeff[ICP_SCALE_X], coeff[ICP_SCALE_Y], 
        coeff[ICP_SCALE_Z]);
    }

    // We want the translation coefficients to be between 0 and 1.  We will do
    // this by using the maximum PC1 AABBOX dimension as a scale factor
    mat.leftMultTranslation(coeff[ICP_POS_X] * cur_translation_scale_, 
      coeff[ICP_POS_Y] * cur_translation_scale_, 
      coeff[ICP_POS_Z] * cur_translation_scale_);
  }

  template <typename T>
  double ICP<T>::optObjFunc(const double* coeff) {
    optCoeffsToMat(cur_mat_, coeff);
    // Now calculate:
    // 1/N * Sum_i ( weight_i * ||D_i - mat * Q_i||_2 ) / mean_i(weight_i)
    
    // Now calculate the final objective function value
    Vec3<double> D_pt, Q_pt, Q_pt_transformed, delta_x;
    double sum = 0.0;
    double scale = 1.0 / (((double)cur_D_.size()) / 3.0);
    for (uint32_t i = 0; i < cur_D_.size() / 3; i++) {
      D_pt.set(cur_D_[i * 3], cur_D_[i * 3 + 1], cur_D_[i * 3 + 2]);
      Q_pt.set(cur_Q_[i * 3], cur_Q_[i * 3 + 1], cur_Q_[i * 3 + 2]);
      Vec3<double>::affineTransformPos(Q_pt_transformed, cur_mat_, Q_pt);
      Vec3<double>::sub(delta_x, D_pt, Q_pt_transformed);
      sum += scale * cur_weights_[i] * Vec3<double>::dot(delta_x, delta_x);
    }

    // To make the objective function a reasonable value, apply a constant
    // scale term
    sum = ICP_BFGS_OBJ_FUNC_SCALE * sum;

    return sum;
  }

  // Jacobian for a single point, see icp_jacobian_calc.m
  // I'm going to assume that the optimizer can do a better job cleaning this
  // up than I can.  Note that there is a verification test using finite
  // differences.
  template <typename T>
  void ICP<T>::optJacobFuncPnt(double* j, const double* c, const double* pc1,
    const double* pc2, const double weight1, const double scale) {
    const double c1 = c[0];
    const double c2 = c[1];
    const double c3 = c[2];
    const double c4 = c[3];
    const double c5 = c[4];
    const double c6 = c[5];
    const double c7 = c[6];
    const double c8 = c[7];
    const double c9 = c[8];
    const double pc1_1 = pc1[0];
    const double pc1_2 = pc1[1];
    const double pc1_3 = pc1[2];
    const double pc2_1 = pc2[0];
    const double pc2_2 = pc2[1];
    const double pc2_3 = pc2[2];
    // The following is generated automatically by icp_jacobian_calc.m
    j[0] += cur_translation_scale_*scale*weight1*(-pc1_1+c1*cur_translation_scale_+c8*pc2_2*(sin(M_PI*c4*2.0)*sin(M_PI*c6*2.0)-cos(M_PI*c4*2.0)*cos(M_PI*c6*2.0)*sin(M_PI*c5*2.0))+c9*pc2_3*(cos(M_PI*c6*2.0)*sin(M_PI*c4*2.0)+cos(M_PI*c4*2.0)*sin(M_PI*c5*2.0)*sin(M_PI*c6*2.0))+c7*pc2_1*cos(M_PI*c4*2.0)*cos(M_PI*c5*2.0))*2.0;
    j[1] += cur_translation_scale_*scale*weight1*(-pc1_2+c2*cur_translation_scale_+c7*pc2_1*sin(M_PI*c5*2.0)+c8*pc2_2*cos(M_PI*c5*2.0)*cos(M_PI*c6*2.0)-c9*pc2_3*cos(M_PI*c5*2.0)*sin(M_PI*c6*2.0))*2.0;
    j[2] += cur_translation_scale_*scale*weight1*(-pc1_3+c3*cur_translation_scale_+c8*pc2_2*(cos(M_PI*c4*2.0)*sin(M_PI*c6*2.0)+cos(M_PI*c6*2.0)*sin(M_PI*c4*2.0)*sin(M_PI*c5*2.0))+c9*pc2_3*(cos(M_PI*c4*2.0)*cos(M_PI*c6*2.0)-sin(M_PI*c4*2.0)*sin(M_PI*c5*2.0)*sin(M_PI*c6*2.0))-c7*pc2_1*cos(M_PI*c5*2.0)*sin(M_PI*c4*2.0))*2.0;
    j[3] += scale*weight1*((c8*pc2_2*(M_PI*cos(M_PI*c4*2.0)*sin(M_PI*c6*2.0)*2.0+M_PI*cos(M_PI*c6*2.0)*sin(M_PI*c4*2.0)*sin(M_PI*c5*2.0)*2.0)+c9*pc2_3*(M_PI*cos(M_PI*c4*2.0)*cos(M_PI*c6*2.0)*2.0-M_PI*sin(M_PI*c4*2.0)*sin(M_PI*c5*2.0)*sin(M_PI*c6*2.0)*2.0)-M_PI*c7*pc2_1*cos(M_PI*c5*2.0)*sin(M_PI*c4*2.0)*2.0)*(-pc1_1+c1*cur_translation_scale_+c8*pc2_2*(sin(M_PI*c4*2.0)*sin(M_PI*c6*2.0)-cos(M_PI*c4*2.0)*cos(M_PI*c6*2.0)*sin(M_PI*c5*2.0))+c9*pc2_3*(cos(M_PI*c6*2.0)*sin(M_PI*c4*2.0)+cos(M_PI*c4*2.0)*sin(M_PI*c5*2.0)*sin(M_PI*c6*2.0))+c7*pc2_1*cos(M_PI*c4*2.0)*cos(M_PI*c5*2.0))*2.0-(c8*pc2_2*(M_PI*sin(M_PI*c4*2.0)*sin(M_PI*c6*2.0)*2.0-M_PI*cos(M_PI*c4*2.0)*cos(M_PI*c6*2.0)*sin(M_PI*c5*2.0)*2.0)+c9*pc2_3*(M_PI*cos(M_PI*c6*2.0)*sin(M_PI*c4*2.0)*2.0+M_PI*cos(M_PI*c4*2.0)*sin(M_PI*c5*2.0)*sin(M_PI*c6*2.0)*2.0)+M_PI*c7*pc2_1*cos(M_PI*c4*2.0)*cos(M_PI*c5*2.0)*2.0)*(-pc1_3+c3*cur_translation_scale_+c8*pc2_2*(cos(M_PI*c4*2.0)*sin(M_PI*c6*2.0)+cos(M_PI*c6*2.0)*sin(M_PI*c4*2.0)*sin(M_PI*c5*2.0))+c9*pc2_3*(cos(M_PI*c4*2.0)*cos(M_PI*c6*2.0)-sin(M_PI*c4*2.0)*sin(M_PI*c5*2.0)*sin(M_PI*c6*2.0))-c7*pc2_1*cos(M_PI*c5*2.0)*sin(M_PI*c4*2.0))*2.0);
    j[4] += scale*weight1*((M_PI*c7*pc2_1*sin(M_PI*c4*2.0)*sin(M_PI*c5*2.0)*2.0+M_PI*c8*pc2_2*cos(M_PI*c5*2.0)*cos(M_PI*c6*2.0)*sin(M_PI*c4*2.0)*2.0-M_PI*c9*pc2_3*cos(M_PI*c5*2.0)*sin(M_PI*c4*2.0)*sin(M_PI*c6*2.0)*2.0)*(-pc1_3+c3*cur_translation_scale_+c8*pc2_2*(cos(M_PI*c4*2.0)*sin(M_PI*c6*2.0)+cos(M_PI*c6*2.0)*sin(M_PI*c4*2.0)*sin(M_PI*c5*2.0))+c9*pc2_3*(cos(M_PI*c4*2.0)*cos(M_PI*c6*2.0)-sin(M_PI*c4*2.0)*sin(M_PI*c5*2.0)*sin(M_PI*c6*2.0))-c7*pc2_1*cos(M_PI*c5*2.0)*sin(M_PI*c4*2.0))*2.0-(M_PI*c7*pc2_1*cos(M_PI*c4*2.0)*sin(M_PI*c5*2.0)*2.0+M_PI*c8*pc2_2*cos(M_PI*c4*2.0)*cos(M_PI*c5*2.0)*cos(M_PI*c6*2.0)*2.0-M_PI*c9*pc2_3*cos(M_PI*c4*2.0)*cos(M_PI*c5*2.0)*sin(M_PI*c6*2.0)*2.0)*(-pc1_1+c1*cur_translation_scale_+c8*pc2_2*(sin(M_PI*c4*2.0)*sin(M_PI*c6*2.0)-cos(M_PI*c4*2.0)*cos(M_PI*c6*2.0)*sin(M_PI*c5*2.0))+c9*pc2_3*(cos(M_PI*c6*2.0)*sin(M_PI*c4*2.0)+cos(M_PI*c4*2.0)*sin(M_PI*c5*2.0)*sin(M_PI*c6*2.0))+c7*pc2_1*cos(M_PI*c4*2.0)*cos(M_PI*c5*2.0))*2.0+(M_PI*c7*pc2_1*cos(M_PI*c5*2.0)*2.0-M_PI*c8*pc2_2*cos(M_PI*c6*2.0)*sin(M_PI*c5*2.0)*2.0+M_PI*c9*pc2_3*sin(M_PI*c5*2.0)*sin(M_PI*c6*2.0)*2.0)*(-pc1_2+c2*cur_translation_scale_+c7*pc2_1*sin(M_PI*c5*2.0)+c8*pc2_2*cos(M_PI*c5*2.0)*cos(M_PI*c6*2.0)-c9*pc2_3*cos(M_PI*c5*2.0)*sin(M_PI*c6*2.0))*2.0);
    j[5] += scale*weight1*((c8*pc2_2*(M_PI*cos(M_PI*c6*2.0)*sin(M_PI*c4*2.0)*2.0+M_PI*cos(M_PI*c4*2.0)*sin(M_PI*c5*2.0)*sin(M_PI*c6*2.0)*2.0)-c9*pc2_3*(M_PI*sin(M_PI*c4*2.0)*sin(M_PI*c6*2.0)*2.0-M_PI*cos(M_PI*c4*2.0)*cos(M_PI*c6*2.0)*sin(M_PI*c5*2.0)*2.0))*(-pc1_1+c1*cur_translation_scale_+c8*pc2_2*(sin(M_PI*c4*2.0)*sin(M_PI*c6*2.0)-cos(M_PI*c4*2.0)*cos(M_PI*c6*2.0)*sin(M_PI*c5*2.0))+c9*pc2_3*(cos(M_PI*c6*2.0)*sin(M_PI*c4*2.0)+cos(M_PI*c4*2.0)*sin(M_PI*c5*2.0)*sin(M_PI*c6*2.0))+c7*pc2_1*cos(M_PI*c4*2.0)*cos(M_PI*c5*2.0))*2.0+(c8*pc2_2*(M_PI*cos(M_PI*c4*2.0)*cos(M_PI*c6*2.0)*2.0-M_PI*sin(M_PI*c4*2.0)*sin(M_PI*c5*2.0)*sin(M_PI*c6*2.0)*2.0)-c9*pc2_3*(M_PI*cos(M_PI*c4*2.0)*sin(M_PI*c6*2.0)*2.0+M_PI*cos(M_PI*c6*2.0)*sin(M_PI*c4*2.0)*sin(M_PI*c5*2.0)*2.0))*(-pc1_3+c3*cur_translation_scale_+c8*pc2_2*(cos(M_PI*c4*2.0)*sin(M_PI*c6*2.0)+cos(M_PI*c6*2.0)*sin(M_PI*c4*2.0)*sin(M_PI*c5*2.0))+c9*pc2_3*(cos(M_PI*c4*2.0)*cos(M_PI*c6*2.0)-sin(M_PI*c4*2.0)*sin(M_PI*c5*2.0)*sin(M_PI*c6*2.0))-c7*pc2_1*cos(M_PI*c5*2.0)*sin(M_PI*c4*2.0))*2.0-(M_PI*c9*pc2_3*cos(M_PI*c5*2.0)*cos(M_PI*c6*2.0)*2.0+M_PI*c8*pc2_2*cos(M_PI*c5*2.0)*sin(M_PI*c6*2.0)*2.0)*(-pc1_2+c2*cur_translation_scale_+c7*pc2_1*sin(M_PI*c5*2.0)+c8*pc2_2*cos(M_PI*c5*2.0)*cos(M_PI*c6*2.0)-c9*pc2_3*cos(M_PI*c5*2.0)*sin(M_PI*c6*2.0))*2.0);
    if (cur_match_scale_) {
      j[6] += scale*weight1*(pc2_1*sin(M_PI*c5*2.0)*(-pc1_2+c2*cur_translation_scale_+c7*pc2_1*sin(M_PI*c5*2.0)+c8*pc2_2*cos(M_PI*c5*2.0)*cos(M_PI*c6*2.0)-c9*pc2_3*cos(M_PI*c5*2.0)*sin(M_PI*c6*2.0))*2.0+pc2_1*cos(M_PI*c4*2.0)*cos(M_PI*c5*2.0)*(-pc1_1+c1*cur_translation_scale_+c8*pc2_2*(sin(M_PI*c4*2.0)*sin(M_PI*c6*2.0)-cos(M_PI*c4*2.0)*cos(M_PI*c6*2.0)*sin(M_PI*c5*2.0))+c9*pc2_3*(cos(M_PI*c6*2.0)*sin(M_PI*c4*2.0)+cos(M_PI*c4*2.0)*sin(M_PI*c5*2.0)*sin(M_PI*c6*2.0))+c7*pc2_1*cos(M_PI*c4*2.0)*cos(M_PI*c5*2.0))*2.0-pc2_1*cos(M_PI*c5*2.0)*sin(M_PI*c4*2.0)*(-pc1_3+c3*cur_translation_scale_+c8*pc2_2*(cos(M_PI*c4*2.0)*sin(M_PI*c6*2.0)+cos(M_PI*c6*2.0)*sin(M_PI*c4*2.0)*sin(M_PI*c5*2.0))+c9*pc2_3*(cos(M_PI*c4*2.0)*cos(M_PI*c6*2.0)-sin(M_PI*c4*2.0)*sin(M_PI*c5*2.0)*sin(M_PI*c6*2.0))-c7*pc2_1*cos(M_PI*c5*2.0)*sin(M_PI*c4*2.0))*2.0);
      j[7] += scale*weight1*(pc2_2*(sin(M_PI*c4*2.0)*sin(M_PI*c6*2.0)-cos(M_PI*c4*2.0)*cos(M_PI*c6*2.0)*sin(M_PI*c5*2.0))*(-pc1_1+c1*cur_translation_scale_+c8*pc2_2*(sin(M_PI*c4*2.0)*sin(M_PI*c6*2.0)-cos(M_PI*c4*2.0)*cos(M_PI*c6*2.0)*sin(M_PI*c5*2.0))+c9*pc2_3*(cos(M_PI*c6*2.0)*sin(M_PI*c4*2.0)+cos(M_PI*c4*2.0)*sin(M_PI*c5*2.0)*sin(M_PI*c6*2.0))+c7*pc2_1*cos(M_PI*c4*2.0)*cos(M_PI*c5*2.0))*2.0+pc2_2*(cos(M_PI*c4*2.0)*sin(M_PI*c6*2.0)+cos(M_PI*c6*2.0)*sin(M_PI*c4*2.0)*sin(M_PI*c5*2.0))*(-pc1_3+c3*cur_translation_scale_+c8*pc2_2*(cos(M_PI*c4*2.0)*sin(M_PI*c6*2.0)+cos(M_PI*c6*2.0)*sin(M_PI*c4*2.0)*sin(M_PI*c5*2.0))+c9*pc2_3*(cos(M_PI*c4*2.0)*cos(M_PI*c6*2.0)-sin(M_PI*c4*2.0)*sin(M_PI*c5*2.0)*sin(M_PI*c6*2.0))-c7*pc2_1*cos(M_PI*c5*2.0)*sin(M_PI*c4*2.0))*2.0+pc2_2*cos(M_PI*c5*2.0)*cos(M_PI*c6*2.0)*(-pc1_2+c2*cur_translation_scale_+c7*pc2_1*sin(M_PI*c5*2.0)+c8*pc2_2*cos(M_PI*c5*2.0)*cos(M_PI*c6*2.0)-c9*pc2_3*cos(M_PI*c5*2.0)*sin(M_PI*c6*2.0))*2.0);
      j[8] += scale*weight1*(pc2_3*(cos(M_PI*c6*2.0)*sin(M_PI*c4*2.0)+cos(M_PI*c4*2.0)*sin(M_PI*c5*2.0)*sin(M_PI*c6*2.0))*(-pc1_1+c1*cur_translation_scale_+c8*pc2_2*(sin(M_PI*c4*2.0)*sin(M_PI*c6*2.0)-cos(M_PI*c4*2.0)*cos(M_PI*c6*2.0)*sin(M_PI*c5*2.0))+c9*pc2_3*(cos(M_PI*c6*2.0)*sin(M_PI*c4*2.0)+cos(M_PI*c4*2.0)*sin(M_PI*c5*2.0)*sin(M_PI*c6*2.0))+c7*pc2_1*cos(M_PI*c4*2.0)*cos(M_PI*c5*2.0))*2.0+pc2_3*(cos(M_PI*c4*2.0)*cos(M_PI*c6*2.0)-sin(M_PI*c4*2.0)*sin(M_PI*c5*2.0)*sin(M_PI*c6*2.0))*(-pc1_3+c3*cur_translation_scale_+c8*pc2_2*(cos(M_PI*c4*2.0)*sin(M_PI*c6*2.0)+cos(M_PI*c6*2.0)*sin(M_PI*c4*2.0)*sin(M_PI*c5*2.0))+c9*pc2_3*(cos(M_PI*c4*2.0)*cos(M_PI*c6*2.0)-sin(M_PI*c4*2.0)*sin(M_PI*c5*2.0)*sin(M_PI*c6*2.0))-c7*pc2_1*cos(M_PI*c5*2.0)*sin(M_PI*c4*2.0))*2.0-pc2_3*cos(M_PI*c5*2.0)*sin(M_PI*c6*2.0)*(-pc1_2+c2*cur_translation_scale_+c7*pc2_1*sin(M_PI*c5*2.0)+c8*pc2_2*cos(M_PI*c5*2.0)*cos(M_PI*c6*2.0)-c9*pc2_3*cos(M_PI*c5*2.0)*sin(M_PI*c6*2.0))*2.0);
    }
  }

  template <typename T>
  void ICP<T>::optJacobFunc(double* jacob, const double* coeff) {
    double scale = 1.0 / (((double)cur_D_.size()) / 3.0);
    for (uint32_t i = 0; i < ICP_OPT_NUM_COEFFS; i++) {
      jacob[i] = 0;  // Zero the jacobian
    }
    for (uint32_t i = 0; i < cur_D_.size() / 3; i++) {
      // Accumulate the current derivative WRT the correspondence pair
      optJacobFuncPnt(jacob, coeff, &cur_D_[i * 3], &cur_Q_[i * 3], 
        cur_weights_[i], scale * ICP_BFGS_OBJ_FUNC_SCALE);
    }
  }

  template <typename T>
  void ICP<T>::optJacobFuncFiniteDiff(double* jacob, const double* coeff) {
    double coeff_tmp[ICP_OPT_NUM_COEFFS];
    // Estimate using central diff.
    // http://math.fullerton.edu/mathews/n2003/differentiation/NumericalDiffProof.pdf
    memcpy(coeff_tmp, coeff, sizeof(coeff_tmp[0]) * ICP_OPT_NUM_COEFFS);
    const double h = ICP_BFGS_JAC_STEP_SIZE;
    for (uint32_t i = 0; i < ICP_OPT_NUM_COEFFS; i++) {
      coeff_tmp[i] = coeff[i] - h;
      double f0 = optObjFunc(coeff_tmp);
      coeff_tmp[i] = coeff[i] + h;
      double f1 = optObjFunc(coeff_tmp);
      coeff_tmp[i] = coeff[i];
      jacob[i] = (f1 - f0) / (2.0 * h);
    }
  }

  template <typename T>
  void ICP<T>::optUpdateFunc(double* coeff) {
    WrapTwoPI(coeff[ICP_ORIENT_X]);
    WrapTwoPI(coeff[ICP_ORIENT_Y]);
    WrapTwoPI(coeff[ICP_ORIENT_Z]);
    if (cur_match_scale_) {
      coeff[ICP_SCALE_X] = fabs(coeff[ICP_SCALE_X]);
      coeff[ICP_SCALE_Y] = fabs(coeff[ICP_SCALE_Y]);
      coeff[ICP_SCALE_Z] = fabs(coeff[ICP_SCALE_Z]);
    }
  }

  template <typename T>
  ICP<T>::~ICP() {
    SAFE_DELETE(edata_);
    SAFE_DELETE(bfgs_);
    SAFE_DELETE(pso_);
  }

  // Test the closed form jacobian using the finite difference jacobian
  template <typename T>
  double ICP<T>::testJacobFunc() {
    MERSINE_TWISTER_ENG eng;
    eng.seed(1000);
    std::tr1::uniform_real_distribution<double> dist(-0.99, +0.99);
    std::tr1::uniform_int_distribution<uint32_t> npts_dist(3, 10);

    const uint32_t num_point_clouds = 10;
    const uint32_t num_evals = 1000;
    double max_err = 0;
    for (uint32_t pc = 0; pc < num_point_clouds; pc++) {
      // Generate random point clouds and random correspondance weights
      uint32_t num_points = npts_dist(eng);
      if (cur_Q_.capacity() < num_points * 3) {
        cur_Q_.capacity(num_points * 3);
      }
      cur_Q_.resize(num_points * 3);
      if (cur_D_.capacity() < num_points * 3) {
        cur_D_.capacity(num_points * 3);
      }
      cur_D_.resize(num_points * 3);
      if (cur_weights_.capacity() < num_points) {
        cur_weights_.capacity(num_points);
      }
      cur_weights_.resize(num_points);
      for (uint32_t i = 0; i < 3*num_points; i++) {
        cur_D_[i] = dist(eng);
        cur_Q_[i] = dist(eng);
      }
      for (uint32_t i = 0; i < num_points; i++) {
        cur_weights_[i] = dist(eng);
      }
      cur_translation_scale_ = dist(eng);
      cur_match_scale_ = true;

      double coeff[ICPOPTCoeffs::ICP_OPT_NUM_COEFFS];
      double j[ICPOPTCoeffs::ICP_OPT_NUM_COEFFS];
      double j_finite[ICPOPTCoeffs::ICP_OPT_NUM_COEFFS];
      for (uint32_t c = 0; c < num_evals; c++) {
        // Evaluate the jacobian a few times at different c values
        for (uint32_t i = 0; i < ICP_OPT_NUM_COEFFS; i++) {
          coeff[i] = dist(eng);
        }
        optUpdateFunc(coeff);

        // Calculate the closed form jacobian
        optJacobFunc(j, coeff);

        // Calculate the finite difference jacobian
        optJacobFuncFiniteDiff(j_finite, coeff);

        // Calculate the squared L2 distance
        double cur_err = 0;
        for (uint32_t i = 0; i < ICP_OPT_NUM_COEFFS; i++) {
          cur_err += (j[i] - j_finite[i])*(j[i] - j_finite[i]);
        }

        max_err = std::max<double>(max_err, cur_err);
      }
    }
    return max_err;
  }

}  // namespace math
}  // namespace jtil

// Explicit template instantiation
template class jtil::math::ICP<float>;
template class jtil::math::ICP<double>;