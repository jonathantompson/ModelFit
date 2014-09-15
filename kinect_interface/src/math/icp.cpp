#include <random>
#include <stdexcept>
#include <iostream>
#include "math/icp.h"
#include <Eigen/Eigen>
#include <Eigen/Geometry>
#include "math/icp_eigen_data.h"
#include <nanoflann/nanoflann.hpp>
#include "math/bfgs.h"
#include "math/math_base.h"

#define SAFE_DELETE(x) if (x != NULL) { delete x; x = NULL; }
#define SAFE_DELETE_ARR(x) if (x != NULL) { delete[] x; x = NULL; }

using Eigen::MatrixXf;
using std::cout;
using std::endl;
using std::runtime_error;

namespace jtil {
namespace math {

  jtil::data_str::Vector<double> ICP::cur_Q_;
  jtil::data_str::Vector<double> ICP::cur_D_;
  jtil::data_str::Vector<double> ICP::cur_weights_;
  ICP* ICP::cur_icp_;
  Double4x4 ICP::cur_mat_;

  ICP::ICP() {
    bfgs_ = NULL;
    edata_ = NULL;

    edata_ = new ICPEigenData();
    bfgs_ = new BFGS<double>(ICP_BFGS_NUM_COEFFS);
    verbose = ICP_DEFAULT_VERBOSE;
    num_iterations = ICP_DEFAULT_ITERATIONS;
    cos_normal_threshold = ICP_DEFAULT_COS_NORMAL_THRESHOLD;
    min_distance_sq = ICP_DEFAULT_MIN_DISTANCE_SQ;
    max_distance_sq = ICP_DEFAULT_MAX_DISTANCE_SQ;
    icp_method = ICP_DEFAULT_METHOD;
  }

  void ICP::match(Float4x4& ret_pc1_pc2, const float* pc1, 
    const uint32_t len_pc1, const float* pc2, const uint32_t len_pc2, 
    const Float4x4& guess_pc1_pc2, const float* norm_pc1, 
    const float* norm_pc2) {

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
    }
    
    transforms_.pushBack(guess_pc1_pc2);
    Float4x4 cur_icp_mat;
    Float4x4 new_composite_mat;
    for (uint32_t i = 0; i < num_iterations; i++) {
      if (verbose) {
        std::cout << "ICP iteration " << (i + 1) << " of " << num_iterations;
        std::cout << std::endl;
      }
      transformPC(&pc2_transformed_[0], &norm_pc2_transformed_[0], transforms_[i], 
        pc2, norm_pc2, len_pc2);
      calcICPMat(cur_icp_mat, pc1, norm_pc1, len_pc1, &pc2_transformed_[0], 
        &norm_pc2_transformed_[0], len_pc2);

      Float4x4::mult(new_composite_mat, cur_icp_mat, transforms_[i]);
      transforms_.pushBack(new_composite_mat);
    }
    ret_pc1_pc2.set(transforms_[transforms_.size() - 1]);
  }

  void ICP::calcICPMat(Float4x4& ret, const float* D, const float* norm_D,
    const uint32_t len_D, const float* Q, const float* norm_Q, 
    const uint32_t len_Q) {
    // Note D is pc1 in the top level code, and Q is pc2

    // Zero out the matches and weights
    for (uint32_t i = 0; i < len_Q; i++) {
      matches_[i] = -1;
      weights_[i] = 1.0f;
    }

    throw std::runtime_error("NANOFLANN SUPPORT MIGRATION NOT FINISHED!");

#ifdef NANOFLAN_MIGRATION_FINISHED
    // Construct a KD tree using pc2
    // Flann library expects features to be row-major (one point on each row)
    // luckily this is exactly how we have it.
    if (verbose) {
      std::cout << "    Finding correspondances..." << std::endl;
    }
    const int dim = 3;
    const int nn = 1;  // Num nearest neighbours to search for
    flann::Matrix<float> dataset(const_cast<float*>(D), len_D, dim);
    flann::Matrix<float> query(const_cast<float*>(Q), len_Q, dim);

    flann::Matrix<int> indices(&matches_[0], len_Q, nn);
    flann::Matrix<float> dists(&weights_[0], len_Q, nn);

    // construct an randomized kd-tree index using 4 kd-trees
    // Note: L2 is the squared distances
    flann::Index<flann::L2<float>> index(dataset, flann::KDTreeIndexParams(4));
    index.buildIndex();

    // do a knn search, using 128 checks
    // For each point in the query, what is its closest neightbour in the dataset
    //flann::SearchParams sp = flann::SearchParams(128);
    flann::SearchParams sp = flann::SearchParams(32);
    sp.cores = ICP_NUM_CORES;  // Must be const in order for OpenMP to compile
                               // Multithreaded code!
    index.knnSearch(query, indices, dists, nn, sp);

    // Compute the weights per match
    if (verbose) {
      std::cout << "    Calculating correspondance weights..." << std::endl;
    }
#endif

#if defined(DEBUG) || defined(_DEBUG)
    // Validate the FLANN min match
    uint32_t i_min = 0;
    float w_min = weights_[0];
    for (uint32_t i = 0; i < len_Q; i++) {
      if (weights_[i] < w_min) {
        w_min = weights_[i];
        i_min = i;
      }
    }
    Float3 Q_min(&Q[i_min * 3]);
    Float3 D_min(&D[matches_[i_min] * 3]);
    Float3 diff;
    Float3::sub(diff, Q_min, D_min);
    float weight = Float3::dot(diff, diff);
    if (fabsf(weight - w_min) > LOOSE_EPSILON) {
      std::cout << "WARNING! manually calculated weight doesn't match Flann's" << std::endl;
      std::cout << weight << " vs " << w_min << std::endl;
    }
    for (uint32_t i = 0; i < len_D; i++) {
      Float3 D_val(&D[i * 3]);
      Float3 diff;
      Float3::sub(diff, Q_min, D_val);
      weight = Float3::dot(diff, diff);
      if (weight < w_min) {
        std::cout << "WARNING! manually calculated weight is lower than Flann's" << std::endl;
        std::cout << weight << " vs " << w_min << std::endl;
      }
    }
#endif
    // weights_[i] is the squared distance of the correspondance
    float min_distance = sqrtf(min_distance_sq);
    for (uint32_t i = 0; i < weights_.size(); i++) {
      if (weights_[i] >= max_distance_sq) {
        weights_[i] = 0.0f;
      } else {
        float angle_adjustment = 1.0f;
        if (norm_Q != NULL) {
          Float3 cur_norm_D(&(norm_D[matches_[i] * 3]));
          Float3 cur_norm_Q(&(norm_Q[i * 3]));
          float cos_angle = Float3::dot(cur_norm_D, cur_norm_Q);
          angle_adjustment = std::max<float>(0, 
            (cos_angle - cos_normal_threshold) / (1 - cos_normal_threshold));
        }
#ifdef ICP_LINEAR_WEIGHT_FUNCTION
        weights_[i] = std::max<float>(sqrtf(weights_[i]), min_distance);
#else
        weights_[i] = std::max<float>(weights_[i], min_distance_sq);
#endif
        weights_[i] = angle_adjustment / (1.0f + weights_[i]);
      }
    }

    // Compute total weight and scale the weights
    float total_weight = 0.0f;
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
        Float3 D_mean, Q_mean;
        D_mean.zeros();
        Q_mean.zeros();
        uint32_t n_pts = 0;
        for (uint32_t i = 0; i < weights_.size(); i++) {
          if (weights_[i] > (float)EPSILON) {
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
        Float3 D_pt, Q_pt;
        Float3x3 cross_cov, outer_prod;
        for (uint32_t i = 0; i < weights_.size(); i++) {
          if (weights_[i] > (float)EPSILON) {
            Q_pt.set(&Q[i * 3]);
            D_pt.set(&D[matches_[i] * 3]);
            Float3::outerProd(outer_prod, Q_pt, D_pt);
            Float3x3::scale(outer_prod, weights_[i]);
            Float3x3::add(cross_cov, cross_cov, outer_prod);
          }
        }
        Float3::outerProd(outer_prod, Q_mean, D_mean);
        Float3x3::sub(cross_cov, cross_cov, outer_prod);

        // Compute SVD using Eigen
        if (verbose) {
          std::cout << "    Performing SVD..." << std::endl;
        }
        for (uint32_t i = 0; i < 3; i++) {
          for (uint32_t j = 0; j < 3; j++) {
            edata_->cross_cov_mat_(i, j) = cross_cov(i, j);
          }
        }
        Eigen::JacobiSVD<Eigen::Matrix3f> svd(edata_->cross_cov_mat_, 
          Eigen::ComputeFullU | Eigen::ComputeFullV);
        edata_->rot_e_mat_ = svd.matrixU() * svd.matrixV().transpose();
        if (edata_->rot_e_mat_.determinant() < 0) {
          Eigen::Matrix3f D = Eigen::Matrix3f::Identity();
          D(2,2) = -1;
          edata_->rot_e_mat_ = svd.matrixU() * D * svd.matrixV().transpose();
          cout << "    fixing reflection" << endl;
        }
        Float4x4 new_rot;
        new_rot.identity();
        for (int i = 0; i < 3; i++) {
          for (int j = 0; j < 3; j++) {
            new_rot(i, j) = edata_->rot_e_mat_(i,j);
          }
        }

        Float4x4 T1;
        Float4x4::translationMat(T1, -Q_mean[0], -Q_mean[1], -Q_mean[2]);
        Float4x4 T2;
        Float4x4::inverse(T2, new_rot);
        Float4x4 T3;
        Float4x4::translationMat(T3, D_mean[0], D_mean[1], D_mean[2]);

        Float4x4 temp;
        Float4x4::mult(temp, T2, T1);
        Float4x4::mult(ret, T3, temp);
      }
      break;
    case UMEYAMA_ICP:
      {
        std::cout << "    Calculating Umeyama method matrix..." << std::endl;
        // Perform Umeyama method
        uint32_t n_pts = 0;
        for (uint32_t i = 0; i < weights_.size(); i++) {
          if (weights_[i] > (float)EPSILON) {
            n_pts++;
          }
        }
        if (n_pts <= 3) {
          throw std::runtime_error("ERROR: Not enough correspondance points!");
        }

        Eigen::MatrixXf X;  // Point set X is brought onto Y
        Eigen::MatrixXf Y;
        X.resize(3, n_pts);  // Each column is a point
        Y.resize(3, n_pts);

        // Fill up the Eigen structure
        uint32_t dst_ind = 0;
        for (uint32_t i = 0; i < matches_.size(); i++) {
          if (weights_[i] > (float)EPSILON) {
            X.col(dst_ind) <<  Q[i * 3], Q[i * 3 + 1], Q[i * 3 + 2];
            uint32_t j = matches_[i];
            Y.col(dst_ind) << D[j * 3], D[j * 3 + 1], D[j * 3 + 2];
            dst_ind++;
          }
        }
        Eigen::MatrixXf mat = Eigen::umeyama(X, Y, true);

        for (int i = 0; i < 4; i++) {
          for (int j = 0; j < 4; j++) {
            ret(i, j) = mat(i,j);
          }
        }
        break;
      }
    case BFGS_ICP:
      {
        std::cout << "    Calculating BFGS method matrix..." << std::endl;

        // Collect the non-zero weighted correspondances and convert them to
        // double for numerical stability:
        uint32_t n_pts = 0;
        for (uint32_t i = 0; i < matches_.size(); i++) {
          if (weights_[i] > (float)EPSILON) {
            n_pts++;
          }
        }
        if (n_pts <= 3) {
          throw std::runtime_error("ERROR: Not enough correspondance points ("
            "at least 3 is needed)!");
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
        for (uint32_t i = 0; i < matches_.size(); i++) {
          if (weights_[i] > (float)EPSILON) {
            uint32_t j = matches_[i];
            cur_D_[ind * 3] = (double)D[j * 3];
            cur_D_[ind * 3 + 1] = (double)D[j * 3 + 1];
            cur_D_[ind * 3 + 2] = (double)D[j * 3 + 2];
            cur_Q_[ind * 3] = (double)Q[i * 3];
            cur_Q_[ind * 3 + 1] = (double)Q[i * 3 + 1];
            cur_Q_[ind * 3 + 2] = (double)Q[i * 3 + 2];
            cur_weights_[ind] = (double)weights_[i];
            ind++;
          }
        }
        cur_icp_ = this;

        // The starting coeffs should result in the identity matrix
        double start_coeff[ICPBFGSCoeffs::ICP_BFGS_NUM_COEFFS];
        for (uint32_t i = ICP_POS_X; i <= ICP_ORIENT_Z; i++) {
          start_coeff[i] = 0.0;
        }
#ifdef ICP_BFGS_INCLUDE_SCALE
        for (uint32_t i = ICP_SCALE_X; i <= ICP_SCALE_Z; i++) {
          start_coeff[i] = 2.0 * M_PI;
        }
#endif
        double end_coeff[ICPBFGSCoeffs::ICP_BFGS_NUM_COEFFS];
        bfgs_->verbose = verbose;
        bfgs_->c1 = 1e-8;
        bfgs_->descent_cond = ARMIJO;
        bfgs_->max_iterations = 100;
        bfgs_->jac_2norm_term = 1e-10;
        bfgs_->delta_f_term = 1e-12;
        bfgs_->delta_x_2norm_term = 1e-12;
        bfgs_->minimize(end_coeff, start_coeff, bfgs_angle_coeffs_, 
          bfgsObjFunc, bfgsJacobFunc, bfgsUpdateFunc);
        Double4x4 bfgs_ret;
        bfgsCoeffsToMat(bfgs_ret, end_coeff);
        for (uint32_t i = 0; i < 16; i++) {
          ret[i] = (float)bfgs_ret[i];
        }
        if (verbose) {
          std::cout << "    BFGS complete using pos, euler, and scale coeffs:";
          std::cout << std::endl;
          for (uint32_t i = 0; i < ICP_BFGS_NUM_COEFFS; i++) {
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
  }

  void ICP::transformPC(float* pc_dst, float* norm_pc_dst, const Float4x4& mat, 
    const float* pc_src, const float* norm_pc_src, const uint32_t len_pc) {
    if (verbose) {
      std::cout << "    Transforming point cloud..." << std::endl;
    }
    Float3 pt;
    Float3 pt_transformed;
    for (uint32_t i = 0; i < len_pc; i++) {
      pt.set(&pc_src[i*3]);
      Float3::affineTransformPos(pt_transformed, mat, pt);
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
      Float4x4 normal_mat;
      Float4x4::inverse(normal_mat, mat);
      normal_mat.transpose();
      Float3 norm;
      Float3 norm_transformed;
      for (uint32_t i = 0; i < len_pc; i++) {
        norm.set(&norm_pc_src[i*3]);
        Float3::affineTransformVec(norm_transformed, normal_mat, norm);
        norm_transformed.normalize();
        norm_pc_dst[i * 3] = norm_transformed[0];
        norm_pc_dst[i * 3 + 1] = norm_transformed[1];
        norm_pc_dst[i * 3 + 2] = norm_transformed[2];
      }
    }
  }

  bool ICP::bfgs_angle_coeffs_[ICP_BFGS_NUM_COEFFS] = {
    false,  // ICP_POSX
    false,  // ICP_POSY
    false,  // ICP_POSZ
    true,  // ICP_ORIENT_X
    true,  // ICP_ORIENT_Y
    true,  // ICP_ORIENT_Z
#ifdef ICP_BFGS_INCLUDE_SCALE
    false,  // ICP_SCALE_X
    false,  // ICP_SCALE_Y
    false,  // ICP_SCALE_Z
#endif
  };

  void ICP::bfgsCoeffsToMat(Double4x4& mat, const double* coeff) {
    Double4x4::euler2RotMat(mat, coeff[ICP_ORIENT_X], 
      coeff[ICP_ORIENT_Y], coeff[ICP_ORIENT_Z]);
#ifdef ICP_BFGS_INCLUDE_SCALE
    mat.rightMultScale(coeff[ICP_SCALE_X] / (2.0 * M_PI), 
      coeff[ICP_SCALE_Y] / (2.0 * M_PI), coeff[ICP_SCALE_Z] / (2.0 * M_PI));
#endif
    mat.leftMultTranslation(coeff[ICP_POS_X] * 100.0, 
      coeff[ICP_POS_Y] * 100.0, coeff[ICP_POS_Z] * 100.0);
  }

  double ICP::bfgsObjFunc(const double* coeff) {
    bfgsCoeffsToMat(cur_mat_, coeff);
    // Now calculate:
    // 1/N * Sum_i ( weight_i * ||D_i - mat * Q_i||_2 )
    
    // Now calculate the final objective function value
    Double3 D_pt, Q_pt, Q_pt_transformed, delta;
    double sum = 0.0;
    double scale = 1.0 / (((double)cur_D_.size()) / 3.0);
    for (uint32_t i = 0; i < cur_D_.size() / 3; i++) {
      D_pt.set(cur_D_[i * 3], cur_D_[i * 3 + 1], cur_D_[i * 3 + 2]);
      Q_pt.set(cur_Q_[i * 3], cur_Q_[i * 3 + 1], cur_Q_[i * 3 + 2]);
      Double3::affineTransformPos(Q_pt_transformed, cur_mat_, Q_pt);
      Double3::sub(delta, D_pt, Q_pt_transformed);
      sum += scale * cur_weights_[i] * Double3::dot(delta, delta);
    }

    // Add a penalty term for small scales so that the global solution of 
    // scale = 0, 0, 0, isn't a solution:

#ifdef ICP_BFGS_INCLUDE_SCALE
    sum += coeff[ICP_SCALE_X] <= 0.4 ? (1.0 / coeff[ICP_SCALE_X]) : 0.0;
    sum += coeff[ICP_SCALE_Y] <= 0.4 ? (1.0 / coeff[ICP_SCALE_Y]) : 0.0;
    sum += coeff[ICP_SCALE_Z] <= 0.4 ? (1.0 / coeff[ICP_SCALE_Z]) : 0.0;
#endif

    return sum;
  }

  void ICP::bfgsJacobFunc(double* jacob, const double* coeff) {
    double coeff_tmp[ICP_BFGS_NUM_COEFFS];
    // Estimate using central diff.
    // http://math.fullerton.edu/mathews/n2003/differentiation/NumericalDiffProof.pdf
    memcpy(coeff_tmp, coeff, sizeof(coeff_tmp[0]) * ICP_BFGS_NUM_COEFFS);
    const double h = 0.00001;
    for (uint32_t i = 0; i < ICP_BFGS_NUM_COEFFS; i++) {
      coeff_tmp[i] = coeff[i] - h;
      double f0 = bfgsObjFunc(coeff_tmp);
      coeff_tmp[i] = coeff[i] + h;
      double f1 = bfgsObjFunc(coeff_tmp);
      coeff_tmp[i] = coeff[i];
      jacob[i] = (f1 - f0) / (2.0 * h);
    }
  }

  void ICP::bfgsUpdateFunc(double* coeff) {
    WrapTwoPI(coeff[ICP_ORIENT_X]);
    WrapTwoPI(coeff[ICP_ORIENT_Y]);
    WrapTwoPI(coeff[ICP_ORIENT_Z]);
#ifdef ICP_BFGS_INCLUDE_SCALE
    coeff[ICP_SCALE_X] = fabs(coeff[ICP_SCALE_X]);
    coeff[ICP_SCALE_Y] = fabs(coeff[ICP_SCALE_Y]);
    coeff[ICP_SCALE_Z] = fabs(coeff[ICP_SCALE_Z]);
#endif
  }

  ICP::~ICP() {
    SAFE_DELETE(edata_);
    SAFE_DELETE(bfgs_);
  }



}  // namespace math
}  // namespace jtil
