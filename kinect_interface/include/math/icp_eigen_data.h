//
//  bfgs_eigen_data.h
//
//  Created by Jonathan Tompson on 4/40/2013.
//
//  A structure to encapsulate all the bfgs data.  Avoids exposing Eigen to 
//  all classes outside of icp::math
//

#pragma once

#include <random>
#include "math/math_types.h"
#include <Eigen/Eigen>

namespace jtil {
namespace math {

  template <typename T>
  struct ICPEigenData {
  public:
    ICPEigenData<T>();
    ~ICPEigenData<T>();
    
    // http://eigen.tuxfamily.org/dox-devel/TopicStructHavingEigenMembers.html
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> cross_cov_mat_;
    Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> rot_e_mat_;
  };

  template <typename T>
  ICPEigenData<T>::ICPEigenData() {
    cross_cov_mat_.resize(3, 3);
    rot_e_mat_.resize(3, 3);
  }

  template <typename T>
  ICPEigenData<T>::~ICPEigenData() {
    // Nothing to do
  }

};  // namespace math
};  // namespace icp
