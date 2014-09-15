//
//  bfgs_eigen_data.h
//
//  Created by Jonathan Tompson on 4/40/2013.
//
//  A structure to encapsulate all the bfgs data.  Avoids exposing Eigen to 
//  all classes outside of jtil::math
//

#pragma once

#include <random>
#include "math/math_types.h"
#include <Eigen/Eigen>

namespace jtil {
namespace math {

  struct ICPEigenData {
  public:
    ICPEigenData();
    ~ICPEigenData();
    
    // http://eigen.tuxfamily.org/dox-devel/TopicStructHavingEigenMembers.html
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    Eigen::MatrixXf cross_cov_mat_;
    Eigen::MatrixXf rot_e_mat_;
  };

};  // namespace math
};  // namespace jtil
