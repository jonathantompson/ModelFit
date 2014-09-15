#include <random>
#include <stdexcept>
#include <iostream>
#include "math/icp_eigen_data.h"

using Eigen::MatrixXf;
using std::cout;
using std::endl;
using std::runtime_error;

namespace jtil {
namespace math {
  ICPEigenData::ICPEigenData() {
    cross_cov_mat_.resize(3, 3);
    rot_e_mat_.resize(3, 3);
  }

  ICPEigenData::~ICPEigenData() {

  }

}  // namespace math
}  // namespace jtil
