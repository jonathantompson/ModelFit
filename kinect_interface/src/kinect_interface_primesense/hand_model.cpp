#include <random>
#include <thread>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include "kinect_interface_primesense/hand_model.h"

namespace kinect_interface_primesense {
namespace hand_net {
  
  // coeff_min_limit_ is the minimum coefficient value before the penalty
  // function heavily penalizes configurations with this value
  const float HandModel::coeff_min_limit_[HAND_NUM_COEFF] = {
    -std::numeric_limits<float>::infinity(),    // HAND_POS_X
    -std::numeric_limits<float>::infinity(),    // HAND_POS_Y
    -std::numeric_limits<float>::infinity(),    // HAND_POS_Z
    -3.14159f,  // HAND_ORIENT_X
    -3.14159f,  // HAND_ORIENT_Y
    -3.14159f,  // HAND_ORIENT_Z
    -0.903f,  // WRIST_THETA
    -1.580f,  // WRIST_PHI
    -0.523f,  // THUMB_THETA
    -0.523f,  // THUMB_PHI
    -0.633f,  // THUMB_K1_THETA
    -1.253f,  // THUMB_K1_PHI
    -1.733f,  // THUMB_K2_PHI
    -0.300f,  // F0_ROOT_THETA
    -0.300f,  // F0_ROOT_PHI
    -0.800f,  // F0_THETA
    -1.443f,  // F0_PHI
    -1.400f,  // F0_KNUCKLE_MID  // Formally -1.363 4/12/2013
    -1.500f,  // F0_KNUCKLE_END  // Formally -1.363 4/12/2013
    -0.300f,  // F1_ROOT_THETA
    -0.300f,  // F1_ROOT_PHI
    -0.800f,  // F1_THETA
    -1.443f,  // F1_PHI
    -1.400f,  // F1_KNUCKLE_MID  // Formally -1.363 4/12/2013
    -1.500f,  // F1_KNUCKLE_END  // Formally -1.363 4/12/2013
    -0.300f,  // F2_ROOT_THETA
    -0.300f,  // F2_ROOT_PHI
    -0.800f,  // F2_THETA
    -1.443f,  // F2_PHI
    -1.400f,  // F2_KNUCKLE_MID  // Formally -1.363 4/12/2013
    -1.500f,  // F2_KNUCKLE_END  // Formally -1.363 4/12/2013
    -0.300f,  // F3_ROOT_THETA
    -0.300f,  // F3_ROOT_PHI
    -0.800f,  // F3_THETA
    -1.443f,  // F3_PHI
    -1.400f,  // F3_KNUCKLE_MID  // Formally -1.363 4/12/2013
    -1.500f,  // F3_KNUCKLE_END  // Formally -1.363 4/12/2013
    -0.300f,  // F0_TWIST
    -0.400f,  // F1_TWIST
    -0.300f,  // F2_TWIST
    -0.300f,  // F3_TWIST
    -0.300f,  // THUMB_TWIST
  };

  // coeff_min_limit_ is the minimum coefficient value before the penalty
  // function heavily penalizes configurations with this value
  const float HandModel::coeff_min_limit_conservative_[HAND_NUM_COEFF] = {
    -std::numeric_limits<float>::infinity(),    // HAND_POS_X
    -std::numeric_limits<float>::infinity(),    // HAND_POS_Y
    -std::numeric_limits<float>::infinity(),    // HAND_POS_Z
    -3.14159f,  // HAND_ORIENT_X
    -3.14159f,  // HAND_ORIENT_Y
    -3.14159f,  // HAND_ORIENT_Z
    -0.903f,  // WRIST_THETA
    -1.580f,  // WRIST_PHI
    -0.523f,  // THUMB_THETA
    -0.523f,  // THUMB_PHI
    -0.170f,  // THUMB_K1_THETA
    -1.253f,  // THUMB_K1_PHI
    -1.733f,  // THUMB_K2_PHI
    -0.300f,  // F0_ROOT_THETA
    -0.300f,  // F0_ROOT_PHI
    -0.800f,  // F0_THETA
    -1.443f,  // F0_PHI
    -1.400f,  // F0_KNUCKLE_MID  // Formally -1.363 4/12/2013
    -1.500f,  // F0_KNUCKLE_END  // Formally -1.363 4/12/2013
    -0.300f,  // F1_ROOT_THETA
    -0.300f,  // F1_ROOT_PHI
    -0.800f,  // F1_THETA
    -1.443f,  // F1_PHI
    -1.400f,  // F1_KNUCKLE_MID  // Formally -1.363 4/12/2013
    -1.500f,  // F1_KNUCKLE_END  // Formally -1.363 4/12/2013
    -0.300f,  // F2_ROOT_THETA
    -0.300f,  // F2_ROOT_PHI
    -0.800f,  // F2_THETA
    -1.443f,  // F2_PHI
    -1.400f,  // F2_KNUCKLE_MID  // Formally -1.363 4/12/2013
    -1.500f,  // F2_KNUCKLE_END  // Formally -1.363 4/12/2013
    -0.300f,  // F3_ROOT_THETA
    -0.300f,  // F3_ROOT_PHI
    -0.800f,  // F3_THETA
    -1.443f,  // F3_PHI
    -1.400f,  // F3_KNUCKLE_MID  // Formally -1.363 4/12/2013
    -1.500f,  // F3_KNUCKLE_END  // Formally -1.363 4/12/2013
    -0.300f,  // F0_TWIST
    -0.400f,  // F1_TWIST
    -0.300f,  // F2_TWIST
    -0.300f,  // F3_TWIST
    -0.300f,  // THUMB_TWIST
  };
  
  // coeff_max_limit_ is the maximum coefficient value before the penalty
  // function heavily penalizes configurations with this value
  const float HandModel::coeff_max_limit_[HAND_NUM_COEFF] = {
    std::numeric_limits<float>::infinity(),    // HAND_POS_X
    std::numeric_limits<float>::infinity(),    // HAND_POS_Y
    std::numeric_limits<float>::infinity(),    // HAND_POS_Z
    3.14159f,  // HAND_ORIENT_X
    3.14159f,  // HAND_ORIENT_Y
    3.14159f,  // HAND_ORIENT_Z
    0.905f,  // WRIST_THETA
    1.580f,  // WRIST_PHI
    0.550f,  // THUMB_THETA
    0.580f,  // THUMB_PHI
    0.700f,  // THUMB_K1_THETA
    0.750f,  // THUMB_K1_PHI
    0.500f,  // THUMB_K2_PHI
    0.300f,  // F0_ROOT_THETA
    0.300f,  // F0_ROOT_PHI
    0.600f,  // F0_THETA
    0.670f,  // F0_PHI
    0.560f,  // F0_KNUCKLE_MID
    0.560f,  // F0_KNUCKLE_END
    0.300f,  // F1_ROOT_THETA
    0.300f,  // F1_ROOT_PHI
    0.600f,  // F1_THETA
    0.670f,  // F1_PHI
    0.560f,  // F1_KNUCKLE_MID
    0.560f,  // F1_KNUCKLE_END
    0.300f,  // F2_ROOT_THETA
    0.300f,  // F2_ROOT_PHI
    0.600f,  // F2_THETA
    0.670f,  // F2_PHI
    0.560f,  // F2_KNUCKLE_MID
    0.560f,  // F2_KNUCKLE_END
    0.300f,  // F3_ROOT_THETA
    0.300f,  // F3_ROOT_PHI
    0.600f,  // F3_THETA
    0.670f,  // F3_PHI
    0.560f,  // F3_KNUCKLE_MID
    0.560f,  // F3_KNUCKLE_END
    0.300f,  // F0_TWIST
    0.300f,  // F1_TWIST
    0.300f,  // F2_TWIST
    0.300f,  // F3_TWIST
    0.300f,  // THUMB_TWIST
  };

  // coeff_max_limit_ is the maximum coefficient value before the penalty
  // function heavily penalizes configurations with this value
  const float HandModel::coeff_max_limit_conservative_[HAND_NUM_COEFF] = {
    std::numeric_limits<float>::infinity(),    // HAND_POS_X
    std::numeric_limits<float>::infinity(),    // HAND_POS_Y
    std::numeric_limits<float>::infinity(),    // HAND_POS_Z
    3.14159f,  // HAND_ORIENT_X
    3.14159f,  // HAND_ORIENT_Y
    3.14159f,  // HAND_ORIENT_Z
    0.905f,  // WRIST_THETA
    1.580f,  // WRIST_PHI
    0.350f,  // THUMB_THETA
    0.300f,  // THUMB_PHI
    0.700f,  // THUMB_K1_THETA
    0.550f,  // THUMB_K1_PHI
    0.300f,  // THUMB_K2_PHI
    0.100f,  // F0_ROOT_THETA
    0.100f,  // F0_ROOT_PHI
    0.600f,  // F0_THETA
    0.270f,  // F0_PHI
    0.360f,  // F0_KNUCKLE_MID
    0.360f,  // F0_KNUCKLE_END
    0.300f,  // F1_ROOT_THETA
    0.300f,  // F1_ROOT_PHI
    0.600f,  // F1_THETA
    0.270f,  // F1_PHI
    0.360f,  // F1_KNUCKLE_MID
    0.360f,  // F1_KNUCKLE_END
    0.100f,  // F2_ROOT_THETA
    0.100f,  // F2_ROOT_PHI
    0.600f,  // F2_THETA
    0.270f,  // F2_PHI
    0.360f,  // F2_KNUCKLE_MID
    0.360f,  // F2_KNUCKLE_END
    0.300f,  // F3_ROOT_THETA
    0.300f,  // F3_ROOT_PHI
    0.600f,  // F3_THETA
    0.270f,  // F3_PHI
    0.360f,  // F3_KNUCKLE_MID
    0.360f,  // F3_KNUCKLE_END
    0.300f,  // F0_TWIST
    0.300f,  // F1_TWIST
    0.300f,  // F2_TWIST
    0.300f,  // F3_TWIST
    0.300f,  // THUMB_TWIST
  };
  
  // coeff_penalty_scale_ is the exponential scale to use when penalizing coeffs
  // outside the min and max values.
  const float HandModel::coeff_penalty_scale_[HAND_NUM_COEFF] = {
    0,    // HAND_POS_X
    0,    // HAND_POS_Y
    0,    // HAND_POS_Z
    0,  // HAND_ORIENT_X
    0,  // HAND_ORIENT_Y
    0,  // HAND_ORIENT_Z
    100,  // WRIST_THETA
    100,  // WRIST_PHI
    100,  // THUMB_THETA
    100,  // THUMB_PHI
    100,  // THUMB_K1_THETA
    100,  // THUMB_K1_PHI
    100,  // THUMB_K2_PHI
    100,  // F0_ROOT_THETA
    100,  // F0_ROOT_PHI
    100,  // F0_THETA
    100,  // F0_PHI
    100,  // F0_KNUCKLE_MID
    100,  // F0_KNUCKLE_END
    100,  // F1_ROOT_THETA
    100,  // F1_ROOT_PHI
    100,  // F1_THETA
    100,  // F1_PHI
    100,  // F1_KNUCKLE_MID
    100,  // F1_KNUCKLE_END
    100,  // F2_ROOT_THETA
    100,  // F2_ROOT_PHI
    100,  // F2_THETA
    100,  // F2_PHI
    100,  // F2_KNUCKLE_MID
    100,  // F2_KNUCKLE_END
    100,  // F3_ROOT_THETA
    100,  // F3_ROOT_PHI
    100,  // F3_THETA
    100,  // F3_PHI
    100,  // F3_KNUCKLE_MID
    100,  // F3_KNUCKLE_END
    100,  // F0_TWIST
    100,  // F1_TWIST
    100,  // F2_TWIST
    100,  // F3_TWIST
    100,  // THUMB_TWIST
  };

// angle_coeffs are boolean values indicating if the coefficient represents
  // a pure angle (0 --> 2pi)
  const bool HandModel::angle_coeffs_[HAND_NUM_COEFF] = {
    // Hand 1
    false,  // HAND_POS_X
    false,  // HAND_POS_Y
    false,  // HAND_POS_Z
    true,  // HAND_ORIENT_X
    true,  // HAND_ORIENT_Y
    true,  // HAND_ORIENT_Z
    true,   // WRIST_THETA
    true,   // WRIST_PHI
    true,   // THUMB_THETA
    true,   // THUMB_PHI
    true,   // THUMB_K1_THETA
    true,   // THUMB_K1_PHI
    true,   // THUMB_K2_PHI
    true,   // F0_ROOT_THETA
    true,   // F0_ROOT_PHI
    true,   // F0_THETA
    true,   // F0_PHI
    true,   // F0_KNUCKLE_MID
    true,   // F0_KNUCKLE_END
    true,   // F1_ROOT_THETA
    true,   // F1_ROOT_PHI
    true,   // F1_THETA
    true,   // F1_PHI
    true,   // F1_KNUCKLE_MID
    true,   // F1_KNUCKLE_END
    true,   // F2_ROOT_THETA
    true,   // F2_ROOT_PHI
    true,   // F2_THETA
    true,   // F2_PHI
    true,   // F2_KNUCKLE_MID
    true,   // F2_KNUCKLE_END
    true,   // F3_ROOT_THETA
    true,   // F3_ROOT_PHI
    true,   // F3_THETA
    true,   // F3_PHI
    true,   // F3_KNUCKLE_MID
    true,   // F3_KNUCKLE_END
    true,   // F0_TWIST
    true,   // F1_TWIST
    true,   // F2_TWIST
    true,   // F3_TWIST
    true,   // THUMB_TWIST
  };

}  // namespace hand_net
}  // namespace kinect_interface_primesense
