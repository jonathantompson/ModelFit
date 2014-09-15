#include <string>
#include "renderer/camera/camera.h"
#include "renderer/open_gl_common.h"
#include "math/math_types.h"
#include "data_str/vector_managed.h"

using jtil::math::Float3;
using jtil::math::Float4x4;
using jtil::math::FloatQuat;
using std::runtime_error;
using std::string;

namespace renderer {

  Camera::Camera(FloatQuat* eye_rot, Float3* eye_pos, int screen_width,
    int screen_height, float field_of_view, float near, float far) {
    proj_.identity();
    eye_rot_.set(*eye_rot);
    FloatQuat::inverse(eye_rot_inv_, eye_rot_);
    eye_pos_.set(*eye_pos);
    x_axis_rot_ = 0;
    y_axis_rot_ = 0;
    
    screen_size_.set(static_cast<float>(screen_width),
                     static_cast<float>(screen_height));
    field_of_view_ = field_of_view;
    if (near > 0 || far > 0) {
      throw std::runtime_error("Camera::Camera() - ERROR: "
        "near_far_[0] > 0 || near_far_[1] > 0! \n"
        "OpenGL convention is to look down the negative z axis!");
    }
    near_far_[0] = near;
    near_far_[1] = far;
    set_view_mat_directly = false;
  }

  Camera::~Camera() {
    // Empty destructor
  }

  void Camera::updateView() {
    // Before updating view, record the old value
    view_prev_frame_.set(view_);
    
    if (!set_view_mat_directly) {
      // Find the inverse rotation using quaternion
      FloatQuat::inverse(eye_rot_inv_, eye_rot_);  // Very fast
      FloatQuat::quat2Mat4x4(view_, eye_rot_inv_);
    
      view_[2] *= -1.0f;
      view_[6] *= -1.0f;
      view_[10] *= -1.0f;

      view_.rightMultTranslation(eye_pos_[0], eye_pos_[1], eye_pos_[2]);
      Float4x4::affineRotationTranslationInverse(view_inverse_, view_);
    } else {
      Float4x4::inverse(view_inverse_, view_);
    }
  }

  void Camera::updateProjection() {
    // Before updating projection, record the old value
    proj_prev_frame_.set(proj_);

    // Recall: OpenGL convention is to look down the negative Z axis,
    //         therefore, more negative values are actually further away.
    proj_.glProjection(-near_far_[0], -near_far_[1], field_of_view_,
      screen_size_[0], screen_size_[1]);
  }
  
  void Camera::rotateCamera(float theta_x, float theta_y) {
    y_axis_rot_ += theta_x;
    // Keep between -pi and +pi (allow wrap around rotations)
    y_axis_rot_ = (y_axis_rot_ > static_cast<float>(M_PI)) ?
      y_axis_rot_ - 2.0f*static_cast<float>(M_PI) : y_axis_rot_;
    y_axis_rot_ = (y_axis_rot_ < -static_cast<float>(M_PI)) ?
      y_axis_rot_ + 2.0f*static_cast<float>(M_PI) : y_axis_rot_;
    // Clamp between -pi_2 and +pi_2
    x_axis_rot_ += theta_y;
    x_axis_rot_ = (x_axis_rot_ > static_cast<float>(M_PI_2)) ?
      static_cast<float>(M_PI_2) : x_axis_rot_;
    x_axis_rot_ = (x_axis_rot_ < -static_cast<float>(M_PI_2)) ?
      -static_cast<float>(M_PI_2) : x_axis_rot_;

    // Rotate by y-axis first
    eye_rot_.identity();
    FloatQuat rot_mouse;
    rot_mouse.yAxisRotation(y_axis_rot_);
    FloatQuat rot_tmp;
    FloatQuat::mult(rot_tmp, eye_rot_, rot_mouse);
    // Now rotate by x-axis
    rot_mouse.xAxisRotation(x_axis_rot_);
    FloatQuat::mult(eye_rot_, rot_tmp, rot_mouse);
  }
  
  void Camera::moveCamera(Float3* dir_eye_space) {
    // First calcuate the direction in world coords
    Float3 dir_world_coords;
    Float3::affineTransformVec(dir_world_coords, view_inverse_, *dir_eye_space);
    eye_pos_.accum(dir_world_coords.m);
  }

}  // namespace renderer
