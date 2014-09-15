//
//  camera.h
//
//  Created by Jonathan Tompson on 6/3/12.
//

#pragma once

#include "math/math_types.h"

namespace renderer {
  class Renderer;

  class Camera {
  public:
    Camera(jtil::math::FloatQuat* eye_rot, jtil::math::Float3* eye_pos, 
      int screen_width, int screen_height, float field_of_view, float near, 
      float far);
    ~Camera();

    void updateView();
    void updateProjection();
    void rotateCamera(float theta_x, float theta_y);
    void moveCamera(jtil::math::Float3* dir_eye_space);

    // getter / setter methods
    inline jtil::math::Float4x4* view() { return &view_; }
    inline jtil::math::Float4x4* proj() { return &proj_; }
    inline jtil::math::Float2* near_far() { return &near_far_; }
    inline void near_far(jtil::math::Float2* set_val) { near_far_.set(*set_val); }
    inline jtil::math::Float2* cur_screen_size() { return &screen_size_; }
    inline void field_of_view(float set_val) { field_of_view_ = set_val; }
    inline void screen_width(float set_val) { screen_size_[0] = set_val; }
    inline void screen_height(float set_val) { screen_size_[1] = set_val; }
    inline jtil::math::Float3* eye_pos() { return &eye_pos_; }
    inline jtil::math::FloatQuat* eye_rot() { return &eye_rot_; }

    bool set_view_mat_directly;  // Hack, allows us to set the view
                                 // matrix directly without using quat + trans

  private:
    jtil::math::FloatQuat eye_rot_;
    jtil::math::FloatQuat eye_rot_inv_;
    jtil::math::Float3 eye_pos_;
    float x_axis_rot_;
    float y_axis_rot_;
    jtil::math::Float2 near_far_;
    jtil::math::Float2 screen_size_;
    float field_of_view_;

    jtil::math::Float4x4 view_;
    jtil::math::Float4x4 view_prev_frame_;
    jtil::math::Float4x4 proj_;
    jtil::math::Float4x4 proj_prev_frame_;  // Used for motion blur    
    jtil::math::Float4x4 view_inverse_;

    // Non-copyable, non-assignable.
    Camera(Camera&);
    Camera& operator=(const Camera&);
  };

};  // namespace renderer
