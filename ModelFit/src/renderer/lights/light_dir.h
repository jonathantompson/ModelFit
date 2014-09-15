//
//  light_dir.h
//
//  Created by Jonathan Tompson on 6/15/12.
//

#pragma once

#include "renderer/lights/light.h"
#include "math/math_types.h"

namespace renderer {
  struct LightDirData {
    jtil::math::Float3 color;
    float ambient_intensity;
    float diffuse_intensity;
    jtil::math::Float3 direction_view;
  };


  class LightDir : public Light {
  public:
    LightDir();
    ~LightDir();

    inline virtual LightType type() { return LIGHT_DIR; }

    // Getters and setters
    inline LightDirData* light_data() { return &light_data_; }
    inline jtil::math::Float3* direction_world() { return &direction_world_; }
    inline jtil::math::Float3* color() { return &light_data_.color; }
    inline float ambient_intensity() { return light_data_.ambient_intensity; }
    inline float diffuse_intensity() { return light_data_.diffuse_intensity; }
    inline void direction_world(const jtil::math::Float3& val) { direction_world_ = val; }
    inline void color(const jtil::math::Float3& val) { light_data_.color = val; }
    inline void ambient_intensity(const float val) { light_data_.ambient_intensity = val; }
    inline void diffuse_intensity(const float val) { light_data_.diffuse_intensity = val; }

    virtual void update(jtil::math::Float4x4* camera_view);

  protected:
    jtil::math::Float3 direction_world_;
    LightDirData light_data_;

    // Non-copyable, non-assignable.
    LightDir(LightDir&);
    LightDir& operator=(const LightDir&);
  };

};  // namespace renderer
