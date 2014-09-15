#include "renderer/lights/light_dir.h"

using jtil::math::Float4x4;
using jtil::math::Float3;

namespace renderer {

  LightDir::LightDir() {
    // Nothing to do
  }

  LightDir::~LightDir() {
    // Nothing to do
  }

  void LightDir::update(Float4x4* camera_view) {
    // Just transform the light into view space
    Float3::affineTransformVec(light_data_.direction_view, *camera_view,
      direction_world_);
  }

}  // namespace renderer