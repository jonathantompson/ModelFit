//
//  aabbox.h
//
//  Created by Jonathan Tompson on 6/24/12.
//

#pragma once

#include "math/math_types.h"

namespace jtil {
namespace data_str { template <typename T> class Vector; }
}

namespace renderer {
  namespace objects {
    class AABBox {
    public:
      AABBox();
      ~AABBox();

      void init(jtil::data_str::Vector<jtil::math::Float3>* vertices);
      void update(jtil::math::Float4x4* mat_world);

      inline jtil::math::Float3* min_bounds() { return &min_; }
      inline jtil::math::Float3* max_bounds() { return &max_; }
      inline jtil::math::Float3* center() { return &center_; }
      inline jtil::math::Float3* half_lengths() { return &half_lengths_; }

    private:
      jtil::math::Float3 min_;
      jtil::math::Float3 max_;
      jtil::math::Float3 object_bounds_[8];
      jtil::math::Float3 world_bounds_[8];
      jtil::math::Float3 center_;
      jtil::math::Float3 half_lengths_;

      // Expand(), check current min/max value against input vector and set new 
      // min/max if appropriate
      void expand(jtil::math::Float3* vec);
    };
  };  // namespace objects
};  // namespace renderer
