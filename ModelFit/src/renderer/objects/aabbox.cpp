#include <limits>
#include "renderer/objects/aabbox.h"
#include "data_str/vector.h"

using jtil::math::Float3;
using jtil::math::Float4x4;
using jtil::data_str::Vector;

namespace renderer {
  namespace objects {

    AABBox::AABBox() {

    }

    AABBox::~AABBox() {

    }

    void AABBox::init(Vector<Float3>* vertices) {
      min_.set(std::numeric_limits<float>::infinity(),
        std::numeric_limits<float>::infinity(),
        std::numeric_limits<float>::infinity());
      max_.set(-std::numeric_limits<float>::infinity(),
        -std::numeric_limits<float>::infinity(),
        -std::numeric_limits<float>::infinity());
      for (uint32_t i = 0; i < vertices->size(); i++) {
        Float3* cur_vertex = vertices->at(i);
        if (cur_vertex->m[0] < min_[0]) {
          min_[0] = cur_vertex->m[0];
        }
        if (cur_vertex->m[1] < min_[1]) {
          min_[1] = cur_vertex->m[1];
        }
        if (cur_vertex->m[2] < min_[2]) {
          min_[2] = cur_vertex->m[2];
        }
        if (cur_vertex->m[0] > max_[0]) {
          max_[0] = cur_vertex->m[0];
        }
        if (cur_vertex->m[1] > max_[1]) {
          max_[1] = cur_vertex->m[1];
        }
        if (cur_vertex->m[2] > max_[2]) {
          max_[2] = cur_vertex->m[2];
        }
      }
      // Now fill up the AABBox coordinates
      object_bounds_[0].set(min_[0], max_[1], min_[2]);  // top front left
      object_bounds_[1].set(min_[0], max_[1], max_[2]);  // top back left
      object_bounds_[2].set(max_[0], max_[1], max_[2]);  // top back right
      object_bounds_[3].set(max_[0], max_[1], min_[2]);  // top front right
      object_bounds_[4].set(min_[0], min_[1], min_[2]);  // bottom front left
      object_bounds_[5].set(min_[0], min_[1], max_[2]);  // bottom back left
      object_bounds_[6].set(max_[0], min_[1], max_[2]);  // bottom back right
      object_bounds_[7].set(max_[0], min_[1], min_[2]);  // bottom front right
    }

    // Check Min/Max against input vector and update
    void AABBox::expand(Float3* vec) {
      if (min_[0] > (*vec)[0]) {
        min_[0] = (*vec)[0];
      }
      if (min_[1] > (*vec)[1]) {
        min_[1] = (*vec)[1];
      }
      if (min_[2] > (*vec)[2]) {
        min_[2] = (*vec)[2];
      }
      if (max_[0] < (*vec)[0]) {
        max_[0] = (*vec)[0];
      }
      if (max_[1] < (*vec)[1]) {
        max_[1] = (*vec)[1];
      }
      if (max_[2] < (*vec)[2]) {
        max_[2] = (*vec)[2];
      }
    }

    void AABBox::update(Float4x4* mat_world) {
      // Get bounding box in world coordinates
      for (uint32_t i = 0; i < 8; i++) {
        Float3::affineTransformPos(world_bounds_[i], *mat_world,
          object_bounds_[i]);
      }

      // Now get bounding box in axis aligned world coordinates --> Box area will 
      // grow --> ie, not necessarily an optimal bounding volume.
      min_[0] = world_bounds_[0][0]; 
      min_[1] = world_bounds_[0][1]; 
      min_[2] = world_bounds_[0][2]; 
      max_.set(min_);

      for (uint32_t i = 1; i < 8; i++) {
        expand(&world_bounds_[i]);
      }

      // Calculate the center (used by most of the collision query routines)
      Float3::add(center_, min_, max_);
      Float3::scale(center_, 0.5f);
      Float3::sub(half_lengths_, max_, min_);
      Float3::scale(half_lengths_, 0.5f);
    }
  }  // namespace objects
}  // namespace physics