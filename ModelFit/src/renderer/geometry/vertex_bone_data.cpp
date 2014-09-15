#include "renderer/geometry/vertex_bone_data.h"
#include "renderer/open_gl_common.h"

namespace renderer {
  void VertexBoneData::attachBone(uint32_t bone_index, float weight) {
    // Use the next avaliable bone attachement (that is zero weight)
    // The only complication is that the weights are split accross 2 variables.
    for (uint32_t i = 0; i < MAX_VERTEX_BONE_COUNT; i++) {
      if (i < 4) {
        if (weights_03[i] == 0.0) {
          weights_03[i] = weight;
          ids_03[i] = bone_index;
          return;
        }
      }/*
      else if (i < 8) {
        if (weights_47[i-4] == 0.0) {
          weights_47[i-4] = weight;
          ids_47[i-4] = bone_index;
          return;
        }
      }*/
    }
  }

  VertexBoneData& VertexBoneData::operator=(const VertexBoneData &rhs) {
    // Only do assignment if RHS is a different object from this.
    if (this != &rhs) {
      for (uint32_t i = 0; i < 4; i++) {
        ids_03[i] = rhs.ids_03[i];
        weights_03[i] = rhs.weights_03[i];
        //ids_47[i] = rhs.ids_47[i];
        //weights_47[i] = rhs.weights_47[i];
      }
    }
    return *this;
  }
}  // namespace renderer

