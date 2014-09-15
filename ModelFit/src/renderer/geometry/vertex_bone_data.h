//
//  vertex_bone_data.h
//
//  Created by Jonathan Tompson on 11/06/12.
//

#pragma once

#include "math/math_types.h"

namespace jtil {
namespace data_str { template <typename TFirst, typename TSecond> class Pair; }
}

#define BONE_VERTEX_DATA_FILE_DATA_SIZE (512 / 8)  // Bytes

namespace renderer {

    struct VertexBoneData { 
    uint32_t ids_03[4];  // Vertex attributes can only be length 4!
    //uint32_t ids_47[4];
    float weights_03[4];
    //float weights_47[4];
    VertexBoneData& operator=(const VertexBoneData &rhs);
    void attachBone(uint32_t bone_index, float weight);
  };
  
};  // renderer namespace
