//
//  bone_info.h
//
//  Created by Jonathan Tompson on 11/06/12.
//

#pragma once

#include <map>
#include <string>
#include "data_str/vector_managed.h"
#include "math/math_types.h"

#define BONE_DATA_SIZE (544 / 8)  // Bytes  --> Not all bone info saved to disk!
#define BONE_FILE_INFO_DATA_SIZE (64 / 8)  // Bytes  --> Not all bone info saved to disk!

namespace jtil {
namespace data_str { template <typename TFirst, typename TSecond> class Pair; }
}

namespace renderer {
  class Geometry;

  struct BoneFileInfoFileData {
    uint32_t num_bones;
    uint32_t size_filename;
    // filename to follow
    // bone data to follow
  };

  struct Bone {
    float bone_offset[16];  // transform to local space
    uint32_t name_length;  // Used when loading from disk
    uint32_t index;  // Index in the shader bone matrix (or dual quat.) array
    std::string name;  // Name of the geometry it attaches to. SAVED TO DISK AS CHAR*
    void* node;  // The Geometry* node in the heirachy corresponding to this 
                 // bone. NOT SAVED TO DISK!
    jtil::math::Float4x4 final_trans;  // NOT SAVED TO DISK!
    float uniform_dual_quaternion[2][4];  // NOT SAVED TO DISK

    Bone& operator=(const Bone &rhs);
    inline Geometry* getNode() { return (Geometry*)node; }
    Bone(const std::string& name, float* bone_transform);
    Bone();
    ~Bone() { }
  };

  struct BoneFileInfo {
    std::string model_filename;
    // We need O(1) string lookup but also we need to linearly iterate through 
    // the database, so we need BOTH a hash map and a vector.
    std::map<std::string, uint32_t> bone_mapping; // maps bone name to index
    jtil::data_str::VectorManaged<Bone*> bones;  // the bones for this file
    Geometry* model_root_node;  // Used when calculating inverse root transform

    Bone* findBone(const std::string& bone_name);

    // Convert the node and it's geometry into a data array for saving to file
    jtil::data_str::Pair<uint8_t*,uint32_t> saveToArray();
    void loadFromArray(const std::string& path, const std::string& filename, 
      const uint8_t* arr);
  };

};  // renderer namespace
