#include "renderer/geometry/bone_info.h"
#include "math/math_types.h"
#include "data_str/pair.h"

using jtil::math::Float4x4;

namespace renderer {

  Bone& Bone::operator=(const Bone &rhs) {
    // Only do assignment if RHS is a different object from this.
    if (this != &rhs) {
      memcpy(this->bone_offset, rhs.bone_offset, 16 * sizeof(this->bone_offset[0]));
      this->final_trans.set(*const_cast<Float4x4*>(&rhs.final_trans));
      this->name = rhs.name;
      this->node = rhs.node;
      for (uint32_t i = 0; i < 2; i++) {
        for(uint32_t j = 0; j < 4; j++) {
          this->uniform_dual_quaternion[i][j] = rhs.uniform_dual_quaternion[i][j];
        }
      }
    }
    return *this;
  }

  Bone::Bone(const std::string& name, float* bone_transform) {
    this->name = name;
    this->name_length = (uint32_t)name.length();
    this->node = NULL;
    Float4x4 mat(bone_transform);
#ifdef COLUMN_MAJOR
    // assimp is row major, we are column major, matrix needs transpose
    mat.transpose();
#endif
    memcpy(bone_offset, mat.m, 16 * sizeof(bone_offset[0]));
  }

  Bone::Bone() {
    this->name = std::string();
    this->name_length = 0;
    this->node = NULL;
    Float4x4 mat;
    mat.identity();
    memcpy(bone_offset, mat.m, 16 * sizeof(bone_offset[0]));
  }

  Bone* BoneFileInfo::findBone(const std::string& bone_name) {
    if (bone_mapping.find(bone_name) != bone_mapping.end()) {
      return bones[bone_mapping[bone_name]];
    } else {
      return NULL;
    }
  }

  // Convert the node and it's geometry into a data array for saving to file
  jtil::data_str::Pair<uint8_t*,uint32_t> BoneFileInfo::saveToArray() {
    jtil::data_str::Pair<uint8_t*,uint32_t> data;
    data.first = NULL;
    data.second = 0;

    char char_dummy; static_cast<void>(char_dummy);
    if (sizeof(char_dummy) != 1) {
      throw std::runtime_error("saveToArray - basic types are the wrong size!");
    }

    uint32_t bone_names_length = 0;
    for (uint32_t i = 0; i < bones.size(); i++) {
      bone_names_length += (uint32_t)bones[i]->name.length() + 1;
    }

    uint32_t data_size = BONE_FILE_INFO_DATA_SIZE +
      (uint32_t)(model_filename.size() + 1) * sizeof(char_dummy) +  // name
      BONE_DATA_SIZE * bones.size() +  // The bones themselves
      bone_names_length * sizeof(char_dummy);  // bone names
    data.second = data_size;

    data.first = (uint8_t*)malloc(data_size);

    // Get the data ready
    BoneFileInfoFileData* preamble =
      reinterpret_cast<BoneFileInfoFileData*>(data.first);
    preamble->num_bones = bones.size();
    preamble->size_filename = (uint32_t)model_filename.size();

    char* name_c_str = reinterpret_cast<char*>(&data.first[BONE_FILE_INFO_DATA_SIZE]);
    strcpy(name_c_str, model_filename.c_str());

    uint8_t* bones = reinterpret_cast<uint8_t*>(&name_c_str[model_filename.size()+1]);
    // Now save each bone
    for (uint32_t i = 0; i < this->bones.size(); i++) {
      uint32_t cur_name_length = (uint32_t)this->bones[i]->name.length();
      Bone* cur_bone_data = (Bone*)(bones);
      memcpy(cur_bone_data->bone_offset, this->bones[i]->bone_offset, 
        16 * sizeof(cur_bone_data->bone_offset[0]));
      cur_bone_data->name_length = cur_name_length;
      // Move the pointer forward to the string
      bones = &bones[BONE_DATA_SIZE];
      char* cur_bone_data_name = (char*)(bones);
      strcpy(cur_bone_data_name, this->bones[i]->name.c_str());
      bones = &bones[cur_name_length + 1];
    }

    return data;
  }

  void BoneFileInfo::loadFromArray(const std::string& path, 
    const std::string& filename, const uint8_t* arr) {
    if (bones.size() != 0) {
      throw std::runtime_error("ERROR: BoneFileInfo is not empty!");
    }
    const BoneFileInfoFileData* preamble = 
      reinterpret_cast<const BoneFileInfoFileData*>(arr);

    const char* name_c_str = (const char*)(&arr[BONE_FILE_INFO_DATA_SIZE]); 
    model_filename = std::string(name_c_str);

    const uint8_t* bones = 
      reinterpret_cast<const uint8_t*>(&name_c_str[preamble->size_filename + 1]);
    this->bones.capacity(preamble->num_bones);
    this->bones.resize(preamble->num_bones);
    for (uint32_t i = 0; i < preamble->num_bones; i++) {
      this->bones[i] = new Bone();
      Bone* cur_bone_data = (Bone*)(bones);
      memcpy(this->bones[i]->bone_offset, cur_bone_data->bone_offset, 
        16 * sizeof(this->bones[i]->bone_offset[0]));
      this->bones[i]->name_length = cur_bone_data->name_length;
      // Move the pointer forward to the string
      bones = &bones[BONE_DATA_SIZE];
      char* cur_bone_data_name = (char*)(bones);
      this->bones[i]->name = std::string(cur_bone_data_name);
      bones = &bones[this->bones[i]->name_length + 1];
    }
  }

}  // namespace renderer

