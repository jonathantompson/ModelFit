#include <string>
#include <iostream>
#include <fstream>
#include "data_str/vector.h"
#include "data_str/pair.h"
#include "data_str/circular_buffer.h"
#include "renderer/geometry/geometry.h"
#include "renderer/open_gl_common.h"
#include "assimp/Importer.hpp"      // C++ importer interface
#include "assimp/scene.h"           // Output data structure
#include "assimp/postprocess.h"     // Post processing flags
#include "math/math_types.h"
#include "renderer/gl_state.h"

using jtil::math::Float3;
using jtil::math::Float4x4;
using std::cout;
using std::endl;
using std::string;
using std::runtime_error;
using jtil::data_str::Vector;
using jtil::data_str::Pair;

namespace renderer {

  Geometry::Geometry() {
    mat_.identity(); 
    parent_ = NULL;
    aabbox_ = NULL;
    name_ = std::string();  // Empty string
    // children_ will be zero size by default
  }

  Geometry::~Geometry() {
    children_.clear();  // Explicitly delete all the children (calling destrs)
  }

  void Geometry::addChild(Geometry* child) {
    children_.pushBack(child);
    child->parent_ = this;
  }

  void Geometry::removeChild(Geometry* child) {
    uint32_t child_index = 0;
    while (children_[child_index] != child && child_index < children_.size()) {
      child_index++;
    }
    if (child_index == children_.size()) {
      throw std::runtime_error("Geometry::removeChild() - child not found!");
    }
    children_[child_index] = children_[children_.size()-1];
    children_.resize(children_.size() - 1);
    child->parent_ = NULL;
  }
  
  void Geometry::setChild(Geometry* child, uint32_t index) {
    if (children_.size() <= index) {
      throw std::runtime_error("setChild - ERROR: children_.size() <= index");
    }
    children_.set(index, child);
  }
  
  void Geometry::setVertexAttribPointer(const int id, int size,
    int type, bool normalized, int stride, const void* pointer) {
    if (type == GL_BYTE || type == GL_UNSIGNED_BYTE || type == GL_SHORT ||
      type == GL_UNSIGNED_SHORT || type == GL_INT || type == GL_UNSIGNED_INT) {
      throw runtime_error(string("Geometry::setVertexAttribPointer() - ERRO") +
        string("R: input type is integer, use setVertexAttribIPointer!"));
    }
    GLState::glsEnableVertexAttribArray(id);
    if (size > 4) {
      cout << "setVertexAttribPointer() - WARNING: size > 4 (not supported";
      cout << " by OpenGL)!" << endl;
    }
    GLState::glsVertexAttribPointer(id, size, type, normalized, stride, pointer);
  }

  void Geometry::setVertexAttribIPointer(const int id, int size,
    int type, int stride, const void* pointer) {
    if (type != GL_BYTE && type != GL_UNSIGNED_BYTE && type != GL_SHORT &&
      type != GL_UNSIGNED_SHORT && type != GL_INT && type != GL_UNSIGNED_INT) {
      throw runtime_error(string("Geometry::setVertexAttribIPointer() - ERR") +
        string("OR: input type is not integer, use setVertexAttribPointer!"));
    }
    GLState::glsEnableVertexAttribArray(id);
    if (size > 4) {
      cout << "setVertexAttribPointer() - WARNING: size > 4 (not supported";
      cout << " by OpenGL)!" << endl;
    }
    GLState::glsVertexAttribIPointer(id, size, type, stride, pointer);
  }

  void Geometry::loadFromArray(const std::string& path, 
      const std::string& filename, const uint8_t* arr) {
    const GeometryFileData* preamble = 
      reinterpret_cast<const GeometryFileData*>(arr);

    if (preamble->type != GeometryType::GEOMETRY_BASE) {
      throw runtime_error(string("Geometry::loadFromArray() - INTERNAL ") +
        string("ERROR: Incorrect data type"));
    }

    mat_.set((float*)preamble->mat_m);
    mtrl_.specular_intensity = preamble->mtrl_specular_intensity;
    mtrl_.specular_power = preamble->mtrl_specular_power;

    // Extract the name
    const char* name_c_str = reinterpret_cast<const char*>(&arr[GEOMETRY_FILE_DATA_SIZE]); 
    name_ = string(name_c_str);
  }

  uint32_t Geometry::numNodes() { 
    uint32_t ret = 1;
    for (uint32_t i = 0; i < children_.size(); i++) {
      ret += children_[i]->numNodes();
    }
    return ret;
  }

  jtil::data_str::Pair<uint8_t*,uint32_t> Geometry::saveToArray() {
    Pair<uint8_t*,uint32_t> data;
    data.first = NULL;
    data.second = 0;

    char char_dummy;
    static_cast<void>(char_dummy);
    if (sizeof(char_dummy) != 1) {
      throw std::runtime_error("saveToArray - sizeof(char) != 1");
    }
    
    uint32_t data_size = GEOMETRY_FILE_DATA_SIZE + 
      (uint32_t)(name_.size() + 1) * sizeof(char_dummy);  // name
    data.second = data_size;

    data.first = (uint8_t*)malloc(data_size);

    // Get the data ready
    GeometryFileData* preamble =
      reinterpret_cast<GeometryFileData*>(data.first);
    preamble->type = GeometryType::GEOMETRY_BASE;
    memcpy(preamble->mat_m, mat_.m, 16 * sizeof(preamble->mat_m[0]));
    preamble->mtrl_specular_intensity = mtrl_.specular_intensity;
    preamble->mtrl_specular_power = mtrl_.specular_power;
    preamble->name_size = (uint32_t)name_.size();

    // Now copy the name
    char* name_c_str = reinterpret_cast<char*>(&data.first[GEOMETRY_FILE_DATA_SIZE]);
    strcpy(name_c_str, name_.c_str());

    return data;
  }

  Geometry* Geometry::copy() {
    Geometry* ret = new Geometry();
    ret->mat_ = this->mat_;
    ret->mtrl_ = this->mtrl_;
    ret->name_ = this->name_;
    for (uint32_t i = 0; i < children_.size(); i++) {
      Geometry* child = children_[i]->copy();
      ret->addChild(child);
    }
    return ret;
  }
  
}  // namespace renderer

