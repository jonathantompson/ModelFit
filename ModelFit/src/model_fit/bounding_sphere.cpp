#include <algorithm>
#include <fstream>
#include <iostream>
#include "data_str/pair.h"
#include "model_fit/bounding_sphere.h"
#include "renderer/colors.h"
#include "math/math_types.h"

using jtil::math::Float3;
using std::wstring;
using std::runtime_error;
using std::string;
using std::cout;
using std::endl;
using jtil::data_str::Pair;

namespace renderer {
  BoundingSphere::BoundingSphere(float rad, Float3& center, Geometry* mesh_node,
    Geometry* hand_root, float* starting_bone_mat) :
  GeometryColoredMesh() {
    GeometryColoredMesh::makeSphere(reinterpret_cast<GeometryColoredMesh*>(this),
      BOUNDING_SPHERE_NSTACKS, BOUNDING_SPHERE_NSLICES, rad, center, white);
    radius_ = rad;
    center_ = center;
    mesh_node_ = mesh_node;
    hand_root_ = hand_root;
    starting_bone_mat_.set(starting_bone_mat);
    mat_.set(starting_bone_mat);
  }

  BoundingSphere::~BoundingSphere() {
    // parent destructor will be called automatically
  }

  Geometry* BoundingSphere::copy() {
    throw runtime_error("BoundingSphere::copy() - ERROR: Not"
      " yet implemented!");
  }

  void BoundingSphere::loadFromArray(const std::string& path, 
      const std::string& filename, const uint8_t* arr) {
    throw runtime_error("BoundingSphere::loadFromArray() - "
      "ERROR: Not yet implemented!");
  }

  Pair<uint8_t*,uint32_t> BoundingSphere::saveToArray() {
    throw runtime_error("BoundingSphere::loadFromArray() - "
      "ERROR: Not yet implemented!");
  }

  void BoundingSphere::transform() {
    Float3::affineTransformPos(transformed_center_, mat_hierarchy_, center_);

    // Figure out the transformed radius.  The following WONT work if there is
    // sqew.  It's a bit of a hack...
    jtil::math::Float3 rad_vec(radius_, 0, 0);
    Float3::affineTransformVec(transformed_rad_vec_, mat_hierarchy_, rad_vec);
    transformed_radius_ = transformed_rad_vec_.length();
    float rad_sq_ = Float3::dot(transformed_rad_vec_, transformed_rad_vec_);

    rad_vec.set(0, radius_, 0);
    Float3::affineTransformVec(transformed_rad_vec_, mat_hierarchy_, rad_vec);
    rad_sq_ = std::max<float>(Float3::dot(transformed_rad_vec_,
      transformed_rad_vec_), rad_sq_);

    rad_vec.set(0, 0, radius_);
    Float3::affineTransformVec(transformed_rad_vec_, mat_hierarchy_, rad_vec);
    rad_sq_ = std::max<float>(Float3::dot(transformed_rad_vec_,
      transformed_rad_vec_), rad_sq_);
    transformed_radius_ = sqrtf(rad_sq_);
  }

}  // namespace renderer

