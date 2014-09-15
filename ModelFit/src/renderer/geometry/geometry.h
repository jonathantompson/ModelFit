//
//  geometry.h
//
//  Created by Jonathan Tompson on 6/1/12.
//
//  Base class for geometry.  Contains simple phong lighting model material
//  parameters and affine-transforms.
//
//  Uses ASSIMP to load in 3D models in many possible formats (includeing: 3ds,
//  x, Collada, etc).  However, this library is slow.
//
//  Can save to what I call "jbin" format, which is a very simple but very fast
//  to load compressed binary format.  The load store memory overhead is high
//  and will need to be fixed later.

#pragma once

#include "math/math_types.h"
#include "data_str/vector_managed.h"

struct aiScene;
struct aiNode;
struct aiMesh;

#define DEFAULT_GEOMETRY_COLOR white
#define GEOMETRY_FILE_DATA_SIZE (640 / 8) // Bytes

namespace jtil {
namespace data_str { template <typename TFirst, typename TSecond> class Pair; }
}

namespace renderer {
  
  namespace objects { class AABBox; }

  struct Material {
    float specular_intensity;
    float specular_power;
    Material() : specular_intensity(1.0f), specular_power(32) { }
    Material& operator= (const Material& other) {
      if (this != &other) {  // protect against invalid self-assignment
        specular_intensity = other.specular_intensity;
        specular_power = other.specular_power;
      }
      // by convention, always return *this
      return *this;
    }
  };

  enum GeometryType {
    GEOMETRY_BASE = 0,
    GEOMETRY_COLORED_MESH = 1,
    GEOMETRY_TEXTURED_MESH = 2,
    GEOMETRY_VERTICES = 3,
    GEOMETRY_POINTS = 4,
    GEOMETRY_TEXTURED_BONED_MESH = 5,
    GEOMETRY_COLORED_BONED_MESH = 6,
    BOUNDING_SPHERE = 7,  // Special case just for hand models
  };

  struct GeometryFileData {
    uint32_t type;
    float mat_m[16];
    float mtrl_specular_intensity;
    float mtrl_specular_power;
    uint32_t name_size;
    // name char* to follow
  };

  class Geometry {
  public:
    friend class GeometryManager;

    // Constructor / Destructor
    Geometry();
    virtual ~Geometry();
    void addChild(Geometry* child);
    void removeChild(Geometry* child);  // O(n)
    void setChild(Geometry* child, uint32_t index);

    virtual void draw() { }
    inline virtual GeometryType type() { return GEOMETRY_BASE; }
    uint32_t numNodes();  // recursive O(n)

    // getter setter methods
    inline Geometry* parent() { return parent_; }
    inline jtil::math::Float4x4* mat() { return &mat_; }
    inline jtil::math::Float4x4* mat_hierarchy() { return &mat_hierarchy_; }
    inline uint32_t numChildren() { return children_.size(); }
    inline Geometry* getChild(uint32_t i) { return children_[i]; }
    inline objects::AABBox* aabbox() { return aabbox_; }
    inline Material* mtrl() { return &mtrl_; }
    inline std::string& name() { return name_; }

    // Perform a deep copy of the entire structure and sync the copy with OGL
    virtual Geometry* copy();

  protected:
    std::string name_;  // nodes are sometimes named (particularly bone trans)
    jtil::math::Float4x4 mat_;
    jtil::math::Float4x4 mat_hierarchy_;  // Updated when rendering
    Geometry* parent_;
    jtil::data_str::VectorManaged<Geometry*> children_;
    objects::AABBox* aabbox_;  // Bounding volume for frustrum tests
    Material mtrl_;

    // Used to set the position of vertex attributes in VBOs
    static void setVertexAttribPointer(const int id, int size, int type,
      bool normalized, int stride, const void* pointer);
    static void setVertexAttribIPointer(const int id, int size,
      int type, int stride, const void* pointer);

    // File IO:
    virtual void loadFromArray(const std::string& path, 
      const std::string& filename, const uint8_t* arr);
    // Convert the node and it's geometry into a data array for saving to file
    virtual jtil::data_str::Pair<uint8_t*,uint32_t> saveToArray();
    
    // Non-copyable, non-assignable.
    Geometry(Geometry&);
    Geometry& operator=(const Geometry&);
  };

};  // renderer namespace
