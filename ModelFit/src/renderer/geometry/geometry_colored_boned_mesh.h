//
//  geometry_colored_boned_mesh.h
//
//  Created by Jonathan Tompson on 11/06/12.
//
//  A simple geometry object for loading an external static mesh using
//  assimp and rendering it.  This version also includes bones for linear blend
//  skinning.
//

#pragma once

#include <map>
#include "renderer/open_gl_common.h"  // For GLtypes
#include "renderer/geometry/geometry.h"
#include "renderer/geometry/vertex_bone_data.h"
#include "renderer/geometry/bone_info.h"
#include "math/math_types.h"
#include "data_str/vector.h"

namespace jtil {
namespace data_str { template <typename TFirst, typename TSecond> class Pair; }
}

#define COLORED_BONED_MESH_FILE_DATA_SIZE (736 / 8)  // Bytes

namespace renderer {

  class Texture;

  struct ColoredBonedMeshVertex {
    GLfloat pos[3];
    GLfloat norm[3];
    GLfloat col[3];
    VertexBoneData bone_data;
    static const void* pos_offset;
    static const void* norm_offset;
    static const void* col_offset;
    static const void* bone_ids_03_offset;
    //static const void* bone_ids_47_offset;
    static const void* bone_weights_03_offset;
    //static const void* bone_weights_47_offset;
  };

  struct ColoredBonedMeshFileData {
    uint32_t type;
    float mat_m[16];
    float mtrl_specular_intensity;
    float mtrl_specular_power;
    uint32_t num_vert;
    uint32_t num_ind;
    uint32_t name_size;
    uint32_t bone_filename_size;
    // Name to follow
    // bone filename to follow
    // Vertex, normal, color, bone_data, index data to follow
  };

  class GeometryColoredBonedMesh : public Geometry {
  public:
    friend class GeometryManager;

    // Constructor / Destructor
    GeometryColoredBonedMesh();
    virtual ~GeometryColoredBonedMesh();

    // getter and setter methods
    inline BoneFileInfo* bones() { return bones_; }

    // Virtual methods
    virtual void draw();
    inline virtual GeometryType type() { return GEOMETRY_COLORED_BONED_MESH; }

    // Perform a deep copy of the entire structure and sync the copy with OGL
    virtual Geometry* copy();

  private:
    jtil::data_str::Vector<jtil::math::Float3> vertices_;
    jtil::data_str::Vector<jtil::math::Float3> normals_;
    jtil::data_str::Vector<jtil::math::Float3> colors_;
    jtil::data_str::Vector<VertexBoneData> vertex_bone_data_;
    jtil::data_str::Vector<uint32_t> indices_;
    BoneFileInfo* bones_;  // NOT OWNED HERE!
    std::string bones_filename_;
    GLuint vao_;  // Containing 1 VBOs which itself has vertex, normal and col
    GLuint vbo_;  // Vertex buffer
    GLuint ibo_;  // index buffer (optional)
    bool synced_;
    uint32_t synced_num_vertices_;
    uint32_t synced_num_indices_;
    
    // Bind the buffers with OpenGL
    void syncVAO();  // At startup
    void bindVAO();  // For rendering
    void unbindVAO();

    // loadFromArray - Internal use for when loading from jbin format.
    virtual void loadFromArray(const std::string& path, 
      const std::string& filename, const uint8_t* arr);

    // convertAssimpMesh - Internal use for parsing assimp data structure
    static GeometryColoredBonedMesh* convertAssimpMesh(const std::string& path, 
      const std::string& filename, const aiScene* scene, 
      const aiMesh* mesh);

    // Convert the node and it's geometry into a data array for saving to file
    virtual jtil::data_str::Pair<uint8_t*,uint32_t> saveToArray();

    // Non-copyable, non-assignable.
    GeometryColoredBonedMesh(GeometryColoredBonedMesh&);
    GeometryColoredBonedMesh& operator=(const GeometryColoredBonedMesh&);
  };
  
};  // renderer namespace
