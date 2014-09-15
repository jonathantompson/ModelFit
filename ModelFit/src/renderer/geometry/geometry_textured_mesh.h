//
//  geometry_textured_mesh.h
//
//  Created by Jonathan Tompson on 10/28/12.
//
//  A simple geometry object for loading an external static textured mesh using
//  assimp and rendering it.
//

#pragma once

#include "renderer/open_gl_common.h"  // For GLtypes
#include "renderer/geometry/geometry.h"
#include "math/math_types.h"
#include "data_str/vector.h"

namespace jtil {
namespace data_str { template <typename TFirst, typename TSecond> class Pair; }
}

#define TEXTURED_MESH_FILE_DATA_SIZE (736 / 8)  // Bytes

namespace renderer {

  class Texture;

  struct TexturedMeshVertex {
    GLfloat pos[3];
    GLfloat norm[3];
    GLfloat tex[2];
    static const void* pos_offset;
    static const void* norm_offset;
    static const void* tex_offset;
  };

  struct TexturedMeshFileData {
    uint32_t type;
    float mat_m[16];
    float mtrl_specular_intensity;
    float mtrl_specular_power;
    uint32_t num_vert;
    uint32_t num_ind;
    uint32_t file_size;
    uint32_t name_size;
    // name to follow
    // Filename to follow
    // Vertex, normal, UV, index data to follow
  };

  class GeometryTexturedMesh : public Geometry {
  public:
    friend class GeometryManager;

    // Constructor / Destructor
    GeometryTexturedMesh();
    virtual ~GeometryTexturedMesh();
    
    // getter and setter methods
    Texture* tex() { return tex_; }

    // Virtual methods
    virtual void draw();
    inline virtual GeometryType type() { return GEOMETRY_TEXTURED_MESH; }

    // Perform a deep copy of the entire structure and sync the copy with OGL
    virtual Geometry* copy();

  private:
    jtil::data_str::Vector<jtil::math::Float3> vertices_;
    jtil::data_str::Vector<jtil::math::Float3> normals_;
    jtil::data_str::Vector<jtil::math::Float2> texture_coords_;
    jtil::data_str::Vector<uint32_t> indices_;
    Texture* tex_;  // NOT OWNED HERE
    std::string texture_filename_;
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
    static GeometryTexturedMesh* convertAssimpMesh(const std::string& path, 
      const std::string& filename, const aiScene* scene, 
      const aiMesh* mesh);

    // Convert the node and it's geometry into a data array for saving to file
    virtual jtil::data_str::Pair<uint8_t*,uint32_t> saveToArray();

    // Non-copyable, non-assignable.
    GeometryTexturedMesh(GeometryTexturedMesh&);
    GeometryTexturedMesh& operator=(const GeometryTexturedMesh&);
  };
  
};  // renderer namespace
