//
//  geometry_colored_mesh.h
//
//  Created by Jonathan Tompson on 6/7/12.
//
//  A simple geometry object for rendering primative shapes that consist of
//  a vertex buffer, a color buffer and an index buffer.
//
//  Optional: If any indices are added to the index buffer vector before 
//            syncVBO() is called, then an index buffer will by synced with 
//            OpenGL and the tringles will be rendered using it.

#pragma once

#include "renderer/open_gl_common.h"  // For GLtypes
#include "renderer/geometry/geometry.h"
#include "math/math_types.h"
#include "data_str/vector.h"

struct aiScene;
struct aiNode;
struct aiMesh;

#define COLORED_MESH_FILE_DATA_SIZE (704 / 8) // Bytes

namespace jtil {
namespace data_str { template <typename TFirst, typename TSecond> class Pair; }
}

namespace renderer {

  struct ColoredMeshVertex {
    GLfloat pos[3];
    GLfloat norm[3];
    GLfloat col[3];
    static const void* pos_offset;
    static const void* norm_offset;
    static const void* col_offset;
  };

  struct ColoredMeshFileData {
    uint32_t type;
    float mat_m[16];
    float mtrl_specular_intensity;
    float mtrl_specular_power;
    uint32_t num_vert;
    uint32_t num_ind;
    uint32_t name_size;
    // name char*, Vertex, normal, color, index data to follow
  };

  class GeometryColoredMesh : public Geometry {
  public:
    friend class GeometryManager;

    // Constructor / Destructor
    GeometryColoredMesh();
    virtual ~GeometryColoredMesh();
    
    // getter and setter methods
    void addVertex(const jtil::math::Float3& vertex);
    void addVertex(const float x, const float y, const float z);
    void addVertex(const float* xyz);
    void addNormal(const jtil::math::Float3& norm);
    void addNormal(const float x, const float y, const float z);
    void addNormal(const float* xyz);
    void addColor(const jtil::math::Float3& color);
    void addColor(const float r, const float g, const float b);
    void addColor(const float* rgb);
    void addFace(const uint32_t* v012);
    void addFace(const uint32_t v0, const uint32_t v1, const uint32_t v2);

    jtil::data_str::Vector<jtil::math::Float3>* vertices() { return &vertices_; }
    jtil::data_str::Vector<jtil::math::Float3>* normals() { return &normals_; }
    jtil::data_str::Vector<jtil::math::Float3>* colors() { return &colors_; }
    jtil::data_str::Vector<uint32_t>* indices() { return &indices_; }

    // Geometry builder methods --> Make new geometry primatives
    static GeometryColoredMesh* makeCube(const jtil::math::Float3& color);
    static GeometryColoredMesh* makeCube(const jtil::math::Float3& ctop,
      const jtil::math::Float3& cbottom, const jtil::math::Float3& cleft,
      const jtil::math::Float3& cright, const jtil::math::Float3& cfront,
      const jtil::math::Float3& cback);
    static GeometryColoredMesh* makeCubeRainbow();
    static GeometryColoredMesh* makePyramid(const jtil::math::Float3& color);
    static GeometryColoredMesh* makePyramidRainbow();
    static GeometryColoredMesh* makeTorusKnot(const jtil::math::Float3& color, 
      uint32_t turns, uint32_t slices, uint32_t stacks);
    static GeometryColoredMesh* makeSphere(uint32_t n_stacks, 
      uint32_t n_slices, float inside_radius, 
      const jtil::math::Float3& color);
    static GeometryColoredMesh* makeCone(uint32_t n_slices, float height, 
      float inside_radius, const jtil::math::Float3& color);
    static GeometryColoredMesh* makeCylinder(uint32_t n_slices, float height,
      float base_inside_radius, float top_inside_radius,
      const jtil::math::Float3& color);
    static GeometryColoredMesh* makeOpenCylinder(uint32_t n_slices, 
      float height, float base_inside_radius, float top_inside_radius,
      const jtil::math::Float3& color);

    static float calcSphereOutsideRadius(uint32_t n_stacks, 
      uint32_t n_slices, float inside_radius);
    static float calcConeOutsideRadius(uint32_t n_slices, float inside_radius);

    // Virtual methods
    virtual void draw();
    inline virtual GeometryType type() { return GEOMETRY_COLORED_MESH; }

    // Perform a deep copy of the entire structure and sync the copy with OGL
    virtual Geometry* copy();

    void syncVAO();  // Sync geometry once at startup
    void unsyncVAO();  // Unsync in preperation for modification

  protected:
    jtil::data_str::Vector<jtil::math::Float3> vertices_;
    jtil::data_str::Vector<jtil::math::Float3> normals_;
    jtil::data_str::Vector<jtil::math::Float3> colors_;
    jtil::data_str::Vector<uint32_t> indices_;
    GLuint vao_;  // Containing 1 VBOs which itself has vertex, normal and col
    GLuint vbo_;  // Vertex buffer
    GLuint ibo_;  // index buffer (optional)
    bool synced_;
    uint32_t synced_num_vertices_;
    uint32_t synced_num_indices_;

    static const jtil::math::Float3 cube_vertices_[8];
    static const jtil::math::Float3 pyramid_vertices_[5];
    static const jtil::math::Float3 rainbow_colors_[9];
    
    static void addTriangle(GeometryColoredMesh* mesh, 
      const jtil::math::Float3& color, int i0, int i1, int i2, 
      const jtil::math::Float3* vert);
    static void makeSphere(GeometryColoredMesh* ret, uint32_t n_stacks,
      uint32_t n_slices, float inside_radius, const jtil::math::Float3& center, 
      const jtil::math::Float3& color);
    
    // Bind the buffers with OpenGL
    void bindVAO();  // For rendering
    void unbindVAO();

    // loadFromArray - Internal use for when loading from jbin format.
    virtual void loadFromArray(const std::string& path, 
      const std::string& filename, const uint8_t* arr);

    // convertAssimpMesh - Internal use for parsing assimp data structure
    static GeometryColoredMesh* convertAssimpMesh(const std::string& path, 
      const std::string& filename, const aiScene* scene, 
      const aiMesh* mesh);

    // Convert the node and it's geometry into a data array for saving to file
    virtual jtil::data_str::Pair<uint8_t*,uint32_t> saveToArray();

    // Non-copyable, non-assignable.
    GeometryColoredMesh(GeometryColoredMesh&);
    GeometryColoredMesh& operator=(const GeometryColoredMesh&);
  };
  
};  // renderer namespace
