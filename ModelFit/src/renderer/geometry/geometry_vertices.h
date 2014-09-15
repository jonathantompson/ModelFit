//
//  geometry_vertices.h
//
//  Created by Jonathan Tompson on 6/17/12.
//
//  A simple geometry object for rendering primative shapes that consist of
//  a vertex buffer only (draw as triplets of triangles).
//

#pragma once

#include "renderer/open_gl_common.h"  // For GLtypes
#include "renderer/geometry/geometry.h"
#include "math/math_types.h"
#include "data_str/vector.h"

namespace renderer {

  struct VerticesVertex {
    GLfloat pos[3];
    static const void* pos_offset;
  };

  class GeometryVertices : public Geometry {
  public:
    // Constructor / Destructor
    GeometryVertices();
    virtual ~GeometryVertices();
    
    // getter and setter methods
    void addVertex(const jtil::math::Float3& vertex);
    void addVertex(const float x, const float y, const float z);
    void addVertex(const float* xyz);

    // Geometry builder methods --> Make new geometry primatives
    static GeometryVertices* makeQuad();

    // Virtual methods
    virtual void draw();
    inline virtual GeometryType type() { return GEOMETRY_VERTICES; }

    // Perform a deep copy of the entire structure and sync the copy with OGL
    virtual Geometry* copy();

  private:
    jtil::data_str::Vector<jtil::math::Float3> vertices_;
    GLuint vao_;  // Containing 1 VBO (just the vertex array)
    GLuint vbo_;  // Vertex buffer
    bool synced_;
    uint32_t synced_num_vertices_;

    static const jtil::math::Float3 vertices_quad_[6];
    
    // Sync the buffers with OpenGL
    void syncVAO();
    void bindVAO();
    void unbindVAO();

    // Non-copyable, non-assignable.
    GeometryVertices(GeometryVertices&);
    GeometryVertices& operator=(const GeometryVertices&);
  };
  
};  // renderer namespace
