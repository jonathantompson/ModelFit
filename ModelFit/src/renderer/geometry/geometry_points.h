//
//  geometry_points.h
//
//  Created by Jonathan Tompson on 10/1/12.
//
//  A simple geometry object for point sprites from a float array.
//
//  Usage: 1. Create the GeometryPoints instance.
//         2. Fill the vertex point buffer
//         3. call syncVAO when done filling it.
//         4. call draw when rendering it (with appropriate shaders)

#pragma once

#include "renderer/open_gl_common.h"  // For GLtypes
#include "renderer/geometry/geometry.h"
#include "math/math_types.h"
#include "data_str/vector.h"

namespace renderer {

  struct PointsVertex {
    GLfloat pos[3];
    static const void* pos_offset;
  };

  class GeometryPoints : public Geometry {
  public:
    // Constructor / Destructor
    GeometryPoints();
    virtual ~GeometryPoints();
    
    // getter and setter methods
    void addPoint(const jtil::math::Float3& point);
    void addPoint(const float x, const float y, const float z);
    void addPoint(const float* xyz);
    void addPoints(const float* xyz, uint32_t num_points);
    
    void syncVAO();  // At startup

    // Virtual methods
    virtual void draw();
    inline virtual GeometryType type() { return GEOMETRY_POINTS; }

    // Perform a deep copy of the entire structure and sync the copy with OGL
    virtual Geometry* copy();

  private:
    jtil::data_str::Vector<jtil::math::Float3> vertices_;
    GLuint vao_;  // Containing 1 VBOs which itself has vertex
    GLuint vbo_;  // Vertex buffer
    bool synced_;
    uint32_t synced_num_vertices_;
    
    // Bind the buffers with OpenG
    void bindVAO();  // For rendering
    void unbindVAO();

    // Non-copyable, non-assignable.
    GeometryPoints(GeometryPoints&);
    GeometryPoints& operator=(const GeometryPoints&);
  };
  
};  // renderer namespace
