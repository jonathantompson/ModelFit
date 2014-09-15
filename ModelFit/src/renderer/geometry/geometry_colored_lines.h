//
//  geometry_colored_lines.h
//
//  Created by Jonathan Tompson on 10/1/12.
//
//  A simple geometry object for line sprites from a float array.
//
//  Usage: 1. Create the GeometryLines instance.
//         2. Fill the vertex and color point buffers
//         3. call syncVAO when done filling it.
//         4. call draw when rendering it (with appropriate shaders)

#ifndef RENDERER_GEOMETRY_COLORED_LINES_HEADER
#define RENDERER_GEOMETRY_COLORED_LINES_HEADER

#include "renderer/open_gl_common.h"  // For GLtypes
#include "renderer/geometry/geometry.h"
#include "math/math_types.h"
#include "data_str/vector.h"

namespace renderer {

  struct ColoredLinesVertex {
    GLfloat pos[3];
    GLfloat col[3];
    static const void* pos_offset;
    static const void* col_offset;
  };

  class GeometryColoredLines : public Geometry {
  public:
    // Constructor / Destructor
    GeometryColoredLines();
    virtual ~GeometryColoredLines();
    
    // getter and setter methods
    void addLine(const float* xyz1, const float* xyz2, const float* rgb1,
      const float* rgb2);
    
    void syncVAO();  // At startup
    void unsyncVAO();  // Might be slow!
    inline bool synced() const { return synced_; }

    jtil::data_str::Vector<jtil::math::Float3>* vertices() { return &vertices_; }
    jtil::data_str::Vector<jtil::math::Float3>* colors() { return &colors_; }

    // Virtual methods
    virtual void draw();
    inline virtual GeometryType type() { return GEOMETRY_POINTS; }

    // Perform a deep copy of the entire structure and sync the copy with OGL
    virtual Geometry* copy();

  private:
    jtil::data_str::Vector<jtil::math::Float3> vertices_;
    jtil::data_str::Vector<jtil::math::Float3> colors_;
    GLuint vao_;  // Containing 1 VBOs which itself has vertex
    GLuint vbo_;  // Vertex buffer
    bool synced_;
    uint32_t synced_num_vertices_;
    
    // Bind the buffers with OpenG
    void bindVAO();  // For rendering
    void unbindVAO();

    // Non-copyable, non-assignable.
    GeometryColoredLines(GeometryColoredLines&);
    GeometryColoredLines& operator=(const GeometryColoredLines&);
  };
  
};  // renderer namespace

#endif  // RENDERER_GEOMETRY_COLORED_POINTS_HEADER
