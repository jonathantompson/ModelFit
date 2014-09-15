#include "renderer/geometry/geometry_colored_lines.h"
#include "renderer/objects/aabbox.h"
#include "math/math_types.h"
#include "renderer/gl_state.h"

#define max std::max

using jtil::math::Float3;
using std::string;
using std::runtime_error;
using renderer::objects::AABBox;

namespace renderer {
  const void* ColoredLinesVertex::pos_offset =
    reinterpret_cast<const void*>(offsetof(struct ColoredLinesVertex, pos));
  const void* ColoredLinesVertex::col_offset =
  reinterpret_cast<const void*>(offsetof(struct ColoredLinesVertex, col));
  
  GeometryColoredLines::GeometryColoredLines() :
  Geometry() {
    // vertices_ will be zero size by default
    // colors_ will be zero size by default
    synced_ = false;
  }

  GeometryColoredLines::~GeometryColoredLines() {
    if (synced_) {
      GLState::glsDeleteBuffers(1, &vbo_);
      GLState::glsDeleteVertexArrays(1, &vao_);
    }

    children_.clear();  // Explicitly delete all the children (calling destrs)
  }

  void GeometryColoredLines::unsyncVAO() {
    if (synced_) {
      GLState::glsDeleteBuffers(1, &vbo_);
      GLState::glsDeleteVertexArrays(1, &vao_);
      synced_ = false;
    } else {
      throw std::runtime_error("unsyncVAO() - VAO not yet sunk!");
    }
  }

  void GeometryColoredLines::addLine(const float* xyz1, const float* xyz2, 
    const float* rgb1, const float* rgb2) {
    vertices_.pushBack(Float3(xyz1[0], xyz1[1], xyz1[2]));
    vertices_.pushBack(Float3(xyz2[0], xyz2[1], xyz2[2]));
    colors_.pushBack(Float3(rgb1[0], rgb1[1], rgb1[2]));
    colors_.pushBack(Float3(rgb2[0], rgb2[1], rgb2[2]));
  }

  void GeometryColoredLines::syncVAO() {
    if (synced_) {
      string err("syncVBO() - ERROR: dynamic VBOs not yet supported");
      throw std::runtime_error(err);
    }

    synced_num_vertices_ = vertices_.size();
    if (synced_num_vertices_ != colors_.size()) {
      string err("syncVBO() - ERROR: num colors != num vertices");
      throw std::runtime_error(err);
    }

    // Allocate a VAO
    GLState::glsGenVertexArrays(1, &vao_);
    // Bind our Vertex Array Object as the current used object
    GLState::glsBindVertexArray(0);  // Forces a rebinding in the GLState
    GLState::glsBindVertexArray(vao_);
    // Allocate and assign two Vertex Buffer Objects to our handle
    GLState::glsGenBuffers(1, &vbo_);
    // Bind our first VBO as being the active buffer and storing vertex 
    // attributes (coordinates)
    GLState::glsBindBuffer(GL_ARRAY_BUFFER, vbo_);

    // Allocate a vertex buffer
    ColoredLinesVertex dummy;  // just for sizeof
    static_cast<void>(dummy);  // Get rid of unreference local variable warning
    GLState::glsBufferData(GL_ARRAY_BUFFER,  // Target
                 vertices_.size() * sizeof(dummy),  // size
                 NULL,  // data --> Data is not initially copied
                 GL_STATIC_DRAW);  // usage

    // Copy the data into the vertex buffer
    ColoredLinesVertex* ptr = reinterpret_cast<ColoredLinesVertex*>(
      GLState::glsMapBuffer(GL_ARRAY_BUFFER, GL_WRITE_ONLY));
    for (uint32_t i = 0; i < vertices_.size(); i++) {
      ptr[i].pos[0] = static_cast<GLfloat>(vertices_[i][0]);
      ptr[i].pos[1] = static_cast<GLfloat>(vertices_[i][1]);
      ptr[i].pos[2] = static_cast<GLfloat>(vertices_[i][2]);
      ptr[i].col[0] = static_cast<GLfloat>(colors_[i][0]);
      ptr[i].col[1] = static_cast<GLfloat>(colors_[i][1]);
      ptr[i].col[2] = static_cast<GLfloat>(colors_[i][2]);
    }
    GLState::glsUnmapBuffer(GL_ARRAY_BUFFER);

    setVertexAttribPointer(VERTEX_POS_LOC, 3, GL_FLOAT, GL_FALSE, 
      sizeof(struct ColoredLinesVertex), ColoredLinesVertex::pos_offset);
    setVertexAttribPointer(VERTEX_COL_LOC, 3, GL_FLOAT, GL_FALSE,
      sizeof(struct ColoredLinesVertex), ColoredLinesVertex::col_offset);

    synced_ = true;
  }

  void GeometryColoredLines::bindVAO() {
    GLState::glsBindVertexArray(vao_);  // encapsulates all the vbo bindings
  }

  void GeometryColoredLines::unbindVAO() {
    // Nothing to do
  }

  void GeometryColoredLines::draw() {
    if (!synced_) {
      throw std::runtime_error(string("GeometryColoredLines::draw() - "
        "ERROR: trying to draw an unsynced vbo object!"));
    }

    bindVAO();  // Bind all required OpenGL resources

    GLState::glsDrawArrays(GL_LINES, 0, synced_num_vertices_);
    
    unbindVAO();  // Unbind all required OpenGL resources
  }

  Geometry* GeometryColoredLines::copy() {
    throw std::runtime_error(std::string("GeometryColoredLines::copy() - "
      "ERROR: copy not yet implemented for this Geometry type"));
  }

}  // namespace renderer

