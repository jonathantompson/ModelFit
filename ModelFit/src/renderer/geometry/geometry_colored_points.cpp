#include "renderer/geometry/geometry_colored_points.h"
#include "renderer/objects/aabbox.h"
#include "math/math_types.h"
#include "renderer/gl_state.h"

#define max std::max

using jtil::math::Float3;
using std::string;
using std::runtime_error;
using renderer::objects::AABBox;

namespace renderer {
  const void* ColoredPointsVertex::pos_offset =
    reinterpret_cast<const void*>(offsetof(struct ColoredPointsVertex, pos));
  const void* ColoredPointsVertex::col_offset =
  reinterpret_cast<const void*>(offsetof(struct ColoredPointsVertex, col));
  
  GeometryColoredPoints::GeometryColoredPoints() :
  Geometry() {
    // vertices_ will be zero size by default
    // colors_ will be zero size by default
    synced_ = false;
  }

  GeometryColoredPoints::~GeometryColoredPoints() {
    if (synced_) {
      GLState::glsDeleteBuffers(1, &vbo_);
      GLState::glsDeleteVertexArrays(1, &vao_);
    }

    children_.clear();  // Explicitly delete all the children (calling destrs)
  }

  void GeometryColoredPoints::unsyncVAO() {
    if (synced_) {
      GLState::glsDeleteBuffers(1, &vbo_);
      GLState::glsDeleteVertexArrays(1, &vao_);
      synced_ = false;
    } else {
      throw std::runtime_error("unsyncVAO() - VAO not yet sunk!");
    }
  }

  void GeometryColoredPoints::addPoints(const float* xyz, const uint8_t* rgb,
                                        uint32_t num_points) {
    for (uint32_t i = 0; i < num_points; i++) {
      vertices_.pushBack(Float3(xyz[i * 3], xyz[i * 3 + 1], xyz[i * 3 + 2]));
      colors_.pushBack(Float3(static_cast<float>(rgb[i * 3])/255.0f,
                              static_cast<float>(rgb[i * 3 + 1])/255.0f,
                              static_cast<float>(rgb[i * 3 + 2])/255.0f));
    }
  }

  void GeometryColoredPoints::addPoint(const float* xyz, const float* rgb) {
    vertices_.pushBack(Float3(xyz[0], xyz[1], xyz[2]));
    colors_.pushBack(Float3(rgb[0], rgb[1], rgb[2]));
  }
  
  void GeometryColoredPoints::addPoints(const float* xyz, const float* rgb,
                                        uint32_t num_points) {
    for (uint32_t i = 0; i < num_points; i++) {
      vertices_.pushBack(Float3(xyz[i * 3], xyz[i * 3 + 1], xyz[i * 3 + 2]));
      colors_.pushBack(Float3(rgb[i * 3], rgb[i * 3 + 1],rgb[i * 3 + 2]));
    }
  }

  void GeometryColoredPoints::syncVAO() {
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
    ColoredPointsVertex dummy;  // just for sizeof
    static_cast<void>(dummy);  // Get rid of unreference local variable warning
    GLState::glsBufferData(GL_ARRAY_BUFFER,  // Target
                 vertices_.size() * sizeof(dummy),  // size
                 NULL,  // data --> Data is not initially copied
                 GL_STATIC_DRAW);  // usage

    // Copy the data into the vertex buffer
    ColoredPointsVertex* ptr = reinterpret_cast<ColoredPointsVertex*>(
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
      sizeof(struct ColoredPointsVertex), ColoredPointsVertex::pos_offset);
    setVertexAttribPointer(VERTEX_COL_LOC, 3, GL_FLOAT, GL_FALSE,
      sizeof(struct ColoredPointsVertex), ColoredPointsVertex::col_offset);

    synced_ = true;
  }

  void GeometryColoredPoints::bindVAO() {
    GLState::glsBindVertexArray(vao_);  // encapsulates all the vbo bindings
  }

  void GeometryColoredPoints::unbindVAO() {
    // Nothing to do
  }

  void GeometryColoredPoints::draw() {
    if (!synced_) {
      throw std::runtime_error("GeometryColoredPoints::draw() - "
        "ERROR: trying to draw an unsynced vbo object!");
    }

    bindVAO();  // Bind all required OpenGL resources

    GLState::glsDrawArrays(GL_POINTS, 0, synced_num_vertices_);
    
    unbindVAO();  // Unbind all required OpenGL resources
  }

  Geometry* GeometryColoredPoints::copy() {
    throw std::runtime_error(std::string("GeometryColoredPoints::copy() - "
      "ERROR: copy not yet implemented for this Geometry type"));
  }

}  // namespace renderer

