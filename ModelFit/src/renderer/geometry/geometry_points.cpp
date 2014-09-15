#include "renderer/geometry/geometry_points.h"
#include "renderer/objects/aabbox.h"
#include "math/math_types.h"
#include "renderer/gl_state.h"

#define max std::max

using jtil::math::Float3;
using std::string;
using std::runtime_error;
using renderer::objects::AABBox;

namespace renderer {
  const void* PointsVertex::pos_offset =
    reinterpret_cast<const void*>(offsetof(struct PointsVertex, pos));

  GeometryPoints::GeometryPoints() :
  Geometry() {
    // vertices_ will be zero size by default
    // colors_ will be zero size by default
    synced_ = false;
  }

  GeometryPoints::~GeometryPoints() {
    if (synced_) {
      GLState::glsDeleteBuffers(1, &vbo_);
      GLState::glsDeleteVertexArrays(1, &vao_);
    }

    children_.clear();  // Explicitly delete all the children (calling destrs)
  }
  
  void GeometryPoints::addPoint(const Float3& point) {
    vertices_.pushBack(point);
  }

  void GeometryPoints::addPoint(const float x, 
                                const float y, 
                                const float z) {
    vertices_.pushBack(Float3(x, y, z));
  }

  void GeometryPoints::addPoint(const float* xyz) {
    vertices_.pushBack(Float3(xyz[0], xyz[1], xyz[2]));
  }

  void GeometryPoints::addPoints(const float* xyz, uint32_t num_points) {
    for (uint32_t i = 0; i < num_points; i++) {
      vertices_.pushBack(Float3(xyz[i * 3], xyz[i * 3 + 1], xyz[i * 3 + 2]));
    }
  }

  void GeometryPoints::syncVAO() {
    if (synced_) {
      string err("syncVBO() - ERROR: dynamic VBOs not yet supported");
      throw std::runtime_error(err);
    }

    synced_num_vertices_ = vertices_.size();

    // Allocate a VAO
    GLState::glsGenVertexArrays(1, &vao_);
    // Bind our Vertex Array Object as the current used object
    GLState::glsBindVertexArray(vao_);
    // Allocate and assign two Vertex Buffer Objects to our handle
    GLState::glsGenBuffers(1, &vbo_);
    // Bind our first VBO as being the active buffer and storing vertex 
    // attributes (coordinates)
    GLState::glsBindBuffer(GL_ARRAY_BUFFER, vbo_);

    // Allocate a vertex buffer
    PointsVertex dummy;  // just for sizeof
    static_cast<void>(dummy);  // Get rid of unreference local variable warning
    GLState::glsBufferData(GL_ARRAY_BUFFER,  // Target
                 vertices_.size() * sizeof(dummy),  // size
                 NULL,  // data --> Data is not initially copied
                 GL_STATIC_DRAW);  // usage

    // Copy the data into the vertex buffer
    PointsVertex* ptr = reinterpret_cast<PointsVertex*>(
      GLState::glsMapBuffer(GL_ARRAY_BUFFER, GL_WRITE_ONLY));
    for (uint32_t i = 0; i < vertices_.size(); i++) {
      ptr[i].pos[0] = static_cast<GLfloat>(vertices_[i][0]);
      ptr[i].pos[1] = static_cast<GLfloat>(vertices_[i][1]);
      ptr[i].pos[2] = static_cast<GLfloat>(vertices_[i][2]);
    }
    GLState::glsUnmapBuffer(GL_ARRAY_BUFFER);

    setVertexAttribPointer(VERTEX_POS_LOC, 3, GL_FLOAT, false, 
      sizeof(struct PointsVertex), PointsVertex::pos_offset);

    synced_ = true;
  }

  void GeometryPoints::bindVAO() {
    GLState::glsBindVertexArray(vao_);  // encapsulates all the vbo bindings
  }

  void GeometryPoints::unbindVAO() {
    // Nothing to do
  }

  void GeometryPoints::draw() {
    if (!synced_) {
      throw std::runtime_error("GeometryPoints::draw() - " 
        "ERROR: trying to draw an unsynced vbo object!");
    }

    bindVAO();  // Bind all required OpenGL resources

    GLState::glsDrawArrays(GL_POINTS, 0, synced_num_vertices_);
    
    unbindVAO();  // Unbind all required OpenGL resources
  }

  Geometry* GeometryPoints::copy() {
    throw std::runtime_error("GeometryPoints::copy() - "
      "ERROR: copy not yet implemented for this Geometry type");
  }

}  // namespace renderer

