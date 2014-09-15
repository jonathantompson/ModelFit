#include "renderer/geometry/geometry_vertices.h"
#include "renderer/gl_state.h"

using jtil::math::Float3;
using std::string;
using std::runtime_error;

namespace renderer {
  const void* VerticesVertex::pos_offset = 
    reinterpret_cast<const void*>(offsetof(struct VerticesVertex, pos));

  GeometryVertices::GeometryVertices() :
  Geometry() {
    // vertices_ will be zero size by default
    synced_ = false;
  }

  GeometryVertices::~GeometryVertices() {
    if (synced_) {
      GLState::glsDeleteBuffers(1, &vbo_);
      GLState::glsDeleteVertexArrays(1, &vao_);
    }

    children_.clear();  // Explicitly delete all the children (calling destrs)
  }
  
  void GeometryVertices::addVertex(const Float3& vertex) {
    vertices_.pushBack(vertex);
  }

  void GeometryVertices::addVertex(const float x, 
                                      const float y, 
                                      const float z) {
    vertices_.pushBack(Float3(x, y, z));
  }

    void GeometryVertices::addVertex(const float* xyz) {
    vertices_.pushBack(Float3(xyz[0], xyz[1], xyz[2]));
  }


  void GeometryVertices::syncVAO() {
    if (synced_) {
      string err("syncVBO() - ERROR: dynamic VBOs not yet supported");
      throw std::runtime_error(err);
    }

    if ((vertices_.size() % 3) != 0) {
      string err = string("syncVBO() - ERROR: num vertices must be a "
        "multiple of 3");
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
    VerticesVertex dummy;  // just for sizeof
    static_cast<void>(dummy);  // Get rid of unreference local variable warning
    GLState::glsBufferData(GL_ARRAY_BUFFER,  // Target
                 vertices_.size() * sizeof(dummy),  // size
                 NULL,  // data --> Data is not initially copied
                 GL_STATIC_DRAW);  // usage

    // Copy the data into the vertex buffer
    VerticesVertex* ptr = reinterpret_cast<VerticesVertex*>(
      GLState::glsMapBuffer(GL_ARRAY_BUFFER, GL_WRITE_ONLY));
    for (uint32_t i = 0; i < vertices_.size(); i ++) {
      ptr[i].pos[0] = static_cast<GLfloat>(vertices_[i][0]);
      ptr[i].pos[1] = static_cast<GLfloat>(vertices_[i][1]);
      ptr[i].pos[2] = static_cast<GLfloat>(vertices_[i][2]);
    }
    GLState::glsUnmapBuffer(GL_ARRAY_BUFFER);

    setVertexAttribPointer(VERTEX_POS_LOC, 3, GL_FLOAT, false,
      sizeof(struct VerticesVertex), VerticesVertex::pos_offset);

    synced_ = true;
  }

  void GeometryVertices::bindVAO() {
    GLState::glsBindVertexArray(vao_);  // encapsulates all the vbo bindings
  }

  void GeometryVertices::unbindVAO() {
    // Nothing to do
  }

  void GeometryVertices::draw() {
    if (!synced_) {
      throw std::runtime_error("GeometryVertices::draw() - "
        "ERROR: trying to draw an unsynced vbo object!");
    }

    bindVAO();  // Bind all required OpenGL resources
    GLState::glsDrawArrays(GL_TRIANGLES, 0, synced_num_vertices_);
    unbindVAO();  // Unbind all required OpenGL resources
  }

  GeometryVertices* GeometryVertices::makeQuad() {
    GeometryVertices* ret_val = new GeometryVertices();
    ret_val->addVertex(vertices_quad_[0]);
    ret_val->addVertex(vertices_quad_[1]);
    ret_val->addVertex(vertices_quad_[2]);
    ret_val->addVertex(vertices_quad_[3]);
    ret_val->addVertex(vertices_quad_[4]);
    ret_val->addVertex(vertices_quad_[5]);
    ret_val->syncVAO();
    return ret_val;
  }

  const Float3 GeometryVertices::vertices_quad_[6] = {
    Float3(-1.0f, -1.0f, 0.0f),
    Float3( 1.0f, -1.0f, 0.0f),
    Float3(-1.0f,  1.0f, 0.0f),
    Float3(-1.0f,  1.0f, 0.0f),
    Float3( 1.0f, -1.0f, 0.0f),
    Float3( 1.0f,  1.0f, 0.0f),
  };

  Geometry* GeometryVertices::copy() {
    throw std::runtime_error("GeometryVertices::copy() - "
      "ERROR: copy not yet implemented for this Geometry type");
  }

}  // namespace renderer

