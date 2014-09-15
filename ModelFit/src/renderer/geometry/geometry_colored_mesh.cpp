#include <fstream>
#include <iostream>
#include "data_str/pair.h"
#include "renderer/geometry/geometry_colored_mesh.h"
#include "renderer/objects/aabbox.h"
#include "math/math_types.h"
#include "renderer/colors.h"
#include "assimp/Importer.hpp"      // C++ importer interface
#include "assimp/scene.h"           // Output data structure
#include "assimp/postprocess.h"     // Post processing flags
#include "fastlz/fastlz.h"
#include "renderer/gl_state.h"

#define max std::max

using jtil::math::Float3;
using std::wstring;
using std::runtime_error;
using std::runtime_error;
using std::string;
using std::cout;
using std::endl;
using renderer::objects::AABBox;
using jtil::data_str::Pair;

namespace renderer {
  const void* ColoredMeshVertex::pos_offset = 
    reinterpret_cast<const void*>(offsetof(struct ColoredMeshVertex, pos));
  const void* ColoredMeshVertex::norm_offset = 
    reinterpret_cast<const void*>(offsetof(struct ColoredMeshVertex, norm));
  const void* ColoredMeshVertex::col_offset = 
    reinterpret_cast<const void*>(offsetof(struct ColoredMeshVertex, col));

  GeometryColoredMesh::GeometryColoredMesh() :
  Geometry() {
    // vertices_ will be zero size by default
    // colors_ will be zero size by default
    synced_ = false;
    aabbox_ = new AABBox();
  }

  GeometryColoredMesh::~GeometryColoredMesh() {
    if (synced_) {
      GLState::glsDeleteBuffers(1, &vbo_);
      GLState::glsDeleteVertexArrays(1, &vao_);
    }
    if (aabbox_) {
      delete aabbox_;
      aabbox_ = NULL;
    }

    children_.clear();  // Explicitly delete all the children (calling destrs)
  }
  
  void GeometryColoredMesh::addVertex(const Float3& vertex) {
    vertices_.pushBack(vertex);
  }

  void GeometryColoredMesh::addVertex(const float x, 
                                      const float y, 
                                      const float z) {
    vertices_.pushBack(Float3(x, y, z));
  }

    void GeometryColoredMesh::addVertex(const float* xyz) {
    vertices_.pushBack(Float3(xyz[0], xyz[1], xyz[2]));
  }

  void GeometryColoredMesh::addNormal(const Float3& normal) {
    normals_.pushBack(normal);
  }

  void GeometryColoredMesh::addNormal(const float x, 
                                      const float y, 
                                      const float z) {
    normals_.pushBack(Float3(x, y, z));
  }

    void GeometryColoredMesh::addNormal(const float* xyz) {
    normals_.pushBack(Float3(xyz[0], xyz[1], xyz[2]));
  }
  
  void GeometryColoredMesh::addColor(const Float3& color) {
    colors_.pushBack(Float3(color[0], color[1], color[2]));
  }

  void GeometryColoredMesh::addColor(const float r, 
                                     const float g, 
                                     const float b) {
    colors_.pushBack(Float3(r, g, b));
  }
  void GeometryColoredMesh::addColor(const float* rgb) {
    colors_.pushBack(Float3(rgb[0], rgb[1], rgb[2]));
  }

  void GeometryColoredMesh::addFace(const uint32_t* v012) {
    indices_.pushBack(v012[0]);
    indices_.pushBack(v012[1]);
    indices_.pushBack(v012[2]);
  }

  void GeometryColoredMesh::addFace(const uint32_t v0, const uint32_t v1, 
    const uint32_t v2) {
    indices_.pushBack(v0);
    indices_.pushBack(v1);
    indices_.pushBack(v2);
  }

  void GeometryColoredMesh::unsyncVAO() {
    if (synced_) {
      GLState::glsDeleteBuffers(1, &vbo_);
      GLState::glsDeleteVertexArrays(1, &vao_);
      synced_ = false;
      synced_num_indices_ = 0;
      synced_num_vertices_ = 0;
    } else {
      throw std::runtime_error(string("GeometryColoredMesh::unsyncVAO() - ") +
        string("ERROR: VAO is not synced"));
    }
  }

  void GeometryColoredMesh::syncVAO() {
    if (synced_) {
      string err = string("syncVBO() - ERROR: dynamic VBOs supported.  "
        "call unsyncVBO() first.");
      throw std::runtime_error(err);
    }

    if (colors_.size() != vertices_.size() || 
        colors_.size() != normals_.size()) {
      string err = string("syncVBO() - ERROR: vertex, color and normal "
        "vectors must be the same size!");
      throw std::runtime_error(err);
    }

    if (indices_.size() == 0) {
      if ((vertices_.size() % 3) != 0) {
        string err = string("syncVBO() - ERROR: num vertices must be a "
          "multiple of 3 when not using the index array");
        throw std::runtime_error(err);
      }
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
    ColoredMeshVertex dummy;  // just for sizeof
    static_cast<void>(dummy);  // Get rid of unreference local variable warning
    GLState::glsBufferData(GL_ARRAY_BUFFER,  // Target
                 vertices_.size() * sizeof(dummy),  // size
                 NULL,  // data --> Data is not initially copied
                 GL_STATIC_DRAW);  // usage


    // Copy the data into the vertex buffer
    ColoredMeshVertex* ptr = reinterpret_cast<ColoredMeshVertex*>(
      GLState::glsMapBuffer(GL_ARRAY_BUFFER, GL_WRITE_ONLY));

    for (uint32_t i = 0; i < vertices_.size(); i++) {
      ptr[i].pos[0] = static_cast<GLfloat>(vertices_[i][0]);
      ptr[i].pos[1] = static_cast<GLfloat>(vertices_[i][1]);
      ptr[i].pos[2] = static_cast<GLfloat>(vertices_[i][2]);
      ptr[i].norm[0] = static_cast<GLfloat>(normals_[i][0]);
      ptr[i].norm[1] = static_cast<GLfloat>(normals_[i][1]);
      ptr[i].norm[2] = static_cast<GLfloat>(normals_[i][2]);
      ptr[i].col[0] = static_cast<GLfloat>(colors_[i][0]);
      ptr[i].col[1] = static_cast<GLfloat>(colors_[i][1]);
      ptr[i].col[2] = static_cast<GLfloat>(colors_[i][2]);
    }
    GLState::glsUnmapBuffer(GL_ARRAY_BUFFER);


    setVertexAttribPointer(VERTEX_POS_LOC, 3, GL_FLOAT, false, 
      sizeof(struct ColoredMeshVertex), ColoredMeshVertex::pos_offset);
    setVertexAttribPointer(VERTEX_NOR_LOC, 3, GL_FLOAT, false,
      sizeof(struct ColoredMeshVertex), ColoredMeshVertex::norm_offset);
    setVertexAttribPointer(VERTEX_COL_LOC, 3, GL_FLOAT, false,
      sizeof(struct ColoredMeshVertex), ColoredMeshVertex::col_offset);

    // Allocate an index buffer if we're using it
    if (indices_.size() != 0) {
      GLState::glsGenBuffers(1, &ibo_);

      GLState::glsBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ibo_);


      // Allocate the space for the index buffer
      GLState::glsBufferData(GL_ELEMENT_ARRAY_BUFFER, 
        indices_.size()*sizeof(indices_[0]), indices_.at(0), GL_STATIC_DRAW);

      synced_num_indices_ = indices_.size();
    } else {
      ibo_ = 0;
      synced_num_indices_ = 0;
    }

    aabbox_->init(&vertices_);

    synced_ = true;
  }

  void GeometryColoredMesh::bindVAO() {
    GLState::glsBindVertexArray(vao_);  // encapsulates all the vbo bindings

  }

  void GeometryColoredMesh::unbindVAO() {
    // Nothing to do
  }

  void GeometryColoredMesh::draw() {
    if (!synced_) {
      throw std::runtime_error(string("GeometryColoredMesh::draw() - "
        "ERROR: trying to draw an unsynced vbo object!"));
    }

    bindVAO();  // Bind all required OpenGL resources
    if (synced_num_indices_ == 0) {  // We're drawing without an index buffer
      GLState::glsDrawArrays(GL_TRIANGLES, 0, synced_num_vertices_);

    } else {
      GLState::glsDrawElements(GL_TRIANGLES, synced_num_indices_, GL_UNSIGNED_INT, 
        static_cast<GLvoid*>(NULL));
    }
    unbindVAO();  // Unbind all required OpenGL resources
  }

  void GeometryColoredMesh::addTriangle(GeometryColoredMesh* mesh, 
                                        const Float3& color,
                                        int i0, int i1, int i2, 
                                        const Float3* vert) {
    Float3 v1;
    Float3 v2;
    Float3 norm;
    Float3::sub(v1, vert[i0], vert[i1]);
    Float3::sub(v2, vert[i2], vert[i1]);
    Float3::cross(norm, v1, v2);
    norm.normalize();
    mesh->addVertex(vert[i0]);
    mesh->addVertex(vert[i1]);
    mesh->addVertex(vert[i2]);
    mesh->addNormal(norm);
    mesh->addNormal(norm);
    mesh->addNormal(norm);
    mesh->addColor(color);
    mesh->addColor(color);
    mesh->addColor(color);
  }

  // Geometry builder methods --> Make new geometry primatives
  GeometryColoredMesh* GeometryColoredMesh::makeCube(
    const Float3& color) {
    GeometryColoredMesh* ret = new GeometryColoredMesh();
    ret->mat_.identity();

    addTriangle(ret, color, 2, 1, 0, cube_vertices_);  // Top face 1
    addTriangle(ret, color, 3, 2, 0, cube_vertices_);  // Top face 2
    addTriangle(ret, color, 6, 2, 3, cube_vertices_);  // Right face 1
    addTriangle(ret, color, 7, 6, 3, cube_vertices_);  // Right face 2
    addTriangle(ret, color, 1, 5, 0, cube_vertices_);  // Left face 1
    addTriangle(ret, color, 5, 4, 0, cube_vertices_);  // Left face 2
    addTriangle(ret, color, 7, 3, 0, cube_vertices_);  // Front face 1
    addTriangle(ret, color, 4, 7, 0, cube_vertices_);  // Front face 2
    addTriangle(ret, color, 5, 1, 2, cube_vertices_);  // Back face 1
    addTriangle(ret, color, 6, 5, 2, cube_vertices_);  // Back face 2
    addTriangle(ret, color, 6, 7, 4, cube_vertices_);  // Bottom face 1
    addTriangle(ret, color, 5, 6, 4, cube_vertices_);  // Bottom face 2

    ret->syncVAO();

    return ret;
  }

  GeometryColoredMesh* GeometryColoredMesh::makeCube(
    const jtil::math::Float3& ctop, const jtil::math::Float3& cbottom, 
    const jtil::math::Float3& cleft, const jtil::math::Float3& cright, 
    const jtil::math::Float3& cfront, const jtil::math::Float3& cback) {
    GeometryColoredMesh* ret = new GeometryColoredMesh();
    ret->mat_.identity();

    addTriangle(ret, ctop, 2, 1, 0, cube_vertices_);  // Top face 1
    addTriangle(ret, ctop, 3, 2, 0, cube_vertices_);  // Top face 2
    addTriangle(ret, cright, 6, 2, 3, cube_vertices_);  // Right face 1
    addTriangle(ret, cright, 7, 6, 3, cube_vertices_);  // Right face 2
    addTriangle(ret, cleft, 1, 5, 0, cube_vertices_);  // Left face 1
    addTriangle(ret, cleft, 5, 4, 0, cube_vertices_);  // Left face 2
    addTriangle(ret, cfront, 7, 3, 0, cube_vertices_);  // Front face 1
    addTriangle(ret, cfront, 4, 7, 0, cube_vertices_);  // Front face 2
    addTriangle(ret, cback, 5, 1, 2, cube_vertices_);  // Back face 1
    addTriangle(ret, cback, 6, 5, 2, cube_vertices_);  // Back face 2
    addTriangle(ret, cbottom, 6, 7, 4, cube_vertices_);  // Bottom face 1
    addTriangle(ret, cbottom, 5, 6, 4, cube_vertices_);  // Bottom face 2

    ret->syncVAO();

    return ret;
  }

  GeometryColoredMesh* GeometryColoredMesh::makeCubeRainbow() {
    GeometryColoredMesh* ret = new GeometryColoredMesh();
    ret->mat_.identity();

    addTriangle(ret, rainbow_colors_[0], 2, 1, 0, cube_vertices_);
    addTriangle(ret, rainbow_colors_[1], 3, 2, 0, cube_vertices_);
    addTriangle(ret, rainbow_colors_[2], 6, 2, 3, cube_vertices_);
    addTriangle(ret, rainbow_colors_[3], 7, 6, 3, cube_vertices_);
    addTriangle(ret, rainbow_colors_[4], 1, 5, 0, cube_vertices_);
    addTriangle(ret, rainbow_colors_[5], 5, 4, 0, cube_vertices_);
    addTriangle(ret, rainbow_colors_[6], 7, 3, 0, cube_vertices_);
    addTriangle(ret, rainbow_colors_[7], 4, 7, 0, cube_vertices_);
    addTriangle(ret, rainbow_colors_[8], 5, 1, 2, cube_vertices_);
    addTriangle(ret, rainbow_colors_[0], 6, 5, 2, cube_vertices_);
    addTriangle(ret, rainbow_colors_[1], 6, 7, 4, cube_vertices_);
    addTriangle(ret, rainbow_colors_[2], 5, 6, 4, cube_vertices_);

    ret->syncVAO();

    return ret;
  }

  GeometryColoredMesh* GeometryColoredMesh::makePyramid(
    const Float3& color) {
    GeometryColoredMesh* ret = new GeometryColoredMesh();
    ret->mat_.identity();

    addTriangle(ret, color, 2, 3, 1, pyramid_vertices_);  // Bottom face 1
    addTriangle(ret, color, 3, 4, 1, pyramid_vertices_);  // Bottom face 2
    addTriangle(ret, color, 1, 4, 0, pyramid_vertices_);  // Front Diagonal
    addTriangle(ret, color, 4, 3, 0, pyramid_vertices_);  // Right Diagonal
    addTriangle(ret, color, 3, 2, 0, pyramid_vertices_);  // Back Diagonal
    addTriangle(ret, color, 2, 1, 0, pyramid_vertices_);  // Left Diagonal

    ret->syncVAO();

    return ret;
  }

  GeometryColoredMesh* GeometryColoredMesh::makePyramidRainbow() {
    GeometryColoredMesh* ret = new GeometryColoredMesh();
    ret->mat_.identity();

    addTriangle(ret, rainbow_colors_[0], 2, 3, 1, pyramid_vertices_);
    addTriangle(ret, rainbow_colors_[1], 3, 4, 1, pyramid_vertices_);
    addTriangle(ret, rainbow_colors_[2], 1, 4, 0, pyramid_vertices_);
    addTriangle(ret, rainbow_colors_[3], 4, 3, 0, pyramid_vertices_);
    addTriangle(ret, rainbow_colors_[4], 3, 2, 0, pyramid_vertices_);
    addTriangle(ret, rainbow_colors_[5], 2, 1, 0, pyramid_vertices_);

    ret->syncVAO();

    return ret;
  }

  const Float3 GeometryColoredMesh::cube_vertices_[8] = {
    Float3(-1.0f, +1.0f, -1.0f),    // top, left, front
    Float3(-1.0f, +1.0f, +1.0f),    // top, left, back
    Float3(+1.0f, +1.0f, +1.0f),    // top, right, back
    Float3(+1.0f, +1.0f, -1.0f),    // top, right, front
    Float3(-1.0f, -1.0f, -1.0f),    // bottom, left, front
    Float3(-1.0f, -1.0f, +1.0f),    // bottom, left, back
    Float3(+1.0f, -1.0f, +1.0f),    // bottom, right, back
    Float3(+1.0f, -1.0f, -1.0f)     // bottom, right, front
  };

  const Float3 GeometryColoredMesh::pyramid_vertices_[5] = {
    Float3(+0.0f, +1.0f, +0.0f),    // top
    Float3(-1.0f, -1.0f, -1.0f),    // bottom, left, front
    Float3(-1.0f, -1.0f, +1.0f),    // bottom, left, back
    Float3(+1.0f, -1.0f, +1.0f),    // bottom, right, back
    Float3(+1.0f, -1.0f, -1.0f)     // bottom, right, front
  };

  const Float3 GeometryColoredMesh::rainbow_colors_[9] = {
    Float3(1.0f, 0.0f, 0.0f),       // red
    Float3(1.0f, 0.5f, 0.0f),       // orange
    Float3(1.0f, 1.0f, 0.0f),       // yellow
    Float3(0.0f, 1.0f, 0.0f),       // green
    Float3(0.0f, 0.0f, 1.0f),       // blue
    Float3(0.435294f, 0.0f, 1.0f),  // indigo
    Float3(0.560784f, 0.0f, 1.0f),  // violet
    Float3(1.0f, 1.0f, 1.0f),       // white
    Float3(0.0f, 0.0f, 0.0f)        // black
  };

  // theta is angle from top [0, pi], phi is angle along slice [0, 2pi]
  // http://en.wikipedia.org/wiki/Spherical_coordinate_system
  void SphericalToCartesean(Float3* xyz, float r, float phi, float theta ) {
    xyz->m[0] = r * sinf(theta) * cosf(phi);
    xyz->m[1] = r * sinf(theta) * sinf(phi);
    xyz->m[2] = r * cosf(theta);
  }

  void GetRotatedAxis(Float3* vec_out, Float3* vec_in, double angle, const Float3& axis) {
    if(angle==0.0) {
      vec_out->set(*vec_in);
      return;
    }

    Float3 u(axis);
    u.normalize();

    Float3 rotMatrixRow0, rotMatrixRow1, rotMatrixRow2;

    float sinAngle = static_cast<float>(sin(M_PI * angle / 180.0));
    float cosAngle = static_cast<float>(cos(M_PI * angle / 180.0));
    float oneMinusCosAngle = 1.0f - cosAngle;

    rotMatrixRow0[0] = (u[0]) * (u[0]) + cosAngle*(1-(u[0])*(u[0]));
    rotMatrixRow0[1] = (u[0]) * (u[1])*(oneMinusCosAngle) - sinAngle*u[2];
    rotMatrixRow0[2] = (u[0]) * (u[2])*(oneMinusCosAngle) + sinAngle*u[1];

    rotMatrixRow1[0] = (u[0]) * (u[1])*(oneMinusCosAngle) + sinAngle*u[2];
    rotMatrixRow1[1] = (u[1]) * (u[1]) + cosAngle*(1-(u[1])*(u[1]));
    rotMatrixRow1[2] = (u[1]) * (u[2])*(oneMinusCosAngle) - sinAngle*u[0];

    rotMatrixRow2[0] = (u[0]) * (u[2])*(oneMinusCosAngle) - sinAngle*u[1];
    rotMatrixRow2[1] = (u[1]) * (u[2])*(oneMinusCosAngle) + sinAngle*u[0];
    rotMatrixRow2[2] = (u[2]) * (u[2]) + cosAngle*(1-(u[2])*(u[2]));

    (*vec_out)[0] = Float3::dot(*vec_in, rotMatrixRow0);
    (*vec_out)[1] = Float3::dot(*vec_in, rotMatrixRow1);
    (*vec_out)[2] = Float3::dot(*vec_in, rotMatrixRow2);
  }

  GeometryColoredMesh* GeometryColoredMesh::makeTorusKnot(
    const Float3& color, uint32_t turns, uint32_t slices, 
    uint32_t stacks) {
    GeometryColoredMesh* ret = new GeometryColoredMesh();
    ret->mat_.identity();

    // Calculate the total number of vertices and reserve space
    uint32_t num_vertices=stacks*slices;
    ret->vertices_.capacity(num_vertices);
    ret->vertices_.resize(num_vertices);

    // Calculate the position of the centre of each ring
    Float3* ring_centres = new Float3[stacks];

    for (int i = 0; i < static_cast<int>(stacks); ++i) {
      // Calculate the value of the parameter t at this point
      double t = i * 2 * (M_PI / stacks);

      // Calculate the position
      ring_centres[i].set(
        static_cast<float>((1.0+0.3*cos(turns*t))*cos(2*t)),
        static_cast<float>((1.0+0.3*cos(turns*t))*0.3f*sin(turns*t)),
        static_cast<float>((1.0+0.3*cos(turns*t))*sin(2*t)));
    }

    // Loop through the rings
    for (int i = 0; i < static_cast<int>(stacks); ++i) {
      // Loop through the vertices making up this ring
      for (int j = 0; j < static_cast<int>(slices); ++j) {
        // Calculate the number of this vertex
        int vertex_number = i * static_cast<int>(slices) + j;

        // Get the vector from the centre of this ring to the centre of the next
        Float3 tangent;
        if (i == (static_cast<int>(stacks) - 1)) {
          Float3::sub(tangent, ring_centres[0], ring_centres[i]);
        } else {
          Float3::sub(tangent, ring_centres[i+1], ring_centres[i]);
        }

        // Calculate the vector perpendicular to the tangent, pointing 
        // approximately in the positive Y direction
        static Float3 yaxis(0.0f, 1.0f, 0.0f);
        Float3 temp1;
        Float3::cross(temp1, yaxis, tangent);
        Float3 temp2;
        Float3::cross(temp2, tangent, temp1);
        temp2.normalize();
        Float3::scale(temp2, 0.2f);

        // Rotate this about the tangent vector to form the ring
        Float3 temp2_rotated;
        GetRotatedAxis(&temp2_rotated, &temp2, j*360.0f/slices, tangent);

        Float3::add(ret->vertices_[vertex_number], ring_centres[i], 
          temp2_rotated);
      }
    }

    // Calculate the total number of indices and reserve space
    uint32_t num_indices = 6 * stacks * slices;
    ret->indices_.capacity(num_indices);
    ret->indices_.resize(num_indices);

    // Calculate the indices
    for (uint32_t i = 0; i < stacks; ++i) {
      for (uint32_t j = 0; j < slices; ++j) {
        // Get the index for the 4 vertices around this "quad"
        uint32_t quad_indices[4];

        quad_indices[0] = i * slices + j;

        if (j != slices - 1) {
          quad_indices[1] = i * slices + j + 1;
        } else {
          quad_indices[1] = i * slices;
        }

        if (i != stacks - 1) {
          quad_indices[2] = (i + 1) * slices + j;
        } else {
          quad_indices[2]=j;
        }

        if (i != stacks - 1) {
          if (j != slices - 1) {
            quad_indices[3] = (i + 1) * slices + j + 1;
          } else {
            quad_indices[3] = (i + 1) * slices;
          }
        } else {
          if (j != slices - 1) {
            quad_indices[3] = j + 1;
          } else {
            quad_indices[3] = 0;
          }
        }

        ret->indices_[(i*slices+j)*6] = quad_indices[0];
        ret->indices_[(i*slices+j)*6+1] = quad_indices[2];
        ret->indices_[(i*slices+j)*6+2] = quad_indices[1];

        ret->indices_[(i*slices+j)*6+3] = quad_indices[3];
        ret->indices_[(i*slices+j)*6+4] = quad_indices[1];
        ret->indices_[(i*slices+j)*6+5] = quad_indices[2];
      }
    }

    ret->normals_.capacity(num_vertices);
    ret->normals_.resize(num_vertices);

    // Clear the normals
    for (uint32_t i = 0; i < num_vertices; ++i) {
      ret->normals_[i].zeros();
    }

    // Loop through the triangles adding the normal to each vertex
    Float3 vec1;
    Float3 vec2;
    Float3 cur_normal;
    for (uint32_t i = 0; i < num_indices; i += 3) {
      uint32_t p0 = ret->indices_[i];
      uint32_t p1 = ret->indices_[i+1];
      uint32_t p2 = ret->indices_[i+2];

      // Calculate the normal for this triangle
      Float3::sub(vec1, ret->vertices_[p0], ret->vertices_[p1]);
      Float3::sub(vec2, ret->vertices_[p2], ret->vertices_[p1]);
      Float3::cross(cur_normal, vec1, vec2);
      cur_normal.normalize();

      // Add this to each of its vertices
      Float3::add(ret->normals_[p0], ret->normals_[p0], cur_normal);
      Float3::add(ret->normals_[p1], ret->normals_[p1], cur_normal);
      Float3::add(ret->normals_[p2], ret->normals_[p2], cur_normal);
    }

    //Normalize the normals
    for (uint32_t i = 0; i < num_vertices; ++i) {
      ret->normals_[i].normalize();
    }

    //Delete the ringCentres array
    if (ring_centres) {
      delete[] ring_centres;
      ring_centres = NULL;
    }

    // Set the color array
    ret->colors_.capacity(num_vertices);
    ret->colors_.resize(num_vertices);
    for (uint32_t i = 0; i < num_vertices; ++i) {
      ret->colors_[i].set(color);
    }

    ret->syncVAO();
    return ret;
  }

  // This function calculates the outside radius of the geometry for a 
  // sphere with a given number of slices, stacks and inside radius.  Very
  // useful to know how far a discrete sphere extends beyond a perfect sphere.
  float GeometryColoredMesh::calcSphereOutsideRadius(uint32_t n_stacks, 
    uint32_t n_slices, float inside_radius) {
    if(n_stacks < 3) {
      throw std::runtime_error(string("GeometryColoredMesh::") +
        string("calcSphereOutsideRadius() - Error: n_stacks < 3"));
    }
    if(n_slices < 4) {
      throw std::runtime_error(string("GeometryColoredMesh::") +
        string("calcSphereOutsideRadius()- Error: n_slices < 4"));
    }

    float slice_seperation_angle = 2.0f * static_cast<float>(M_PI) / 
        static_cast<float>(n_slices);
    float slice_outside_rad = inside_radius / 
      cosf(slice_seperation_angle * 0.5f); // for vertex points

    float stack_seperation_angle = 1.0f * static_cast<float>(M_PI) / 
      static_cast<float>(n_stacks - 1);
    float stack_outside_radius = inside_radius / 
      cosf(stack_seperation_angle * 0.5f); // for vertex points

    return max(slice_outside_rad, stack_outside_radius);
  }

  GeometryColoredMesh* GeometryColoredMesh::makeSphere(uint32_t n_stacks, 
    uint32_t n_slices, float inside_radius, const Float3& color) {
    GeometryColoredMesh* ret = new GeometryColoredMesh();
    Float3 center(0, 0, 0);
    makeSphere(ret, n_stacks, n_slices, inside_radius, center, color);
    return ret;
  }

  void GeometryColoredMesh::makeSphere(GeometryColoredMesh* ret, 
    uint32_t n_stacks, uint32_t n_slices, float inside_radius, 
    const Float3& center, const Float3& color) {
    if(n_stacks < 3) {
      throw std::runtime_error(string("GeometryColoredMesh::makeSphere()") +
        string("- Error: n_stacks < 3"));
    }
    if(n_slices < 4) {
      throw std::runtime_error(string("GeometryColoredMesh::makeSphere()") +
        string("- Error: n_slices < 4"));
    }

    // bottom and top stacks only have 1 point each
    uint32_t n_vert = max((n_stacks - 2), static_cast<uint32_t>(0)) *
      n_slices + 2;
    ret->vertices_.capacity(n_vert);
    ret->vertices_.resize(n_vert);
    ret->normals_.capacity(n_vert);
    ret->normals_.resize(n_vert);

    uint32_t n_ind = (2 * n_slices * 3) + max((n_stacks - 3),
      static_cast<uint32_t>(0)) * (2 * n_slices * 3);
    ret->indices_.capacity(n_ind);
    ret->indices_.resize(n_ind);

    float slice_seperation_angle = 2.0f * static_cast<float>(M_PI) /
      static_cast<float>(n_slices);

    float stack_seperation_angle = 1.0f * static_cast<float>(M_PI) / 
      static_cast<float>(n_stacks - 1);

    float outside_radius = calcSphereOutsideRadius(n_stacks, n_slices, 
      inside_radius);

    // classic spherical coords: - theta = azimuth angle from top
    //                           - phi = zenith angle along slice
    float phi; float theta; 
    uint32_t cur_vertex = 0;

    // top first
    phi = 0; theta = 0;
    SphericalToCartesean(ret->vertices_.at(cur_vertex), 
      outside_radius, phi, theta); 
    // Just point the normal outwards
    Float3::normalize(ret->normals_[cur_vertex], ret->vertices_[cur_vertex]);
    ret->vertices_.at(cur_vertex)->accum(center.m);
    cur_vertex++;

    // Intermediate stacks
    for (uint32_t i = 1; i < (n_stacks - 1); i++) {
      theta = i * stack_seperation_angle;  // [0, pi]
      for (uint32_t j = 0; j < n_slices; j++) {
        phi = j * slice_seperation_angle;  // [0, 2*pi]
        SphericalToCartesean(ret->vertices_.at(cur_vertex), 
          outside_radius, phi, theta); 
        // Just point the normal outwards
        Float3::normalize(ret->normals_[cur_vertex], ret->vertices_[cur_vertex]);
        ret->vertices_.at(cur_vertex)->accum(center.m);
        cur_vertex++;
      }
    }

    // bottom
    phi = 0; theta = static_cast<float>(M_PI);
    SphericalToCartesean(ret->vertices_.at(cur_vertex), 
      outside_radius, phi, theta); 
    // Just point the normal outwards
    Float3::normalize(ret->normals_[cur_vertex], ret->vertices_[cur_vertex]);
    ret->vertices_.at(cur_vertex)->accum(center.m);
    cur_vertex++;

#if defined(DEBUG) || defined(_DEBUG)
    if(cur_vertex != n_vert) {
      throw std::runtime_error(string("GeometryColoredMesh::makeSphere() -") +
        string(" Internal Error: Didn't create enough verticies"));
    }
#endif

    // Top Indices
    uint32_t cur_index = 0;
    for (uint32_t j = 0; j < (n_slices - 1); j ++) {
      ret->indices_[cur_index] = 0; // Top Vertex
      cur_index++;
      ret->indices_[cur_index] = j + 2;
      cur_index++;
      // First vertex is the top, so first slice vertex starts at 1
      ret->indices_[cur_index] = j + 1; 
      cur_index++;
    }
    ret->indices_[cur_index] = 0; // Top Vertex
    cur_index++;
    ret->indices_[cur_index] = 1; 
    cur_index++;
    // Back to the first vertex
    ret->indices_[cur_index] = n_slices; 
    cur_index++;

    // Intermediate indices
    for (uint32_t i = 1; i < (n_stacks - 2); i++) {
      uint32_t cur_stack_start_index = 1 + (i - 1) * n_slices;
      uint32_t next_stack_start_index = 1 + i * n_slices;
      for (uint32_t j = 0; j < (n_slices - 1); j++) {
        ret->indices_[cur_index] = cur_stack_start_index + j;
        cur_index++;
        ret->indices_[cur_index] = cur_stack_start_index + j + 1;
        cur_index++;
        ret->indices_[cur_index] = next_stack_start_index + j;
        cur_index++;
        ret->indices_[cur_index] = cur_stack_start_index + j + 1;
        cur_index++;
        ret->indices_[cur_index] = next_stack_start_index + j + 1;
        cur_index++;
        ret->indices_[cur_index] = next_stack_start_index + j;
        cur_index++;
      }
      ret->indices_[cur_index] = cur_stack_start_index + (n_slices-1);
      cur_index++;
      ret->indices_[cur_index] = cur_stack_start_index;  // Back to first vert
      cur_index++;
      ret->indices_[cur_index] = next_stack_start_index + (n_slices-1);
      cur_index++;
      ret->indices_[cur_index] = cur_stack_start_index;  // Back to first vert
      cur_index++;
      ret->indices_[cur_index] = next_stack_start_index;  // Back to first vert
      cur_index++;
      ret->indices_[cur_index] = next_stack_start_index + (n_slices-1);
      cur_index++;
    }

    // Bottom Indices
    uint32_t cur_stack_start_index = ( n_stacks - 3 ) * n_slices + 1;
    for (uint32_t j = 0; j < ( n_slices - 1); j++) {
      // First vertex is the top, so first slice vertex starts at 1
      ret->indices_[cur_index] = cur_stack_start_index + j; 
      cur_index++;
      ret->indices_[cur_index] = cur_stack_start_index + j + 1;
      cur_index++;
      ret->indices_[cur_index] = n_vert-1;  // Bottom Vertex
      cur_index++;
    }
    ret->indices_[cur_index] = cur_stack_start_index + n_slices - 1;
    cur_index++;
    ret->indices_[cur_index] = cur_stack_start_index;  // Back to first vert
    cur_index++;
    ret->indices_[cur_index] = n_vert-1; // Top Vertex
    cur_index++;

#if defined(DEBUG) || defined(_DEBUG)
    if(cur_index != n_ind) {
      throw std::runtime_error(string("GeometryColoredMesh::makeSphere() -") +
        string(" Internal Error: Didn't create enough indices"));
    }
#endif

    ret->colors_.capacity(n_vert);
    ret->colors_.resize(n_vert);
    for (uint32_t i = 0; i < n_vert; ++i) {
      ret->colors_[i].set(color);
    }

    ret->syncVAO();
  }

  // This function calculates the outside radius of the geometry for a 
  // cone with a given number of slices and inside radius.  Very
  // useful to know how far a discrete cone extends beyond a perfect cone.
  float GeometryColoredMesh::calcConeOutsideRadius(uint32_t n_slices, 
    float inside_radius) {
    if(n_slices < 3) {
      throw std::runtime_error(string("GeometryColoredMesh::") +
        string("calcConeOutsideRadius()- Error: n_slices < 3"));
    }

    float seperation_angle = 2.0f * static_cast<float>(M_PI) / 
      static_cast<float>(n_slices);
    return inside_radius / cos(seperation_angle * 0.5f);
  }

  GeometryColoredMesh* GeometryColoredMesh::makeCone(uint32_t n_slices, 
    float height, float inside_radius, const Float3& color) {
    GeometryColoredMesh* ret = new GeometryColoredMesh();

    if(n_slices < 3) {
      throw std::runtime_error(string("GeometryColoredMesh::makeCone()") +
        string("- Error: n_slices < 3"));
    }
   

    uint32_t n_vert = n_slices + 1 + // base vertices
                      n_slices * 2;  // Each side needs a top vert for the norm
    ret->vertices_.capacity(n_vert);
    ret->vertices_.resize(n_vert);
    ret->normals_.capacity(n_vert);
    ret->normals_.resize(n_vert);

    uint32_t n_ind = (n_slices * 3) + (n_slices * 3);
    ret->indices_.capacity(n_ind);
    ret->indices_.resize(n_ind);

    uint32_t cur_vertex = 0;
    float seperation_angle = 2.0f * static_cast<float>(M_PI) / 
      static_cast<float>(n_slices);
    float outside_rad = calcConeOutsideRadius(n_slices, inside_radius);

    // Define the base center vertex
    ret->normals_.at(cur_vertex)->set(0.0f, 1.0f, 0.0f);
    ret->vertices_.at(cur_vertex)->set(0.0f, height, 0.0f);
    cur_vertex ++;

    // Define the base indices
    for (uint32_t i = 0; i < n_slices; i++) {
      float cur_angle = seperation_angle * static_cast<float>(i);
      ret->normals_.at(cur_vertex)->set(0.0f, 1.0f, 0.0f);
      ret->vertices_.at(cur_vertex)->set(outside_rad * cosf(cur_angle), height, 
        outside_rad * sinf(cur_angle));
      cur_vertex ++;
    }

    Float3 cur_base_point, next_base_point, cur_normal, v1, v2;
    Float3 top_point(0, 0, 0);
    float cur_angle, tangent_angle;

    // Define the cone side and top vertices
    for (uint32_t i = 0; i < n_slices; i++) {
      // base
      cur_angle = seperation_angle * static_cast<float>(i);
      cur_base_point.set(outside_rad * cosf(cur_angle), height, 
        outside_rad * sinf(cur_angle));
      ret->vertices_.at(cur_vertex)->set(cur_base_point);
      tangent_angle = cur_angle + 
        (static_cast<float>(M_PI) / 2.0f);  // Tangent is 90deg away
      v1.set(cosf(tangent_angle), 0.0f, sinf(tangent_angle));  // length 1!
      Float3::normalize(v2, cur_base_point);
      Float3::scale(v2, -1.0f);  // vector from cur position --> Top (0,0,0)
      Float3::cross(cur_normal, v1, v2 );
      cur_normal.normalize();
      ret->normals_.at(cur_vertex)->set(cur_normal);
      cur_vertex ++;
      
      // next base (there is some re-work here, but that's OK).
      cur_angle = seperation_angle * static_cast<float>(i+1);
      next_base_point.set(outside_rad * cosf(cur_angle), height,
        outside_rad * sinf(cur_angle));

      // Treat the top point as
      ret->vertices_.at(cur_vertex)->set(top_point);
      Float3::sub(v1, cur_base_point, top_point);
      Float3::sub(v2, next_base_point, top_point);
      v1.normalize();
      v2.normalize();
      Float3::cross(cur_normal, v1, v2 );
      cur_normal.normalize();
      ret->normals_.at(cur_vertex)->set(cur_normal);
      cur_vertex ++;
    }

#if defined(DEBUG) || defined(_DEBUG)
    if(cur_vertex != n_vert) {
      throw std::runtime_error(string("GeometryColoredMesh::makeCone() -") +
        string(" Internal Error: Didn't create enough vertices"));
    }
#endif

    // Now define the base indices
    uint32_t cur_index = 0;
    for (uint32_t i = 1; i < n_slices; i++)
    {
      ret->indices_[cur_index] = i;
      cur_index ++;
      ret->indices_[cur_index] = i + 1;
      cur_index ++;
      ret->indices_[cur_index] = 0; // Center
      cur_index ++;
    }
    // Now define the last triangle in the base
    ret->indices_[cur_index] = n_slices;
    cur_index ++;
    ret->indices_[cur_index] = 1;
    cur_index ++;
    ret->indices_[cur_index] = 0; // Center
    cur_index ++;

    // Define the cone side indices
    for (uint32_t i = 0; i < (n_slices-1); i++) {
      ret->indices_[cur_index] = n_slices + 1 + (2*i); // base point 1
      cur_index ++;
      ret->indices_[cur_index] = n_slices + 1 + (2*i + 1); // top
      cur_index ++;
      ret->indices_[cur_index] = n_slices + 1 + (2*i + 2);  // base point 2
      cur_index ++;
    }
    // Now define the last triangle on the side
    ret->indices_[cur_index] = n_slices + 1 + (2*(n_slices-1)); // base point 1
    cur_index ++;
    ret->indices_[cur_index] = n_slices + 1 + (2*(n_slices-1) + 1); // top
    cur_index ++;
    ret->indices_[cur_index] = n_slices + 1; // Back to the start
    cur_index ++;


#if defined(DEBUG) || defined(_DEBUG)
    if(cur_index != n_ind) {
      throw std::runtime_error(string("GeometryColoredMesh::makeCone() -") +
        string(" Internal Error: Didn't create enough indices"));
    }
#endif

    ret->colors_.capacity(n_vert);
    ret->colors_.resize(n_vert);
    for (uint32_t i = 0; i < n_vert; ++i) {
      ret->colors_[i].set(color);
    }

    ret->syncVAO();
    return ret;
  }
  
  GeometryColoredMesh* GeometryColoredMesh::makeCylinder(uint32_t n_slices,
    float height, float base_inside_radius, float top_inside_radius,
    const Float3& color) {
    GeometryColoredMesh* ret = new GeometryColoredMesh();
    
    if(n_slices < 3) {
      throw std::runtime_error(string("GeometryColoredMesh::makeCylinder()") +
                           string("- Error: n_slices < 3"));
    }
    
    
    uint32_t n_vert = (n_slices + 1 + // base vertices
                       n_slices + 1 + // top vertices
                       n_slices * 2); // Side vertices
    ret->vertices_.capacity(n_vert);
    ret->vertices_.resize(n_vert);
    ret->normals_.capacity(n_vert);
    ret->normals_.resize(n_vert);
    
    uint32_t n_ind = ((n_slices * 3) + // base triangles
                      (n_slices * 3) + // top triangles
                      2 * (n_slices * 3));  // Side triangles
    ret->indices_.capacity(n_ind);
    ret->indices_.resize(n_ind);
    
    uint32_t cur_vertex = 0;
    float seperation_angle = (2.0f * static_cast<float>(M_PI) /
                              static_cast<float>(n_slices));
    float base_outside_rad = calcConeOutsideRadius(n_slices, base_inside_radius);
    float top_outside_rad = calcConeOutsideRadius(n_slices, top_inside_radius);
    
    // Define the base center vertex
    ret->normals_.at(cur_vertex)->set(0.0f, 1.0f, 0.0f);
    ret->vertices_.at(cur_vertex)->set(0.0f, 0.5f * height, 0.0f);
    cur_vertex ++;
    
    // Define the base radius vertices
    for (uint32_t i = 0; i < n_slices; i++) {
      float cur_angle = seperation_angle * static_cast<float>(i);
      ret->normals_.at(cur_vertex)->set(0.0f, 1.0f, 0.0f);
      ret->vertices_.at(cur_vertex)->set(base_outside_rad * cosf(cur_angle),
        0.5f * height, base_outside_rad * sinf(cur_angle));
      cur_vertex ++;
    }
    
    // Define the top center vertex
    ret->normals_.at(cur_vertex)->set(0.0f, -1.0f, 0.0f);
    ret->vertices_.at(cur_vertex)->set(0.0f, -0.5f * height, 0.0f);
    cur_vertex ++;
    
    // Define the top radius vertices
    for (uint32_t i = 0; i < n_slices; i++) {
      float cur_angle = seperation_angle * static_cast<float>(i);
      ret->normals_.at(cur_vertex)->set(0.0f, -1.0f, 0.0f);
      ret->vertices_.at(cur_vertex)->set(top_outside_rad * cosf(cur_angle),
        -0.5f * height, top_outside_rad * sinf(cur_angle));
      cur_vertex ++;
    }
    
    Float3 cur_base_point, cur_top_point, v1, v2, cur_normal;
    float cur_angle, tangent_angle;
    
    // Define the cone side and top vertices
    for (uint32_t i = 0; i < n_slices; i++) {
      cur_angle = seperation_angle * static_cast<float>(i);
      cur_base_point.set(base_outside_rad * cosf(cur_angle), 0.5f * height,
                         base_outside_rad * sinf(cur_angle));
      cur_top_point.set(top_outside_rad * cosf(cur_angle), -0.5f * height,
                        top_outside_rad * sinf(cur_angle));
      ret->vertices_.at(cur_vertex)->set(cur_base_point);
      ret->vertices_.at(cur_vertex+1)->set(cur_top_point);
      
      // Tangent is 90deg away: use it to calculate the normal along the slope
      tangent_angle = cur_angle + (static_cast<float>(M_PI) / 2.0f);  
      v1.set(cosf(tangent_angle), 0.0f, sin(tangent_angle));  // length 1!
      Float3::sub(v2, cur_base_point, cur_top_point);
      v2.normalize();
      Float3::cross(cur_normal, v2, v1 );
      cur_normal.normalize();
      ret->normals_.at(cur_vertex)->set(cur_normal);
      ret->normals_.at(cur_vertex+1)->set(cur_normal);
      cur_vertex += 2;
    }
    
#if defined(DEBUG) || defined(_DEBUG)
    if(cur_vertex != n_vert) {
      throw std::runtime_error(string("GeometryColoredMesh::makeCylinder() -") +
        string(" Internal Error: Didn't create enough vertices"));
    }
#endif
    
    // Now define the base indices
    uint32_t cur_index = 0;
    for (uint32_t i = 1; i < n_slices; i++)
    {
      ret->indices_[cur_index] = i;
      cur_index ++;
      ret->indices_[cur_index] = i + 1;
      cur_index ++;
      ret->indices_[cur_index] = 0; // Center
      cur_index ++;
    }
    // Now define the last triangle in the base
    ret->indices_[cur_index] = n_slices;
    cur_index ++;
    ret->indices_[cur_index] = 1;
    cur_index ++;
    ret->indices_[cur_index] = 0; // Center
    cur_index ++;
    
    // Now define the top indices
    uint32_t top_indices_start = n_slices + 1;
    for (uint32_t i = 1; i < n_slices; i++)
    {
      ret->indices_[cur_index] = top_indices_start + i;
      cur_index ++;
      ret->indices_[cur_index] = top_indices_start + 0; // Center
      cur_index ++;
      ret->indices_[cur_index] = top_indices_start + i + 1;
      cur_index ++;
    }
    // Now define the last triangle in the top
    ret->indices_[cur_index] = top_indices_start + n_slices;
    cur_index ++;
    ret->indices_[cur_index] = top_indices_start + 0; // Center
    cur_index ++;
    ret->indices_[cur_index] = top_indices_start + 1;
    cur_index ++;
    
    // Define the cylinder side indices --> Each is a quad patch with 2 tris
    uint32_t side_indices_start = top_indices_start + n_slices + 1;
    for (uint32_t i = 0; i < (n_slices-1); i++) {
      ret->indices_[cur_index] = side_indices_start + (2*i); // base
      cur_index ++;
      ret->indices_[cur_index] = side_indices_start + (2*i + 1);  // top
      cur_index ++;
      ret->indices_[cur_index] = side_indices_start + (2*i + 3); // top
      cur_index ++;
      ret->indices_[cur_index] = side_indices_start + (2*i + 2); // base
      cur_index ++;
      ret->indices_[cur_index] = side_indices_start + (2*i);  // base
      cur_index ++;
      ret->indices_[cur_index] = side_indices_start + (2*i + 3); // top
      cur_index ++;
    }
    // Now define the last 2 triangles on the side
    ret->indices_[cur_index] = side_indices_start + (2*(n_slices-1)); // base
    cur_index ++;
    ret->indices_[cur_index] = side_indices_start + (2*(n_slices-1) + 1); // top
    cur_index ++;
    ret->indices_[cur_index] = side_indices_start + 1; // top
    cur_index ++;
    ret->indices_[cur_index] = side_indices_start; // base
    cur_index ++;
    ret->indices_[cur_index] = side_indices_start + (2*(n_slices-1)); // top
    cur_index ++;
    ret->indices_[cur_index] = side_indices_start + 1; // top
    cur_index ++;
    
#if defined(DEBUG) || defined(_DEBUG)
    if(cur_index != n_ind) {
      throw std::runtime_error(string("GeometryColoredMesh::makeCylinder() -") +
        string(" Internal Error: Didn't create enough indices"));
    }
#endif
    
    ret->colors_.capacity(n_vert);
    ret->colors_.resize(n_vert);
    for (uint32_t i = 0; i < n_vert; ++i) {
      ret->colors_[i].set(color);
    }
    
    ret->syncVAO();
    return ret;
  }

  GeometryColoredMesh* GeometryColoredMesh::makeOpenCylinder(uint32_t n_slices,
    float height, float base_inside_radius, float top_inside_radius,
    const Float3& color) {
    GeometryColoredMesh* ret = new GeometryColoredMesh();
    
    if(n_slices < 3) {
      throw std::runtime_error(string("GeometryColoredMesh::makeCylinder()") +
                           string("- Error: n_slices < 3"));
    }
    
    
    uint32_t n_vert = (n_slices * 2); // Side vertices
    ret->vertices_.capacity(n_vert);
    ret->vertices_.resize(n_vert);
    ret->normals_.capacity(n_vert);
    ret->normals_.resize(n_vert);
    
    uint32_t n_ind = (2 * (n_slices * 3));  // Side triangles
    ret->indices_.capacity(n_ind);
    ret->indices_.resize(n_ind);
    
    uint32_t cur_vertex = 0;
    float seperation_angle = (2.0f * static_cast<float>(M_PI) /
                              static_cast<float>(n_slices));
    float base_outside_rad = calcConeOutsideRadius(n_slices, base_inside_radius);
    float top_outside_rad = calcConeOutsideRadius(n_slices, top_inside_radius);
    
    Float3 cur_base_point, cur_top_point, v1, v2, cur_normal;
    float cur_angle, tangent_angle;
    
    // Define the cone side and top vertices
    for (uint32_t i = 0; i < n_slices; i++) {
      cur_angle = seperation_angle * static_cast<float>(i);
      cur_base_point.set(base_outside_rad * cosf(cur_angle), 0.5f * height,
                         base_outside_rad * sinf(cur_angle));
      cur_top_point.set(top_outside_rad * cosf(cur_angle), -0.5f * height,
                        top_outside_rad * sinf(cur_angle));
      ret->vertices_.at(cur_vertex)->set(cur_base_point);
      ret->vertices_.at(cur_vertex+1)->set(cur_top_point);
      
      // Tangent is 90deg away: use it to calculate the normal along the slope
      tangent_angle = cur_angle + (static_cast<float>(M_PI) / 2.0f);  
      v1.set(cosf(tangent_angle), 0.0f, sin(tangent_angle));  // length 1!
      Float3::sub(v2, cur_base_point, cur_top_point);
      v2.normalize();
      Float3::cross(cur_normal, v2, v1);
      cur_normal.normalize();
      ret->normals_.at(cur_vertex)->set(cur_normal);
      ret->normals_.at(cur_vertex+1)->set(cur_normal);
      cur_vertex += 2;
    }
    
#if defined(DEBUG) || defined(_DEBUG)
    if(cur_vertex != n_vert) {
      throw std::runtime_error(string("GeometryColoredMesh::makeCylinder() -") +
        string(" Internal Error: Didn't create enough vertices"));
    }
#endif
    int cur_index = 0;

    // Define the cylinder side indices --> Each is a quad patch with 2 tris
    uint32_t side_indices_start = 0;
    for (uint32_t i = 0; i < (n_slices-1); i++) {
      ret->indices_[cur_index] = side_indices_start + (2*i); // base
      cur_index ++;
      ret->indices_[cur_index] = side_indices_start + (2*i + 1);  // top
      cur_index ++;
      ret->indices_[cur_index] = side_indices_start + (2*i + 3); // top
      cur_index ++;
      ret->indices_[cur_index] = side_indices_start + (2*i + 2); // base
      cur_index ++;
      ret->indices_[cur_index] = side_indices_start + (2*i);  // base
      cur_index ++;
      ret->indices_[cur_index] = side_indices_start + (2*i + 3); // top
      cur_index ++;
    }
    // Now define the last 2 triangles on the side
    ret->indices_[cur_index] = side_indices_start + (2*(n_slices-1)); // base
    cur_index ++;
    ret->indices_[cur_index] = side_indices_start + (2*(n_slices-1) + 1); // top
    cur_index ++;
    ret->indices_[cur_index] = side_indices_start + 1; // top
    cur_index ++;
    ret->indices_[cur_index] = side_indices_start; // base
    cur_index ++;
    ret->indices_[cur_index] = side_indices_start + (2*(n_slices-1)); // top
    cur_index ++;
    ret->indices_[cur_index] = side_indices_start + 1; // top
    cur_index ++;
    
#if defined(DEBUG) || defined(_DEBUG)
    if(cur_index != n_ind) {
      throw std::runtime_error(string("GeometryColoredMesh::makeCylinder() -") +
        string(" Internal Error: Didn't create enough indices"));
    }
#endif
    
    ret->colors_.capacity(n_vert);
    ret->colors_.resize(n_vert);
    for (uint32_t i = 0; i < n_vert; ++i) {
      ret->colors_[i].set(color);
    }
    
    ret->syncVAO();
    return ret;
  }

  Pair<uint8_t*,uint32_t> GeometryColoredMesh::saveToArray() {
    Pair<uint8_t*,uint32_t> data;
    data.first = NULL;
    data.second = 0;

    char char_dummy;
    static_cast<void>(char_dummy);
    if (sizeof(char_dummy) != 1) {
      throw std::runtime_error("saveToArray - sizeof(char) != 1");
    }
    uint32_t data_size = COLORED_MESH_FILE_DATA_SIZE +
      (uint32_t)(name_.size() + 1) * sizeof(char_dummy) +  // name
      vertices_.size() * 4 * 3 +  // vertices - 32bit float x 3
      normals_.size() * 4 * 3 +  // normals - 32bit float x 3
      colors_.size() * 4 * 3 +  // colors - 32bit float x 3
      indices_.size() * 4;  // indices - 32bit uint x 1
    data.second = data_size;

    data.first = (uint8_t*)malloc(data_size);

    // Get the data ready
    ColoredMeshFileData* preamble =
      reinterpret_cast<ColoredMeshFileData*>(data.first);
    preamble->type = GeometryType::GEOMETRY_COLORED_MESH;
    memcpy(preamble->mat_m, mat_.m, 16 * sizeof(preamble->mat_m[0]));
    preamble->mtrl_specular_intensity = mtrl_.specular_intensity;
    preamble->mtrl_specular_power = mtrl_.specular_power;
    preamble->num_ind = indices_.size();
    preamble->num_vert = vertices_.size();
    preamble->name_size = (uint32_t)name_.size();

    // Now copy the name string
    char* name_c_str = reinterpret_cast<char*>(&data.first[COLORED_MESH_FILE_DATA_SIZE]);
    strcpy(name_c_str, name_.c_str());

    float* vert = reinterpret_cast<float*>(&name_c_str[name_.size()+1]);
    for (uint32_t i = 0; i < vertices_.size(); i++) {
      vert[i * 3] = vertices_[i][0];
      vert[i * 3 + 1] = vertices_[i][1];
      vert[i * 3 + 2] = vertices_[i][2];
    }

    float* norm = &vert[vertices_.size() * 3];
    for (uint32_t i = 0; i < normals_.size(); i++) {
      norm[i * 3] = normals_[i][0];
      norm[i * 3 + 1] = normals_[i][1];
      norm[i * 3 + 2] = normals_[i][2];
    }

    float* col = &norm[normals_.size() * 3];
    for (uint32_t i = 0; i < colors_.size(); i++) {
      col[i * 3] = colors_[i][0];
      col[i * 3 + 1] = colors_[i][1];
      col[i * 3 + 2] = colors_[i][2];
    }

    uint32_t* ind = reinterpret_cast<uint32_t*>(&col[colors_.size() * 3]);
    for (uint32_t i = 0; i < indices_.size(); i++) {
      ind[i] = indices_[i];
    }

    return data;
  }

  void GeometryColoredMesh::loadFromArray(const std::string& path, 
      const std::string& filename, const uint8_t* arr) {
    const ColoredMeshFileData* preamble = 
      reinterpret_cast<const ColoredMeshFileData*>(arr);

    if (preamble->type != GeometryType::GEOMETRY_COLORED_MESH) {
      throw runtime_error(string("GeometryColoredMesh::loadFromArray() - ") +
        string("INTERNAL ERROR: Incorrect data type"));
    }

    mat_.set((float*)preamble->mat_m);
    mtrl_.specular_intensity = preamble->mtrl_specular_intensity;
    mtrl_.specular_power = preamble->mtrl_specular_power;

    // Extract the name
    const char* name_c_str = reinterpret_cast<const char*>(&arr[COLORED_MESH_FILE_DATA_SIZE]); 
    name_ = string(name_c_str);

    const float* vert = reinterpret_cast<const float*>(&name_c_str[preamble->name_size + 1]);
    vertices_.capacity(preamble->num_vert);
    vertices_.resize(preamble->num_vert);
    for (uint32_t i = 0; i < preamble->num_vert; i++) {
      vertices_[i].set(vert[i * 3], vert[i * 3 + 1], vert[i * 3 + 2]);
    }

    const float* norm = &vert[preamble->num_vert*3];
    normals_.capacity(preamble->num_vert);
    normals_.resize(preamble->num_vert);
    for (uint32_t i = 0; i < preamble->num_vert; i++) {
      normals_[i].set(norm[i * 3], norm[i * 3 + 1], norm[i * 3 + 2]);
    }

    const float* col = &norm[preamble->num_vert*3];
    colors_.capacity(preamble->num_vert);
    colors_.resize(preamble->num_vert);
    for (uint32_t i = 0; i < preamble->num_vert; i++) {
      colors_[i].set(col[i * 3], col[i * 3 + 1], col[i * 3 + 2]);
    }

    const uint32_t* ind = reinterpret_cast<const uint32_t*>(&col[preamble->num_vert*3]);
    indices_.capacity(preamble->num_ind);
    indices_.resize(preamble->num_ind);
    for (uint32_t i = 0; i < preamble->num_ind; i++) {
      indices_[i] = ind[i];
    }

    syncVAO();
    cout << "    - loaded GeometryColoredMesh with " << synced_num_vertices_;
    cout << " vertices and " << synced_num_indices_ << " faces" << endl;
  }

  GeometryColoredMesh* GeometryColoredMesh::convertAssimpMesh(
    const std::string& path, const std::string& filename,
    const aiScene* scene, const aiMesh* mesh) {
    GeometryColoredMesh* geom = new GeometryColoredMesh();

    Float3 color = DEFAULT_GEOMETRY_COLOR;
    Material* mtrl = geom->mtrl();
    aiMaterial* assimp_mtrl = scene->mMaterials[mesh->mMaterialIndex];

    aiColor4D assimp_color(0.0f, 0.0f, 0.0f, 0.0f);
    if (AI_SUCCESS == assimp_mtrl->Get(AI_MATKEY_COLOR_DIFFUSE, assimp_color)) {
      color[0] = assimp_color.r;
      color[1] = assimp_color.g;
      color[2] = assimp_color.b;
    }

    float spec_pow;
    if (AI_SUCCESS == assimp_mtrl->Get(AI_MATKEY_SHININESS, spec_pow)) {
      mtrl->specular_power = spec_pow;
    }

    float spec_intens;
    if (AI_SUCCESS == 
      assimp_mtrl->Get(AI_MATKEY_SHININESS_STRENGTH, spec_intens)) {
      mtrl->specular_intensity = spec_intens;
    }

    // TO DO: Allow seperate specular colors for all meshes
    // TO DO: Allow seperate ambient colors
    // ie make a much more comprehensive material class.

    if (!mesh->HasNormals() || !mesh->HasFaces()) {
      throw runtime_error(string("Geometry::convertAssimpMesh() - ") + 
        string("ERROR: mesh does not have normals or faces!"));
    }

    geom->vertices_.capacity(mesh->mNumVertices);
    geom->vertices_.resize(mesh->mNumVertices);
    geom->normals_.capacity(mesh->mNumVertices);
    geom->normals_.resize(mesh->mNumVertices);
    geom->colors_.capacity(mesh->mNumVertices);
    geom->colors_.resize(mesh->mNumVertices);

    for (uint32_t i = 0; i < mesh->mNumVertices; i++) {
      geom->vertices_[i][0] = mesh->mVertices[i].x;
      geom->vertices_[i][1] = mesh->mVertices[i].y;
      geom->vertices_[i][2] = mesh->mVertices[i].z;

      geom->normals_[i][0] = mesh->mNormals[i].x;
      geom->normals_[i][1] = mesh->mNormals[i].y;
      geom->normals_[i][2] = mesh->mNormals[i].z;

      geom->colors_[i][0] = color[0];
      geom->colors_[i][1] = color[1];
      geom->colors_[i][2] = color[2];
    }

    geom->indices_.capacity(mesh->mNumFaces * 3);
    geom->indices_.resize(mesh->mNumFaces * 3);
    for (uint32_t i = 0; i < mesh->mNumFaces; i++) {
      aiFace face = mesh->mFaces[i];
      if (face.mNumIndices != 3) {
        throw runtime_error(string("Geometry::convertAssimpMesh() - ") + 
          string("ERROR: Found a face that isn't 3 verticies!"));
      }
      // Assimp indicies are wound the wrong way..., so reverse the order of
      // last two.
      geom->indices_[i*3] = face.mIndices[0];
      geom->indices_[i*3+1] = face.mIndices[2];
      geom->indices_[i*3+2] = face.mIndices[1];
    }

    geom->syncVAO();

    cout << "    - loaded GeometryColoredMesh with " << geom->synced_num_vertices_;
    cout << " vertices and " << geom->synced_num_indices_ << " faces" << endl;

    return geom;
  }

  Geometry* GeometryColoredMesh::copy() {
    GeometryColoredMesh* ret = new GeometryColoredMesh();
    ret->mat_ = this->mat_;
    ret->name_ = this->name_;
    ret->mtrl_ = this->mtrl_;
    ret->vertices_ = this->vertices_;
    ret->normals_ = this->normals_;
    ret->colors_ = this->colors_;
    ret->indices_ = this->indices_;
    for (uint32_t i = 0; i < children_.size(); i++) {
      Geometry* child = children_[i]->copy();
      ret->addChild(child);
    }
    ret->syncVAO();
    return ret;
  }

}  // namespace renderer

