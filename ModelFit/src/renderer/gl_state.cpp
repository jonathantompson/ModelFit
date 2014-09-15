#include <sstream>
#include <iostream>
#include "renderer/gl_state.h"
#include "data_str/hash_map.h"
#include "data_str/hash_funcs.h"
#include "data_str/vector_managed.h"

#ifndef GLEW_STATIC
  #define GLEW_STATIC
#endif
#include "glew/glew.h"

using jtil::data_str::HashMap;
using jtil::data_str::HashInt;
using jtil::data_str::HashUInt;
using jtil::data_str::VectorManaged;
using jtil::data_str::Pair;
using std::runtime_error;
using jtil::math::Vec2;

#define START_DATABASE_SIZE 101
#define SAFE_DELETE(x) if (x != NULL) { delete x; x = NULL; }

// Sizes are all in bytes
#define GLTYPE_FLOAT_SIZE 4
#define GLTYPE_ENUM_SIZE 4
#define GLTYPE_INT_SIZE 4
#define GLTYPE_BOOL_SIZE 1

namespace renderer {
  GLStateElem::GLStateElem(const GLenum state_name, const void* data, 
    const uint32_t size, const int type) {
    this->state_name = state_name;
    this->data = new uint8_t[size];
    this->size = size;
    if (size > MAX_DATA_SIZE) {
      throw std::runtime_error("GLStateElem::GLStateElem() - max data size"
        " not large enough! Make it larger!");
    }
    this->type = type;
    memcpy(this->data, data, size);
  }

  GLStateElem::~GLStateElem() {
    delete[] data;
  }

  GLStateElem& GLStateElem::operator=(const void* rhs) {
    memcpy(data, rhs, size);
    return *this;
  }

  bool GLStateElem::operator==(const void* rhs) {
    bool ret_val = true;
    for (uint32_t i = 0; i < size; i++) {
      ret_val = ret_val && ((uint8_t*)data)[i] == ((uint8_t*)rhs)[i];
    }
    return ret_val;
  }

  bool GLStateElem::operator!=(const void* rhs) {
    return !(*this == rhs);
  }

  std::mutex GLState::startup_mutex_;
  bool GLState::gl_state_initialized_ = false;

  jtil::data_str::HashMap<uint32_t, uint32_t>* GLState::ht_GLEnum_2_idata_ = NULL;
  jtil::data_str::VectorManaged<GLStateElem*>* GLState::idata_2_data_ = NULL;

  GLuint GLState::bound_textures_[MAX_NUM_TEXTURE_UNITS][NUM_TEXTURE_TARGETS];
  GLint GLState::max_texture_units_;

  uint8_t GLState::temp_data_[MAX_DATA_SIZE];
  std::mutex GLState::state_data_mutex_;

  void GLState::initGLState() {
    state_data_mutex_.lock();
    startup_mutex_.lock();
    if (gl_state_initialized_) { 
      startup_mutex_.unlock();
      return;
    }

#if defined(DEBUG) || defined(_DEBUG)
    // Check that the size of our variables are the same as openGL
    GLenum dummy_enum;
    GLboolean dummy_boolean;
    GLfloat dummy_float;
    GLint dummy_int;
    static_cast<void>(dummy_enum);
    static_cast<void>(dummy_boolean);
    static_cast<void>(dummy_float);
    static_cast<void>(dummy_int);
    if (sizeof(dummy_enum) != GLTYPE_ENUM_SIZE ||
      sizeof(dummy_boolean) != GLTYPE_BOOL_SIZE || 
      sizeof(dummy_float) != GLTYPE_FLOAT_SIZE ||
      sizeof(dummy_int) != GLTYPE_INT_SIZE) {
      startup_mutex_.unlock();
      throw std::runtime_error("GLState::GLState() - ERROR: type size mismatch!");
    }
#endif

    ht_GLEnum_2_idata_ = new HashMap<uint32_t, uint32_t>(START_DATABASE_SIZE, 
      &HashUInt);
    idata_2_data_ = new VectorManaged<GLStateElem*>(START_DATABASE_SIZE);

    // Now set all the texture units to default value
    glGetIntegerv(GL_MAX_COMBINED_TEXTURE_IMAGE_UNITS, &max_texture_units_);
    if (max_texture_units_ > MAX_NUM_TEXTURE_UNITS) {
      startup_mutex_.unlock();
      throw std::runtime_error("GLState::GLState() - ERROR: max_texture_units_ >"
        "MAX_NUM_TEXTURE_UNITS, consider increasing size.");
    }
    for (GLint i = 0; i < MAX_NUM_TEXTURE_UNITS; i++) {
      // http://www.opengl.org/sdk/docs/man/xhtml/glBindTexture.xml
      // Texture names are unsigned integers. The value zero is reserved to 
      // represent the default texture for each texture target
      for (uint32_t j = 0; j < NUM_TEXTURE_TARGETS; j++) {
        bound_textures_[i][j] = 0;
      }
    }
    gl_state_initialized_ = true;
    state_data_mutex_.unlock();
    startup_mutex_.unlock();
  }

  void GLState::shutdownGLState() {
    state_data_mutex_.lock();
    SAFE_DELETE(ht_GLEnum_2_idata_);
    SAFE_DELETE(idata_2_data_);
    gl_state_initialized_ = false;
    state_data_mutex_.unlock();
  }

  void GLState::verifyOpenGLState() {
    state_data_mutex_.lock();
    for (uint32_t i = 0; i < idata_2_data_->size(); i++) {
      GLStateElem* elem = (*idata_2_data_)[i];
      glsGetStateData(elem->state_name, (GLType)elem->type, temp_data_);
      if (*elem != temp_data_) {
        std::stringstream ss;
        ss << "GLState::verifyOpenGLState() - ERROR: OpenGL state is corrupt!";
        ss << " state_name (hex) = " << std::hex << elem->state_name;
        ss << " size = " << elem->size;
        ss << " type = " << elem->type;
        ss << " value (as uint8_t arr) = ";
        for (uint32_t i = 0; i < elem->size; i++) {
          ss << (int)((uint8_t*)elem->data)[i];
        }
        ss << " actual value (as uint8_t arr) = ";
        for (uint32_t i = 0; i < elem->size; i++) {
          ss << (int)temp_data_[i];
        }
        throw std::runtime_error(ss.str());
      }
    }
    state_data_mutex_.unlock();
  }

  void GLState::glsGetStateData(const GLenum state_name, const GLType type, 
    void* data) {
    switch (type) {
    case GLTYPE_BOOL:
      glGetBooleanv(state_name, (GLboolean*)data);
      ERROR_CHECK;
      break;
    case GLTYPE_ENUM:
    case GLTYPE_INT:
      glGetIntegerv(state_name, (GLint*)data);
      ERROR_CHECK;
      break;
    case GLTYPE_FLOAT:
      glGetFloatv(state_name, (GLfloat*)data);
      ERROR_CHECK;
      break;
    case GLTYPE_DOUBLE:
      glGetDoublev(state_name, (GLdouble*)data);
      ERROR_CHECK;
      break;
    case GLTYPE_INT64:
      glGetInteger64v(state_name, (GLint64*)data);
      ERROR_CHECK;
      break;
    }
  }

  void GLState::glsEnable(const GLenum cap) {
    GLboolean val = GL_TRUE;
    if (setGLenum(cap, &val, GLTYPE_BOOL_SIZE, GLTYPE_BOOL)) {
      glEnable(cap);
      ERROR_CHECK;
    }
  }

  void GLState::glsDisable(const GLenum cap) {
    GLboolean val = GL_FALSE;
    if (setGLenum(cap, &val, GLTYPE_BOOL_SIZE, GLTYPE_BOOL)) {
      glDisable(cap);
      ERROR_CHECK;
    }
  }

  void GLState::glsDepthFunc(const GLenum func) {
    if (setGLenum(GL_DEPTH_FUNC, &func, GLTYPE_ENUM_SIZE, GLTYPE_ENUM)) {
      glDepthFunc(func);
      ERROR_CHECK;
    }
  }

  void GLState::glsDepthMask(const GLboolean flag) {
    if (setGLenum(GL_DEPTH_WRITEMASK, &flag, GLTYPE_BOOL_SIZE, 
      GLTYPE_BOOL)) {
      glDepthMask(flag);
      ERROR_CHECK;
    }
  }

  void GLState::glsCullFace(const GLenum mode) {
    if (setGLenum(GL_CULL_FACE_MODE, &mode, GLTYPE_ENUM_SIZE, GLTYPE_ENUM)) {
      glCullFace(mode);
      ERROR_CHECK;
    }
  }

  void GLState::glsBlendEquation(const GLenum mode) {
    if (setGLenum(GL_BLEND_EQUATION_RGB, &mode, GLTYPE_ENUM_SIZE, 
      GLTYPE_ENUM) || setGLenum(GL_BLEND_EQUATION_ALPHA, &mode, 
      GLTYPE_ENUM_SIZE, GLTYPE_ENUM)) {
      glBlendEquation(mode);
      ERROR_CHECK;
    }
  }

  void GLState::glsBlendFunc(const GLenum sfactor, const GLenum dfactor) {
    if (setGLenum(GL_BLEND_SRC, &sfactor, GLTYPE_ENUM_SIZE, GLTYPE_ENUM) || 
      setGLenum(GL_BLEND_DST, &dfactor, GLTYPE_ENUM_SIZE, GLTYPE_ENUM)){
      glBlendFunc(sfactor, dfactor);
      ERROR_CHECK;
    }
  }

  void GLState::glsDrawBuffer(const GLenum mode) {
    if (setGLenum(GL_DRAW_BUFFER, &mode, GLTYPE_ENUM_SIZE, GLTYPE_ENUM)){
      glDrawBuffer(mode);
      ERROR_CHECK;
    }
  }

  void GLState::glsPolygonMode(const GLenum face, const GLenum mode) {
    //// EDIT: 3/2/2013
    //// ON ATI Hardware glGetIntegerv( GL_POLYGON_MODE, previous ); throws
    //// invalid enumerant!
    /*
    GLenum cur_mode[2];
    getGLenum(GL_POLYGON_MODE, cur_mode, 2 * GLTYPE_ENUM_SIZE, GLTYPE_ENUM);
    if (face == GL_FRONT_AND_BACK || face == GL_FRONT) {
      cur_mode[0] = mode;
    }
    if (face == GL_FRONT_AND_BACK || face == GL_BACK) {
      cur_mode[1] = mode;
    }
    if (setGLenum(GL_POLYGON_MODE, cur_mode, 2 * GLTYPE_ENUM_SIZE, 
      GLTYPE_ENUM)) {
      glPolygonMode(face, mode);
      ERROR_CHECK;
    }
    */
    glPolygonMode(face, mode);
    ERROR_CHECK;
  }

  void GLState::glsClearColor(const GLfloat red, const GLfloat green, 
    const GLfloat blue, const GLfloat alpha) {
    GLfloat rgba[4];
    rgba[0] = red;
    rgba[1] = green;
    rgba[2] = blue;
    rgba[3] = alpha;
    if (setGLenum(GL_COLOR_CLEAR_VALUE, rgba, 4 * GLTYPE_FLOAT_SIZE, 
      GLTYPE_FLOAT)) {
      glClearColor(red, green, blue, alpha);
      ERROR_CHECK;
    }
  }

  void GLState::glsCheckFramebufferStatus(const GLenum target) {
    GLenum status = glCheckFramebufferStatus(target);
    ERROR_CHECK;

    if (status != GL_FRAMEBUFFER_COMPLETE) {
      std::stringstream ss;
      ss << "GLState::glsCheckFramebufferStatus() - ERROR: ";
      ss << "glCheckFramebufferStatus returned ";
      std::string err;
      switch (status) {
      case GL_FRAMEBUFFER_INCOMPLETE_ATTACHMENT:
        ss << "GL_FRAMEBUFFER_INCOMPLETE_ATTACHMENT";
        break;
      case GL_FRAMEBUFFER_INCOMPLETE_MISSING_ATTACHMENT:
        ss << "GL_FRAMEBUFFER_INCOMPLETE_MISSING_ATTACHMENT";
        break;
      case GL_FRAMEBUFFER_INCOMPLETE_DRAW_BUFFER:
        ss << "GL_FRAMEBUFFER_INCOMPLETE_DRAW_BUFFER";
        break;
      case GL_FRAMEBUFFER_INCOMPLETE_READ_BUFFER:
        ss << "GL_FRAMEBUFFER_INCOMPLETE_READ_BUFFER";
        break;
      case GL_FRAMEBUFFER_UNSUPPORTED:
        ss << "GL_FRAMEBUFFER_UNSUPPORTED";
        break;
      case GL_FRAMEBUFFER_INCOMPLETE_MULTISAMPLE:
        ss << "GL_FRAMEBUFFER_INCOMPLETE_MULTISAMPLE";
        break;
      case GL_FRAMEBUFFER_INCOMPLETE_LAYER_TARGETS:
        ss << "GL_FRAMEBUFFER_INCOMPLETE_LAYER_TARGETS";
        break;
      default:
        ss << "the error code: " << status;
        break;
      }
      throw std::runtime_error(ss.str());
    }
  }

  void GLState::glsViewport(const GLint x, const GLint y, const GLsizei width, 
    const GLsizei height) {
    GLint xywh[4];
    xywh[0] = x;
    xywh[1] = y;
    xywh[2] = width;
    xywh[3] = height;
    if (setGLenum(GL_VIEWPORT, xywh, 4 * GLTYPE_INT_SIZE, GLTYPE_INT)) {
      glViewport(x, y, width, height);
      ERROR_CHECK;
    }
  }

  void GLState::glsBindFramebuffer(const GLenum target, 
    const GLuint framebuffer) {

    switch (target) {
    case GL_FRAMEBUFFER:
      /*
      if (setGLenum(GL_FRAMEBUFFER_BINDING, &framebuffer, 
        GLTYPE_ENUM_SIZE, GLTYPE_ENUM)) {
        glBindFramebuffer(GL_FRAMEBUFFER, framebuffer);
      }
      */
      glsBindFramebuffer(GL_READ_FRAMEBUFFER, framebuffer);
      glsBindFramebuffer(GL_DRAW_FRAMEBUFFER, framebuffer);
      break;
    case GL_READ_FRAMEBUFFER:
      if (setGLenum(GL_READ_FRAMEBUFFER_BINDING, &framebuffer, 
        GLTYPE_ENUM_SIZE, GLTYPE_ENUM)) {
        glBindFramebuffer(GL_READ_FRAMEBUFFER, framebuffer);
      }
      break;
    case GL_DRAW_FRAMEBUFFER:
      if (setGLenum(GL_DRAW_FRAMEBUFFER_BINDING, &framebuffer, 
        GLTYPE_ENUM_SIZE, GLTYPE_ENUM)) {
        glBindFramebuffer(GL_DRAW_FRAMEBUFFER, framebuffer);
      }
      break;
    default:
      throw std::runtime_error("GLState::glsBindFramebuffer() - target"
        " invalid");
    }
    ERROR_CHECK;
  }

  void GLState::setupQuadRendering() {
    glsDisable(GL_DEPTH_TEST);
    glsDisable(GL_CULL_FACE);
    glsDisable(GL_BLEND);
    glsPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
  }

  void GLState::glsClear(const GLbitfield mask) {
    glClear(mask);
    ERROR_CHECK;
  }

  void GLState::glsStencilOpSeparate(const GLenum face, const GLenum sfail, 
    const GLenum dpfail, const GLenum dppass) {
    switch (face) {
    case GL_FRONT_AND_BACK:
      /*
      if (setGLenumGLenum(GL_STENCIL_FAIL, sfail) ||  // Front
        setGLenumGLenum(GL_STENCIL_PASS_DEPTH_FAIL, dpfail) ||  // Front
        setGLenumGLenum(GL_STENCIL_PASS_DEPTH_PASS, dppass) ||  // Front
        setGLenumGLenum(GL_STENCIL_BACK_FAIL, sfail) ||  // back
        setGLenumGLenum(GL_STENCIL_BACK_PASS_DEPTH_FAIL, sfail) ||  // back
        setGLenumGLenum(GL_STENCIL_BACK_PASS_DEPTH_PASS, sfail)) {  // back
        glStencilOpSeparate(GL_FRONT_AND_BACK, sfail, dpfail, dppass);
        ERROR_CHECK;
      }
      */
      glsStencilOpSeparate(GL_FRONT, sfail, dpfail, dppass);
      glsStencilOpSeparate(GL_BACK, sfail, dpfail, dppass);
      break;
    case GL_FRONT:
      if (setGLenum(GL_STENCIL_FAIL, &sfail, GLTYPE_ENUM_SIZE, GLTYPE_ENUM) || 
        setGLenum(GL_STENCIL_PASS_DEPTH_FAIL, &dpfail, GLTYPE_ENUM_SIZE, GLTYPE_ENUM) ||
        setGLenum(GL_STENCIL_PASS_DEPTH_PASS, &dppass, GLTYPE_ENUM_SIZE, GLTYPE_ENUM)) { 
        glStencilOpSeparate(GL_FRONT, sfail, dpfail, dppass);
        ERROR_CHECK;
      }
      break;
    case GL_BACK:
      if (setGLenum(GL_STENCIL_BACK_FAIL, &sfail, GLTYPE_ENUM_SIZE, GLTYPE_ENUM) ||
        setGLenum(GL_STENCIL_BACK_PASS_DEPTH_FAIL, &dpfail, GLTYPE_ENUM_SIZE, GLTYPE_ENUM) ||
        setGLenum(GL_STENCIL_BACK_PASS_DEPTH_PASS, &dppass, GLTYPE_ENUM_SIZE, GLTYPE_ENUM)) {
        glStencilOpSeparate(GL_BACK, sfail, dpfail, dppass);
        ERROR_CHECK;
      }
      break;
    default:
      throw std::runtime_error("GLState::glsStencilOpSeparate() - face"
        " invalid");
    }
  }

  void GLState::glsStencilOp(const GLenum sfail, const GLenum dpfail, 
    const GLenum dppass) {
    glsStencilOpSeparate(GL_FRONT_AND_BACK, sfail, dpfail, dppass);
  }

  void GLState::glsStencilFuncSeparate(const GLenum face, 
    const GLenum func, const GLint ref, const GLuint mask) {
    switch (face) {
    case GL_FRONT_AND_BACK:
      /*
      if (setGLenumGLenum(GL_STENCIL_FUNC, func) ||  // Front
        setGLenumGLenum(GL_STENCIL_REF, ref) ||  // Front
        setGLenumGLenum(GL_STENCIL_VALUE_MASK, mask) ||  // Front
        setGLenumGLenum(GL_STENCIL_BACK_FUNC, func) ||  // back
        setGLenumGLenum(GL_STENCIL_BACK_REF, ref) ||  // back
        setGLenumGLenum(GL_STENCIL_BACK_VALUE_MASK, mask)) {  // back
        glStencilFuncSeparate(GL_FRONT_AND_BACK, func, ref, mask);
        ERROR_CHECK;
      }
      break;
      */
      glsStencilFuncSeparate(GL_FRONT, func, ref, mask);
      glsStencilFuncSeparate(GL_BACK, func, ref, mask);
    case GL_FRONT:
      if (setGLenum(GL_STENCIL_FUNC, &func, GLTYPE_ENUM_SIZE, GLTYPE_ENUM) ||
        setGLenum(GL_STENCIL_REF, &ref, GLTYPE_ENUM_SIZE, GLTYPE_ENUM) ||
        setGLenum(GL_STENCIL_VALUE_MASK, &mask, GLTYPE_ENUM_SIZE, GLTYPE_ENUM)) {
        glStencilFuncSeparate(GL_FRONT, func, ref, mask);
        ERROR_CHECK;
      }
      break;
    case GL_BACK:
      if (setGLenum(GL_STENCIL_BACK_FUNC, &func, GLTYPE_ENUM_SIZE, GLTYPE_ENUM) ||
        setGLenum(GL_STENCIL_BACK_REF, &ref, GLTYPE_ENUM_SIZE, GLTYPE_ENUM) ||
        setGLenum(GL_STENCIL_BACK_VALUE_MASK, &mask, GLTYPE_ENUM_SIZE, GLTYPE_ENUM)) {
        glStencilFuncSeparate(GL_BACK, func, ref, mask);
        ERROR_CHECK;
      }
      break;
    default:
      throw std::runtime_error("GLState::glsStencilFuncSeparate() - face"
        " invalid");
    }
  }

  void GLState::glsStencilFunc(const GLenum func, const GLint ref, 
    const GLuint mask) {
    glsStencilFuncSeparate(GL_FRONT_AND_BACK, func, ref, mask);
  }

  void GLState::glsBindVertexArray(const GLuint arr) {
    if (setGLenum(GL_VERTEX_ARRAY_BINDING, &arr, GLTYPE_INT_SIZE, 
      GLTYPE_INT)) {
      glBindVertexArray(arr);  // encapsulates all the vbo bindings
      ERROR_CHECK;
    }
  }

  void GLState::glsBindTexture(const GLenum target, const GLuint texture) {
#ifdef FORCE_STATE_SETTING
    glBindTexture(target, texture);
    ERROR_CHECK;
#endif

    // TEMP CODE:
    // Bind anyway
    glBindTexture(target, texture);
    ERROR_CHECK;
    return;
    // 

    // This is tricky because the bound texture depends on which texture unit
    // is currently active, first convert the GLenum target to our own format:
    GLStateTexture query_enum = GLTarget2GLStateTexture(target);
    
    // Now query the currently active texture
    GLenum active_tex;
    getGLenum(GL_ACTIVE_TEXTURE, &active_tex, GLTYPE_ENUM_SIZE, GLTYPE_ENUM);
    int active_tex_index = active_tex - GL_TEXTURE0;

    // Now see if the active is bound to the requested texture
    if (bound_textures_[active_tex_index][query_enum] != texture) {
      glBindTexture(target, texture);
      ERROR_CHECK;
      bound_textures_[active_tex_index][query_enum] = texture;
    }
  }

  void GLState::glsActiveTexture(const GLenum texture) {
    if (setGLenum(GL_ACTIVE_TEXTURE, &texture, GLTYPE_ENUM_SIZE, 
      GLTYPE_ENUM)) {
      glActiveTexture(texture);
      ERROR_CHECK;
    }
  }

  GLStateTexture GLState::GLTarget2GLStateTexture(const GLenum target) {
    switch (target) {
    case GL_TEXTURE_1D:
      return TEXTURE_1D;
    case GL_TEXTURE_2D:
      return TEXTURE_2D;
    case GL_TEXTURE_3D:
      return TEXTURE_3D;
    case GL_TEXTURE_1D_ARRAY:
      return TEXTURE_1D_ARRAY;
    case GL_TEXTURE_2D_ARRAY:
      return TEXTURE_2D_ARRAY;
    case GL_TEXTURE_RECTANGLE:
      return TEXTURE_RECTANGLE;
    case GL_TEXTURE_CUBE_MAP:
      return TEXTURE_CUBE_MAP;
    case GL_TEXTURE_CUBE_MAP_ARRAY:
      return TEXTURE_CUBE_MAP_ARRAY;
    case GL_TEXTURE_BUFFER:
      return TEXTURE_BUFFER;
    case GL_TEXTURE_2D_MULTISAMPLE:
      return TEXTURE_2D_MULTISAMPLE;
    case GL_TEXTURE_2D_MULTISAMPLE_ARRAY:
      return TEXTURE_2D_MULTISAMPLE_ARRAY;
    default:
      throw std::runtime_error("GLState::GLTarget2GLStateTexture() - "
        "target invalid");
    }
  }

  GLenum GLState::GLStateTexture2GLTargetBinding(const GLStateTexture target) {
    switch (target) {
    case TEXTURE_1D:
      return GL_TEXTURE_BINDING_1D;
    case TEXTURE_2D:
      return GL_TEXTURE_BINDING_2D;
    case TEXTURE_3D:
      return GL_TEXTURE_BINDING_3D;
    case TEXTURE_1D_ARRAY:
      return GL_TEXTURE_BINDING_1D_ARRAY;
    case TEXTURE_2D_ARRAY:
      return GL_TEXTURE_BINDING_2D_ARRAY;
    case TEXTURE_RECTANGLE:
      return GL_TEXTURE_BINDING_RECTANGLE;
    case TEXTURE_CUBE_MAP:
      return GL_TEXTURE_BINDING_CUBE_MAP;
    case TEXTURE_CUBE_MAP_ARRAY:
      return GL_TEXTURE_BINDING_CUBE_MAP_ARRAY;
    case TEXTURE_BUFFER:
      return GL_TEXTURE_BINDING_BUFFER;
    case TEXTURE_2D_MULTISAMPLE:
      return GL_TEXTURE_BINDING_2D_MULTISAMPLE;
    case TEXTURE_2D_MULTISAMPLE_ARRAY:
      return GL_TEXTURE_BINDING_2D_MULTISAMPLE_ARRAY;
    default:
      throw std::runtime_error("GLState::GLStateTexture2GLTargetBinding() - "
        "target invalid");
    }
  }

  void GLState::glsDrawBuffers(const GLsizei n, const GLenum* bufs) {
    /*
    bool change_state = false;
    for (GLsizei i = 0; i < n; i++) {
      if (setGLenumGLenum(GL_DRAW_BUFFER0 + i, bufs[i])) {
        change_state = true;
      }
    }
    if (change_state) {
      glDrawBuffers(n, bufs);
    }
    */
    // Just set them anyway.
    glDrawBuffers(n, bufs);
    ERROR_CHECK;
  }

  void GLState::glsScissor(const GLint x, const GLint y, const GLsizei width, 
    const GLsizei height) {
    GLint params[4] = {x, y, width, height};
    if (setGLenum(GL_SCISSOR_BOX, params, 4 * GLTYPE_INT_SIZE, GLTYPE_INT)) {
      glScissor(x, y, width, height);
      ERROR_CHECK;
    }
  }

  void GLState::glsDrawArrays(const GLenum mode, const GLint first, 
    const GLsizei count) {
    glDrawArrays(mode, first, count);
    ERROR_CHECK;
  }

  void GLState::glsDrawElements(const GLenum mode, const GLsizei count,
    const GLenum type, const GLvoid* indices) {
    glDrawElements(mode, count, type, indices);
    ERROR_CHECK;
  }

  void GLState::glsDeleteBuffers(const GLsizei n, GLuint* buffers) {
    glDeleteBuffers(n, buffers);
    //ERROR_CHECK;
  }

  void GLState::glsDeleteVertexArrays(const GLsizei n, GLuint* arrays) {
    glDeleteVertexArrays(n, arrays);
    //ERROR_CHECK;
  }

  void GLState::glsGenVertexArrays(const GLsizei n, GLuint* arrays) {
    glGenVertexArrays(n, arrays);
    ERROR_CHECK;
  }

  void GLState::glsGenBuffers(const GLsizei n, GLuint* buffers) {
    glGenBuffers(n, buffers);
    ERROR_CHECK;
  }

  void GLState::glsBindBuffer(const GLenum target, const GLuint buffer) {
    glBindBuffer(target, buffer);
    ERROR_CHECK;
  }

  void GLState::glsBufferData(const GLenum target, const GLsizeiptr size, 
    const GLvoid* data, const GLenum usage) {
    glBufferData(target, size, data, usage);
    ERROR_CHECK;
  }

  void* GLState::glsMapBuffer(const GLenum target, const GLenum access) {
    void* ret_val = glMapBuffer(target, access);
    ERROR_CHECK;
    return ret_val;
  }

  GLboolean GLState::glsUnmapBuffer(const GLenum target) {
    GLboolean ret_val = glUnmapBuffer(target);
    ERROR_CHECK;
    return ret_val;
  }

  void GLState::glsVertexAttribPointer(const GLuint index, const GLint size,
    const GLenum type, const GLboolean normalized, const GLsizei stride, 
    const GLvoid* pointer) {
    glVertexAttribPointer(index, size, type, normalized, stride, pointer);
    ERROR_CHECK;
  }

  void GLState::glsVertexAttribIPointer(const GLuint index, const GLint size,
    const GLenum type, const GLsizei stride, const GLvoid* pointer) {
    glVertexAttribIPointer(index, size, type, stride, pointer);
    ERROR_CHECK;
  }

  void GLState::glsVertexAttribLPointer(const GLuint index, const GLint size,
    const GLenum type, const GLsizei stride, const GLvoid* pointer) {
    glVertexAttribLPointer(index, size, type, stride, pointer);
    ERROR_CHECK;
  }

  void GLState::glsEnableVertexAttribArray(const GLuint index) {
    glEnableVertexAttribArray(index);
    ERROR_CHECK;
  }

  void GLState::glsCompileShader(const GLuint shader) {
    glCompileShader(shader);
    ERROR_CHECK;
  }

  void GLState::glsShaderSource(const GLuint shader, const GLsizei count, 
    const GLchar** string, const GLint* length){
    glShaderSource(shader, count, string, length);
    ERROR_CHECK;
  }

  void GLState::glsDeleteShader(const GLuint shader) {
    glDeleteShader(shader);
    ERROR_CHECK;
  }

  GLuint GLState::glsCreateShader(const GLenum shaderType) {
    GLuint ret_val = glCreateShader(shaderType);
    ERROR_CHECK;
    return ret_val;
  }

  void GLState::glsGetShaderiv(const GLuint shader, const GLenum pname, 
    GLint *params) {
    glGetShaderiv(shader, pname, params);
    ERROR_CHECK;
  }

  void GLState::glsGetShaderInfoLog(const GLuint shader, 
    const GLsizei maxLength, GLsizei* length, GLchar* infoLog) {
    glGetShaderInfoLog(shader, maxLength, length, infoLog);
    ERROR_CHECK;
  }
  
  void GLState::glsAttachShader(const GLuint program, const GLuint shader) {
    glAttachShader(program, shader);
    ERROR_CHECK;
  }

  GLuint GLState::glsCreateProgram(void) {
    GLuint ret_val = glCreateProgram();
    ERROR_CHECK;
    return ret_val;
  }

  void GLState::glsLinkProgram(const GLuint program) {
    glLinkProgram(program);
    ERROR_CHECK;
  }

  void GLState::glsGetProgramiv(const GLuint program, const GLenum pname, 
    GLint* params) {
    glGetProgramiv(program, pname, params);
    ERROR_CHECK;
  }

  void GLState::glsGetProgramInfoLog(const GLuint program, 
    const GLsizei maxLength, GLsizei* length, GLchar* infoLog) {
    glGetProgramInfoLog(program, maxLength, length, infoLog);
    ERROR_CHECK;
  }

  void GLState::glsGetActiveUniform(const GLuint program, const GLuint index,
    const GLsizei bufSize, GLsizei* length, GLint* size, GLenum* type, 
    GLchar* name) {
    glGetActiveUniform(program, index, bufSize, length, size, type, name);
    ERROR_CHECK;
  }

  GLint GLState::glsGetUniformLocation(const GLuint program, 
    const GLchar* name) {
    GLint ret_val = glGetUniformLocation(program, name);
    ERROR_CHECK;
    return ret_val;
  }

  void GLState::glsBindFragDataLocation(const GLuint program, 
    const GLuint colorNumber, const char* name) {
    glBindFragDataLocation(program, colorNumber, name);
    ERROR_CHECK;
  }

  void GLState::glsBindAttribLocation(const GLuint program, const GLuint index,
    const GLchar* name) {
    glBindAttribLocation(program, index, name);
    ERROR_CHECK;
  }

  void GLState::glsDetachShader(const GLuint program, const GLuint shader) {
    glDetachShader(program, shader);
    ERROR_CHECK;
  }

  void GLState::glsDeleteProgram(const GLuint program) {
    glDeleteProgram(program);
    ERROR_CHECK;
  }

  void GLState::glsUseProgram(const GLuint program) {
    glUseProgram(program);
    ERROR_CHECK;
  }

  GLint GLState::glsGetAttribLocation(GLuint program, const GLchar* name) {
    GLint ret_val = glGetAttribLocation(program, name);
    ERROR_CHECK;
    return ret_val;
  }

  void GLState::UniformFuncs::glsUniform1f(const GLint location, 
    const GLfloat v0) {
    glUniform1f(location, v0);
    ERROR_CHECK;
  }

  void GLState::UniformFuncs::glsUniform2f(const GLint location, 
    const GLfloat v0, const GLfloat v1) {
    glUniform2f(location, v0, v1);
    ERROR_CHECK;
  }

  void GLState::UniformFuncs::glsUniform3f(const GLint location, 
    const GLfloat v0, const GLfloat v1, const GLfloat v2) {
    glUniform3f(location, v0, v1, v2);
    ERROR_CHECK;
  }

  void GLState::UniformFuncs::glsUniform4f(const GLint location, 
    const GLfloat v0, const GLfloat v1, const GLfloat v2, const GLfloat v3) {
    glUniform4f(location, v0, v1, v2, v3);
    ERROR_CHECK;
  }

  void GLState::UniformFuncs::glsUniform1i(const GLint location, 
    const GLint v0) {
    glUniform1i(location, v0);
    ERROR_CHECK;
  }

  void GLState::UniformFuncs::glsUniform2i(const GLint location, 
    const GLint v0, const GLint v1) {
    glUniform2i(location, v0, v1);
    ERROR_CHECK;
  }

  void GLState::UniformFuncs::glsUniform3i(const GLint location, 
    const GLint v0, const GLint v1, const GLint v2) {
    glUniform3i(location, v0, v1, v2);
    ERROR_CHECK;
  }

  void GLState::UniformFuncs::glsUniform4i(const GLint location, 
    const GLint v0, const GLint v1, const GLint v2, const GLint v3) {
    glUniform4i(location, v0, v1, v2, v3);
    ERROR_CHECK;
  }

  void GLState::UniformFuncs::glsUniform1ui(const GLint location, 
    const GLuint v0) {
    glUniform1ui(location, v0);
    ERROR_CHECK;
  }

  void GLState::UniformFuncs::glsUniform2ui(const GLint location, 
    const GLuint v0, const GLuint v1) {
    glUniform2ui(location, v0, v1);
    ERROR_CHECK;
  }

  void GLState::UniformFuncs::glsUniform3ui(const GLint location,
    const GLuint v0, const GLuint v1, const GLuint v2) {
    glUniform3ui(location, v0, v1, v2);
    ERROR_CHECK;
  }

  void GLState::UniformFuncs::glsUniform4ui(const GLint location, 
    const GLuint v0, const GLuint v1, const GLuint v2, const GLuint v3) {
    glUniform4ui(location, v0, v1, v2, v3);
    ERROR_CHECK;
  }

  void GLState::UniformFuncs::glsUniform1fv(const GLint location, 
    const GLsizei count, const GLfloat* value) {
    glUniform1fv(location, count, value);
    ERROR_CHECK;
  }

  void GLState::UniformFuncs::glsUniform2fv(const GLint location, 
    const GLsizei count, const GLfloat* value) {
    glUniform2fv(location, count, value);
    ERROR_CHECK;
  }

  void GLState::UniformFuncs::glsUniform3fv(const GLint location, 
    const GLsizei count, const GLfloat* value) {
    glUniform3fv(location, count, value);
    ERROR_CHECK;
  }

  void GLState::UniformFuncs::glsUniform4fv(const GLint location, 
    const GLsizei count, const GLfloat* value) {
    glUniform4fv(location, count, value);
    ERROR_CHECK;
  }

  void GLState::UniformFuncs::glsUniform1iv(const GLint location, 
    const GLsizei count, const GLint* value) {
    glUniform1iv(location, count, value);
    ERROR_CHECK;
  }

  void GLState::UniformFuncs::glsUniform2iv(const GLint location, 
    const GLsizei count, const GLint* value) {
    glUniform2iv(location, count, value);
    ERROR_CHECK;
  }

  void GLState::UniformFuncs::glsUniform3iv(const GLint location, 
    const GLsizei count, const GLint* value) {
    glUniform3iv(location, count, value);
    ERROR_CHECK;
  }

  void GLState::UniformFuncs::glsUniform4iv(const GLint location, 
    const GLsizei count, const GLint* value) {
    glUniform4iv(location, count, value);
    ERROR_CHECK;
  }

  void GLState::UniformFuncs::glsUniform1uiv(const GLint location, 
    const GLsizei count, const GLuint* value) {
    glUniform1uiv(location, count, value);
    ERROR_CHECK;
  }

  void GLState::UniformFuncs::glsUniform2uiv(const GLint location, 
    const GLsizei count, const GLuint* value) {
    glUniform2uiv(location, count, value);
    ERROR_CHECK;
  }

  void GLState::UniformFuncs::glsUniform3uiv(const GLint location, 
    const GLsizei count, const GLuint* value) {
    glUniform3uiv(location, count, value);
    ERROR_CHECK;
  }

  void GLState::UniformFuncs::glsUniform4uiv(const GLint location, 
    const GLsizei count, const GLuint* value) {
    glUniform4uiv(location, count, value);
    ERROR_CHECK;
  }

  void GLState::UniformFuncs::glsUniformMatrix2fv(const GLint location, 
    const GLsizei count, const GLboolean transpose, const GLfloat* value) {
    glUniformMatrix2fv(location, count, transpose, value);
    ERROR_CHECK;
  }

  void GLState::UniformFuncs::glsUniformMatrix3fv(const GLint location, 
    const GLsizei count, const GLboolean transpose, const GLfloat* value) {
    glUniformMatrix3fv(location, count, transpose, value);
    ERROR_CHECK;
  }

  void GLState::UniformFuncs::glsUniformMatrix4fv(const GLint location, 
    const GLsizei count, const GLboolean transpose, const GLfloat* value) {
    glUniformMatrix4fv(location, count, transpose, value);
    ERROR_CHECK;
  }

  void GLState::glsGenerateMipmap(const GLenum target) {
    glGenerateMipmap(target);
    ERROR_CHECK;
  }

  void GLState::glsDeleteTextures(const GLsizei n, const GLuint* textures) {
    glDeleteTextures(n, textures);
    ERROR_CHECK;
  }

  void GLState::glsGenTextures(const GLsizei n, GLuint* textures) {
    glGenTextures(n, textures);
    ERROR_CHECK;
  }

  void GLState::glsTexImage2D(const GLenum target, const GLint level, 
    const GLint internalFormat, const GLsizei width, const GLsizei height, 
    const GLint border, const GLenum format, const GLenum type, 
    const GLvoid* data) {
    glTexImage2D(target, level, internalFormat, width, height, border, 
      format, type, data);
    ERROR_CHECK;
  }

  void GLState::glsTexImage3D(const GLenum target, const GLint level,
    const GLint internalFormat, const GLsizei width, const GLsizei height,
    const GLsizei depth, const GLint border, const GLenum format,
    const GLenum type, const GLvoid* data) {
    glTexImage3D(target, level, internalFormat, width, height, depth, border,
      format, type, data);
    ERROR_CHECK;
  }

  void GLState::glsTexParameterf(const GLenum target, const GLenum pname,
    const GLfloat param) {
    glTexParameterf(target, pname, param);
    ERROR_CHECK;
  }

  void GLState::glsTexParameteri(const GLenum target, const GLenum pname,
    const GLint param) {
    glTexParameteri(target, pname, param);
    ERROR_CHECK;
  }

  void GLState::glsTexParameterfv(const GLenum target, const GLenum pname,
    const GLfloat* params) {
    glTexParameterfv(target, pname, params);
    ERROR_CHECK;
  }

  void GLState::glsGenFramebuffers(GLsizei n, GLuint* ids) {
    glGenFramebuffers(n, ids);
    ERROR_CHECK;
  }

  void GLState::glsFramebufferTexture(const GLenum target, 
    const GLenum attachment, const GLuint texture, const GLint level) {
    glFramebufferTexture(target, attachment, texture, level);
    ERROR_CHECK;
  }

  void GLState::glsFramebufferTexture2D(const GLenum target, 
    const GLenum attachment, const GLenum textarget, const GLuint texture,
    const GLint level) {
    glFramebufferTexture2D(target, attachment, textarget, texture, level);
    ERROR_CHECK;
  }

  void GLState::glsFramebufferTexture3D(const GLenum target, 
    const GLenum attachment, const GLenum textarget, const GLuint texture,
    const GLint level, const GLint layer) {
    glFramebufferTexture3D(target, attachment, textarget, texture, level,
      layer);
    ERROR_CHECK;
  }

  GLenum GLState::glsGetError(void) {
    return glGetError();
  }

  const GLubyte* GLState::glsuErrorString(const GLenum error) {
    return gluErrorString(error);
  }

  void GLState::glsDeleteFramebuffers(const GLsizei n, 
    const GLuint *framebuffers) {
    glDeleteFramebuffers(n, framebuffers);
    ERROR_CHECK;
  }

  void GLState::glsFramebufferTextureLayer(const GLenum target, 
    const GLenum attachment, const GLuint texture, const GLint level,
    const GLint layer) {
    glFramebufferTextureLayer(target, attachment, texture, level, layer);
    ERROR_CHECK;
  }

  void GLState::glsBufferSubData(const GLenum target, const GLintptr offset,
    const GLsizeiptr size, const GLvoid* data) {
    glBufferSubData(target, offset, size, data);
    ERROR_CHECK;
  }

  // INTERNAL FUNCTIONS TO QUERY AND SET THE LOCAL STATE

  bool GLState::setGLenum(const GLenum state_name, const void* data, 
    const uint32_t size, const GLType type) {
    state_data_mutex_.lock();
    uint32_t index;
    if (!ht_GLEnum_2_idata_->lookup(static_cast<uint32_t>(state_name), 
      index)) {
      index = idata_2_data_->size();  // Assign a new index
      GLStateElem* new_elem = new GLStateElem(state_name, data, size, type);
      idata_2_data_->pushBack(new_elem);
      ht_GLEnum_2_idata_->insert(static_cast<uint32_t>(state_name), index);
      state_data_mutex_.unlock();
      return true;  // Force a state change
    }
    if (*(*idata_2_data_)[index] != data) {
      *(*idata_2_data_)[index] = data;
      state_data_mutex_.unlock();
      return true;
    } else {
      state_data_mutex_.unlock();
#ifndef FORCE_STATE_SETTING
      return false;
#else
      return true;
#endif
    }
  }

  void GLState::getGLenum(const GLenum state_name, void* ret_data, 
    const uint32_t size, const GLType type) {
    uint32_t index;
    state_data_mutex_.lock();
    if (!ht_GLEnum_2_idata_->lookup(static_cast<uint32_t>(state_name), 
      index)) {
      glsGetStateData(state_name, type, temp_data_);

      index = idata_2_data_->size();  // Assign a new index
      GLStateElem* new_elem = new GLStateElem(state_name, temp_data_, size, 
        type);
      idata_2_data_->pushBack(new_elem);
      ht_GLEnum_2_idata_->insert(static_cast<uint32_t>(state_name), index);
      
    }
    GLStateElem* cur_elem = (*idata_2_data_)[index];
    if (cur_elem->size != size || cur_elem->type != type) {
      state_data_mutex_.unlock();
      throw std::runtime_error("GLState::getGLenum() - ERROR: Retrieved state"
        " params do not match the input params!");
    }
    for (uint32_t i = 0; i < cur_elem->size; i++) {
      ((uint8_t*)ret_data)[i] = ((uint8_t*)cur_elem->data)[i];
    }
    state_data_mutex_.unlock();
  }
}
