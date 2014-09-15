//
//  gl_state.h
//
//  Created by Jonathan Tompson on 1/8/13.
//
//  Interface layer to prevent redundant setting of openGL render states.
//  One should be created per context.
//  

#pragma once

#include <mutex>
#include "renderer/open_gl_common.h"
#include "math/math_types.h"
#include "data_str/pair.h"

namespace jtil {
namespace data_str { template <typename T> class VectorManaged; }
namespace data_str { template <class TKey, class TValue> class HashMap; }
}

#define MAX_NUM_TEXTURE_UNITS 256
#define FORCE_STATE_SETTING  // Define to force all redundant state changes
#define MAX_DATA_SIZE 256

namespace renderer {

  struct GLenum2;
  struct GLfloat4;
  struct GLint4;
  class ShaderProgram;
  class Texture;
  class TextureRenderable;

  typedef enum {
    TEXTURE_1D = 0,
    TEXTURE_2D = 1,
    TEXTURE_3D = 2,
    TEXTURE_1D_ARRAY = 3,
    TEXTURE_2D_ARRAY = 4,
    TEXTURE_RECTANGLE = 5,
    TEXTURE_CUBE_MAP = 6,
    TEXTURE_CUBE_MAP_ARRAY = 7,
    TEXTURE_BUFFER = 8,
    TEXTURE_2D_MULTISAMPLE = 9,
    TEXTURE_2D_MULTISAMPLE_ARRAY = 10,
    NUM_TEXTURE_TARGETS = 11,
    TEXTURE_UNDEFINED = 12,
  } GLStateTexture;

  typedef enum {
    GLTYPE_INT = 0,
    GLTYPE_BOOL = 1,
    GLTYPE_ENUM = 2,
    GLTYPE_FLOAT = 3,
    GLTYPE_INT64 = 4,
    GLTYPE_DOUBLE = 5,
  } GLType;

  struct GLStateElem {
    GLenum state_name;
    void* data;  // data storage abstracted
    uint32_t size;  // size is in units of 1 byte (8 bits), total size
    int type;

    GLStateElem(const GLenum state_name, const void* data, const uint32_t size,
      const int type);
    ~GLStateElem();
    GLStateElem& operator=(const void* rhs);
    bool operator==(const void* rhs);
    bool operator!=(const void* rhs);
  };

  class GLState {
  public:
    virtual ~GLState() = 0;  // Abstact class
    static void initGLState();
    static void shutdownGLState();

    // verifyOpenGLState - Expensive.  Stalls GPU pipeline to get state.
    //                     Used in debug mode to check consistency
    static void verifyOpenGLState();

    // Set open GL states
    static void glsEnable(const GLenum cap);
    static void glsDisable(const GLenum cap);
    static void glsDepthFunc(const GLenum func);
    static void glsDepthMask(const GLboolean flag);
    static void glsPolygonMode(const GLenum face, const GLenum mode);
    static void glsCullFace(const GLenum mode);
    static void glsBlendEquation(const GLenum mode);
    static void glsBlendFunc(const GLenum sfactor, const GLenum dfactor);
    static void glsDrawBuffer(const GLenum mode);
    static void glsClearColor(const GLfloat red, const GLfloat green, 
      const GLfloat blue, const GLfloat alpha);
    static void glsCheckFramebufferStatus(const GLenum target);
    static void glsViewport(const GLint x, const GLint y, const GLsizei width, 
      const GLsizei height);
    static void glsBindFramebuffer(const GLenum target, const GLuint framebuffer);
    static void glsClear(const GLbitfield mask);
    static void glsStencilOpSeparate(const GLenum face, const GLenum sfail, 
      const GLenum dpfail, const GLenum dppass);
    static void glsStencilOp(const GLenum sfail, const GLenum dpfail, 
      const GLenum dppass);
    static void glsStencilFuncSeparate(const GLenum face, const GLenum func,
      const GLint ref, const GLuint mask);
    static void glsStencilFunc(const GLenum func, const GLint ref, const GLuint mask);
    static void glsBindVertexArray(const GLuint arr);
    static void glsDrawArrays(const GLenum mode, const GLint first, 
      const GLsizei count);
    static void glsDrawElements(const GLenum mode, const GLsizei count,
      const GLenum type, const GLvoid* indices);
    static void glsBindTexture(const GLenum target, const GLuint texture);
    static void glsActiveTexture(const GLenum texture);
    static void glsDrawBuffers(const GLsizei n, const GLenum* bufs);
    static void glsDeleteBuffers(const GLsizei n, GLuint* buffers);
    static void glsDeleteVertexArrays(const GLsizei n, GLuint* arrays);
    static void glsGenVertexArrays(const GLsizei n, GLuint* arrays);
    static void glsGenBuffers(const GLsizei n, GLuint* buffers);
    static void glsBindBuffer(const GLenum target, const GLuint buffer);
    static void glsBufferData(const GLenum target, const GLsizeiptr size, 
      const GLvoid* data, const GLenum usage);
    static void* glsMapBuffer(const GLenum target, const GLenum access);
    static GLboolean glsUnmapBuffer(const GLenum target);
    static void glsVertexAttribPointer(const GLuint index, const GLint size,
      const GLenum type, const GLboolean normalized, const GLsizei stride, 
      const GLvoid* pointer);
    static void glsVertexAttribIPointer(const GLuint index, const GLint size,
      const GLenum type, const GLsizei stride, const GLvoid* pointer);
    static void glsVertexAttribLPointer(const GLuint index, const GLint size,
      const GLenum type, const GLsizei stride, const GLvoid* pointer);
    static void glsEnableVertexAttribArray(const GLuint index);
    static void glsCompileShader(const GLuint shader);
    static void glsShaderSource(const GLuint shader, const GLsizei count, 
      const GLchar** string, const GLint* length);
    static void glsDeleteShader(const GLuint shader);
    static GLuint glsCreateShader(const GLenum shaderType);
    static void glsGetShaderiv(const GLuint shader, const GLenum pname, 
      GLint *params);
    static void glsGetShaderInfoLog(const GLuint shader, 
      const GLsizei maxLength, GLsizei* length, GLchar* infoLog);
    static void glsAttachShader(const GLuint program, const GLuint shader);
    static GLuint glsCreateProgram(void);
    static void glsLinkProgram(const GLuint program);
    static void glsGetProgramiv(const GLuint program, const GLenum pname, 
      GLint* params);
    static void glsGetProgramInfoLog(const GLuint program, 
      const GLsizei maxLength, GLsizei* length, GLchar* infoLog);
    static void glsGetActiveUniform(const GLuint program, const GLuint index,
      const GLsizei bufSize, GLsizei* length, GLint* size, GLenum* type, 
      GLchar* name);
    static GLint glsGetUniformLocation(const GLuint program, 
      const GLchar* name);
    static void glsBindFragDataLocation(const GLuint program, 
      const GLuint colorNumber, const char* name);
    static void glsBindAttribLocation(const GLuint program, const GLuint index,
      const GLchar* name);
    static void glsDetachShader(const GLuint program, const GLuint shader);
    static void glsDeleteProgram(const GLuint program);
    static void glsUseProgram(const GLuint program);
    static GLint glsGetAttribLocation(GLuint program, const GLchar* name);
    static void glsGenerateMipmap(const GLenum target);
    static void glsDeleteTextures(const GLsizei n, const GLuint* textures);
    static void glsGenTextures(const GLsizei n, GLuint* textures);
    static void glsTexImage2D(const GLenum target, const GLint level, 
      const GLint internalFormat, const GLsizei width, const GLsizei height, 
      const GLint border, const GLenum format, const GLenum type, 
      const GLvoid* data);
    static void glsTexImage3D(const GLenum target, const GLint level,
      const GLint internalFormat, const GLsizei width, const GLsizei height,
      const GLsizei depth, const GLint border, const GLenum format,
      const GLenum type, const GLvoid* data);
    static void glsTexParameterf(const GLenum target, const GLenum pname,
      const GLfloat param);
    static void glsTexParameteri(const GLenum target, const GLenum pname,
      const GLint param);
    static void glsTexParameterfv(const GLenum target, const GLenum pname,
      const GLfloat* params);
    static void glsGenFramebuffers(const GLsizei n, GLuint* ids);
    static void glsFramebufferTexture(const GLenum target, 
      const GLenum attachment, const GLuint texture, const GLint level);
    static void glsFramebufferTexture2D(const GLenum target, 
      const GLenum attachment, const GLenum textarget, const GLuint texture,
      const GLint level);
    static void glsFramebufferTexture3D(const GLenum target, 
      const GLenum attachment, const GLenum textarget, const GLuint texture,
      const GLint level, const GLint layer);
    static GLenum glsGetError(void);
    static const GLubyte* glsuErrorString(const GLenum error);
    static void glsDeleteFramebuffers(const GLsizei n, 
      const GLuint *framebuffers);
    static void glsFramebufferTextureLayer(const GLenum target, 
      const GLenum attachment, const GLuint texture, const GLint level,
      const GLint layer);
    static void glsBufferSubData(const GLenum target, const GLintptr offset,
      const GLsizeiptr size, const GLvoid* data);
    static void glsScissor(const GLint x, const GLint y, const GLsizei width, 
      const GLsizei height);

    // Functions that encapsulate a few state changes
    static void setupQuadRendering();

    // Only ShaderProgram should have access to these functions...
    class UniformFuncs {
      friend class ShaderProgram;
      friend class Texture;
      friend class TextureRenderable;
    private:
      // NOTE: ALL uniforms should be bound through the ShaderProgram interface.
      static void glsUniform1f(const GLint location, 
        const GLfloat v0);
      static void glsUniform2f(const GLint location, const GLfloat v0, 
        const GLfloat v1);
      static void glsUniform3f(const GLint location, const GLfloat v0, 
        const GLfloat v1, const GLfloat v2);
      static void glsUniform4f(const GLint location, const GLfloat v0, 
        const GLfloat v1, const GLfloat v2, const GLfloat v3);
      static void glsUniform1i(const GLint location, const GLint v0);
      static void glsUniform2i(const GLint location, const GLint v0, 
        const GLint v1);
      static void glsUniform3i(const GLint location, const GLint v0, 
        const GLint v1, const GLint v2);
      static void glsUniform4i(const GLint location, const GLint v0, 
        const GLint v1, const GLint v2, const GLint v3);
      static void glsUniform1ui(const GLint location, const GLuint v0);
      static void glsUniform2ui(const GLint location, const GLuint v0, 
        const GLuint v1);
      static void glsUniform3ui(const GLint location, const GLuint v0, 
        const GLuint v1, const GLuint v2);
      static void glsUniform4ui(const GLint location, const GLuint v0, 
        const GLuint v1, const GLuint v2, const GLuint v3);
      static void glsUniform1fv(const GLint location, const GLsizei count, 
        const GLfloat* value);
      static void glsUniform2fv(const GLint location, const GLsizei count, 
        const GLfloat* value);
      static void glsUniform3fv(const GLint location, const GLsizei count, 
        const GLfloat* value);
      static void glsUniform4fv(const GLint location, const GLsizei count, 
        const GLfloat* value);
      static void glsUniform1iv(const GLint location, const GLsizei count, 
        const GLint* value);
      static void glsUniform2iv(const GLint location, const GLsizei count, 
        const GLint* value);
      static void glsUniform3iv(const GLint location, const GLsizei count, 
        const GLint* value);
      static void glsUniform4iv(const GLint location, const GLsizei count, 
        const GLint* value);
      static void glsUniform1uiv(const GLint location, const GLsizei count, 
        const GLuint* value);
      static void glsUniform2uiv(const GLint location, const GLsizei count, 
        const GLuint* value);
      static void glsUniform3uiv(const GLint location, const GLsizei count, 
        const GLuint* value);
      static void glsUniform4uiv(const GLint location, const GLsizei count, 
        const GLuint* value);
      static void glsUniformMatrix2fv(const GLint location, 
        const GLsizei count, const GLboolean transpose, const GLfloat* value);
      static void glsUniformMatrix3fv(const GLint location, 
        const GLsizei count, const GLboolean transpose, const GLfloat* value);
      static void glsUniformMatrix4fv(const GLint location, 
        const GLsizei count, const GLboolean transpose, const GLfloat* value);
    };
  private:
   
    // For verification we need linear enumeration of the hash table values,
    // so use a hash table to find the lookup index into a vector.

    // Enum name --> data mapping
    static jtil::data_str::HashMap<uint32_t, uint32_t>* ht_GLEnum_2_idata_;
    static jtil::data_str::VectorManaged<GLStateElem*>* idata_2_data_;
    static std::mutex startup_mutex_;
    static bool gl_state_initialized_;

    // Textures
    static GLStateTexture GLTarget2GLStateTexture(const GLenum target);
    static GLenum GLStateTexture2GLTargetBinding(const GLStateTexture target);
    static GLuint bound_textures_[MAX_NUM_TEXTURE_UNITS][NUM_TEXTURE_TARGETS];
    static GLint max_texture_units_;

    // Temp data
    static uint8_t temp_data_[MAX_DATA_SIZE];
    static std::mutex state_data_mutex_;

    // set the gl value in the database, returns true if the value didn't match
    // Size is in units of 1 byte
    static bool setGLenum(const GLenum state_name, const void* data, 
      const uint32_t size, const GLType type);
    static void getGLenum(const GLenum state_name, void* ret_data, 
      const uint32_t size, const GLType type);
    static void glsGetStateData(const GLenum state_name, const GLType type, 
      void* data);

  };

};  // renderer namespace
