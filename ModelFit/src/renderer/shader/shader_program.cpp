#include <sstream>
#include <string>
#include <iostream>
#include "renderer/shader/shader_program.h"
#include "renderer/shader/shader.h"
#include "string_util/string_util.h"
#include "data_str/pair.h"
#include "renderer/gl_state.h"
#include "string_util/string_util.h"

using std::wstring;
using std::string;
using std::runtime_error;
using std::string;
using std::cout;
using std::endl;
using jtil::data_str::Pair;
using jtil::data_str::Vector;

namespace renderer {
  GLuint ShaderProgram::cur_program_ = MAX_UINT32;
  std::mutex ShaderProgram::cur_program_mutex_;

  ShaderProgram::ShaderProgram(Shader* vertex_shader, 
    Shader* fragment_shader) {
    linked_ = false;
    if (!vertex_shader->compiled() || !fragment_shader->compiled()) {
      throw std::runtime_error(string("ShaderProgram::ShaderProgram() - ERROR") +
        string(": one of the shaders is not compiled!"));
    }

    vertex_shader_ = vertex_shader;
    fragment_shader_ = fragment_shader;
    
    shader_program_ = GLState::glsCreateProgram();

    // Attach the shaders to this shader program
    GLState::glsAttachShader(shader_program_, vertex_shader_->shader());
    GLState::glsAttachShader(shader_program_, fragment_shader_->shader());
  }

  void ShaderProgram::link() {
    // Link the program
    GLState::glsLinkProgram(shader_program_);
    ERROR_CHECK;

    // At this stage, the vertex and fragment programs are inspected, optimized
    // and a binary code is generated for the shader.
    // The binary code is uploaded to the GPU, if there is no error.

    // We must check and make sure that it linked. If it fails, it would
    // mean either there is a mismatch between the vertex and fragment shaders.
    // It might be that you have surpassed your GPU's abilities. Perhaps too 
    // many ALU operations or too many texel fetch instructions or too many 
    // interpolators or dynamic loops.
    int is_linked;
    GLState::glsGetProgramiv(shader_program_, GL_LINK_STATUS, &is_linked);
    if(is_linked == 0) {
      // Noticed that glGetProgramiv is used to get the length for a shader 
      // program, not glGetShaderiv.
      int info_length;
      GLState::glsGetProgramiv(shader_program_, GL_INFO_LOG_LENGTH, &info_length);

      // The maxLength includes the NULL character
      char* program_info_log;
      program_info_log = new char[info_length];

      // Again we use glGetProgramInfoLog, not glGetShaderInfoLog.
      GLState::glsGetProgramInfoLog(shader_program_, info_length, &info_length, 
        program_info_log);
      ERROR_CHECK;
      wstring err_log = jtil::string_util::ToWideString(program_info_log);
      delete[] program_info_log;

      throw std::runtime_error(string("ShaderProgram::ShaderProgram() - ERROR ") + 
        string("compiling shader program from file: ") + 
        jtil::string_util::ToNarrowString(vertex_shader_->filename()) + string(" and ") + 
        jtil::string_util::ToNarrowString(fragment_shader_->filename()) + string(": ") + 
        jtil::string_util::ToNarrowString(err_log));
      return;
    } else {
      linked_ = true;
    }
  }

  void ShaderProgram::bindFragShaderOutputLocation(
    const ShaderLocationNamePair output) {
#if defined(DEBUG) || defined(_DEBUG)
    if (linked_) {
      throw std::runtime_error(string("ShaderProgram::bindFragShaderOutput") +
        string("Location() - Error, trying to bind a frag shader location") +
        string(" on a shader program that is already linked"));
    }
#endif
    GLState::glsBindFragDataLocation(shader_program_, output.id, output.name.c_str());
  }

  void ShaderProgram::bindVertShaderInputLocation(
    const ShaderLocationNamePair input) {
#if defined(DEBUG) || defined(_DEBUG)
    if (linked_) {
      throw std::runtime_error(string("ShaderProgram::bindFragShaderOutput") +
        string("Location() - Error, trying to bind a frag shader location") +
        string(" on a shader program that is already linked"));
    }
#endif
    GLint n_vertex_attrib;
    glGetIntegerv(GL_MAX_VERTEX_ATTRIBS, &n_vertex_attrib);
    if (input.id >= n_vertex_attrib) {
      throw std::runtime_error(string("ShaderProgram::bindFragShaderOutput") +
        string("Location() - Error, vertex attribute location >= ") +
        string(" GL_MAX_VERTEX_ATTRIBS"));
    }
    GLState::glsBindAttribLocation(shader_program_, input.id, input.name.c_str());
    ERROR_CHECK;
  }

  ShaderProgram::~ShaderProgram() {
    GLState::glsDetachShader(shader_program_, vertex_shader_->shader());
    GLState::glsDetachShader(shader_program_, fragment_shader_->shader());
    GLState::glsDeleteProgram(shader_program_);
  }

  void ShaderProgram::useProgram() {
#if defined(DEBUG) || defined(_DEBUG)
    if (!linked_) {
      throw std::runtime_error(string("ShaderProgram::useProgram() - ") +
        string("Error, trying to use a shader program that is not linked!"));
    }
#endif
    cur_program_mutex_.lock();
    if (cur_program_ != shader_program_) {
      cur_program_ = shader_program_;
      GLState::glsUseProgram(shader_program_);
    }
    cur_program_mutex_.unlock();
  }

  GLint ShaderProgram::getUniformLocation(const char* name) {
#if defined(DEBUG) || defined(_DEBUG)
    if (!linked_) {
      throw std::runtime_error(string("ShaderProgram::getUniformLocation() - ") +
        string("Error, shader program is not linked!"));
    }
#endif
    GLint Location = GLState::glsGetUniformLocation(shader_program_, name);
    if (static_cast<uint32_t>(Location) == 0xFFFFFFFF) {
      // First spew the shader code:
      cout << "ERROR: Cannot find: " << name << " in: " << endl;
      cout << "*********************************************************";
      cout << endl << "1. Vertex Shader Code: ";
      cout << jtil::string_util::ToNarrowString(vertex_shader_->filename()) << endl;
      vertex_shader_->printToStdOut();
      cout << endl;
      cout << "*********************************************************";
      cout << endl << "2. Fragment Shader Code: ";
      cout << jtil::string_util::ToNarrowString(fragment_shader_->filename()) << endl;
      fragment_shader_->printToStdOut();
      cout << endl;
      cout << "*********************************************************";
      cout << endl;

      std::stringstream ss;
      ss << "ShaderProgram::getUniformLocation() - ERROR: Unable to get the";
      ss << " location of uniform '" << name << "'";
      throw std::runtime_error(ss.str());
    }
    ERROR_CHECK;
    return Location;
  }

  void ShaderProgram::setShaderAttrib(const char *attribName, int size, 
    GLenum type, bool normalized, int stride, const void* pointer) {
    int id = GLState::glsGetAttribLocation(shader_program_, attribName);
    ERROR_CHECK;
    if(id < 0) {
      std::stringstream ss;
      ss << "ShaderProgram::setShaderAttrib() - ERROR: Unable to get the";
      ss << " location of attribute '" << attribName << "'";
      throw std::runtime_error(ss.str());
    }
    GLState::glsVertexAttribPointer(id, size, type, normalized, stride, pointer);
    ERROR_CHECK;
    GLState::glsEnableVertexAttribArray(id);
    ERROR_CHECK;
  }

  GLint ShaderProgram::getNumUniforms() {
    GLint num_uniforms;
    GLState::glsGetProgramiv(shader_program_, GL_ACTIVE_UNIFORMS, &num_uniforms);
    ERROR_CHECK;
    return num_uniforms;
  }

}  // namespace misc
