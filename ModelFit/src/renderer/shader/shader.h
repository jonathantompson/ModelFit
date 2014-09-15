//
//  shader.h
//
//  Created by Jonathan Tompson on 6/7/12.
//
//  A simple wrapper class to load OpenGL shaders from disk and manage their
//  resources

#pragma once

#include <string>
#include "renderer/open_gl_common.h"  // For GL* types and ERROR_CHECK
#include "data_str/vector.h"

namespace renderer {

  enum ShaderType {
    VERTEX_SHADER,
    FRAGMENT_SHADER
  };

  class Shader {
  public:

    // Constructor / Destructor
    Shader(const std::string& filename, ShaderType type);
    ~Shader();

    // getter and setter methods
    inline GLuint shader() { return shader_; }
    inline bool compiled() { return compiled_; }
    inline std::wstring& filename() { return filename_; }

    void printToStdOut();

  private:
    bool compiled_;
    GLchar* shader_source_;
    GLuint shader_;
    std::wstring filename_;
    static const std::string inc_str_;

    static char* readFileToBuffer(const std::string& filename);
    int findInclude(GLchar* source);
    std::string extractIncludeFilename(GLchar* source, int inc_pos);
    GLchar* insertIncludeSource(GLchar* source, GLchar* inc, int inc_pos);

    // Non-copyable, non-assignable.
    Shader(Shader&);
    Shader& operator=(const Shader&);
  };
};  // renderer namespace
