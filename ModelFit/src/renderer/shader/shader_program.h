//
//  shader_program.h
//
//  Created by Jonathan Tompson on 6/7/12.
//
//  A simple wrapper class to build and manage shader programs

#pragma once

#include <string>
#include <mutex>
#include "renderer/open_gl_common.h"  // For GL Types (GLint) and ERROR_CHECK
#include "math/math_types.h"
#include "data_str/vector.h"
#include "data_str/pair.h"
#include "renderer/shader/shader_location_name_pair.h"

namespace renderer {

  class Shader;

  class ShaderProgram {
  public:
    // frag_output_loc -> Use this to explicitly specify the fragment shader 
    //                    output locations (used when setting up render to 
    //                    texture).
    // vert_input_loc -> Use this to explicitly specify the vertex shader input
    //                   locations.
    ShaderProgram(Shader* vertex_shader, Shader* fragment_shader);
    ~ShaderProgram();

    // Before linking you can explicitly bind the fragment shader output 
    // locations to a user specified value.  Used when rendering to textures.
    void bindFragShaderOutputLocation(const ShaderLocationNamePair output);

    // Before linking you can explicitly bind the vertex shader input locations
    // to a user specified value.
    void bindVertShaderInputLocation(const ShaderLocationNamePair input);

    // Link the program before use!
    void link();
    inline bool linked() { return linked_; }

    GLuint shader_program() { return shader_program_; };

    void useProgram();  // call before rendering

    // getter setter methods
    inline GLuint program() { return shader_program_; }
    GLint getUniformLocation(const char* name);
    void setShaderAttrib(const char *attribName, int size, GLenum type, 
      bool normalized, int stride, const void* pointer);
    GLint getNumUniforms();

  private:
    Shader* vertex_shader_;
    Shader* fragment_shader_;
    GLuint shader_program_;
    bool linked_;
    static GLuint cur_program_;
    static std::mutex cur_program_mutex_;

    // Non-copyable, non-assignable.
    ShaderProgram(ShaderProgram&);
    ShaderProgram& operator=(const ShaderProgram&);
  };
};  // renderer namespace
