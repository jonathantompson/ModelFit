#include <cstring>
#include <iostream>
#include <string>
#include "renderer/shader/shader.h"
#include "string_util/string_util.h"
#include "renderer/gl_state.h"

using std::string;
using std::wstring;
using std::runtime_error;

namespace renderer {

  Shader::Shader(const string& filename, ShaderType type) {
    filename_ = jtil::string_util::ToWideString(filename);
    compiled_ = false;

    // Read in the raw shader source
    shader_source_ = readFileToBuffer(filename);

    // Append the include code: We need to scan through the code looking for
    // lines that have "#include" and replace them with the correct text.
    // This could be much more efficient, but is fast enough for just startup
    int include_pos = findInclude(shader_source_);
    while (include_pos != -1) {
      string f_include = extractIncludeFilename(shader_source_, include_pos);
      GLchar* inc_source = readFileToBuffer(f_include);
      shader_source_ = insertIncludeSource(shader_source_, inc_source, 
        include_pos);
      free(inc_source);

      // There might be more, so find the next one
      include_pos = findInclude(shader_source_);
    }

    // Assign a shader handle
    switch (type) {
    case VERTEX_SHADER:
      shader_ = GLState::glsCreateShader(GL_VERTEX_SHADER);
      break;
    case FRAGMENT_SHADER:
      shader_ = GLState::glsCreateShader(GL_FRAGMENT_SHADER);
      break;
    default:
      throw std::runtime_error("Shader::Shader() - unrecognized shader type!");
    }

    // Send the shader source code to OpenGL
    // Note that the source code is NULL character terminated.
    // GL will automatically detect that therefore the length info can be 0 
    // in this case (the last parameter)
    GLState::glsShaderSource(shader_, 1, const_cast<const GLchar**>(&shader_source_), 0);

    // Compile the shader
    GLState::glsCompileShader(shader_);

    // Check that everything compiled OK
    int is_compiled;
    GLState::glsGetShaderiv(shader_, GL_COMPILE_STATUS, &is_compiled);
    if(is_compiled == 0) {
      int info_length;
      GLState::glsGetShaderiv(shader_, GL_INFO_LOG_LENGTH, &info_length);

      // The maxLength includes the NULL character
      char* shader_info_log;
      shader_info_log = new char[info_length];

      GLState::glsGetShaderInfoLog(shader_, info_length, &info_length, shader_info_log);
      ERROR_CHECK;
      wstring err_log = jtil::string_util::ToWideString(shader_info_log);
      delete[] shader_info_log;
      
      std::cout << std::endl;
      std::cout << "Cannot compile the following shader code: ";
      std::cout << jtil::string_util::ToNarrowString(filename_).c_str() << std::endl;
      std::cout << "  --> Compilation error: " << std::endl;
      std::cout << jtil::string_util::ToNarrowString(err_log).c_str();
      std::cout << "  --> For the code:" << std::endl;
      std::cout << "*********************************************************";
      std::cout << std::endl;
      printToStdOut();
      std::cout << "*********************************************************";
      std::cout << std::endl;

      throw std::runtime_error(string("Shader::Shader() - ERROR compiling") +
        string(" shader from file: ") + jtil::string_util::ToNarrowString(filename_) + string(": ") + 
        jtil::string_util::ToNarrowString(err_log));
    } else {
      compiled_ = true;
    }
  }

  Shader::~Shader() {
    GLState::glsDeleteShader(shader_);
    free(shader_source_);
  }

  void Shader::printToStdOut() {
    int cur_line = 1;
    int cur_char = 0;
    std::cout << "Line " << cur_line << ": ";
    int str_length = static_cast<int>(strlen(shader_source_));
    do {
      if (shader_source_[cur_char] == '\n') {
        cur_line++;
        std::cout << std::endl << "Line " << cur_line << ": ";
      } else {
        std::cout << shader_source_[cur_char];
      }
      cur_char++;
    } while (cur_char < str_length);
    std::cout << std::endl;
  }

  // Base code taken from: http://www.opengl.org/wiki/ (tutorial 2)
  char* Shader::readFileToBuffer(const std::string& filename) {
    FILE *fptr;
    long length;
    char *buf;

    fptr = fopen(filename.c_str(), "rb");  // Open file for reading
    if (!fptr) {
      string err = string("Renderer::readFileToBuffer() - ERROR: could not") +
        string(" open file (") + filename + 
        string(") for reading");
      throw std::runtime_error(err);
    }
    fseek(fptr, 0, SEEK_END);  // Seek to the end of the file
    length = ftell(fptr);  // Find out how many bytes into the file we are
    buf = (char*)malloc(length+1);  // Allocate a buffer for the entire length 
                                    // of the file and a null terminator
    fseek(fptr, 0, SEEK_SET);  // Go back to the beginning of the file
    fread(buf, length, 1, fptr);  // Read the contents of the file in to the
                                  // buffer
    fclose(fptr);  // Close the file
    buf[length] = 0;  // Null terminator

    return buf;
  }

  const std::string Shader::inc_str_("#include");
  int Shader::findInclude(GLchar* source) {
    static_cast<void>(source);
    int cur_char = 0;
    int cur_inc_str_ptr = 0;
    int str_length = static_cast<int>(strlen(shader_source_));
    do {
      if (shader_source_[cur_char] == inc_str_.at(cur_inc_str_ptr)) {
        cur_inc_str_ptr++;
        if (cur_inc_str_ptr == static_cast<int>(inc_str_.size())) {
          return cur_char - static_cast<int>(inc_str_.size()) + 1;
        }
      } else {
        cur_inc_str_ptr = 0;  // reset
      }
      cur_char++;
    } while (cur_char < str_length);

    return -1;  // We didn't find anything
  }

  // extractIncludeFilename looks for "xxx" on the current line starting
  // at pos
  string Shader::extractIncludeFilename(GLchar* source, int pos) {
    static_cast<void>(source);
    int str_start = -1;
    int str_end = -1;
    int cur_char = pos;
    int str_length = static_cast<int>(strlen(shader_source_));
    do {
      if (str_start == -1) {
        if (shader_source_[cur_char] == '"') {
          str_start = cur_char + 1;  // Don't include the " character
        }
      } else {
        if (shader_source_[cur_char] == '\n') {
          std::cout << "Shader::extractIncludeFilename() - ERROR: couldn't ";
          std::cout << "extract include filename in ";
          std::cout << jtil::string_util::ToNarrowString(filename_) << std::endl;
          printToStdOut();
          throw std::runtime_error(string("Shader::extractIncludeFilename() -") +
            string("ERROR: couldn't extract include filename!")); 
        } else if (shader_source_[cur_char] == '"') {
          str_end = cur_char - 1;  // Don't include the " character

          std::stringstream ss;
          for (int i = str_start; i <= str_end; i++) {
            ss << static_cast<char>(shader_source_[i]);
          }
          return ss.str();
        }
      }
      cur_char++;
    } while (cur_char < str_length);

    // We didn't find anything
    throw std::runtime_error(string("Shader::extractIncludeFilename() - ERROR") +
      string(" Check couldn't extract include filename!")); 
  }

  GLchar* Shader::insertIncludeSource(GLchar* source, GLchar* inc, 
    int inc_pos) {
    int source_length = static_cast<int>(strlen(source));
    int inc_length = static_cast<int>(strlen(inc));
    if (inc_pos > source_length) {
      throw std::runtime_error(string("Shader::insertIncludeSource() - ERROR") +
      string(" inc_pos > source_length!")); 
    }

    std::stringstream ss;
    // Insert source code up to inc:
    ss.write(static_cast<char*>(source), inc_pos);

    // Insert include source
    ss.write(static_cast<char*>(inc), inc_length);

    // Now find the end of the line after inc_pos
    int end_line = inc_pos;
    while (end_line < source_length) {
      if (source[end_line] == '\n') {
        end_line++;
        break;
      } else {
        end_line++;
      }
    }

    // Now write the rest of the source
    ss.write(static_cast<char*>(&source[end_line]), source_length - end_line);
    int new_length = static_cast<int>(ss.tellp()) + 1;  // include '\0' char

    // Now free the existing source memory, free the existing inc memory and
    // copy over the stringstream source into a new array
    free(source);
    ss.seekg(0);
    source = new GLchar[new_length];
    ss.read(source, new_length);
    source[new_length-1] = '\0';

    return source;
  }


}  // namespace misc
