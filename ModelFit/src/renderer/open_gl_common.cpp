#include <sstream>
#include <string>
#include "string_util/string_util.h"
#include "renderer/open_gl_common.h"

namespace renderer {
  void CheckOpenGLError() {
    GLenum err = glGetError();
    if (err != GL_NO_ERROR) {
      std::stringstream ss;
      ss << "Renderer::checkOpenGLError() - ERROR: '";
      ss << reinterpret_cast<const char *>(gluErrorString(err));
      ss << "'";
      throw std::runtime_error(ss.str());
    }
  }
  
};  // namespace renderer

