#include <string>
#include <sstream>
#include <fstream>
#include "renderer/texture/texture_renderable.h"
#include "string_util/string_util.h"
#include "renderer/texture/texture_utils.h"
#include "renderer/gl_state.h"

using std::wstring;
using std::runtime_error;

namespace renderer {

  // Load a texture from disk
  TextureRenderable::TextureRenderable(GLint internal_format, int w, int h, 
    GLenum format, GLenum type, uint32_t num_textures, bool create_depth_texture) {
    internal_format_ = internal_format; 
    w_ = w;
    h_ = h;
    format_ = format;
    type_ = type;
    num_textures_ = num_textures;

#ifdef _DEBUG
    if (num_textures == 0) {
      throw std::runtime_error("TextureRenderable::TextureRenderable()"
        " - ERROR: num_textures should not be zero");
    }
#endif

    // Create the FBO
    GLState::glsGenFramebuffers(1, &fbo_); 
    GLState::glsBindFramebuffer(GL_FRAMEBUFFER, fbo_);

    // Create the openGL texture IDs
    textures_ = new GLuint[num_textures_];
    GLState::glsGenTextures(num_textures, textures_);

    // Create the openGL textures
    for (uint32_t i = 0; i < num_textures_; i ++) {
      GLState::glsBindTexture(GL_TEXTURE_2D, 0);  // Force a binding
      GLState::glsBindTexture(GL_TEXTURE_2D, textures_[i]);
      GLState::glsTexImage2D(GL_TEXTURE_2D, 0, internal_format_ , w_, h_, 0, format_, 
        type_, NULL);
      GLState::glsFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0 + i, 
        GL_TEXTURE_2D, textures_[i], 0);

      // No filtering for the render textures
      GLState::glsTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
      GLState::glsTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST); 
      GLState::glsTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
      GLState::glsTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    }

    if (!create_depth_texture) {
      depth_texture_ = 0;
    } else {
    //  GLState::glsGenRenderbuffers(1, &depth_texture_);
    //  GLState::glsBindRenderbuffer(GL_RENDERBUFFER, depth_texture_);
    //  GLState::glsRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH32F_STENCIL8, w_, h_);
    //  GLState::glsFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, 
    //    GL_RENDERBUFFER, depth_texture_);
      GLState::glsGenTextures(1, &depth_texture_);
      GLState::glsBindTexture(GL_TEXTURE_2D, 0);  // Force a binding
      GLState::glsBindTexture(GL_TEXTURE_2D, depth_texture_);
      GLState::glsTexImage2D(GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT24, w_, h_, 
        0, GL_DEPTH_COMPONENT, GL_UNSIGNED_INT, NULL);
      GLState::glsTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, 
        GL_NEAREST);
      GLState::glsTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, 
        GL_NEAREST); 
      GLState::glsTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, 
        GL_CLAMP_TO_EDGE);
      GLState::glsTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, 
        GL_CLAMP_TO_EDGE);
      GLState::glsFramebufferTexture2D(GL_FRAMEBUFFER, 
        GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D, depth_texture_, 0);
    }

    draw_buffers_ = new GLenum[num_textures_];
    for (uint32_t i = 0; i < num_textures_; i ++) {
      draw_buffers_[i] = GL_COLOR_ATTACHMENT0 + i;
    }
    GLState::glsDrawBuffers(num_textures_, draw_buffers_);
    ERROR_CHECK;

    GLState::glsCheckFramebufferStatus(GL_FRAMEBUFFER);

  }

  TextureRenderable::~TextureRenderable() {
    if (fbo_ != 0) {
        GLState::glsDeleteFramebuffers(1, &fbo_);
    }
    for (uint32_t i = 0; i < num_textures_; i ++) {
      GLState::glsDeleteTextures(1, &textures_[i]);
    }
    delete[] textures_;
    if (depth_texture_ != 0) {
      GLState::glsDeleteTextures(1, &depth_texture_);
    }
    if (draw_buffers_) {
      delete[] draw_buffers_;
      draw_buffers_ = NULL;
    }
  }

  GLuint TextureRenderable::texture(uint32_t i) {
#ifdef _DEBUG
    if (i > num_textures_) {
      throw std::runtime_error("TextureRenderable::texture() - i out of bounds");
    }
#endif
    return textures_[i];
  }

  void TextureRenderable::begin() {
    GLState::glsBindFramebuffer(GL_FRAMEBUFFER, fbo_);
    GLState::glsViewport(0, 0, w_, h_);
  }

  void TextureRenderable::end() {
    GLState::glsBindFramebuffer(GL_FRAMEBUFFER, 0);
  }

  // bind() - typical usage for single texture render target: 
  //          bind(0, GL_TEXTURE0, tex_sampler_id)
  void TextureRenderable::bind(uint32_t texture_index, GLenum target_id, 
    GLint h_texture_sampler) {
    // glBindFramebuffer(GL_READ_FRAMEBUFFER, fbo_);  
    // --> Necessary when blitting.  But we are not blitting.  See: 
    // http://www.opengl.org/wiki/GL_EXT_framebuffer_object
    // ERROR_CHECK;
    GLState::glsActiveTexture(target_id);
    GLState::glsBindTexture(GL_TEXTURE_2D, textures_[texture_index]);
    GLint uniform_val = target_id - GL_TEXTURE0;
    GLState::UniformFuncs::glsUniform1iv(h_texture_sampler, 1, &uniform_val);
  }

  bool TextureRenderable::compareFormat(TextureRenderable* texture) {
    return (this->format_ == texture->format_) &&
           (this->internal_format_ == texture->internal_format_) &&
           (this->w_ == texture->w_) &&
           (this->h_ == texture->h_) &&
           (this->type_ == texture->type_);
  }

  void TextureRenderable::saveTexture0Data(std::string filename) {
    uint32_t num_elements = NumElementsOfGLFormat(format_);
    uint32_t element_size = ElementSizeOfGLType(type_);
    uint32_t data_size = num_elements * element_size * w_ * h_;

    // Allocate some data and copy from OpenGL GPU memory
    
    char* data = new char[data_size];
    getTexture0Data<char>(data);

    // Save to disk
    std::ofstream file(filename.c_str(), std::ios::out | std::ios::binary);
    if (!file.is_open()) {
      throw std::runtime_error(std::string("saveTexture0Data()") + 
        std::string(": error opening file:") + filename);
    }
    file.write(reinterpret_cast<const char*>(data), data_size);
    file.flush();
    file.close();

    // Clean up
    delete[] data;
  }

}  // namespace renderer
