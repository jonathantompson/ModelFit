//
//  texture_renderable.h
//
//  Created by Jonathan Tompson on 6/8/12.
//
//  Note: the framebuffer may have many textures (multiple render targets),
//  num_textures controls the number of renderable textures.

#pragma once

#include <mutex>
#include <string>
#include "renderer/open_gl_common.h"
#include "renderer/gl_state.h"

namespace renderer {

  class TextureRenderable {
  public:
    // Load texture from file:
    TextureRenderable(GLint internal_format, int w, int h, GLenum format, 
      GLenum type, uint32_t num_textures, bool create_depth_texture);
    ~TextureRenderable();

    void begin();  // begin rendering
    void end();  // end rendering

    // bind() - typical usage for single texture render target: 
    //          bind(0, GL_TEXTURE0, tex_sampler_id)
    void bind(uint32_t texture_index, GLenum target_id, 
      GLint h_texture_sampler);

    template <class T>
    void getTexture0Data(T* data);
    void saveTexture0Data(std::string filename);

    GLuint texture(uint32_t i);
    inline int w() { return w_; }
    inline int h() { return h_; }
    inline GLenum format() { return format_; }

    bool compareFormat(TextureRenderable* texture);  // true -> formats match

  private:
    GLuint* textures_;
    GLuint depth_texture_;
    GLuint fbo_;
    GLint internal_format_; 
    int w_;
    int h_;
    GLenum format_; 
    GLenum type_; 
    uint32_t num_textures_;
    GLenum* draw_buffers_;

    // Non-copyable, non-assignable.
    TextureRenderable(TextureRenderable&);
    TextureRenderable& operator=(const TextureRenderable&);
  };

  template <class T>
  void TextureRenderable::getTexture0Data(T* data) {
    GLState::glsBindFramebuffer(GL_READ_FRAMEBUFFER, fbo_);
    // From http://www.khronos.org/opengles/sdk/docs/man/xhtml/glReadPixels.xml
    // If the currently bound framebuffer is not the default framebuffer 
    // object, color components are read from the color image attached to the 
    // GL_COLOR_ATTACHMENT0 attachment point.
    //GLState::glsReadBuffer(GL_COLOR_ATTACHMENT0);
    glReadBuffer(GL_COLOR_ATTACHMENT0);
    glReadPixels(0, 0, w_, h_, format_, type_, data);
    //GLState::glsReadPixels(0, 0, w_, h_, format_, type_, data);
    GLState::glsBindFramebuffer(GL_READ_FRAMEBUFFER, 0);
  }

};  // namespace renderer
