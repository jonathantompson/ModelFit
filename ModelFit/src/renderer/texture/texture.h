//
//  texture.h
//
//  Created by Jonathan Tompson on 6/8/12.
//
//  The underlining loading of textures uses the FreeImage library.

#pragma once

#include <mutex>
#include <string>

namespace renderer {

  typedef enum {
    TEXTURE_CLAMP,
    TEXTURE_REPEAT
  } TEXTURE_WRAP_MODE;

  typedef enum {
    TEXTURE_LINEAR,
    TEXTURE_NEAREST
  } TEXTURE_FILTER_MODE;

  class Texture {
  public:
    static void initTextureSystem();
    static void shutdownTextureSystem();

    // Load texture from file:
    Texture(const std::string& filename, const TEXTURE_WRAP_MODE wrap, 
      bool origin_ul = false, const TEXTURE_FILTER_MODE filter = TEXTURE_LINEAR);
    // Create texture from internal data structure:
    Texture(const int format_internal, const int w, const int h, 
      const int format, const int type, const unsigned char *bits, 
      const TEXTURE_WRAP_MODE wrap, bool managed, 
      const TEXTURE_FILTER_MODE filter = TEXTURE_LINEAR);
    ~Texture();

    inline GLuint texture() { return texture_; }
    inline std::string& filename() { return filename_; }
    inline int w() { return w_; }
    inline int h() { return h_; }

    void reloadData(const unsigned char *bits);

    // Some wrappers to FreeImage
    static bool saveRGBToFile(const std::string& filename, const uint8_t* rgb,
      const uint32_t width, const uint32_t height, const bool save_flipped);
    static bool saveGreyscaleToFile(const std::string& filename, 
      const uint8_t* grey, const uint32_t width, const uint32_t height, 
      const bool save_flipped);
    static void loadImFromFile(const std::string& filename, uint8_t*& im, 
      uint32_t& width, uint32_t& height, uint32_t& num_channels);

    void bind(GLenum target_id,  // ie target_id = GL_TEXTURE0
      GLint texture_sampler_id);  // id in the in the shader program

    // Perform a deep copy and load the copy into OpenGL
    Texture* copy();

  private:
    std::string filename_;
    GLuint texture_;
    int format_internal_; 
    int w_;
    int h_;
    GLenum format_; 
    GLenum type_;
    unsigned char *bits_;
    TEXTURE_WRAP_MODE wrap_;
    TEXTURE_FILTER_MODE filter_;
    bool managed_;
    bool from_file_;
    // void* img_;  // Used to be GLFWimage, now using FreeImage library

    static std::mutex freeimage_init_lock_;
    static bool freeimage_init_;

    void generateTextureID();
    void loadTextureIntoOpenGL();
    void setTextureProperties();

    static bool saveImToFile(const std::string& filename, 
      const uint8_t* im, const uint32_t width, const uint32_t height, 
      const bool save_flipped, const uint32_t num_channels);

    // Non-copyable, non-assignable.
    Texture(Texture& other);
    Texture& operator=(const Texture&);
  };
};  // namespace renderer
