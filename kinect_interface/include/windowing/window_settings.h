//
//  window_settings.h
//
//  Created by Jonathan Tompson on 09/25/12.
//

#pragma once

#include <mutex>
#include <string>
#include "math/math_types.h"  // for uint32_t

namespace jtil {
namespace windowing {
  
  struct WindowSettings {
    std::string title;
    int width;
    int height;
    bool fullscreen;
    bool double_buffering;
    int samples;

    // To be set by the window class (internal use only)
    int viewport_width;
    int viewport_height;
    
    // Back buffer format
    int num_depth_bits;
    int num_stencil_bits;
    int num_rgba_bits;
    
    // OpenGL version
    int gl_major_version;
    int gl_minor_version;
    bool gl_core_profile;
    
    WindowSettings& operator=(const WindowSettings &rhs);
  };

};  // namespace windowing
};  // namespace jtil
