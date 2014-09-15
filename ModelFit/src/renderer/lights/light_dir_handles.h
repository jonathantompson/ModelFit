//
//  light_dir_handles.h
//
//  Created by Jonathan Tompson on 9/18/12.
//

#pragma once

#include "renderer/open_gl_common.h"

namespace renderer {

  class ShaderProgram;
  class LightDir;
  class Renderer;

  struct LightDirHandles {
    GLint h_color;
    GLint h_ambient_intensity;
    GLint h_diffuse_intensity;
    GLint h_direction_view;
    void getHandles(ShaderProgram* sp);
    void setHandles(LightDir* light, Renderer* render);
  };

};  // renderer namespace
