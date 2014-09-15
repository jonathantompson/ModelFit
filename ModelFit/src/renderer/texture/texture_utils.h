//
//  texture_utils.h
//
//  Created by Jonathan Tompson on 10/7/12.
//
//  Just a few simple utility functions

#pragma once

#include "renderer/open_gl_common.h"
#include "math/math_types.h"

namespace renderer {
  uint32_t ElementSizeOfGLType(GLint gl_type);
  uint32_t NumElementsOfGLFormat(GLint gl_format);
};  // namespace renderer
