#include <string>
#include "renderer/texture/texture_utils.h"
#include "renderer/open_gl_common.h"

namespace renderer {
  uint32_t ElementSizeOfGLType(GLint gl_type) {
    // This code is borrowed and adapted from:
    // https://github.com/sgothel/jogl/blob/master/src/jogl/classes/com/jogamp/opengl/util/GLBuffers.java
    switch (gl_type) {
    case GL_BYTE:
    case GL_UNSIGNED_BYTE:
    case GL_UNSIGNED_BYTE_3_3_2:
    case GL_UNSIGNED_BYTE_2_3_3_REV:
      GLbyte dummy_byte;
      static_cast<void>(dummy_byte);
      return static_cast<uint32_t>(sizeof(dummy_byte));

    case GL_SHORT:
    case GL_UNSIGNED_SHORT:
    case GL_UNSIGNED_SHORT_5_6_5:
    case GL_UNSIGNED_SHORT_5_6_5_REV:
    case GL_UNSIGNED_SHORT_4_4_4_4:
    case GL_UNSIGNED_SHORT_4_4_4_4_REV:
    case GL_UNSIGNED_SHORT_5_5_5_1:
    case GL_UNSIGNED_SHORT_1_5_5_5_REV:
    case GL_HALF_FLOAT:
      GLshort dummy_short;
      static_cast<void>(dummy_short);
      return static_cast<uint32_t>(sizeof(dummy_short));

    case GL_FIXED:
    case GL_INT:
    case GL_UNSIGNED_INT:
    case GL_UNSIGNED_INT_8_8_8_8:
    case GL_UNSIGNED_INT_8_8_8_8_REV:
    case GL_UNSIGNED_INT_10_10_10_2:
    case GL_UNSIGNED_INT_2_10_10_10_REV:                
    case GL_UNSIGNED_INT_24_8:
    case GL_UNSIGNED_INT_10F_11F_11F_REV:
    case GL_UNSIGNED_INT_5_9_9_9_REV:
    case GL_HILO16_NV:
    case GL_SIGNED_HILO16_NV:
      GLint dummy_int;
      static_cast<void>(dummy_int);
      return static_cast<uint32_t>(sizeof(dummy_int));

    case GL_FLOAT_32_UNSIGNED_INT_24_8_REV:
      GLulong dummy_long;
      static_cast<void>(dummy_long);
      return static_cast<uint32_t>(sizeof(dummy_long));

    case GL_FLOAT:
      GLfloat dummy_float;
      static_cast<void>(dummy_float);
      return static_cast<uint32_t>(sizeof(dummy_float));

    case GL_DOUBLE:
      GLdouble dummy_double;
      static_cast<void>(dummy_double);
      return static_cast<uint32_t>(sizeof(dummy_double));
    }
    throw std::runtime_error("SizeOfGLType() - ERROR: Unrecognized type!");
  }

  uint32_t NumElementsOfGLFormat(GLint gl_format) {
    switch (gl_format) /* 26 */ {
    case GL_COLOR_INDEX:
    case GL_STENCIL_INDEX:
    case GL_DEPTH_COMPONENT:
    case GL_DEPTH_STENCIL:
    case GL_RED:
    case GL_RED_INTEGER:
    case GL_GREEN:
    case GL_GREEN_INTEGER:
    case GL_BLUE:
    case GL_BLUE_INTEGER:
    case GL_ALPHA:
    case GL_LUMINANCE:
      return 1;
    case GL_LUMINANCE_ALPHA:
    case GL_RG:
    case GL_RG_INTEGER:
    case GL_HILO_NV:
    case GL_SIGNED_HILO_NV:
      return 2;
    case GL_RGB:
    case GL_RGB_INTEGER:
    case GL_BGR:
    case GL_BGR_INTEGER: 
      return 3;
    case GL_RGBA:
    case GL_RGBA_INTEGER:
    case GL_BGRA:
    case GL_BGRA_INTEGER:
    case GL_ABGR_EXT:
      return 4;
    }
    throw std::runtime_error("SizeOfGLType() - ERROR: Unrecognized format!");
  }

}  // namespace renderer
