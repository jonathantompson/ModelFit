//
//  opengl.h
//
//  Created by Jonathan Tompson on 5/29/12.
//

#pragma once

#ifndef GLEW_STATIC
  #define GLEW_STATIC
#endif
#include <glew/glew.h>

#if defined(_DEBUG) || defined(DEBUG)
  #define ERROR_CHECK renderer::CheckOpenGLError()
#else
  #define ERROR_CHECK
#endif

#define LINEAR_BLEND_SKINNING  // If Dual Quaternion blend skinning is slow!
#define MAX_VERTEX_BONE_COUNT 4
//#define MAX_VERTEX_BONE_COUNT 8
#define MAX_BONE_COUNT 32

// Shaders and VBOs need to agree on a position for the various common
// attribute locations.
#define VERTEX_POS_LOC 0
#define VERTEX_COL_LOC 1
#define VERTEX_NOR_LOC 2
#define VERTEX_TEX_LOC 3
#define VERTEX_BONE_IDS_03_LOC 4
#define VERTEX_BONE_WEIGHTS_03_LOC 5

//#define VERTEX_BONE_IDS_03_LOC 4
//#define VERTEX_BONE_IDS_47_LOC 5
//#define VERTEX_BONE_WEIGHTS_03_LOC 7
//#define VERTEX_BONE_WEIGHTS_47_LOC 8

namespace renderer {
  void CheckOpenGLError();
}  // namespace renderer
