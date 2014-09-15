#version 150
#include "shaders/common_include.frag"

in vec3 v_position;
in vec3 v_color;
in ivec4 v_bone_ids_03;
//in ivec4 v_bone_ids_47;
in vec4 v_bone_weights_03;
//in vec4 v_bone_weights_47;

uniform mat4 PVW_mat;
uniform mat4 VW_mat;
uniform mat4x4 bone_trans[MAX_BONE_COUNT];  // Dual quaternion skinning

out vec3 f_position_view;
out vec3 f_color;

void main(void) {
  // Linear blend skinning:
  mat4 bone_transform = bone_trans[v_bone_ids_03[0]] * v_bone_weights_03[0];
  bone_transform     += bone_trans[v_bone_ids_03[1]] * v_bone_weights_03[1];
  bone_transform     += bone_trans[v_bone_ids_03[2]] * v_bone_weights_03[2];
  bone_transform     += bone_trans[v_bone_ids_03[3]] * v_bone_weights_03[3];

  //bone_transform     += bone_trans[v_bone_ids_47[0]] * v_bone_weights_47[0];
  //bone_transform     += bone_trans[v_bone_ids_47[1]] * v_bone_weights_47[1];
  //bone_transform     += bone_trans[v_bone_ids_47[2]] * v_bone_weights_47[2];
  //bone_transform     += bone_trans[v_bone_ids_47[3]] * v_bone_weights_47[3];

  vec4 pos = bone_transform * vec4(v_position, 1.0);

  // Multiply the mvp matrix by the vertex to obtain our final vertex position
  gl_Position = PVW_mat * pos;
  gl_Position.y = -1.0 * gl_Position.y;  // Flip the coords!

  f_color = v_color;
  f_position_view = (VW_mat * pos).xyz;
}
