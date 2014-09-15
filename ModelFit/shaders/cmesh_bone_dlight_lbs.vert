#version 150
#include "shaders/common_include.frag"

// Based on the implementation found here:
// http://isg.cs.tcd.ie/projects/DualQuaternions/
// http://www.jarmilakavanova.cz/ladislav/papers/sdq-i3d07/sdq-i3d07.pdf

in vec3 v_position;
in vec3 v_color;
in vec3 v_normal;
in ivec4 v_bone_ids_03;
//in ivec4 v_bone_ids_47;
in vec4 v_bone_weights_03;
//in vec4 v_bone_weights_47;

uniform mat4 PVW_mat;
uniform mat4 VW_mat;
uniform mat4 Normal_mat;
uniform mat4x4 bone_trans[MAX_BONE_COUNT];  // Linear blend skinning

out vec3 f_color;
out vec3 f_normal_view;
out vec3 f_position_view;

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
  vec3 nor = (bone_transform * vec4(v_normal, 0.0)).xyz;

  // Multiply the mvp matrix by the vertex to obtain our final vertex position
  gl_Position = PVW_mat * pos, 1.0;

  f_color = v_color;
  f_normal_view = (Normal_mat * vec4(nor, 0.0)).xyz;
  f_position_view = (VW_mat * pos).xyz;
}
