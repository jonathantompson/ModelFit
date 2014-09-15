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
uniform mat2x4 bone_trans[MAX_BONE_COUNT];  // Dual quaternion skinning

out vec3 f_color;
out vec3 f_normal_view;
out vec3 f_position_view;

void main(void) {
  // Dual quaternion blend skinning
  mat2x4 dq0 = bone_trans[v_bone_ids_03[0]];
  mat2x4 dq1 = bone_trans[v_bone_ids_03[1]];
  mat2x4 dq2 = bone_trans[v_bone_ids_03[2]];
  mat2x4 dq3 = bone_trans[v_bone_ids_03[3]];

  //mat2x4 dq4 = bone_trans[v_bone_ids_47[0]];
  //mat2x4 dq5 = bone_trans[v_bone_ids_47[1]];
  //mat2x4 dq6 = bone_trans[v_bone_ids_47[2]];
  //mat2x4 dq7 = bone_trans[v_bone_ids_47[3]];

  vec4 bone_weights_03 = v_bone_weights_03;
  vec4 bone_weights_47 = v_bone_weights_47;

  // Antipodality checks:
  if (dot(dq0[0], dq1[0]) < 0.0) bone_weights_03.y *= -1.0;
  if (dot(dq0[0], dq2[0]) < 0.0) bone_weights_03.z *= -1.0;
  if (dot(dq0[0], dq3[0]) < 0.0) bone_weights_03.w *= -1.0;

  //if (dot(dq0[0], dq4[0]) < 0.0) bone_weights_47.y *= -1.0;
  //if (dot(dq0[0], dq5[0]) < 0.0) bone_weights_47.z *= -1.0;
  //if (dot(dq0[0], dq6[0]) < 0.0) bone_weights_47.w *= -1.0;
  //if (dot(dq0[0], dq7[0]) < 0.0) bone_weights_47.y *= -1.0;

  mat2x4 blendDQ =
         bone_weights_03.x * dq0 +
         bone_weights_03.y * dq1 +
         bone_weights_03.z * dq2 +
         bone_weights_03.w * dq3 /*+
		 bone_weights_47.x * dq4 +
         bone_weights_47.y * dq5 +
         bone_weights_47.z * dq6 +
         bone_weights_47.w * dq7*/;

  // Fast dual quaternion blend skinning:
  float length = sqrt(blendDQ[0].w * blendDQ[0].w + blendDQ[0].x * blendDQ[0].x + blendDQ[0].y * blendDQ[0].y + blendDQ[0].z * blendDQ[0].z);
  blendDQ = blendDQ / length;
  vec3 vPos = v_position + 2.0 * cross(blendDQ[0].xyz, cross(blendDQ[0].xyz, v_position) + blendDQ[0].w * v_position);
  vec3 translation = 2.0 * (blendDQ[0].w * blendDQ[1].xyz - blendDQ[1].w * blendDQ[0].xyz + cross(blendDQ[0].xyz, blendDQ[1].xyz));
  vPos += translation;
  vec3 vNor = v_normal + 2.0 * cross(blendDQ[0].xyz, cross(blendDQ[0].xyz, v_normal) + blendDQ[0].w * v_normal);

  // Multiply the mvp matrix by the vertex to obtain our final vertex position
  gl_Position = PVW_mat * vec4(vPos, 1.0), 1.0;

  f_color = v_color;
  f_normal_view = (Normal_mat * vec4(vNor, 0.0)).xyz;
  f_position_view = (VW_mat * vec4(vPos, 1.0)).xyz;
}
