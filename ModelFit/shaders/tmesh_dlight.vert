#version 150

in vec3 v_position;
in vec2 v_texture;
in vec3 v_normal;

uniform mat4 PVW_mat;
uniform mat4 VW_mat;
uniform mat4 Normal_mat;

out vec2 f_texture;
out vec3 f_normal_view;
out vec3 f_position_view;

void main(void) {
  // Multiply the mvp matrix by the vertex to obtain our final vertex position
  gl_Position = PVW_mat * vec4(v_position, 1.0);

  f_texture = v_texture;
  f_normal_view = (Normal_mat * vec4(v_normal, 0.0)).xyz;
  f_position_view = (VW_mat * vec4(v_position, 1.0)).xyz;
}
