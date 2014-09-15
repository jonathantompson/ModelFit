#version 150

in vec3 f_position_view;
in vec3 f_color;

out vec4 cdepth;

void main(void) {
  cdepth = vec4(abs(f_position_view.z), f_color[0], f_color[1], f_color[2]);

}
