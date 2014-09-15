#version 150

in vec3 f_position_view;

out float depth;

void main(void) {
  depth = abs(f_position_view.z);
}
