#version 150

uniform vec3 point_color;

out vec4 frag_color;

void main(void) {
  frag_color = vec4(point_color, 1.0);
}
