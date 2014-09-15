#version 150

in vec2 f_texture;

out vec4 color;

uniform sampler2D f_texture_sampler;
uniform float f_depth_min;
uniform float f_depth_max;

void main(){
  float depth = texture(f_texture_sampler, f_texture).x;
  vec4 rgb_color = vec4(texture(f_texture_sampler, f_texture).gba, 1.0);
  depth = (depth - f_depth_min) / (f_depth_max - f_depth_min);
  color = rgb_color * vec4(depth, depth, depth, 1.0);
}