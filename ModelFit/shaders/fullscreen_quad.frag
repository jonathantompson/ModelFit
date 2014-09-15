#version 150

in vec2 f_texture;

out vec4 color;

uniform sampler2D f_texture_sampler;

void main(){
  color = vec4(texture(f_texture_sampler, f_texture).xyz, 1.0);
}