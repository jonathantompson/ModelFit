#version 150

in vec2 f_texture;

out vec4 color;

uniform sampler2D f_texture_sampler;
uniform vec2 texel_size;

// These offsets were tested and they are correct:
// (I rendered a 4x image with 1 pixel border and then downsampled it)
const vec2 s1 = vec2(-0.5, -0.5);
const vec2 s2 = vec2(-0.5,  0.5);
const vec2 s3 = vec2( 0.5, -0.5);
const vec2 s4 = vec2( 0.5,  0.5);

void main(){
  color = texture(f_texture_sampler, f_texture + s1 * texel_size) +
          texture(f_texture_sampler, f_texture + s2 * texel_size) +
		  texture(f_texture_sampler, f_texture + s3 * texel_size) +
		  texture(f_texture_sampler, f_texture + s4 * texel_size);
}