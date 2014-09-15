#version 150

in vec2 f_texture;

out vec4 color;

uniform sampler2D f_texture_sampler;
uniform vec2 texel_size;

// These offsets were tested and they are correct:
// (I rendered a 16x image with 1 pixel border and then downsampled it)
const vec2 s1 = vec2(-1.5, -1.5);
const vec2 s2 = vec2(-1.5, -0.5);
const vec2 s3 = vec2(-1.5,  0.5);
const vec2 s4 = vec2(-1.5,  1.5);

const vec2 s5 = vec2(-0.5, -1.5);
const vec2 s6 = vec2(-0.5, -0.5);
const vec2 s7 = vec2(-0.5,  0.5);
const vec2 s8 = vec2(-0.5,  1.5);

const vec2 s9 =  vec2( 0.5, -1.5);
const vec2 s10 = vec2( 0.5, -0.5);
const vec2 s11 = vec2( 0.5,  0.5);
const vec2 s12 = vec2( 0.5,  1.5);

const vec2 s13 = vec2( 1.5, -1.5);
const vec2 s14 = vec2( 1.5, -0.5);
const vec2 s15 = vec2( 1.5,  0.5);
const vec2 s16 = vec2( 1.5,  1.5);

void main(){
  color = (texture(f_texture_sampler, f_texture + s1 * texel_size) +
           texture(f_texture_sampler, f_texture + s2 * texel_size) +
		   texture(f_texture_sampler, f_texture + s3 * texel_size) +
		   texture(f_texture_sampler, f_texture + s4 * texel_size) +
		   texture(f_texture_sampler, f_texture + s5 * texel_size) +
           texture(f_texture_sampler, f_texture + s6 * texel_size) +
		   texture(f_texture_sampler, f_texture + s7 * texel_size) +
		   texture(f_texture_sampler, f_texture + s8 * texel_size) +
		   texture(f_texture_sampler, f_texture + s9 * texel_size) +
           texture(f_texture_sampler, f_texture + s10 * texel_size) +
		   texture(f_texture_sampler, f_texture + s11 * texel_size) +
		   texture(f_texture_sampler, f_texture + s12 * texel_size) +
		   texture(f_texture_sampler, f_texture + s13 * texel_size) +
           texture(f_texture_sampler, f_texture + s14 * texel_size) +
		   texture(f_texture_sampler, f_texture + s15 * texel_size) +
		   texture(f_texture_sampler, f_texture + s16 * texel_size));
}