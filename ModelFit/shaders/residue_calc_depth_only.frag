#version 150

in vec2 f_texture;

out vec4 color;

uniform sampler2D kinect_depth;
uniform sampler2D synth_depth;

const float epsilon = 0.00001;
uniform float max_depth;

void main(){
  float depth_kinect = texture(kinect_depth, f_texture).x;
  float depth_synth = texture(synth_depth, f_texture).x;
  float depth_integ = min(abs(depth_kinect - depth_synth), max_depth);
  // float depth_integ = abs(depth_kinect - depth_synth) + 0.00000001 * max_depth;
  // depth_integ = depth_integ * depth_integ;  

  color = vec4(depth_integ, 0, 0, 0);   
}