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

  float d_intersection = 0;
  if (depth_kinect > epsilon && depth_synth > epsilon) {
    d_intersection = 1;
  }

  float d_union = 0; 
  if (depth_kinect > epsilon || depth_synth > epsilon) {
    d_union = 1;
  }

  float depth_difference = min(abs(depth_kinect - depth_synth), max_depth);

  color = vec4(depth_difference*depth_difference, d_union, d_intersection, 0.0);       
  //color = vec4(depth_difference*depth_difference, 1.0, 1.0, 0.0);       
}