#version 150

in vec2 f_texture;

out vec4 color;

uniform sampler2D kinect_depth;
uniform sampler2D synth_depth;

const float epsilon = 0.00001;
const int sample_rad = 5;
const int sample_rad_sq = sample_rad * sample_rad;
uniform float max_depth;

void main(){
  float depth_kinect = texture(kinect_depth, f_texture).x;
  float depth_synth = texture(synth_depth, f_texture).x;
  float depth_integ = min(abs(depth_kinect - depth_synth), max_depth);
  // depth_integ = depth_integ * depth_integ;

  // NO LONGER USING UNION OR INTERSECTIONS
  float d_intersection = 0;
  if (depth_kinect > epsilon && depth_synth > epsilon) {
    d_intersection = 1;
  }
  float d_union = 0; 
  if (depth_kinect > epsilon || depth_synth > epsilon) {
    d_union = 1;
  }
  color = vec4(depth_integ, d_union, d_intersection, 0.0);    
}