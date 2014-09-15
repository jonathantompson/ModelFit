#version 150
#include "shaders/common_include.frag"

in vec3 f_color;
in vec3 f_normal_view;
in vec3 f_position_view;

uniform DirectionalLight light;
uniform float mat_specular_intensity;
uniform float mat_specular_power;

out vec4 color;

void main(void) {
  vec4 diffuse_color  = vec4(0, 0, 0, 0);
  vec4 specular_color = vec4(0, 0, 0, 0);

  // Calculate ambient component
  vec4 ambient_color = vec4(light.color, 1.0f) * light.ambient_intensity;

  // Calculate diffuse component
  vec3 normal = normalize(f_normal_view);
  float diffuse_factor = max(dot(normal, -light.direction_view), 0.0);
  diffuse_color = (vec4(light.color, 1.0f) * light.diffuse_intensity * 
	               diffuse_factor);

  // Calculate specular component
  vec3 vertex_to_eye = normalize(-f_position_view);
  vec3 light_reflect = normalize(reflect(light.direction_view, normal));
  float specular_factor = dot(vertex_to_eye, light_reflect);
  if (specular_factor > 0) {
    specular_factor = pow(specular_factor, mat_specular_power);
    specular_color = (vec4(light.color, 1.0f) * mat_specular_intensity * 
	                  specular_factor);
  }   

  color = vec4(f_color, 1.0) * (ambient_color + diffuse_color + specular_color);
}
