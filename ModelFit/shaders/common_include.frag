// *********************
// common_include.frag
// *********************

struct DirectionalLight {
    vec3 color;
    float ambient_intensity;
    float diffuse_intensity;
    vec3 direction_view;
}; 

const int MAX_BONE_COUNT = 32;