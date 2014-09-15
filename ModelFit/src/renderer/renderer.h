//
//  renderer.h
//
//  Created by Jonathan Tompson on 5/29/12.
//

#pragma once

#include "renderer/open_gl_common.h"
#include "data_str/vector.h"
#include "data_str/vector_managed.h"
#include "renderer/shader/shader_location_name_pair.h"

#ifndef BUFFER_OFFSET
	#define BUFFER_OFFSET(bytes) ((GLubyte*) NULL + bytes)
#endif

#define DIR_LIGHT_COLOR renderer::white
#define DIR_LIGHT_AMBIENT_INTENSITY 0.4f
#define DIR_LIGHT_DIFFUSE_INTENSITY 0.7f

namespace windowing { class Window; }
namespace kinect_interface { namespace hand_net { class HandModelCoeff; } }

namespace renderer {

  class Camera;
  class Shader;
  class ShaderProgram;
  class Geometry;
  class GeometryColoredMesh;
  class GeometryColoredBonedMesh;
  class GeometryTexturedMesh;
  class GeometryTexturedBonedMesh;
  class GeometryVertices;
  class GeometryPoints;
  class GeometryColoredPoints;
  class GeometryColoredLines;
  class Texture;
  class TextureRenderable;
  class LightDir;
  class GeometryManager;
  struct LightDirHandles;

  class Renderer {
  public:
    friend kinect_interface::hand_net::HandModelCoeff;  // TEMP CODE!
    // Constructor / Destructor
    Renderer();  // NOT THREAD SAFE!  Only call it once, from one thread!
    ~Renderer();

    // Top level functions
    void init(jtil::math::FloatQuat& eye_rot, jtil::math::Float3& eye_pos, int screen_width, 
      int screen_height, float znear, float zfar, float field_of_view);
    void renderFrame(float dt);
    void resize(uint32_t width, uint32_t height);
    static Renderer* g_renderer() { return g_renderer_; }
    
    // getter methods
    inline Camera* camera() { return camera_; }
    inline LightDir* light_dir() { return light_dir_; }
    inline GeometryVertices* quad() { return quad_; }
    inline Shader* v_shader_fullscreen_quad() { return v_shader_fullscreen_quad_; }

    // These are utility functions to bind certain data values to handles in
    // shaders
    void bindFloat4x4(const jtil::math::Float4x4* mat, const GLint h_mat); 
    void bindFloat2x4(const float mat[2][4], const GLint h_mat);
    void bindFloat4x4Array(const float* mat_array, uint32_t count, const GLint h_mat); 
    void bindFloat1(const float val, const GLint h_vec);
    void bindFloat2(const jtil::math::Float2* vec, const GLint h_vec);
    void bindFloat3(const jtil::math::Float3* vec, const GLint h_vec);
    void bindFloat4(const jtil::math::Float4* vec, const GLint h_vec);
    // This will bind a vertex attribute (in a VAO) with a specified user ID
    static void setVertexAttribPointer(const int id, int size, 
      int type, bool normalized, int stride, const void* pointer);

    // Some common vertex shader variable location pairs, that many shaders 
    // will use.  It is better to standardize vertex input locations so that 
    // VAOs can be used accross different shaders.
    static const ShaderLocationNamePair pos;
    static const ShaderLocationNamePair norm;
    static const ShaderLocationNamePair col;
    static const ShaderLocationNamePair tex;
    static const ShaderLocationNamePair bone_ids_03;
    //static const ShaderLocationNamePair bone_ids_47;
    static const ShaderLocationNamePair bone_weights_03;
    //static const ShaderLocationNamePair bone_weights_47;

    bool wireframe;
    bool render_bounding_spheres;

    void renderFullscreenQuad(TextureRenderable* tex, GLint frame_buffer_dst, 
      uint32_t texture_index);
    // Specify a framebuffer destination
    void renderFullscreenQuad(Texture* tex, GLint frame_buffer_dst);
    // Just render to the current framebuffer and viewport:
    void renderFullscreenQuad(Texture* tex, const bool clear = true);
    void renderPointCloud(GeometryPoints* points, jtil::math::Float4x4* mat_world,
      const jtil::math::Float3* color, const float point_size_constant);
    void renderColoredPointCloud(GeometryColoredPoints* points,
      jtil::math::Float4x4* mat_world, const float point_size_constant);
    void renderColoredLines(GeometryColoredLines* points,
      jtil::math::Float4x4* mat_world, const float line_size_constant);
    void downsample2Texture(TextureRenderable* dst, TextureRenderable* src);
    void downsample2IntegTexture(TextureRenderable* dst, TextureRenderable* src);
    void downsample4Texture(TextureRenderable* dst, TextureRenderable* src);
    void downsample4IntegTexture(TextureRenderable* dst, TextureRenderable* src);
    void downsample5IntegTexture(TextureRenderable* dst, TextureRenderable* src);

    jtil::math::Float4 background_color;

  private:
    static Renderer* g_renderer_;
    int screen_width_, screen_height_;
    float field_of_view_;

    Camera* camera_;
    uint64_t frame_counter_;
    
    // Test texture
    Texture* test_texture_;

    // The only light source supported so far: Directional Light
    LightDir* light_dir_;
    
    // Colored Mesh shader
    Shader* v_shader_cmesh_dlight_;
    Shader* f_shader_cmesh_dlight_;
    ShaderProgram* sp_cmesh_dlight_;
    LightDirHandles* h_sp_cmesh_dlight_;
    GLint h_PVW_mat_cmesh_dlight_;
    GLint h_VW_mat_cmesh_dlight_;
    GLint h_Normal_mat_cmesh_dlight_;
    GLint h_mat_specular_intensity_cmesh_dlight_;
    GLint h_mat_specular_power_cmesh_dlight_;

    // Colored and boned Mesh shader
    Shader* v_shader_cmesh_bone_dlight_;
    Shader* f_shader_cmesh_bone_dlight_;
    ShaderProgram* sp_cmesh_bone_dlight_;
    LightDirHandles* h_sp_cmesh_bone_dlight_;
    GLint h_PVW_mat_cmesh_bone_dlight_;
    GLint h_VW_mat_cmesh_bone_dlight_;
    GLint h_bone_trans_cmesh_bone_dlight_[MAX_BONE_COUNT];
    GLint h_Normal_mat_cmesh_bone_dlight_;
    GLint h_mat_specular_intensity_cmesh_bone_dlight_;
    GLint h_mat_specular_power_cmesh_bone_dlight_;

    // Textured Mesh shader
    Shader* v_shader_tmesh_dlight_;
    Shader* f_shader_tmesh_dlight_;
    ShaderProgram* sp_tmesh_dlight_;
    LightDirHandles* h_sp_tmesh_dlight_;
    GLint h_PVW_mat_tmesh_dlight_;
    GLint h_VW_mat_tmesh_dlight_;
    GLint h_Normal_mat_tmesh_dlight_;
    GLint h_mat_specular_intensity_tmesh_dlight_;
    GLint h_mat_specular_power_tmesh_dlight_;
    GLint h_tmesh_dlight_texture_sampler_;

    // Textured and boned Mesh shader
    Shader* v_shader_tmesh_bone_dlight_;
    Shader* f_shader_tmesh_bone_dlight_;
    ShaderProgram* sp_tmesh_bone_dlight_;
    LightDirHandles* h_sp_tmesh_bone_dlight_;
    GLint h_PVW_mat_tmesh_bone_dlight_;
    GLint h_VW_mat_tmesh_bone_dlight_;
    GLint h_bone_trans_tmesh_bone_dlight_[MAX_BONE_COUNT];
    GLint h_Normal_mat_tmesh_bone_dlight_;
    GLint h_mat_specular_intensity_tmesh_bone_dlight_;
    GLint h_mat_specular_power_tmesh_bone_dlight_;
    GLint h_tmesh_bone_dlight_texture_sampler_;

    // Fullscreen Quad shader
    GeometryVertices* quad_;
    Shader* v_shader_fullscreen_quad_;
    Shader* f_shader_fullscreen_quad_;
    ShaderProgram* sp_fullscreen_quad_;
    GLint h_fullscreen_quad_texture_sampler_;

    // Downsample by 2 x 2
    Shader* f_shader_downsample2_;
    ShaderProgram* sp_downsample2_;
    GLint h_downsample2_texture_sampler_;
    GLint h_downsample2_texel_size_;

    // Downsample and integrate by 2 x 2
    Shader* f_shader_downsample2_integ_;
    ShaderProgram* sp_downsample2_integ_;
    GLint h_downsample2_integ_texture_sampler_;
    GLint h_downsample2_integ_texel_size_;

    // Downsample by 4 x 4
    Shader* f_shader_downsample4_;
    ShaderProgram* sp_downsample4_;
    GLint h_downsample4_texture_sampler_;
    GLint h_downsample4_texel_size_;

    // Downsample and integrate by 4 x 4
    Shader* f_shader_downsample4_integ_;
    ShaderProgram* sp_downsample4_integ_;
    GLint h_downsample4_integ_texture_sampler_;
    GLint h_downsample4_integ_texel_size_;

    // Downsample and integrate by 5 x 5
    Shader* f_shader_downsample5_integ_;
    ShaderProgram* sp_downsample5_integ_;
    GLint h_downsample5_integ_texture_sampler_;
    GLint h_downsample5_integ_texel_size_;
    
    // Point cloud rendering
    Shader* v_shader_points_;
    Shader* f_shader_points_;
    ShaderProgram* sp_points_;
    GLint h_VW_mat_points_;
    GLint h_PVW_mat_points_;
    GLint h_point_color_points_;
    GLint h_point_size_constant_;
    
    // Colored Point cloud rendering
    Shader* v_shader_cpoints_;
    Shader* f_shader_cpoints_;
    ShaderProgram* sp_cpoints_;
    GLint h_VW_mat_cpoints_;
    GLint h_PVW_mat_cpoints_;
    GLint h_cpoint_size_constant_;

    // Colored Line rendering
    Shader* v_shader_clines_;
    Shader* f_shader_clines_;
    ShaderProgram* sp_clines_;
    //GLint h_VW_mat_clines_;
    GLint h_PVW_mat_clines_;
    //GLint h_cline_size_constant_;
    
    // Temporary variables
    jtil::math::Float4x4 Normal_mat_;
    jtil::math::Float4x4 VW_mat_;
    jtil::math::Float4x4 PVW_mat_;
    
    jtil::data_str::Vector<float> matrix_data_;  // concatenated matrix data

    void updateCameraFOVScreenSize();
    void updateMatrices();

    void renderColoredMesh(GeometryColoredMesh* geom);
    void renderColoredBonedMesh(GeometryColoredBonedMesh* geom);
    void renderTexturedMesh(GeometryTexturedMesh* geom);
    void renderTexturedBonedMesh(GeometryTexturedBonedMesh* geom);

    // Non-copyable, non-assignable.
    Renderer(Renderer&);
    Renderer& operator=(const Renderer&);
  };
};  // renderer namespace
