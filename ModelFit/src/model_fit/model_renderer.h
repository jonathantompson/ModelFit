//
//  model_renderer.h
//
//  Created by Jonathan Tompson on 10/02/12.
//
//  Renderers an array of pose_model model in OpenGL 3.2 using my simple 
//  forward-renderer class to create a depth class.
//
//  Additionally, it will render tiles of these so that we can aggregate some
//  of the residue overhead.
//

#pragma once

#include "renderer/open_gl_common.h"  // GLfloat
#include "math/math_types.h"
#include "data_str/pair.h"
#include "data_str/vector.h"

#define DEPTH_ONLY_RESIDUE_FUNC  // Faster but less accurate
#define EQUAL_CAMERA_IMPORTANCE  // Otherwise Residual = d / (i_cam + 1)
#ifdef DEPTH_ONLY_RESIDUE_FUNC
  #define RESIDUE_INT_FORMAT GL_R32F
  #define RESIDUE_FORMAT GL_RED
  #define RESIDUE_TYPE GL_FLOAT
  #define UNION 1000000.0f  // A made up scaling factor
#else
  #define RESIDUE_INT_FORMAT GL_RGB32F
  #define RESIDUE_FORMAT GL_RGB
  #define RESIDUE_TYPE GL_FLOAT
#endif
#define LINEAR_PENALTY  // otherwise quadratic

namespace renderer { class Camera; }
namespace renderer { class Geometry; }
namespace renderer { class Renderer; }
namespace renderer { class TextureRenderable; }
namespace renderer { class Texture; }
namespace renderer { class Shader; }
namespace renderer { class ShaderProgram; }
namespace renderer { class GeometryTexturedBonedMesh; }
namespace renderer { class GeometryColoredMesh; }
namespace renderer { class BoundingSphere; }
namespace renderer { class GeometryColoredBonedMesh; }
namespace jtil { namespace windowing { class Window; } }

namespace model_fit {
  class PoseModel;

  class ModelRenderer {
  public:
    // Constructor / Destructor
    ModelRenderer(const uint32_t num_cameras);
    ~ModelRenderer();

    // Call before rendering depth maps:
    void uploadDepth(const uint32_t i_camera, const int16_t* kinect_depth);  

    // Top level functions
    void drawDepthMap(const float* coeff, const uint32_t num_coeff_per_model, 
      PoseModel** models, const uint32_t num_models, const uint32_t i_camera,
      const bool color);

    // max_num_interpenetration_groups --> if the number of bounding sphere
    // groups is larger than you want to consider (ie, ignore all groups beyond n)
    void drawDepthMapTiled(jtil::data_str::Vector<float*>& coeff, 
      const uint32_t num_coeff_per_model, PoseModel** models, 
      const uint32_t num_models, const uint32_t i_camera,
      jtil::data_str::Vector<float>* interpenetration,
      const uint32_t max_num_interpenetration_groups);

    void visualizeDepthMap(jtil::windowing::Window* wnd,
      const uint32_t i_camera, const bool color = false);

    float calculateResidualDataTerm(const uint32_t i_camera);
    void calculateResidualDataTermTiled(jtil::data_str::Vector<float>& residues,
      const uint32_t i_camera);
    float calcInterpenetrationTerm(const uint32_t max_groups);
    void extractDepthMap(float* depth_vals);

    inline renderer::TextureRenderable* depth_texture() { return depth_texture_; }
    inline renderer::TextureRenderable* cdepth_texture() { return cdepth_texture_; }
    inline float** depth_tmp() { return depth_tmp_; }
    inline renderer::Camera* camera(const uint32_t id) { return cameras_[id]; }

    renderer::TextureRenderable* residue_texture_1() { return residue_texture_1_; }

  private:
    renderer::Renderer* g_renderer_;  // Not owned here
    uint32_t num_cameras_;
    renderer::Camera** cameras_;
    
    renderer::TextureRenderable* depth_texture_;  // 640 x 480
    renderer::TextureRenderable* depth_texture_tiled_;  // 5120 x 3840 (8x8)
    renderer::TextureRenderable* residue_texture_1_;   // 640 x 480
    renderer::TextureRenderable* residue_texture_2_;   // 320 x 240
    renderer::TextureRenderable* residue_texture_4_;   // 160 x 120
    renderer::TextureRenderable* residue_texture_16_;  // 40 x 30
    renderer::TextureRenderable* residue_texture_20_;  // 32 x 24
    renderer::TextureRenderable* residue_texture_32_;  // 20 x 15
    renderer::TextureRenderable* residue_texture_160_;  // 4 x 3
    renderer::TextureRenderable* residue_texture_x2_;   // 1280 x 960
    renderer::TextureRenderable* residue_texture_x8_;   // 5120 x 3840
    renderer::Texture** kinect_depth_textures_;
    renderer::Texture** kinect_depth_textures_tiled_;
    renderer::TextureRenderable* cdepth_texture_;  // depth and color

    float** depth_tmp_;  // num_cameras * (DEPTH_IMAGE_DIM * NTILES)

    // Shader for rendering depth
    renderer::Shader* v_shader_depth_;
    renderer::Shader* f_shader_depth_;
    renderer::ShaderProgram* sp_depth_;
    GLint h_PVW_mat_sp_depth_;
    GLint h_VW_mat_sp_depth_;

    // Shader for rendering colored depth
    renderer::Shader* v_shader_cdepth_;
    renderer::Shader* f_shader_cdepth_;
    renderer::ShaderProgram* sp_cdepth_;
    GLint h_PVW_mat_sp_cdepth_;
    GLint h_VW_mat_sp_cdepth_;

    // Shader for rendering depth from skinned mesh
    renderer::Shader* v_shader_depth_skinned_;
    renderer::Shader* f_shader_depth_skinned_;
    renderer::ShaderProgram* sp_depth_skinned_;
    GLint h_PVW_mat_sp_depth_skinned_;
    GLint h_VW_mat_sp_depth_skinned_;
    GLint h_bone_trans_sp_depth_skinned_[MAX_BONE_COUNT];

    // Shader for rendering depth and face color from skinned mesh
    renderer::Shader* v_shader_cdepth_skinned_;
    renderer::Shader* f_shader_cdepth_skinned_;
    renderer::ShaderProgram* sp_cdepth_skinned_;
    GLint h_PVW_mat_sp_cdepth_skinned_;
    GLint h_VW_mat_sp_cdepth_skinned_;
    GLint h_bone_trans_sp_cdepth_skinned_[MAX_BONE_COUNT];

    // Shader to visualize depth
    renderer::Shader* v_shader_visualize_depth_;
    renderer::Shader* f_shader_visualize_depth_;
    renderer::ShaderProgram* sp_visualize_depth_;
    GLint h_visualize_depth_texture_sampler_;
    GLint h_f_depth_min_visualize_depth_;
    GLint h_f_depth_max_visualize_depth_;

    // Shader to visualize colored depth
    renderer::Shader* v_shader_visualize_cdepth_;
    renderer::Shader* f_shader_visualize_cdepth_;
    renderer::ShaderProgram* sp_visualize_cdepth_;
    GLint h_visualize_cdepth_texture_sampler_;
    GLint h_f_depth_min_visualize_cdepth_;
    GLint h_f_depth_max_visualize_cdepth_;

    // Shader for calculating residue
    renderer::Shader* v_shader_residue_calc_;
    renderer::Shader* f_shader_residue_calc_;
    renderer::ShaderProgram* sp_residue_calc_;
    GLint h_residue_calc_kinect_depth_;
    GLint h_residue_calc_synth_depth_;
    GLint h_residue_calc_max_depth_;
    GLint h_residue_calc_texel_dim_;

    jtil::math::Float4x4 VW_mat_;
    jtil::math::Float4x4 PVW_mat_;

    jtil::data_str::Vector<float> matrix_data_;  // concatenated matrix data

    void renderTexturedBonedMesh(renderer::GeometryTexturedBonedMesh* geom, 
      const uint32_t i_camera, bool color = false);
    void renderColoredMesh(renderer::GeometryColoredMesh* geom, 
      const uint32_t i_camera, bool color = false);
    void renderColoredBonedMesh(renderer::GeometryColoredBonedMesh* geom, 
      const uint32_t i_camera, bool color = false);

    void insertionSortAxisExtents(uint32_t* arr, 
      uint32_t iextent);
    void findIntersections(uint32_t* min, uint32_t* max, uint32_t iextent_min, 
      uint32_t iextent_max, bool* collisions);
    void findCollisions();

    void drawDepthMapInternal(const float* coeff, 
      const uint32_t num_coeff_per_model, PoseModel** models, 
      const uint32_t num_models, const uint32_t i_camera, const bool color, 
      const bool tiled);

    // Non-copyable, non-assignable.
    ModelRenderer(ModelRenderer&);
    ModelRenderer& operator=(const ModelRenderer&);
   };

};  // unnamed namespace
