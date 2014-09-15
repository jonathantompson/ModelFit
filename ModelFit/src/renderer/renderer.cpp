#include <sstream>
#include "renderer/open_gl_common.h"
#include "renderer/renderer.h"
#include "renderer/shader/shader.h"
#include "renderer/shader/shader_program.h"
#include "renderer/camera/camera.h"
#include "renderer/geometry/geometry_manager.h"
#include "renderer/geometry/geometry.h"
#include "renderer/geometry/geometry_colored_mesh.h"
#include "renderer/geometry/geometry_colored_boned_mesh.h"
#include "renderer/geometry/geometry_textured_mesh.h"
#include "renderer/geometry/geometry_textured_boned_mesh.h"
#include "renderer/geometry/geometry_vertices.h"
#include "renderer/geometry/geometry_points.h"
#include "renderer/geometry/geometry_colored_points.h"
#include "renderer/geometry/geometry_colored_lines.h"
#include "renderer/texture/texture.h"
#include "renderer/texture/texture_renderable.h"
#include "renderer/colors.h"
#include "renderer/lights/light_dir.h"
#include "renderer/lights/light_dir_handles.h"
#include "model_fit/bounding_sphere.h"
#include "windowing/window.h"
#include "math/math_types.h"
#include "renderer/gl_state.h"

using std::string;
using std::runtime_error;
using jtil::math::Float2;
using jtil::math::Float3;
using jtil::math::Float4;
using jtil::math::FloatQuat;
using jtil::math::Float4x4;
using renderer::Geometry;
using windowing::Window;

#define SAFE_DELETE(target) \
  if (target != NULL) { \
    delete target; \
    target = NULL; \
  }

namespace renderer {

  Renderer* Renderer::g_renderer_;

  // These are vertex shader positions
  const ShaderLocationNamePair Renderer::pos(VERTEX_POS_LOC, "v_position");
  const ShaderLocationNamePair Renderer::col(VERTEX_COL_LOC, "v_color");
  const ShaderLocationNamePair Renderer::norm(VERTEX_NOR_LOC, "v_normal");
  const ShaderLocationNamePair Renderer::tex(VERTEX_TEX_LOC, "v_texture");
  const ShaderLocationNamePair Renderer::bone_ids_03(VERTEX_BONE_IDS_03_LOC, "v_bone_ids_03");
  const ShaderLocationNamePair Renderer::bone_weights_03(VERTEX_BONE_WEIGHTS_03_LOC, "v_bone_weights_03");
  //const ShaderLocationNamePair Renderer::bone_ids_47(VERTEX_BONE_IDS_47_LOC, "v_bone_ids_47");
  //const ShaderLocationNamePair Renderer::bone_weights_47(VERTEX_BONE_WEIGHTS_47_LOC, "v_bone_weights_47");

  Renderer::Renderer() {
    if (g_renderer_ != NULL) {
      throw std::runtime_error("ERROR: g_renderer_ does not equal NULL!");
    }
    g_renderer_ = this;

    camera_ = NULL;

    sp_cmesh_dlight_ = NULL;
    v_shader_cmesh_dlight_ = NULL;
    f_shader_cmesh_dlight_ = NULL;
    h_sp_cmesh_dlight_ = NULL;

    sp_cmesh_bone_dlight_ = NULL;
    v_shader_cmesh_bone_dlight_ = NULL;
    f_shader_cmesh_bone_dlight_ = NULL;
    h_sp_cmesh_bone_dlight_ = NULL;

    sp_tmesh_dlight_ = NULL;
    v_shader_tmesh_dlight_ = NULL;
    f_shader_tmesh_dlight_ = NULL;
    h_sp_tmesh_dlight_ = NULL;

    sp_tmesh_bone_dlight_ = NULL;
    v_shader_tmesh_bone_dlight_ = NULL;
    f_shader_tmesh_bone_dlight_ = NULL;
    h_sp_tmesh_bone_dlight_ = NULL;

    quad_ = NULL;

    test_texture_ = NULL;
    light_dir_ = NULL;

    sp_points_ = NULL;
    v_shader_points_ = NULL;
    f_shader_points_ = NULL;

    sp_cpoints_ = NULL;
    v_shader_cpoints_ = NULL;
    f_shader_cpoints_ = NULL;

    sp_clines_ = NULL;
    v_shader_clines_ = NULL;
    f_shader_clines_ = NULL;

    sp_downsample2_ = NULL;
    sp_downsample2_integ_ = NULL;
    sp_downsample4_ = NULL;
    sp_downsample4_integ_ = NULL;
    sp_downsample5_integ_ = NULL;
    f_shader_downsample2_ = NULL;
    f_shader_downsample2_integ_ = NULL;
    f_shader_downsample4_ = NULL;
    f_shader_downsample4_integ_ = NULL;
    f_shader_downsample5_integ_ = NULL;

    sp_fullscreen_quad_ = NULL;
    v_shader_fullscreen_quad_ = NULL;
    f_shader_fullscreen_quad_ = NULL;

    frame_counter_ = 0;
    wireframe = false;
    render_bounding_spheres = false;

    background_color.set(0.15f, 0.15f, 0.3f, 1.0f);
  }

  Renderer::~Renderer() {
    GeometryManager::destroyGeometryManager();
    SAFE_DELETE(camera_);

    SAFE_DELETE(sp_cmesh_dlight_);
    SAFE_DELETE(v_shader_cmesh_dlight_);
    SAFE_DELETE(f_shader_cmesh_dlight_);
    SAFE_DELETE(h_sp_cmesh_dlight_);

    SAFE_DELETE(sp_cmesh_bone_dlight_);
    SAFE_DELETE(v_shader_cmesh_bone_dlight_);
    SAFE_DELETE(f_shader_cmesh_bone_dlight_);
    SAFE_DELETE(h_sp_cmesh_bone_dlight_);

    SAFE_DELETE(sp_tmesh_dlight_);
    SAFE_DELETE(v_shader_tmesh_dlight_);
    SAFE_DELETE(f_shader_tmesh_dlight_);
    SAFE_DELETE(h_sp_tmesh_dlight_);

    SAFE_DELETE(sp_tmesh_bone_dlight_);
    SAFE_DELETE(v_shader_tmesh_bone_dlight_);
    SAFE_DELETE(f_shader_tmesh_bone_dlight_);
    SAFE_DELETE(h_sp_tmesh_bone_dlight_);

    SAFE_DELETE(quad_);

    SAFE_DELETE(test_texture_);
    SAFE_DELETE(light_dir_);

    SAFE_DELETE(sp_points_);
    SAFE_DELETE(v_shader_points_);
    SAFE_DELETE(f_shader_points_);

    SAFE_DELETE(sp_cpoints_);
    SAFE_DELETE(v_shader_cpoints_);
    SAFE_DELETE(f_shader_cpoints_);

    SAFE_DELETE(sp_clines_);
    SAFE_DELETE(v_shader_clines_);
    SAFE_DELETE(f_shader_clines_);

    SAFE_DELETE(sp_downsample2_);
    SAFE_DELETE(sp_downsample2_integ_);
    SAFE_DELETE(sp_downsample4_);
    SAFE_DELETE(sp_downsample4_integ_);
    SAFE_DELETE(sp_downsample5_integ_);
    SAFE_DELETE(f_shader_downsample2_);
    SAFE_DELETE(f_shader_downsample2_integ_);
    SAFE_DELETE(f_shader_downsample4_);
    SAFE_DELETE(f_shader_downsample4_integ_);
    SAFE_DELETE(f_shader_downsample5_integ_);

    SAFE_DELETE(sp_fullscreen_quad_);
    SAFE_DELETE(v_shader_fullscreen_quad_);
    SAFE_DELETE(f_shader_fullscreen_quad_);
  }

  void Renderer::init(FloatQuat& eye_rot, Float3& eye_pos, int screen_width,
                      int screen_height, float znear, float zfar, float field_of_view) {
    screen_width_ = screen_width;
    screen_height_ = screen_height;
    field_of_view_ = field_of_view;

    GeometryManager::initGeometryManager();
    
    camera_ = new Camera(&eye_rot, &eye_pos, screen_width_, screen_height_,
                         field_of_view_, znear, zfar);
    
    test_texture_ = new Texture("./textures/bee.tga",
                                TEXTURE_WRAP_MODE::TEXTURE_CLAMP);
    
    // Build the Lights
    light_dir_ = new LightDir();
    light_dir_->direction_world(Float3(-0.2352941f, 0.0f, 0.9411764f));
    light_dir_->color(DIR_LIGHT_COLOR);
    light_dir_->ambient_intensity(DIR_LIGHT_AMBIENT_INTENSITY);
    light_dir_->diffuse_intensity(DIR_LIGHT_DIFFUSE_INTENSITY);
    
    // Fullscreen quad vertices
    quad_ = GeometryVertices::makeQuad();
    
    // Colored mesh shader
    v_shader_cmesh_dlight_ = new Shader("shaders/cmesh_dlight.vert",
                                        VERTEX_SHADER);
    f_shader_cmesh_dlight_ = new Shader("shaders/cmesh_dlight.frag",
                                        FRAGMENT_SHADER);
    sp_cmesh_dlight_ = new ShaderProgram(v_shader_cmesh_dlight_, 
      f_shader_cmesh_dlight_);
    sp_cmesh_dlight_->bindVertShaderInputLocation(Renderer::pos);
    sp_cmesh_dlight_->bindVertShaderInputLocation(Renderer::norm);
    sp_cmesh_dlight_->bindVertShaderInputLocation(Renderer::col);
    sp_cmesh_dlight_->link();
    h_sp_cmesh_dlight_ = new LightDirHandles();
    h_sp_cmesh_dlight_->getHandles(sp_cmesh_dlight_);
    h_PVW_mat_cmesh_dlight_ = sp_cmesh_dlight_->getUniformLocation("PVW_mat");
    h_VW_mat_cmesh_dlight_ = sp_cmesh_dlight_->getUniformLocation("VW_mat");
    h_Normal_mat_cmesh_dlight_ = sp_cmesh_dlight_->getUniformLocation("Normal_mat");
    h_mat_specular_intensity_cmesh_dlight_ =
    sp_cmesh_dlight_->getUniformLocation("mat_specular_intensity");
    h_mat_specular_power_cmesh_dlight_ =
    sp_cmesh_dlight_->getUniformLocation("mat_specular_power");

    // Colored and boned mesh shader
#ifndef LINEAR_BLEND_SKINNING
    v_shader_cmesh_bone_dlight_ = new Shader("shaders/cmesh_bone_dlight.vert",
      VERTEX_SHADER);
#else
    v_shader_cmesh_bone_dlight_ = new Shader("shaders/cmesh_bone_dlight_lbs.vert",
      VERTEX_SHADER);
#endif
    f_shader_cmesh_bone_dlight_ = new Shader("shaders/cmesh_bone_dlight.frag",
                                             FRAGMENT_SHADER);
    sp_cmesh_bone_dlight_ = new ShaderProgram(v_shader_cmesh_bone_dlight_, 
      f_shader_cmesh_bone_dlight_);
    sp_cmesh_bone_dlight_->bindVertShaderInputLocation(Renderer::pos);
    sp_cmesh_bone_dlight_->bindVertShaderInputLocation(Renderer::norm);
    sp_cmesh_bone_dlight_->bindVertShaderInputLocation(Renderer::col);
    sp_cmesh_bone_dlight_->bindVertShaderInputLocation(Renderer::bone_ids_03);
    sp_cmesh_bone_dlight_->bindVertShaderInputLocation(Renderer::bone_weights_03);
    //sp_cmesh_bone_dlight_->bindVertShaderInputLocation(Renderer::bone_ids_47);
    //sp_cmesh_bone_dlight_->bindVertShaderInputLocation(Renderer::bone_weights_47);
    sp_cmesh_bone_dlight_->link();
    h_sp_cmesh_bone_dlight_ = new LightDirHandles();
    h_sp_cmesh_bone_dlight_->getHandles(sp_cmesh_bone_dlight_);
    h_PVW_mat_cmesh_bone_dlight_ = sp_cmesh_bone_dlight_->getUniformLocation("PVW_mat");
    h_VW_mat_cmesh_bone_dlight_ = sp_cmesh_bone_dlight_->getUniformLocation("VW_mat");
    std::stringstream ss;
    for (uint32_t i = 0; i < MAX_BONE_COUNT; i++) {
      ss.str(std::string());  // Clear the string stream
      ss << "bone_trans[" << i << "]";
      h_bone_trans_cmesh_bone_dlight_[i] = 
        sp_cmesh_bone_dlight_->getUniformLocation(ss.str().c_str());
    }
    h_Normal_mat_cmesh_bone_dlight_ = sp_cmesh_bone_dlight_->getUniformLocation("Normal_mat");
    h_mat_specular_intensity_cmesh_bone_dlight_ =
    sp_cmesh_bone_dlight_->getUniformLocation("mat_specular_intensity");
    h_mat_specular_power_cmesh_bone_dlight_ =
    sp_cmesh_bone_dlight_->getUniformLocation("mat_specular_power");

    // Textured mesh shader
    v_shader_tmesh_dlight_ = new Shader("shaders/tmesh_dlight.vert",
                                        VERTEX_SHADER);
    f_shader_tmesh_dlight_ = new Shader("shaders/tmesh_dlight.frag",
                                        FRAGMENT_SHADER);
    sp_tmesh_dlight_ = new ShaderProgram(v_shader_tmesh_dlight_, 
      f_shader_tmesh_dlight_);
    sp_tmesh_dlight_->bindVertShaderInputLocation(Renderer::pos);
    sp_tmesh_dlight_->bindVertShaderInputLocation(Renderer::norm);
    sp_tmesh_dlight_->bindVertShaderInputLocation(Renderer::tex);
    sp_tmesh_dlight_->link();
    h_sp_tmesh_dlight_ = new LightDirHandles();
    h_sp_tmesh_dlight_->getHandles(sp_tmesh_dlight_);
    h_PVW_mat_tmesh_dlight_ = sp_tmesh_dlight_->getUniformLocation("PVW_mat");
    h_VW_mat_tmesh_dlight_ = sp_tmesh_dlight_->getUniformLocation("VW_mat");
    h_Normal_mat_tmesh_dlight_ = sp_tmesh_dlight_->getUniformLocation("Normal_mat");
    h_mat_specular_intensity_tmesh_dlight_ =
    sp_tmesh_dlight_->getUniformLocation("mat_specular_intensity");
    h_mat_specular_power_tmesh_dlight_ =
    sp_tmesh_dlight_->getUniformLocation("mat_specular_power");
    h_tmesh_dlight_texture_sampler_ = 
      sp_tmesh_dlight_->getUniformLocation("f_texture_sampler");

    // Textured and boned mesh shader
#ifndef LINEAR_BLEND_SKINNING
    v_shader_tmesh_bone_dlight_ = new Shader("shaders/tmesh_bone_dlight.vert",
      VERTEX_SHADER);
#else
    v_shader_tmesh_bone_dlight_ = new Shader("shaders/tmesh_bone_dlight_lbs.vert",
      VERTEX_SHADER);
#endif
    f_shader_tmesh_bone_dlight_ = new Shader("shaders/tmesh_bone_dlight.frag",
      FRAGMENT_SHADER);
    sp_tmesh_bone_dlight_ = new ShaderProgram(v_shader_tmesh_bone_dlight_, 
      f_shader_tmesh_bone_dlight_);
    sp_tmesh_bone_dlight_->bindVertShaderInputLocation(Renderer::pos);
    sp_tmesh_bone_dlight_->bindVertShaderInputLocation(Renderer::norm);
    sp_tmesh_bone_dlight_->bindVertShaderInputLocation(Renderer::tex);
    sp_tmesh_bone_dlight_->bindVertShaderInputLocation(Renderer::bone_ids_03);
    sp_tmesh_bone_dlight_->bindVertShaderInputLocation(Renderer::bone_weights_03);
    //sp_tmesh_bone_dlight_->bindVertShaderInputLocation(Renderer::bone_ids_47);
    //sp_tmesh_bone_dlight_->bindVertShaderInputLocation(Renderer::bone_weights_47);
    sp_tmesh_bone_dlight_->link();
    h_sp_tmesh_bone_dlight_ = new LightDirHandles();
    h_sp_tmesh_bone_dlight_->getHandles(sp_tmesh_bone_dlight_);
    h_PVW_mat_tmesh_bone_dlight_ = sp_tmesh_bone_dlight_->getUniformLocation("PVW_mat");
    h_VW_mat_tmesh_bone_dlight_ = sp_tmesh_bone_dlight_->getUniformLocation("VW_mat");
    for (uint32_t i = 0; i < MAX_BONE_COUNT; i++) {
      ss.str(std::string());  // Clear the string stream
      ss << "bone_trans[" << i << "]";
      h_bone_trans_tmesh_bone_dlight_[i] = 
        sp_tmesh_bone_dlight_->getUniformLocation(ss.str().c_str());
    }
    h_Normal_mat_tmesh_bone_dlight_ = sp_tmesh_bone_dlight_->getUniformLocation("Normal_mat");
    h_mat_specular_intensity_tmesh_bone_dlight_ =
    sp_tmesh_bone_dlight_->getUniformLocation("mat_specular_intensity");
    h_mat_specular_power_tmesh_bone_dlight_ =
    sp_tmesh_bone_dlight_->getUniformLocation("mat_specular_power");
    h_tmesh_bone_dlight_texture_sampler_ = 
      sp_tmesh_bone_dlight_->getUniformLocation("f_texture_sampler");
    
    // Fullscreen quad shaders
    v_shader_fullscreen_quad_ = new Shader("shaders/fullscreen_quad.vert",
                                           ShaderType::VERTEX_SHADER);
    f_shader_fullscreen_quad_ = new Shader("shaders/fullscreen_quad.frag",
                                           ShaderType::FRAGMENT_SHADER);
    sp_fullscreen_quad_ = new ShaderProgram(v_shader_fullscreen_quad_,
                                            f_shader_fullscreen_quad_);
    sp_fullscreen_quad_->bindVertShaderInputLocation(Renderer::pos);
    sp_fullscreen_quad_->link();
    h_fullscreen_quad_texture_sampler_ =
      sp_fullscreen_quad_->getUniformLocation("f_texture_sampler");

    // Downsample shader
    f_shader_downsample2_ = new Shader("shaders/downsample2.frag",
                                       ShaderType::FRAGMENT_SHADER);
    sp_downsample2_ = new ShaderProgram(v_shader_fullscreen_quad_,
                                        f_shader_downsample2_);
    sp_downsample2_->bindVertShaderInputLocation(Renderer::pos);
    sp_downsample2_->link();
    h_downsample2_texture_sampler_ = sp_downsample2_->
      getUniformLocation("f_texture_sampler");
    h_downsample2_texel_size_ = sp_downsample2_->
      getUniformLocation("texel_size");

    // Downsample + integrate shader
    f_shader_downsample2_integ_ = new Shader("shaders/downsample2_integ.frag",
                                             ShaderType::FRAGMENT_SHADER);
    sp_downsample2_integ_ = new ShaderProgram(v_shader_fullscreen_quad_,
                                              f_shader_downsample2_integ_);
    sp_downsample2_integ_->bindVertShaderInputLocation(Renderer::pos);
    sp_downsample2_integ_->link();
    h_downsample2_integ_texture_sampler_ = sp_downsample2_integ_->
      getUniformLocation("f_texture_sampler");
    h_downsample2_integ_texel_size_ = sp_downsample2_integ_->
      getUniformLocation("texel_size");

    // Downsample shader
    f_shader_downsample4_ = new Shader("shaders/downsample4.frag",
                                       ShaderType::FRAGMENT_SHADER);
    sp_downsample4_ = new ShaderProgram(v_shader_fullscreen_quad_,
                                        f_shader_downsample4_);
    sp_downsample4_->bindVertShaderInputLocation(Renderer::pos);
    sp_downsample4_->link();
    h_downsample4_texture_sampler_ = sp_downsample4_->
      getUniformLocation("f_texture_sampler");
    h_downsample4_texel_size_ = sp_downsample4_->
      getUniformLocation("texel_size");

    // Downsample + integrate shader
    f_shader_downsample4_integ_ = new Shader("shaders/downsample4_integ.frag",
                                             ShaderType::FRAGMENT_SHADER);
    sp_downsample4_integ_ = new ShaderProgram(v_shader_fullscreen_quad_,
                                              f_shader_downsample4_integ_);
    sp_downsample4_integ_->bindVertShaderInputLocation(Renderer::pos);
    sp_downsample4_integ_->link();
    h_downsample4_integ_texture_sampler_ = sp_downsample4_integ_->
      getUniformLocation("f_texture_sampler");
    h_downsample4_integ_texel_size_ = sp_downsample4_integ_->
      getUniformLocation("texel_size");

    // Downsample + integrate shader
    f_shader_downsample5_integ_ = new Shader("shaders/downsample5_integ.frag",
                                             ShaderType::FRAGMENT_SHADER);
    sp_downsample5_integ_ = new ShaderProgram(v_shader_fullscreen_quad_,
                                              f_shader_downsample5_integ_);
    sp_downsample5_integ_->bindVertShaderInputLocation(Renderer::pos);
    sp_downsample5_integ_->link();
    h_downsample5_integ_texture_sampler_ = sp_downsample5_integ_->
      getUniformLocation("f_texture_sampler");
    h_downsample5_integ_texel_size_ = sp_downsample5_integ_->
      getUniformLocation("texel_size");
    
    // Point cloud rendering
    v_shader_points_ = new Shader("shaders/points.vert", ShaderType::VERTEX_SHADER);
    f_shader_points_ = new Shader("shaders/points.frag", ShaderType::FRAGMENT_SHADER);
    sp_points_ = new ShaderProgram(v_shader_points_, f_shader_points_);
    sp_points_->bindVertShaderInputLocation(Renderer::pos);
    sp_points_->link();
    h_PVW_mat_points_ = sp_points_->getUniformLocation("PVW_mat");
    h_VW_mat_points_ = sp_points_->getUniformLocation("VW_mat");
    h_point_color_points_ = sp_points_->getUniformLocation("point_color");
    h_point_size_constant_ = sp_points_->getUniformLocation("point_size_constant");
    
    // Colored Point cloud rendering
    v_shader_cpoints_ = new Shader("shaders/cpoints.vert", ShaderType::VERTEX_SHADER);
    f_shader_cpoints_ = new Shader("shaders/cpoints.frag", ShaderType::FRAGMENT_SHADER);
    sp_cpoints_ = new ShaderProgram(v_shader_cpoints_, f_shader_cpoints_);
    sp_cpoints_->bindVertShaderInputLocation(Renderer::pos);
    sp_cpoints_->bindVertShaderInputLocation(Renderer::col);
    sp_cpoints_->link();
    h_PVW_mat_cpoints_ = sp_cpoints_->getUniformLocation("PVW_mat");
    h_VW_mat_cpoints_ = sp_cpoints_->getUniformLocation("VW_mat");
    h_cpoint_size_constant_ = sp_cpoints_->getUniformLocation("point_size_constant");

    // Colored line rendering
    v_shader_clines_ = new Shader("shaders/clines.vert", ShaderType::VERTEX_SHADER);
    f_shader_clines_ = new Shader("shaders/clines.frag", ShaderType::FRAGMENT_SHADER);
    sp_clines_ = new ShaderProgram(v_shader_clines_, f_shader_clines_);
    sp_clines_->bindVertShaderInputLocation(Renderer::pos);
    sp_clines_->bindVertShaderInputLocation(Renderer::col);
    sp_clines_->link();
    h_PVW_mat_clines_ = sp_clines_->getUniformLocation("PVW_mat");
    //h_VW_mat_clines_ = sp_clines_->getUniformLocation("VW_mat");
    //h_cline_size_constant_ = sp_clines_->getUniformLocation("line_size_constant");
    
    // Force a resize event TO set the correct viewport and projection matricies
    resize(screen_width_, screen_height_);
  }

  void Renderer::renderFrame(float dt) {
#if defined(_DEBUG) || defined(DEBUG)
    GLState::verifyOpenGLState();
#endif

    static_cast<void>(dt);
    GLState::glsViewport(0, 0, screen_width_, screen_height_);
    GeometryManager* geom_manager = GeometryManager::g_geom_manager();
    
    // Update all heirachical matrices (for renderable geometry)
    updateMatrices();  
    geom_manager->updateBoneMatrices();
  
    // Update camera parameters
    updateCameraFOVScreenSize();  // SettingsManager may have changed them
    camera_->updateView();
    camera_->updateProjection();

    // Update lights
    light_dir_->update(camera_->view());
    
    // Render final result to screen
    GLState::glsBindFramebuffer(GL_FRAMEBUFFER, 0);  // FB 0 is the screen
    
    GLState::glsDepthMask(GL_TRUE);  // Enable the depth buffer for writing
    GLState::glsDisable(GL_BLEND);
    GLState::glsEnable(GL_DEPTH_TEST);
    GLState::glsDepthFunc(GL_LESS);
    // GLState::glsDisable(GL_VERTEX_PROGRAM_POINT_SIZE);
    
    GLState::glsClearColor(background_color[0], background_color[1],
      background_color[2], background_color[3]);
    GLState::glsClear(GL_DEPTH_BUFFER_BIT | GL_COLOR_BUFFER_BIT);

    // Render all the Bounding sphere's in wireframe
    // Batch render to avoid overhead of switching shaders
    sp_cmesh_dlight_->useProgram();
    h_sp_cmesh_dlight_->setHandles(light_dir_, this);

    if (render_bounding_spheres) {
      GLState::glsPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
      GLState::glsDisable(GL_CULL_FACE);
      geom_manager->renderStackReset();
      Float4x4 tmp;
      Float4x4 root_inverse;
      Geometry* root = NULL;  // Avoid calculating inverse many times
      // --> A hack. Might actually calculate many times!
      while (!geom_manager->renderStackEmpty()) {
        Geometry* cur_geom =  geom_manager->renderStackPop();
        if (cur_geom->type() == GeometryType::BOUNDING_SPHERE) {
          // This is a bit of a hack, but we have to undo the parent's root
          // transform (by left multiplying by its inverse).
          // Then we have to left multiply by the mesh node's transform
          BoundingSphere* sphere = reinterpret_cast<BoundingSphere*>(cur_geom);
          Geometry* cur_root = sphere->hand_root();
          if (cur_root != root) {
            Float4x4::inverse(root_inverse, *sphere->hand_root()->mat());
            root = cur_root;
          }
          Float4x4::mult(tmp, root_inverse, *sphere->mat_hierarchy());
          Float4x4::mult(*sphere->mat_hierarchy(), *sphere->mesh_node()->mat_hierarchy(), tmp);
          renderColoredMesh(reinterpret_cast<GeometryColoredMesh*>(cur_geom));
        }
      }
    }

    // Done rendering wireframe objects...
    if (!wireframe) {
      GLState::glsPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
      GLState::glsEnable(GL_CULL_FACE);
      GLState::glsCullFace(GL_BACK);
    } else {
      GLState::glsPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
      GLState::glsDisable(GL_CULL_FACE);
    }

    // Render all the colored meshes 
    geom_manager->renderStackReset();
    while (!geom_manager->renderStackEmpty()) {
      Geometry* cur_geom =  geom_manager->renderStackPop();
      if (cur_geom->type() == GeometryType::GEOMETRY_COLORED_MESH) {
        renderColoredMesh(reinterpret_cast<GeometryColoredMesh*>(cur_geom));
      }
    }

    // Render all the textured meshes
    sp_tmesh_dlight_->useProgram();
    h_sp_tmesh_dlight_->setHandles(light_dir_, this);

    geom_manager->renderStackReset();
    while (!geom_manager->renderStackEmpty()) {
      Geometry* cur_geom =  geom_manager->renderStackPop();
      if (cur_geom->type() == GeometryType::GEOMETRY_TEXTURED_MESH) {
        renderTexturedMesh(reinterpret_cast<GeometryTexturedMesh*>(cur_geom));
      }
    }

    // Render all the textured and boned meshes
    sp_tmesh_bone_dlight_->useProgram();
    h_sp_tmesh_bone_dlight_->setHandles(light_dir_, this);

    geom_manager->renderStackReset();
    while (!geom_manager->renderStackEmpty()) {
      Geometry* cur_geom =  geom_manager->renderStackPop();
      if (cur_geom->type() == GeometryType::GEOMETRY_TEXTURED_BONED_MESH) {
        renderTexturedBonedMesh(reinterpret_cast<GeometryTexturedBonedMesh*>(cur_geom));
      }
    }

    // Render all the colored and boned meshes
    sp_cmesh_bone_dlight_->useProgram();
    h_sp_cmesh_bone_dlight_->setHandles(light_dir_, this);

    geom_manager->renderStackReset();
    while (!geom_manager->renderStackEmpty()) {
      Geometry* cur_geom =  geom_manager->renderStackPop();
      if (cur_geom->type() == GeometryType::GEOMETRY_COLORED_BONED_MESH) {
        renderColoredBonedMesh(reinterpret_cast<GeometryColoredBonedMesh*>(cur_geom));
      }
    }
    
    // TEST: Just try rendering a full screen quad to make sure shaders are
    //       working and the window manager is working.
    // renderFullscreenQuad(test_texture_);

    frame_counter_++;
  }

  void Renderer::renderColoredMesh(GeometryColoredMesh* geom) {
    // Calculate the model view matrix and bind it to the shader
    Float4x4::mult(VW_mat_, *camera_->view(), *geom->mat_hierarchy());
    bindFloat4x4(&VW_mat_, h_VW_mat_cmesh_dlight_);

    // Calculate the model view normal matrix and bind it to the shader
    // Normal matrix is the (M_modelview^-1)^T:
    // --> http://www.songho.ca/opengl/gl_transform.html
    Float4x4::affineInverse(Normal_mat_, VW_mat_);
    Normal_mat_.transpose();
    bindFloat4x4(&Normal_mat_, h_Normal_mat_cmesh_dlight_);

    // Calculate model view projection matrix and bind it to the shader
    Float4x4::mult(PVW_mat_, *camera_->proj(), VW_mat_ );
    bindFloat4x4(&PVW_mat_, h_PVW_mat_cmesh_dlight_);

    // Set the material properties
    bindFloat1(geom->mtrl()->specular_intensity, 
      h_mat_specular_intensity_cmesh_dlight_);
    bindFloat1(geom->mtrl()->specular_power, 
      h_mat_specular_power_cmesh_dlight_);

    // Draw the current geometry
    geom->draw();
  }

  void Renderer::renderColoredBonedMesh(GeometryColoredBonedMesh* geom) {
    // Calculate the model view matrix and bind it to the shader
    Float4x4::mult(VW_mat_, *camera_->view(), *geom->mat_hierarchy());
    bindFloat4x4(&VW_mat_, h_VW_mat_cmesh_bone_dlight_);

    // Calculate the model view normal matrix and bind it to the shader
    // Normal matrix is the (M_modelview^-1)^T:
    // --> http://www.songho.ca/opengl/gl_transform.html
    Float4x4::affineInverse(Normal_mat_, VW_mat_);
    Normal_mat_.transpose();
    bindFloat4x4(&Normal_mat_, h_Normal_mat_cmesh_bone_dlight_);

    // Calculate model view projection matrix and bind it to the shader
    Float4x4::mult(PVW_mat_, *camera_->proj(), VW_mat_ );
    bindFloat4x4(&PVW_mat_, h_PVW_mat_cmesh_bone_dlight_);

    // Send the bone matrix heirachy down to the shader:
    BoneFileInfo* bones_in_file = geom->bones();
    for (uint32_t i = 0; i < bones_in_file->bones.size(); i++) {
#ifndef LINEAR_BLEND_SKINNING
      bindFloat2x4(bones_in_file->bones[i]->uniform_dual_quaternion, 
        h_bone_trans_cmesh_bone_dlight_[i]);
#else
      bindFloat4x4(&bones_in_file->bones[i]->final_trans, 
        h_bone_trans_cmesh_bone_dlight_[i]);
#endif
    }

    // Set the material properties
    bindFloat1(geom->mtrl()->specular_intensity, 
      h_mat_specular_intensity_cmesh_bone_dlight_);
    bindFloat1(geom->mtrl()->specular_power, 
      h_mat_specular_power_cmesh_bone_dlight_);

    // Draw the current geometry
    geom->draw();
  }

  void Renderer::renderTexturedMesh(GeometryTexturedMesh* geom) {
    // Calculate the model view matrix and bind it to the shader
    Float4x4::mult(VW_mat_, *camera_->view(), *geom->mat_hierarchy());
    bindFloat4x4(&VW_mat_, h_VW_mat_tmesh_dlight_);

    // Calculate the model view normal matrix and bind it to the shader
    // Normal matrix is the (M_modelview^-1)^T:
    // --> http://www.songho.ca/opengl/gl_transform.html
    Float4x4::affineInverse(Normal_mat_, VW_mat_);
    Normal_mat_.transpose();
    bindFloat4x4(&Normal_mat_, h_Normal_mat_tmesh_dlight_);

    // Calculate model view projection matrix and bind it to the shader
    Float4x4::mult(PVW_mat_, *camera_->proj(), VW_mat_ );
    bindFloat4x4(&PVW_mat_, h_PVW_mat_tmesh_dlight_);

    // Set the material properties
    bindFloat1(geom->mtrl()->specular_intensity, 
      h_mat_specular_intensity_tmesh_dlight_);
    bindFloat1(geom->mtrl()->specular_power, 
      h_mat_specular_power_tmesh_dlight_);

    // Bind the texture
    geom->tex()->bind(GL_TEXTURE0, h_tmesh_dlight_texture_sampler_);

    // Draw the current geometry
    geom->draw();
  }

  void Renderer::renderTexturedBonedMesh(GeometryTexturedBonedMesh* geom) {
    // Calculate the model view matrix and bind it to the shader
    Float4x4::mult(VW_mat_, *camera_->view(), *geom->mat_hierarchy());
    bindFloat4x4(&VW_mat_, h_VW_mat_tmesh_bone_dlight_);

    // Calculate the model view normal matrix and bind it to the shader
    // Normal matrix is the (M_modelview^-1)^T:
    // --> http://www.songho.ca/opengl/gl_transform.html
    Float4x4::affineInverse(Normal_mat_, VW_mat_);
    Normal_mat_.transpose();
    bindFloat4x4(&Normal_mat_, h_Normal_mat_tmesh_bone_dlight_);

    // Calculate model view projection matrix and bind it to the shader
    Float4x4::mult(PVW_mat_, *camera_->proj(), VW_mat_ );
    bindFloat4x4(&PVW_mat_, h_PVW_mat_tmesh_bone_dlight_);

    // Send the bone matrix heirachy down to the shader:
//    BoneFileInfo* bones_in_file = geom->bones();
//    for (uint32_t i = 0; i < bones_in_file->bones.size(); i++) {
//#ifndef LINEAR_BLEND_SKINNING
//      bindFloat2x4(bones_in_file->bones[i]->uniform_dual_quaternion, 
//        h_bone_trans_tmesh_bone_dlight_[i]);
//#else
//      bindFloat4x4(&bones_in_file->bones[i]->final_trans, 
//        h_bone_trans_tmesh_bone_dlight_[i]);
//#endif
//    }
// Send the bone matrix heirachy down to the shader:
    BoneFileInfo* bones_in_file = geom->bones();
    // First copy the bone data into a flat float array:
    if (matrix_data_.capacity() < bones_in_file->bones.size() * 16) {
      matrix_data_.capacity(bones_in_file->bones.size() * 16);
      matrix_data_.resize(bones_in_file->bones.size() * 16);
    }

    for (uint32_t i = 0; i < bones_in_file->bones.size(); i++) {
#ifndef LINEAR_BLEND_SKINNING
      memcpy(matrix_data_.at(i*8), 
        bones_in_file->bones[i]->uniform_dual_quaternion, 8 * 
        sizeof(matrix_data_[0]));
#else
      memcpy(matrix_data_.at(i*16), bones_in_file->bones[i]->final_trans.m,
        16 * sizeof(matrix_data_[0]));
#endif
      //#ifndef LINEAR_BLEND_SKINNING
      //      if (color) {
      //        g_renderer_->bindFloat2x4(bones_in_file->bones[i]->uniform_dual_quaternion, 
      //          h_bone_trans_sp_cdepth_skinned_[i]);
      //      } else {
      //        g_renderer_->bindFloat2x4(bones_in_file->bones[i]->uniform_dual_quaternion, 
      //          h_bone_trans_sp_depth_skinned_[i]);
      //      }
      //#else
      //      if (color) {
      //        g_renderer_->bindFloat4x4(&bones_in_file->bones[i]->final_trans, 
      //          h_bone_trans_sp_depth_skinned_[i]);
      //      } else {
      //        g_renderer_->bindFloat4x4(&bones_in_file->bones[i]->final_trans, 
      //          h_bone_trans_sp_cdepth_skinned_[i]);
      //      }
      //#endif
    }
    // Now BIND the entire array at once
#ifndef LINEAR_BLEND_SKINNING
    glUniformMatrix2x4fv(h_bone_trans_tmesh_bone_dlight_[0], 
      bones_in_file->bones.size(), GL_FALSE, matrix_data_.at(0));
#else
    glUniformMatrix4fv(h_bone_trans_tmesh_bone_dlight_[0], 
      bones_in_file->bones.size(), GL_FALSE, matrix_data_.at(0));
#endif
    ERROR_CHECK;

    // Set the material properties
    bindFloat1(geom->mtrl()->specular_intensity, 
      h_mat_specular_intensity_tmesh_bone_dlight_);
    bindFloat1(geom->mtrl()->specular_power, 
      h_mat_specular_power_tmesh_bone_dlight_);

    // Bind the texture
    geom->tex()->bind(GL_TEXTURE0, h_tmesh_bone_dlight_texture_sampler_);

    // Draw the current geometry
    geom->draw();
  }

  void Renderer::resize(uint32_t width, uint32_t height) {
    camera_->updateProjection();
    camera_->updateView();
    GLState::glsViewport(0, 0, width, height);
    ERROR_CHECK;
  }

  void Renderer::updateMatrices() {
    GeometryManager* geom_manager = GeometryManager::g_geom_manager();
    geom_manager->renderStackReset();
    while (!geom_manager->renderStackEmpty()) {
      Geometry* cur_geom =  geom_manager->renderStackPop();
      // Update the render matrix based on our parents position
      if (cur_geom->parent() != NULL) {
        Float4x4::mult(*cur_geom->mat_hierarchy(),
                       *cur_geom->parent()->mat_hierarchy(), *cur_geom->mat());
      } else {
        cur_geom->mat_hierarchy()->set(*cur_geom->mat());
      }
    }
  }

  void Renderer::bindFloat4x4(const Float4x4* mat, const GLint h_mat) {
    glUniformMatrix4fv(h_mat, 1, GL_FALSE, mat->m);
    ERROR_CHECK;
  }

  void Renderer::bindFloat2x4(const float mat[2][4], const GLint h_mat) {
    // TEMP CODE:
    float tmp[8];
    tmp[0] = mat[0][0];
    tmp[1] = mat[0][1];
    tmp[2] = mat[0][2];
    tmp[3] = mat[0][3];
    tmp[4] = mat[1][0];
    tmp[5] = mat[1][1];
    tmp[6] = mat[1][2];
    tmp[7] = mat[1][3];
    glUniformMatrix2x4fv(h_mat, 1, GL_FALSE, tmp);
    // glUniformMatrix2x4fv(h_mat, 1, GL_FALSE, reinterpret_cast<const float*>(&mat[0][0]));
    // END TEMP CODE:
    ERROR_CHECK;
  }

  void Renderer::bindFloat4x4Array(const float* mat_array, uint32_t count, const GLint h_mat) {
    glUniformMatrix4fv(h_mat, count, GL_FALSE, mat_array);
    ERROR_CHECK;
  }

  void Renderer::bindFloat1(const float val, const GLint h_vec) {
    glUniform1f(h_vec, val);
    ERROR_CHECK;
  }

  void Renderer::bindFloat2(const Float2* vec, const GLint h_vec) {
    glUniform2fv(h_vec, 1, vec->m);
    ERROR_CHECK;
  }

  void Renderer::bindFloat3(const Float3* vec, const GLint h_vec) {
    glUniform3fv(h_vec, 1, vec->m);
    ERROR_CHECK;
  }

  void Renderer::bindFloat4(const Float4* vec, const GLint h_vec) {
    glUniform4fv(h_vec, 1, vec->m);
    ERROR_CHECK;
  }

  void Renderer::setVertexAttribPointer(const int id, int size, 
    int type, bool normalized, int stride, const void* pointer) {
    GLState::glsVertexAttribPointer(id, size, type, normalized, stride, pointer);
    GLState::glsEnableVertexAttribArray(id);
  }
  
  void Renderer::updateCameraFOVScreenSize() {
    camera_->screen_width(static_cast<float>(screen_width_));
    camera_->screen_height(static_cast<float>(screen_height_));
    camera_->field_of_view(field_of_view_);
  }
  
  void Renderer::renderFullscreenQuad(Texture* tex, GLint frame_buffer_dst) {
    // Render to the framebuffer
    GLState::glsBindFramebuffer(GL_FRAMEBUFFER, frame_buffer_dst);
    // TO DO: Fix this --> Need width and height of the framebuffer!
    GLState::glsViewport(0, 0, screen_width_, screen_height_);
    
    GLState::glsClearColor(1.0f, 1.0f, 1.0f, 1.0f);
    GLState::glsClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    
    sp_fullscreen_quad_->useProgram();
    
    tex->bind(GL_TEXTURE0, h_fullscreen_quad_texture_sampler_);
    
    GLState::glsEnable(GL_CULL_FACE);
    GLState::glsDisable(GL_DEPTH_TEST);
    GLState::glsCullFace(GL_BACK);
    GLState::glsDisable(GL_BLEND);
    
    quad_->draw();
  }

  void Renderer::renderFullscreenQuad(Texture* tex, const bool clear) {
    if (clear) {
      GLState::glsClearColor(1.0f, 1.0f, 1.0f, 1.0f);
      GLState::glsClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    }
    
    sp_fullscreen_quad_->useProgram();
    
    tex->bind(GL_TEXTURE0, h_fullscreen_quad_texture_sampler_);
    
    GLState::glsEnable(GL_CULL_FACE);
    GLState::glsDisable(GL_DEPTH_TEST);
    GLState::glsCullFace(GL_BACK);
    GLState::glsDisable(GL_BLEND);
    
    quad_->draw();
  }

  void Renderer::renderFullscreenQuad(TextureRenderable* tex, 
    GLint frame_buffer_dst, uint32_t texture_index) {
    // Render to the framebuffer
    GLState::glsBindFramebuffer(GL_FRAMEBUFFER, frame_buffer_dst);
    // TO DO: Fix this --> Need width and height of the framebuffer!
    GLState::glsViewport(0, 0, screen_width_, screen_height_);
    
    GLState::glsClearColor(1.0f, 1.0f, 1.0f, 1.0f);
    GLState::glsClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    
    sp_fullscreen_quad_->useProgram();
    
    tex->bind(texture_index, GL_TEXTURE0, h_fullscreen_quad_texture_sampler_);
    
    GLState::glsEnable(GL_CULL_FACE);
    GLState::glsDisable(GL_DEPTH_TEST);
    GLState::glsCullFace(GL_BACK);
    GLState::glsDisable(GL_BLEND);
    
    quad_->draw();
  }
  
  void Renderer::renderPointCloud(GeometryPoints* points, Float4x4* mat_world,
    const Float3* color, const float point_size_constant) {
    sp_points_->useProgram();
    
    GLState::glsEnable(GL_VERTEX_PROGRAM_POINT_SIZE);
    bindFloat1(point_size_constant, h_point_size_constant_);
    bindFloat3(color, h_point_color_points_);
    
    // Calculate the model view projection matrix and bind it to the shader
    Float4x4::mult(VW_mat_, *camera_->view(), *mat_world);
    bindFloat4x4(&VW_mat_, h_VW_mat_points_);
    Float4x4::mult(PVW_mat_, *camera_->proj(), VW_mat_ );
    bindFloat4x4(&PVW_mat_, h_PVW_mat_points_);
    
    points->draw();
  }
  
  void Renderer::renderColoredPointCloud(GeometryColoredPoints* points,
    Float4x4* mat_world, const float point_size_constant) {
    sp_cpoints_->useProgram();
    
    GLState::glsEnable(GL_VERTEX_PROGRAM_POINT_SIZE);
    bindFloat1(point_size_constant, h_cpoint_size_constant_);
    
    // Calculate the model view projection matrix and bind it to the shader
    Float4x4::mult(VW_mat_, *camera_->view(), *mat_world);
    bindFloat4x4(&VW_mat_, h_VW_mat_cpoints_);
    Float4x4::mult(PVW_mat_, *camera_->proj(), VW_mat_ );
    bindFloat4x4(&PVW_mat_, h_PVW_mat_cpoints_);
    
    points->draw();
  }

  void Renderer::renderColoredLines(GeometryColoredLines* lines,
    Float4x4* mat_world, const float line_size_constant) {
    sp_clines_->useProgram();
    
    //GLState::glsEnable(GL_VERTEX_PROGRAM_POINT_SIZE);
    //bindFloat1(line_size_constant, h_cline_size_constant_);
    
    // Calculate the model view projection matrix and bind it to the shader
    Float4x4::mult(VW_mat_, *camera_->view(), *mat_world);
    //bindFloat4x4(&VW_mat_, h_VW_mat_clines_);
    Float4x4::mult(PVW_mat_, *camera_->proj(), VW_mat_ );
    bindFloat4x4(&PVW_mat_, h_PVW_mat_clines_);
    
    lines->draw();
  }


  void Renderer::downsample2Texture(TextureRenderable* dst, 
    TextureRenderable* src) {
    if (dst->w()*2 != src->w() || dst->h()*2 != src->h()) {
      throw std::runtime_error(string("Renderer::downsample2Texture - ERROR") +
        string(": dst and src textures are incompatible!"));
    }

    // Render to the dst texture
    dst->begin();
    
    sp_downsample2_->useProgram();
    
    src->bind(0, GL_TEXTURE0, h_downsample2_texture_sampler_);
    Float2 texel_size(1.0f / src->w(), 1.0f / src->h());
    bindFloat2(&texel_size, h_downsample2_texel_size_);
    
    GLState::glsEnable(GL_CULL_FACE);
    GLState::glsDisable(GL_DEPTH_TEST);
    GLState::glsCullFace(GL_BACK);
    GLState::glsDisable(GL_BLEND);
    
    quad_->draw();

    dst->end();
  }

  void Renderer::downsample2IntegTexture(TextureRenderable* dst, 
    TextureRenderable* src) {
    if (dst->w()*2 != src->w() || dst->h()*2 != src->h()) {
      throw std::runtime_error(string("Renderer::downsample2IntegTexture - ") +
        string("ERROR: dst and src textures are incompatible!"));
    }

    // Render to the dst texture
    dst->begin();
    
    sp_downsample2_integ_->useProgram();
    
    src->bind(0, GL_TEXTURE0, h_downsample2_integ_texture_sampler_);
    Float2 texel_size(1.0f / src->w(), 1.0f / src->h());
    bindFloat2(&texel_size, h_downsample2_integ_texel_size_);
    
    GLState::glsEnable(GL_CULL_FACE);
    GLState::glsDisable(GL_DEPTH_TEST);
    GLState::glsCullFace(GL_BACK);
    GLState::glsDisable(GL_BLEND);
    
    quad_->draw();

    dst->end();
  }

  void Renderer::downsample4Texture(TextureRenderable* dst, 
    TextureRenderable* src) {
    if (dst->w()*4 != src->w() || dst->h()*4 != src->h()) {
      throw std::runtime_error(string("Renderer::downsample4Texture - ERROR") +
        string(": dst and src textures are incompatible!"));
    }

    // Render to the dst texture
    dst->begin();
    
    sp_downsample4_->useProgram();
    
    src->bind(0, GL_TEXTURE0, h_downsample4_texture_sampler_);
    Float2 texel_size(1.0f / src->w(), 1.0f / src->h());
    bindFloat2(&texel_size, h_downsample4_texel_size_);

    GLState::glsTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    GLState::glsTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST); 
    GLState::glsTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    GLState::glsTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    
    GLState::glsEnable(GL_CULL_FACE);
    GLState::glsDisable(GL_DEPTH_TEST);
    GLState::glsCullFace(GL_BACK);
    GLState::glsDisable(GL_BLEND);
    
    quad_->draw();

    dst->end();
  }

  void Renderer::downsample4IntegTexture(TextureRenderable* dst, 
    TextureRenderable* src) {
    if (dst->w()*4 != src->w() || dst->h()*4 != src->h()) {
      throw std::runtime_error(string("Renderer::downsample4IntegTexture - ") +
        string("ERROR: dst and src textures are incompatible!"));
    }

    // Render to the dst texture
    dst->begin();
    
    sp_downsample4_integ_->useProgram();
    
    src->bind(0, GL_TEXTURE0, h_downsample4_integ_texture_sampler_);
    Float2 texel_size(1.0f / src->w(), 1.0f / src->h());
    bindFloat2(&texel_size, h_downsample4_integ_texel_size_);

    GLState::glsTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    GLState::glsTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST); 
    GLState::glsTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    GLState::glsTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);

    GLState::glsEnable(GL_CULL_FACE);
    GLState::glsDisable(GL_DEPTH_TEST);
    GLState::glsCullFace(GL_BACK);
    GLState::glsDisable(GL_BLEND);
    
    quad_->draw();

    dst->end();
  }

  void Renderer::downsample5IntegTexture(TextureRenderable* dst, 
    TextureRenderable* src) {
    if (dst->w()*5 != src->w() || dst->h()*5 != src->h()) {
      throw std::runtime_error(string("Renderer::downsample5IntegTexture - ") +
        string("ERROR: dst and src textures are incompatible!"));
    }

    // Render to the dst texture
    dst->begin();
    
    sp_downsample5_integ_->useProgram();
    
    src->bind(0, GL_TEXTURE0, h_downsample5_integ_texture_sampler_);
    Float2 texel_size(1.0f / src->w(), 1.0f / src->h());
    bindFloat2(&texel_size, h_downsample5_integ_texel_size_);

    GLState::glsTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    GLState::glsTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST); 
    GLState::glsTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    GLState::glsTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);

    GLState::glsEnable(GL_CULL_FACE);
    GLState::glsDisable(GL_DEPTH_TEST);
    GLState::glsCullFace(GL_BACK);
    GLState::glsDisable(GL_BLEND);
    
    quad_->draw();

    dst->end();
  }
}
