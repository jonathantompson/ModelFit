//
//  main.cpp
//

#if defined(_WIN32) && defined(_DEBUG)
  #include <Windows.h>
  #include <crtdbg.h>  // for _CrtSetDbgFlag
#endif

#include <stdlib.h>
#include <cmath>
#include <thread>
#include <iostream>
#include <iomanip>
#include <limits>

#define OPEN_NI_FUNCS_USE_OMP

#include "renderer/open_gl_common.h"
#include "renderer/renderer.h"
#include "renderer/geometry/geometry.h"
#include "renderer/colors.h"
#include "renderer/camera/camera.h"
#include "renderer/lights/light_dir.h"
#include "renderer/texture/texture_renderable.h"
#include "renderer/texture/texture.h"
#include "renderer/geometry/geometry_colored_mesh.h"
#include "renderer/geometry/geometry_manager.h"
#include "renderer/geometry/geometry_colored_points.h"
#include "renderer/geometry/geometry_colored_lines.h"
#include "renderer/gl_state.h"
#include "windowing/window.h"
#include "windowing/window_settings.h"
#include "model_fit/calibrate_geometry.h"
#include "model_fit/hand_geometry_mesh.h"
#include "model_fit/model_renderer.h"
#include "model_fit/model_fit.h"
#include "kinect_interface_primesense/open_ni_funcs.h"
#include "kinect_interface_primesense/kinect_interface_primesense.h"
#include "math/math_types.h"
#include "data_str/triple.h"
#include "data_str/vector.h"
#include "clk/clk.h"
#include "string_util/string_util.h"
#include "threading/thread_pool.h"
#include "math/icp.h"
#include "image_util/image_util.h"
#include "file_io/file_io.h"

#if defined(WIN32) || defined(_WIN32)  
  #define snprintf _snprintf_s
  #pragma warning( disable : 4099 )
#endif
#define SAFE_DELETE(x) if (x != NULL) { delete x; x = NULL; }
#define SAFE_DELETE_ARR(x) if (x != NULL) { delete[] x; x = NULL; }

// Some hard coded settings
static const int filter_size = 10;
static const float icp_pc_model_dist_thresh = 15.0f;  // mm
static const bool icp_use_points_near_model = false;
static const int icp_num_iterations = 100;
static const jtil::math::ICPMethod icp_method = jtil::math::ICPMethod::BFGS_ICP;
static const float icp_cos_norm_threshold = acosf((35.0f / 360.0f) * 2.0f * (float)M_PI);
static const float icp_min_distance_sq = 1.0f;
static const float icp_max_distance_sq = 1600.0f;  // 4cm ^ 2 = 40mm ^ 2
static const int max_icp_pts = 100000;
static const int gdt_max_dist = 5000;
static const int max_kinects = 3;
static const int num_worker_threads = 6;
static const char im_dir[] = "../data/hand_data/";

#ifndef HAND_FIT
  #error "HAND_FIT is not defined in the preprocessor definitions!"
#endif

using namespace std;
using namespace jtil::math;
using namespace jtil::data_str;
using namespace jtil::image_util;
using namespace jtil::threading;
using namespace model_fit;
using namespace renderer;
using namespace jtil::windowing;
using namespace kinect_interface_primesense;

jtil::clk::Clk* clk = NULL;
double t1, t0;

// The main window and basic rendering system
Window* wnd = NULL;
Renderer* render = NULL;
int render_output = 1;  // 1 - color, 
                        // 2 - synthetic depth, 

// Camera class for handling view and proj matrices
const float mouse_speed_rotation = 0.005f;
const float camera_speed = 300.0f;
const float camera_run_mulitiplier = 10.0f;
Float3 cur_dir(0.0f, 0.0f, 0.0f);
Float3 delta_pos;
int mouse_x, mouse_y, mouse_x_prev, mouse_y_prev;
bool camera_rotate = false;
bool scale_coeff = false;
bool middle_down = false;
bool running = false;
bool shift_down = false;

// model
Float4x4 camera_view[max_kinects];
PoseModel** models;
int cur_kinect = 0;
int cur_icp_dst_kinect = 0;
int32_t last_icp_kinect = -1;
bool render_correspondances = true;
CalibrateGeometryType cal_type = CalibrateGeometryType::ICOSAHEDRON; 
const float max_icp_dist = gdt_max_dist;
const uint32_t num_models = 1;
const uint32_t num_coeff = CalibrateCoeff::NUM_PARAMETERS;
const uint32_t num_coeff_fit = CAL_GEOM_NUM_COEFF;
bool render_all_views = 0;
jtil::data_str::VectorManaged<char*> depth_files[max_kinects];  // [kinect][frame]
jtil::data_str::VectorManaged<char*> rgb_files[max_kinects];  // [kinect][frame]
jtil::data_str::VectorManaged<int16_t*> depth_database[max_kinects];  // [kinect][frame][pix]
jtil::data_str::VectorManaged<uint8_t*> rgb_database[max_kinects];
jtil::data_str::VectorManaged<float*> coeffs[max_kinects];  // [kinect][frame][coeff]
uint32_t num_model_fit_cameras = 1;
uint32_t cur_coeff = 0;
ModelFit* fit = NULL;
bool continuous_fit = false;  // fit frames continuously each frame
bool continuous_play = false;  // Play back recorded frames
bool render_models = true;
uint32_t coeff_src = 0;
float** coeff = NULL;  // Temp space only used when performing fit
float** prev_coeff = NULL; 

// Kinect Image data 
float cur_xyz_data[max_kinects][src_dim*3];
float cur_norm_data[max_kinects][src_dim*3];
float cur_uvd_data[max_kinects][src_dim*3];
int16_t cur_depth_data[max_kinects][src_dim*3];
uint8_t cur_label_data[max_kinects][src_dim];
uint8_t cur_image_rgb[max_kinects][src_dim*3];
uint32_t cur_image = 0;
GeometryColoredPoints* geometry_points[max_kinects];
GeometryColoredLines* geometry_lines[max_kinects-1];  // For displaying ICP correspondances
OpenNIFuncs openni_funcs;
Texture* tex = NULL;
uint8_t tex_data[src_dim * 3];
const bool color_point_clouds = false;
const float point_cloud_scale = 4.0f;
const uint32_t num_point_clouds_to_render = 1;

// ICP
jtil::math::ICP<float> icp;

// Multithreading
ThreadPool* tp;

using std::cout;
using namespace jtil::file_io;
using std::endl;
jtil::math::Float4x4 mat_tmp;
WindowSettings settings;

void quit() {
  tp->stop();
  delete tp;
  for (uint32_t i = 0; i < num_models; i++) {
    models[i]->setRendererAttachement(true);  // Make sure Geometry manager deletes models
    SAFE_DELETE(models[i]);
  }
  SAFE_DELETE(tex);
  SAFE_DELETE_ARR(models);
  SAFE_DELETE_ARR(coeff);
  SAFE_DELETE_ARR(prev_coeff);
  SAFE_DELETE(clk);
  SAFE_DELETE(render);
  SAFE_DELETE(fit);
  Texture::shutdownTextureSystem();
  GLState::shutdownGLState();
  SAFE_DELETE(wnd);
  Window::killWindowSystem();
  exit(0);
}

void loadCurrentImage(bool print_to_screen = true) {
  // Average the non-zero pixels over some temporal filter kernel
  int16_t cur_depth[filter_size];
  for (int32_t k = 0; k < max_kinects; k++) {
    for (int32_t i = 0; i < src_dim; i++) {
      uint32_t filt = 0;
      for (int32_t f = (int32_t)cur_image; f < (int32_t)cur_image + filter_size && 
        f < (int32_t)depth_database[k].size(); f++, filt++) {
        cur_depth[filt] = depth_database[k][f][i];
      }
      // Now calculate the std and mean of the non-zero entries
      for ( ; filt < filter_size; filt++) {
        cur_depth[filt] = gdt_max_dist + 1;
      }
      float sum = 0;
      float sum_sqs = 0;
      float cnt = 0;
      for (int32_t filt = 0; filt < filter_size; filt++) {
        if (cur_depth[filt] != 0 && cur_depth[filt] < gdt_max_dist) {
          sum += (float)cur_depth[filt];
          sum_sqs += (float)cur_depth[filt] * (float)cur_depth[filt];
          cnt++;
        }
      }
      if (cnt < LOOSE_EPSILON) {
        cur_depth_data[k][i] = gdt_max_dist + 1;
      } else {
        float mean = sum / cnt;
        float var = sum_sqs / cnt - (mean * mean);
        if (var > 100) {
          // Calculate a new mean of all the depth values IN FRONT of the mean
          sum = 0;
          cnt = 0;
          for (int32_t filt = 0; filt < filter_size; filt++) {
            if (cur_depth[filt] < mean && cur_depth[filt] != 0) {
              sum += (float)cur_depth[filt];
              cnt++;
            }
          }
          mean = sum / cnt;
        }
        cur_depth_data[k][i] = (int16_t)mean;
      }
    }
    memcpy(cur_image_rgb[k], rgb_database[k][cur_image], 
      sizeof(cur_image_rgb[k][0]) * src_dim * 3);
    memset(cur_label_data[k], 0, sizeof(cur_label_data[k][0]) * src_dim);

    openni_funcs.ConvertDepthImageToProjective((uint16_t*)cur_depth_data[k], 
      cur_uvd_data[k]);
    openni_funcs.convertDepthToWorldCoordinates(cur_uvd_data[k], cur_xyz_data[k], 
      src_dim);
  }
}

void InitXYZPointsForRendering() {
  for (uint32_t k = 0; k < max_kinects; k++) {
    if (geometry_points[k]->synced()) {
      geometry_points[k]->unsyncVAO();
    }

    jtil::data_str::Vector<jtil::math::Float3>* vert = geometry_points[k]->vertices();
    jtil::data_str::Vector<jtil::math::Float3>* cols = geometry_points[k]->colors();

    float red_mult = 1.0;
    float green_mult = 1.0;
    if (k == 2 && color_point_clouds) {
      red_mult = 0.1f;
    }
    if (k == 1 && color_point_clouds) {
      green_mult = 0.1f;
    }

#pragma omp parallel for num_threads(4)
    for (int32_t i = 0; i < src_dim; i++) {
      float* cur_col = cols->at(i)->m;
      vert->at(i)->set(&cur_xyz_data[k][i*3]);
      cur_col[0] = red_mult * static_cast<float>(cur_image_rgb[k][i*3]) / 255.0f;
      cur_col[1] = static_cast<float>(cur_image_rgb[k][i*3+1]) / 255.0f;
      cur_col[2] = green_mult * static_cast<float>(cur_image_rgb[k][i*3+2]) / 255.0f;
    }
    
    geometry_points[k]->syncVAO();
  }
}

void saveCurrentCoeffs() {
  // Save all kinect coeffs in the same file
  char full_path[256];
  snprintf(full_path, 255, "%scoeff_%07d.bin", im_dir, cur_image);

  float calb_data[max_kinects * num_coeff];
  for (uint32_t i = 0; i < max_kinects; i++) {
    memcpy(&calb_data[i * num_coeff], coeffs[i][cur_image], 
      sizeof(*calb_data) * num_coeff);
  }
  SaveArrayToFile<float>(calb_data, max_kinects * num_coeff, full_path);
  cout << "hand data saved to file: " << full_path << endl;
}

void MouseButtonCB(int button, int action, int mods) {
  if (button == MOUSE_BUTTON_LEFT) {
    if (action == PRESSED) {
      camera_rotate = true;
    }
    if (action == RELEASED) {
      camera_rotate = false;
    }
  } else if (button == MOUSE_BUTTON_RIGHT) {
    if (action == PRESSED) {
      scale_coeff = true;
    }
    if (action == RELEASED) {
      scale_coeff = false;
    }
  } else if (button == MOUSE_BUTTON_MIDDLE) {
    if (action == PRESSED) {
      middle_down = true;
    }
    if (action == RELEASED) {
      middle_down = false;
    }
  }
}

void MousePosCB(double x, double y) {
  mouse_x_prev = mouse_x;
  mouse_y_prev = mouse_y;
  mouse_x = (int)floor(x);
  mouse_y = (int)floor(y);
  if (camera_rotate) {
    int dx = mouse_x - mouse_x_prev;
    int dy = mouse_y - mouse_y_prev;
    float theta_x = dx * mouse_speed_rotation;
    float theta_y = dy * mouse_speed_rotation;
    render->camera()->rotateCamera(theta_x, theta_y);
  }
  if (scale_coeff) {
    int dy = mouse_y - mouse_y_prev;
    float theta_y = dy * mouse_speed_rotation;
    if (/*cur_coeff >= 0 &&*/ cur_coeff <= 2) {
      theta_y *= 50.0f;
    }
    if (shift_down) {
      if (cal_type == BOX) {
        Float3& box_size = ((CalibrateGeometry*)models[0])->box_size();
        box_size[cur_coeff % 3] -= theta_y;
        ((CalibrateGeometry*)models[0])->updateSize();
        cout << "cur_coeff " << cur_coeff % 3;
        cout << " --> " << box_size[cur_coeff % 3] << endl;
      } else if (cal_type == ICOSAHEDRON) {
        ((CalibrateGeometry*)models[0])->icosahedron_scale() -= 0.001f * theta_y;
        ((CalibrateGeometry*)models[0])->updateSize();
        cout << "icosahedron_scale --> " << 
          ((CalibrateGeometry*)models[0])->icosahedron_scale() << endl;
      }
    } else {
      coeffs[cur_kinect][cur_image][cur_coeff] -= theta_y;
      cout << "cur_coeff " << cur_coeff;
      cout << " --> " << coeffs[cur_kinect][cur_image][cur_coeff] << endl;
    }
  }
  if (middle_down) {
    int dy = mouse_y - mouse_y_prev;
    float theta_y = dy * mouse_speed_rotation;
    Float3 trans(0, 0, 0);
    trans[cur_coeff % 3] -= theta_y;
    camera_view[cur_kinect].leftMultTranslation(trans);

    // Now save the results to file
    std::stringstream ss;
    ss << im_dir << "calibration_data" << cur_kinect << ".bin";
    SaveArrayToFile<float>(camera_view[cur_kinect].m, 16, ss.str());
    std::cout << "Calibration data saved to " << ss.str() << endl;
  }
}

void renderFrame(float dt);
void fitFrame(bool seed_with_last_frame, bool query_only);
double continuous_fit_timer_start;
double continuous_play_timer_start;
const double continuous_play_frame_time = 1.0/15.0;
string full_im_filename;
string r_coeff_file;
string l_coeff_file;
string new_full_im_filename;
string new_r_coeff_file;
string new_l_coeff_file;
void KeyboardCB(int key, int scancode, int action, int mods) {
  int repeat = 1;

  switch (key) {
    case KEY_LSHIFT:
      if (action == PRESSED) {
        running = true;
        shift_down = true;
      } else {
        running = false;
        shift_down = false;
      }
      break;
    case KEY_ESC:
      if (action == RELEASED) {
        quit();
      }
      break;
    case static_cast<int>('w'):
    case static_cast<int>('W'):
      if (action == PRESSED) {
        cur_dir[2] += 1;
      } else {
        cur_dir[2] -= 1;
      }
      break;
    case static_cast<int>('s'):
    case static_cast<int>('S'):
      if (action == PRESSED) {
        cur_dir[2] -= 1;
      } else {
        cur_dir[2] += 1;
      }
      break;
    case static_cast<int>('a'):
    case static_cast<int>('A'):
      if (action == PRESSED) {
        cur_dir[0] += 1;
      } else {
        cur_dir[0] -= 1;
      }
      break;
    case static_cast<int>('d'):
    case static_cast<int>('D'):
      if (action == PRESSED) {
        cur_dir[0] -= 1;
      } else {
        cur_dir[0] += 1;
      }
      break;
    case static_cast<int>('q'):
    case static_cast<int>('Q'):
      if (action == PRESSED) {
        cur_dir[1] += 1;
      } else {
        cur_dir[1] -= 1;
      }
      break;
    case static_cast<int>('e'):
    case static_cast<int>('E'):
      if (action == PRESSED) {
        cur_dir[1] -= 1;
      } else {
        cur_dir[1] += 1;
      }
      break;
    case static_cast<int>('t'):
    case static_cast<int>('T'):
      if (action == RELEASED) {
        render->wireframe = !render->wireframe;
      }
      break;
    case static_cast<int>('1'):
    case static_cast<int>('2'):
      if (action == RELEASED && !shift_down) {
        render_output = key - static_cast<int>('1') + 1;
      }
      break;
    case KEY_KP_ADD:
      if (action == RELEASED) {
        cur_image = cur_image < static_cast<uint32_t>(depth_database[0].size())-1 ? 
          cur_image+1 : cur_image;
        loadCurrentImage();
        InitXYZPointsForRendering();
      }
      break;
    case KEY_KP_SUBTRACT:
      if (action == RELEASED) {
        cur_image = cur_image > 0 ? cur_image-1 : 0;
        loadCurrentImage();
        InitXYZPointsForRendering();
      }
      break;
    case static_cast<int>('}'):
    case static_cast<int>(']'):
      if (action == RELEASED) {
        cur_coeff = (cur_coeff + 1) % num_coeff;
        cout << "cur_coeff = " << cur_coeff << std::endl;
      }
      break;
    case static_cast<int>('{'):
    case static_cast<int>('['):
      if (action == RELEASED) {
        cur_coeff = cur_coeff != 0 ? (cur_coeff - 1) : num_coeff - 1;
        cout << "cur_coeff = " << cur_coeff << std::endl;
      }
      break;
    case static_cast<int>('f'):
    case static_cast<int>('F'):
      if (action == RELEASED) {
        cout << "Fitting model..." << endl;
        fitFrame(false, false);
      }
      break;
    case static_cast<int>('g'):
    case static_cast<int>('G'):
      if (action == RELEASED) {
        continuous_fit = !continuous_play && !continuous_fit;
        if (continuous_fit) {
          fit->resetFuncEvalCount();
          continuous_fit_timer_start = clk->getTime();
        }
      }
      break;
    case static_cast<int>('p'):
    case static_cast<int>('P'):
      if (action == RELEASED) {
        if (!shift_down) {
          continuous_play = !continuous_fit && !continuous_play;
        }
      }
      break;
    case static_cast<int>('l'):
    case static_cast<int>('L'):
      if (action == RELEASED) {
        cur_image = 0;
        loadCurrentImage();
        InitXYZPointsForRendering();
      }
      break;
    case static_cast<int>('c'):
    case static_cast<int>('C'): 
      if (action == RELEASED) {
        if (cur_kinect == cur_icp_dst_kinect) {
          std::cout << "Source and destination kinects are the same!" << std::endl;
          break;
        }
        uint32_t k_dst = cur_icp_dst_kinect;  // This point cloud wont move
        uint32_t k_src = cur_kinect;  // 
        CalcNormalImage(cur_norm_data[k_src], cur_xyz_data[k_src], src_width, 
          src_height, 50, SimpleNormalApproximation);
        CalcNormalImage(cur_norm_data[k_dst], cur_xyz_data[k_dst], src_width, 
          src_height, 50, SimpleNormalApproximation);

        // Since k_dst wont move, we can save it's data to file.
        std::stringstream ss;
        ss << im_dir << "calibration_data" << k_dst << ".bin";
        SaveArrayToFile<float>(camera_view[k_dst].m, 16, ss.str());
        std::cout << "Calibration data saved to " << ss.str() << endl;

        Vector<float> pc1_src, pc2_src, npc1_src, npc2_src;
        bool use_points_near_model = icp_use_points_near_model;
        if (use_points_near_model) {
          // Collect the points that are just near the fitted model
          CalibrateGeometry* cal_model = (CalibrateGeometry*)models[0];
          cal_model->findPointsCloseToModel(pc1_src, npc1_src, 
            cur_xyz_data[k_dst], cur_norm_data[k_dst], coeffs[k_dst][cur_image], 
            icp_pc_model_dist_thresh);
          cal_model->findPointsCloseToModel(pc2_src, npc2_src, 
            cur_xyz_data[k_src], cur_norm_data[k_src], coeffs[k_src][cur_image], 
            icp_pc_model_dist_thresh);
        } else {
          for (uint32_t i = 0; i < src_dim; i++) {
            if (cur_xyz_data[k_dst][i * 3 + 2] < gdt_max_dist &&
              cur_xyz_data[k_dst][i * 3 + 2] > 0) {
              // We have to pre-transform PC1:
  
              pc1_src.pushBack(cur_xyz_data[k_dst][i * 3]);
              pc1_src.pushBack(cur_xyz_data[k_dst][i * 3 + 1]);
              pc1_src.pushBack(cur_xyz_data[k_dst][i * 3 + 2]);
              npc1_src.pushBack(cur_norm_data[k_dst][i * 3]);
              npc1_src.pushBack(cur_norm_data[k_dst][i * 3 + 1]);
              npc1_src.pushBack(cur_norm_data[k_dst][i * 3 + 2]);
            }
            if (cur_xyz_data[k_src][i * 3 + 2] < gdt_max_dist &&
              cur_xyz_data[k_src][i * 3 + 2] > 0) {
              pc2_src.pushBack(cur_xyz_data[k_src][i * 3]);
              pc2_src.pushBack(cur_xyz_data[k_src][i * 3 + 1]);
              pc2_src.pushBack(cur_xyz_data[k_src][i * 3 + 2]);
              npc2_src.pushBack(cur_norm_data[k_src][i * 3]);
              npc2_src.pushBack(cur_norm_data[k_src][i * 3 + 1]);
              npc2_src.pushBack(cur_norm_data[k_src][i * 3 + 2]);
            }
          }
        }

        // Now, since the PC2 point cloud gets transformed by it's matrix
        // in the ICP routine, we have to pre-transform PC1 points by it's
        // matrix incase the view matrix isn't identity
        Float3 pt, pt_transformed, norm, norm_transformed;
        Float4x4 normal_mat;
        Float4x4::inverse(normal_mat, camera_view[k_dst]);
        normal_mat.transpose();
        for (uint32_t i = 0; i < pc1_src.size(); i+= 3) {
          pt.set(&pc1_src[i]);
          Float3::affineTransformPos(pt_transformed, camera_view[k_dst], pt);
          pc1_src[i] = pt_transformed[0];
          pc1_src[i + 1] = pt_transformed[1];
          pc1_src[i + 2] = pt_transformed[2];
          norm.set(&npc1_src[i]);
          Float3::affineTransformVec(norm_transformed, normal_mat, norm);
          norm_transformed.normalize();
          npc1_src[i] = norm_transformed[0];
          npc1_src[i + 1] = norm_transformed[1];
          npc1_src[i + 2] = norm_transformed[2];
        }

        // Randomly permute the point clouds to find a subset
        Vector<float> pc1, npc1;
        MERSINE_TWISTER_ENG eng;
        jtil::data_str::Vector<int> indices;
        for (uint32_t i = 0; i < pc1_src.size() / 3; i++) {
          indices.pushBack(i);
        }
        int size = std::min<int>((int)(pc1_src.size()/3), max_icp_pts);
        for (int i = 0; i < size; i++) {
          UNIFORM_INT_DISTRIBUTION dist(i, indices.size()-1);
          int rand_index = dist(eng);
          int tmp = indices[i];
          indices[i] = indices[rand_index];
          indices[rand_index] = tmp;
        }
        indices.resize(size);
        for (int i = 0; i < (int)indices.size(); i++) {
          pc1.pushBack(pc1_src[indices[i] * 3]);
          pc1.pushBack(pc1_src[indices[i] * 3 + 1]);
          pc1.pushBack(pc1_src[indices[i] * 3 + 2]);
          npc1.pushBack(npc1_src[indices[i] * 3]);
          npc1.pushBack(npc1_src[indices[i] * 3 + 1]);
          npc1.pushBack(npc1_src[indices[i] * 3 + 2]);
        }

        Vector<float> pc2, npc2;
        indices.resize(0);
        for (uint32_t i = 0; i < pc2_src.size() / 3; i++) {
          indices.pushBack(i);
        }
        size = std::min<int>((int)(pc2_src.size()/3), max_icp_pts);
        for (int i = 0; i < size; i++) {
          UNIFORM_INT_DISTRIBUTION dist(i, indices.size()-1);
          int rand_index = dist(eng);
          int tmp = indices[i];
          indices[i] = indices[rand_index];
          indices[rand_index] = tmp;
        }
        indices.resize(size);
        for (int i = 0; i < (int)indices.size(); i++) {
          pc2.pushBack(pc2_src[indices[i] * 3]);
          pc2.pushBack(pc2_src[indices[i] * 3 + 1]);
          pc2.pushBack(pc2_src[indices[i] * 3 + 2]);
          npc2.pushBack(npc2_src[indices[i] * 3]);
          npc2.pushBack(npc2_src[indices[i] * 3 + 1]);
          npc2.pushBack(npc2_src[indices[i] * 3 + 2]);
        }

        // Approximate the camera by using the fitted model coeffs
        ((CalibrateGeometry*)models[0])->calcCameraView(
          camera_view[k_dst], camera_view[k_src], k_dst, k_src, 
          coeffs, cur_image);

        // Now perform ICP for a tight fit
        icp.num_iterations = icp_num_iterations;
        icp.cos_normal_threshold = icp_cos_norm_threshold;
        icp.min_distance_sq = icp_min_distance_sq;
        icp.max_distance_sq = icp_max_distance_sq;
        icp.icp_method = icp_method;
        std::cout << "Performing ICP on " << (pc1.size()/3) << " and ";
        std::cout << (pc2.size()/3) << " pts" << std::endl;
        // Use Normals
        icp.match(camera_view[k_src], &pc1[0], (pc1.size()/3), 
          &pc2[0], (pc2.size()/3), camera_view[k_src], &npc1[0],
          &npc2[0]);

        // Create lines geometry from the last correspondance points:
        float* pc2_transformed = icp.getLastPC2Transformed();
        int* correspondances = icp.getLastCorrespondances();
        float* weights = icp.getLastWeights();
        float red[3] = {1, 0, 0};
        float blue[3] = {0, 0, 1};
        SAFE_DELETE(geometry_lines[k_src-1]);
        geometry_lines[k_src-1] = new GeometryColoredLines();
        for (uint32_t i = 0; i < (pc2.size()/3); i++) {
          if (weights[i] > EPSILON) {
            geometry_lines[k_src-1]->addLine(&pc2_transformed[i * 3],
              &pc1[correspondances[i] * 3], red, blue);
          }
        }
        geometry_lines[k_src-1]->syncVAO();

        // Now save the results to file
        ss.str("");
        ss << im_dir << "calibration_data" << k_src << ".bin";
        SaveArrayToFile<float>(camera_view[k_src].m, 16, ss.str());
        std::cout << "Calibration data saved to " << ss.str() << endl;
        last_icp_kinect = k_src;
      }
      break;
    case static_cast<int>('v'):
    case static_cast<int>('V'): 
      if (action == RELEASED) {
        render_correspondances = !render_correspondances;
        cout << "render_correspondances = " << render_correspondances << endl;
      }
      break;
    case static_cast<int>('x'):
    case static_cast<int>('X'): 
      if (action == RELEASED) {
        render_all_views = !render_all_views;
        cout << "render_all_views = " << render_all_views << endl;
      }
      break;
    case static_cast<int>('h'):
    case static_cast<int>('H'):
      if (action == RELEASED) {
        saveCurrentCoeffs();
      }
      break;
    case static_cast<int>('i'):
    case static_cast<int>('I'):
      if (action == RELEASED) {
        if (!shift_down) {
          cur_kinect = (cur_kinect + 1) % max_kinects;
          cout << "cur_kinect = " << cur_kinect << endl;
        } else {
          cur_icp_dst_kinect = (cur_icp_dst_kinect + 1) % max_kinects;
          cout << "cur_icp_dst_kinect = " << cur_icp_dst_kinect << endl;
        }
      }
      break;
    case static_cast<int>('Y'):
    case static_cast<int>('y'):
      if (action == RELEASED) {
        render_models = !render_models;
        for (uint32_t i = 0; i < num_models; i++) {
          models[i]->setRendererAttachement(render_models);
        }
      }
      break;
    case static_cast<int>('j'):
    case static_cast<int>('J'):
      if (action == RELEASED) {
        fitFrame(false, true);
      }
      break;
  }
}

void fitFrame(bool seed_with_last_frame, bool query_only) {
  // if query_only = true then we'll save the images to file (for the paper)
  if (seed_with_last_frame && cur_image > 0) {
    for (uint32_t k = 0; k < max_kinects; k++) {
      memcpy(coeffs[k][cur_image], coeffs[k][cur_image-1], 
        sizeof(coeffs[k][cur_image][0]) * num_coeff);
    }
  }
  // Just fit each kinect independantly
  for (uint32_t k = 0; k < max_kinects; k++) {
    int16_t* depth = cur_depth_data[k];
    uint8_t* labels = cur_label_data[k];
    fit->fitModel(&depth, &labels, models, 
      &coeffs[k][cur_image], NULL, CalibrateGeometry::renormalizeCoeffs);
  }
}

void renderFrame(float dt) {
  // Move the camera
  delta_pos.set(cur_dir);
  const Float3 zeros(0, 0, 0);
  if (!Float3::equal(delta_pos, zeros)) {
    delta_pos.normalize();
    Float3::scale(delta_pos, camera_speed * dt);
    if (running) {
      Float3::scale(delta_pos, camera_run_mulitiplier);
    }
    render->camera()->moveCamera(&delta_pos);
  }

  if (!render_all_views) {
    models[0]->updateMatrices(coeffs[cur_kinect][cur_image]);
  } else {
    models[0]->updateMatrices(coeffs[0][cur_image]);
  }

  // Now render the final frame
  Float4x4 identity;
  identity.identity();
  switch (render_output) {
  case 1:
    render->renderFrame(dt);
    {
      if (!render_all_views) {
        render->renderColoredPointCloud(geometry_points[cur_kinect], 
          &identity, 1.5f * static_cast<float>(settings.width) / 4.0f);
      } else {
        for (uint32_t k = 0; k < max_kinects; k++) {
          if (k == last_icp_kinect && icp.getTransforms().size() > 0) {
            render->renderColoredPointCloud(geometry_points[k], 
              &icp.getTransforms()[icp.getTransforms().size()-1], 
              point_cloud_scale * 1.5f * static_cast<float>(settings.width) / 4.0f);
          } else {
            render->renderColoredPointCloud(geometry_points[k], 
              &camera_view[k], 
              point_cloud_scale * 1.5f * static_cast<float>(settings.width) / 4.0f);
          }
        }

      }
      if (render_correspondances) {
        for (uint32_t k = 0; k < max_kinects - 1; k++) {
          if (geometry_lines[k] != NULL) {
            render->renderColoredLines(geometry_lines[k], &identity, 4.0f);
          }
        }
      }
    }
    break;
  case 2:
    float coeff[num_models * num_coeff_fit];
    memcpy(coeff, coeffs[cur_kinect][cur_image], sizeof(coeff[0]) * num_coeff_fit);

    fit->model_renderer()->drawDepthMap(coeff, num_coeff_fit, models,
      num_models, 0, false);
    fit->model_renderer()->visualizeDepthMap(wnd, 0);
    break;
  default:
    throw runtime_error("ERROR: render_output is an incorrect value");
  }
  wnd->swapBackBuffer();
}

int main(int argc, char *argv[]) {
  static_cast<void>(argc); static_cast<void>(argv);
#if defined(_DEBUG) && defined(_WIN32)
  _CrtSetDbgFlag(_CRTDBG_ALLOC_MEM_DF | _CRTDBG_LEAK_CHECK_DF);
  //_CrtSetBreakAlloc(3022);
#endif

  cout << "Usage:" << endl;
  cout << "WSADQE - Move camera" << endl;
  cout << "shift - Sprint" << endl;
  cout << "mouse left click + drag - Rotate camera" << endl;
  cout << "space - Change hand to control" << endl;
  cout << "mouse right click + drag - Adjust coefficient" << endl;
  cout << "[,] - Change the current coeff to adjust" << endl;
  cout << "t - wireframe rendering" << endl;
  cout << "y - Render Hands ON/OFF" << endl;
  cout << "1,2 - Render output type" << endl;
  cout << "+,- - Change the current depth image" << endl;
  cout << "h - Store model coefficient data to file" << endl;
  cout << "f - Fit model to current frame" << endl;
  cout << "g - Fit model to all remaining frames" << endl;
  cout << "p - Playback frames (@15fps)" << endl;
  cout << "l - Go to start frame" << endl;
  cout << "c - Save calibration data" << endl;
  cout << "x - Render all views" << endl;
  cout << "v - Render Correspondances (if ICP has been run)" << endl;
  cout << "j - Query Objective Function Value" << endl;
  cout << "shift+12345 - Copy finger1234/thumb from last frame" << endl;
  cout << "k - (3 times) delete current file" << endl;
  cout << "i - Change the current kinect (for ICP to move onto dst)" << endl;
  cout << "I - Change the current dst kinect" << endl << endl;
  
  try {
    tp = new ThreadPool(num_worker_threads);

    clk = new jtil::clk::Clk();
    t1 = clk->getTime();
    
    // Initialize Windowing system
    Window::initWindowSystem();
    Texture::initTextureSystem();
    
    // Fill the settings structure
    settings.width = src_width*2;
    settings.height = src_height*2;
    settings.fullscreen = false;
    settings.title = string("CalibrateKinects");
    settings.gl_major_version = 3;
    settings.gl_minor_version = 2;
    settings.num_depth_bits = 24;
    settings.num_stencil_bits = 0;
    settings.num_rgba_bits = 8;
    settings.samples = 1;
    
    // Create the window
    wnd = new Window(settings);
    GLState::initGLState();    
    
    // Create an instance of the renderer
    FloatQuat eye_rot; eye_rot.identity();
    Float3 eye_pos(0, 0, 0);
    render = new Renderer();
    render->background_color.set(0.098f, 0.098f, 0.3922f, 1.0f);
    float fov_vert_deg = 360.0f * OpenNIFuncs::fVFOV_primesense_109 / 
      (2.0f * (float)M_PI);
    render->init(eye_rot, eye_pos, settings.width, settings.height,
      -HAND_MODEL_CAMERA_VIEW_PLANE_NEAR, -HAND_MODEL_CAMERA_VIEW_PLANE_FAR, 
      fov_vert_deg);

    tex = new Texture(GL_RGB8, src_width, src_height, GL_RGB, 
      GL_UNSIGNED_BYTE, (unsigned char*)tex_data, 
      TEXTURE_WRAP_MODE::TEXTURE_CLAMP, false,
      TEXTURE_FILTER_MODE::TEXTURE_NEAREST);

    // Initialize the XYZ points geometry
    for (uint32_t k = 0; k < max_kinects; k++) {
      geometry_points[k] = new GeometryColoredPoints;
      geometry_points[k]->vertices()->capacity(src_dim);
      geometry_points[k]->vertices()->resize(src_dim);
      geometry_points[k]->colors()->capacity(src_dim);
      geometry_points[k]->colors()->resize(src_dim);
    }
    for (uint32_t k = 0; k < max_kinects-1; k++) {
      geometry_lines[k] = NULL;
    }
 
    // Load the Kinect data for fitting from file and process it
    char full_path[256];
    for (uint32_t k = 0; k < max_kinects; k++) {
      snprintf(full_path, 255, "%sdepth_%d_*", im_dir, k+1);
      jtil::file_io::ls(full_path, depth_files[k]);
      snprintf(full_path, 255, "%srgb_%d_*", im_dir, k+1);
      jtil::file_io::ls(full_path, rgb_files[k]);
      if (k > 0) {
        if (depth_files[k].size() != depth_files[0].size() || 
            rgb_files[k].size() != rgb_files[0].size()) {
          throw std::runtime_error("Inconsistent number of frames!");
        }
      }
      if (depth_files[k].size() != rgb_files[k].size()) {
        throw std::runtime_error("Inconsistent number of frames!");
      }
    }

    	  // Make sure the 0th index kinect has at least one file
	  if (depth_files[0].size() < 1) {
      std::cout << "ERROR: No kinect data found on disk. Quitting..." <<
        std::endl;
#if defined(_WIN32) || defined(WIN32)
      system("pause");
#endif
      exit(-1);
    }

    // Load in all the images from file.  We do this for calibration data since
    // there wont be that many frames and because we need access to all the
    // point clouds to average them anyway.
    for (uint32_t k = 0; k < max_kinects; k++) {
      for (uint32_t f = 0; f < rgb_files[k].size(); f++) {
        // Load in the RGB
        uint32_t w, h, nchan;
        uint8_t* cur_rgb = NULL;
        snprintf(full_path, 255, "%s%s", im_dir, rgb_files[k][f]);
        renderer::Texture::loadImFromFile(full_path, cur_rgb, w, h, nchan);
        if (!cur_rgb || w != src_width || h != src_height || nchan != 3) {
          throw std::runtime_error("Data might be corrupted!");
        }
        rgb_database[k].pushBack(cur_rgb);

        // Load in the depth
        uint8_t* rgb_tmp = NULL;
        snprintf(full_path, 255, "%s%s", im_dir, depth_files[k][f]);
        renderer::Texture::loadImFromFile(full_path, rgb_tmp, w, h, nchan);
        if (!rgb_tmp || w != src_width || h != src_height || nchan != 3) {
          throw std::runtime_error("Data might be corrupted!");
        }
        // Unpack the depth (the MSB is in the green and the LSB in the red)
        int16_t* cur_depth = new int16_t[src_dim];
        for (uint32_t i = 0; i < src_dim; i++) {
          uint8_t g = rgb_tmp[i*3+1];
          uint8_t b = rgb_tmp[i*3+2];
          cur_depth[i] = (((int16_t)g) << 8) | (int16_t)b;
        }
        depth_database[k].pushBack(cur_depth);
        delete[] rgb_tmp;
      }
    }

    loadCurrentImage();
  
    // Attach callback functions for event handling
    wnd->registerKeyboardCB(&KeyboardCB);
    wnd->registerMousePosCB(&MousePosCB);
    wnd->registerMouseButCB(&MouseButtonCB);
    wnd->registerMouseWheelCB(NULL);

    // Create hand_renderer instances of the models to fit
    coeff = new float*[num_models];
    prev_coeff = new float*[num_models];
    models = new PoseModel*[num_models];
    models[0] = new CalibrateGeometry(cal_type);

    // Create the optimizer that will fit the models
    fit = new ModelFit(num_models, num_coeff_fit, num_model_fit_cameras);

    // Load the coeffs from file (if they exist)
    float calb_data[max_kinects * num_coeff];
    for (uint32_t k = 0; k < max_kinects; k++) {
      for (uint32_t i = 0; i < depth_database[0].size(); i++) {
        coeffs[k].pushBack(new float[num_coeff]);
      }
    }

    for (uint32_t i = 0; i < depth_database[0].size(); i++) {
      snprintf(full_path, 255, "%scoeff_%07d.bin", im_dir, i);
      if (fileExists(full_path)) {
        LoadArrayFromFile<float>(calb_data, num_coeff * max_kinects, full_path);
        for (uint32_t k = 0; k < max_kinects; k++) {
          memcpy(coeffs[k][i], &calb_data[k * num_coeff], sizeof(calb_data[0])*
            num_coeff);
        }
      } else {
        // No saved coeffient files.  Start with default
        for (uint32_t k = 0; k < max_kinects; k++) {
          for (uint32_t c = 0; c < num_coeff; c++) {
            coeffs[k][i][c] = 0.0f;
          }
          coeffs[k][i][CALIB_POS_Z] = 700.0f;  // Start 700 away 
        } 
      }
    }

    for (uint32_t i = 0; i < num_models; i++) {
      models[i]->setRendererAttachement(render_models);
    }

    // Finally, initialize the points for rendering
    InitXYZPointsForRendering();
    
    // Main render loop
    while (true) {
      t0 = t1;
      t1 = clk->getTime();
      float dt = static_cast<float>(t1-t0);

      if (continuous_fit) {
        if (cur_image < depth_database[0].size() - 1) {
          cout << "fitting frame " << cur_image + 1 << " of ";
          cout << depth_database[0].size() << endl;
          cur_image++;
          loadCurrentImage();
          fitFrame(true, false);
          saveCurrentCoeffs();
          InitXYZPointsForRendering();
        } else {
          std::cout << "Function Evals Per Second = ";
          std::cout << fit->func_eval_count() / (clk->getTime() - 
            continuous_fit_timer_start);
          continuous_fit = false;
        }
      }
      if (continuous_play) {
        if (cur_image < depth_database[0].size() - 1) {
          continuous_play_timer_start += (t1 - t0);
          if (continuous_play_timer_start >= continuous_play_frame_time) {
            cur_image ++;
            loadCurrentImage(false);
            continuous_play_timer_start = 0;
            InitXYZPointsForRendering();
          }
        } else {
          continuous_play = false;
          continuous_play_timer_start = 0;
        }
      }
      
      renderFrame(dt);
      
      // let someone else do some work
      std::this_thread::yield();
    }
  } catch (std::runtime_error e) {
    printf("%s\n", e.what());
    system("PAUSE");
  }
  
  return 0;
}
