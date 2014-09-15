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
#include "kinect_interface_primesense/hand_model_coeff.h"
#include "kinect_interface_primesense/open_ni_funcs.h"
#include "math/math_types.h"
#include "data_str/vector.h"
#include "clk/clk.h"
#include "file_io/file_io.h"
#include "string_util/string_util.h"
#include "threading/thread_pool.h"
#include "math/icp.h"
#include "image_util/image_util.h"

#if defined(WIN32) || defined(_WIN32)  
  #define snprintf _snprintf_s
  #pragma warning( disable : 4099 )
#endif
#define SAFE_DELETE(x) if (x != NULL) { delete x; x = NULL; }
#define SAFE_DELETE_ARR(x) if (x != NULL) { delete[] x; x = NULL; }

// KINECT DATA
//#define IM_DIR_BASE string("data/hand_depth_data_2013_01_11_1/")  // Fit
//#define IM_DIR_BASE string("data/hand_depth_data_2013_01_11_2_1/")  // Fit
//#define IM_DIR_BASE string("data/hand_depth_data_2013_01_11_2_2/")  // Fit
//#define IM_DIR_BASE string("data/hand_depth_data_2013_01_11_3/")  // Fit
//#define IM_DIR_BASE string("data/hand_depth_data_2013_03_04_4/")  // Fit
//#define IM_DIR_BASE string("data/hand_depth_data_2013_03_04_5/")  // Fit
//#define IM_DIR_BASE string("data/hand_depth_data_2013_03_04_6/")  // Fit
//#define IM_DIR_BASE string("data/hand_depth_data_2013_03_04_7/")  // Fit (Tr-data)

// PRIMESENSE DATA
#define BACKUP_HDD
//#define IM_DIR_BASE string("hand_depth_data/")
//#define IM_DIR_BASE string("hand_depth_data_2013_05_01_1/")  // Cal + Fit (5405)
//#define IM_DIR_BASE string("hand_depth_data_2013_05_03_1/")  // Cal + Fit (6533)
//#define IM_DIR_BASE string("hand_depth_data_2013_05_06_1/")  // Cal + Fit (8709)
//#define IM_DIR_BASE string("hand_depth_data_2013_05_06_2/")  // Cal + Fit (8469)
//#define IM_DIR_BASE string("hand_depth_data_2013_05_06_3/")  // Cal + Fit (5815) MURPHY
#define IM_DIR_BASE string("hand_depth_data_2013_05_08_1/")  // Cal + Fit (2440) (Te-data)
//#define IM_DIR_BASE string("hand_depth_data_2013_05_19_1/")  // Cal + Fit (5969)
//#define IM_DIR_BASE string("hand_depth_data_2013_05_19_2/")  // Cal + Fit (6781)
//#define IM_DIR_BASE string("hand_depth_data_2013_06_15_1/")  // Cal + Fit (3049)
//#define IM_DIR_BASE string("hand_depth_data_2013_06_15_2/")  // Cal + Fit (7676)
//#define IM_DIR_BASE string("hand_depth_data_2013_06_15_3/")  // Cal + Fit (4935)
//#define IM_DIR_BASE string("hand_depth_data_2013_06_15_4/")  // Cal + Fit (9752)
//#define IM_DIR_BASE string("hand_depth_data_2013_06_15_5/")  //Cal + Fit (5480)  Total: 81013

//#define KINECT_DATA  // Otherwise Primesense 1.09 data
#define MAX_KINECTS 3
#define NUM_WORKER_THREADS 6

#if defined(__APPLE__)
  #error "Apple is not yet supported!"
#else
#ifdef BACKUP_HDD
    //#define KINECT_HANDS_ROOT string("F:/hand_data/")
    #define KINECT_HANDS_ROOT string("G:/hand_data/")
  #else
    #define KINECT_HANDS_ROOT string("./../data/")
  #endif
  #define FOREST_ROOT string("./../")
#endif

#ifndef HAND_FIT
  #error "HAND_FIT is not defined in the preprocessor definitions!"
#endif

//#define IM_DIR (KINECT_HANDS_ROOT + IM_DIR_BASE)
#define IM_DIR (KINECT_HANDS_ROOT + IM_DIR_BASE)

// #define LOAD_AND_SAVE_OLD_FORMAT_COEFFS
const bool fit_left = false;
const bool fit_right = true; 

using namespace std;
using namespace jtil::math;
using namespace jtil::data_str;
using namespace jtil::file_io;
using namespace jtil::image_util;
using namespace jtil::threading;
using namespace model_fit;
using namespace renderer;
using namespace jtil::windowing;
using namespace kinect_interface_primesense;
using namespace kinect_interface_primesense::hand_net;

jtil::clk::Clk* clk = NULL;
double t1, t0;

// The main window and basic rendering system
Window* wnd = NULL;
Renderer* render = NULL;
bool rotate_light = false;
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
Float4x4 camera_view[MAX_KINECTS];
PoseModel** models;
int cur_kinect = 0;
int cur_icp_dst_kinect = 0;
uint32_t cur_icp_mat = 0;
int32_t last_icp_kinect = -1;
bool render_correspondances = true;
HandModelCoeff** l_hand_coeffs = NULL;  // Left Hand coefficients
HandModelCoeff** r_hand_coeffs = NULL;  // Right hand coeffs
const uint32_t num_models = (fit_left ? 1 : 0) + (fit_right ? 1 : 0);
int hand_to_modify = fit_left ? 0 : 1;
const uint32_t num_coeff = HandCoeff::NUM_PARAMETERS;
const uint32_t num_coeff_fit = HAND_NUM_COEFF;
uint32_t num_model_fit_cameras = MAX_KINECTS;
uint32_t cur_coeff = 0;
ModelFit* fit = NULL;
bool continuous_fit = false;  // fit frames continuously each frame
bool continuous_play = false;  // Play back recorded frames
bool render_models = true;
uint32_t coeff_src = 0;
float** coeff = NULL;  // Temp space only used when performing fit
float** prev_coeff = NULL; 

// Kinect Image data 
Vector<Triple<char*, int64_t, int64_t>> im_files[MAX_KINECTS];  // filename, kinect time, global time
float cur_xyz_data[MAX_KINECTS][src_dim*3];
float cur_norm_data[MAX_KINECTS][src_dim*3];
float cur_uvd_data[MAX_KINECTS][src_dim*3];
int16_t** cur_depth_data;  // Size: [MAX_KINECTS][src_dim*3]
uint8_t** cur_label_data;  // Size: [MAX_KINECTS][src_dim]
uint8_t cur_image_rgb[MAX_KINECTS][src_dim*3];
uint32_t cur_image = 0;
GeometryColoredPoints* geometry_points[MAX_KINECTS];
GeometryColoredLines* geometry_lines[MAX_KINECTS-1];  // For displaying ICP correspondances
float temp_xyz[3 * src_dim];
float temp_rgb[3 * src_dim];
bool render_depth = true;
int playback_step = 1;
OpenNIFuncs openni_funcs;
Texture* tex = NULL;
uint8_t tex_data[src_dim * 3];
const bool color_point_clouds = false;
const float point_cloud_scale = 4.0f;
const uint32_t num_point_clouds_to_render = 1;

// ICP
jtil::math::ICP icp;

// Multithreading
ThreadPool* tp;

using std::cout;
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
  if (cur_depth_data) {
    for (uint32_t i = 0; i < MAX_KINECTS; i++) {
      SAFE_DELETE_ARR(cur_depth_data[i]);
    }
  }
  SAFE_DELETE_ARR(cur_depth_data);
  if (cur_label_data) {
    for (uint32_t i = 0; i < MAX_KINECTS; i++) {
      SAFE_DELETE_ARR(cur_label_data[i]);
    }
  }
  SAFE_DELETE_ARR(cur_label_data);
  if (l_hand_coeffs) {
    for (uint32_t i = 0; i < im_files[0].size(); i++) { 
      delete l_hand_coeffs[i];
    }
    delete[] l_hand_coeffs;
  }
  if (r_hand_coeffs) {
    for (uint32_t i = 0; i < im_files[0].size(); i++) {
      delete r_hand_coeffs[i];
    }
    delete[] r_hand_coeffs;
  }
  SAFE_DELETE(render);
  SAFE_DELETE(fit);
  for (uint32_t k = 0; k < MAX_KINECTS; k++) {
    SAFE_DELETE(geometry_points[k]);
    for (uint32_t i = 0; i < im_files[k].size(); i++) {
      SAFE_DELETE_ARR(im_files[k][i].first);
    }
  }
  Texture::shutdownTextureSystem();
  GLState::shutdownGLState();
  SAFE_DELETE(wnd);
  Window::killWindowSystem();
  exit(0);
}

uint32_t findClosestFrame(const uint32_t i_kinect) {
  if (i_kinect == 0) {
    return cur_image;
  }
  int64_t src_timestamp = im_files[0][cur_image].second;
  int32_t i_start = 0;
  int32_t i_end = (int32_t)im_files[i_kinect].size();
  int32_t frame = i_start;
  int64_t min_delta_t = std::abs(src_timestamp - im_files[i_kinect][frame].second);
  for (int32_t i = i_start + 1; i < i_end; i++) {
    int64_t delta_t = std::abs(src_timestamp - im_files[i_kinect][i].second);
    if (delta_t < min_delta_t) {
      min_delta_t = delta_t;
      frame = i;
    }
  }
  return (uint32_t)frame;
}

void loadCurrentImage(bool print_to_screen = true) {
  char* file = im_files[0][cur_image].first;
  string full_filename = IM_DIR + string(file);
  if (print_to_screen) {
    std::cout << "loading image: " << full_filename << std::endl;
    std::cout << "cur_image = " << cur_image << " of ";
    std::cout << im_files[0].size() << std::endl;
  }
  // Now load the other Kinect data
  for (uint32_t k = 0; k < MAX_KINECTS; k++) {
    if (im_files[k].size() == 0) {
      for (uint32_t j = 0; j < src_dim; j++) {
        cur_depth_data[k][j] = GDT_MAX_DIST;
      }
      memset(cur_label_data[k], 0, sizeof(cur_label_data[k][0]) * src_dim); 
      memset(cur_image_rgb[k], 0, sizeof(cur_image_rgb[k][0]) * src_dim * 3); 
    } else {
      // find the correct file (with smallest timestamp difference) - O(60)
      uint32_t i_match = findClosestFrame(k);
      // Now we've found the correct file, load it
      file = im_files[k][i_match].first;
      full_filename = IM_DIR + string(file);
      if (!continuous_play || (cur_image % 30) == 0) {
        std::cout << "loading image (frame " << cur_image << " of " << 
          im_files[0].size() << "): " << full_filename << std::endl;
      }
      image_io->LoadCompressedImage(full_filename, 
        cur_depth_data[k], cur_label_data[k], cur_image_rgb[k]);
      memset(cur_label_data[k], 0, src_dim * sizeof(cur_label_data[k][0]));
      openni_funcs.ConvertDepthImageToProjective((uint16_t*)cur_depth_data[k], 
        cur_uvd_data[k]);
      openni_funcs.convertDepthToWorldCoordinates(cur_uvd_data[k], cur_xyz_data[k], 
        src_dim);
    }
  }
}

void InitXYZPointsForRendering() {
  for (uint32_t k = 0; k < MAX_KINECTS; k++) {
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
#if defined(CALIBRATION_RUN)
  // Save both kinect coeffs in the same file
  string filename = IM_DIR + string("coeff_") + im_files[0][cur_image].first;
  float calb_data[MAX_KINECTS * num_coeff];
  for (uint32_t i = 0; i < MAX_KINECTS; i++) {
    memcpy(&calb_data[i * num_coeff], coeffs[i][cur_image], 
      sizeof(*calb_data) * num_coeff);
  }
  SaveArrayToFile<float>(calb_data, MAX_KINECTS * num_coeff, filename);
  cout << "hand data saved to file" << endl;
#else
  string r_hand_file = string("coeffr_") + im_files[0][cur_image].first;
  string l_hand_file = string("coeffl_") + im_files[0][cur_image].first;
  if (fit_right) {
    r_hand_coeffs[cur_image]->saveToFile(IM_DIR, r_hand_file);
  } else {
    r_hand_coeffs[cur_image]->saveBlankFile(IM_DIR, r_hand_file);
  }
  if (fit_left) {
    l_hand_coeffs[cur_image]->saveToFile(IM_DIR, l_hand_file);
  } else {
    l_hand_coeffs[cur_image]->saveBlankFile(IM_DIR, l_hand_file);
  }
  cout << "hand data saved to file" << endl;
#endif
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
    std::cout << "rot = " << std::endl;
    render->camera()->eye_rot()->print();
  }
  if (scale_coeff) {
    int dy = mouse_y - mouse_y_prev;
    float theta_y = dy * mouse_speed_rotation;
    if (/*cur_coeff >= 0 &&*/ cur_coeff <= 2) {
      theta_y *= 50.0f;
    }
    float coeff_val;
    if (cur_coeff == HandCoeff::SCALE) {
      // We must set both scales at once
      coeff_val = l_hand_coeffs[cur_image]->getCoeff(cur_coeff);
      l_hand_coeffs[cur_image]->setCoeff(cur_coeff, coeff_val - theta_y);
      r_hand_coeffs[cur_image]->setCoeff(cur_coeff, coeff_val - theta_y);
    } else if (hand_to_modify == 0) {
      coeff_val = l_hand_coeffs[cur_image]->getCoeff(cur_coeff);
      l_hand_coeffs[cur_image]->setCoeff(cur_coeff, coeff_val - theta_y);
    } else {
      coeff_val = r_hand_coeffs[cur_image]->getCoeff(cur_coeff);
      r_hand_coeffs[cur_image]->setCoeff(cur_coeff, coeff_val - theta_y);
    }
    cout << "cur_coeff " << kinect_interface_primesense::hand_net::HandCoeffToString(cur_coeff);
    cout << " --> " << coeff_val - theta_y << endl;
  }
  if (middle_down) {
    int dy = mouse_y - mouse_y_prev;
    float theta_y = dy * mouse_speed_rotation;
    Float3 trans(0, 0, 0);
    trans[cur_coeff % 3] -= theta_y;
    camera_view[cur_kinect].leftMultTranslation(trans);

    // Now save the results to file
    std::stringstream ss;
    ss << IM_DIR << "calibration_data" << cur_kinect << ".bin";
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
    case static_cast<int>('r'):
    case static_cast<int>('R'):
      if (action == RELEASED) {
        rotate_light = !rotate_light;
      }
      break;
    case static_cast<int>('t'):
    case static_cast<int>('T'):
      if (action == RELEASED) {
        render->wireframe = !render->wireframe;
      }
      break;
    case static_cast<int>('b'):
    case static_cast<int>('B'):
      if (action == RELEASED) {
        render->render_bounding_spheres = !render->render_bounding_spheres;
      }
      break;
    case static_cast<int>('1'):
    case static_cast<int>('2'):
    case static_cast<int>('3'):
    case static_cast<int>('4'):
    case static_cast<int>('5'):
    case static_cast<int>('6'):
    case static_cast<int>('7'):
      if (action == RELEASED && !shift_down) {
        render_output = key - static_cast<int>('1') + 1;
      }
      if (action == RELEASED && shift_down && cur_image > 0) {
        HandModelCoeff* src;
        HandModelCoeff* dst;
        switch (key) {
          case static_cast<int>('5'):
            std::cout << "copying thumb from previous frame..." << std::endl;
            if (fit_left) {
              src = l_hand_coeffs[cur_image-1];
              dst = l_hand_coeffs[cur_image];
            }
            if (fit_right) {
              src = r_hand_coeffs[cur_image-1];
              dst = r_hand_coeffs[cur_image];
            }
            dst->setCoeff(HandCoeff::THUMB_THETA, 
              src->getCoeff(HandCoeff::THUMB_THETA));
            dst->setCoeff(HandCoeff::THUMB_PHI, 
              src->getCoeff(HandCoeff::THUMB_PHI));
            dst->setCoeff(HandCoeff::THUMB_K1_PHI, 
              src->getCoeff(HandCoeff::THUMB_K1_PHI));
            dst->setCoeff(HandCoeff::THUMB_K1_THETA, 
              src->getCoeff(HandCoeff::THUMB_K1_THETA));
            dst->setCoeff(HandCoeff::THUMB_K2_PHI, 
              src->getCoeff(HandCoeff::THUMB_K2_PHI));
            dst->setCoeff(HandCoeff::THUMB_TWIST, 
              src->getCoeff(HandCoeff::THUMB_TWIST));
            dst->setCoeff(HandCoeff::THUMB_LENGTH, 
              src->getCoeff(HandCoeff::THUMB_LENGTH));
            break;
          case static_cast<int>('1'):
          case static_cast<int>('2'):
          case static_cast<int>('3'):
          case static_cast<int>('4'):
            uint32_t finger = (int)key - (int)'1';
            uint32_t off = FINGER_NUM_COEFF * finger;
            std::cout << "copying finger " << finger;
            std::cout << " from previous frame..." << std::endl;
            if (fit_left) {
              src = l_hand_coeffs[cur_image-1];
              dst = l_hand_coeffs[cur_image];
            }
            if (fit_right) {
              src = r_hand_coeffs[cur_image-1];
              dst = r_hand_coeffs[cur_image];
            }
            dst->setCoeff(HandCoeff::F0_ROOT_PHI + off, 
              src->getCoeff(HandCoeff::F0_ROOT_PHI + off));
            dst->setCoeff(HandCoeff::F0_ROOT_THETA + off, 
              src->getCoeff(HandCoeff::F0_ROOT_THETA + off));
            dst->setCoeff(HandCoeff::F0_PHI + off, 
              src->getCoeff(HandCoeff::F0_PHI + off));
            dst->setCoeff(HandCoeff::F0_THETA + off, 
              src->getCoeff(HandCoeff::F0_THETA + off));
            dst->setCoeff(HandCoeff::F0_KNUCKLE_MID + off, 
              src->getCoeff(HandCoeff::F0_KNUCKLE_MID + off));
            dst->setCoeff(HandCoeff::F0_KNUCKLE_END + off, 
              src->getCoeff(HandCoeff::F0_KNUCKLE_END + off));
            dst->setCoeff(HandCoeff::F0_TWIST + finger, 
              src->getCoeff(HandCoeff::F0_TWIST + finger));
            dst->setCoeff(HandCoeff::F0_LENGTH + finger, 
              src->getCoeff(HandCoeff::F0_LENGTH + finger));
            break;
        }
        saveCurrentCoeffs();
      }
      break;
    case KEY_KP_ADD:
      if (action == RELEASED) {
        cur_image = cur_image < static_cast<uint32_t>(im_files[0].size())-1 ? 
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
    case static_cast<int>('0'):
      if (action == RELEASED) {
        cur_image = cur_image + 100;
        if (cur_image >= im_files[0].size()) {
          cur_image = im_files[0].size()-1;
        }
        loadCurrentImage();
        InitXYZPointsForRendering();
      }
      break;
    case static_cast<int>('9'):
      if (action == RELEASED) {
        cur_image = cur_image >= 100 ? cur_image-100 : 0;
        loadCurrentImage();
        InitXYZPointsForRendering();
      }
      break;
    case static_cast<int>('}'):
    case static_cast<int>(']'):
      if (action == RELEASED) {
        cur_coeff = (cur_coeff + 1) % num_coeff;
        cout << "cur_coeff = " << kinect_interface_primesense::hand_net::HandCoeffToString(cur_coeff); 
        if (hand_to_modify == 0) {
          cout << " = " << l_hand_coeffs[cur_image]->getCoeff(cur_coeff);
        } else {
          cout << " = " << r_hand_coeffs[cur_image]->getCoeff(cur_coeff);
        }
        cout << std::endl;
      }
      break;
    case static_cast<int>('{'):
    case static_cast<int>('['):
      if (action == RELEASED) {
        cur_coeff = cur_coeff != 0 ? (cur_coeff - 1) : num_coeff - 1;
        cout << "cur_coeff = " << kinect_interface_primesense::hand_net::HandCoeffToString(cur_coeff); 
        if (hand_to_modify == 0) {
          cout << " = " << l_hand_coeffs[cur_image]->getCoeff(cur_coeff);
        } else {
          cout << " = " << r_hand_coeffs[cur_image]->getCoeff(cur_coeff);
        }
        cout << std::endl;
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
        if (shift_down) {

        } else {
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
    case static_cast<int>('o'):
    case static_cast<int>('O'): 
      if (action == RELEASED) {
        playback_step = playback_step + 1;
        playback_step = playback_step == 11 ? 1 : playback_step;
        cout << "playback_step = " << playback_step << endl;
      }
      break;
    case static_cast<int>('z'):
    case static_cast<int>('Z'): 
      if (action == RELEASED) {
        if (icp.getTransforms().size() > 0) {
          cur_icp_mat = (cur_icp_mat + 1) % icp.getTransforms().size();
        }
        std::cout << "cur_icp_mat = " << cur_icp_mat << std::endl;
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
#ifdef CALIBRATION_RUN
        render_all_views = !render_all_views;
        cout << "render_all_views = " << render_all_views << endl;
#endif
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
          cur_kinect = (cur_kinect + 1) % MAX_KINECTS;
          cout << "cur_kinect = " << cur_kinect << endl;
        } else {
          cur_icp_dst_kinect = (cur_icp_dst_kinect + 1) % MAX_KINECTS;
          cout << "cur_icp_dst_kinect = " << cur_icp_dst_kinect << endl;
        }
      }
      break;
    case KEY_SPACE:
      if (action == RELEASED) {
#ifndef CALIBRATION_RUN
        if (fit_left && fit_right) {
          hand_to_modify = (hand_to_modify + 1) % 2;
        } else if (fit_left) {
          hand_to_modify = 0;
        } else {
          hand_to_modify = 1;
        }
        if (hand_to_modify == 0) {
          cout << "Adjusting LEFT HAND" << endl;
        } else {
          cout << "Adjusting RIGHT HAND" << endl;
        }
#endif
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
    case static_cast<int>('u'):
    case static_cast<int>('U'):
      if (action == RELEASED) {
        render_depth = !render_depth;
      }
      break;
    case static_cast<int>('j'):
    case static_cast<int>('J'):
      if (action == RELEASED) {
        fitFrame(false, true);
      }
      break;
    case static_cast<int>('k'):
    case static_cast<int>('K'):
#if defined(WIN32) || defined(_WIN32)
      if (action == RELEASED) {
        if (shift_down) {
          repeat = 10;
        }
        {  // Closure
          for (int i = 0; i < repeat; i++) {
            // We only need to mark the first kinect's file as deleted...
            std::string full_im_filename = IM_DIR + 
              (im_files[0][cur_image].first);
            std::string new_full_im_filename = IM_DIR + string("deleted_") + 
              string(im_files[0][cur_image].first);
            r_coeff_file = IM_DIR + string("coeffr_") + im_files[0][cur_image].first;
            new_r_coeff_file = IM_DIR + string("deleted_coeffr_") + 
              im_files[0][cur_image].first;
            l_coeff_file = IM_DIR + string("coeffl_") + im_files[0][cur_image].first;
            new_l_coeff_file = IM_DIR + string("deleted_coeffl_") + 
              im_files[0][cur_image].first;

            bool move_OK = (bool)
              MoveFileW(jtil::string_util::ToWideString(full_im_filename).c_str(), 
              jtil::string_util::ToWideString(new_full_im_filename).c_str());
            if (!move_OK) {
              cout << "Error moving files: " << endl;
            } else {
              cout << "Image file marked as deleted sucessfully: " << endl;
              delete r_hand_coeffs[cur_image]; 
              delete l_hand_coeffs[cur_image];
              for (uint32_t i = cur_image; i < im_files[0].size() - 1; i++) {
                r_hand_coeffs[i] = r_hand_coeffs[i+1];
                l_hand_coeffs[i] = l_hand_coeffs[i+1];
              }
              SAFE_DELETE(im_files[0][cur_image].first); 
              im_files[0].deleteAtAndShift(cur_image);

              // Coeff file may not exist yet (if we haven't fit it)...
              move_OK = 
                MoveFileW(jtil::string_util::ToWideString(r_coeff_file).c_str(), 
                jtil::string_util::ToWideString(new_r_coeff_file).c_str()) &&
                MoveFileW(jtil::string_util::ToWideString(l_coeff_file).c_str(), 
                jtil::string_util::ToWideString(new_l_coeff_file).c_str());
            }
            cout << "    - " << full_im_filename.c_str() << " to " << endl;
            cout << "      " << new_full_im_filename.c_str() << endl;
            cout << "    - " << r_coeff_file.c_str() << endl;
            cout << "      " << new_r_coeff_file.c_str() << endl;
            cout << "    - " << l_coeff_file.c_str() << endl;
            cout << "      " << new_l_coeff_file.c_str() << endl;
            cout << endl;
          }  // for (int i = 0; i < repeat; i++) {
          loadCurrentImage();
          InitXYZPointsForRendering();
        }  // Closure
      }
#else
      cout << "Move function not implemented for Mac OS X" << endl;
#endif
      break;
  }
}

void fitFrame(bool seed_with_last_frame, bool query_only) {
  // if query_only = true then we'll save the images to file (for the paper)
  if (seed_with_last_frame && cur_image > 0) {
    cout << "Using the previous frame as the optimization seed." << endl;
    if (fit_right) {
      r_hand_coeffs[cur_image]->copyCoeffFrom(r_hand_coeffs[cur_image - 1]);
    }
    if (fit_left) {
      l_hand_coeffs[cur_image]->copyCoeffFrom(l_hand_coeffs[cur_image - 1]);
    }
  }

  if (fit_left && !fit_right) {
    coeff[0] = l_hand_coeffs[cur_image]->coeff();
  } else if (!fit_left && fit_right) {
    coeff[0] = r_hand_coeffs[cur_image]->coeff();
  } else {
    coeff[0] = l_hand_coeffs[cur_image]->coeff();
    coeff[1] = r_hand_coeffs[cur_image]->coeff();
  }
  if (cur_image > 0) {
    if (fit_left && !fit_right) {
      prev_coeff[0] = l_hand_coeffs[cur_image - 1]->coeff();
    } else if (!fit_left && fit_right) {
      prev_coeff[0] = r_hand_coeffs[cur_image - 1]->coeff();
    } else {
      prev_coeff[0] = l_hand_coeffs[cur_image - 1]->coeff();
      prev_coeff[1] = r_hand_coeffs[cur_image - 1]->coeff();
    }
  }
  HandGeometryMesh::setCurrentStaticHandProperties(coeff[0]);
  if (query_only) {
    fit->queryObjFunc(cur_depth_data, cur_label_data, models, coeff);
  } else {
    if (cur_image > 0) {
      fit->fitModel(cur_depth_data, cur_label_data, models, coeff, 
        prev_coeff, HandModelCoeff::renormalizeCoeffs);
    } else {
      fit->fitModel(cur_depth_data, cur_label_data, models, coeff, 
        NULL, HandModelCoeff::renormalizeCoeffs);
    }
  }
}

void renderFrame(float dt) {
  if (rotate_light) {
    renderer::LightDir* light = render->light_dir();
    Float3* dir = light->direction_world();
    Float4x4::rotateMatYAxis(mat_tmp, dt);
    Float3 new_dir;
    Float3::affineTransformVec(new_dir, mat_tmp, *dir);
    dir->set(new_dir);
  }

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
    std::cout << "camera position: " << std::endl;
    render->camera()->eye_pos()->print();
  }

  float* cur_coeff;
  if (fit_right && fit_left) {
    cur_coeff = l_hand_coeffs[cur_image]->coeff();
    models[0]->updateMatrices(l_hand_coeffs[cur_image]->coeff());
    models[1]->updateMatrices(r_hand_coeffs[cur_image]->coeff());
  }
  if (fit_right) {
    cur_coeff = r_hand_coeffs[cur_image]->coeff();
    models[0]->updateMatrices(r_hand_coeffs[cur_image]->coeff());
  } else if (fit_left) {
    cur_coeff = l_hand_coeffs[cur_image]->coeff();
    models[0]->updateMatrices(l_hand_coeffs[cur_image]->coeff());
  }
  HandGeometryMesh::setCurrentStaticHandProperties(cur_coeff);

  // Now render the final frame
  Float4x4 identity;
  identity.identity();
  switch (render_output) {
  case 1:
    render->renderFrame(dt);
    if (render_depth) {
      for (uint32_t k = 0; k < std::min<uint32_t>(MAX_KINECTS, 
        num_point_clouds_to_render); k++) {
        if (k == last_icp_kinect && icp.getTransforms().size() > 0) {
          render->renderColoredPointCloud(geometry_points[k], 
            &icp.getTransforms()[cur_icp_mat], 
            point_cloud_scale * 1.5f * static_cast<float>(settings.width) / 4.0f);
        } else {
          render->renderColoredPointCloud(geometry_points[k], 
            &camera_view[k], 
            point_cloud_scale * 1.5f * static_cast<float>(settings.width) / 4.0f);
        }
      }
      if (render_correspondances) {
        for (uint32_t k = 0; k < MAX_KINECTS - 1; k++) {
          if (geometry_lines[k] != NULL) {
            render->renderColoredLines(geometry_lines[k], &identity, 4.0f);
          }
        }
      }
    }
    break;
  case 2:
    float coeff[num_models * num_coeff_fit];
    if (fit_left && !fit_right) {
      memcpy(coeff, l_hand_coeffs[cur_image]->coeff(), sizeof(coeff[0])*num_coeff_fit);
    } else if (!fit_left && fit_right) {
      memcpy(coeff, r_hand_coeffs[cur_image]->coeff(), sizeof(coeff[0])*num_coeff_fit);
    } else {
      memcpy(coeff, l_hand_coeffs[cur_image]->coeff(), sizeof(coeff[0])*num_coeff_fit);
      memcpy(&coeff[num_coeff_fit], r_hand_coeffs[cur_image]->coeff(), 
        sizeof(coeff[0])*num_coeff_fit);
    }

    fit->model_renderer()->drawDepthMap(coeff, num_coeff_fit, models,
      num_models, cur_kinect, false);
    fit->model_renderer()->visualizeDepthMap(wnd, cur_kinect);
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
  // _CrtSetBreakAlloc(2997);
#endif

  cout << "Usage:" << endl;
  cout << "WSADQE - Move camera" << endl;
  cout << "shift - Sprint" << endl;
  cout << "mouse left click + drag - Rotate camera" << endl;
  cout << "space - Change hand to control" << endl;
  cout << "mouse right click + drag - Adjust coefficient" << endl;
  cout << "[] - Change adjustment coeff" << endl;
  cout << "r - rotate light" << endl;
  cout << "t - wireframe rendering" << endl;
  cout << "b - bounding sphere rendering" << endl;
  cout << "y - Render Hands ON/OFF" << endl;
  cout << "u - Render Point Cloud ON/OFF" << endl;
  cout << "12 - Render output type" << endl;
  cout << "+- - Change the current depth image" << endl;
  cout << "09 - Change the current depth image x 100" << endl;
  cout << "h - Store hand data to file" << endl;
  cout << "f - Fit model to current frame" << endl;
  cout << "g - Fit model to all remaining frames" << endl;
  cout << "p - Playback frames (@15fps)" << endl;
  cout << "P - Perform targeted zoom" << endl;
  cout << "o - Change playback frame skip" << endl;
  cout << "l - Go to start frame" << endl;
  cout << "n - Open video stream (playback will save to stream)" << endl;
  cout << "c - Save calibration data (calibration mode only)" << endl;
  cout << "x - Render all views (calibration mode only)" << endl;
  cout << "z - Step through ICP frames (if ICP has been run) (calibration mode only)" << endl;
  cout << "v - Render Correspondances (if ICP has been run) (calibration mode only)" << endl;
  cout << "j - Query Objective Function Value" << endl;
  cout << "shift+12345 - Copy finger1234/thumb from last frame" << endl;
  cout << "k - (3 times) delete current file" << endl;
  cout << "i - Change the current kinect (for ICP to move onto dst)" << endl;
  cout << "I - Change the current dst kinect" << endl << endl;
  
  try {
    tp = new ThreadPool(NUM_WORKER_THREADS);

    clk = new jtil::clk::Clk();
    t1 = clk->getTime();
    
    // Initialize Windowing system
    Window::initWindowSystem();
    Texture::initTextureSystem();
    
    // Fill the settings structure
    settings.width = src_width*2;
    settings.height = src_height*2;
    //settings.width = 1280;
    //settings.height = 720;
    settings.fullscreen = false;
    settings.title = string("Hand Fit Project");
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
    for (uint32_t k = 0; k < MAX_KINECTS; k++) {
      geometry_points[k] = new GeometryColoredPoints;
      geometry_points[k]->vertices()->capacity(src_dim);
      geometry_points[k]->vertices()->resize(src_dim);
      geometry_points[k]->colors()->capacity(src_dim);
      geometry_points[k]->colors()->resize(src_dim);
    }
    for (uint32_t k = 0; k < MAX_KINECTS-1; k++) {
      geometry_lines[k] = NULL;
    }
 
    // Load the Kinect data for fitting from file and process it
    image_io = new DepthImagesIO();
    for (uint32_t k = 0; k < MAX_KINECTS; k++) {
      image_io->GetFilesInDirectory(im_files[k], IM_DIR, k);
      if (im_files[k].size() == 0) {
        throw std::runtime_error("ERROR: No frames exist for one of the "
          "sensors!");
      }
      std::stringstream ss;
      ss << IM_DIR << "calibration_data" << k << ".bin";
      if (!fileExists(ss.str())) {
        camera_view[k].identity();
      } else {
        LoadArrayFromFile<float>(camera_view[k].m, 16, ss.str());
      }
    }
    image_io->AlignKinects(im_files, MAX_KINECTS);

    cur_depth_data = new int16_t*[MAX_KINECTS];
    cur_label_data = new uint8_t*[MAX_KINECTS];
    for (uint32_t k = 0; k < MAX_KINECTS; k++) {
      cur_depth_data[k] = new int16_t[src_dim * 3];
      cur_label_data[k] = new uint8_t[src_dim];
    }

    loadCurrentImage();
  
    // Attach callback functions for event handling
    wnd->registerKeyboardCB(&KeyboardCB);
    wnd->registerMousePosCB(&MousePosCB);
    wnd->registerMouseButCB(&MouseButtonCB);
    wnd->registerMouseWheelCB(NULL);

    // Creathand_renderere instances of the models to fit
    coeff = new float*[num_models];
    prev_coeff = new float*[num_models];
    models = new PoseModel*[num_models];

    if (fit_left && fit_right) {
      models[0] = new HandGeometryMesh(LEFT);
      models[1] = new HandGeometryMesh(RIGHT);
    } else if (fit_left) {
      models[0] = new HandGeometryMesh(LEFT);
    } else if (fit_right) {
      models[0] = new HandGeometryMesh(RIGHT);
    }

    // Create the optimizer that will fit the models
    fit = new ModelFit(num_models, num_coeff_fit, num_model_fit_cameras);

    Float4x4 old_view, cur_view, camera_view_inv;
    for (uint32_t k = 0; k < num_model_fit_cameras; k++) {
      fit->getCameraView(k, old_view);
      Float4x4::inverse(camera_view_inv, camera_view[k]);
      Float4x4::mult(cur_view, old_view, camera_view_inv);
      fit->setCameraView(k, cur_view);
    }

    // Load the coeffs from file
    r_hand_coeffs = new HandModelCoeff*[im_files[0].size()];
    l_hand_coeffs = new HandModelCoeff*[im_files[0].size()];
    for (uint32_t i = 0; i < im_files[0].size(); i++) {
      r_hand_coeffs[i] = new HandModelCoeff(kinect_interface_primesense::hand_net::HandType::RIGHT);
      r_hand_coeffs[i]->loadFromFile(IM_DIR, string("coeffr_") + im_files[0][i].first);
      l_hand_coeffs[i] = new HandModelCoeff(kinect_interface_primesense::hand_net::HandType::LEFT);
      l_hand_coeffs[i]->loadFromFile(IM_DIR, string("coeffl_") + im_files[0][i].first);
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
        if (cur_image < im_files[0].size() - 1) {
          cout << "fitting frame " << cur_image + 1 << " of ";
          cout << im_files[0].size() << endl;
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
        if (cur_image < im_files[0].size() - playback_step) {
          continuous_play_timer_start += (t1 - t0);
          if (continuous_play_timer_start >= continuous_play_frame_time) {
            cur_image += playback_step;
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
#if defined(WIN32) || defined(_WIN32)
    system("PAUSE");
#endif
  }
  
  return 0;
}
