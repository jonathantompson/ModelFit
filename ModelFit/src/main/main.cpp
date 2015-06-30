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

static const int max_kinects = 3;
static const int num_worker_threads = 6;
static const char calib_im_dir[] = "../data/calib/";
static const char im_dir[] = "../data/hand_data/";

#ifndef HAND_FIT
  #error "HAND_FIT is not defined in the preprocessor definitions!"
#endif

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
Float4x4 camera_view[max_kinects];
PoseModel** models;
int cur_kinect = 0;
jtil::data_str::VectorManaged<HandModelCoeff*> l_hand_coeffs;  // Left Hand coefficients
jtil::data_str::VectorManaged<HandModelCoeff*> r_hand_coeffs;   // Right hand coeffs
const uint32_t num_models = (fit_left ? 1 : 0) + (fit_right ? 1 : 0);
int hand_to_modify = fit_left ? 0 : 1;
const uint32_t num_coeff = HandCoeff::NUM_PARAMETERS;
const uint32_t num_coeff_fit = HAND_NUM_COEFF;
uint32_t cur_coeff = 0;
ModelFit* fit = NULL;
bool continuous_fit = false;  // fit frames continuously each frame
bool continuous_play = false;  // Play back recorded frames
bool render_models = true;
uint32_t coeff_src = 0;
float** coeff = NULL;  // Temp space only used when performing fit
float** prev_coeff = NULL; 

// Kinect Image data 
jtil::data_str::VectorManaged<char*> depth_files[max_kinects];  // [kinect][frame]
jtil::data_str::VectorManaged<char*> rgb_files[max_kinects];  
float cur_xyz_data[max_kinects][src_dim*3];
float cur_norm_data[max_kinects][src_dim*3];
float cur_uvd_data[max_kinects][src_dim*3];
int16_t** cur_depth_data;  // Size: [max_kinects][src_dim*3]
uint8_t** cur_label_data;  // Size: [max_kinects][src_dim]
uint8_t cur_image_rgb[max_kinects][src_dim*3];
uint32_t cur_image = 0;
GeometryColoredPoints* geometry_points[max_kinects];
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
  SAFE_DELETE(render);
  SAFE_DELETE(fit);
 if (cur_depth_data) {
    for (uint32_t i = 0; i < max_kinects; i++) {
      SAFE_DELETE_ARR(cur_depth_data[i]);
    }
  }
  SAFE_DELETE_ARR(cur_depth_data);
  if (cur_label_data) {
    for (uint32_t i = 0; i < max_kinects; i++) {
      SAFE_DELETE_ARR(cur_label_data[i]);
    }
  }
  SAFE_DELETE_ARR(cur_label_data);
  for (uint32_t k = 0; k < max_kinects; k++) {
    SAFE_DELETE(geometry_points[k]);
  }
  Texture::shutdownTextureSystem();
  GLState::shutdownGLState();
  SAFE_DELETE(wnd);
  Window::killWindowSystem();
  exit(0);
}

void loadCurrentImage(bool print_to_screen = true) {
  char full_path[256];

  string full_filename = string(im_dir) + string(depth_files[0][cur_image]);
  if (print_to_screen) {
    std::cout << "loading image: " << full_filename << std::endl;
    std::cout << "cur_image = " << cur_image << " of ";
    std::cout << depth_files[0].size() << std::endl;
  }
  // load the Kinect data
  for (uint32_t k = 0; k < max_kinects; k++) {
    // Load the RGB data
    uint32_t w, h, nchan;
    snprintf(full_path, 255, "%s%s", im_dir, rgb_files[k][cur_image]);
    uint8_t* rgb = NULL;
    renderer::Texture::loadImFromFile(full_path, rgb, w, h, nchan);
    if (!rgb || w != src_width || h != src_height || nchan != 3) {
      throw std::runtime_error("Data might be corrupted!");
    }
    memcpy(cur_image_rgb[k], rgb, sizeof(cur_image_rgb[k][0]) * src_dim * 3);
    delete[] rgb;

    // Load the depth data
    snprintf(full_path, 255, "%s%s", im_dir, depth_files[k][cur_image]);
    renderer::Texture::loadImFromFile(full_path, rgb, w, h, nchan);
    if (!rgb || w != src_width || h != src_height || nchan != 3) {
      throw std::runtime_error("Data might be corrupted!");
    }
    // Unpack the depth (the MSB is in the green and the LSB in the red)
    int16_t* cur_depth = new int16_t[src_dim];
    for (uint32_t i = 0; i < src_dim; i++) {
      uint8_t g = rgb[i*3+1];
      uint8_t b = rgb[i*3+2];
      cur_depth_data[k][i] = (((int16_t)g) << 8) | (int16_t)b;
    }
    delete[] rgb;

    // I'm no longer using the labels, so just set them all to true, which
    // treats all points in the depth cloud as a potential hand point
    memset(cur_label_data[k], 1, sizeof(cur_label_data[k][0]) * src_dim); 

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
  char full_path[256];
  // Save the right hand coeff
  snprintf(full_path, 255, "coeffr_%07d.bin", cur_image+1);
  if (fit_right) {
    r_hand_coeffs[cur_image]->saveToFile(im_dir, full_path);
  } else {
    r_hand_coeffs[cur_image]->saveBlankFile(im_dir, full_path);
  }
  cout << "hand data saved to file: " << full_path << endl;
  // Save the left hand coeff
  snprintf(full_path, 255, "coeffl_%07d.bin", cur_image+1);
  if (fit_left) {
    l_hand_coeffs[cur_image]->saveToFile(im_dir, full_path);
  } else {
    l_hand_coeffs[cur_image]->saveBlankFile(im_dir, full_path);
  }
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
        cur_image = cur_image < static_cast<uint32_t>(depth_files[0].size())-1 ? 
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
    case static_cast<int>('h'):
    case static_cast<int>('H'):
      if (action == RELEASED) {
        saveCurrentCoeffs();
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
      for (uint32_t k = 0; k < std::min<uint32_t>(max_kinects, 
        num_point_clouds_to_render); k++) {
        render->renderColoredPointCloud(geometry_points[k], 
          &camera_view[k], 
          point_cloud_scale * 1.5f * static_cast<float>(settings.width) / 4.0f);
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
  cout << "h - Store hand data to file" << endl;
  cout << "f - Fit model to current frame" << endl;
  cout << "g - Fit model to all remaining frames" << endl;
  cout << "p - Playback frames (@15fps)" << endl;
  cout << "P - Perform targeted zoom" << endl;
  cout << "o - Change playback frame skip" << endl;
  cout << "l - Go to start frame" << endl;
  cout << "j - Query Objective Function Value" << endl;
  cout << "shift+12345 - Copy finger1234/thumb from last frame" << endl;
  
  try {
    tp = new ThreadPool(num_worker_threads);

    clk = new jtil::clk::Clk();
    t1 = clk->getTime();
    
    // Initialize Windowing system
    Window::initWindowSystem();
    Texture::initTextureSystem();
    
    // Fill the settings structure
    settings.width = src_width * 1.5;
    settings.height = src_height  *1.5;
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
    for (uint32_t k = 0; k < max_kinects; k++) {
      geometry_points[k] = new GeometryColoredPoints;
      geometry_points[k]->vertices()->capacity(src_dim);
      geometry_points[k]->vertices()->resize(src_dim);
      geometry_points[k]->colors()->capacity(src_dim);
      geometry_points[k]->colors()->resize(src_dim);
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

      snprintf(full_path, 255, "%scalibration_data%d.bin", calib_im_dir, k);
      if (!fileExists(full_path)) {
        camera_view[k].identity();
        std::cout << "**********************************" << std::endl;
        std::cout << "WARNING: CALIBRATION DATA MISSING!" << std::endl;
        std::cout << "**********************************" << std::endl;
      } else {
        LoadArrayFromFile<float>(camera_view[k].m, 16, full_path);
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


    cur_depth_data = new int16_t*[max_kinects];
    cur_label_data = new uint8_t*[max_kinects];
    for (uint32_t k = 0; k < max_kinects; k++) {
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
    fit = new ModelFit(num_models, num_coeff_fit, max_kinects);

    Float4x4 old_view, cur_view, camera_view_inv;
    for (uint32_t k = 0; k < max_kinects; k++) {
      fit->getCameraView(k, old_view);
      Float4x4::inverse(camera_view_inv, camera_view[k]);
      Float4x4::mult(cur_view, old_view, camera_view_inv);
      fit->setCameraView(k, cur_view);
    }

    // Load the coeffs from file
    for (uint32_t i = 0; i < depth_files[0].size(); i++) {
      r_hand_coeffs.pushBack(new HandModelCoeff(kinect_interface_primesense::hand_net::HandType::RIGHT));
      snprintf(full_path, 255, "coeffr_%07d.bin", i+1);
      r_hand_coeffs[i]->loadFromFile(im_dir, full_path);

      l_hand_coeffs.pushBack(new HandModelCoeff(kinect_interface_primesense::hand_net::HandType::LEFT));
      snprintf(full_path, 255, "coeffl_%07d.bin", i+1);
      l_hand_coeffs[i]->loadFromFile(im_dir, full_path);
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
        if (cur_image < depth_files[0].size() - 1) {
          cout << "fitting frame " << cur_image + 1 << " of ";
          cout << depth_files[0].size() << endl;
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
        if (cur_image < depth_files[0].size() - playback_step) {
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
