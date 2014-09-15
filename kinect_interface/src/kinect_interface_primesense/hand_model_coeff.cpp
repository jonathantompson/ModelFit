//
//  hand_model.h
//
//  Created by Jonathan Tompson on 8/17/12.
//

#include <random>
#include <thread>
#include <iostream>
#include <stdexcept>
#include <iomanip>
#include <iostream>
#include <fstream>
#include <string>
#include "kinect_interface_primesense/hand_model_coeff.h"

#define SAFE_DELETE(x) if (x != NULL) { delete x; x = NULL; }

using std::string;
using std::runtime_error;
using std::cout;
using std::endl;
using namespace jtil::math;
using namespace jtil::renderer;

namespace kinect_interface_primesense {
namespace hand_net {
 
  HandModelCoeff::HandModelCoeff(HandType hand_type) {
    resetPose();
    hand_type_ = hand_type;
  }

  HandModelCoeff::~HandModelCoeff() {

  }

  void HandModelCoeff::resetPose() {
    coeff_[HAND_POS_X] = 0.0f;
    coeff_[HAND_POS_Y] = 0.0f;
    coeff_[HAND_POS_Z] = 750.0f;
    coeff_[HAND_ORIENT_X] = 3.141592654f;  // Pi
    coeff_[HAND_ORIENT_Y] = -1.5707963f;  // -Pi / 2
    coeff_[HAND_ORIENT_Z] = 0;
    coeff_[WRIST_THETA] = 0;
    coeff_[WRIST_PHI] = 0;
    coeff_[THUMB_THETA] = 0;
    coeff_[THUMB_PHI] = 0;
    coeff_[THUMB_K1_THETA] = 0;
    coeff_[THUMB_K1_PHI] = 0;
    coeff_[THUMB_K2_PHI] = 0;
    coeff_[F0_ROOT_THETA] = 0;
    coeff_[F0_ROOT_PHI] = 0;
    coeff_[F0_THETA] = 0;
    coeff_[F0_PHI] = 0;
    coeff_[F0_KNUCKLE_MID] = 0;
    coeff_[F0_KNUCKLE_END] = 0;
    coeff_[F1_ROOT_THETA] = 0;
    coeff_[F1_ROOT_PHI] = 0;
    coeff_[F1_THETA] = 0;
    coeff_[F1_PHI] = 0;
    coeff_[F1_KNUCKLE_MID] = 0;
    coeff_[F1_KNUCKLE_END] = 0;
    coeff_[F2_ROOT_THETA] = 0;
    coeff_[F2_ROOT_PHI] = 0;
    coeff_[F2_THETA] = 0;
    coeff_[F2_PHI] = 0;
    coeff_[F2_KNUCKLE_MID] = 0;
    coeff_[F2_KNUCKLE_END] = 0;
    coeff_[F3_ROOT_THETA] = 0;
    coeff_[F3_ROOT_PHI] = 0;
    coeff_[F3_THETA] = 0;
    coeff_[F3_PHI] = 0;
    coeff_[F3_KNUCKLE_MID] = 0;
    coeff_[F3_KNUCKLE_END] = 0;
    coeff_[F0_TWIST] = 0;
    coeff_[F1_TWIST] = 0;
    coeff_[F2_TWIST] = 0;
    coeff_[F3_TWIST] = 0;
    coeff_[THUMB_TWIST] = 0;
    coeff_[F0_LENGTH] = 0;
    coeff_[F1_LENGTH] = 0;
    coeff_[F2_LENGTH] = 0;
    coeff_[F3_LENGTH] = 0;
    coeff_[THUMB_LENGTH] = 0;
    coeff_[SCALE] = HAND_MODEL_DEFAULT_SCALE;
    renormalizeCoeffs(coeff_);  // Just in case
  }
    
  void HandModelCoeff::setCoeff(uint32_t index, float coeff_value) {
    coeff_[index] = coeff_value;
    renormalizeCoeffs(coeff_);
  }
  
  const float HandModelCoeff::getCoeff(const uint32_t index) const {
    return coeff_[index];
  }

  void HandModelCoeff::setRotation(const Float3& euler) {
    coeff_[HandCoeff::HAND_ORIENT_X] = euler.m[0];
    coeff_[HandCoeff::HAND_ORIENT_Y] = euler.m[1];
    coeff_[HandCoeff::HAND_ORIENT_Z] = euler.m[2];
  }

  void HandModelCoeff::getRotation(Float3& euler) const {
    euler.m[0] = coeff_[HandCoeff::HAND_ORIENT_X];
    euler.m[1] = coeff_[HandCoeff::HAND_ORIENT_Y];
    euler.m[2] = coeff_[HandCoeff::HAND_ORIENT_Z];
  }

  // modulu - similar to matlab's mod()
  // result is always possitive. not similar to fmod()
  // Mod(-3,4)= 1   fmod(-3,4)= -3
#if defined(_WIN32)
  float inline __fastcall Mod(float x, float y) {
#else
  float inline Mod(float x, float y) {
#endif
    if (0 == y) {
      return x;
    }
    
    return x - y * floor(x / y);
  }

  // wrap [rad] angle to [0...2PI)
  inline void WrapTwo2PI(float& angle) {
    angle = Mod(angle, static_cast<float>(2.0 * M_PI));
  }

  // wrap [rad] angle to [-PI...PI)
  inline void WrapTwoPI(float& angle) {
    angle = Mod(angle + static_cast<float>(M_PI), 
      static_cast<float>(2.0 * M_PI)) - static_cast<float>(M_PI);
  }

  void HandModelCoeff::copyCoeffFrom(const HandModelCoeff* model) {
    copyCoeffFrom(model->coeff_, NUM_PARAMETERS);
  }

  void HandModelCoeff::copyCoeffFrom(const float* coeff, 
    const uint32_t ncoeffs) {
    memcpy(coeff_, coeff, sizeof(coeff_[0]) * ncoeffs);
  }

  void HandModelCoeff::renormalizeCoeffs(float* coeff) {
    // Set all angles 0 --> 2pi
    for (uint32_t i = HAND_ORIENT_X; i < HAND_NUM_COEFF; i++) {
      WrapTwoPI(coeff[i]);
    }
  }
  
  string HandCoeffToString(const uint32_t coeff) {
    switch(coeff) {
      case HAND_POS_X:
        return "HAND_POS_X";
      case HAND_POS_Y:
        return "HAND_POS_Y";
      case HAND_POS_Z:
        return "HAND_POS_Z";
      case HAND_ORIENT_X:
        return "HAND_ORIENT_X";
      case HAND_ORIENT_Y:
        return "HAND_ORIENT_Y";
      case HAND_ORIENT_Z:
        return "HAND_ORIENT_Z";
      case WRIST_THETA:
        return "WRIST_THETA";
      case WRIST_PHI:
        return "WRIST_PHI";
      case THUMB_THETA:
        return "THUMB_THETA";
      case THUMB_PHI:
        return "THUMB_PHI";
      case THUMB_K1_THETA:
        return "THUMB_K1_THETA";
      case THUMB_K1_PHI:
        return "THUMB_K1_PHI";
      case THUMB_K2_PHI:
        return "THUMB_K2_PHI";
      case F0_ROOT_THETA:
        return "F0_ROOT_THETA";
      case F0_ROOT_PHI:
        return "F0_ROOT_PHI";
      case F0_THETA:
        return "F0_THETA";
      case F0_PHI:
        return "F0_PHI";
      case F0_KNUCKLE_MID:
        return "F0_KNUCKLE_MID";
      case F0_KNUCKLE_END:
        return "F0_KNUCKLE_END";
      case F1_ROOT_THETA:
        return "F1_ROOT_THETA";
      case F1_ROOT_PHI:
        return "F1_ROOT_PHI";
      case F1_THETA:
        return "F1_THETA";
      case F1_PHI:
        return "F1_PHI";
      case F1_KNUCKLE_MID:
        return "F1_KNUCKLE_MID";
      case F1_KNUCKLE_END:
        return "F1_KNUCKLE_END";
      case F2_ROOT_THETA:
        return "F2_ROOT_THETA";
      case F2_ROOT_PHI:
        return "F2_ROOT_PHI";
      case F2_THETA:
        return "F2_THETA";
      case F2_PHI:
        return "F2_PHI";
      case F2_KNUCKLE_MID:
        return "F2_KNUCKLE_MID";
      case F2_KNUCKLE_END:
        return "F2_KNUCKLE_END";
      case F3_ROOT_THETA:
        return "F3_ROOT_THETA";
      case F3_ROOT_PHI:
        return "F3_ROOT_PHI";
      case F3_THETA:
        return "F3_THETA";
      case F3_PHI:
        return "F3_PHI";
      case F3_KNUCKLE_MID:
        return "F3_KNUCKLE_MID";
      case F3_KNUCKLE_END:
        return "F3_KNUCKLE_END";
      case F0_TWIST:
        return "F0_TWIST";
      case F1_TWIST:
        return "F1_TWIST";
      case F2_TWIST:
        return "F2_TWIST";
      case F3_TWIST:
        return "F3_TWIST";
      case THUMB_TWIST:
        return "THUMB_TWIST";
      case F0_LENGTH:
        return "F0_LENGTH";
      case F1_LENGTH:
        return "F1_LENGTH";
      case F2_LENGTH:
        return "F2_LENGTH";
      case F3_LENGTH:
        return "F3_LENGTH";
      case THUMB_LENGTH:
        return "THUMB_LENGTH";
      case SCALE:
        return "Scale";
      case NUM_PARAMETERS:
        return "NUM_PARAMETERS";
    }
    return "undefined";
  };

  void HandModelCoeff::saveToFile(const std::string& dir, 
    const std::string& filename) const {
    string full_filename = dir + filename;
    std::ofstream file(full_filename.c_str(), std::ios::out | std::ios::binary);
    if (!file.is_open()) {
      throw std::runtime_error(std::string("error opening file:") + filename);
    }
    file.write(reinterpret_cast<const char*>(coeff_), 
      NUM_PARAMETERS * sizeof(coeff_[0]));
    file.flush();
    file.close();
  }

  void HandModelCoeff::saveBlankFile(const std::string& dir, 
    const std::string& filename) const {
    string full_filename = dir + filename;
    std::ofstream file(full_filename.c_str(), std::ios::out|std::ios::binary);
    if (!file.is_open()) {
      throw std::runtime_error(std::string("error opening file:") + filename);
    }

    // Might be kinda slow to do this every time, but saving to file is pretty
    // rare, so OK for now.
    float blank[NUM_PARAMETERS];
    for (uint32_t i = 0; i < NUM_PARAMETERS; i++) {
      blank[i] = 0;
    }
    
    file.write(reinterpret_cast<const char*>(blank), 
      NUM_PARAMETERS * sizeof(blank[0]));
    file.flush();
    file.close();
  }

  bool HandModelCoeff::loadFromFile(const std::string& dir, 
    const std::string& filename) {
    string full_filename = dir + filename;
    std::ifstream file(full_filename.c_str(), std::ios::in | std::ios::binary);
    if (!file.is_open()) {
      return false;
    }
    file.seekg(0, std::ios::beg);
    // Make sure this isn't a blank file (indicating no hands on the screen)
    file.read(reinterpret_cast<char*>(coeff_), 
      NUM_PARAMETERS * sizeof(coeff_[0]));
    if (coeff_[HAND_POS_X] < EPSILON && coeff_[HAND_POS_Y] < EPSILON && 
      coeff_[HAND_POS_Z] < EPSILON) {
      resetPose();
    }
    file.close();
    return true;
  }

  bool HandModelCoeff::loadOldModelFromFile(const std::string& dir, 
    const std::string& filename) {

    string full_filename = dir + filename;
    std::ifstream file(full_filename.c_str(), std::ios::in | std::ios::binary);
    if (!file.is_open()) {
      return false;
    }
    file.seekg(0, std::ios::beg);
    // Make sure this isn't a blank file (indicating no hands on the screen)
    float coeff_old[OLD_F3_KNUCKLE_CURL+1];
    file.read(reinterpret_cast<char*>(coeff_old), 
      (OLD_F3_KNUCKLE_CURL+1) * sizeof(coeff_old[0]));
    float wrist_length;
    file.read(reinterpret_cast<char*>(&wrist_length), sizeof(wrist_length));
    float local_scale;
    file.read(reinterpret_cast<char*>(&local_scale), sizeof(local_scale));

    if (coeff_[OLD_HAND_POS_X] < EPSILON && coeff_[OLD_HAND_POS_Y] < EPSILON && 
      coeff_[OLD_HAND_POS_Z] < EPSILON) {
      resetPose();
    }

    file.close();

    coeff_[HAND_POS_X] = coeff_old[OLD_HAND_POS_X];
    coeff_[HAND_POS_Y] = coeff_old[OLD_HAND_POS_Y];
    coeff_[HAND_POS_Z] = coeff_old[OLD_HAND_POS_Z];
    coeff_[HAND_ORIENT_X] = coeff_old[OLD_HAND_ORIENT_X];
    coeff_[HAND_ORIENT_Y] = coeff_old[OLD_HAND_ORIENT_Y];
    coeff_[HAND_ORIENT_Z] = coeff_old[OLD_HAND_ORIENT_Z];
    coeff_[WRIST_THETA] = coeff_old[OLD_WRIST_THETA];
    coeff_[WRIST_PHI] = coeff_old[OLD_WRIST_PHI];
    coeff_[THUMB_THETA] = coeff_old[OLD_THUMB_THETA];
    coeff_[THUMB_PHI] = coeff_old[OLD_THUMB_PHI];
    coeff_[THUMB_K1_THETA] = coeff_old[OLD_THUMB_K1_THETA];
    coeff_[THUMB_K1_PHI] = coeff_old[OLD_THUMB_K1_PHI];
    coeff_[THUMB_K2_PHI] = coeff_old[OLD_THUMB_K2_PHI];
    coeff_[F0_ROOT_THETA] = 0;
    coeff_[F0_ROOT_PHI] = 0;
    coeff_[F0_THETA] = coeff_old[OLD_F0_THETA];
    coeff_[F0_PHI] = coeff_old[OLD_F0_PHI];
    coeff_[F0_KNUCKLE_MID] = coeff_old[OLD_F0_KNUCKLE_CURL];
    coeff_[F0_KNUCKLE_END] = coeff_old[OLD_F0_KNUCKLE_CURL];
    coeff_[F1_ROOT_THETA] = 0;
    coeff_[F1_ROOT_PHI] = 0;
    coeff_[F1_THETA] = coeff_old[OLD_F1_THETA];
    coeff_[F1_PHI] = coeff_old[OLD_F1_PHI];
    coeff_[F1_KNUCKLE_MID] = coeff_old[OLD_F1_KNUCKLE_CURL];
    coeff_[F1_KNUCKLE_END] = coeff_old[OLD_F1_KNUCKLE_CURL];
    coeff_[F2_ROOT_THETA] = 0;
    coeff_[F2_ROOT_PHI] = 0;
    coeff_[F2_THETA] = coeff_old[OLD_F2_THETA];
    coeff_[F2_PHI] = coeff_old[OLD_F2_PHI];
    coeff_[F2_KNUCKLE_MID] = coeff_old[OLD_F2_KNUCKLE_CURL];
    coeff_[F2_KNUCKLE_END] = coeff_old[OLD_F2_KNUCKLE_CURL];
    coeff_[F3_ROOT_THETA] = 0;
    coeff_[F3_ROOT_PHI] = 0;
    coeff_[F3_THETA] = coeff_old[OLD_F3_THETA];
    coeff_[F3_PHI] = coeff_old[OLD_F3_PHI];
    coeff_[F3_KNUCKLE_MID] = coeff_old[OLD_F3_KNUCKLE_CURL];
    coeff_[F3_KNUCKLE_END] = coeff_old[OLD_F3_KNUCKLE_CURL];
    coeff_[F0_TWIST] = 0;
    coeff_[F1_TWIST] = 0;
    coeff_[F2_TWIST] = 0;
    coeff_[F3_TWIST] = 0;
    coeff_[THUMB_TWIST] = 0;
    coeff_[F0_LENGTH] = 0;
    coeff_[F1_LENGTH] = 0;
    coeff_[F2_LENGTH] = 0;
    coeff_[F3_LENGTH] = 0;
    coeff_[THUMB_LENGTH] = 0;
    coeff_[SCALE] = local_scale;

    return true;
  }

  const uint8_t labelFromRGB(const float r, const float g, const float b) {
    uint32_t red_type;
    if (r < 0.25f) {
      red_type = 0;
    } else if (r < 0.75f) {
      red_type = 1;
    } else {
      red_type = 2;
    }

    uint32_t green_type;
    if (g < 0.25f) {
      green_type = 0;
    } else if (g < 0.75f) {
      green_type = 1;
    } else {
      green_type = 2;
    }

    uint32_t blue_type;
    if (b < 0.25f) {
      blue_type = 0;
    } else if (b < 0.75f) {
      blue_type = 1;
    } else {
      blue_type = 2;
    }

    int type = (red_type << 4) | (green_type << 2) | blue_type;
    switch (type) {
    case 0:   // bit(000000) = rgb(0.0, 0.0, 0.0)
      return HandLabel::BACKGROUND;
    case 1:   // bit(000001) = rgb(0.0, 0.0, 0.5)  // Dark-blue
      return HandLabel::THUMB_3;
    case 2:   // bit(000010) = rgb(0.0, 0.0, 1.0)  // Blue
      return HandLabel::THUMB_1;

    case 4:   // bit(000100) = rgb(0.0, 0.5, 0.0)  // Dark-Green
      return HandLabel::FINGER_3_2;
    case 5:   // bit(000101) = rgb(0.0, 0.5, 0.5)  // Cyan
      return HandLabel::FINGER_4_2;
    // case 6:   // bit(000110) = rgb(0.0, 0.5, 1.0)

    case 8:   // bit(001000) = rgb(0.0, 1.0, 0.0)  // Green
      return HandLabel::PALM_FRONT;
    // case 9:   // bit(001001) = rgb(0.0, 1.0, 0.5)
    case 10:  // bit(001010) = rgb(0.0, 1.0, 1.0)  // Aqua
      return HandLabel::FINGER_2_1;

    case 16:  // bit(010000) = rgb(0.5, 0.0, 0.0)  // Dark-Red
      return HandLabel::FINGER_3_1;
    case 17:  // bit(010001) = rgb(0.5, 0.0, 0.5)  // Purple
      return HandLabel::FINGER_2_3;
    // case 18:  // bit(010010) = rgb(0.5, 0.0, 1.0)

    case 20:  // bit(010100) = rgb(0.5, 0.5, 0.0)  // Dark-Yellow
      return HandLabel::FINGER_3_3;
    // case 21:  // bit(010101) = rgb(0.5, 0.5, 0.5)
    case 22:  // bit(010110) = rgb(0.5, 0.5, 1.0)  // Light-Blue
      return HandLabel::THUMB_2;

    case 24:  // bit(011000) = rgb(0.5, 1.0, 0.0)  // Lime
      return HandLabel::FINGER_4_1;
    case 25:  // bit(011001) = rgb(0.5, 1.0, 0.5)  // Light-Green
      return HandLabel::FINGER_1_1;
    // case 26:  // bit(011010) = rgb(0.5, 1.0, 1.0)

    case 32:  // bit(100000) = rgb(1.0, 0.0, 0.0)  // Red
      return HandLabel::WRIST;
    case 33:  // bit(100001) = rgb(1.0, 0.0, 0.5)  // Magenta
      return HandLabel::FINGER_4_3;
    case 34:  // bit(100010) = rgb(1.0, 0.0, 1.0)  // Pink
      return HandLabel::FINGER_1_2;

    case 36:  // bit(100100) = rgb(1.0, 0.5, 0.0)  // Gold
      return HandLabel::PALM_BACK;
    case 37:  // bit(100101) = rgb(1.0, 0.5, 0.5)  // Light-Red
      return HandLabel::FINGER_2_2;
    // case 38:  // bit(100110) = rgb(1.0, 0.5, 1.0)

    case 40:  // bit(101000) = rgb(1.0, 1.0, 0.0)  // Yellow
      return HandLabel::FINGER_1_3;
    // case 41:  // bit(101001) = rgb(1.0, 1.0, 0.5)
    // case 42:  // bit(101010) = rgb(1.0, 1.0, 1.0)
    default:
      std::cout << "ERROR: Couldn't classify pixel from RGB!" << std::endl;
      std::cout << "r = " << r << std::endl;
      std::cout << "g = " << g << std::endl;
      std::cout << "b = " << b << std::endl;
      throw std::runtime_error("ERROR: Couldn't classify pixel from RGB!");
    }
  }

  void HandModelCoeff::printCoeff() const {
    std::cout << "coeff = [";
    std::cout << std::setprecision(6);
    std::cout << std::fixed;
    for (uint32_t i = 0; i < NUM_PARAMETERS; i++) {
      std::cout << coeff_[i];
      if (i < NUM_PARAMETERS - 1) {
        std::cout << ", ";
      }
    }
    std::cout << "]" << std::endl;
  }

  const float HandModelCoeff::sph_off_[NUM_BOUNDING_SPHERES*3] = { 
    -0.1355f, -0.00849999f, -0.2875f,  // F1_KNU3_A,
    0.002f, 0.007f, -0.1205f,  // F1_KNU3_B,
    -0.13f, 0.0305f, -0.1975f,  // F1_KNU2_A,
    0.0295f, 0.00149996f, -0.0615f,  // F1_KNU2_B,
    -0.3195f, 0.0315f, -0.211f,  // F1_KNU1_A,
    0.0115f, -0.0235f, -0.1275f,  // F1_KNU1_B,
    -0.2615f, -0.1135f, -0.3965f,  // F2_KNU3_A,
    -0.126f, -0.0245f, -0.131f,  // F2_KNU3_B,
    -0.144f, -0.00450001f, -0.0855f,  // F2_KNU2_A,
    0.0705f, 0.00400001f, 0.03f,  // F2_KNU2_B,
    -0.3505f, -0.0275f, -0.281f,  // F2_KNU1_A,
    -0.002f, -0.0635f, -0.1945f,  // F2_KNU1_B,
    -0.157f, -0.0285f, -0.279f,  // F3_KNU3_A,
    -0.0195f, 0.0375f, 0.001f,  // F3_KNU3_B,
    -0.1665f, 0.022f, -0.205f,  // F3_KNU2_A,
    0.029f, 0.0545f, -0.0535f,  // F3_KNU2_B,
    -0.419f, 0.0565f, -0.044f,  // F3_KNU1_A,
    -0.0095f, 0.0005f, 0.0085f,  // F3_KNU1_B,
    -0.343f, 0.012f, -0.3445f,  // F4_KNU3_A,
    -0.144f, 0.0295f, -0.189f,  // F4_KNU3_B,
    -0.2485f, 0.008f, -0.172f,  // F4_KNU2_A,
    0.0f, 0.0335f, -0.0125f,  // F4_KNU2_B,
    -0.5595f, -0.035f, -0.0315f,  // F4_KNU1_A,
    -0.0325f, -0.0405f, 0.0f,  // F4_KNU1_B,
    -0.432f, 0.0775f, -0.104f,  // TH_KNU3_A,
    -0.066f, 0.0950001f, -0.038f,  // TH_KNU3_B,
    -0.341f, 0.017f, 0.0175f,  // TH_KNU2_A,
    -0.0335f, 0.0585f, 0.044f,  // TH_KNU2_B,
    -0.4485f, -0.343f, -0.115f,  // TH_KNU1_A,
    0.0f, 0.0f, 0.0f,  // TH_KNU1_B,
    -0.1f, 0.305f, -0.064f,  // PALM_1,
    -0.1f, -0.305f, -0.064f,  // PALM_2,
    -1.467f, 0.0f, 0.0f,  // PALM_3,
    -1.307f, 0.4095f, -0.2f,  // PALM_4,
    -0.986f, 0.0f, 0.0f,  // PALM_5,
    0.0f, 0.0f, 0.0f,  // PALM_6,
  };

  // 3/28/2013 --> Reduced sphere radius by 10% and thumb tip by 38.8%
  const float HandModelCoeff::sph_size_[NUM_BOUNDING_SPHERES] = {
    0.086f,   // F1_KNU3_A,  // prev 0.095f - 3/28/2013
    0.108f,   // F1_KNU3_B,  // prev 0.12f - 3/28/2013
    0.126f,   // F1_KNU2_A,  // prev 0.14f - 3/28/2013
    0.144f,   // F1_KNU2_B,  // prev 0.16f - 3/28/2013
    0.103f,   // F1_KNU1_A,  // prev 0.17f - 3/28/2013
    0.140f,   // F1_KNU1_B,  // prev 0.20f - 3/28/2013
    0.104f,   // F2_KNU3_A,  // prev 0.115f - 3/28/2013
    0.126f,   // F2_KNU3_B,  // prev 0.14f - 3/28/2013
    0.153f,   // F2_KNU2_A,  // prev 0.17f - 3/28/2013
    0.162f,   // F2_KNU2_B,  // prev 0.18f - 3/28/2013
    0.121f,   // F2_KNU1_A,  // prev 0.19f - 3/28/2013
    0.140f,   // F2_KNU1_B,  // prev 0.20f - 3/28/2013
    0.104f,   // F3_KNU3_A,  // prev 0.115f - 3/28/2013
    0.133f,   // F3_KNU3_B,  // prev 0.17f - 3/28/2013
    0.162f,   // F3_KNU2_A,  // prev 0.18f - 3/28/2013
    0.180f,   // F3_KNU2_B,  // prev 0.20f - 3/28/2013
    0.130f,   // F3_KNU1_A,  // prev 0.20f - 3/28/2013
    0.149f,   // F3_KNU1_B,  // prev 0.21f - 3/28/2013
    0.095f,   // F4_KNU3_A,  // prev 0.105f - 3/28/2013
    0.120f,   // F4_KNU3_B,  // prev 0.16f - 3/28/2013
    0.153f,   // F4_KNU2_A,  // prev 0.17f - 3/28/2013
    0.162f,   // F4_KNU2_B,  // prev 0.18f - 3/28/2013
    0.130f,   // F4_KNU1_A,  // prev 0.20f - 3/28/2013
    0.149f,   // F4_KNU1_B,  // prev 0.21f - 3/28/2013
    0.104f,   // TH_KNU3_A,  // prev 0.17f - 3/28/2013  // Tip
    0.171f,   // TH_KNU3_B,  // prev 0.19f - 3/28/2013
    0.180f,   // TH_KNU2_A,  // prev 0.20f - 3/28/2013
    0.225f,   // TH_KNU2_B,  // prev 0.25f - 3/28/2013
    0.252f,   // TH_KNU1_A,  // prev 0.28f - 3/28/2013
    0.252f,   // TH_KNU1_B,  // prev 0.28f - 3/28/2013
    0.250f,   // PALM_1  // base 1
    0.250f,   // PALM_2  // base 2
    0.280f,   // PALM_3  // top 1 (very top)
    0.280f,   // PALM_4  // top 2
    0.300f,   // PALM_5
    0.280f,   // PALM_6
  };

}  // namespace hand_net
}  // namespace kinect_interface_primesense
