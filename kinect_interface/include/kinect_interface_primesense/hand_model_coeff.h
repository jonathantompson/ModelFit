//
//  hand_model_coeff.h
//
//  Created by Jonathan Tompson on 3/1/13.
//
//  Just a container for storing hand model coefficients and loading/saving
//  them from/to file.
//

#pragma once

#include "math/math_types.h"
#include "data_str/vector.h"
#include "kinect_interface_primesense/kinect_interface_primesense.h"  // for src_width/height

#define HAND_NUM_COEFF 42  // The num of coefficients to use when optimizing
#define FINGER_NUM_COEFF 6
#define HAND_MODEL_DEFAULT_SCALE 58.0f
#define HAND_MODEL_CAMERA_VIEW_PLANE_NEAR 10.0f
#define HAND_MODEL_CAMERA_VIEW_PLANE_FAR 5000.0f
//#define HAND_CAMERA_FOV 45.25f  // OLD Kinect VALUE! --> Now use OpenNIFuncs
#define LOAD_JBIN_FILES

#define NSPH_PER_GROUP 6  // Spheres are split into groups, within each group
                          // of n objects interpenetation has no penalty

namespace jtil {
namespace renderer {
  class GeometryInstance;
};
};

namespace kinect_interface_primesense {
namespace hand_net {

  class HandImageGenerator;

  // NOTE: ALL ANGLES (STARTING AT WRIST_THETA) ARE DEFINED TO BE RESTING AT
  //       ROUGHLY 180 DEG (PI RAD).  This makes the penalty calculation easier.
  typedef enum {
    HAND_POS_X        = 0, 
    HAND_POS_Y        = 1,
    HAND_POS_Z        = 2,
    HAND_ORIENT_X     = 3,
    HAND_ORIENT_Y     = 4,
    HAND_ORIENT_Z     = 5,
    WRIST_THETA       = 6,
    WRIST_PHI         = 7,
    THUMB_THETA       = 8,
    THUMB_PHI         = 9,
    THUMB_K1_THETA    = 10,
    THUMB_K1_PHI      = 11,
    THUMB_K2_PHI      = 12,
    F0_ROOT_THETA     = 13,
    F0_ROOT_PHI       = 14,
    F0_THETA          = 15,
    F0_PHI            = 16,
    F0_KNUCKLE_MID    = 17,
    F0_KNUCKLE_END    = 18,
    F1_ROOT_THETA     = 19,
    F1_ROOT_PHI       = 20,
    F1_THETA          = 21,
    F1_PHI            = 22,
    F1_KNUCKLE_MID    = 23,
    F1_KNUCKLE_END    = 24,
    F2_ROOT_THETA     = 25,
    F2_ROOT_PHI       = 26,
    F2_THETA          = 27,
    F2_PHI            = 28,
    F2_KNUCKLE_MID    = 29,
    F2_KNUCKLE_END    = 30,
    F3_ROOT_THETA     = 31,
    F3_ROOT_PHI       = 32,
    F3_THETA          = 33,
    F3_PHI            = 34,
    F3_KNUCKLE_MID    = 35,
    F3_KNUCKLE_END    = 36,
    F0_TWIST          = 37,
    F1_TWIST          = 38,
    F2_TWIST          = 39,
    F3_TWIST          = 40,
    THUMB_TWIST       = 41,
    F0_LENGTH         = 42,  // Not used in optimization
    F1_LENGTH         = 43,  // Not used in optimization
    F2_LENGTH         = 44,  // Not used in optimization
    F3_LENGTH         = 45,  // Not used in optimization
    THUMB_LENGTH      = 46,  // Not used in optimization
    SCALE             = 47,  // Not used in optimization
    NUM_PARAMETERS    = 48,  // NOTE: Not to be confused with HAND_NUM_COEFF!!
  } HandCoeff;

  // The old version before adding in seperable finger joints, finger root
  // nodes, finger twist and lengths.
  typedef enum {
    OLD_HAND_POS_X        = 0, 
    OLD_HAND_POS_Y        = 1,
    OLD_HAND_POS_Z        = 2,
    OLD_HAND_ORIENT_X     = 3,
    OLD_HAND_ORIENT_Y     = 4,
    OLD_HAND_ORIENT_Z     = 5,
    OLD_WRIST_THETA       = 6,
    OLD_WRIST_PHI         = 7,
    OLD_THUMB_THETA       = 8,
    OLD_THUMB_PHI         = 9,
    OLD_THUMB_K1_THETA    = 10,
    OLD_THUMB_K1_PHI      = 11,
    OLD_THUMB_K2_PHI      = 12,
    OLD_F0_THETA          = 13,
    OLD_F0_PHI            = 14,
    OLD_F0_KNUCKLE_CURL   = 15,
    OLD_F1_THETA          = 16,
    OLD_F1_PHI            = 17,
    OLD_F1_KNUCKLE_CURL   = 18,
    OLD_F2_THETA          = 19,
    OLD_F2_PHI            = 20,
    OLD_F2_KNUCKLE_CURL   = 21,
    OLD_F3_THETA          = 22,
    OLD_F3_PHI            = 23,
    OLD_F3_KNUCKLE_CURL   = 24,
    OLD_WRIST_LENGTH      = 25,  // Not used in optimization
    OLD_SCALE             = 26,  // Not used in optimization
    OLD_WRIST_TWIST       = 27,  // Not used in optimization
    OLD_NUM_PARAMETERS    = 28,  // NOTE: Not to be confused with HAND_NUM_COEFF!!
  } HandCoeffOld;

  // For generating "colored glove" images for Murphy...
  typedef enum {
    WRIST             = 0, 
    PALM_FRONT        = 1,
    PALM_BACK         = 2,
    THUMB_1           = 3,
    THUMB_2           = 4,
    THUMB_3           = 5,
    FINGER_1_1        = 6,
    FINGER_1_2        = 7,
    FINGER_1_3        = 8,
    FINGER_2_1        = 9,
    FINGER_2_2        = 10,
    FINGER_2_3        = 11,
    FINGER_3_1        = 12,
    FINGER_3_2        = 13,
    FINGER_3_3        = 14,
    FINGER_4_1        = 15,
    FINGER_4_2        = 16,
    FINGER_4_3        = 17,
    BACKGROUND        = 18,
    NUM_HANDLABEL     = 19,
  } HandLabel;
  
  typedef enum {
    F1_KNU3_A = 0,  // tip
    F1_KNU3_B = 1,
    F1_KNU2_A = 2,
    F1_KNU2_B = 3,
    F1_KNU1_A = 4,
    F1_KNU1_B = 5,  // base
    F2_KNU3_A = 6,  // tip
    F2_KNU3_B = 7,
    F2_KNU2_A = 8,
    F2_KNU2_B = 9,
    F2_KNU1_A = 10,
    F2_KNU1_B = 11,  // base
    F3_KNU3_A = 12,  // tip
    F3_KNU3_B = 13,
    F3_KNU2_A = 14,
    F3_KNU2_B = 15,
    F3_KNU1_A = 16,
    F3_KNU1_B = 17,  // base
    F4_KNU3_A = 18,  // tip
    F4_KNU3_B = 19,
    F4_KNU2_A = 20,
    F4_KNU2_B = 21,
    F4_KNU1_A = 22,
    F4_KNU1_B = 23,  // base
    TH_KNU3_A = 24,  // Tip
    TH_KNU3_B = 25,
    TH_KNU2_A = 26,
    TH_KNU2_B = 27,
    TH_KNU1_A = 28,
    TH_KNU1_B = 29,  // Base
    PALM_1    = 30,
    PALM_2    = 31,
    PALM_3    = 32,
    PALM_4    = 33,
    PALM_5    = 34,
    PALM_6    = 35,  // At (0,0,0)
    NUM_BOUNDING_SPHERES = 36  // Must be a mulitple of NSPH_PER_FING
  } HandSphereIndices;
  
  typedef enum {
    LEFT = 0,
    RIGHT = 1,
    UNDEFINED = 2,
  } HandType;
  
  std::string HandCoeffToString(const uint32_t coeff);
  const uint8_t labelFromRGB(const float r, const float g, const float b);

  class HandModelCoeff {
  public:
    // Constructor / Destructor
    HandModelCoeff(const HandType hand_type);
    ~HandModelCoeff();

    // Accessors
    const float getCoeff(const uint32_t index) const;
    float* coeff() { return coeff_; }
    const float* coeff() const { return coeff_; }
    const HandType hand_type() const { return hand_type_; }
    void setRotation(const jtil::math::Float3& euler);
    void getRotation(jtil::math::Float3& euler) const;
    void printCoeff() const;
    void setCoeff(uint32_t index, float coeff_value);
    
    // FILE IO
    void saveToFile(const std::string& dir, const std::string& filename) const;
    void saveBlankFile(const std::string& dir, const std::string& filename) 
      const;
    bool loadFromFile(const std::string& dir, const std::string& filename);
    // Model update 4/11 with the Primesense 1.09 (added thumb twist and other stuff)
    bool loadOldModelFromFile(const std::string& dir, const std::string& filename);  

    void copyCoeffFrom(const HandModelCoeff* model);
    void copyCoeffFrom(const float* coeff, const uint32_t ncoeffs);

    static void renormalizeCoeffs(float* coeff);
    void resetPose();

    // Bounding sphere positions:
    // Bones aren't in the correct position (need offsets)
    static const float sph_off_[NUM_BOUNDING_SPHERES * 3];  
    static const float sph_size_[NUM_BOUNDING_SPHERES];

  private:
    float coeff_[NUM_PARAMETERS];  // The current state
    HandType hand_type_;

    // TO DO: Move these to another class
    //static jtil::renderer::GeometryInstance* lhand;
    //static jtil::renderer::GeometryInstance* rhand;
    //static void setHandModelPose(jtil::renderer::GeometryInstance* hand, 
    //  const HandImageGenerator* im_gen, const float* convnet_coeff);

    // Non-copyable, non-assignable.
    HandModelCoeff(HandModelCoeff&);
    HandModelCoeff& operator=(const HandModelCoeff&);
  };

};  // namespace hand_net
};  // namespace kinect_interface_primesense
