//
//  open_ni_funcs.h
//
//  This is just a bunch of functions taken out of the OpenNI2 source code that
//  I need and that make calculating some of the pixel transformations orders
//  of magnitude faster (as well as resulting in being able to call them 
//  without OpenNI).
//

#pragma once

#include "math/math_types.h"
		
namespace kinect_interface_primesense {
  struct CalibrationData;
  
  class OpenNIFuncs {
  public:
    // Top level interface
    
    // internal_dev_id --> I hacked the OpenNI library to save all the 
    // calibration data to disk on startup.  The filename is enumerated by
    // the number of kinects that have been opened.
    OpenNIFuncs(const uint32_t nXRes, const uint32_t nYRes, 
      const float hFOV, const float vFOV, const uint32_t internal_dev_id);
    OpenNIFuncs();  // The default is for the primesense 1.09
    ~OpenNIFuncs();

    // This is for the new Primesense 1.09 sensor
    void convertDepthToWorldCoordinates(const float* uvd, float* xyz, 
      const uint32_t nCount);
    void convertWorldToDepthCoordinates(const float* xyz, float* uvd, 
      const uint32_t nCount);
    void ConvertDepthImageToProjective(const uint16_t* aDepth,
      float* aProjective);

    // TranslateSinglePixel is taken from DepthUtilsImpl.cpp and all the
    // constants were intercepted at runtime.
    // Returns false if transformation is impossible (depth = 0 for instance)
    bool TranslateSinglePixel(const uint32_t x, const uint32_t y, 
      uint16_t z, int& imageX, int& imageY, const bool m_isMirrored);

    // The following are for the Kinect
    static uint32_t xnConvertProjectiveToRealWorld(uint32_t nCount,
      const float* aProjective, float* aRealWorld);
    static uint32_t xnConvertRealWorldToProjective(uint32_t nCount,
      const float* aRealWorld, float* aProjective);
    static uint32_t ConvertDepthImageToProjectiveKinect(const uint16_t* aDepth,
      float* aProjective);
    
    // Primesense 1.09 constants
    void update109Constants();

    static const float fHFOV_primesense_109;
    static const float fVFOV_primesense_109;
    static const uint32_t nXRes_primesense_109;
    static const uint32_t nYRes_primesense_109;

  private:
    // Kinect constants
    static const double m_fRealWorldXtoZ_kinect_;
		static const double m_fRealWorldYtoZ_kinect_;
    static const double fHFOV_kinect_;
    static const double fVFOV_kinect_;
    static const uint32_t nXRes_kinect_;
    static const uint32_t nYRes_kinect_;
    static const uint32_t nFPS_kinect_;

    float nXRes_;
    float nYRes_;
    float fHFOV_;
    float fVFOV_;
    float xzFactor_;
    float yzFactor_;
    float halfResX_;
    float halfResY_;
    float coeffX_;
    float coeffY_;
    uint32_t internal_dev_id_;
    CalibrationData* cal_data_;
    uint16_t* m_pRegTable;  // Pointer into Calibration data (depending on current resolution)
    uint16_t* m_pDepth2ShiftTable;  // Pointer into Calibration data (depending on current resolution)
    struct {
      int x, y;
    } m_depthResolution, m_colorResolution;

    inline static double GetRealWorldXtoZKinect() { return m_fRealWorldXtoZ_kinect_; }
		inline static double GetRealWorldYtoZKinect() { return m_fRealWorldYtoZ_kinect_; }

    void loadCalibrationData();
  };
  
};  // namespace kinect_interface_primesense
