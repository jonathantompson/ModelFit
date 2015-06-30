#include <cmath>
#include <string>
#include <iostream>
#include <stdexcept>
#include <iostream>
#include <fstream>
#include "kinect_interface_primesense/open_ni_funcs.h"
#include "kinect_interface_primesense/calibration_data.h"

#define SAFE_DELETE(x) if (x != NULL) { delete x; x = NULL; }
#define SAFE_DELETE_ARR(x) if (x != NULL) { delete[] x; x = NULL; }
#define FALSE 0
#define TRUE 1

using std::cout;
using std::endl;

#ifndef XN_STATUS_OK
  #define XN_STATUS_OK   ((uint32_t)0)
#endif

// #define OPEN_NI_FUNCS_USE_OMP  // Actually slows down KinectHands

namespace kinect_interface_primesense {
  
  // Kinect constants FROM: XnOpenNI.cpp (and slightly edited)
  const double OpenNIFuncs::fHFOV_kinect_ = 1.0144686707507438;
  const double OpenNIFuncs::fVFOV_kinect_ = 0.78980943449644714;

  const float OpenNIFuncs::fHFOV_primesense_109 = 1.01707470885081f * 0.98f;  // openNI depth
  const float OpenNIFuncs::fVFOV_primesense_109 = 0.791989554540429f * 0.98f;  // openNI depth

  //const float OpenNIFuncs::fHFOV_primesense_109 = 1.017314617897f;  // measured depth
  //const float OpenNIFuncs::fVFOV_primesense_109 = 0.795009466072f;  // measured depth

  //const float OpenNIFuncs::fHFOV_primesense_109 = 1.075848937034607f;  // approx rgb
  //const float OpenNIFuncs::fVFOV_primesense_109 = 0.8383198380470276f;  // approx rgb
  //const float OpenNIFuncs::fHFOV_primesense_109 = 1.076187422640391f;  // measured rgb
  //const float OpenNIFuncs::fVFOV_primesense_109 = 0.844611400289787f;  // measured rgb

  const double OpenNIFuncs::m_fRealWorldXtoZ_kinect_ = tan(OpenNIFuncs::fHFOV_kinect_/2)*2;
  const double OpenNIFuncs::m_fRealWorldYtoZ_kinect_ = tan(OpenNIFuncs::fVFOV_kinect_/2)*2;
  const uint32_t OpenNIFuncs::nXRes_kinect_ = 640;
  const uint32_t OpenNIFuncs::nYRes_kinect_ = 480;
  const uint32_t OpenNIFuncs::nFPS_kinect_ = 30;
  const uint32_t OpenNIFuncs::nXRes_primesense_109 = 640;
  const uint32_t OpenNIFuncs::nYRes_primesense_109 = 480;

  OpenNIFuncs::OpenNIFuncs(const uint32_t nXRes, const uint32_t nYRes, 
    const float hFOV, const float vFOV, const uint32_t internal_dev_id) {
    internal_dev_id_ = internal_dev_id;
    cal_data_ = NULL;
    nXRes_ = (float)nXRes;
    nYRes_ = (float)nYRes;
    fHFOV_ = hFOV;
    fVFOV_ = vFOV;
    xzFactor_ = tan(fHFOV_ / 2) * 2;
	  yzFactor_ = tan(fVFOV_ / 2) * 2;
    halfResX_ = nXRes_ / 2;
	  halfResY_ = nYRes_ / 2;
	  coeffX_ = nXRes_ / xzFactor_;
	  coeffY_ = nYRes_ / yzFactor_;
    try {
      loadCalibrationData();
    } catch (std::exception e) {
      std::cout << "Warning: Calibration file does not exist.  Some ";
      std::cout << "functionality might be disabled." << std::endl;
    }
  }

  OpenNIFuncs::OpenNIFuncs() {
    internal_dev_id_ = 0;
    nXRes_ = (float)nXRes_primesense_109;
    nYRes_ = (float)nYRes_primesense_109;
    fHFOV_ = fHFOV_primesense_109;
    fVFOV_ = fVFOV_primesense_109;
    xzFactor_ = tan(fHFOV_ / 2.0f) * 2.0f;
	  yzFactor_ = tan(fVFOV_ / 2.0f) * 2.0f;
    halfResX_ = nXRes_ / 2.0f;
	  halfResY_ = nYRes_ / 2.0f;
	  coeffX_ = nXRes_ / xzFactor_;
	  coeffY_ = nYRes_ / yzFactor_;
    cal_data_ = NULL;
    try {
      loadCalibrationData();
    } catch (std::exception e) {
      std::cout << "Warning: Calibration file does not exist.  Some ";
      std::cout << "functionality might be disabled." << std::endl;
    }
  }

  OpenNIFuncs::~OpenNIFuncs() {
    SAFE_DELETE(cal_data_);
  }

  // FROM: XnOpenNI.cpp (and slightly edited)
  uint32_t OpenNIFuncs::xnConvertProjectiveToRealWorld(uint32_t nCount,
    const float* aProjective, float* aRealWorld) {
    uint32_t nRetVal = XN_STATUS_OK;
    
    /**
     * X_RW = (X_proj / X_res - 1/2) * Z * x_to_z
     */
    
    double fXToZ = GetRealWorldXtoZKinect();
    double fYToZ = GetRealWorldYtoZKinect();
    
    for (uint32_t i = 0; i < nCount; ++i)
    {
      double fNormalizedX = (aProjective[i*3] / nXRes_kinect_ - 0.5);
      aRealWorld[i*3] = (float)(fNormalizedX * aProjective[i*3+2] * fXToZ);
      
      double fNormalizedY = (0.5 - aProjective[i*3+1] / nYRes_kinect_);
      aRealWorld[i*3+1] = (float)(fNormalizedY * aProjective[i*3+2] * fYToZ);
      
      aRealWorld[i*3+2] = aProjective[i*3+2];
    }
    
    return nRetVal;
  }
  
  // FROM: XnOpenNI.cpp (and slightly edited)
  uint32_t OpenNIFuncs::xnConvertRealWorldToProjective(uint32_t nCount,
    const float* aRealWorld, float* aProjective)
  {
    uint32_t nRetVal = XN_STATUS_OK;
    
    /**
     * X_proj = X_res * (X_RW / (z*x_to_z) + 1/2)
     *
     *		= X_res / x_to_z * X_RW / z + X_res/2     (more efficient)
     */
    
    double fXToZ = GetRealWorldXtoZKinect();
    double fYToZ = GetRealWorldYtoZKinect();
    
    double fCoeffX = nXRes_kinect_ / fXToZ;
    double fCoeffY = nYRes_kinect_ / fYToZ;
    
    // we can assume resolution is even (so integer div is sufficient)
    uint32_t nHalfXres = nXRes_kinect_ / 2;
    uint32_t nHalfYres = nYRes_kinect_ / 2;
    
    for (uint32_t i = 0; i < nCount; ++i)
    {
      aProjective[i*3] = (float)fCoeffX * aRealWorld[i*3] / aRealWorld[i*3+2] + nHalfXres;
      aProjective[i*3+1] = nHalfYres - (float)fCoeffY * aRealWorld[i*3+1] / aRealWorld[i*3+2];
      aProjective[i*3+2] = aRealWorld[i*3+2];
    }
    
    return nRetVal;
  }
  
  uint32_t OpenNIFuncs::ConvertDepthImageToProjectiveKinect(const uint16_t* aDepth,
    float* aProjective) {
    int nIndex = 0;
    for (uint32_t nY = 0; nY < nYRes_kinect_; nY += 1) {
      for (uint32_t nX = 0; nX < nXRes_kinect_; nX += 1, nIndex += 1) {
        aProjective[nIndex*3] = static_cast<float>(nX);
        aProjective[nIndex*3+1] = static_cast<float>(nY);
        aProjective[nIndex*3+2] = aDepth[nIndex];
      }
    }
    return XN_STATUS_OK;
  }

  void OpenNIFuncs::ConvertDepthImageToProjective(const uint16_t* aDepth,
    float* aProjective) {
#ifdef OPEN_NI_FUNCS_USE_OMP
    #pragma omp parallel for num_threads(4)
#endif
    for (int32_t nY = 0; nY < (int32_t)nYRes_; nY += 1) {
      for (int32_t nX = 0; nX < (int32_t)nXRes_; nX += 1) {
        int32_t nIndex = nY * (int32_t)nXRes_ + nX;
        aProjective[nIndex*3] = static_cast<float>(nX);
        aProjective[nIndex*3+1] = static_cast<float>(nY);
        aProjective[nIndex*3+2] = aDepth[nIndex];
      }
    }
  }

  // From OniStream.cpp (and edited)
  // https://github.com/OpenNI/OpenNI2/blob/master/Source/Core/OniStream.cpp
  void OpenNIFuncs::convertDepthToWorldCoordinates(const float* uvd, float* xyz, 
    const uint32_t nCount) {
#ifdef OPEN_NI_FUNCS_USE_OMP
    #pragma omp parallel for num_threads(4)
#endif
    for (int32_t i = 0; i < (int32_t)nCount; i++) {
      float normalizedX = uvd[i*3] / nXRes_ - .5f;
	    float normalizedY = .5f - uvd[i*3+1] / nYRes_;
      xyz[i*3] = normalizedX * uvd[i*3+2] * xzFactor_;
	    xyz[i*3+1] = normalizedY * uvd[i*3+2] * yzFactor_;
	    xyz[i*3+2] = uvd[i*3+2];
    }
  }
  void OpenNIFuncs::convertWorldToDepthCoordinates(const float* xyz, float* uvd, 
    const uint32_t nCount) {
    for (uint32_t i = 0; i < nCount; i++) {
      uvd[3*i] = coeffX_ * xyz[3*i] / xyz[3*i+2] + halfResX_;
	    uvd[3*i+1] = halfResY_ - coeffY_ * xyz[3*i+1] / xyz[3*i+2];
	    uvd[3*i+2] = xyz[3*i+2];
    }
  }

  bool TranslateSinglePixel_error_msg = false;
  bool OpenNIFuncs::TranslateSinglePixel(const uint32_t x, const uint32_t y, 
    uint16_t z, int& imageX, int& imageY, const bool m_isMirrored) {
    if (cal_data_ == NULL) {
      imageX = 0;
      imageY = 0;
      if (!TranslateSinglePixel_error_msg) {
        std::cout << "Warning: Calibration data wasn't loaded!" << std::endl;
        std::cout << "  TranslateSinglePixel will only return (0,0)" << std::endl;
        TranslateSinglePixel_error_msg = true;
      }
      return false;
    }
    imageX = 0;
    imageY = 0;

    uint32_t nDepthXRes = m_depthResolution.x;
    bool bMirror = m_isMirrored;
    uint32_t nIndex = bMirror ? ((y+1)*nDepthXRes - x - 1) * 2 : 
      (y*nDepthXRes + x) * 2;
    uint16_t* pRegTable = (uint16_t*)&m_pRegTable[nIndex];
    uint16_t* pRGBRegDepthToShiftTable = m_pDepth2ShiftTable; 
    uint32_t nNewX = 0;
    uint32_t nNewY = 0;

    uint32_t nLinesShift = cal_data_->m_pPadInfo->nCroppingLines - 
      cal_data_->m_pPadInfo->nStartLines;

    if (z == 0) {
      return false;
    }

    nNewX = (uint32_t)(*pRegTable + pRGBRegDepthToShiftTable[z]) / 
      cal_data_->m_blob.params1080.rgbRegXValScale;
    nNewY = *(pRegTable+1);
    if (nNewX >= nDepthXRes || nNewY < nLinesShift) {
      //return false;
    }

    imageX = bMirror ? (nDepthXRes - nNewX - 1) : nNewX;
    imageY = nNewY - nLinesShift;

    /////////////////////////////////////
     

    double fullXRes = m_colorResolution.x;
    double fullYRes;
    bool bCrop = FALSE;

    if ((9 * m_colorResolution.x / m_colorResolution.y) == 16)
    {
      fullYRes = m_colorResolution.x * 4 / 5;
      bCrop = TRUE;
    }
    else
    {
      fullYRes = m_colorResolution.y;
      bCrop = FALSE;
    }

    // inflate to full res
    imageX = (uint32_t)(fullXRes / m_depthResolution.x * imageX);
    imageY = (uint32_t)(fullYRes / m_depthResolution.y * imageY);

    if (bCrop)
    {
      // crop from center
      imageY -= (uint32_t)(fullYRes - m_colorResolution.y)/2;
      if ((uint32_t)imageY > (uint32_t)m_colorResolution.y)
      {
        return false;
      }
    }

    return true;

  }

  void OpenNIFuncs::loadCalibrationData() {
    char filename[256];
#if defined(WIN32) || defined(_WIN32)
#pragma warning(push)
#pragma warning(disable:4996)
  _snprintf(filename, 256, "calibration_info1080_%d.bin", internal_dev_id_);
#pragma warning(pop)
#else
  snprintf(filename, 256, "calibration_info1080_%d.bin", internal_dev_id_);
#endif
    std::ifstream file(filename, std::ios::in | std::ios::binary);
    if (!file.is_open()) {
      throw std::runtime_error(std::string("OpenNIFuncs::loadCalibrationData()"
        " - ERROR: Cannot open output file:") + filename);
    }

    cal_data_ = new CalibrationData();
    file.read((char*)(&cal_data_->m_blob),
      sizeof(DepthUtilsSensorCalibrationInfo));
    file.read((char*)cal_data_->m_pRegistrationTable_QQVGA, 
      sizeof(cal_data_->m_pRegistrationTable_QQVGA[0]) * 160*120*2);
    file.read((char*)cal_data_->m_pRegistrationTable_QVGA, 
      sizeof(cal_data_->m_pRegistrationTable_QVGA[0]) * 320*240*2);
    file.read((char*)cal_data_->m_pRegistrationTable_VGA, 
      sizeof(cal_data_->m_pRegistrationTable_VGA[0]) * 640*480*2);
    file.read((char*)cal_data_->m_pDepthToShiftTable_QQVGA, 
      sizeof(cal_data_->m_pDepthToShiftTable_QQVGA[0]) * (MAX_Z+1));
    file.read((char*)cal_data_->m_pDepthToShiftTable_QVGA, 
      sizeof(cal_data_->m_pDepthToShiftTable_QVGA[0]) * (MAX_Z+1));
    file.read((char*)cal_data_->m_pDepthToShiftTable_VGA, 
      sizeof(cal_data_->m_pDepthToShiftTable_VGA[0]) * (MAX_Z+1));
    file.close();

    switch ((int)nXRes_) {
    case 640:
      m_pRegTable = cal_data_->m_pRegistrationTable_VGA;
      m_pDepth2ShiftTable = cal_data_->m_pDepthToShiftTable_VGA;
      cal_data_->m_pPadInfo = &cal_data_->m_blob.params1080.padInfo_VGA;
      break;
    case 320:
      m_pRegTable = cal_data_->m_pRegistrationTable_QVGA;
      m_pDepth2ShiftTable = cal_data_->m_pDepthToShiftTable_QVGA;
      cal_data_->m_pPadInfo = &cal_data_->m_blob.params1080.padInfo_QVGA;
      break;
    case 160:
      m_pRegTable = cal_data_->m_pRegistrationTable_QQVGA;
      m_pDepth2ShiftTable = cal_data_->m_pDepthToShiftTable_QQVGA;
      cal_data_->m_pPadInfo = &cal_data_->m_blob.params1080.padInfo_QQVGA;
      break;
    default:
      throw std::runtime_error("OpenNIFuncs::loadCalibrationData() - ERROR: "
        "No calibration data for the current resolution mode!");
    }

    m_depthResolution.x = (int)nXRes_;
    m_depthResolution.y = (int)nYRes_;
    m_colorResolution.x = (int)nXRes_;
    m_colorResolution.y = (int)nYRes_;
  }

}  // namespace kinect_interface_primesense


