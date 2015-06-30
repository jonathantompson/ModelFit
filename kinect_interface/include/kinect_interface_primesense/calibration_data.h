//
//  calibration_data.h
//
//  Container to store the intercepted Primesense calibration data
//

#pragma once

#include "math/math_types.h"

#define MAX_Z 65535

#ifdef ADD_OPEN_NI_DEPENDANCY

#include <OpenNI.h>
#include "OniCAPI.h"
#include "DepthUtils.h"

#else

// ***********************************************
// From OpenNI.h:
// ***********************************************
typedef struct
{
	int nRGS_DX_CENTER;
	int nRGS_AX;
	int nRGS_BX;
	int nRGS_CX;
	int nRGS_DX;
	int nRGS_DX_START;
	int nRGS_AY;
	int nRGS_BY;
	int nRGS_CY;
	int nRGS_DY;
	int nRGS_DY_START;
	int nRGS_DX_BETA_START;
	int nRGS_DY_BETA_START;
	int nRGS_ROLLOUT_BLANK;
	int nRGS_ROLLOUT_SIZE;
	int nRGS_DX_BETA_INC;
	int nRGS_DY_BETA_INC;
	int nRGS_DXDX_START;
	int nRGS_DXDY_START;
	int nRGS_DYDX_START;
	int nRGS_DYDY_START;
	int nRGS_DXDXDX_START;
	int nRGS_DYDXDX_START;
	int nRGS_DXDXDY_START;
	int nRGS_DYDXDY_START;
	int nBACK_COMP1;
	int nRGS_DYDYDX_START;
	int nBACK_COMP2;
	int nRGS_DYDYDY_START;
} RegistrationInfo;

typedef struct
{
	unsigned short nStartLines;
	unsigned short nEndLines;
	unsigned short nCroppingLines;
} PadInfo;

typedef struct
{
	int magic;
	int version;
	char deviceName[80];
	char serial[80];
	struct  // 1080
	{
		PadInfo padInfo_QQVGA;	
		PadInfo padInfo_QVGA;
		PadInfo padInfo_VGA;
		RegistrationInfo registrationInfo_QQVGA;
		RegistrationInfo registrationInfo_QVGA;
		RegistrationInfo registrationInfo_VGA;

		double zpps;
		int zpd;
		double dcrcdist;

		int rgbRegXRes;
		int rgbRegYRes;
		int cmosVGAOutputXRes;
		int sensorWinOffsetX;
		int sensorWinOffsetY;
		int rgbRegXValScale;
		int s2dPelConst;
		double s2dConstOffset;

	} params1080;
} DepthUtilsSensorCalibrationInfo;

#endif
		
namespace kinect_interface_primesense {
  
  struct CalibrationData {
  public:
    DepthUtilsSensorCalibrationInfo m_blob;
    uint16_t m_pRegistrationTable_QQVGA[160*120*2];
    uint16_t m_pRegistrationTable_QVGA[320*240*2];
    uint16_t m_pRegistrationTable_VGA[640*480*2];

    uint16_t m_pDepthToShiftTable_QQVGA[MAX_Z+1];
    uint16_t m_pDepthToShiftTable_QVGA[MAX_Z+1];
    uint16_t m_pDepthToShiftTable_VGA[MAX_Z+1];

    PadInfo* m_pPadInfo;
  };
  
};  // namespace kinect_interface_primesense
