//
//  kinect_device_listener.h
// 

#pragma once

#include "OpenNI.h"
		
namespace kinect_interface_primesense {
  class KinectDeviceListener : public openni::OpenNI::DeviceStateChangedListener,
    public openni::OpenNI::DeviceDisconnectedListener {
  public:
    KinectDeviceListener();
    ~KinectDeviceListener();

    virtual void onDeviceStateChanged(const openni::DeviceInfo* pInfo, 
      openni::DeviceState errorState);
	  virtual void onDeviceDisconnected(const openni::DeviceInfo* pInfo);

  private:
  };

};  // namespace kinect_interface_primesense
