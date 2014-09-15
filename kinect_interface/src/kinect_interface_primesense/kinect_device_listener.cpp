#include <string>
#include <sstream>
#include <iostream>
#include "kinect_interface_primesense/kinect_device_listener.h"
#include "kinect_interface_primesense/kinect_interface_primesense.h"

namespace kinect_interface_primesense {
  void KinectDeviceListener::onDeviceStateChanged(
    const openni::DeviceInfo* pInfo, openni::DeviceState errorState) {
    std::string uri = pInfo->getUri();
    std::cout << "Device " << uri << " state change:" << errorState << std::endl;
    // Find an open KinectInterface that shares this URI
    for (uint32_t i = 0; i < KinectInterfacePrimesense::open_kinects_.size(); i++) {
      if (KinectInterfacePrimesense::open_kinects_[i]->device_uri_ == uri) {
        if (errorState != 0) {
          std::stringstream ss;
          ss << "Device " << i << " is in error state: " << errorState;
          throw std::runtime_error(ss.str());
        } else {
          // Nothing, everything is OK
        }
      }
    }
  }

  void KinectDeviceListener::onDeviceDisconnected(const openni::DeviceInfo* pInfo) {
    std::string uri = pInfo->getUri();
    std::cout << "Device " << uri << " disconnected" << std::endl;
    // Find an open KinectInterface that shares this URI
    for (uint32_t i = 0; i < KinectInterfacePrimesense::open_kinects_.size(); i++) {
      if (KinectInterfacePrimesense::open_kinects_[i]->device_uri_ == uri) {
        std::stringstream ss;
        ss << "Device " << i << " disconnected!";
        throw std::runtime_error(ss.str());
      }
    }
  }

  KinectDeviceListener::KinectDeviceListener() {

  }

  KinectDeviceListener::~KinectDeviceListener() {

  }

}  // namespace kinect_interface_primesense


