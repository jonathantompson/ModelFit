#include <mutex>
#include <thread>
#include <string>
#include <sstream>
#include <fstream>
#include <iostream>
#include <limits>
#include "kinect_interface_primesense/kinect_interface_primesense.h"
#include "kinect_interface_primesense/kinect_device_listener.h"
#include "kinect_interface_primesense/open_ni_funcs.h"
#include "clk/clk.h"
#include "image_util/image_util.h"
#include "threading/callback.h"
#include "threading/thread.h"
#include "threading/thread_pool.h"

#include "OpenNI.h"
#include "OniCAPI.h"
#include "PS1080.h"

#define SAFE_DELETE(x) if (x != NULL) { delete x; x = NULL; }
#define SAFE_DELETE_ARR(x) if (x != NULL) { delete[] x; x = NULL; }

#define MIN_NUM_CHUNKS(data_size, chunk_size)	((((data_size)-1) / (chunk_size) + 1))
#define MIN_CHUNKS_SIZE(data_size, chunk_size)	(MIN_NUM_CHUNKS(data_size, chunk_size) * (chunk_size))

#ifdef _WIN32
#ifndef snprintf
#define snprintf _snprintf_s
#endif
#endif

using std::cout;
using std::endl;
using std::thread;
using jtil::math::Float3;
using jtil::clk::Clk;
using namespace jtil::data_str;
using namespace jtil::threading;
using openni::PixelFormat;
using openni::DepthPixel;

namespace kinect_interface_primesense {

  bool KinectInterfacePrimesense::openni_init_ = false;
  std::mutex KinectInterfacePrimesense::openni_static_lock_;
  uint32_t KinectInterfacePrimesense::openni_devices_open_ = 0;
  jtil::clk::Clk KinectInterfacePrimesense::shared_clock_;
  KinectDeviceListener* KinectInterfacePrimesense::device_listener_ = NULL;
  Vector<KinectInterfacePrimesense*> KinectInterfacePrimesense::open_kinects_;

  KinectInterfacePrimesense::KinectInterfacePrimesense(const char* device_uri) {
    device_initialized_ = false;
    device_ = NULL;
    pts_world_ = NULL;
    pts_uvd_ = NULL;
    labels_ = NULL;
    depth_1mm_ = NULL;
    registered_rgb_ = NULL;
    openni_funcs_ = NULL;
    depth_dim_.zeros();
    rgb_dim_.zeros();
    ir_dim_.zeros();
    for (uint32_t i = 0; i < NUM_STREAMS; i++) {
      streams_[i] = NULL;
      frames_[i] = NULL;
    }

    depth_frame_number_ = 0;
    rgb_frame_number_ = 0;
    ir_frame_number_ = 0;
    tp_ = NULL;

    init(device_uri);

    //  Now spawn the Kinect Update Thread
    kinect_running_ = true;
    Callback<void>* threadBody = MakeCallableOnce(
      &KinectInterfacePrimesense::kinectUpdateThread, this);
    kinect_thread_ = MakeThread(threadBody);

    device_initialized_ = true;
    // Increment the devices counter
    openni_static_lock_.lock();
    openni_devices_open_++;
    open_kinects_.pushBack(this);
    openni_static_lock_.unlock();
  }

  KinectInterfacePrimesense::~KinectInterfacePrimesense() {
    // Stop the openNI device
    for (uint32_t i = 0; i < NUM_STREAMS; i++) {
      if (frames_[i]) {
        frames_[i]->release();
      }
      if (streams_[i]) {
        streams_[i]->stop();
        streams_[i]->destroy();
      }
    }
    if (device_) {
      device_->close();
    }

    if (tp_) {
      tp_->stop();
    }
    SAFE_DELETE(tp_);
    SAFE_DELETE(device_listener_);
    if (depth_format_100um_) {
      SAFE_DELETE_ARR(depth_1mm_);
    }
    SAFE_DELETE_ARR(registered_rgb_);
    SAFE_DELETE_ARR(pts_uvd_);
    SAFE_DELETE_ARR(pts_world_);
    SAFE_DELETE_ARR(labels_);
    SAFE_DELETE(openni_funcs_);
    for (uint32_t i = 0; i < NUM_STREAMS; i++) {
      SAFE_DELETE(frames_[i]);
      SAFE_DELETE(streams_[i]);
    }
    SAFE_DELETE(device_);

    // Decrement the devices counter
    if (device_initialized_) {
      openni_static_lock_.lock();
      openni_devices_open_--;
      openni_static_lock_.unlock();
    }

    // Request a shutdown of the openNI framework
    shutdownOpenNIStatic();
  }

  // ************************************************************
  // Initialization
  // ************************************************************

  void KinectInterfacePrimesense::initOpenNIStatic() {
    // Initialize OpenNI static methods
    openni_static_lock_.lock();
    if (!openni_init_) {
      openni::Status rc = openni::OpenNI::initialize();
      if (rc == openni::STATUS_OK) {
        openni_init_ = true;
        device_listener_ = new KinectDeviceListener();
        openni::OpenNI::addDeviceDisconnectedListener(device_listener_);
	      openni::OpenNI::addDeviceStateChangedListener(device_listener_);
      } else {
        std::stringstream ss;
        ss << "KinectInterfacePrimesense::initOpenNI() - ERROR: ";
        ss << "Failed to initilize OpenNI API: ";
        ss << openni::OpenNI::getExtendedError();
        openni_static_lock_.unlock();
        throw std::runtime_error(ss.str());
      }
    }
    openni_static_lock_.unlock();
  }

  void KinectInterfacePrimesense::shutdownOpenNIStatic() {
    openni_static_lock_.lock();
    if (openni_devices_open_ == 0) {
      // Only shutdown if there aren't any devices left open
      std::cout << "Shutting down the OpenNI framework..." << std::endl;
      openni::OpenNI::shutdown();
    }
    openni_static_lock_.unlock();
  }

  void KinectInterfacePrimesense::checkOpenNIRC(int rc, const char* error_msg) {
    if (rc != openni::STATUS_OK) {
      std::stringstream ss;
      ss << error_msg << ": " << openni::OpenNI::getExtendedError();
      shutdownOpenNIStatic();
      throw std::runtime_error(ss.str());
    }
  }

  std::string KinectInterfacePrimesense::formatToString(const int mode) {
    switch (mode) {
    case PixelFormat::PIXEL_FORMAT_DEPTH_1_MM:
      return "PIXEL_FORMAT_DEPTH_1_MM";
    case PixelFormat::PIXEL_FORMAT_DEPTH_100_UM:
      return "PIXEL_FORMAT_DEPTH_100_UM";
    case PixelFormat::PIXEL_FORMAT_SHIFT_9_2:
      return "PIXEL_FORMAT_SHIFT_9_2";
    case PixelFormat::PIXEL_FORMAT_SHIFT_9_3:
      return "PIXEL_FORMAT_SHIFT_9_3";
    case PixelFormat::PIXEL_FORMAT_RGB888:
      return "PIXEL_FORMAT_RGB888";
    case PixelFormat::PIXEL_FORMAT_YUV422:
      return "PIXEL_FORMAT_YUV422";
    case PixelFormat::PIXEL_FORMAT_GRAY8:
      return "PIXEL_FORMAT_GRAY8";
    case PixelFormat::PIXEL_FORMAT_GRAY16:
      return "PIXEL_FORMAT_GRAY16";
    case PixelFormat::PIXEL_FORMAT_JPEG:
      return "PIXEL_FORMAT_JPEG";
    default: 
      shutdownOpenNIStatic();
      throw std::runtime_error("KinectInterfacePrimesense::formatToString() - ERROR: "
        "Format enum not recognized!");
    }
  }

  void KinectInterfacePrimesense::printMode(const openni::VideoMode& mode) {
    std::cout << "Res: " << mode.getResolutionX() << "x";
    std::cout << mode.getResolutionY() << ", fps = " << mode.getFps();
    std::cout << ", format = " << formatToString(mode.getPixelFormat());
    std::cout << std::endl;
  }

  void KinectInterfacePrimesense::init(const char* device_uri) {
    initOpenNI(device_uri);

    // Create a thread pool for parallelizing the UVD to depth calculations
    uint32_t num_threads = KINECT_INTERFACE_NUM_WORKER_THREADS;
    tp_ = new jtil::threading::ThreadPool(num_threads);
    if (KINECT_INTERFACE_NUM_CONVERTER_THREADS > 1) {
      uint32_t n_pixels = depth_dim_[0] * depth_dim_[1];
      uint32_t n_pixels_per_thread = 1 + n_pixels / num_threads;  // round up
      for (uint32_t i = 0; i < num_threads; i++) {
        uint32_t start = i * n_pixels_per_thread;
        uint32_t end = std::min<uint32_t>(((i + 1) * n_pixels_per_thread) - 1,
          n_pixels - 1);
        pts_world_thread_cbs_.pushBack(MakeCallableMany(
          &KinectInterfacePrimesense::convertDepthToWorld, this, start, end));
        rgb_thread_cbs_.pushBack(MakeCallableMany(
          &KinectInterfacePrimesense::convertRGBToDepth, this, start, end));
      }
    }

    if (device_uri) {
      cout << "Finished initializing device " << device_uri << "..." << endl;
    } else {
      cout << "Finished initializing openNI device..." << endl;
    }
  }

  void KinectInterfacePrimesense::initOpenNI(const char* device_uri) {
    initOpenNIStatic();

    if (device_uri) {
      cout << "Initializing device " << device_uri << "..." << endl;
    } else {
      cout << "Initializing first avaliable openNI device..." << endl;
    }

    // Open a connection to the OpenNI device
    const char* deviceURI = openni::ANY_DEVICE;
    if (device_uri != NULL) {
      deviceURI = device_uri;
    }
    device_ = new openni::Device();
    checkOpenNIRC(device_->open(deviceURI), "Failed to connect to device");
    const openni::DeviceInfo& info = device_->getDeviceInfo();
    device_uri_ = info.getUri();

     // Note: XYZ (real world values are ALL wrong when image registration is
    // turned on).  It looks OK for a single Kinect, but the space is warped.
    // This is an OpenNI bug:
    setCropDepthToRGB(false);
    setDepthColorSync(false);

    // Open a connection to the device's depth channel
    initDepth();

    const int depth_size = depth_dim_[0] * depth_dim_[1];
    labels_ = new uint8_t[depth_size];
    pts_world_ = new float[3 * depth_size];
    pts_uvd_ = new float[3 * depth_size];
    registered_rgb_ = new uint8_t[3 * depth_size];

    // Open a connection to the device's rgb channel
    bool sync_ir_stream_ = false;
    initRGB(!sync_ir_stream_);
    initIR(sync_ir_stream_);

    bool flip_image_ = true;
    setFlipImage(flip_image_);

    std::this_thread::sleep_for(std::chrono::milliseconds(1));

    // Update the OpenNIFuncs constants
    float depth_hfov = streams_[DEPTH_STREAM]->getHorizontalFieldOfView();
    float depth_vfov = streams_[DEPTH_STREAM]->getVerticalFieldOfView();
    openni_funcs_ = new OpenNIFuncs(depth_dim_[0], depth_dim_[1],
      depth_hfov, depth_vfov, openni_devices_open_);

    // Check OpenNI's math:
    //uint64_t zpd; 
    //streams_[DEPTH_STREAM]->getProperty(XN_STREAM_PROPERTY_ZERO_PLANE_DISTANCE, &zpd); 
    //double zpps; 
    //streams_[DEPTH_STREAM]->getProperty(XN_STREAM_PROPERTY_ZERO_PLANE_PIXEL_SIZE, &zpps); 
    //double focal_length_cmos = (double)zpd / zpps;
    //// http://en.wikipedia.org/wiki/Angle_of_view FOV calculation
    //double FOVh = 2.0 * atan( ( 1280.0 * 0.5 ) / focal_length_cmos );
    //double FOVv = 2.0 * atan( ( 960.0 * 0.5 ) / focal_length_cmos );
    //std::streamsize old_precision = std::cout.precision();
    //std::cout.precision(15);
    //std::cout << "FOVh = " << FOVh << std::endl;
    //std::cout << "FOVv = " << FOVv << std::endl;
    //std::cout << "depth_hfov = " << depth_hfov << std::endl;
    //std::cout << "depth_vfov = " << depth_vfov << std::endl;
    //std::cout.precision(old_precision);

    std::cout << "Finished initializaing OpenNI device" << std::endl;
  }

  openni::VideoMode KinectInterfacePrimesense::findMaxResYFPSMode(
    const openni::SensorInfo& sensor, const int required_format) const {
      const openni::Array<openni::VideoMode>& modes = 
        sensor.getSupportedVideoModes();
      int ibest = -1;
#if defined(DEBUG) || defined(_DEBUG)
      std::cout << "Supported Modes:" << std::endl;
#endif
      for (int i = 0; i < modes.getSize(); i++) {
        const openni::VideoMode& mode = modes[i];
#if defined(DEBUG) || defined(_DEBUG)
        printMode(mode);
#endif
        PixelFormat format = mode.getPixelFormat();
        int res_x = mode.getResolutionX();
        int res_y = mode.getResolutionY();
        int fps = mode.getFps();
        if (format == required_format) {
          if (ibest == -1 || res_y > modes[ibest].getResolutionY()) {
            ibest = i;
          } else if (res_y == modes[ibest].getResolutionY() && 
            fps > modes[ibest].getFps()){
              ibest = i;
          }
        }
      }
      if (ibest == -1) {
        shutdownOpenNIStatic();
        throw std::runtime_error("KinectInterfacePrimesense::findMaxResYFPSMode() - "
          "ERROR: Couldn't find a good format!");
      }
      return modes[ibest];
  }

  void KinectInterfacePrimesense::findDevices(VectorManaged<char*>& devices) {
    initOpenNIStatic();
    openni::Array<openni::DeviceInfo> deviceInfoList;
    openni::OpenNI::enumerateDevices(&deviceInfoList);
    std::cout << "KinectInterfacePrimesense::findDevices(): Connected devices: ";
    std::cout << std::endl;
    for (int32_t i = 0; i < deviceInfoList.getSize(); i++) {
      std::cout << "    " << i << ": name = " << deviceInfoList[i].getName();
      std::cout << ", uri = " << deviceInfoList[i].getUri() << std::endl;
      const char* cur_uri_src = deviceInfoList[i].getUri();
      char* cur_uri_dst = new char[strlen(cur_uri_src) + 1];
      strncpy(cur_uri_dst, cur_uri_src, strlen(cur_uri_src) + 1);
      devices.pushBack(cur_uri_dst);
    }
  }


  openni::VideoMode KinectInterfacePrimesense::findMatchingMode(
    const openni::SensorInfo& sensor, const jtil::math::Int2& dim, 
    const int fps, const int format) const {
      const openni::Array<openni::VideoMode>& modes = 
        sensor.getSupportedVideoModes();
      int ibest = -1;
#if defined(DEBUG) || defined(_DEBUG)
      std::cout << "Supported Modes:" << std::endl;
#endif
      for (int i = 0; i < modes.getSize(); i++) {
        const openni::VideoMode& mode = modes[i];
#if defined(DEBUG) || defined(_DEBUG)
        printMode(mode);
#endif
        PixelFormat cur_format = mode.getPixelFormat();
        int res_x = mode.getResolutionX();
        int res_y = mode.getResolutionY();
        int cur_fps = mode.getFps();
        if (cur_format == format && res_x == dim[0] && res_y == dim[1] &&
          fps == fps) {
            ibest = i;
        }
      }
      if (ibest == -1) {
        shutdownOpenNIStatic();
        throw std::runtime_error("KinectInterfacePrimesense::findMatchingMode() - "
          "ERROR: Couldn't find a matching format!");
      }
      return modes[ibest];
  }

  void KinectInterfacePrimesense::initDepth() {
    streams_[DEPTH_STREAM] = new openni::VideoStream();
    checkOpenNIRC(streams_[DEPTH_STREAM]->create(*device_, openni::SENSOR_DEPTH),
      "Failed to connect to device depth channel");

    // Find the supported mode with the highest depth resolution.  If more
    // than one format has the max resolution, choose the highest fps mode
    const openni::SensorInfo& depth_sensor_info = 
      streams_[DEPTH_STREAM]->getSensorInfo();
#if defined(DEBUG) || defined(_DEBUG)
    std::cout << "Depth ";
#endif
    openni::VideoMode mode = findMaxResYFPSMode(depth_sensor_info,
      PixelFormat::PIXEL_FORMAT_DEPTH_1_MM);
    std::cout << "Setting Depth mode: ";
    printMode(mode);
    checkOpenNIRC(streams_[DEPTH_STREAM]->setVideoMode(mode), 
      "Failed to set depth video mode");
    depth_fps_setting_ = mode.getFps();

    streams_[DEPTH_STREAM]->setProperty(XN_STREAM_PROPERTY_CLOSE_RANGE, true);

    // Now retrieve the current mode to make sure everything went OK.
    openni::VideoMode depth_mode = streams_[DEPTH_STREAM]->getVideoMode();
    if (depth_mode.getPixelFormat() != PixelFormat::PIXEL_FORMAT_DEPTH_1_MM) {
      throw std::runtime_error("KinectInterfacePrimesense::init() - ERROR: "
        "Depth stream is not a 16 bit grayscale format!");
    }
    depth_format_100um_ = false;
    depth_dim_.set(depth_mode.getResolutionX(), depth_mode.getResolutionY());

    checkOpenNIRC(streams_[DEPTH_STREAM]->start(), "Failed to start depth stream");
    if (!streams_[DEPTH_STREAM]->isValid()) {
      shutdownOpenNIStatic();
      throw std::runtime_error("KinectInterfacePrimesense::init() - ERROR: "
        "Depth stream is not valid");
    }

    frames_[DEPTH_STREAM] = new openni::VideoFrameRef();

    if (depth_format_100um_) {
      depth_1mm_ = new uint16_t[depth_dim_[0] * depth_dim_[1]];
    }
  }

  void KinectInterfacePrimesense::initRGB(const bool start) {
    streams_[RGB_STREAM] = new openni::VideoStream();
    checkOpenNIRC(streams_[RGB_STREAM]->create(*device_, openni::SENSOR_COLOR),
      "Failed to connect to device rgb channel");

    // Find a resolution mode to match the depth mode
    const openni::SensorInfo& rgb_sensor_info = 
      streams_[RGB_STREAM]->getSensorInfo();
#if defined(DEBUG) || defined(_DEBUG)
    std::cout << "RGB_STREAM ";
#endif
    openni::VideoMode mode = findMatchingMode(rgb_sensor_info, depth_dim_,
      depth_fps_setting_, PixelFormat::PIXEL_FORMAT_RGB888);
    std::cout << "Setting RGB_STREAM mode: ";
    printMode(mode);
    checkOpenNIRC(streams_[RGB_STREAM]->setVideoMode(mode), 
      "Failed to set rgb video mode");
    rgb_fps_setting_ = mode.getFps();

    // Now retrieve the current mode to make sure everything went OK.
    openni::VideoMode rgb_mode = streams_[RGB_STREAM]->getVideoMode();
    if (rgb_mode.getPixelFormat() != PixelFormat::PIXEL_FORMAT_RGB888) {
      throw std::runtime_error("KinectInterfacePrimesense::init() - ERROR: "
        "RGB_STREAM stream is not a 24 bit rgb format!");
    }
    rgb_dim_.set(rgb_mode.getResolutionX(), rgb_mode.getResolutionY());

    if (start) {
      checkOpenNIRC(streams_[RGB_STREAM]->start(), "Failed to start rgb stream");
      if (!streams_[RGB_STREAM]->isValid()) {
        shutdownOpenNIStatic();
        throw std::runtime_error("KinectInterfacePrimesense::init() - ERROR: "
          "RGB_IR_STREAM stream is not valid");
      }
    }

    frames_[RGB_STREAM] = new openni::VideoFrameRef(); 
  }

  void KinectInterfacePrimesense::initIR(const bool start) {
    bool has_ir = start && device_->hasSensor(openni::SENSOR_IR);
    if (!has_ir) {
      streams_[IR_STREAM] = NULL;
      frames_[IR_STREAM] = NULL;
      return;
    }
    streams_[IR_STREAM] = new openni::VideoStream();
    checkOpenNIRC(streams_[IR_STREAM]->create(*device_, openni::SENSOR_IR),
      "Failed to connect to device IR channel");

    // Find a resolution mode to match the depth mode
    const openni::SensorInfo& ir_sensor_info = 
      streams_[IR_STREAM]->getSensorInfo(); 
#if defined(DEBUG) || defined(_DEBUG)
    std::cout << "IR_STREAM ";
#endif
    openni::VideoMode mode = findMatchingMode(ir_sensor_info, depth_dim_,
      depth_fps_setting_, PixelFormat::PIXEL_FORMAT_GRAY16);
    std::cout << "Setting IR_STREAM mode: ";
    printMode(mode);
    checkOpenNIRC(streams_[IR_STREAM]->setVideoMode(mode), 
      "Failed to set ir video mode");
    ir_fps_setting_ = mode.getFps();

    // Now retrieve the current mode to make sure everything went OK.
    openni::VideoMode ir_mode = streams_[IR_STREAM]->getVideoMode();
    if (ir_mode.getPixelFormat() != PixelFormat::PIXEL_FORMAT_GRAY16) {
      throw std::runtime_error("KinectInterfacePrimesense::init() - ERROR: "
        "IR_STREAM stream is not a 24 bit rgb format!");
    }
    ir_dim_.set(ir_mode.getResolutionX(), ir_mode.getResolutionY());

    if (start) {
      checkOpenNIRC(streams_[IR_STREAM]->start(), "Failed to start IR stream");
      if (!streams_[IR_STREAM]->isValid()) {
        shutdownOpenNIStatic();
        throw std::runtime_error("KinectInterfacePrimesense::init() - ERROR: "
          "RGB_IR_STREAM stream is not valid");
      }
    }

    frames_[IR_STREAM] = new openni::VideoFrameRef(); 
  }

  void KinectInterfacePrimesense::setCropDepthToRGB(const bool crop_depth_to_rgb) {
    if (crop_depth_to_rgb) {
      device_->setImageRegistrationMode(openni::IMAGE_REGISTRATION_DEPTH_TO_COLOR);
    } else {
      device_->setImageRegistrationMode(openni::IMAGE_REGISTRATION_OFF);
    }
  }

  void KinectInterfacePrimesense::setFlipImage(const bool flip_image) {
    for (uint32_t i = 0; i < NUM_STREAMS; i++) {
      if (i == RGB_STREAM && sync_ir_stream_) {
        continue;
      }
      if (i == IR_STREAM && !sync_ir_stream_) {
        continue;
      }
      // Note: mirroring fails on the Kinect, so it shouldn't be a termination
      // error condition.
      openni::Status rc = streams_[i]->setMirroringEnabled(flip_image);
    }
  }

  void KinectInterfacePrimesense::setDepthColorSync(const bool depth_color_sync) {
    checkOpenNIRC(device_->setDepthColorSyncEnabled(depth_color_sync),
      "Failed to set depth/color sync");
  }

  void KinectInterfacePrimesense::kinectUpdateThread() {
    SetThreadName("KinectInterfacePrimesense::kinectUpdateThread()");

    while (kinect_running_) {
      // Wait for all streams individually.  We need Depth and RGB_IR_STREAM frames to be
      // consistent in time so we should for both rather than process each one
      // independantly
      bool a_frame_changed = false;
      bool stream_ready[NUM_STREAMS];
      for (uint32_t i = 0; i < NUM_STREAMS; i++) {
        stream_ready[i] = false;
        if (i == RGB_STREAM && sync_ir_stream_) {
          continue;
        }
        if (i == IR_STREAM && !sync_ir_stream_) {
          continue;
        }
        int changedIndex;
        openni::Status rc = openni::OpenNI::waitForAnyStream(&streams_[i], 1, 
          &changedIndex, OPENNI_WAIT_TIMEOUT);
        if (changedIndex == 0 && rc == openni::Status::STATUS_OK) {
          stream_ready[i] = true;
          a_frame_changed = true;
        }
      }

      if (!a_frame_changed) {
        continue;  // Happens when USB bus is timing out I think
      }

      data_lock_.lock();

      for (uint32_t i = 0; i < NUM_STREAMS; i++) {
        if (stream_ready[i]) {
          streams_[i]->readFrame(frames_[i]);
          switch (i) {
          case DEPTH_STREAM:
            depth_frame_number_++;
            depth_frame_time_ = 1e-6 * (double)frames_[i]->getTimestamp();
            break;
          case RGB_STREAM:
            rgb_frame_number_++;
            break;
          case IR_STREAM:
            ir_frame_number_++;
            break;
          }
        }
      }

      // Check if we're running again.  Someone may have shut down the thread
      // while we were waiting on the OpenNI library
      if (!kinect_running_) {
        data_lock_.unlock();
        break;
      }

      if (stream_ready[DEPTH_STREAM]) {
        if (depth_format_100um_) {
          uint16_t* depth_src = (uint16_t*)frames_[DEPTH_STREAM]->getData(); 
          for (int32_t i = 0; i < depth_dim_[0] * depth_dim_[1]; i++) {
            depth_1mm_[i] = depth_src[i] / 10;
          }
        } else {
          depth_1mm_ = (uint16_t*)frames_[DEPTH_STREAM]->getData(); 
        }

        performConversions();

        bool detect_hands, found_hand;
        /*
        GET_SETTING("detect_hands", bool, detect_hands);
        if (detect_hands) {
          found_hand = hand_detector_->findHandLabels((int16_t*)depth_1mm_, 
            pts_world_, HDLabelMethod::HDFloodfill, labels_);
          if (!found_hand) {
            memset(labels_, 0, sizeof(labels_[0]) * src_dim);
          }
        }
        */
        // EDIT: Hand detector removed from this repo
        detect_hands = false;
        found_hand = false;

        if (!detect_hands || !found_hand) {
          memset(labels_, 0, sizeof(labels_[0]) * src_dim);
        }
      }  // if (stream_ready[DEPTH_STREAM]) {
      
      data_lock_.unlock();

      std::this_thread::yield();

    }  // end while (app::App::app_running)
    cout << "kinectUpdateThread shutting down..." << endl;
  }

  void KinectInterfacePrimesense::performConversions() {
    openni_funcs_->ConvertDepthImageToProjective(depth(), pts_uvd_);
    if (depth_format_100um_) {
      for (int32_t i = 2; i < 3 * depth_dim_[0] * depth_dim_[1]; i+=3) {
        pts_uvd_[i] /= 10.0f;
      }
    }

    if (KINECT_INTERFACE_NUM_CONVERTER_THREADS == 1) {
      convertDepthToWorld(0, src_dim-1);
      convertRGBToDepth(0, src_dim-1);
    } else {
      executeThreadCallbacks(tp_, pts_world_thread_cbs_);
      executeThreadCallbacks(tp_, rgb_thread_cbs_);
    }
  }

  void KinectInterfacePrimesense::executeThreadCallbacks(jtil::threading::ThreadPool* tp, 
    jtil::data_str::VectorManaged<jtil::threading::Callback<void>*>& cbs) {
    threads_finished_ = 0;
    for (uint32_t i = 0; i < cbs.size(); i++) {
      tp_->addTask(cbs[i]);
    }
    // Wait for all threads to finish
    std::unique_lock<std::mutex> ul(thread_update_lock_);  // Get lock
    while (threads_finished_ != cbs.size()) {
      not_finished_.wait(ul);
    }
    ul.unlock();  // Release lock
  }

  void KinectInterfacePrimesense::convertDepthToWorld(const uint32_t start, 
    const uint32_t end) {
    // There are two ways to do this: one is with the 
    // openni::CoordinateConverter class, but looking through the source on
    // github it looks expensive.  The other is digging into the c src directly
    // and defining our own version.
    float* pts_uvd_start = &pts_uvd_[start*3];
    float* pts_world_start = &pts_world_[start*3];
    const uint32_t count = end - start + 1;
    openni_funcs_->convertDepthToWorldCoordinates(pts_uvd_start, 
      pts_world_start, count);

    //const uint16_t* d = depth();
    //for (uint32_t i = start; i <= end; i++) {
    //  uint32_t u = i % depth_dim_[0];
    //  uint32_t v = i / depth_dim_[0];
    //  openni::CoordinateConverter::convertDepthToWorld(*streams_[DEPTH_STREAM], u, v, 
    //    d[i], &pts_world_[i*3], &pts_world_[i*3+1], &pts_world_[i*3+2]);
    //}

    std::unique_lock<std::mutex> ul(thread_update_lock_);
    threads_finished_++;
    not_finished_.notify_all();  // Signify that all threads might have finished
    ul.unlock();
  }

  void KinectInterfacePrimesense::convertRGBToDepth(const uint32_t start, 
    const uint32_t end) {
    if (!sync_ir_stream_) {
      const uint16_t* d = depth();
      OniStreamHandle dhandle = streams_[DEPTH_STREAM]->_getHandle();
      OniStreamHandle chandle = streams_[RGB_STREAM]->_getHandle();
      int rgb_u;
      int rgb_v;
      uint8_t* rgb = (uint8_t*)frames_[RGB_STREAM]->getData();
      for (uint32_t i = start; i <= end; i++) {
        uint32_t u = i % depth_dim_[0];
        uint32_t v = i / depth_dim_[0];
        // This is the API version (REALLY slow)
        //openni::Status rc = openni::CoordinateConverter::convertDepthToColor(
        //  *streams_[DEPTH_STREAM], *streams_[RGB_IR_STREAM], u, v, d[i], &rgb_u, &rgb_v);
        //if (rc == openni::Status::STATUS_OK) {
        //  int src_index = rgb_v * src_width + rgb_u;
        //  registered_rgb_[i * 3] = rgb[src_index * 3];
        //  registered_rgb_[i * 3 + 1] = rgb[src_index * 3 + 1];
        //  registered_rgb_[i * 3 + 2] = rgb[src_index * 3 + 2];
        //} else {
        //  registered_rgb_[i * 3] = 0;
        //  registered_rgb_[i * 3 + 1] = 0;
        //  registered_rgb_[i * 3 + 2] = 0;
        //}

        //  // My version:
        openni_funcs_->TranslateSinglePixel(u, v, d[i], rgb_u, 
          rgb_v, flip_image_);
        if (d[i] != 0 && rgb_u < src_width && rgb_v < src_height && rgb_u >= 0 &&
          rgb_v >= 0) {
            int src_index = rgb_v * src_width + rgb_u;
            registered_rgb_[i * 3] = rgb[src_index * 3];
            registered_rgb_[i * 3 + 1] = rgb[src_index * 3 + 1];
            registered_rgb_[i * 3 + 2] = rgb[src_index * 3 + 2];
        } else {
          registered_rgb_[i * 3] = 0;
          registered_rgb_[i * 3 + 1] = 0;
          registered_rgb_[i * 3 + 2] = 0;
        }
      }
    } else {
      for (uint32_t i = start; i <= end; i++) {
        registered_rgb_[i * 3] = 255;
        registered_rgb_[i * 3 + 1] = 255;
        registered_rgb_[i * 3 + 2] = 255;
      }
    }

    std::unique_lock<std::mutex> ul(thread_update_lock_);
    threads_finished_++;
    not_finished_.notify_all();  // Signify that all threads might have finished
    ul.unlock();
  }

  const uint8_t* KinectInterfacePrimesense::rgb() const { 
    return (uint8_t*)frames_[RGB_STREAM]->getData(); 
  } 

  const uint8_t* KinectInterfacePrimesense::ir() const { 
    return (uint8_t*)frames_[IR_STREAM]->getData(); 
  } 

  const uint8_t* KinectInterfacePrimesense::registered_rgb() const { 
    return registered_rgb_; 
  } 

  const float* KinectInterfacePrimesense::xyz() const {
    return pts_world_;
  }

  const uint16_t* KinectInterfacePrimesense::depth() const { 
    return (uint16_t*)frames_[DEPTH_STREAM]->getData(); 
  }

  const uint16_t* KinectInterfacePrimesense::depth1mm() const { 
    return depth_1mm_; 
  }

  void KinectInterfacePrimesense::shutdownKinect() {
    data_lock_.lock();
    kinect_running_ = false;
    data_lock_.unlock();
    cout << "kinectUpdateThread shutdown requested..." << endl;
    kinect_thread_.join();
  }

}  // namespace kinect_interface_primesense


