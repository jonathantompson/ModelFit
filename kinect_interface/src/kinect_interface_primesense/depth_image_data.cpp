#include <random>
#include <thread>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include "kinect_interface_primesense/depth_image_data.h"
#include "file_io/file_io.h"

#if defined(WIN32) || defined(_WIN32)  
  #define snprintf _snprintf_s
  #pragma warning( disable : 4099 )
#endif

namespace kinect_interface_primesense {

  void GetDataFileNames(const uint32_t max_kinects, const char* im_dir, 
    const char* calib_im_dir, 
    jtil::data_str::VectorManaged<char*>* depth_files, 
    jtil::data_str::VectorManaged<char*>* rgb_files,
    jtil::math::Float4x4* camera_view) {
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
      if (!jtil::file_io::fileExists(full_path)) {
        camera_view[k].identity();
        std::cout << "**********************************" << std::endl;
        std::cout << "WARNING: CALIBRATION DATA MISSING!" << std::endl;
        std::cout << "**********************************" << std::endl;
      } else {
        jtil::file_io::LoadArrayFromFile<float>(camera_view[k].m, 16, full_path);
      }
    }
  }

}  // namespace kinect_interface_primesense
