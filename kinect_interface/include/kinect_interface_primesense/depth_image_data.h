//
//  depth_image_data.h
//
//  Created by Jonathan Tompson on 3/1/13.
//

#pragma once

#include <string>
#include <iostream>
#include <fstream>
#include "data_str/vector.h"
#include "data_str/vector_managed.h"
#include "math/math_types.h"

namespace kinect_interface_primesense {
   
  // DepthImageData used really only when generating the decision tree
  // for the HandDetector
  struct DepthImageData {
    int16_t* image_data;
    uint8_t* label_data;
    uint8_t* rgb_data;
    int32_t num_images;
    int32_t im_width;
    int32_t im_height;
    char** filenames;
  };

  void GetDataFileNames(const uint32_t max_kinects, const char* im_dir, 
    const char* calib_im_dir,
    jtil::data_str::VectorManaged<char*>* depth_files, 
    jtil::data_str::VectorManaged<char*>* rgb_files,
    jtil::math::Float4x4* camera_view);

};  // namespace kinect_interface_primesense
