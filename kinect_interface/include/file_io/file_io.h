//
//  file_io.h
//  KinectHands
//
//  Created by Jonathan Tompson on 6/5/12.
//

#pragma once

#include <stdexcept>
#include <iostream>
#include <fstream>
#include "math/math_types.h"

namespace jtil { namespace data_str { template <typename T> class VectorManaged; } }

namespace jtil {
namespace file_io {

  typedef enum {
    DIRECTORY_PATH = 0,
    FILE_PATH = 1,
    UNKNOWN_PATH = 2,
  } PathType;

  template <class T>
  void SaveArrayToFile(const T* arr, const uint32_t size, 
    const std::string& filename) {
    std::ofstream file(filename.c_str(), std::ios::out | std::ios::binary);
    if (!file.is_open()) {
      throw std::runtime_error(std::string("file_io::SaveArrayToFile() - "
        "ERROR: Cannot open output file:") + filename);
    }
    file.write(reinterpret_cast<const char*>(arr), size * sizeof(arr[0]));
    file.flush();
    file.close();
  }
  
  template <class T>
  void LoadArrayFromFile(T* arr, const uint32_t size, 
    const std::string& filename) {
    std::ifstream file(filename.c_str(), std::ios::in | std::ios::binary);
    if (!file.is_open()) {
      throw std::runtime_error(std::string("file_io::LoadArrayFromFile() - "
        "ERROR: Cannot open output file:") + filename);
    }
    std::streampos fsize = file.tellg();
    file.seekg( 0, std::ios::end );
    fsize = file.tellg() - fsize;
    if ((uint32_t)fsize < (uint32_t)sizeof(arr[0]) * size) {
      throw std::runtime_error("jtil::LoadArrayFromFile() - ERROR: "
        "File is too small for data request!");
    }
    file.seekg(0);

    file.read(reinterpret_cast<char*>(arr), size * sizeof(arr[0]));
    file.close();
  }

  bool fileExists(const std::string& filename);
  PathType getPathType(const std::string& path);
  void ls(const std::string& path, jtil::data_str::VectorManaged<char*>& files);
  

};  // namespace file_io
};  // namespace jtil
