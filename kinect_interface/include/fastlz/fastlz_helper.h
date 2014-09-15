//
//  fastlz_helper.h
//
//  Created by Jonathan Tompson on 1/5/13.
//
//  Wrapper to make using fastlz (which is a compression and 
//  decompression library) easier.  Compression ratios are OK, but it is fast!
//
//  WARNING ALLOCATED ARRAY MUST BE LONGER THAN data_size_bytes. Use 
//  calcInPlaceDe/CompressSizeRequirement to determine required size.
// 

#pragma once

#include <mutex>
#include "math/math_types.h"

namespace jtil {
namespace fastlz {

  class FastlzHelper {
  public:
    static uint32_t Compress(uint8_t* src, uint8_t* dst, 
      const uint32_t src_size_bytes, const uint32_t level);  // level 1-2
    static uint32_t DeCompress(uint8_t* src, uint8_t* dst, 
      const uint32_t src_size_bytes, const uint32_t max_dst_size_bytes);
    static uint32_t Decompress(uint8_t* src, uint8_t* dst, 
      const uint32_t data_size_bytes);  // level 1-2

    static const uint32_t calcInPlaceCompressSizeRequirement(
      const uint32_t data_size_bytes);
    static const uint32_t calcInPlaceDeCompressiSizeRequirement(
      const uint32_t data_size_bytes);

  private:
    FastlzHelper() { }
    ~FastlzHelper() { }

    static const uint32_t calcCompressOverhead(const uint32_t size);
    static const uint32_t calcDeCompressOverhead(const uint32_t size);
  };

};  // namespace fastlz
};  // namespace jtil
