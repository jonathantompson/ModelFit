#include <algorithm>
#include <stdlib.h>
#include <stdexcept>
#include "fastlz/fastlz_helper.h"
extern "C" {
  #include "fastlz/fastlz.h"
}

namespace jtil {
namespace fastlz {

  uint32_t FastlzHelper::Compress(uint8_t* src, uint8_t* dst, 
      const uint32_t data_size_bytes, const uint32_t level) {

    if (level < 1 || level > 2) {
      throw std::runtime_error("UCLHelper::inPlaceCompress() - ERROR: "
        "comrpression level is outside the valid range.");
    }

    int compressed_length = fastlz_compress_level(level, src, 
      data_size_bytes, dst);

    return static_cast<uint32_t>(compressed_length);
  }

  uint32_t FastlzHelper::DeCompress(uint8_t* src, uint8_t* dst, 
      const uint32_t data_size_bytes, const uint32_t max_dst_size_bytes) {

    int compressed_length = fastlz_decompress(src, 
      data_size_bytes, dst, max_dst_size_bytes);

    return static_cast<uint32_t>(compressed_length);
  }

  const uint32_t FastlzHelper::calcInPlaceCompressSizeRequirement(
    const uint32_t data_size_bytes) {
    const uint32_t overhead = calcCompressOverhead(data_size_bytes);
    return overhead + 2 * data_size_bytes;
  }

  const uint32_t FastlzHelper::calcInPlaceDeCompressiSizeRequirement(
    const uint32_t data_size_bytes) {
    const uint32_t overhead = calcDeCompressOverhead(data_size_bytes);
    return overhead + data_size_bytes;
  }
  
  // For both compression and decompression:
  // "The output buffer must be at least 5% larger than the input buffer  
  // and can not be smaller than 66 bytes."
  const uint32_t FastlzHelper::calcCompressOverhead(const uint32_t size) {
    return (uint32_t)floorf(std::max<float>((float)size * 1.05f, 66) + 1);
  }

  const uint32_t FastlzHelper::calcDeCompressOverhead(const uint32_t size) {
    return (uint32_t)floorf(std::max<float>((float)size * 1.05f, 66) + 1);
  }

}  // namespace fastlz
}  // namespace jtil
