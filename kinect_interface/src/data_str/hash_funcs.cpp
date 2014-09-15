#include <cstring>  // For strlen
#include <string>
#include "data_str/hash_funcs.h"  // for uint

namespace jtil {
namespace data_str {
  uint32_t HashUInt(const uint32_t size, const uint32_t& key) {
    // From here:
    // http://code.google.com/p/smhasher/wiki/MurmurHash3
    // No idea if it's any good!
    uint32_t h = key;
    h ^= h >> 16;
    h *= 0x85ebca6b;
    h ^= h >> 13;
    h *= 0xc2b2ae35;
    h ^= h >> 16;
    return h % size;
  }

  uint32_t HashInt(const uint32_t size, const int32_t& key) {
    uint32_t h = (uint32_t)key;
    h ^= h >> 16;
    h *= 0x85ebca6b;
    h ^= h >> 13;
    h *= 0xc2b2ae35;
    h ^= h >> 16;
    return h % size;
  }

  uint32_t HashUInt(const uint32_t size, const uint64_t& key) {
    // From here:
    // http://code.google.com/p/smhasher/wiki/MurmurHash3
    // No idea if it's any good!
    uint64_t h = key;
    h ^= h >> 33;
    h *= 0xff51afd7ed558ccd;
    h ^= h >> 33;
    h *= 0xc4ceb9fe1a85ec53;
    h ^= h >> 33;
    return (uint32_t)(h % (uint64_t)size);
  }

  uint32_t HashInt(const uint32_t size, const int64_t& key) {
    uint64_t h = (uint64_t)key;
    h ^= h >> 33;
    h *= 0xff51afd7ed558ccd;
    h ^= h >> 33;
    h *= 0xc4ceb9fe1a85ec53;
    h ^= h >> 33;
    return (uint32_t)(h % (uint64_t)size);
  }

  // Dynamic string hash version.  This example is from: 
  // http://www.altdevblogaday.com/2011/10/27/quasi-compile-time-string-hashing
  uint32_t HashString(const uint32_t size, const std::string& key) {
    const char* c_str = key.c_str();
    const size_t length = strlen(c_str) + 1;
    uint32_t hash = 2166136261u;
    for (size_t i = 0; i < length; ++i) {
      hash ^= *c_str++;
      hash *= 16777619u;
    }

    return hash % (size - 1);
  }


}  // namespace data_str 
}  // namespace jtil