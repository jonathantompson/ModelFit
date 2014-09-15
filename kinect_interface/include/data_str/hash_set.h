//
//  hash_set.h
//
//  Created by Jonathan Tompson on 4/26/12.
//
//  This is an open addressing hash table.  Closed hashing with chaining can be
//  slow due to lots of malloc calls.  The probing is just simple linear 
//  probing.  The max load factor is 0.5, in order to maintain reasonable
//  lookup times.
//
//  This is a hash_set so it is a set of keys (use hash_map for key/values).
//  WORKS BEST IF SIZE IS A PRIME NUMBER, ESPECIALLY WITH A MOD HASH FUNCTION
//

#pragma once

#include <stdlib.h>  // For exit()
#include <stdio.h>  // For printf()
#include "math/math_types.h"  // for uint
#include "math/math_base.h"  // for NextPrime

#ifndef NULL
#define NULL 0
#endif

namespace jtil {
namespace data_str {

  template <class TKey>
  class HashSet {
  public:
    // Function pointer for the hash function
    typedef uint32_t (*HashFunc) (const uint32_t size, const TKey& key);

    HashSet(const uint32_t size, HashFunc hash_func);
    ~HashSet();

    inline uint32_t size() const { return size_; }
    inline uint32_t count() const { return count_; }
    inline float load() const { return load_factor_; }
    bool insert(const TKey& key);
    bool insertPrehash(const uint32_t prehash, const TKey& key);
    // REMOVAL MIGHT BE BROKEN AND NEEDS TO BE CHECKED!!!
    // DO I NEED TO MARK FORMALLY OCCUPIED?  PUT TOGETHER A TEST CASE FOR THIS.
    //bool remove(const TKey& key);
    //bool removePrehash(const uint32_t prehash, const TKey& key);
    bool lookup(const TKey& key) const;
    bool lookupPrehash(const uint32_t prehash, const TKey& key) const;
    void clear();  // O(m) - m is the number of buckets (potentially slow)

  private:
    uint32_t size_;
    uint32_t count_;
    TKey* table_;
    bool* bucket_full_;  // Array of booleans telling us if an item exists there
    float load_factor_;
    static const float max_load_;

    HashFunc hash_func_;
    static uint32_t linearProbeFunc(const uint32_t hash, 
      const uint32_t probe_index, const uint32_t size);
    void rehash();

    // Non-copyable, non-assignable.
    HashSet(HashSet&);
    HashSet& operator=(const HashSet&);
  };

  template <class TKey>
  const float HashSet<TKey>::max_load_ = 0.5f;

  template <class TKey>
  uint32_t HashSet<TKey>::linearProbeFunc(const uint32_t hash, 
    const uint32_t probe_index, const uint32_t size) {
    return (hash + probe_index) % size;
  };

  template <class TKey>
  HashSet<TKey>::HashSet(const uint32_t size, HashFunc hash_func) {
    hash_func_ = hash_func;
    load_factor_ = 0;
    count_ = 0;
    size_ = size;
    if (size_ < 1) {
      throw std::runtime_error("HashSet<TKey>::HashSet: size < 1");
    }
    table_ = new TKey[size_];
    bucket_full_ = new bool[size_];
    memset(bucket_full_, false, size_*sizeof(bucket_full_[0]));
  };

  template <class TKey>
  void HashSet<TKey>::rehash() {
    // printf("HashSet rehash\n");
    uint32_t new_size = size_*2;
    new_size = static_cast<uint32_t>(math::NextPrime(new_size));
    TKey* new_table = new TKey[new_size];
    bool* new_bucket_full = new bool[new_size];
    memset(new_bucket_full, false, new_size*sizeof(new_bucket_full[0]));
    // manually insert all the old key/value pairs into the hash table
    for (uint32_t j = 0; j < size_; j ++) {
      if (bucket_full_[j]) {
        TKey curKey = table_[j];
        bool value_inserted = false;
        // Now insert this key/value pair
        for (uint32_t i = 0; i < new_size; i ++) {
          uint32_t hash = linearProbeFunc(hash_func_(new_size, curKey), i,
            new_size);
          if (!new_bucket_full[hash]) {
            new_bucket_full[hash] = true;
            new_table[hash] = curKey;
            value_inserted = true;
            break;
          }
        }  // end for (uint32_t i = 0; i < size_; i ++)
        if (!value_inserted) {
          printf("HashSet<TKey>::rehash - Couldn't insert a value!  ");
          printf("This shouldn't happen.  Check hash table logic.\n");
          throw std::runtime_error("HashSet<TKey>::rehash: insert failed");
        }
      }  // end if (bucket_full_[i])
    }
    size_ = new_size;
    delete[] table_;
    delete[] bucket_full_;
    table_ = new_table;
    bucket_full_ = new_bucket_full;
  };

  template <class TKey>
  HashSet<TKey>::~HashSet() {
    if (table_)
      delete[] table_;
    if (bucket_full_)
      delete[] bucket_full_;
  };

  template <class TKey>
  void HashSet<TKey>::clear() {
    if (table_) {
      // Nothing to do for non-pointer class
    }
    if (bucket_full_) {
      memset(bucket_full_, false, size_*sizeof(bucket_full_[0]));
    }
    load_factor_ = 0;
    count_ = 0;
  };

  template <class TKey>
  bool HashSet<TKey>::insert(const TKey& key) {
    uint32_t hash = hash_func_(size_, key);
    return insertPrehash(hash, key);
  };

  template <class TKey>
  bool HashSet<TKey>::insertPrehash(const uint32_t prehash, const TKey& key) {
    // key trying to insert until the table is full.  We're doing linear probes
    // http://en.wikipedia.org/wiki/Linear_probing
    for (uint32_t i = 0; i < size_; i ++) {
      uint32_t hash = linearProbeFunc(prehash, i, size_);
      if (!bucket_full_[hash]) {
        bucket_full_[hash] = true;
        table_[hash] = key;
        count_++;
        load_factor_ = static_cast<float>(count_) / static_cast<float>(size_);
        while (load_factor_ > max_load_) {
          rehash();
          load_factor_ = static_cast<float>(count_) / 
            static_cast<float>(size_);
        }
        return true;
      } else {
        if (table_[hash] == key)  // Key already exists
          return false;
      }
    }
    return false;
  };

  //template <class TKey>
  //bool HashSet<TKey>::remove(const TKey& key) {
  //  uint32_t hash = hash_func_(size_, key);
  //  return removePrehash(hash, key);
  //};

  //template <class TKey>
  //bool HashSet<TKey>::removePrehash(const uint32_t prehash, const TKey& key) {
  //  for (uint32_t i = 0; i < size_; i ++) {
  //    uint32_t hash = linearProbeFunc(prehash, i, size_);
  //    if (!bucket_full_[hash]) {
  //      return false;
  //    } else {
  //      if (table_[hash] == key) {
  //        bucket_full_[hash] = false;
  //        count_--;
  //        load_factor_ = static_cast<float>(count_) / 
  //          static_cast<float>(size_);
  //        return true;
  //      }
  //    }
  //  }
  //  return false;
  //};

  template <class TKey>
  bool HashSet<TKey>::lookup(const TKey& key) const {
    uint32_t hash = hash_func_(size_, key);
    return lookupPrehash(hash, key);
  };

  template <class TKey>
  bool HashSet<TKey>::lookupPrehash(const uint32_t prehash, const TKey& key) 
    const {
    for (uint32_t i = 0; i < size_; i ++) {
      uint32_t hash = linearProbeFunc(prehash, i, size_);
      if (!bucket_full_[hash]) {
        return false;
      } else {
        if (table_[hash] == key) {
          return true;
        }
      }
    }
    return false;
  };

};  // namespace data_str
};  // namespace jtil
