//
//  hash_map.h
//
//  Created by Jonathan Tompson on 4/26/12.
//
//  This is an open addressing hash table.  Closed hashing with chaining can be
//  slow due to lots of malloc calls.  The probing is just simple linear 
//  probing.  The max load factor is 0.5, in order to maintain reasonable
//  lookup times.
//
//  This is a hash_map so it is a set of key and value pairs.
//  WORKS BEST IF SIZE IS A PRIME NUMBER, ESPECIALLY WITH A MOD HASH FUNCTION
//
//  NOTE: When insert is called, HashMap will make a copy of the input 
//        element's value field.  Ownership for pointers is NOT transferred.  
//        That is, the hash_map is not responsible for handling cleanup when 
//        Vector<type*> is used.
//

#pragma once

#include <cstring>  // For gcc builds, get rid of fpermissive compile error
#include <stdio.h>  // For printf()
#include "math/math_types.h"  // for uint
#include "math/math_base.h"  // for NextPrime
#include "data_str/pair.h"

#ifndef NULL
#define NULL 0
#endif

namespace jtil {
namespace data_str {

  template <class TKey, class TValue>
  class HashMap {
  public:
    // Function pointer for the hash function
    typedef uint32_t (*HashFunc) (const uint32_t size, const TKey& key);

    HashMap(const uint32_t size, HashFunc hash_func);
    ~HashMap();

    inline uint32_t size() const { return size_; }
    inline uint32_t count() const { return count_; }
    inline float load() const { return load_factor_; }
    bool insert(const TKey& key, const TValue& value);
    bool insertPrehash(const uint32_t prehash, const TKey& key, 
      const TValue& value);
    // REMOVAL MIGHT BE BROKEN AND NEEDS TO BE CHECKED!!!
    // DO I NEED TO MARK FORMALLY OCCUPIED?  PUT TOGETHER A TEST CASE FOR THIS.
    //bool remove(const TKey& key);
    //bool removePrehash(const uint32_t prehash, const TKey& key);
    bool lookup(const TKey& key, TValue& value) const;
    bool lookupPrehash(const uint32_t prehash, const TKey& key, TValue& value)
      const;
    bool set(const TKey& key, const TValue& value);
    bool setPrehash(const uint32_t prehash, const TKey& key, 
      const TValue& value);
    void clear();  // O(m) - m is the number of buckets (potentially slow)

    inline const Pair<TKey, TValue>* table() { return table_; }
    inline const bool* bucket_full() { return bucket_full_; }

  protected:
    uint32_t size_;
    uint32_t count_;
    Pair<TKey, TValue>* table_;
    bool* bucket_full_;  // Array of booleans telling us if an item exists there
    float load_factor_;
    static const float max_load_;

    HashFunc hash_func_;
    static uint32_t linearProbeFunc(const uint32_t hash, 
      const uint32_t probe_index, const uint32_t size);
    void rehash();

    // Non-copyable, non-assignable.
    HashMap(HashMap&);
    HashMap& operator=(const HashMap&);
  };

  template <class TKey, class TValue>
  const float HashMap<TKey, TValue>::max_load_ = 0.5f;

  template <class TKey, class TValue>
  uint32_t HashMap<TKey, TValue>::linearProbeFunc(const uint32_t hash, 
    const uint32_t probe_index, const uint32_t size) {
    return (hash + probe_index) % size;
  };

  template <class TKey, class TValue>
  HashMap<TKey, TValue>::HashMap(const uint32_t size, HashFunc hash_func) {
    hash_func_ = hash_func;
    load_factor_ = 0;
    count_ = 0;
    size_ = size;
    if (size_ < 1) {
      throw std::runtime_error("HashMap<TKey, TValue>::HashMap: size < 1");
    }
    table_ = new Pair<TKey, TValue>[size_];
    bucket_full_ = new bool[size_];
    memset(bucket_full_, false, size_*sizeof(bucket_full_[0]));
  };

  template <class TKey, class TValue>
  void HashMap<TKey, TValue>::rehash() {
    // printf("HashMap rehash\n");
    uint32_t new_size = size_*2;
    new_size = static_cast<uint32_t>(math::NextPrime(new_size));
    Pair<TKey, TValue>* new_table = new Pair<TKey, TValue>[new_size];
    bool* new_bucket_full = new bool[new_size];
    memset(new_bucket_full, false, new_size*sizeof(new_bucket_full[0]));
    // manually insert all the old key/value pairs into the hash table
    for (uint32_t j = 0; j < size_; j ++) {
      if (bucket_full_[j]) {
        TKey curKey = table_[j].first;
        TValue curValue = table_[j].second;
        bool value_inserted = false;
        // Now insert this key/value pair
        for (uint32_t i = 0; i < new_size; i ++) {
          uint32_t hash = linearProbeFunc(hash_func_(new_size, curKey), i, 
            new_size);
          if (!new_bucket_full[hash]) {
            new_bucket_full[hash] = true;
            new_table[hash].first = curKey;
            new_table[hash].second = curValue;
            value_inserted = true;
            break;
          }
        }  // end for (uint32_t i = 0; i < size_; i ++)
        if (!value_inserted) {
          printf("HashMap<TKey, TValue>::rehash - Couldn't insert a value!  ");
          printf("This shouldn't happen.  Check hash table logic.\n");
          throw std::runtime_error("HashMap::rehash: insert failed");
        }
      }  // end if (bucket_full_[i])
    }
    size_ = new_size;
    delete[] table_;
    delete[] bucket_full_;
    table_ = new_table;
    bucket_full_ = new_bucket_full;
  };

  template <class TKey, class TValue>
  HashMap<TKey, TValue>::~HashMap() {
    if (table_)
      delete[] table_;
    if (bucket_full_)
      delete[] bucket_full_;
  };

  template <class TKey, class TValue>
  void HashMap<TKey, TValue>::clear() {
    if (table_) {
      // Nothing to do for non-pointer class
    }
    if (bucket_full_) {
      memset(bucket_full_, false, size_*sizeof(bucket_full_[0]));
    }
    load_factor_ = 0;
    count_ = 0;
  };

  template <class TKey, class TValue>
  bool HashMap<TKey, TValue>::insert(const TKey& key, const TValue& value) {
    uint32_t hash = hash_func_(size_, key);
    return insertPrehash(hash, key, value);
  };

  template <class TKey, class TValue>
  bool HashMap<TKey, TValue>::insertPrehash(const uint32_t prehash, 
    const TKey& key, const TValue& value) {
    // keep trying to insert until the table is full.  We're doing linear probe
    // http://en.wikipedia.org/wiki/Linear_probing
    for (uint32_t i = 0; i < size_; i ++) {
      uint32_t hash = linearProbeFunc(prehash, i, size_);
      if (!bucket_full_[hash]) {
        bucket_full_[hash] = true;
        table_[hash].first = key;
        table_[hash].second = value;
        count_++;
        load_factor_ = static_cast<float>(count_) / static_cast<float>(size_);
        while (load_factor_ > max_load_) {
          rehash();
          load_factor_ = static_cast<float>(count_) / 
            static_cast<float>(size_);
        }
        return true;
      } else {
        if (table_[hash].first == key)  // Key already exists
          return false;
      }
    }
    return false;
  };

  template <class TKey, class TValue>
  bool HashMap<TKey, TValue>::set(const TKey& key, const TValue& value) {
    uint32_t hash = hash_func_(size_, key);
    return setPrehash(hash, key, value);
  };

  template <class TKey, class TValue>
  bool HashMap<TKey, TValue>::setPrehash(const uint32_t prehash, 
    const TKey& key, const TValue& value) {
    // Set only if the value is found.
    for (uint32_t i = 0; i < size_; i ++) {
      uint32_t hash = linearProbeFunc(prehash, i, size_);
      if (!bucket_full_[hash]) {
        return false;
      } else {
        if (table_[hash].first == key) {  // Key already exists, set it
          table_[hash].first = value;
          return true;
        }
      }
    }
    return false;
  };

  //template <class TKey, class TValue>
  //bool HashMap<TKey, TValue>::remove(const TKey& key) {
  //  uint32_t hash = hash_func_(size_, key);
  //  return removePrehash(hash, key);
  //};

  //template <class TKey, class TValue>
  //bool HashMap<TKey, TValue>::removePrehash(const uint32_t prehash, 
  //  const TKey& key) {
  //  for (uint32_t i = 0; i < size_; i ++) {
  //    uint32_t hash = linearProbeFunc(prehash, i, size_);
  //    if (!bucket_full_[hash]) {
  //      return false;
  //    } else {
  //      if (table_[hash].first == key) {
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

  template <class TKey, class TValue>
  bool HashMap<TKey, TValue>::lookup(const TKey& key, TValue& value) const {
    uint32_t hash = hash_func_(size_, key);
    return lookupPrehash(hash, key, value);
  };

  template <class TKey, class TValue>
  bool HashMap<TKey, TValue>::lookupPrehash(const uint32_t prehash, 
    const TKey& key, TValue& value) const {
    for (uint32_t i = 0; i < size_; i ++) {
      uint32_t hash = linearProbeFunc(prehash, i, size_);
      if (!bucket_full_[hash]) {
        return false;
      } else {
        if (table_[hash].first == key) {
          value = table_[hash].second;
          return true;
        }
      }
    }
    return false;
  };
  
};  // namespace data_str
};  // namespace jtil
