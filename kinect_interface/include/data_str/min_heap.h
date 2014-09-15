//
//  min_heap.h
//
//  Created by Jonathan Tompson on 6/20/12.
//
//  A very simple templated binary min heap class
//  
//  NOTE: Heap is built from a pre-existing array.  The data for the heap is
//        copied from the array.  The existing input array remains intact.
//        If the template class is a complex container (sorry for the Java
//        lingo), then many copy constructors will be called.
//

#pragma once

#include <stdio.h>  // For printf()
#ifdef __APPLE__
#include <stdexcept>
#endif
#include "math/math_types.h"  // for uint
#include "data_str/vector.h"

namespace jtil {
namespace data_str {

  template <typename T>
  class MinHeap {
  public:
    MinHeap(const T* data, const uint32_t data_size);  // O(n) build from data
    MinHeap(const uint32_t reserved_size);  // O(n) insertions will be O(nlogn)
    ~MinHeap();

    T removeMin();
    void insert(const T& val);
    inline uint32_t size() const;
    inline uint32_t capacity() const;
    
    bool validate() const;  // For testing purposes (do not delete)
    void print() const;

  private:
    data_str::Vector<T> pvec_;
    
    inline static uint32_t nextPow2(const uint32_t v);
    uint32_t reheapifyUp(const uint32_t i);  // Return new pos after heapify
    uint32_t reheapifyDown(const uint32_t i);
    inline static uint32_t getParent(const uint32_t i);
    inline static uint32_t getLChild(const uint32_t i);
    inline static uint32_t getRChild(const uint32_t i);
  };

  template <typename T>
  MinHeap<T>::MinHeap(const T* data, const uint32_t data_size) {
    if (data_size == 0) {
      throw std::runtime_error("Heap<T>::Heap() - Error, data size is 0!");
    }
    uint32_t capacity_ = nextPow2(data_size);
    pvec_.capacity(capacity_);
    // Copy over the data
    pvec_.resize(data_size);
    memcpy(pvec_.at(0), data, sizeof(*pvec_.at(0)) * data_size);
    
    // Now heapify from the bottom up
    // Note: the first non-leaf is capacity / 2
    for (int i = pvec_.capacity()/2; i != MAX_UINT32; i--) {  // Assume wrapping
      // Reheapify the subtree starting from i:
      uint32_t old_pos = MAX_UINT32;
      uint32_t cur_pos = i;
      while (old_pos != cur_pos) {
        old_pos = cur_pos;
        cur_pos = reheapifyDown(cur_pos);
      }
    }
  };
  
  template <typename T>
  MinHeap<T>::MinHeap(const uint32_t reserved_size) {
    if (reserved_size == 0) {
      throw std::runtime_error("Heap<T>::Heap() - Error, reserved size is 0!");
    }
    uint32_t capacity_ = nextPow2(reserved_size);
    pvec_.capacity(capacity_);
  };  

  template <typename T>
  MinHeap<T>::~MinHeap() {
    
  };
  
  template <typename T>
  void MinHeap<T>::insert(const T& val) {
    pvec_.pushBack(val);
    uint32_t old_pos = MAX_UINT32;
    uint32_t cur_pos = pvec_.size()-1;
    while (old_pos != cur_pos) {
      old_pos = cur_pos;
      cur_pos = reheapifyUp(cur_pos);
    }
  };
  
  template <typename T>
  bool MinHeap<T>::validate() const {
    for (uint32_t i = 1; i < pvec_.size(); i++) {
      uint32_t ind_parent = getParent(i);
      if (pvec_[ind_parent] > pvec_[i]) {
        return false;  // the parent is greater than a child!  not a min heap!
      }
    }
    return true;
  };  
  
  template <typename T>
  void MinHeap<T>::print() const {
    if (pvec_.size() == 0) {
      printf("MinHeap[] = []\n");
      return;
    }
    printf("MinHeap[0:%d] = [", pvec_.size()-1);
    for (uint32_t i = 0; i < pvec_.size(); i++) {
      printf("%d", pvec_[i]);
      if (i != pvec_.size()-1) {
        printf(", ");
      }
    }
    printf("]\n");
  };   
  
  template <typename T>
  uint32_t MinHeap<T>::reheapifyUp(const uint32_t i) {
    if (i == 0) {
      return i;
    }
    uint32_t ind_parent = getParent(i);
    if (pvec_[ind_parent] > pvec_[i]) {
      T parent = pvec_[ind_parent];
      pvec_.set(ind_parent, pvec_[i]);
      pvec_.set(i, parent);
      return ind_parent;
    } else {
      return i;
    }
  };
  
  template <typename T>
  uint32_t MinHeap<T>::size() const {
    return pvec_.size();
  };
  
  template <typename T>
  T MinHeap<T>::removeMin() {
    if (pvec_.size() == 0) {
      throw std::runtime_error("Heap<T>::removeMin() - Heap size is 0!");
    }
    T ret_val = pvec_[0];
    pvec_.set(0, pvec_[pvec_.size()-1]);  // Move the last element to the root
    pvec_.resize(pvec_.size()-1);  // resize without changing capacity
    if (pvec_.size() > 0) {
      uint32_t old_pos = MAX_UINT32;
      uint32_t cur_pos = 0;
      while (old_pos != cur_pos) {
        old_pos = cur_pos;
        cur_pos = reheapifyDown(cur_pos);
      }
    }
    return ret_val;
  };
  
  template <typename T>
  uint32_t MinHeap<T>::reheapifyDown(const uint32_t i) {
    uint32_t max_index = pvec_.size()-1;
    if (i >= max_index) {
      return i;
    }
    uint32_t ind_l_child = getLChild(i);
    uint32_t ind_r_child = getRChild(i);
    bool bigger_than_l_child = false;
    bool bigger_than_r_child = false;
    
    if ((ind_l_child <= max_index) && (pvec_[i] > pvec_[ind_l_child])) {
      bigger_than_l_child = true;
    }
    if ((ind_r_child <= max_index) && (pvec_[i] > pvec_[ind_r_child])) {
      bigger_than_r_child = true;
    }
    
    uint32_t swap_index = i;
    if (bigger_than_l_child && bigger_than_r_child) {
      // Swap with the smallest child
      if (pvec_[ind_l_child] > pvec_[ind_r_child]) {
        swap_index = ind_r_child;
      } else {
        swap_index = ind_l_child;
      }
    } else if (bigger_than_l_child) {
      swap_index = ind_l_child;
    } else if (bigger_than_r_child) {
      swap_index = ind_r_child;
    }
    
    if (swap_index != i) {
      T child = pvec_[swap_index];
      pvec_.set(swap_index, pvec_[i]);
      pvec_.set(i, child);
    }
    return swap_index;
  };  
  
  template <typename T>
  uint32_t MinHeap<T>::capacity() const {
      return pvec_.capacity();
  };
  
  // Define this locally (it actually exists in math base) to maintain the
  // independance of this header (the cost is a little code bloat).
  template <typename T>
  uint32_t MinHeap<T>::nextPow2(const uint32_t v) {
    // From: graphics.stanford.edu/~seander/bithacks.html#RoundUpPowerOf2
    uint32_t s = v;
    s--;
    s |= s >> 1;
    s |= s >> 2;
    s |= s >> 4;
    s |= s >> 8;
    s |= s >> 16;
    s++;
    return s;
  };
  
  template <typename T>
  uint32_t MinHeap<T>::getParent(const uint32_t i) {
    return static_cast<uint32_t>(floor((static_cast<float>(i)-1.0f)/2.0f));
  };
  
  template <typename T>
  uint32_t MinHeap<T>::getLChild(const uint32_t i) {
    return 2*i + 1;
  };
  
  template <typename T>
  uint32_t MinHeap<T>::getRChild(const uint32_t i) {
    return 2*i + 2;
  };

};  // namespace data_str
};  // namespace jtil
