//
//  vector_managed.h
//
//  Created by Jonathan Tompson on 4/26/12.
//
//  A very simple templated vector class
//  
//  NOTE: When the <type> is a pointer (ie <int*>), then ownership
//        IS TRANSFERRED.  The Vector will call the destructor for that
//        memory when clear is called or the Vector destructor is called.
//        Also, set(i, ...) will free anything that is in index i before
//        writting to that index.
//
//        BOUNDS CHECKS ONLY PERFORMED ON _DEBUG builds.

#pragma once

#include <stdio.h>  // For printf()
#include "math/math_types.h"  // for uint

namespace jtil {
namespace data_str {

  template <typename T>
  class VectorManaged {
  public:
    explicit VectorManaged(const uint32_t capacity = 0);
    ~VectorManaged();

    void capacity(const uint32_t capacity);  // Request manual capacity incr
    void clear();
    inline void pushBack(const T& elem);  // Add a copy of the element to back
    inline void popBack();  // remove last element and call it's destructor
    inline void popBackUnsafe();  // No bounds checking
    inline T* at(uint32_t index);  // Get an internal reference
    void deleteAtAndShift(const uint32_t index);  // remove elem and shift down
    inline void set(const uint32_t index, const T& elem);
    void resize(const uint32_t size_);
    inline const uint32_t& size() const { return size_; }
    inline const uint32_t& capacity() const { return capacity_; }
    bool operator==(const VectorManaged<T>& a) const;  // O(n) - linear search
    VectorManaged<T>& operator=(const VectorManaged<T>& other);  // O(n) - copy
    T operator[](const uint32_t index) const;
    T & operator[](const uint32_t index);

  private:
    uint32_t size_;
    uint32_t capacity_;  // will only grow or shrink by a factor of 2
    T* pvec_;
  };

  template <typename T>
  VectorManaged<T>::VectorManaged(const uint32_t capacity) {  // capacity = 0
    pvec_ = NULL;
    capacity_ = 0;
    size_ = 0;
    if (capacity != 0) {
      this->capacity(capacity);
    }
  };

  template <typename T>
  VectorManaged<T>::~VectorManaged() {
    if (pvec_) { 
      for (uint32_t i = 0; i < capacity_; i ++) {
        (&pvec_[i])->~T();  // Call the destructor on each element of the array
      }
#ifdef _WIN32
      _aligned_free(pvec_); 
#else
      free(pvec_); 
#endif
      pvec_ = NULL; 
    }
    capacity_ = 0;
    size_ = 0;
  };

  template <typename T>
  void VectorManaged<T>::capacity(const uint32_t capacity) {
    if (capacity != capacity_ && capacity != 0) {
      T* pvec_old = pvec_;

#ifdef _WIN32
      T dummy;
      static_cast<void>(dummy);  // Get rid of unreference local variable warn
      void* temp = NULL;
      temp = _aligned_malloc(capacity * sizeof(dummy), ALIGNMENT);
#else
      T dummy;
      void* temp = NULL;
      temp = malloc(capacity * sizeof(dummy));
#endif

      if (temp == NULL) { 
        throw std::runtime_error("VectorManaged<T>::capacity: Malloc Failed.");
      }

      // Use placement new to call the constructors for the array
      pvec_ = reinterpret_cast<T*>(temp);
      for (uint32_t i = 0; i < capacity; i ++) {
        pvec_[i] = *(new(pvec_ + i) T());  // Call placement new on each item
      }

      if (pvec_old) {
        if (capacity <= capacity_) {
          for (uint32_t i = 0; i < capacity; i ++) {
            pvec_[i] = pvec_old[i];
          }
        } else {
          for (uint32_t i = 0; i < capacity_; i ++) {
            pvec_[i] = pvec_old[i];
          }
        }

#ifdef _WIN32
        _aligned_free(pvec_old); 
#else
        free(pvec_old); 
#endif
        pvec_old = NULL;
      }

      capacity_ = capacity;
      if (size_ > capacity_) {  // If we've truncated the array then resize_
        size_ = capacity_;
      }
    } else if (capacity == 0) {  // Clear array if desired capacity is zero
      clear();
    }
  };

  template <typename T>
  void VectorManaged<T>::clear() {
    size_ = 0;
    if (pvec_) { 
      for (uint32_t i = 0; i < capacity_; i ++) {
        (&pvec_[i])->~T();  // explicitly call the destructor on each element
      }
#ifdef _WIN32
      _aligned_free(pvec_); 
#else
      free(pvec_);
#endif      
      pvec_ = NULL; 
    }
    capacity_ = 0;
  };

  template <typename T>
  void VectorManaged<T>::pushBack(const T& elem) {
    if (capacity_ == 0)
      capacity(1);
    else if (size_ == capacity_)
      capacity(capacity_ * 2);  // Grow the array by size_ 2
    pvec_[size_] = elem;
    size_ += 1;
  };
  
  template <typename T>
  void VectorManaged<T>::popBack() {
    if (size_ > 0)
      size_ -= 1;  // just reduce the size_ by 1
    else
      throw std::runtime_error("VectorManaged<T>::popBack: Out of bounds");
  };

  template <typename T>
  void VectorManaged<T>::popBackUnsafe() {
    size_ -= 1;  // just reduce the size_ by 1
  };

  template <typename T>
  T* VectorManaged<T>::at(const uint32_t index ) {
#if defined(_DEBUG) || defined(DEBUG)
    if (index > (size_-1))
      throw std::runtime_error("VectorManaged<T>::at: Out of bounds");
#endif
    return &pvec_[index];
  };

  template <typename T>
  T VectorManaged<T>::operator[](const uint32_t index) const { 
#if defined(_DEBUG) || defined(DEBUG)
    if (index > (size_-1))
      throw std::runtime_error("Vector<T>::at: Out of bounds");
#endif
    return pvec_[index]; 
  };

  template <typename T>
  T& VectorManaged<T>::operator[](const uint32_t index) { 
#if defined(_DEBUG) || defined(DEBUG)
    if (index > (size_-1))
      throw std::runtime_error("Vector<T>::at: Out of bounds");
#endif
    return pvec_[index]; 
  };

  template <typename T>
  void VectorManaged<T>::set(const uint32_t index, const T& val ) {
#if defined(_DEBUG) || defined(DEBUG)
    if (index > (size_-1))
      throw std::runtime_error("VectorManaged<T>::at: Out of bounds");
#endif
    pvec_[index] = val;
  };

  template <typename T>
  void VectorManaged<T>::resize(const uint32_t size) {
#if defined(_DEBUG) || defined(DEBUG)
    if ( size > capacity_ || size < 0 ) { 
      throw std::runtime_error("VectorManaged<T>::resize: Out of bounds");
    } else { 
#endif
      size_ = size; 
#if defined(_DEBUG) || defined(DEBUG)
    } 
#endif
  };   

  template <typename T>
  bool VectorManaged<T>::operator==(const VectorManaged& a) const {
    if (this == &a) {  // if both point to the same memory
      return true; 
    }
    if (size_ != a.size_) { 
      return false; 
    }
    for (uint32_t i = 0; i <= (size_-1); i++) {
      if (pvec_[i] != a.pvec_[i]) { 
        return false; 
      }
    }
    return true;
  };

  template <typename T>
  VectorManaged<T>& VectorManaged<T>::operator=(
    const VectorManaged<T>& other) {
    if (this != &other) {  // protect against invalid self-assignment
      this->clear();
      this->capacity(other.capacity_);
      for (uint32_t i = 0; i < other.size_; i ++) {
        this->pvec_[i] = other.pvec_[i];
      }
      this->size_ = other.size_;
    }
    // by convention, always return *this
    return *this;
  };

  template <typename T>
  void VectorManaged<T>::deleteAtAndShift(const uint32_t index) {
#ifdef _DEBUG
    if (index > (size_-1)) {
      throw std::runtime_error("VectorManaged<T>::at: Out of bounds");
    }
#endif
    for (uint32_t i = index; i < size_-1; i++) {
      pvec_[i] = pvec_[i+1];
    }
    size_--;
  };

  // *************************************************************************
  // Template specialization for pointers, note if VectorManaged<int*> is used, 
  // T is an int in this case.  NOT a int*.  Hence, pvec needs to be a double 
  // ptr
  template <typename T>
  class VectorManaged<T*> {
  public:
    explicit VectorManaged<T*>(const uint32_t capacity = 0);
    ~VectorManaged<T*>();

    void capacity(const uint32_t capacity);  // Request manual capacity incr
    void clear();
    inline void pushBack(T * const elem);  // Add copy of the ptr to back
    inline void popBack();  // remove last element and call it's destructor
    inline void popBackUnsafe();  // No bounds checking
    inline T** at(const uint32_t index);  // Get an internal reference
    void deleteAtAndShift(const uint32_t index);  // remove elem and shift down
    inline void deleteAt(const uint32_t index);
    inline void set(const uint32_t index, T * const elem);
    void resize(const uint32_t size_);
    inline const uint32_t& size() const { return size_; }
    inline const uint32_t& capacity() const { return capacity_; }
    bool operator==(const VectorManaged<T*> &a) const;  // O(n) - linear search
    VectorManaged& operator=(const VectorManaged& other);  // O(n) - copy
    T* operator[](const uint32_t index) const;
    T* & operator[](const uint32_t index);

  private:
    uint32_t size_;
    uint32_t capacity_;  // will only grow or shrink by a factor of 2
    T** pvec_;
  };

  template <typename T>
  VectorManaged<T*>::VectorManaged(const uint32_t capacity) {  // capacity = 0
    pvec_ = NULL;
    capacity_ = 0;
    size_ = 0;
    if (capacity != 0) {
      this->capacity(capacity);
    }
  };

  template <typename T>
  VectorManaged<T*>::~VectorManaged() {
    if (pvec_) { 
      for (uint32_t i = 0; i < capacity_; i ++) {
        if (pvec_[i]) {
          delete pvec_[i];  // Call the destructor on each element of the array
          pvec_[i] = NULL;
        }
      }
#ifdef _WIN32     
      _aligned_free(pvec_); 
#else
      free(pvec_);      
#endif
      pvec_ = NULL; 
    }
    capacity_ = 0;
    size_ = 0;
  };

  template <typename T>
  void VectorManaged<T*>::capacity(const uint32_t capacity) {
    if (capacity != capacity_ && capacity != 0) {
      T** pvec_old = pvec_;

#ifdef _WIN32
      T* dummy;
      dummy = NULL;
      void ** temp = NULL;
      temp = reinterpret_cast<void**>(_aligned_malloc(capacity * 
                                                      sizeof(dummy), 
                                                      ALIGNMENT));
#else
      T** temp = new T*[capacity];
#endif

      if (temp == NULL) { 
        throw std::runtime_error("VectorManaged<T>::capacity: Malloc Failed.");
      }

      // Zero the array elements
      pvec_ = (T**)(temp);  // cannot use reinterpret case in case of const T
      for (uint32_t i = 0; i < capacity; i ++) {
        pvec_[i] = NULL;
      }

      if (pvec_old) {
        if (capacity <= capacity_) {
          for (uint32_t i = 0; i < capacity; i ++) {
            pvec_[i] = pvec_old[i];
          }
        } else {
          for (uint32_t i = 0; i < capacity_; i ++) {
            pvec_[i] = pvec_old[i];
          }
        }

#ifdef _WIN32
        _aligned_free(pvec_old); 
#else
        free(pvec_old);
#endif
        pvec_old = NULL;
      }

      capacity_ = capacity;
      if (size_ > capacity_) {  // If we've truncated the array then resize_
        size_ = capacity_;
      }
    } else if (capacity == 0) {  // Clear array if desired capacity is zero
      clear();
    }
  };

  // A partial specialization of for pointer types 
  template<typename T> 
  void VectorManaged<T*>::clear() {
    size_ = 0;
    if (pvec_) { 
      for (uint32_t i = 0; i < capacity_; i ++) {
        if (pvec_[i]) {
          delete pvec_[i];  // delete each element
          pvec_[i] = NULL;
        }
      }
#ifdef _WIN32
      _aligned_free(pvec_); 
#else
      free(pvec_);
#endif
      pvec_ = NULL; 
    }
    capacity_ = 0;
  };

  template <typename T>
  void VectorManaged<T*>::pushBack(T * const elem) {
    if (capacity_ == 0)
      capacity(1);
    else if (size_ == capacity_)
      capacity(capacity_ * 2);  // Grow the array by size_ 2
    pvec_[size_] = elem;
    size_ += 1;
  };
  
  template <typename T>
  void VectorManaged<T*>::popBack() {
    if (size_ > 0) {
      if (pvec_[size_-1]) {
        delete pvec_[size_-1];
        pvec_[size_-1] = NULL;
      }
      size_ -= 1;
    } else {
      throw std::runtime_error("VectorManaged<T>::popBack: Out of bounds");
    }
  };

  template <typename T>
  void VectorManaged<T*>::popBackUnsafe() {
    if (pvec_[size_-1]) {
      delete pvec_[size_-1];
      pvec_[size_-1] = NULL;
    }
    size_ -= 1;
  };

  template <typename T>
  T** VectorManaged<T*>::at(const uint32_t index ) {
#if defined(_DEBUG) || defined(DEBUG)
    if (index > (size_-1))
      throw std::runtime_error("VectorManaged<T>::at: Out of bounds");
#endif
    return &pvec_[index];
  };

  template <typename T>
  T* VectorManaged<T*>::operator[](const uint32_t index) const { 
#if defined(_DEBUG) || defined(DEBUG)
    if (index > (size_-1))
      throw std::runtime_error("Vector<T>::at: Out of bounds");
#endif
    return pvec_[index]; 
  };

  template <typename T>
  T*& VectorManaged<T*>::operator[](const uint32_t index) { 
#if defined(_DEBUG) || defined(DEBUG)
    if (index > (size_-1))
      throw std::runtime_error("Vector<T>::at: Out of bounds");
#endif
    return pvec_[index]; 
  };

  template <typename T>
  void VectorManaged<T*>::set(const uint32_t index, T * const val ) {
#if defined(_DEBUG) || defined(DEBUG)
    if (index > (size_-1))
      throw std::runtime_error("VectorManaged<T>::at: Out of bounds");
#endif
    pvec_[index] = val;
  };

  template <typename T>
  void VectorManaged<T*>::deleteAt(const uint32_t index) {
#if defined(_DEBUG) || defined(DEBUG)
    if (index > (size_-1)) {
      throw std::runtime_error("VectorManaged<T>::at: Out of bounds");
    }
#endif
    if (pvec_[index]) {
      delete pvec_[index];
      pvec_[index] = NULL;
    }
  };

  template <typename T>
  void VectorManaged<T*>::resize(const uint32_t size) { 
#if defined(_DEBUG) || defined(DEBUG)
    if ( size > capacity_) { 
      throw std::runtime_error("VectorManaged<T>::resize: Out of bounds");
    } else { 
#endif
      if ( size < size_ ) {
        for (uint32_t i = size_ - 1; i > size; i--) {
          if (pvec_[i]) {
            delete pvec_[i];
            pvec_[i] = NULL;
          }
        }
      }
      size_ = size; 
#if defined(_DEBUG) || defined(DEBUG)
    }
#endif
  };   

  template <typename T>
  bool VectorManaged<T*>::operator==(const VectorManaged<T*>& a) const {
    if (this == &a) {  // if both point to the same memory
      return true; 
    }
    if (size_ != a.size_) { 
      return false; 
    }
    for (uint32_t i = 0; i <= (size_-1); i++) {
      if (*this->pvec_[i] != *a.pvec_[i]) { 
        return false; 
      }
    }
    return true;
  };

  template <typename T>
  VectorManaged<T*>& VectorManaged<T*>::operator=(
    const VectorManaged<T*>& other) {
    if (this != &other) {  // protect against invalid self-assignment
      this->clear();
      this->capacity(other.capacity_);
      for (uint32_t i = 0; i < other.size_; i ++) {
        this->pvec_[i] = other.pvec_[i];
      }
      this->size_ = other.size_;
    }
    // by convention, always return *this
    return *this;
  };

  template <typename T>
  void VectorManaged<T*>::deleteAtAndShift(const uint32_t index) {
#ifdef _DEBUG
    if (index > (size_-1)) {
      throw std::runtime_error("VectorManaged<T>::at: Out of bounds");
    }
#endif
    if (pvec_[index]) {
      delete pvec_[index];
      pvec_[index] = NULL;
    }
    for (uint32_t i = index; i < size_-1; i++) {
      pvec_[i] = pvec_[i+1];
    }
    pvec_[size_-1] = NULL;
    size_--;
  };

};  // namespace data_str
};  // namespace jtil
