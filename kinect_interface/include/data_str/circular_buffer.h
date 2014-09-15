//
//  circular_buffer.h
//
//  Created by Jonathan Tompson on 4/26/12.
//
// This is a fixed-size, circular buffer.
//
// The class is not thread-safe,
//
// To distinguish between empty and full states, the array never gets completely
// full.  If: p_write == p_read, then the array is empty.
//            p_write == p_read-1, then the array is full.
// Alternatively:
// 1. We could keep a num_items variable.
// 2. We could keep a read items and written items count.
// 3. Use absolute indices for p_write and p_read with modulo on each array
//    look-up.
// 4. We could keep a Boolean of the last operation.
//
// Wiki has good info: http://en.wikipedia.org/wiki/Circular_buffer
//

#pragma once

#include <stdio.h>  // For printf()
#ifdef __APPLE__
#include <inttypes.h>
#endif
#include "math/math_types.h"  // for uint

#define MCP_CIRCULAR_BUFFER_EMPTY -1

namespace jtil {
namespace data_str {

  template <class T>
  class CircularBuffer {
  public:
    explicit CircularBuffer(const uint32_t slots);
    ~CircularBuffer();

    // Writes 'value' to the next available slot. It may overwrite values that
    // were not yet read out of the buffer.
    void write(const T& value);

    // Returns the next value available for reading, in the order they were
    // written, and marks slot as read.  Returns false if the buffer was empty.
    bool read(T& ret_val);

    void clear();  // Removes all the elements from the buffer.
    bool empty() const;
    bool full() const;

  private:
    uint32_t p_write_;
    uint32_t p_read_;
    uint32_t size_;
    T* arr_;

    // Non-copyable, non-assignable.
    CircularBuffer(CircularBuffer&);
    CircularBuffer& operator=(const CircularBuffer&);
  };

  template <class T>
  CircularBuffer<T>::CircularBuffer(const uint32_t slots) {
    if (slots < 1) {
      throw std::runtime_error("CircularBuffer<T>::CircularBuffer: slots < 1");
    }
    size_ = slots + 1;
    p_write_ = 0;
    p_read_ = 0;
    arr_ = new T[size_];
  };

  template <class T>
  CircularBuffer<T>::~CircularBuffer() {
    delete [] arr_;
  };

  template <class T>
  void CircularBuffer<T>::write(const T& value) {
    // Write the next value
    arr_[p_write_] = value;
    p_write_ = (p_write_ + 1) % size_;

    // Check if we filled up the buffer, if so then increment the read pointer
    if (p_write_ == p_read_)
      p_read_ = (p_read_ + 1) % size_;
  };

  template <class T>
  bool CircularBuffer<T>::read(T& ret_val) {
    if (p_write_ == p_read_)
      return false;  // Nothing to read

    ret_val =  arr_[p_read_];
    p_read_ = (p_read_ + 1) % size_;
    return true;
  };

  template <class T>
  bool CircularBuffer<T>::full() const {
    return (p_write_ + 1) % size_ == p_read_;
  };

  template <class T>
  bool CircularBuffer<T>::empty() const {
    return p_write_ == p_read_;
  };

  template <class T>
  void CircularBuffer<T>::clear() {
    p_write_ = p_read_ = 0;
  };

};  // namespace data_str
};  // namespace jtil
