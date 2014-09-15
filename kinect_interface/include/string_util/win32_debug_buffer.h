//
//  win32_debug_buffer.h
//
//  Created by Jonathan Tompson on 5/6/12.
//
//  A small helper class to redirect std out to the Visual studio debug window
//  blog.tomaka17.com/2011/07/redirecting-cerr-and-clog-to-outputdebugstring/

#pragma once

#include <windows.h>
#include <streambuf>
#include <string>
#include <iostream>

class BufferedStringBuf : public std::streambuf {
 public:
  explicit BufferedStringBuf(const int bufferSize) {
    if (bufferSize) {
      char *ptr = new char[bufferSize];
      setp(ptr, ptr + bufferSize);
    } else {
      setp(0, 0);
    }
  }

  virtual ~BufferedStringBuf() {
    sync();
    delete[] pbase();
  }

  virtual void writeString(const std::string &str) = 0;

 private:
  int overflow(int c) {
    sync();
    if (c != EOF) {
      if (pbase() == epptr()) {
        std::string temp;
        temp += static_cast<char>(c);
        writeString(temp);
      } else {
        sputc(static_cast<char>(c));
      }
    }
    return 0;
  }


  int sync() {
    if (pbase() != pptr()) {
      int len = static_cast<int>(pptr() - pbase());
      std::string temp(pbase(), len);
      writeString(temp);
      setp(pbase(), epptr());
    }
    return 0;
  }
};

const int LineSize = 256;

class DebugBuf : public BufferedStringBuf {
 public:
  DebugBuf() : BufferedStringBuf(LineSize) {}

  virtual void writeString(const std::string &str) {
    OutputDebugStringA(str.c_str());
  }
};
